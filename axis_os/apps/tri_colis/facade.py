"""
Façade Robot de l'app de tri, branchée sur les services d'AXIS-OS.

La logique (logique.py) parle à cette façade ; la façade parle aux services :

    chercher   -> PerceptionService.find(..., FRAME_WORLD)
    aller      -> NavigationService.aller_vers(poste)
    accoster   -> NavigationService jusqu'à une pré-pose, puis ACCOSTAGE FIN
                  par consignes de vitesse au LocomotionService
    prendre    -> ManipulationService.prendre / poser / reinitialiser
    sécurité   -> LocomotionService.dernier_verdict (GardienProximite)

Rien ici ne contourne la sécurité : toute consigne de vitesse passe par le
LocomotionService, donc par le gardien de proximité puis par le garde-fou
des articulations. La façade n'écrit aucune articulation.

POURQUOI UN ACCOSTAGE FIN : le planificateur gonfle les obstacles du rayon
du robot plus une marge ; il ne peut donc pas amener le robot à 29 cm d'un
carton posé au bord d'une table. Comme un chariot qui s'amarre à un poste,
la navigation amène le robot à une PRÉ-POSE dégagée, face à la table, puis un
contrôleur d'accostage parcourt les derniers centimètres vers une cible
calculée à partir du carton — et jamais au-delà.

CONFIGURATION DE SITE REQUISE : la table de prise doit figurer dans les
obstacles du gardien avec sa hauteur de chargement (Rectangle.zmax = plateau
+ hauteur d'un carton). Les cartons posés dessus sont alors « expliqués » ;
sinon les voisins du carton visé arrêteraient le robot à chaque prise. Un
humain penché au-dessus, plus haut que zmax, reste vu.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

from axis_os.apps.tri_colis.logique import Colis
from axis_os.hal.core.frames import FRAME_WORLD
from axis_os.services.manipulation.service import Etat, Objet
from axis_os.services.navigation.service import EtatNavigation, Pose2D


def _angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass(frozen=True)
class ConfigFacade:
    requete: str = "a {couleur} box"
    confiance_min: float = 0.15
    z_min: float = 0.50            # un colis SUR la table — pas le sol
    z_max: float = 1.30
    fusion_m: float = 0.10         # deux détections plus proches = un colis
    largeur: float = 0.24          # dimensions des colis (entre les mains)
    profondeur: float = 0.20
    hauteur: float = 0.20
    # La profondeur mesurée par la caméra est celle de la FACE AVANT
    # (estimateur du quartile le plus proche) : on recule d'une
    # demi-profondeur pour obtenir le centre.
    position_face_avant: bool = True
    distance_prise: float = 0.29   # centre des mains devant la base
    recul_pre_pose: float = 0.45   # pré-pose, en arrière de la cible
    tol_position: float = 0.015
    tol_cap: float = 0.04
    gain: float = 1.2
    v_max: float = 0.12
    w_max: float = 0.30
    v_min: float = 0.06            # jamais dans la bande morte de la marche
    delai_accostage_s: float = 25.0
    recul_sortie: float = 0.35     # recul en ligne droite avant de repartir
    # Un accostage qui DIVERGE est abandonné : distance à la cible plus
    # grande que la distance de départ + cette marge. (Première version : un
    # seuil fixe de 0.60 m, dépassé d'emblée par un poste à 0.53 m + la
    # tolérance d'arrivée de la navigation — l'accostage échouait avant
    # d'avoir bougé.)
    marge_divergence: float = 0.10
    distance_max_accostage: float = 1.00  # au-delà, la cible n'est pas crédible


@dataclass(frozen=True)
class PosteDepose:
    """Un poste de dépose équipé d'un plateau.

    avance_m : ce qu'il reste à parcourir, en accostage fin, depuis la pose
               du poste dans la carte (dégagée, hors de la marge du
               planificateur) jusqu'à la pose de dépose, où le carton tenu
               se trouve au-dessus du plateau.
    hauteur  : hauteur du plateau (m, monde).
    """
    avance_m: float
    hauteur: float


class FacadeServices:
    def __init__(self, navigation, locomotion, manipulation, perception,
                 source_pose: Callable[[], Pose2D], horloge: Callable[[], float],
                 cap_prise: float, journal: Callable[[str], None] = print,
                 cfg: ConfigFacade = ConfigFacade(),
                 postes_depose: Optional[Dict[str, PosteDepose]] = None):
        """
        cap_prise     : orientation du robot face à la table de prise (celle
                        du poste de prise dans la carte).
        postes_depose : postes équipés d'un plateau. Aux autres, le carton
                        est relâché à hauteur de port.
        """
        self.nav, self.loco, self.manip = navigation, locomotion, manipulation
        self.percep = perception
        self._pose, self._horloge = source_pose, horloge
        self.cap_prise = cap_prise
        self._journal = journal
        self.cfg = cfg
        self.postes_depose = dict(postes_depose or {})
        # Phases : NAV, NAV_ACCOSTAGE (-> ACCOSTAGE), RECUL (-> NAV...), ECHEC
        self._phase: Optional[str] = None
        self._cible: Optional[Pose2D] = None     # cible de l'accostage fin
        self._etat_accostage = "EN_ROUTE"
        self._t_accostage = 0.0
        self._accoste = False                    # collé à une table
        self._dest = None                        # destination de navigation
        self._apres_nav: Optional[Pose2D] = None # accostage à faire ensuite
        self._poste_depose: Optional[str] = None
        self._d0 = 0.0
        self.stats = {"accostages": 0, "accostages_ratees": 0,
                      "detections_filtrees": 0, "reculs": 0}

    # ---- temps et sécurité ----------------------------------------------
    def maintenant(self) -> float:
        return self._horloge()

    def bloque_par_securite(self) -> bool:
        v = getattr(self.loco, "dernier_verdict", None)
        return v is not None and v.facteur <= 0.0

    def journal(self, message: str) -> None:
        self._journal(message)

    # ---- perception -----------------------------------------------------
    def _centre(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        if not self.cfg.position_face_avant:
            return (x, y, z)
        d = self.cfg.profondeur / 2.0
        return (x + d * math.cos(self.cap_prise), y + d * math.sin(self.cap_prise), z)

    def chercher(self, couleur: str) -> List[Colis]:
        r = self.percep.find(self.cfg.requete.format(couleur=couleur),
                             now_ns=int(self._horloge() * 1e9), frame=FRAME_WORLD)
        if not r.ok or not r.value:
            return []
        colis: List[Colis] = []
        for d in r.value:
            p = d.position_3d
            if p is None or d.confidence < self.cfg.confiance_min \
                    or not (self.cfg.z_min <= p.z <= self.cfg.z_max):
                self.stats["detections_filtrees"] += 1
                continue
            c = self._centre(p.x, p.y, p.z)
            if any(math.dist(c, k.position) < self.cfg.fusion_m for k in colis):
                continue                 # triées par confiance : on garde la 1re
            colis.append(Colis(couleur, c, d.confidence))
        return colis

    # ---- navigation et accostage -----------------------------------------
    #
    # Toute sortie d'un accostage commence par un RECUL en ligne droite :
    # pivoter sur place collé à une table, c'est y cogner un genou ou le
    # carton. Puis navigation, puis éventuellement un nouvel accostage fin.

    def _partir(self, dest, accostage: Optional[Pose2D],
                poste_depose: Optional[str] = None) -> Tuple[bool, str]:
        self._dest, self._apres_nav = dest, accostage
        self._poste_depose = poste_depose
        if self._accoste:
            self._accoste = False
            p = self._pose()
            r0 = self.cfg.recul_sortie
            self._cible = Pose2D(p.x - r0 * math.cos(p.cap),
                                 p.y - r0 * math.sin(p.cap), p.cap)
            self._phase, self._etat_accostage = "RECUL", "EN_ROUTE"
            self._t_accostage = self._horloge()
            self._d0 = self.cfg.recul_sortie
            self.stats["reculs"] += 1
            return True, ""
        return self._lancer_nav()

    def _lancer_nav(self) -> Tuple[bool, str]:
        self._phase = "NAV" if self._apres_nav is None else "NAV_ACCOSTAGE"
        r = self.nav.aller_vers(self._dest)
        if not r.ok:
            self._phase = "ECHEC"
        return r.ok, r.reason or ""

    def aller_au_poste(self, poste: str) -> Tuple[bool, str]:
        pd = self.postes_depose.get(poste)
        if pd is None:
            return self._partir(poste, None)
        but = self.nav.carte.postes.get(poste)
        if but is None:
            return False, f"poste inconnu : « {poste} »"
        c, s = math.cos(but.cap), math.sin(but.cap)
        cible = Pose2D(but.x + pd.avance_m * c, but.y + pd.avance_m * s, but.cap)
        return self._partir(poste, cible, poste_depose=poste)

    def accoster(self, colis: Colis) -> Tuple[bool, str]:
        c, s = math.cos(self.cap_prise), math.sin(self.cap_prise)
        cx, cy, _ = colis.position
        d = self.cfg.distance_prise
        cible = Pose2D(cx - d * c, cy - d * s, self.cap_prise)
        p = self._pose()
        if math.hypot(cible.x - p.x, cible.y - p.y) > 3.0:
            return False, "colis à plus de 3 m : ce n'est pas celui de la table"
        r0 = self.cfg.recul_pre_pose
        pre = Pose2D(cible.x - r0 * c, cible.y - r0 * s, self.cap_prise)
        return self._partir(pre, cible)

    def navigation(self) -> str:
        ph = self._phase
        if ph == "ECHEC":
            return "ECHEC"
        if ph == "ACCOSTAGE":
            return self._etat_accostage
        if ph == "RECUL":
            return "ECHEC" if self._etat_accostage == "ECHEC" else "EN_ROUTE"
        e = self.nav.etat
        if e is EtatNavigation.ECHEC:
            return "ECHEC"
        if e is EtatNavigation.ARRIVE and ph == "NAV":
            return "ARRIVE"
        return "EN_ROUTE"               # y compris NAV_ACCOSTAGE arrivé : tick() enchaîne

    def _franche(self, v: float, vmax: float) -> float:
        if abs(v) < 1e-9:
            return 0.0
        return math.copysign(min(vmax, max(self.cfg.v_min, abs(v))), v)

    def tick(self) -> None:
        """À appeler à chaque pas de commande, APRÈS navigation.tick()."""
        if self._phase == "NAV_ACCOSTAGE" and self.nav.etat is EtatNavigation.ARRIVE:
            self._cible = self._apres_nav
            self._phase, self._etat_accostage = "ACCOSTAGE", "EN_ROUTE"
            self._t_accostage = self._horloge()
            p = self._pose()
            self._d0 = math.hypot(self._cible.x - p.x, self._cible.y - p.y)
            self.stats["accostages"] += 1
        if self._phase not in ("ACCOSTAGE", "RECUL") \
                or self._etat_accostage != "EN_ROUTE":
            return
        etat = self._pas_accostage()
        if etat == "ARRIVE" and self._phase == "RECUL":
            self._lancer_nav()
        elif etat == "ARRIVE":
            self._accoste = True
            self._etat_accostage = "ARRIVE"
        elif etat == "ECHEC":
            self._etat_accostage = "ECHEC"
            self.stats["accostages_ratees"] += 1

    def _pas_accostage(self) -> str:
        """Contrôleur d'accostage : vers self._cible, en ligne quasi droite,
        par consignes de vitesse au LocomotionService (donc sous le gardien).
        Ne va jamais au-delà de la cible."""
        cfg, p, b = self.cfg, self._pose(), self._cible
        dxw, dyw = b.x - p.x, b.y - p.y
        c, s = math.cos(p.cap), math.sin(p.cap)
        ex, ey = c * dxw + s * dyw, -s * dxw + c * dyw     # repère du robot
        ecap = _angle(b.cap - p.cap)
        d = math.hypot(ex, ey)
        if d > self._d0 + cfg.marge_divergence or d > cfg.distance_max_accostage \
                or self._horloge() - self._t_accostage > cfg.delai_accostage_s:
            self.loco.halt()
            return "ECHEC"
        vx = self._franche(cfg.gain * ex, cfg.v_max) if abs(ex) > cfg.tol_position else 0.0
        vy = self._franche(cfg.gain * ey, cfg.v_max) if abs(ey) > cfg.tol_position else 0.0
        wz = self._franche(cfg.gain * ecap, cfg.w_max) if abs(ecap) > cfg.tol_cap else 0.0
        if vx == vy == wz == 0.0:
            self.loco.halt()
            return "ARRIVE"
        self.loco.set_velocity(vx, vy, wz)
        return "EN_ROUTE"

    # ---- manipulation -----------------------------------------------------
    def prendre(self, colis: Colis) -> Tuple[bool, str]:
        r = self.manip.prendre(Objet(colis.position, self.cfg.largeur,
                                     self.cfg.profondeur, self.cfg.hauteur))
        return r.ok, r.reason or ""

    def poser(self) -> Tuple[bool, str]:
        pd = self.postes_depose.get(self._poste_depose) if self._accoste else None
        r = self.manip.poser(hauteur_surface=pd.hauteur if pd else None)
        return r.ok, r.reason or ""

    def manipulation(self) -> str:
        e = self.manip.etat
        if e in (Etat.LIBRE, Etat.TENU, Etat.ECHEC):
            return e.name
        return "EN_COURS"

    def reinitialiser_manipulation(self) -> None:
        self.manip.reinitialiser()
