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
    # Approche finale « comme un humain » (constaté sur GPU : le robot
    # avançait bras en posture de port et HEURTAIT le carton) : arrêt à
    # `recul_insertion` de la cible, mains encore DEVANT le carton ; ouverture
    # large des bras, à l'arrêt ; puis avance lente pour les glisser autour.
    recul_insertion: float = 0.18
    v_insertion: float = 0.08
    # Blocage : s'il pousse sans se rapprocher de la cible de 1 cm en 2 s,
    # le robot est bloqué (constaté sur GPU : carton contre le bord de la
    # table de dépose, robot qui insistait). Il s'arrête, et l'app réessaie
    # après un recul.
    delai_blocage_s: float = 2.0
    # Stabilisation : arrivé, le robot s'IMMOBILISE avant de bouger les
    # bras (ouvrir, prendre, poser). Constaté sur GPU : il ouvrait ou
    # serrait les bras en piétinant encore — la politique de marche finit
    # ses pas après la consigne d'arrêt —, touchait et déplaçait le carton.
    # Immobile = moins de `derive_stable_m` de déplacement pendant
    # `duree_stabilisation_s`, attendu au plus `stabilisation_max_s`.
    # Visée : arrivé à la pré-pose, face au carton, le robot le REGARDE à
    # nouveau avant l'approche finale. Constaté sur RTX 5090 : position
    # perçue depuis le poste de prise fausse de 4 à 9 cm en latéral — l'erreur
    # croît avec l'angle de vue —, une main poussait le carton. Vu de face,
    # le carton est près de l'axe de la caméra : l'erreur est bien plus
    # faible. Correction bornée (au-delà, ce n'est pas le même carton).
    duree_visee_s: float = 0.6
    correction_visee_max_m: float = 0.15
    duree_stabilisation_s: float = 0.6
    # Après une prise ratée : temps laissé aux mains pour s'ouvrir en grand
    # avant de reculer (voir reinitialiser_manipulation).
    duree_liberation_s: float = 1.0
    derive_stable_m: float = 0.01
    stabilisation_max_s: float = 3.0
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
        self._insertion: Optional[Pose2D] = None   # cible finale, après pré-saisie
        self._apres_stab, self._phase_avant_stab = "ARRIVE", "ACCOSTAGE"
        self._liberation_jusqua = 0.0
        self._colis_vise: Optional[Colis] = None
        self.veut_percevoir = False        # la boucle doit rendre la caméra
        self._t_visee = 0.0
        self._t_stab0, self._ref_stab = 0.0, (0.0, 0.0, 0.0)
        self._progres = None                       # (distance, instant, cible)
        self._t_tick: Optional[float] = None
        self._vmax = cfg.v_max
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
        self._colis_vise, self.veut_percevoir = None, False
        if self.manip.etat is Etat.PRET:
            self.manip.reinitialiser()      # pré-saisie abandonnée : bras au port
        if self._accoste:
            self._accoste = False
            p = self._pose()
            r0 = self.cfg.recul_sortie
            self._cible = Pose2D(p.x - r0 * math.cos(p.cap),
                                 p.y - r0 * math.sin(p.cap), p.cap)
            self._phase, self._etat_accostage = "RECUL", "EN_ROUTE"
            self._vmax = self.cfg.v_max
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
        self._insertion = None
        return self._partir(poste, cible, poste_depose=poste)

    def accoster(self, colis: Colis) -> Tuple[bool, str]:
        c, s = math.cos(self.cap_prise), math.sin(self.cap_prise)
        cx, cy, _ = colis.position
        d = self.cfg.distance_prise
        cible = Pose2D(cx - d * c, cy - d * s, self.cap_prise)
        p = self._pose()
        if math.hypot(cible.x - p.x, cible.y - p.y) > 3.0:
            return False, "colis à plus de 3 m : ce n'est pas celui de la table"
        r0, ri = self.cfg.recul_pre_pose, self.cfg.recul_insertion
        pre = Pose2D(cible.x - r0 * c, cible.y - r0 * s, self.cap_prise)
        attente = Pose2D(cible.x - ri * c, cible.y - ri * s, self.cap_prise)
        ok, raison = self._partir(pre, attente)
        self._insertion = cible
        self._colis_vise = colis
        return ok, raison

    def navigation(self) -> str:
        ph = self._phase
        if ph == "ECHEC":
            return "ECHEC"
        if ph in ("ACCOSTAGE", "PRESAISIE", "INSERTION", "FRANCHISSEMENT",
                  "STABILISATION", "VISEE"):
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
        # Le temps passé arrêté par la SÉCURITÉ ne compte pas dans les délais
        # d'accostage : attendre qu'une personne s'éloigne n'est pas un échec.
        # (Constaté sur GPU : un collègue resté près du poste faisait échouer
        # l'accostage par dépassement de délai.)
        maintenant = self._horloge()
        dt = 0.0 if self._t_tick is None else maintenant - self._t_tick
        self._t_tick = maintenant
        if self.bloque_par_securite():
            self._t_accostage += dt
            self._t_stab0 += dt
        if self._phase == "NAV_ACCOSTAGE" and self.nav.etat is EtatNavigation.ARRIVE:
            self._cible = self._apres_nav
            self._vmax = self.cfg.v_max
            self._phase, self._etat_accostage = "ACCOSTAGE", "EN_ROUTE"
            pd = self.postes_depose.get(self._poste_depose)
            if pd is not None and self.manip.etat is Etat.TENU \
                    and not self.manip.degage_au_dessus(pd.hauteur):
                # Devant la table, AVANT d'avancer : lever le carton au-dessus.
                self.manip.franchir(pd.hauteur)
                self._phase = "FRANCHISSEMENT"
            self._t_accostage = self._horloge()
            p = self._pose()
            self._d0 = math.hypot(self._cible.x - p.x, self._cible.y - p.y)
            self.stats["accostages"] += 1
            if self._colis_vise is not None and self._phase == "ACCOSTAGE":
                self.loco.halt()
                self._phase, self._t_visee = "VISEE", self._horloge()
                self.veut_percevoir = True
        if self._phase == "VISEE" and self._etat_accostage == "EN_ROUTE":
            self.loco.halt()
            if self._horloge() - self._t_visee < self.cfg.duree_visee_s:
                return                   # immobile, et une image fraîche rendue
            self._viser()
            self.veut_percevoir = False
            self._phase, self._t_accostage = "ACCOSTAGE", self._horloge()
            p = self._pose()
            self._d0 = math.hypot(self._cible.x - p.x, self._cible.y - p.y)
            return
        if self._phase == "FRANCHISSEMENT" and self._etat_accostage == "EN_ROUTE":
            if self.manip.etat is Etat.TENU:
                pd = self.postes_depose.get(self._poste_depose)
                if pd is not None and self.manip.peut_franchir(pd.hauteur):
                    self._phase = "ACCOSTAGE"
                    self._t_accostage = self._horloge()
                    p = self._pose()
                    self._d0 = math.hypot(self._cible.x - p.x, self._cible.y - p.y)
                else:
                    self._journal(f"dépose : {self.manip.raison or 'objet trop bas'}")
                    self._etat_accostage = "ECHEC"
                    self.stats["accostages_ratees"] += 1
            return
        if self._phase == "STABILISATION" and self._etat_accostage == "EN_ROUTE":
            self.loco.halt()
            if not self._immobile():
                return
            self.stats["stabilisations"] = self.stats.get("stabilisations", 0) + 1
            if self._apres_stab == "PRESAISIE":
                # Immobile devant le carton : on ouvre les bras en grand.
                r = self.manip.presaisir()
                if not r.ok:
                    self._etat_accostage = "ECHEC"
                    self.stats["accostages_ratees"] += 1
                    return
                self._phase, self._t_accostage = "PRESAISIE", self._horloge()
                self.stats["presaisies"] = self.stats.get("presaisies", 0) + 1
            else:
                self._phase = self._phase_avant_stab
                self._accoste = True
                self._etat_accostage = "ARRIVE"
            return
        if self._phase == "PRESAISIE" and self._etat_accostage == "EN_ROUTE":
            if self.manip.etat is Etat.PRET:           # bras grands ouverts
                self._phase, self._cible = "INSERTION", self._insertion
                self._vmax = self.cfg.v_insertion
                self._t_accostage = self._horloge()
                p = self._pose()
                self._d0 = math.hypot(self._cible.x - p.x, self._cible.y - p.y)
            elif self._horloge() - self._t_accostage > self.cfg.delai_accostage_s:
                self._etat_accostage = "ECHEC"
                self.stats["accostages_ratees"] += 1
            return
        if self._phase not in ("ACCOSTAGE", "INSERTION", "RECUL") \
                or self._etat_accostage != "EN_ROUTE":
            return
        if self._phase == "RECUL" and self._horloge() < self._liberation_jusqua:
            self.loco.halt()                  # mains en train de lâcher
            self._t_accostage = self._horloge()
            return
        etat = self._pas_accostage()
        if etat == "ARRIVE" and self._phase == "RECUL":
            if self.manip.etat is Etat.LIBRE:
                self.manip.reinitialiser()    # dégagé : bras au port
            self._lancer_nav()
        elif etat == "ARRIVE":
            # Arrivé : d'abord s'immobiliser, PUIS bouger les bras.
            ouvrir = self._phase == "ACCOSTAGE" and self._insertion is not None
            self._stabiliser("PRESAISIE" if ouvrir else "ARRIVE")
        elif etat in ("ECHEC", "BLOQUE"):
            self._etat_accostage = "ECHEC"
            self.stats["accostages_ratees"] += 1
            if etat == "BLOQUE":
                # Collé à quelque chose : la prochaine sortie commence par un
                # recul, au lieu de repartir droit dans l'obstacle.
                self._accoste = True

    def _viser(self) -> None:
        """Corrige la cible latérale d'après une détection faite DE FACE."""
        v = self._colis_vise
        vus = [c for c in self.chercher(v.couleur)
               if math.dist(c.position[:2], v.position[:2]) < self.cfg.correction_visee_max_m]
        if not vus:
            self.stats["visees_sans_detection"] = self.stats.get("visees_sans_detection", 0) + 1
            self._journal(f"visée : colis {v.couleur} pas revu de face — cible inchangée")
            return
        n = min(vus, key=lambda c: math.dist(c.position[:2], v.position[:2]))
        # Seulement la composante LATÉRALE (perpendiculaire au cap de prise) :
        # la profondeur est fixée par le bord de la table.
        lx, ly = -math.sin(self.cap_prise), math.cos(self.cap_prise)
        e = (n.position[0] - v.position[0]) * lx + (n.position[1] - v.position[1]) * ly
        dx, dy = e * lx, e * ly
        for nom in ("_insertion", "_cible"):
            q = getattr(self, nom)
            if q is not None:
                setattr(self, nom, Pose2D(q.x + dx, q.y + dy, q.cap))
        self._colis_vise = Colis(v.couleur, (v.position[0] + dx, v.position[1] + dy,
                                             v.position[2]), v.confiance)
        self.stats["visees"] = self.stats.get("visees", 0) + 1
        self._journal(f"visée : colis {v.couleur} revu de face — correction latérale "
                      f"{100 * e:+.1f} cm")

    def _stabiliser(self, ensuite: str) -> None:
        self.loco.halt()
        self._phase_avant_stab, self._apres_stab = self._phase, ensuite
        self._phase = "STABILISATION"
        t, p = self._horloge(), self._pose()
        self._t_stab0, self._ref_stab = t, (t, p.x, p.y)

    def _immobile(self) -> bool:
        """Vrai quand la base n'a pas bougé de plus de derive_stable_m depuis
        duree_stabilisation_s (ou au bout de stabilisation_max_s). Le temps
        d'arrêt de SÉCURITÉ ne compte pas : on attend d'être libre."""
        cfg, t, p = self.cfg, self._horloge(), self._pose()
        t_ref, x_ref, y_ref = self._ref_stab
        if math.hypot(p.x - x_ref, p.y - y_ref) > cfg.derive_stable_m:
            self._ref_stab = (t, p.x, p.y)       # il bouge encore
            t_ref = t
        return (t - t_ref >= cfg.duree_stabilisation_s
                or t - self._t_stab0 >= cfg.stabilisation_max_s)

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
        t = self._horloge()
        v = getattr(self.loco, "dernier_verdict", None)
        if self._progres is None or self._progres[2] is not self._cible \
                or d < self._progres[0] - 0.01 \
                or (v is not None and v.facteur < 0.5):
            # Il avance — ou la SÉCURITÉ le retient (personne proche) : ce
            # n'est pas un blocage, le délai repart de zéro.
            self._progres = (d, t, self._cible)          # il avance : on repart
        elif t - self._progres[1] > cfg.delai_blocage_s and d > cfg.tol_position * 2:
            self.loco.halt()
            self.stats["blocages"] = self.stats.get("blocages", 0) + 1
            self._journal(f"accostage BLOQUÉ à {d:.2f} m de la cible — arrêt, "
                          f"pas d'insistance")
            self._progres = None
            return "BLOQUE"
        if d > self._d0 + cfg.marge_divergence or d > cfg.distance_max_accostage \
                or self._horloge() - self._t_accostage > cfg.delai_accostage_s:
            self.loco.halt()
            return "ECHEC"
        vx = self._franche(cfg.gain * ex, self._vmax) if abs(ex) > cfg.tol_position else 0.0
        vy = self._franche(cfg.gain * ey, self._vmax) if abs(ey) > cfg.tol_position else 0.0
        wz = self._franche(cfg.gain * ecap, cfg.w_max) if abs(ecap) > cfg.tol_cap else 0.0
        if vx == vy == wz == 0.0:
            self.loco.halt()
            return "ARRIVE"
        self.loco.set_velocity(vx, vy, wz)
        return "EN_ROUTE"

    # ---- manipulation -----------------------------------------------------
    def prendre(self, colis: Colis) -> Tuple[bool, str]:
        v = self._colis_vise
        if v is not None and v.couleur == colis.couleur \
                and math.dist(v.position[:2], colis.position[:2]) < self.cfg.correction_visee_max_m:
            colis = v                    # position corrigée par la visée
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
        """Après une prise ratée : on OUVRE GRAND et on attend que les mains
        aient lâché avant de reculer — sinon on emporte le carton avec soi."""
        self.manip.reinitialiser(large=True)
        self._liberation_jusqua = self._horloge() + self.cfg.duree_liberation_s
