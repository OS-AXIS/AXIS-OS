"""
Façade Robot de l'app de tri, branchée sur les services d'AXIS-OS.

La logique de tri (logique.py) ne parle qu'à cette façade, qui implémente
son interface `Robot` ; la façade, elle, traduit chaque demande en appels
aux services de l'OS :

    chercher   -> PerceptionService.find(..., FRAME_WORLD)
    aller      -> NavigationService.aller_vers(poste)
    accoster   -> NavigationService jusqu'à une pré-pose, puis accostage fin
                  par consignes de vitesse au LocomotionService
    prendre    -> ManipulationService.prendre / poser / reinitialiser
    sécurité   -> LocomotionService.dernier_verdict (GardienProximite)

FRAME_WORLD est le repère « monde », fixe, celui de la carte. Une pré-pose
est une position d'attente dégagée, face à la table, d'où part l'accostage
fin.

Rien ici ne contourne la sécurité : toute consigne de vitesse passe par le
LocomotionService, donc par le gardien de proximité (il ralentit ou arrête
le robot d'après le lidar, un télémètre laser qui balaie les alentours),
puis par le garde-fou des articulations. La façade n'écrit jamais
directement une articulation.

Pourquoi un accostage fin : le planificateur de trajectoire gonfle chaque
obstacle du rayon du robot plus une marge ; il ne peut donc pas amener le
robot à 29 cm d'un carton posé au bord d'une table. Comme un chariot qui
vient s'amarrer à un poste, la navigation amène le robot à une pré-pose
dégagée, face à la table, puis un contrôleur d'accostage parcourt les
derniers centimètres vers une cible calculée à partir de la position du
carton, sans jamais la dépasser.

Configuration de site requise : la table de prise doit figurer dans les
obstacles du gardien avec sa hauteur de chargement (Rectangle.zmax =
hauteur du plateau + hauteur d'un carton). Les cartons posés dessus sont
alors « expliqués » par la table, et le gardien ne s'en inquiète pas ; sans
cela, les cartons voisins de celui qu'on vise arrêteraient le robot à
chaque prise. Un humain penché au-dessus de la table, plus haut que zmax,
reste détecté.

Les pièges dits « constatés sur GPU » (sur carte graphique) ou « sur RTX
5090 » (un modèle de carte graphique) ont été observés lors des essais de
la démonstration en simulation physique (Isaac Sim) ; voir
docs/demo_tri_usine.md.
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
    """Ramène un angle, en radians, dans l'intervalle [-π, π[."""
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass(frozen=True)
class ConfigFacade:
    """Réglages de la façade. Distances en mètres, vitesses en m/s (rad/s
    pour les rotations), angles en radians, durées en secondes."""

    # Requête envoyée à la perception ; {couleur} y est remplacé par la
    # couleur cherchée. Avec PerceptionCouleur (couleur.py), qui attend le
    # seul nom de la couleur, on passe requete="{couleur}".
    requete: str = "a {couleur} box"
    confiance_min: float = 0.15    # confiance minimale d'une détection
    # Hauteur admise (z, repère monde) pour la position détectée d'un
    # colis : un colis posé sur la table, jamais un objet au sol.
    z_min: float = 0.50
    z_max: float = 1.30
    fusion_m: float = 0.10         # deux détections plus proches = un colis
    # Dimensions des colis ; la largeur est celle prise entre les mains.
    largeur: float = 0.24
    profondeur: float = 0.20
    hauteur: float = 0.20
    # La caméra de profondeur mesure la position de la face avant du carton,
    # celle tournée vers la caméra (l'estimateur retient le quartile des
    # points les plus proches). Pour obtenir le centre, on ajoute une
    # demi-profondeur dans la direction du regard (le cap de prise).
    position_face_avant: bool = True
    # Centre des mains devant la base. En prise mesurée (bras avec
    # mesures d'effort), c'est la distance minimale : le service de
    # manipulation l'allonge s'il le faut pour que les avant-bras passent
    # au-dessus du plateau (constaté sur GPU : ils s'y appuyaient).
    distance_prise: float = 0.29
    recul_pre_pose: float = 0.45   # pré-pose, en arrière de la cible
    # Approche finale « comme un humain ». Pourquoi : constaté sur GPU, le
    # robot avançait les bras en posture de port (celle où il tient un
    # carton devant lui) et heurtait le carton. Désormais, il s'arrête à
    # `recul_insertion` de la cible, les mains encore devant le carton ;
    # il ouvre largement les bras, à l'arrêt ; puis il avance lentement
    # (`v_insertion`) pour les glisser autour du carton.
    recul_insertion: float = 0.18
    v_insertion: float = 0.08
    # Blocage : si le robot pousse sans se rapprocher de la cible d'au moins
    # 1 cm en `delai_blocage_s` (2 s), il est bloqué. Constaté sur GPU :
    # carton coincé contre le bord de la table de dépose, robot qui
    # insistait. Il s'arrête donc, et l'app réessaie après un recul.
    delai_blocage_s: float = 2.0
    # Un blocage à moins de `tol_blocage_prise` d'une cible de prise est
    # accepté comme une arrivée (voir _pas_accostage).
    tol_blocage_prise: float = 0.06
    # Stabilisation (champs `duree_stabilisation_s`, `derive_stable_m` et
    # `stabilisation_max_s`, plus bas) : une fois arrivé, le robot
    # s'immobilise avant de bouger les bras (ouvrir, prendre, poser).
    # Pourquoi : constaté sur GPU, il ouvrait ou serrait les bras en
    # piétinant encore (la politique de marche, le réseau de neurones qui
    # commande les jambes, finit ses pas après la consigne d'arrêt) ; il
    # touchait alors le carton et le déplaçait. Immobile signifie : moins de
    # `derive_stable_m` de déplacement pendant `duree_stabilisation_s`.
    # L'attente dure au plus `stabilisation_max_s`.
    #
    # Visée : arrivé à la pré-pose, face au carton, le robot le regarde de
    # nouveau avant l'approche finale. Pourquoi : constaté sur RTX 5090, la
    # position perçue depuis le poste de prise était fausse de 4 à 9 cm sur
    # le côté (l'erreur croît avec l'angle de vue), et une main poussait le
    # carton. Vu de face, le carton est près de l'axe de la caméra : l'erreur
    # est bien plus faible. La correction est bornée par
    # `correction_visee_max_m` : au-delà, ce n'est pas le même carton.
    duree_visee_s: float = 0.6
    correction_visee_max_m: float = 0.15
    duree_stabilisation_s: float = 0.6
    # Après une prise ratée : temps laissé aux mains pour s'ouvrir en grand
    # avant de reculer (voir reinitialiser_manipulation).
    duree_liberation_s: float = 1.0
    derive_stable_m: float = 0.01
    stabilisation_max_s: float = 3.0
    # Contrôleur d'accostage : tolérances d'arrivée (position, cap), gain
    # proportionnel appliqué à l'écart, vitesses maximales d'avance et de
    # rotation.
    tol_position: float = 0.015
    tol_cap: float = 0.04
    gain: float = 1.2
    v_max: float = 0.12
    w_max: float = 0.30
    # Vitesse minimale d'une consigne non nulle : plus lente, elle tomberait
    # dans la « bande morte » de la marche, où le robot ne bouge pas.
    v_min: float = 0.06
    # Durée maximale d'un accostage ; le temps d'arrêt dû à la sécurité n'y
    # est pas compté (voir tick).
    delai_accostage_s: float = 25.0
    recul_sortie: float = 0.35     # recul en ligne droite avant de repartir
    # Un accostage qui diverge est abandonné : la distance à la cible dépasse
    # la distance de départ plus cette marge. (La première version utilisait
    # un seuil fixe de 0.60 m, dépassé d'emblée par un poste à 0.53 m auquel
    # s'ajoutait la tolérance d'arrivée de la navigation : l'accostage
    # échouait avant même d'avoir bougé.)
    marge_divergence: float = 0.10
    distance_max_accostage: float = 1.00  # au-delà, la cible n'est pas crédible


@dataclass(frozen=True)
class PosteDepose:
    """Un poste de dépose équipé d'un plateau, où le carton est posé (et
    non relâché à hauteur de port).

    avance_m : distance qu'il reste à parcourir en accostage fin, depuis la
               pose du poste dans la carte (dégagée, hors de la marge du
               planificateur) jusqu'à la pose de dépose, où le carton tenu
               se trouve au-dessus du plateau.
    hauteur  : hauteur du plateau (en mètres, dans le repère monde).
    """
    avance_m: float
    hauteur: float


class FacadeServices:
    """L'interface `Robot` de logique.py, réalisée avec les services
    d'AXIS-OS (navigation, locomotion, manipulation, perception).

    La façade est elle-même une petite machine à états, que tick() fait
    avancer à chaque pas de commande ; la logique de tri n'en voit que le
    résumé rendu par navigation() et manipulation().
    """

    def __init__(self, navigation, locomotion, manipulation, perception,
                 source_pose: Callable[[], Pose2D], horloge: Callable[[], float],
                 cap_prise: float, journal: Callable[[str], None] = print,
                 cfg: ConfigFacade = ConfigFacade(),
                 postes_depose: Optional[Dict[str, PosteDepose]] = None):
        """
        source_pose   : rend la pose courante de la base (x, y, cap) dans
                        la carte.
        horloge       : rend l'instant courant, en secondes.
        cap_prise     : orientation du robot face à la table de prise (celle
                        du poste de prise dans la carte).
        journal       : reçoit les messages destinés à l'opérateur.
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
        # Phase courante de la façade :
        #   NAV             navigation simple vers un poste ;
        #   NAV_ACCOSTAGE   navigation vers une pré-pose, suivie d'un
        #                   accostage fin (-> VISEE, FRANCHISSEMENT ou
        #                   ACCOSTAGE) ;
        #   VISEE           arrêt à la pré-pose pour revoir le carton de face ;
        #   FRANCHISSEMENT  levée du carton tenu au-dessus du plateau de
        #                   dépose, avant d'avancer ;
        #   ACCOSTAGE       contrôleur d'accostage fin vers la cible ;
        #   STABILISATION   attente que la base soit immobile ;
        #   PRESAISIE       ouverture des bras, robot à l'arrêt ;
        #   INSERTION       avance lente, mains autour du carton ;
        #   RECUL           recul en ligne droite (-> NAV...) ;
        #   ECHEC           navigation refusée.
        self._phase: Optional[str] = None
        self._cible: Optional[Pose2D] = None     # cible de l'accostage fin
        # Ce que navigation() rend pendant l'accostage : EN_ROUTE, ARRIVE
        # ou ECHEC.
        self._etat_accostage = "EN_ROUTE"
        self._t_accostage = 0.0
        self._accoste = False                    # collé à une table
        self._dest = None                        # destination de navigation
        self._apres_nav: Optional[Pose2D] = None # accostage à faire ensuite
        self._poste_depose: Optional[str] = None
        # Distance à la cible au début de l'accostage : sert à repérer un
        # accostage qui diverge.
        self._d0 = 0.0
        self._insertion: Optional[Pose2D] = None   # cible finale, après pré-saisie
        # Ce qui suit la stabilisation, et la phase à retrouver ensuite.
        self._apres_stab, self._phase_avant_stab = "ARRIVE", "ACCOSTAGE"
        # Jusqu'à cet instant, les mains lâchent le carton : pas de recul.
        self._liberation_jusqua = 0.0
        self._colis_vise: Optional[Colis] = None   # colis à revoir de face
        self.veut_percevoir = False        # la boucle doit rendre la caméra
        self._t_visee = 0.0
        # Début de la stabilisation ; dernière position de référence
        # (instant, x, y) pour juger de l'immobilité.
        self._t_stab0, self._ref_stab = 0.0, (0.0, 0.0, 0.0)
        # Dernier progrès vers la cible, pour repérer un blocage.
        self._progres = None                       # (distance, instant, cible)
        self._t_tick: Optional[float] = None       # instant du tick précédent
        self._penche_signale = False
        # Vitesse maximale de l'accostage en cours (réduite à l'insertion).
        self._vmax = cfg.v_max
        self.stats = {"accostages": 0, "accostages_ratees": 0,
                      "detections_filtrees": 0, "reculs": 0}

    # ---- temps et sécurité ----------------------------------------------
    def maintenant(self) -> float:
        return self._horloge()

    def bloque_par_securite(self) -> bool:
        """Vrai quand le gardien de proximité a ramené la vitesse à zéro
        (une personne ou un objet trop près)."""
        v = getattr(self.loco, "dernier_verdict", None)
        return v is not None and v.facteur <= 0.0

    def journal(self, message: str) -> None:
        self._journal(message)

    # ---- perception -----------------------------------------------------
    def _centre(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        """Centre du carton à partir du point mesuré (voir
        ConfigFacade.position_face_avant)."""
        if not self.cfg.position_face_avant:
            return (x, y, z)
        d = self.cfg.profondeur / 2.0
        return (x + d * math.cos(self.cap_prise), y + d * math.sin(self.cap_prise), z)

    def chercher(self, couleur: str) -> List[Colis]:
        """Colis de cette couleur vus sur la table, du plus sûr au moins
        sûr. Sont écartées les détections sans position, peu confiantes ou
        hors de la hauteur admise ; deux détections trop proches comptent
        pour un seul colis."""
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
            # Les détections arrivent triées par confiance : de deux
            # détections trop proches, on garde la première.
            if any(math.dist(c, k.position) < self.cfg.fusion_m for k in colis):
                continue
            colis.append(Colis(couleur, c, d.confidence))
        return colis

    # ---- navigation et accostage -----------------------------------------
    #
    # Toute sortie d'un accostage commence par un recul en ligne droite.
    # Pourquoi : pivoter sur place collé à une table, c'est y cogner un genou
    # ou le carton. Viennent ensuite la navigation, puis, s'il y a lieu, un
    # nouvel accostage fin.

    def _partir(self, dest, accostage: Optional[Pose2D],
                poste_depose: Optional[str] = None) -> Tuple[bool, str]:
        """Départ vers `dest` (un poste ou une pose), suivi de l'accostage
        `accostage` s'il est donné. Commence par un recul si le robot est
        collé à une table."""
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
        """Navigation vers un poste. Un poste de dépose équipé d'un plateau
        est atteint en deux temps : navigation jusqu'à sa pose dans la
        carte, puis accostage fin de `avance_m` jusqu'au-dessus du
        plateau."""
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
        """Approche d'un colis à prendre, en trois poses alignées sur le cap
        de prise : la pré-pose (navigation), la pose d'attente, mains encore
        devant le carton (accostage fin), puis la cible de prise, mains
        autour du carton (insertion, après l'ouverture des bras). Une cible
        à plus de 3 m du robot est refusée : ce colis n'est pas celui de la
        table."""
        c, s = math.cos(self.cap_prise), math.sin(self.cap_prise)
        cx, cy, _ = colis.position
        d = self.cfg.distance_prise
        if getattr(self.manip, "mesures", None) is not None:
            d, raison = self.manip.distance_de_prise(
                Objet(colis.position, self.cfg.largeur, self.cfg.profondeur, self.cfg.hauteur),
                d_min=self.cfg.distance_prise)
            if d is None:
                return False, f"prise impossible : {raison}"
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
        """État du déplacement en cours, vu par la logique de tri :
        EN_ROUTE, ARRIVE ou ECHEC. Les phases internes (recul, accostage,
        stabilisation…) se résument à EN_ROUTE tant qu'elles durent."""
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
        # Y compris NAV_ACCOSTAGE arrivé : c'est tick() qui enchaîne sur
        # l'accostage fin.
        return "EN_ROUTE"

    def _franche(self, v: float, vmax: float) -> float:
        """Consigne de vitesse « franche » : nulle, ou bornée entre v_min
        (hors de la bande morte de la marche) et `vmax`, signe conservé."""
        if abs(v) < 1e-9:
            return 0.0
        return math.copysign(min(vmax, max(self.cfg.v_min, abs(v))), v)

    def tick(self) -> None:
        """Fait avancer la façade d'un pas. À appeler à chaque pas de
        commande, après navigation.tick() (le tick du service de
        navigation)."""
        # Le temps passé à l'arrêt à cause de la sécurité ne compte pas dans
        # les délais d'accostage : attendre qu'une personne s'éloigne n'est
        # pas un échec. (Constaté sur GPU : un collègue resté près du poste
        # faisait échouer l'accostage par dépassement de délai.)
        maintenant = self._horloge()
        dt = 0.0 if self._t_tick is None else maintenant - self._t_tick
        self._t_tick = maintenant
        if self.bloque_par_securite():
            self._t_accostage += dt
            self._t_stab0 += dt
        if self.manip.etat is Etat.RAMENER or getattr(self.manip, "penche", False):
            # Carton en train de revenir contre le buste, ou penché : la
            # base ne bouge pas (le temps ne compte pas dans les délais).
            self.loco.halt()
            self._t_accostage += dt
            self._t_stab0 += dt
            if self.manip.a_reposer and not self._penche_signale:
                self._penche_signale = True
                self._journal(f"transport arrêté : {self.manip.raison}")
            return
        self._penche_signale = False
        if self._phase == "NAV_ACCOSTAGE" and self.nav.etat is EtatNavigation.ARRIVE:
            self._cible = self._apres_nav
            self._vmax = self.cfg.v_max
            self._phase, self._etat_accostage = "ACCOSTAGE", "EN_ROUTE"
            pd = self.postes_depose.get(self._poste_depose)
            if pd is not None and self.manip.etat is Etat.TENU \
                    and not self.manip.degage_au_dessus(pd.hauteur):
                # Devant la table, et avant d'avancer : lever le carton
                # au-dessus du plateau, pour ne pas le cogner contre le bord.
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
                # Le robot reste immobile le temps qu'une image fraîche soit
                # rendue (veut_percevoir).
                return
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
                # Immobile devant le carton : on ouvre les bras en grand
                # (pré-saisie).
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
            # Arrivé : d'abord s'immobiliser, ensuite seulement bouger les
            # bras (voir ConfigFacade, stabilisation).
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
        """Corrige la position latérale de la cible d'après une nouvelle
        détection, faite de face depuis la pré-pose (voir ConfigFacade,
        visée). Sans détection assez proche, la cible reste inchangée."""
        v = self._colis_vise
        vus = [c for c in self.chercher(v.couleur)
               if math.dist(c.position[:2], v.position[:2]) < self.cfg.correction_visee_max_m]
        if not vus:
            self.stats["visees_sans_detection"] = self.stats.get("visees_sans_detection", 0) + 1
            self._journal(f"visée : colis {v.couleur} pas revu de face — cible inchangée")
            return
        n = min(vus, key=lambda c: math.dist(c.position[:2], v.position[:2]))
        # On ne corrige que la composante latérale, perpendiculaire au cap de
        # prise : la profondeur, elle, est fixée par le bord de la table.
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
        """Arrête la base et attend qu'elle soit immobile ; `ensuite` dit ce
        qui suivra (PRESAISIE ou ARRIVE)."""
        self.loco.halt()
        self._phase_avant_stab, self._apres_stab = self._phase, ensuite
        self._phase = "STABILISATION"
        t, p = self._horloge(), self._pose()
        self._t_stab0, self._ref_stab = t, (t, p.x, p.y)

    def _immobile(self) -> bool:
        """Vrai quand la base n'a pas bougé de plus de derive_stable_m depuis
        duree_stabilisation_s, ou au bout de stabilisation_max_s. Le temps
        d'arrêt dû à la sécurité ne compte pas dans cette attente : on
        attend que le passage soit libre."""
        cfg, t, p = self.cfg, self._horloge(), self._pose()
        t_ref, x_ref, y_ref = self._ref_stab
        if math.hypot(p.x - x_ref, p.y - y_ref) > cfg.derive_stable_m:
            self._ref_stab = (t, p.x, p.y)       # il bouge encore
            t_ref = t
        return (t - t_ref >= cfg.duree_stabilisation_s
                or t - self._t_stab0 >= cfg.stabilisation_max_s)

    def _pas_accostage(self) -> str:
        """Un pas du contrôleur d'accostage : vers self._cible, en ligne
        quasi droite, par consignes de vitesse au LocomotionService (donc
        sous le contrôle du gardien de proximité). Ne va jamais au-delà de
        la cible.

        Rend EN_ROUTE, ARRIVE, BLOQUE (aucun progrès depuis
        delai_blocage_s) ou ECHEC (divergence, cible trop lointaine ou délai
        dépassé)."""
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
            # Le robot avance, ou bien la sécurité le retient (personne
            # proche, vitesse réduite de plus de moitié) : ce n'est pas un
            # blocage, le délai repart de zéro.
            self._progres = (d, t, self._cible)
        elif t - self._progres[1] > cfg.delai_blocage_s and d > cfg.tol_position * 2:
            self.loco.halt()
            self._progres = None
            if self._phase in ("ACCOSTAGE", "INSERTION") and self._insertion is not None \
                    and d < cfg.tol_blocage_prise:
                # Presque arrivé devant le carton. Constaté sur RTX 5090 :
                # « BLOQUÉ à 0.05 m », la marche n'avançant plus par si
                # petits pas. Mains grandes ouvertes, il reste 10 cm de jeu
                # de chaque côté : on continue, et c'est la règle « entre
                # les mains » du service de manipulation qui décide si la
                # prise est possible.
                self.stats["arrivees_approchees"] = self.stats.get("arrivees_approchees", 0) + 1
                self._journal(f"accostage : {self._phase} arrêté à {100 * d:.0f} cm de la "
                              f"cible (devant {100 * ex:+.0f}, côté {100 * ey:+.0f} cm) — assez "
                              f"près, on continue")
                return "ARRIVE"
            self.stats["blocages"] = self.stats.get("blocages", 0) + 1
            self._journal(f"accostage BLOQUÉ ({self._phase}) à {d:.2f} m de la cible "
                          f"(devant {100 * ex:+.0f}, côté {100 * ey:+.0f} cm) — arrêt, "
                          f"pas d'insistance")
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
        """Lance la prise du colis. Si la visée l'a revu de face, c'est sa
        position corrigée qui est transmise au service de manipulation."""
        v = self._colis_vise
        if v is not None and v.couleur == colis.couleur \
                and math.dist(v.position[:2], colis.position[:2]) < self.cfg.correction_visee_max_m:
            colis = v                    # position corrigée par la visée
        r = self.manip.prendre(Objet(colis.position, self.cfg.largeur,
                                     self.cfg.profondeur, self.cfg.hauteur))
        return r.ok, r.reason or ""

    def poser(self) -> Tuple[bool, str]:
        """Lance la dépose : sur le plateau si le robot est accosté à un
        poste qui en a un, sinon en relâchant le carton à hauteur de port."""
        pd = self.postes_depose.get(self._poste_depose) if self._accoste else None
        r = self.manip.poser(hauteur_surface=pd.hauteur if pd else None)
        return r.ok, r.reason or ""

    def manipulation(self) -> str:
        """État des bras vu par la logique de tri : LIBRE, TENU, ECHEC, ou
        EN_COURS pour tout état intermédiaire."""
        e = self.manip.etat
        if e is Etat.TENU and getattr(self.manip, "a_reposer", False):
            return "ECHEC"              # penché malgré la correction
        if e in (Etat.LIBRE, Etat.TENU, Etat.ECHEC):
            return e.name
        return "EN_COURS"

    def suspendre(self, au_contact: bool = False) -> None:
        """Un autre pilote prend la base : la façade cesse toute consigne.

        C'est le cas pendant l'évaluation d'un VLA (modèle
        vision-langage-action, un réseau de neurones qui commande le robot
        directement d'après l'image et une consigne). Pourquoi tout arrêter :
        avec deux pilotes sur la même base, la consigne sauterait de l'un à
        l'autre sans que personne ne l'ait décidé. La navigation est donc
        annulée, l'accostage abandonné, et la base mise à l'arrêt, en
        équilibre.

        au_contact : l'autre pilote a laissé le robot contre une table ; la
        prochaine sortie commencera par un recul, comme après un accostage."""
        self.nav.annuler()
        self._phase, self._etat_accostage = "NAV", "ARRIVE"
        self._apres_nav, self._insertion, self._colis_vise = None, None, None
        self.veut_percevoir = False
        self._accoste = self._accoste or au_contact
        self.loco.halt()

    def reprendre_bras(self) -> None:
        """À appeler quand un autre programme a commandé les bras (le VLA)
        et que ceux-ci sont revenus en posture de port : le service de
        manipulation repart de la posture réelle, sans saut de consigne."""
        self.manip.resynchroniser()

    def reinitialiser_manipulation(self) -> None:
        """Après une prise ratée : les mains s'ouvrent en grand, et le robot
        attend qu'elles aient lâché avant de reculer. Sinon, il emporterait
        le carton avec lui."""
        self.manip.reinitialiser(large=True)
        self._liberation_jusqua = self._horloge() + self.cfg.duree_liberation_s
