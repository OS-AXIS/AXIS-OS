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

import numpy as np

from axis_os.apps.tri_colis.logique import Colis
from axis_os.hal.core.frames import FRAME_WORLD
from axis_os.services.manipulation.service import Etat, Objet
from axis_os.services.navigation.service import EtatNavigation, Pose2D
from axis_os.services.perception.boite import FusionCarton, ajuster_boite


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
    # Boîte orientée (perception/boite.py) ajustée aux points 3D de chaque
    # détection, fusionnés image après image pendant l'approche : centre et
    # lacet. Au-delà de `residu_max_m`, ou dessus vu sur moins de
    # `couverture_min`, le carton est « mal vu » et n'est pas pris (jamais
    # deviné). Un nouveau carton : à plus de `fusion_reset_m` du précédent.
    residu_max_m: float = 0.015
    couverture_min: float = 0.6
    fusion_reset_m: float = 0.15
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
    #
    # 0.28 et non plus 0.18 : c'est à cette pose, bras ouverts, que le robot
    # regarde le carton une dernière fois avant de le prendre (voir
    # « observation » plus bas). Sur le banc T1 (caméra de tête montée à
    # 0.40 m au-dessus du bassin, 15° vers le bas, champ vertical de 47°),
    # le bord bas de l'image passe 38.7° sous l'horizontale : à 0.18 m, on
    # ne voyait que 66 % du dessus du carton et pas sa face avant ; à
    # 0.28 m, l'arête avant du dessus est à 35°, tout le dessus est vu. À
    # la pose de prise, le carton est entièrement hors du champ.
    recul_insertion: float = 0.28
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
    # Observation après l'arrêt. Constaté sur GPU (essai T1 de main,
    # novembre 2026) : trois fois, le carton avait été déplacé de 3 à 10 cm
    # pendant l'approche, et la prise visait la position estimée AVANT, sur
    # une seule image prise depuis la pré-pose. Désormais, arrivé à la pose
    # d'attente, robot immobile et bras ouverts, le robot regarde de
    # nouveau : il faut au moins `images_observation` images prises APRÈS
    # cet arrêt (horodatées par la caméra), distantes d'au moins
    # `intervalle_observation_s`, dont les boîtes, ajustées chacune sur sa
    # seule image, s'accordent à `tol_observation_m` et
    # `tol_observation_deg` près. Sinon il regarde encore, jusqu'à
    # `observation_max_s` ; au-delà, l'accostage échoue (l'app recule et
    # cherche de nouveau). Si cette estimation déplace la cible de plus de
    # `tol_realignement_m`, le robot se réaligne devant le carton (au plus
    # `realignements_max` fois) et regarde de nouveau.
    images_observation: int = 3
    intervalle_observation_s: float = 0.1
    tol_observation_m: float = 0.01
    tol_observation_deg: float = 3.0
    observation_max_s: float = 4.0
    tol_realignement_m: float = 0.03
    realignements_max: int = 2
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
                 postes_depose: Optional[Dict[str, PosteDepose]] = None,
                 points_lidar: Optional[Callable[[], object]] = None):
        """
        source_pose   : rend la pose courante de la base (x, y, cap) dans
                        la carte.
        horloge       : rend l'instant courant, en secondes.
        cap_prise     : orientation du robot face à la table de prise (celle
                        du poste de prise dans la carte).
        journal       : reçoit les messages destinés à l'opérateur.
        postes_depose : postes équipés d'un plateau. Aux autres, le carton
                        est relâché à hauteur de port.
        points_lidar  : rend les points du LiDAR (N × 3, repère monde), ou
                        None : ceux du carton s'ajoutent à ceux de la caméra.
        """
        self.nav, self.loco, self.manip = navigation, locomotion, manipulation
        self.percep = perception
        self._pose, self._horloge = source_pose, horloge
        self.cap_prise = cap_prise
        self._journal = journal
        self.cfg = cfg
        self.postes_depose = dict(postes_depose or {})
        self._points_lidar = points_lidar
        # Cartons suivis : couleur, nuages fusionnés, dernière boîte ajustée.
        self._suivis: List[dict] = []
        self.derniere_boite: Dict[str, object] = {}
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
        #   OBSERVATION     le carton revu, robot immobile et bras ouverts ;
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
        # Observation après l'arrêt : en cours (dict), son résultat (le colis
        # revu, seul transmis à la prise), et si la prise l'exige (après un
        # accostage de prise).
        self._obs: Optional[dict] = None
        self._observe: Optional[Colis] = None
        self._observation_requise = False
        self._realignements = 0
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

    def chercher(self, couleur: str, fusion: bool = True) -> List[Colis]:
        """Colis de cette couleur vus sur la table, du plus sûr au moins
        sûr. Sont écartées les détections sans position, peu confiantes ou
        hors de la hauteur admise ; deux détections trop proches comptent
        pour un seul colis. fusion=False : boîte ajustée sur cette seule
        image (l'observation compare les images entre elles).
        self.instant_image : instant de l'image lue (horodatage de la
        caméra ; à défaut, celui de l'appel)."""
        maintenant = self._horloge()
        r = self.percep.find(self.cfg.requete.format(couleur=couleur),
                             now_ns=int(maintenant * 1e9), frame=FRAME_WORLD)
        t_ns = getattr(getattr(self.percep, "perception", self.percep), "derniere_image_ns", None)
        self.instant_image = maintenant if t_ns is None else t_ns / 1e9
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
            lacet, mal_vu, vu = 0.0, "", ""
            b = self._boite(couleur, d, c, fusion)
            if b is not None:
                c, lacet, mal_vu, vu = b
            # Les détections arrivent triées par confiance : de deux
            # détections trop proches, on garde la première.
            if any(math.dist(c, k.position) < self.cfg.fusion_m for k in colis):
                continue
            colis.append(Colis(couleur, c, d.confidence, lacet, mal_vu, vu))
        return colis

    def _boite(self, couleur: str, d, c_face, fusion: bool = True):
        """Boîte orientée du carton de la détection `d` : ses points 3D
        (ceux que la perception a gardés, masque compris), fusionnés avec les
        images précédentes du même carton et les points du LiDAR proches,
        puis ajustés (perception/boite.py). Rend (centre, lacet, raison s'il
        est mal vu, résumé), ou None sans points (on garde alors le centre
        d'après la face avant)."""
        service = getattr(self.percep, "perception", self.percep)
        pts = getattr(service, "derniers_points", {}).get(tuple(d.bbox))
        if pts is None or len(pts) < 30:
            return None
        if not fusion:
            # Cette image seule, sans toucher aux cartons suivis.
            suivi = {"couleur": couleur, "fusion": FusionCarton(), "centre": c_face}
            suivi["fusion"].ajouter(self._horloge(), pts)
            return self._ajuster(suivi, c_face)
        suivi = next((s for s in self._suivis if s["couleur"] == couleur and math.dist(
            s["centre"][:2], c_face[:2]) < self.cfg.fusion_reset_m), None)
        if suivi is None:
            suivi = {"couleur": couleur, "fusion": FusionCarton(), "centre": c_face}
            self._suivis = [s for s in self._suivis if s["couleur"] != couleur][-4:] + [suivi]
        suivi["fusion"].ajouter(self._horloge(), pts)
        return self._ajuster(suivi, c_face)

    def _ajuster(self, suivi: dict, c_face):
        """Boîte orientée ajustée aux nuages de `suivi` et aux points proches
        du LiDAR (voir _boite)."""
        nuage = suivi["fusion"].points()
        if self._points_lidar is not None:
            try:
                lid = self._points_lidar()
            except Exception:              # noqa: BLE001 — capteur en panne : sans lui
                lid = None
            if lid is not None and len(lid):
                lid = np.asarray(lid, dtype=float)
                r = math.hypot(self.cfg.largeur, self.cfg.profondeur) / 2 + 0.02
                pres = (np.hypot(lid[:, 0] - suivi["centre"][0], lid[:, 1] - suivi["centre"][1]) < r) \
                    & (lid[:, 2] > suivi["centre"][2] - self.cfg.hauteur / 2 - 0.05) \
                    & (lid[:, 2] < suivi["centre"][2] + self.cfg.hauteur / 2 + 0.03)
                if pres.any():
                    nuage = np.concatenate([nuage, lid[pres]])
        p = self._pose()
        vers = (p.x - c_face[0], p.y - c_face[1], 0.0)
        b = ajuster_boite(nuage, (self.cfg.largeur, self.cfg.profondeur, self.cfg.hauteur),
                          np.asarray(vers, dtype=float))
        if b is None:
            return None
        suivi["centre"] = tuple(float(v) for v in b.centre)
        self.derniere_boite[suivi["couleur"]] = b
        self.stats["boites"] = self.stats.get("boites", 0) + 1
        mal = ""
        if b.residu_m > self.cfg.residu_max_m:
            mal = (f"résidu de l'ajustement {100 * b.residu_m:.1f} cm (au plus "
                   f"{100 * self.cfg.residu_max_m:.1f})")
        elif b.couverture < self.cfg.couverture_min:
            mal = (f"dessus vu à {100 * b.couverture:.0f} % (au moins "
                   f"{100 * self.cfg.couverture_min:.0f})")
        vu = (f"centre ({b.centre[0]:.3f}, {b.centre[1]:.3f}, {b.centre[2]:.3f}), lacet "
              f"{math.degrees(b.lacet):+.1f}°, résidu {100 * b.residu_m:.2f} cm, dessus vu à "
              f"{100 * b.couverture:.0f} %, {suivi['fusion'].images} image(s)")
        return suivi["centre"], b.lacet, mal, vu

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
        self._obs, self._observe, self._observation_requise = None, None, False
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
        table.

        Toute estimation antérieure du carton est oubliée : il sera revu à
        la visée, puis après l'arrêt devant lui (voir ConfigFacade,
        observation)."""
        c, s = math.cos(self.cap_prise), math.sin(self.cap_prise)
        self._oublier(colis.couleur)
        cible, raison = self._cible_prise(colis.position)
        if cible is None:
            return False, raison
        p = self._pose()
        if math.hypot(cible.x - p.x, cible.y - p.y) > 3.0:
            return False, "colis à plus de 3 m : ce n'est pas celui de la table"
        r0, ri = self.cfg.recul_pre_pose, self.cfg.recul_insertion
        pre = Pose2D(cible.x - r0 * c, cible.y - r0 * s, self.cap_prise)
        attente = Pose2D(cible.x - ri * c, cible.y - ri * s, self.cap_prise)
        ok, raison = self._partir(pre, attente)
        self._insertion = cible
        self._colis_vise = colis
        self._observation_requise, self._realignements = True, 0
        return ok, raison

    def _cible_prise(self, centre) -> Tuple[Optional[Pose2D], str]:
        """Pose de prise devant un carton de centre `centre` : sur le cap de
        prise, à la distance de prise (allongée par le service de
        manipulation si les avant-bras doivent passer au-dessus du
        plateau). (None, raison) si aucune ne convient."""
        d = self.cfg.distance_prise
        if getattr(self.manip, "mesures", None) is not None:
            d, raison = self.manip.distance_de_prise(
                Objet(tuple(centre), self.cfg.largeur, self.cfg.profondeur, self.cfg.hauteur),
                d_min=self.cfg.distance_prise)
            if d is None:
                return None, f"prise impossible : {raison}"
        c, s = math.cos(self.cap_prise), math.sin(self.cap_prise)
        return Pose2D(centre[0] - d * c, centre[1] - d * s, self.cap_prise), ""

    def _oublier(self, couleur: Optional[str] = None) -> None:
        """Oublie les estimations d'un carton (de tous, sans couleur) :
        nuages fusionnés, dernière boîte, colis visé et observé. À chaque
        nouvel accostage et à chaque nouvel essai : un carton a pu bouger
        entre-temps (constaté sur GPU, voir ConfigFacade, observation)."""
        self._suivis = [s for s in self._suivis if couleur is not None and s["couleur"] != couleur]
        if couleur is None:
            self.derniere_boite.clear()
        else:
            self.derniere_boite.pop(couleur, None)
        self._colis_vise, self._observe, self._obs = None, None, None

    def navigation(self) -> str:
        """État du déplacement en cours, vu par la logique de tri :
        EN_ROUTE, ARRIVE ou ECHEC. Les phases internes (recul, accostage,
        stabilisation…) se résument à EN_ROUTE tant qu'elles durent."""
        ph = self._phase
        if ph == "ECHEC":
            return "ECHEC"
        if ph in ("ACCOSTAGE", "PRESAISIE", "INSERTION", "FRANCHISSEMENT",
                  "STABILISATION", "VISEE", "OBSERVATION"):
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
            if self._obs is not None:
                self._obs["debut"] += dt
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
                if self._observation_requise and self._colis_vise is not None:
                    self._debut_observation()
                else:
                    self._lancer_insertion()
            elif self._horloge() - self._t_accostage > self.cfg.delai_accostage_s:
                self._etat_accostage = "ECHEC"
                self.stats["accostages_ratees"] += 1
            return
        if self._phase == "OBSERVATION" and self._etat_accostage == "EN_ROUTE":
            self.loco.halt()
            self._observer()
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

    def _lancer_insertion(self) -> None:
        """Avance lente, bras ouverts, jusqu'à la pose de prise."""
        self._phase, self._cible = "INSERTION", self._insertion
        self._vmax = self.cfg.v_insertion
        self._t_accostage = self._horloge()
        p = self._pose()
        self._d0 = math.hypot(self._cible.x - p.x, self._cible.y - p.y)

    def _debut_observation(self) -> None:
        """Robot immobile, bras ouverts : on revoit le carton (voir
        ConfigFacade, observation). Seules comptent les images prises après
        cet instant."""
        t, p = self._horloge(), self._pose()
        self._phase = "OBSERVATION"
        self._obs = {"debut": t, "t0": t, "pose": (p.x, p.y), "images": [], "appel": -math.inf,
                     "image_prec": None, "lues": 0, "rejets": []}
        self._observe = None
        self.veut_percevoir = True
        self.loco.halt()

    @staticmethod
    def _demi_tour(a: float) -> float:
        """Écart de lacet ramené dans [-π/2, π/2[ : une boîte retournée d'un
        demi-tour est la même boîte."""
        return (a + math.pi / 2) % math.pi - math.pi / 2

    def _dispersion(self, images: List[Colis]) -> Tuple[float, float]:
        """Plus grands écarts, deux à deux, entre les boîtes de ces images :
        centre (m) et lacet (degrés)."""
        de, dl = 0.0, 0.0
        for i, a in enumerate(images):
            for b in images[i + 1:]:
                de = max(de, math.dist(a.position, b.position))
                dl = max(dl, abs(math.degrees(self._demi_tour(a.lacet - b.lacet))))
        return de, dl

    def _observer(self) -> None:
        """Un pas de l'observation : une image de plus, si elle est neuve et
        prise après l'arrêt ; fin quand les dernières s'accordent."""
        cfg, o, v = self.cfg, self._obs, self._colis_vise
        t, p = self._horloge(), self._pose()
        if math.hypot(p.x - o["pose"][0], p.y - o["pose"][1]) > cfg.derive_stable_m:
            # La base a bougé : les images d'avant ne comptent plus.
            o.update(t0=t, pose=(p.x, p.y), images=[])
        if t - o["debut"] > cfg.observation_max_s:
            self._echec_observation()
            return
        if t - o["appel"] < cfg.intervalle_observation_s:
            return
        o["appel"] = t
        vus = self.chercher(v.couleur, fusion=False)
        t_img = self.instant_image
        if t_img <= o["t0"] or t_img == o["image_prec"]:
            return                         # image d'avant l'arrêt, ou déjà lue
        o["image_prec"] = t_img
        o["lues"] += 1
        ref = o["images"][-1] if o["images"] else v
        vus = [c for c in vus if math.dist(c.position[:2], ref.position[:2])
               < cfg.correction_visee_max_m]
        if not vus:
            o["rejets"].append("carton pas vu")
            return
        n = min(vus, key=lambda c: math.dist(c.position[:2], ref.position[:2]))
        if n.mal_vu:
            o["rejets"].append(f"mal vu : {n.mal_vu}")
            return
        o["images"] = (o["images"] + [n])[-cfg.images_observation:]
        if len(o["images"]) < cfg.images_observation:
            return
        de, dl = self._dispersion(o["images"])
        if de < cfg.tol_observation_m and dl < cfg.tol_observation_deg:
            self._fin_observation(o["images"], de, dl)

    def _echec_observation(self) -> None:
        o, v = self._obs, self._colis_vise
        de, dl = self._dispersion(o["images"]) if len(o["images"]) > 1 else (0.0, 0.0)
        rejets = sorted(set(o["rejets"]))
        self._journal(f"carton {v.couleur} pas revu de façon cohérente après l'arrêt en "
                      f"{self.cfg.observation_max_s:.0f} s : {o['lues']} image(s) neuve(s), "
                      f"{len(o['images'])} retenue(s) (écart max {100 * de:.1f} cm, {dl:.1f}°)"
                      + (f", rejets : {'; '.join(rejets)}" if rejets else "")
                      + " — pas de prise, nouvel accostage")
        self.stats["observations_ratees"] = self.stats.get("observations_ratees", 0) + 1
        self._obs, self.veut_percevoir = None, False
        self._etat_accostage = "ECHEC"
        self.stats["accostages_ratees"] += 1

    def _fin_observation(self, images: List[Colis], de: float, dl: float) -> None:
        """Les images s'accordent : le carton revu devient celui de la prise ;
        la cible suit, et s'il a trop bougé, le robot se réaligne."""
        cfg, v = self.cfg, self._colis_vise
        n = len(images)
        centre = tuple(float(np.mean([c.position[k] for c in images])) for k in range(3))
        l0 = images[0].lacet
        lacet = l0 + float(np.mean([self._demi_tour(c.lacet - l0) for c in images]))
        vu = (f"centre ({centre[0]:.3f}, {centre[1]:.3f}, {centre[2]:.3f}), lacet "
              f"{math.degrees(lacet):+.1f}°, {n} images prises après l'arrêt (écart max "
              f"{100 * de:.1f} cm, {dl:.1f}°)")
        obs = Colis(v.couleur, centre, max(c.confiance for c in images), lacet, "", vu)
        bouge = math.dist(centre[:2], v.position[:2])
        tourne = abs(math.degrees(self._demi_tour(lacet - v.lacet)))
        self._obs, self.veut_percevoir = None, False
        self.stats["observations"] = self.stats.get("observations", 0) + 1
        if bouge > cfg.tol_observation_m or tourne > cfg.tol_observation_deg:
            self.stats["cartons_bouges"] = self.stats.get("cartons_bouges", 0) + 1
            self._journal(f"carton {v.couleur} a BOUGÉ de {100 * bouge:.1f} cm et {tourne:.1f}° "
                          f"depuis la dernière estimation ({v.vu or 'détection'}) — prise sur "
                          f"la nouvelle : {vu}")
        cible, raison = self._cible_prise(centre)
        if cible is None:
            self._journal(f"carton {v.couleur} revu : {raison}")
            self._etat_accostage = "ECHEC"
            self.stats["accostages_ratees"] += 1
            return
        decalage = math.hypot(cible.x - self._insertion.x, cible.y - self._insertion.y)
        self._colis_vise, self._insertion = obs, cible
        if decalage > cfg.tol_realignement_m:
            if self._realignements >= cfg.realignements_max:
                self._journal(f"carton {v.couleur} : cible déplacée de {100 * decalage:.1f} cm "
                              f"après {self._realignements} réalignement(s) — nouvel accostage")
                self._etat_accostage = "ECHEC"
                self.stats["accostages_ratees"] += 1
                return
            # Se replacer devant le carton revu (bras ouverts, lentement),
            # puis le regarder de nouveau.
            self._realignements += 1
            c, s = math.cos(self.cap_prise), math.sin(self.cap_prise)
            ri = cfg.recul_insertion
            self._cible = Pose2D(cible.x - ri * c, cible.y - ri * s, self.cap_prise)
            self._phase, self._vmax = "ACCOSTAGE", cfg.v_insertion
            self._t_accostage = self._horloge()
            p = self._pose()
            self._d0 = math.hypot(self._cible.x - p.x, self._cible.y - p.y)
            self.stats["realignements"] = self.stats.get("realignements", 0) + 1
            self._journal(f"carton {v.couleur} : cible déplacée de {100 * decalage:.1f} cm — "
                          f"réalignement ({self._realignements}/{cfg.realignements_max}), "
                          f"puis nouvelle observation")
            return
        if bouge <= cfg.tol_observation_m and tourne <= cfg.tol_observation_deg:
            self._journal(f"carton {v.couleur} revu après l'arrêt : {vu}, à {100 * bouge:.1f} cm "
                          f"et {tourne:.1f}° de la dernière estimation")
        self._observe = obs
        self._lancer_insertion()

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
        # Le carton, lui, prend la boîte revue (fusionnée), centre et lacet ;
        # sans boîte, la seule correction latérale.
        self._colis_vise = (Colis(v.couleur, n.position, n.confiance, n.lacet, n.mal_vu, n.vu)
                            if n.vu else Colis(v.couleur, (v.position[0] + dx, v.position[1] + dy,
                                                           v.position[2]), v.confiance))
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
        if self._observation_requise:
            # Après un accostage de prise : seul compte le carton revu après
            # l'arrêt (voir ConfigFacade, observation), jamais une estimation
            # plus ancienne.
            if self._observe is None or self._observe.couleur != colis.couleur:
                self.stats["prises_sans_observation"] = \
                    self.stats.get("prises_sans_observation", 0) + 1
                return False, (f"prise refusée — carton {colis.couleur} pas revu après l'arrêt "
                               f"du robot (il faut {self.cfg.images_observation} images "
                               f"cohérentes)")
            colis = self._observe
        v = self._colis_vise
        if not self._observation_requise and v is not None and v.couleur == colis.couleur \
                and math.dist(v.position[:2], colis.position[:2]) < self.cfg.correction_visee_max_m:
            colis = v                    # position corrigée par la visée
        if colis.vu:
            self._journal(f"carton {colis.couleur} vu : {colis.vu}")
        if colis.mal_vu:
            self.stats["cartons_mal_vus"] = self.stats.get("cartons_mal_vus", 0) + 1
            return False, f"prise refusée — carton mal vu : {colis.mal_vu}"
        r = self.manip.prendre(Objet(colis.position, self.cfg.largeur,
                                     self.cfg.profondeur, self.cfg.hauteur, lacet=colis.lacet))
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

    def raison_manipulation(self) -> str:
        """Pourquoi la dernière prise ou dépose a échoué ("" sinon)."""
        return getattr(self.manip, "raison", "") or ""

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
        self._obs, self._observe, self._observation_requise = None, None, False
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
        self._oublier()                    # nouvel essai : tout carton est à revoir
