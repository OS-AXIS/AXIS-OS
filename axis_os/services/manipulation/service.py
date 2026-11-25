"""
AXIS-OS — Service de manipulation : prendre et poser un objet.

Sixième service système d'AXIS-OS (voir docs/architecture_services.md).
Une app (une application installée sur le robot) dit « prends ce carton »,
puis « pose-le », comme elle dit « va au poste B » au service de
navigation. Elle n'a à connaître ni les angles des bras, ni les pièges de
la prise : le service s'en charge.

Ce service est le seul à commander les bras. Pourquoi : deux programmes
qui écriraient en même temps dans les mêmes articulations produiraient des
gestes contradictoires. C'est le premier pas de l'arbitrage des
actionneurs (la répartition des moteurs entre services) : la locomotion
possède les jambes et la taille, la manipulation possède les bras. Une app
n'écrit jamais directement un angle d'épaule.

Les compétences (les gestes proposés) reprennent ce que les essais en
simulation ont validé : la prise d'un carton posé sur une table, puis
« va prendre le carton » sans lui dire où il est
(docs/validation_port_carton.md, §2 et §5) :

  prendre : ouvrir les bras autour de l'objet ; SERRER, avec une consigne
            qui vise au-delà du contact, pour que la raideur des
            articulations, qui tirent vers cette position inatteignable,
            produise la force de serrage ; SOULEVER ; VÉRIFIER que l'objet
            a suivi.
  poser   : redescendre l'objet sur la surface ; OUVRIR les bras ; le
            relâcher.

Trois règles, chacune tirée d'un échec constaté :

  1. On ne serre que si l'objet est entre les mains (« règle de Kenzo »,
     du prénom de l'auteur du projet, établie lors de l'essai où le robot
     devait trouver le carton seul). Les mains sont connues par les
     articulations, l'objet par la perception (la caméra). Sinon, refus
     motivé : c'est à l'app de placer le robot.

  2. On vérifie la prise avant de la déclarer tenue. Un robot qui part les
     mains vides en croyant porter un carton est pire qu'un robot qui
     s'arrête. En simulation, la vérification lit la hauteur de l'objet ;
     sur le vrai robot, ce sera l'effort mesuré aux épaules.

  3. L'objet à prendre, posé sur une table, arrêterait le robot : le
     gardien de proximité (axis_os/services/locomotion/proximite.py), qui
     arrête le robot devant tout obstacle que rien n'explique, le
     prendrait pour un obstacle. Le service lui demande donc une
     exemption, une zone bornée autour de l'objet et limitée dans le
     temps, où ce qu'il voit est ignoré. Il la lève dès la fin de
     l'opération, réussie ou non.

Le temps est celui du robot (horloge injectée), comme partout ailleurs
dans AXIS-OS : la simulation et les tests fournissent leur propre horloge.

Prise mesurée. Quand le robot donne ses mesures (`mesures` : cinématique
des bras lue dans l'URDF, angles, efforts des articulations, hauteur du
bassin), le serrage ne va plus vers une posture fixe, la même pour tout
objet. Pourquoi : constaté sur GPU (banc du 2026-10-28, 2 prises réussies
sur 5), les mains effleuraient le carton sans le serrer, 6 cm devant son
centre, ou se refermaient dans le vide devant un carton hors de portée.
La prise suit alors ces étapes (voir prise.py pour les réglages) :

  PLACEMENT  vérifier que l'objet est à portée (sinon refus OUT_OF_REACH),
             puis mains au milieu de ses deux faces, en profondeur et en
             hauteur, écartées de `jeu_ouverture_m` ; corrigées si elles
             sont mal placées ;
  FERMETURE  chaque bras se referme jusqu'au CONTACT, détecté par l'effort
             ou par l'arrêt du rapprochement ; plus rien entre les mains :
             échec, jamais de serrage dans le vide ;
  SERRAGE    serrage supplémentaire jusqu'à l'effort visé, borné sous les
             limites de couple du garde-fou ;
  LEVAGE     quelques centimètres, puis vérifier que l'objet a suivi ;
             sinon reposer, desserrer et recommencer une fois (REPOSE) ;
  RAMENER    l'objet contre le buste, pour marcher ; son inclinaison est
             surveillée pendant le transport.

Effort absent, non fini ou périmé : on ne serre pas, comme pour la
température (axis_os/services/health.py). Sans `mesures`, le service garde
le serrage en boucle ouverte d'avant (tests unitaires, robots qui ne
mesurent rien) ; le robot simulé et Isaac Sim donnent leurs mesures.
"""
from __future__ import annotations

import math
import threading
import time
from collections import deque
from datetime import datetime, timezone
from dataclasses import replace
from dataclasses import dataclass
from enum import Enum, auto
from typing import Callable, Dict, Optional, Tuple, Union

import numpy as np

from axis_os.hal.core.types import HALStatus
from axis_os.services.base import IService, ServiceHealth, ServiceResult, sous_verrou
from axis_os.store.sdk.manifest import Capability

from .prise import (
    CalibrationCouples, MesuresBras, ReglagesPrise, cibles_mains, distance_ideale,
    ecart_ligne_cg, efforts_valides, gain_coude, garde_plateau, hauteur_prise, main_retenue,
    poids_pese,
    positions_valides, resoudre, signal_contact, tolerances_cg,
)


@dataclass(frozen=True)
class Postures:
    """Angles (en radians) et durées (en secondes) des gestes de bras.

    Les angles sont BRUTS, dans la convention du robot Unitree G1 : ils ont
    été calculés par cinématique directe (le calcul de la position des
    mains à partir des angles des articulations) et validés en simulation
    lors des essais de prise d'un carton. Ils ne sont pas déduits des
    étiquettes anatomiques (« flexion »…) de la table de conventions du
    robot : deux d'entre elles, celles du tangage d'épaule et du coude, se
    sont révélées inversées (docs/validation_port_carton.md, §3).

    Vocabulaire : le tangage d'épaule lève ou baisse le bras vers l'avant
    (plus il est négatif, plus les mains sont hautes) ; le roulis d'épaule
    écarte le bras du corps (positif : ouvert, négatif : serré).
    """
    tangage_port: float = -0.30        # mains devant le bassin, à ~0.90 m
    tangage_leve: float = -0.50        # soulevé : les mains montent de ~6 cm
    coude: float = 0.0                 # angle droit : c'est le zéro du G1
    roulis_ouvert: float = 0.20        # 33 cm entre les mains
    roulis_serre: float = -0.40        # vise au-delà du contact (voir plus haut)
    duree_serrage_s: float = 0.5
    duree_levage_s: float = 0.8
    # Confirmation de la prise. Pourquoi ce délai : constaté en simulation
    # complète sur carte graphique (GPU, ici une RTX 5090), un carton bien
    # serré était déclaré « non confirmé » à l'instant exact où la consigne
    # de levage finissait, et la prise était refaite en boucle. Le bras
    # chargé rejoint sa consigne avec retard (voir aussi la montée, plus
    # bas). La prise est donc confirmée dès que l'objet a suivi, en
    # laissant jusqu'à ce délai après la fin de la consigne de levage.
    attente_confirmation_s: float = 1.0
    duree_pose_s: float = 0.8
    duree_ouverture_s: float = 0.5
    # Pré-saisie : bras ouverts plus large avant de glisser les mains autour
    # de l'objet, comme le ferait un humain. Pourquoi : sans elle, le robot
    # avançait vers un carton de 24 cm avec les mains à 33 cm l'une de
    # l'autre, et le heurtait (constaté sur GPU, lors de la démonstration
    # de tri de colis : docs/demo_tri_usine.md).
    roulis_presaisie: float = 0.32
    duree_presaisie_s: float = 0.6
    # Pré-saisie d'un carton donné (presaisir(objet)) : ouverte juste assez
    # pour lui, sa largeur vue de face (lacet compris) plus jeu_ouverture_m de
    # chaque côté, sans jamais moins que roulis_presaisie ni plus que
    # roulis_presaisie_max (0,80 rad : 42 cm libres entre les paumes), le
    # tangage d'épaule abaissé d'autant que l'ouverture lève les mains : elles
    # restent à la hauteur de la pré-saisie fixe (sinon au-dessus d'un carton
    # bas : prise refusée, mains plus hautes que lui). Pourquoi : roulis_presaisie, réglé sur le
    # carton de la scène T1 (24 cm), n'en laisse que 27 ; la campagne de
    # cartons variés (axis_os/testing/cartons.py) a vu des bras ouverts
    # heurter à l'insertion tout carton plus large vu de face (plafonds de
    # couple dépassés de 1 à 2 N·m) ; au-delà de l'ouverture maximale, le
    # carton est refusé d'avance (distance_de_prise : trop large).
    roulis_presaisie_max: float = 0.80
    # Franchissement : avant d'avancer au-dessus d'un plateau, le fond de
    # l'objet porté doit le dépasser de `marge_franchissement`. Pourquoi :
    # constaté sur GPU, le carton, qui glisse dans les mains pendant le
    # trajet (jusqu'à 9 cm mesurés lors de l'essai de prise sur table),
    # butait contre le bord de la table. La marge est large parce que la
    # hauteur du fond n'est pas mesurée : elle est estimée à partir de la
    # position des mains.
    marge_franchissement: float = 0.10
    tangage_haut: float = -0.85        # bras au plus haut : jamais au-delà
    vitesse_montee: float = 0.30       # rad/s
    # Stabilisation de la montée. Pourquoi : constaté sur GPU, l'échec
    # « plateau trop haut » tombait à l'instant où la consigne atteignait
    # sa limite, alors que les mains étaient encore en train de monter. Le
    # bras chargé suit la consigne avec retard (réglages doux des moteurs,
    # poids du carton). On laisse donc le bras la rejoindre avant de
    # conclure.
    stabilisation_montee_s: float = 1.2
    # Après stabilisation, une marge réduite suffit : si le carton touche
    # malgré tout, la détection de blocage de l'app de tri de colis
    # (axis_os/apps/tri_colis/facade.py, qui repère un robot qui pousse
    # sans avancer) reste le filet de sécurité.
    marge_franchissement_min: float = 0.05
    # Dépose sur un plateau : descente en boucle fermée, c'est-à-dire
    # corrigée en continu d'après la hauteur mesurée des mains
    # (proprioception : ce que le robot sait de ses propres articulations),
    # jusqu'à ce que le fond de l'objet soit à `jeu_depose` du plateau.
    # Jamais au-delà de `tangage_bas`.
    tangage_bas: float = 0.10
    vitesse_descente: float = 0.25     # rad/s
    jeu_depose: float = 0.01           # m : on ouvre juste au-dessus


def _rotation_vecteur(m: np.ndarray) -> np.ndarray:
    """Vecteur rotation (axe × angle, radians) de la rotation `m` (3 × 3)."""
    a = math.acos(max(-1.0, min(1.0, (float(np.trace(m)) - 1.0) / 2.0)))
    v = np.array([m[2, 1] - m[1, 2], m[0, 2] - m[2, 0], m[1, 0] - m[0, 1]])
    return v / 2.0 if a < 1e-6 else v * (a / (2.0 * math.sin(a)))


def _matrice_rotation(v) -> np.ndarray:
    """Rotation (3 × 3) du vecteur rotation `v` (axe × angle, radians)."""
    v = np.asarray(v, dtype=float)
    a = float(np.linalg.norm(v))
    if a < 1e-12:
        return np.eye(3)
    k = v / a
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(a) * kx + (1.0 - math.cos(a)) * kx @ kx


def posture_bras(roulis: float, tangage: float, coude: float) -> Dict[str, float]:
    """Cibles des six articulations des bras, pour une posture symétrique.

    Le roulis du bras droit est l'opposé de celui du gauche : la même
    valeur ouvre (ou serre) les deux bras en miroir. Le tangage et le coude
    sont les mêmes des deux côtés.
    """
    return {"left_shoulder_pitch_joint": tangage,
            "right_shoulder_pitch_joint": tangage,
            "left_shoulder_roll_joint": roulis,
            "right_shoulder_roll_joint": -roulis,
            "left_elbow_joint": coude, "right_elbow_joint": coude}


@dataclass(frozen=True)
class Objet:
    """Ce que la perception sait de l'objet : son centre et ses dimensions,
    en mètres, dans le repère monde (les coordonnées fixes du site, pas
    celles du robot). La largeur se mesure entre les mains, la profondeur
    vers l'avant du robot. `lacet` : rotation de l'objet autour de la
    verticale (rad, repère monde), connue ou nulle. `bord_m` : distance de
    son centre au bord du plateau qui le porte, moins sa demi-profondeur (de
    face : de sa face avant au bord ; 0 : au bord, comme sur les tables de
    tri) ; le bord n'est jamais pris à moins de ReglagesPrise.bord_min_m du
    bassin. `lacet_bord` : direction (rad, repère monde) de la normale de ce
    bord, vers la table ; None : face au robot, comme avant (le robot aborde
    la table de face). Connue, elle permet d'aborder l'objet dans l'axe de
    ses faces, le bord vu en biais (distance_de_prise, cap)."""
    centre: Tuple[float, float, float]
    largeur: float
    profondeur: float
    hauteur: float
    lacet: float = 0.0
    bord_m: float = 0.0
    lacet_bord: Optional[float] = None


class Etat(Enum):
    LIBRE = auto()        # bras en posture de port, rien en main
    PRESAISIE = auto()    # ouverture large en cours, avant l'approche finale
    PRET = auto()         # bras grands ouverts, prêts à entourer l'objet
    SERRAGE = auto()      # les bras se referment sur l'objet
    LEVAGE = auto()       # l'objet est soulevé, puis la prise vérifiée
    TENU = auto()         # prise confirmée : l'objet est porté
    POSE = auto()         # redescente à hauteur de port, sans plateau
    DESCENTE = auto()     # vers un plateau, en boucle fermée
    MONTEE = auto()       # au-dessus d'un plateau, avant d'avancer vers lui
    OUVERTURE = auto()    # les bras s'ouvrent pour relâcher l'objet
    LACHER = auto()       # après un échec : ouverture large PROGRESSIVE
    ECHEC = auto()        # prise non confirmée : appeler reinitialiser()
    # Prise mesurée (voir l'en-tête du module).
    PLACEMENT = auto()    # mains au milieu des faces, écartées
    FERMETURE = auto()    # chaque bras jusqu'au contact
    REPOSE = auto()       # prise non confirmée : reposer, desserrer, recommencer
    RAMENER = auto()      # l'objet vers le buste, pour marcher


class ManipulationService(IService):
    """Seul écrivain des articulations des bras. Enchaîne les gestes de
    prise, de port et de pose sous forme d'une machine à états, que chaque
    appel de tick() fait avancer d'un pas."""

    def __init__(self, bras: Dict[str, float],
                 mains: Callable[[], Tuple[np.ndarray, np.ndarray]],
                 vers_robot: Callable[[np.ndarray], np.ndarray],
                 verifier_prise: Callable[[], bool],
                 gardien=None, horloge: Callable[[], float] = time.monotonic,
                 postures: Postures = Postures(),
                 arret_securite: Optional[Callable[[], bool]] = None,
                 mesures: Optional[MesuresBras] = None,
                 reglages: ReglagesPrise = ReglagesPrise(),
                 journal: Optional[Callable[[str], None]] = None,
                 posture: Optional[Callable[..., ServiceResult]] = None):
        """
        bras            : le dictionnaire des cibles de bras de l'adaptateur
                          de locomotion, qui les transmet au robot. Ce
                          service est son SEUL écrivain.
        mains           : () -> positions, dans le repère monde, des mains
                          gauche et droite (proprioception : calculées à
                          partir des angles des articulations).
        vers_robot      : convertit un point du repère monde dans le repère
                          du robot (x devant, y à gauche).
        verifier_prise  : () -> l'objet a-t-il suivi les mains ?
        gardien         : GardienProximite (le gardien de proximité), pour
                          l'exemption autour de l'objet. Optionnel.
        horloge         : () -> temps du robot, en secondes.
        postures        : angles et durées des gestes (voir Postures).
        arret_securite  : () -> un arrêt de sécurité est-il en cours ? Par
                          défaut, lu sur le gardien. Pendant l'arrêt, les
                          bras sont figés en place. Pourquoi : un humain
                          assez proche pour arrêter la marche l'est assez
                          pour qu'un bras en mouvement le heurte. Le geste
                          reprend ensuite là où il s'était figé. Les bras
                          gardent leur prise : figer n'est pas lâcher.
        mesures         : ce que le robot mesure de ses bras (MesuresBras) :
                          avec elles, la prise est mesurée (voir l'en-tête
                          du module) ; sans elles, serrage en boucle ouverte.
        reglages        : réglages de la prise mesurée (ReglagesPrise).
        journal         : (message) -> None ; reçoit, à chaque échec ou
                          refus de prise mesurée, la position des mains
                          (mesurées et calculées) et celle de l'objet visé.
        posture         : la consigne de posture du service de marche
                          (LocomotionService.posture : bornée, équilibre
                          vérifié, gardée par le gardien de proximité), pour
                          se pencher vers un objet trop loin (plan_penche,
                          changer_tangage). None : jamais penché.
        """
        self._bras = bras
        self._mains = mains
        self._posture = posture
        # Tangage du torse visé pendant que l'objet est tenu (changer_tangage) ;
        # None : aucun changement en cours.
        self._tangage_vise: Optional[float] = None
        self._vers_robot = vers_robot
        self._verifier = verifier_prise
        self._gardien = gardien
        self._horloge = horloge
        self.p = postures
        self._journal = journal
        # Raison du refus de toute prise mesurée (calibration des mains
        # ratée, voir prise.CalibrationMains) ; vide : prise permise.
        self.calibration_refus = ""
        self.raideur_active = False        # gains des poignets raidis (MesuresBras.raideur)
        # Articulations libérées pour la prise (lacet d'épaule, poignets) et
        # leur consigne d'avant : rendue en fin de prise, en douceur (LACHER).
        self._libres0: Dict[str, float] = {}
        self.etat = Etat.LIBRE
        self.objet: Optional[Objet] = None
        self.raison = ""
        self._t0 = 0.0
        self._demarre = False
        self._verrou = threading.RLock()
        if arret_securite is None and gardien is not None \
                and hasattr(type(gardien), "arrete"):
            arret_securite = lambda: gardien.arrete
        self._arret_securite = arret_securite
        self._surface: Optional[float] = None
        # Hauteur du plateau que la dernière montée a permis de franchir
        # (None si aucune) : voir peut_franchir().
        self._franchi: Optional[float] = None
        self._t_limite: Optional[float] = None
        self._roulis_depart_serrage = postures.roulis_ouvert
        self._tangage_courant = postures.tangage_port
        self._roulis_courant = postures.roulis_ouvert
        # Pré-saisie en cours (presaisir) : roulis et tangage d'épaule.
        self._roulis_presaisie, self._tangage_presaisie = (postures.roulis_presaisie,
                                                           postures.tangage_port)
        self._lacher_de = self._lacher_vers = (postures.roulis_ouvert, postures.tangage_port)
        self._tangage_depart = postures.tangage_leve
        self._tangage_ouverture = postures.tangage_port
        self._dernier_tick: Optional[float] = None
        self.stats = {"prises": 0, "prises_ratees": 0, "refus": 0, "poses": 0,
                      "gels_securite": 0, "deposes_plateau": 0,
                      "deposes_impossibles": 0}
        # Consigne complète des bras (les six articulations), et la posture
        # vers laquelle va le geste en cours (interpolée articulation par
        # articulation, depuis `_q_de`).
        self._q: Dict[str, float] = posture_bras(postures.roulis_ouvert, postures.tangage_port,
                                                 postures.coude)
        self._q_de: Dict[str, float] = dict(self._q)
        self._q_vers: Dict[str, float] = dict(self._q)
        self.mesures = mesures
        self.r = reglages
        self._init_mesure()
        # Biais des couples estimés (N·m, par articulation), trouvés au
        # démarrage (voir calibration des couples, plus bas) ; retirés du
        # couple mesuré dans le calcul de la force à la main seulement.
        self.calibration_couples = CalibrationCouples()
        self.biais_couples: Dict[str, float] = {}
        # Gains des couples estimés des coudes (estimé = gain × vrai), de la
        # calibration de charge du profil du robot (appliquer_calibration_charge,
        # calibrer_charge) ; vides sans elle : la force à la main se calcule
        # comme avant. Comme les biais, ils ne corrigent que la force à la main ;
        # les plafonds de couple restent jugés sur le couple mesuré.
        self.gains_couples: Dict[str, float] = {}
        self.calibration_charge: Optional[dict] = None
        self._calibration_en_cours: Optional[dict] = None
        self._calibration_finie = False
        # Les couples estimés suivent-ils la pesanteur ? (calibration des
        # couples ; None : pas jugé). Faux : la force à la main ne décide plus
        # ni du contact, ni du serrage, ni de la confirmation ; elle reste au
        # diagnostic, et les plafonds de couple restent jugés sur le couple.
        self.couples_fiables: Optional[bool] = None
        self._t_raideur = -math.inf        # dernier changement de gains
        self._hors_consigne0: Optional[Dict[str, float]] = {}
        self._attente_pose: Optional[dict] = None

    # ---- IService : l'interface commune des services système -------------

    @property
    def service_id(self) -> str:
        return "manipulation"

    @property
    def capability(self) -> Capability:
        # Prise à deux mains : sans elle, le service n'est pas démarré (voir
        # IService.capability).
        return Capability.GRASP_DUAL

    def start(self) -> HALStatus:
        # Le service part d'une posture connue : bras ouverts, en posture de
        # port.
        self._demarre = True
        self._ecrire(self.p.roulis_ouvert, self.p.tangage_port)
        return HALStatus.OK

    def stop(self) -> None:
        # Ne laisser aucune exemption derrière soi : le gardien doit de
        # nouveau surveiller partout.
        self._lever_exemption()
        self._demarre = False

    def health(self) -> ServiceHealth:
        return ServiceHealth(running=self._demarre,
                             healthy=self.etat is not Etat.ECHEC,
                             detail=self.raison or self.etat.name)

    # ---- outils internes -------------------------------------------------

    def _ecrire(self, roulis: float, tangage: float) -> None:
        """Écrit la posture des bras, et la retient comme posture courante."""
        self._tangage_courant, self._roulis_courant = tangage, roulis
        self._q = posture_bras(roulis, tangage, self.p.coude)
        self._bras.update(self._q)

    def _ecrire_q(self, q: Dict[str, float]) -> None:
        """Écrit une consigne articulation par articulation (prise mesurée) ;
        les articulations libérées pour la prise que `q` ne donne pas gardent
        leur consigne."""
        garde = {j: self._q[j] for j in self._libres0 if j in self._q and j not in q}
        self._q = {**garde, **{j: float(v) for j, v in q.items()}}
        self._tangage_courant = self._q.get("left_shoulder_pitch_joint", self._tangage_courant)
        self._roulis_courant = self._q.get("left_shoulder_roll_joint", self._roulis_courant)
        self._bras.update(self._q)

    def _interp_q(self, duree: float) -> Tuple[Dict[str, float], bool]:
        """Consigne qui va linéairement de `_q_de` à `_q_vers` en `duree`."""
        f = min(1.0, (self._horloge() - self._t0) / duree) if duree > 0 else 1.0
        return ({j: self._q_de.get(j, self._q_vers[j])
                 + f * (self._q_vers[j] - self._q_de.get(j, self._q_vers[j]))
                 for j in self._q_vers}, f >= 1.0)

    def _interp_c(self, duree: float) -> Tuple[Dict[str, float], bool]:
        """Objet tenu : le milieu des mains va en ligne droite de `_c_de` à
        `_c_vers` en `duree`, le serrage gardé à chaque pas (une
        interpolation des angles le relâcherait en chemin)."""
        f = min(1.0, (self._horloge() - self._t0) / duree) if duree > 0 else 1.0
        # Progressif : départ et arrivée en douceur (3f² - 2f³), sans à-coup
        # sur l'objet tenu.
        lisse = f * f * (3.0 - 2.0 * f)
        c = self._c_de + lisse * (self._c_vers - self._c_de)
        q = self._q_mains(c, self._ecart_mains)
        if q is None and self._rigide is not None:
            # Corps rigide : jamais d'interpolation des angles, qui fait tourner
            # les mains d'un coup (30° et 7 cm de chute sur le robot simulé). Le
            # geste s'arrête au dernier point tenu et finit à l'heure, sans bouger.
            tenu = self._c_actuel if self._c_actuel is not None else self._c_de
            self._c_de = self._c_vers = self._c_actuel = np.array(tenu, dtype=float)
            return dict(self._q), f >= 1.0
        if q is None:
            return self._interp_q(duree)
        self._c_actuel = c
        return q, f >= 1.0

    def _interp(self, a: float, b: float, duree: float) -> Tuple[float, bool]:
        """Valeur qui va linéairement de a à b en `duree` secondes, depuis
        le début du geste en cours (self._t0), et vrai une fois `duree`
        écoulée."""
        f = min(1.0, (self._horloge() - self._t0) / duree)
        return a + f * (b - a), f >= 1.0

    def _aller(self, etat: Etat) -> None:
        """Passe dans `etat`, et note l'instant de départ du geste (au
        retour, les mains mesurées : la référence de _rotation_estimee)."""
        self.etat, self._t0 = etat, self._horloge()
        if etat is Etat.RAMENER:
            angles = self._angles_mesures()
            self._poses_mesurees0 = None if angles is None else self._poses(angles)
            self.rotation_carton = None

    def _lever_exemption(self) -> None:
        if self._gardien is not None:
            self._gardien.lever_exemption()

    def entre_les_mains(self, objet: Objet) -> Tuple[bool, str]:
        """Règle de Kenzo (voir l'en-tête du module) : l'objet est-il entre
        les mains ?

        Sur le côté : son centre doit être entre les deux mains, avec la
        place de s'y loger et 1 cm de marge. En profondeur : les mains
        doivent être entre sa face avant et sa face arrière, à 2 cm près. En
        hauteur : les mains doivent être entre son dessous et son dessus, à
        2 cm près. Pourquoi cette dernière règle : prendre un objet fait
        ignorer au gardien de proximité un volume autour de lui ; une revue
        adversariale a obtenu cette exemption pour un « objet » posé bien
        plus bas que les mains, là où le gardien ne voyait alors plus une
        personne accroupie.
        Les mains : leurs points de contact (ReglagesPrise.contact_main_m),
        là où elles touchent l'objet ; angles illisibles, leurs repères (plus
        strict : le repère est en deçà du contact).
        Rend (True, "") ou (False, raison du refus).
        """
        g, d = self._mains()
        og, od = self._decalages_contact()
        g, d = np.asarray(g, dtype=float) + np.array([0.0, 0.0, og[2]]), \
            np.asarray(d, dtype=float) + np.array([0.0, 0.0, od[2]])
        gr, dr = self._vers_robot(g) + og * [1, 1, 0], self._vers_robot(d) + od * [1, 1, 0]
        cr = self._vers_robot(np.asarray(objet.centre, dtype=float))
        milieu = (gr[1] + dr[1]) / 2.0
        jeu = abs(gr[1] - dr[1]) / 2.0 - objet.largeur / 2.0
        mains_x = (gr[0] + dr[0]) / 2.0
        if jeu <= 0.01:
            return False, (f"mains trop proches ({abs(gr[1] - dr[1]):.2f} m) "
                           f"pour un objet de {objet.largeur:.2f} m")
        if abs(cr[1] - milieu) >= jeu - 0.01:
            return False, (f"objet décalé de {cr[1] - milieu:+.2f} m sur le "
                           f"côté (marge {jeu - 0.01:.2f} m)")
        avant = cr[0] - objet.profondeur / 2.0 + 0.02
        arriere = cr[0] + objet.profondeur / 2.0 - 0.02
        if not (avant < mains_x < arriere):
            return False, (f"mains à {mains_x:.2f} m, objet entre "
                           f"{avant:.2f} et {arriere:.2f} m devant le robot")
        mains_z = (float(g[2]) + float(d[2])) / 2.0
        dessous = float(objet.centre[2]) - objet.hauteur / 2.0 - 0.02
        dessus = float(objet.centre[2]) + objet.hauteur / 2.0 + 0.02
        if not (dessous < mains_z < dessus):
            return False, (f"mains à {mains_z:.2f} m de haut, objet entre "
                           f"{dessous + 0.02:.2f} et {dessus - 0.02:.2f} m")
        return True, ""

    def _decalages_contact(self) -> Tuple[np.ndarray, np.ndarray]:
        """Du repère de chaque main à son point de contact (m, repère du
        bassin : x, y du robot, z vers le haut), sur les angles mesurés ;
        zéro sans mesures ou angles illisibles."""
        zero = (np.zeros(3), np.zeros(3))
        if self.mesures is None:
            return zero
        c, rep = self._mains_modele(), self._mains_modele(reperes=True)
        if c is None or rep is None:
            return zero
        return c[0] - rep[0], c[1] - rep[1]

    # ---- prise mesurée (voir l'en-tête du module et prise.py) --------------

    # Roulis d'épaule de chaque bras, et sens de la fermeture (le bras
    # gauche se referme quand son roulis diminue, le droit quand il croît).
    _ROULIS = {"left": "left_shoulder_roll_joint", "right": "right_shoulder_roll_joint"}
    _FERME = {"left": -1.0, "right": 1.0}
    # Toutes les articulations d'un bras (épaule 3, coude, poignet 3) : la
    # force à la main s'estime sur toutes celles dont le robot mesure le couple.
    _BRAS_COTE = {c: tuple(f"{c}_{a}" for a in (
        "shoulder_pitch_joint", "shoulder_roll_joint", "shoulder_yaw_joint", "elbow_joint",
        "wrist_roll_joint", "wrist_pitch_joint", "wrist_yaw_joint")) for c in ("left", "right")}

    def _init_mesure(self) -> None:
        """Remet à zéro l'état de la prise mesurée."""
        self._plan: Optional[dict] = None
        self._corrections = 0
        self._essai = 0
        self._contact: Dict[str, Optional[float]] = {"left": None, "right": None}
        self._compte_contact = {"left": 0, "right": 0}
        self._tau0: Dict[str, float] = {}
        self._delta = {"left": 0.0, "right": 0.0}
        self._prec_serrage: Dict[str, Tuple[float, float]] = {}
        self._t_phase: Optional[float] = None
        self._sous = ""
        self._q_serre: Optional[Dict[str, float]] = None
        self._c_serre: Optional[np.ndarray] = None
        # À la prise : hauteur des points de contact mesurés au-dessus du centre
        # perçu de l'objet (m) ; _fond_objet() en part tant qu'il est tenu.
        self._dz_prise: Optional[float] = None
        self._delta_serre = {"left": 0.0, "right": 0.0}
        self._c_de = self._c_vers = np.zeros(3)
        self._q_tenu: Optional[Dict[str, float]] = None
        self._centre_porte: Optional[np.ndarray] = None
        self._centre_transport: Optional[np.ndarray] = None
        self._ecart_mains: Optional[float] = None
        self.penche = False            # au-delà du seuil d'inclinaison
        self.a_reposer = False         # … et encore après la correction
        self.inclinaison: Optional[float] = None
        self.inclinaison_max = 0.0
        self._corrections_inclinaison = 0
        self.effort_max_mesure = 0.0
        self._fixes: Dict[str, float] = {}
        self._visee = None                 # visée compensée des mains (placement)
        self._lam = {"left": 0.0, "right": 0.0}        # avancement de la fermeture, 0 à 1
        self._q_ferm0: Dict[str, float] = {}
        self._cible_ferm: Dict[str, Dict[str, float]] = {}
        self._attendue: Dict[str, float] = {}
        # Contact estimé par l'OS, par main : {"t": instant, "signal": …}.
        self.contacts_estimes: Dict[str, Optional[Dict]] = {"left": None, "right": None}
        # Force estimée à chaque main (voir _mesurer_forces) : vecteur (N,
        # repère du bassin), serrage filtré le long de la normale (N, vers
        # l'objet), le plus fort vu, bruit au repos et seuil de contact.
        self.forces: Dict[str, Optional[np.ndarray]] = {"left": None, "right": None}
        self.force_serrage: Dict[str, float] = {"left": 0.0, "right": 0.0}
        self.force_max_mesuree = 0.0
        self.bruit_force: Dict[str, float] = {}
        # Force absolue (N) à chaque main juste avant de fermer (voir _mesurer_bruit).
        self.force_avant_fermeture: Dict[str, float] = {}
        self._seuil_contact = self.r.force_contact_n
        self._hist_force: Dict[str, deque] = {c: deque(maxlen=max(1, self.r.filtre_force))
                                              for c in ("left", "right")}
        self._g0: Dict[str, float] = {}
        self._echantillons: list = []
        self._delta_max = {"left": 0.0, "right": 0.0}
        self._raideur_vue: Dict[str, float] = {}
        self._hausse_roulis = {"left": 0.0, "right": 0.0}
        self._ecart_face0: Dict[str, float] = {}
        self._plafond: Dict[str, str] = {}       # main arrêtée par la limite de couple de…
        self.recalage: Optional[dict] = None     # recalage au toucher (voir _recaler)
        # Ligne des points de contact et centre de gravité : (profondeur,
        # hauteur) en m (voir _ecart_cg) ; charge de l'objet tenu, d'après les
        # couples : (poids en N, couple de basculement autour de la ligne des
        # mains en N·m), filtrée ; rotation vue par la caméra (degrés) ;
        # appui contre le torse (retard des mains, m) ; poids de référence au
        # début du port, et depuis quand il manque.
        self.ligne_cg: Optional[Tuple[float, float]] = None
        self.charge: Optional[Tuple[float, float]] = None
        self._hist_charge: deque = deque(maxlen=max(1, self.r.filtre_force * 3))
        # Somme des forces normales des deux mains au port, et ses instants :
        # la dernière foulée (fenetre_marge_s).
        self._hist_normales: deque = deque()
        self._t_normales: deque = deque()
        # Poids lu à chaque pas du port (instant, N) : sa médiane sur la foulée
        # compte dans la marge s'il passe la pesée (le plus prudent des deux).
        self._poids_port: deque = deque()
        # Instants des écartements des mains au port, et l'écart au début de
        # celui en cours (un carton qui tourne entre les paumes).
        self._ecartements: deque = deque()
        self._ecart_debut: Optional[float] = None
        self.rotation_vue: Optional[float] = None
        self.appui_torse: Optional[float] = None
        self._poids_ref: Optional[float] = None
        # Le niveau du poids lu au début du port, dans sa posture : les
        # lectures (instant, N) de sa première foulée, puis leur moyenne figée.
        self._niveau_port: list = []
        self._niveau_port_fige: Optional[Tuple[float, float]] = None
        self._niveau_chute: Optional[Tuple[float, float]] = None   # celui d'avant une chute en cours
        self._poids_entree = 0.0                     # le poids lu (0,1 s) en entrant au port (N)
        # Le poids lu base arrêtée, en posture de port : les lectures de la fin
        # du retour (fin_retour_s), puis leur médiane, figée au port.
        self._poids_retour: deque = deque()
        self._poids_statique: Optional[float] = None
        self._bruit_pesee: Optional[float] = None    # écart-type d'une lecture à la pesée (N)
        self._t_sans_poids: Optional[float] = None
        self._raison_repose = ""
        self._c_actuel: Optional[np.ndarray] = None
        self._port_interrompu = False
        # Serrage d'après le poids (ReglagesPrise.securite_serrage) : visé par
        # main (N), depuis quand la prise confirmée attend en haut ce serrage,
        # écart des faces mesurées au début du levage, borne de la consigne ;
        # au port : place des faces au serrage, marge de frottement, début du
        # resserrage ; torse atteignable par le carton ramené.
        self.serrage_vise: Optional[float] = None
        self.serrage_requis: Optional[float] = None
        self._t_attente_serrage: Optional[float] = None
        self._t_serrage_bloque: Optional[float] = None
        # Serrage d'après le poids : le plus loin où chaque main est allée.
        self._record_serrage: Dict[str, float] = {}
        self._ecart_faces0: Optional[float] = None
        self._delta_max_poids: Dict[str, float] = {}
        self._serrage_port_ref: Optional[float] = None
        self._delta_port: Optional[Dict[str, float]] = None
        self.marge_frottement: Optional[float] = None
        self.marge_courte: Optional[float] = None      # sur 0,3 s : le plancher
        self._t_resserrage: Optional[float] = None
        self._resserrage_marge = False     # arrêté pour la marge : reprise au-dessus de securite_reprise
        self._t_bloque: Optional[float] = None   # dernier pas où les bras ne pouvaient serrer plus
        self._torse_possible = True
        # Poids du carton (N), pesé une fois, bras immobiles, en haut du
        # levage, avant de serrer plus : celui du serrage requis et de la
        # marge de frottement au port (le plus grand porté, _poids_ref, ne
        # sert plus qu'au contrôle de glissement du port).
        self.poids_carton: Optional[float] = None
        # Le même, taré : d'après la hausse des couples depuis le zéro d'avant
        # la fermeture, où leur biais s'annule (calibré ou non) ; décide du
        # plafond de masse (masse_max_kg).
        self.poids_tare: Optional[float] = None
        # Pesée (ReglagesPrise.lectures_pesee) : lectures (poids, poussée,
        # taré) prises bras immobiles depuis la confirmation, dernière lecture
        # de la charge pas encore comptée, angles des bras à la lecture
        # d'avant (horodatés), vitesse la plus forte des bras gardée, lectures
        # écartées (bras qui bougeaient).
        self._lectures_pesee: list = []
        # Et, pour la calibration de charge, les couples, les angles et la
        # pesanteur de chacune de ces lectures (calibrer_charge).
        self._releves_pesee: list = []
        self._lecture_charge: Optional[tuple] = None
        self._angles_pesee: Optional[Tuple[float, Dict[str, float]]] = None
        self._vitesse_pesee = 0.0
        self._ecartees_pesee = 0
        # Retour en corps rigide : (centre de départ, pose de chaque main
        # commandée, serrage) au début du geste ; poses des mains MESURÉES
        # au début du geste ; roulis et tangage estimés du carton (degrés) ;
        # resserrage demandé par un carton qui a tourné au retour.
        self._rigide: Optional[tuple] = None
        self._poses_mesurees0: Optional[Dict[str, Tuple[np.ndarray, np.ndarray]]] = None
        self.rotation_carton: Optional[Tuple[float, float]] = None
        self._resserrage_force = False
        # Port (ReglagesPrise, derive_port_m…) : au début du port, les paumes
        # mesurées et la ligne des mains (références de la rotation) ; à chaque
        # entrée en TENU, la posture commandée (graine des bras du port) et la
        # position mesurée commune des mains (référence de leur dérive) ;
        # moyenne glissante de la dérive, correction appliquée (translation et
        # vecteur rotation communs) ; estimations et compteurs du port.
        self._port0: Optional[dict] = None
        self._port_actif = False
        self._hist_derive: deque = deque()
        self._correction_port = (np.zeros(3), np.zeros(3))
        self._correction_visee = (np.zeros(3), np.zeros(3))
        self._t_correction: Optional[float] = None
        self._alarme_rotation = False
        self.rotation_port: Optional[Tuple[float, float, float]] = None
        self.rotation_port_max = 0.0
        self.derive_port: Optional[Tuple[float, float]] = None
        self.resserrages_ce_port = 0
        self.corrections_ce_port = 0

    def _vers_bassin(self, p_monde) -> np.ndarray:
        """Point du repère monde -> repère du bassin (x, y du robot ; z
        au-dessus du bassin)."""
        p = np.asarray(p_monde, dtype=float)
        if self.mesures.vers_bassin is not None:
            return np.asarray(self.mesures.vers_bassin(p), dtype=float)
        r = self._vers_robot(p)
        return np.array([float(r[0]), float(r[1]), float(p[2]) - float(self.mesures.bassin_z())])

    def _cap_bord(self, objet: Objet) -> float:
        """Angle (rad) de la normale du bord de la table avec l'avant du
        robot, d'après sa pose ; 0 si elle n'est pas connue (face au robot)."""
        if objet.lacet_bord is None:
            return 0.0
        c = np.asarray(objet.centre, dtype=float)
        a = self._vers_robot(c)
        b = self._vers_robot(c + np.array([math.cos(objet.lacet_bord),
                                           math.sin(objet.lacet_bord), 0.0]))
        return math.atan2(float(b[1] - a[1]), float(b[0] - a[0]))

    def _lacet_relatif(self, objet: Objet) -> float:
        """Écart (rad) entre les faces de l'objet et les axes du robot,
        replié dans [-π/4, π/4]. Le lacet de l'objet est donné dans le repère
        monde ; un carton tourné d'un quart de tour a les mêmes faces, et sa
        largeur, par définition, est prise entre les mains : les faces serrées
        sont celles qui regardent les mains. Constaté sur GPU (2026-10-28) :
        sans ce repli, un carton aligné sur le monde, devant un robot tourné
        d'un quart de tour (scène T1), donnait -90°, et les « faces » visées
        étaient l'avant et l'arrière du carton (l'une à 0,49 m) : tout était
        « hors de portée »."""
        c = np.asarray(objet.centre, dtype=float)
        a = self._vers_robot(c)
        b = self._vers_robot(c + np.array([math.cos(objet.lacet), math.sin(objet.lacet), 0.0]))
        brut = math.atan2(float(b[1] - a[1]), float(b[0] - a[0]))
        return (brut + math.pi / 4) % (math.pi / 2) - math.pi / 4

    def _articulations_fixes(self) -> Dict[str, float]:
        """Angles mesurés des articulations de la chaîne que la cinématique
        inverse ne commande pas (taille, lacet d'épaule, poignets) : la
        portée se calcule sur le bras tel qu'il est."""
        try:
            brut = self.mesures.positions()
        except Exception:                 # noqa: BLE001 — capteur en panne : zéro
            return {}
        sortie = {}
        for j, v in (brut.items() if isinstance(brut, dict) else ()):
            try:
                v = float(v)
            except (TypeError, ValueError):
                continue
            if j not in self._q and math.isfinite(v):
                sortie[j] = v
        return sortie

    def _graine(self, q: Dict[str, float]) -> Dict[str, float]:
        """Départ d'une cinématique inverse : `q`, plus les articulations fixes mesurées."""
        return {**self._fixes, **q}

    def _planifier(self, objet: Objet) -> Tuple[bool, str]:
        """Portée et distance au plateau (prise mesurée) : rend (True, "") et
        retient le plan, ou (False, raison du refus)."""
        r, modele = self.r, self._modele
        try:
            centre = self._vers_bassin(objet.centre)
            lacet = self._lacet_relatif(objet)
        except Exception as exc:          # noqa: BLE001 — pose ou hauteur illisible
            return False, f"position du robot illisible ({type(exc).__name__})"
        if not np.all(np.isfinite(centre)) or not math.isfinite(lacet):
            return False, "position de l'objet ou du robot non finie"
        # Prise en pendule : les mains visées au-dessus du centre de gravité.
        dz = hauteur_prise(r, objet.hauteur)
        if dz is None:
            return False, (f"carton trop bas pour une prise en pendule : {100 * objet.hauteur:.0f} "
                           f"cm de haut, il en faut "
                           f"{200 * (r.marge_pendule_m + r.bord_face_m):.0f} (mains à "
                           f"{100 * r.marge_pendule_m:.0f} cm au moins au-dessus de son centre, à "
                           f"{100 * r.bord_face_m:.0f} cm au moins sous son arête)")
        prise = centre + np.array([0.0, 0.0, dz])
        self._fixes = self._articulations_fixes()
        q_ouvert, ouvert, touche = self._ouvert_pendule(prise, objet.largeur, lacet,
                                                        self._graine(self._q))
        q_contact, _ = resoudre(modele, *cibles_mains(prise, objet.largeur, lacet,
                                                     r.demi_main_m), self._graine(self._q))
        q_vide, _ = resoudre(modele, *cibles_mains(prise, objet.largeur, lacet,
                                                  r.demi_main_m - r.penetration_max_m),
                             self._graine(self._q))
        if q_ouvert is None or q_contact is None or q_vide is None:
            return False, (f"hors de portée des bras : objet à {centre[0]:.2f} m devant le "
                           f"bassin et {centre[2]:+.2f} m en hauteur, les mains n'atteignent "
                           f"pas ses faces, {100 * dz:.0f} cm au-dessus de son centre")
        if touche is not None and touche - float(centre[2]) < r.marge_pendule_m + 0.01:
            return False, (f"prise en pendule impossible d'ici : en fermant (le roulis seul), "
                           f"les mains toucheraient {100 * (touche - float(centre[2])):.1f} cm "
                           f"au-dessus du centre de gravité (il en faut "
                           f"{100 * r.marge_pendule_m:.0f}, et 1 de marge) : changer la "
                           f"distance")
        plateau = float(centre[2]) - objet.hauteur / 2.0
        # Du bassin au bord, le long de sa normale (vue en biais si le robot
        # aborde l'objet dans l'axe de ses faces).
        cap_bord = self._cap_bord(objet)
        bord = (float(centre[0]) * math.cos(cap_bord) + float(centre[1]) * math.sin(cap_bord)
                - objet.profondeur / 2.0 - objet.bord_m)
        garde = garde_plateau(modele, self._graine(q_contact), plateau, max(r.bord_min_m, bord),
                              r.rayon_avant_bras_m, cap_bord)
        if garde < r.marge_plateau_m:
            return False, (f"avant-bras à {100 * garde:.1f} cm du plateau (il faut "
                           f"{100 * r.marge_plateau_m:.0f}) : changer la distance à la table")
        # La ligne des points de contact, au contact attendu, doit passer par
        # le centre de gravité (ReglagesPrise, basculement).
        ecart = self._ecart_cg(modele.mains(self._graine(q_contact)), centre)
        if ecart:
            return False, f"prise refusée — {ecart} au contact prévu"
        self._visee = None
        self._plan = {"centre": centre, "prise": prise, "lacet": lacet, "q_ouvert": q_ouvert,
                      "q_contact": q_contact, "q_vide": q_vide, "largeur": objet.largeur,
                      "garde": garde, "bord": bord, "cap_bord": cap_bord,
                      "cibles": cibles_mains(ouvert, objet.largeur, lacet,
                                             r.demi_main_m + r.jeu_ouverture_m)}
        return True, ""

    def _ouvert_pendule(self, prise: np.ndarray, largeur: float, lacet: float,
                        graine: Dict[str, float]
                        ) -> Tuple[Optional[Dict[str, float]], np.ndarray, Optional[float]]:
        """Mains ouvertes pour une prise en `prise` : la fermeture ne tourne
        que le roulis d'épaule, la main décrit un arc autour de l'épaule et
        descend en fermant, d'autant plus qu'elle est haut (robot simulé,
        campagne de cartons : 5,5 cm pour le carton T1), et avance (3,4 mm :
        la ligne des mains devant le centre de gravité, que son poids fait
        basculer et que la pesée lit comme une force). Les mains ouvertes sont
        visées plus haut de ce qu'elles descendront, et plus en arrière de ce
        qu'elles avanceront, tant que les bras y arrivent. Rend (angles
        ouverts, leur cible, hauteur où elles toucheront ; None si la
        fermeture n'y arrive pas par le roulis seul)."""
        r = self.r
        ouvert = np.array(prise, dtype=float)
        q, _ = resoudre(self._modele, *cibles_mains(ouvert, largeur, lacet,
                                                   r.demi_main_m + r.jeu_ouverture_m), graine)
        if q is None:
            return None, ouvert, None
        chute = self._chute_fermeture(q, prise, largeur, lacet, graine)
        for _ in range(4):
            if chute is None or float(np.abs(chute).max()) < 0.002:
                break
            essai = ouvert + chute
            q2, _ = resoudre(self._modele, *cibles_mains(essai, largeur, lacet,
                                                        r.demi_main_m + r.jeu_ouverture_m),
                             {**graine, **q})
            c2 = None if q2 is None else self._chute_fermeture(q2, prise, largeur, lacet, graine)
            if q2 is None or c2 is None:
                break                         # plus haut, hors de portée : on garde
            ouvert, q, chute = essai, q2, c2
        return q, ouvert, None if chute is None else float(prise[2] - chute[2])

    def _chute_fermeture(self, q_ouvert: Dict[str, float], prise: np.ndarray, largeur: float,
                         lacet: float, graine: Dict[str, float]) -> Optional[np.ndarray]:
        """De combien (m) le milieu des points de contact s'écarte de
        `prise`, quand la fermeture (le seul roulis d'épaule,
        _cible_fermeture) amène chaque main des angles `q_ouvert` jusqu'à sa
        face (à `largeur` / 2 de `prise` le long de la normale) : `prise`
        moins ce milieu, sans sa part le long de la normale (en hauteur : de
        combien elles descendent ; en profondeur : de combien elles reculent,
        négatif si elles avancent). None si le roulis n'y suffit pas dans ses
        bornes (la fermeture irait alors vers q_vide, tout le bras)."""
        n = np.array([-math.sin(lacet), math.cos(lacet), 0.0])
        base = {**graine, **q_ouvert}
        z = []
        for k, (cote, j) in enumerate(self._ROULIS.items()):
            chaine = self._chaine(cote)
            signe = 1.0 if k == 0 else -1.0
            cible = signe * (largeur / 2.0 + self.r.demi_main_m)

            def ecart(roulis):
                q2 = dict(base)
                q2[j] = roulis
                return signe * (float((chaine.main(q2) - prise) @ n) - cible)
            lo_b, hi_b = chaine.bornes[j]
            a, b = base[j], (lo_b + 0.02 if self._FERME[cote] < 0 else hi_b - 0.02)
            if ecart(a) <= 0.0 or ecart(b) > 0.0:
                return None
            for _ in range(40):
                m = (a + b) / 2.0
                a, b = (m, b) if ecart(m) > 0.0 else (a, m)
            q2 = dict(base)
            q2[j] = b
            z.append(chaine.main(q2))
        e = np.asarray(prise, dtype=float) - sum(z) / len(z)
        return e - float(e @ n) * n

    def _echouer(self, raison: str) -> None:
        """Échec de la prise mesurée : on ne bouge plus (la consigne reste
        celle du dernier pas), on lève l'exemption ; reinitialiser() rouvre."""
        self.stats["prises_ratees"] += 1
        self.raison = raison
        self._lever_exemption()
        self._regler_raideur(False)
        self._abandonner_calibration(raison)
        self._journaliser_echec(raison)
        self.etat = Etat.ECHEC             # en dernier : qui voit ÉCHEC voit tout le reste

    def _regler_raideur(self, active: bool) -> None:
        """Gains des poignets et du coude : raidis pour fermer et tenir,
        normaux au dépôt, à l'échec, sur arrêt d'urgence (MesuresBras.raideur)."""
        if self.mesures is None or self.mesures.raideur is None or self.raideur_active == active:
            return
        try:
            self.mesures.raideur(active)
        except Exception:                 # noqa: BLE001 — gains inchangés : on le dit
            self.stats["raideur_en_echec"] = self.stats.get("raideur_en_echec", 0) + 1
            return
        self.raideur_active = active
        self._t_raideur = self._horloge()  # le bras bouge encore : pas de zéro avant longtemps

    def _consigne_complete(self) -> Dict[str, float]:
        """La consigne des bras, complétée des articulations qu'elle ne
        commande pas (taille, lacet d'épaule, poignets : leur valeur au plan)."""
        return {**self._fixes, **self._q}

    def _faces(self, mains) -> Tuple[float, float]:
        """Position des faces intérieures des mains (gauche, droite) le long
        de la normale de l'objet, depuis son centre (repère du bassin)."""
        plan, r = self._plan, self.r
        n = np.array([-math.sin(plan["lacet"]), math.cos(plan["lacet"]), 0.0])
        c = np.asarray(plan["centre"], dtype=float)
        return (float((mains[0] - c) @ n) - r.demi_main_m,
                float((mains[1] - c) @ n) + r.demi_main_m)

    def _diagnostic_fermeture(self, mod) -> str:
        """Faces intérieures commandées et mesurées, et les trois plus
        grands écarts consigne-mesure des articulations des bras."""
        cmd = self._modele.mains(self._consigne_complete())
        fg_c, fd_c = self._faces(cmd)
        fg_m, fd_m = self._faces(mod)
        try:
            mesures = {j: float(v) for j, v in self.mesures.positions().items()}
        except Exception:                 # noqa: BLE001
            mesures = {}
        consigne = self._consigne_complete()
        ecarts = sorted(((mesures[j] - v, j) for j, v in consigne.items()
                         if j in mesures and ("shoulder" in j or "elbow" in j or "wrist" in j)),
                        key=lambda e: -abs(e[0]))[:3]
        return (f"faces intérieures commandées à {fg_c - fd_c:.3f} m (G {fg_c:+.3f}, D "
                f"{fd_c:+.3f}), mesurées à {fg_m - fd_m:.3f} m (G {fg_m:+.3f}, D {fd_m:+.3f}) ; "
                f"écarts consigne-mesure : " + ", ".join(f"{j} {e:+.3f} rad" for e, j in ecarts))

    _PHASES_RELEVE = (Etat.FERMETURE, Etat.SERRAGE, Etat.LEVAGE, Etat.RAMENER, Etat.TENU,
                      Etat.REPOSE)

    def releve_fermeture(self) -> Optional[dict]:
        """De la fermeture au port, objet tenu (None sinon), pour les relevés
        du banc (jamais pour décider) : par main, la force estimée (vecteur,
        repère du bassin ; le long de la normale, vers l'objet ; serrage
        filtré), les faces intérieures commandée et mesurée (m, le long de
        la normale, depuis le centre de l'objet), les couples mesurés de ses
        articulations (N·m, bruts, sans correction des biais) et leurs angles
        mesurés (rad) ; pour la prise, la ligne des mains par rapport au
        centre de gravité (m), la charge (poids, poussée vers le robot, N) et
        la rotation vue par la caméra (degrés)."""
        if self.etat not in self._PHASES_RELEVE or self._plan is None:
            return None
        try:
            fc = self._faces(self._modele.mains(self._consigne_complete()))
            mod = self._mains_modele()
            fm = self._faces(mod) if mod is not None else (math.nan, math.nan)
            brut = self.mesures.efforts()
            angles = self.mesures.positions()
        except Exception:                 # noqa: BLE001 — relevé seulement : rien à casser
            return None
        sortie = {"phase": self.etat.name, "sous_phase": self._sous,
                  "ligne_cg": self.ligne_cg, "charge": self.charge,
                  "rotation_vue": self.rotation_vue,
                  # Port : rotation du carton et dérive des mains (mains
                  # mesurées, repère du torse), marge de frottement, resserrages
                  # de ce port ; angles mesurés de la taille (la marche la fait
                  # tourner : le torse sur le bassin).
                  "rotation_port": self.rotation_port, "derive_port": self.derive_port,
                  "marge_frottement": self.marge_frottement,
                  # Le poids de la marge (pesé une fois, bras immobiles), à
                  # côté de la charge lue à chaque instant.
                  "poids_pese": self.poids_carton,
                  "resserrages": self.resserrages_ce_port,
                  "taille": {j: float(v) for j, v in angles.items() if j.startswith("waist_")}}
        for k, cote in enumerate(("left", "right")):
            f = self.forces.get(cote)
            sortie[cote] = {
                "force": None if f is None else tuple(float(v) for v in f),
                "normale": None if f is None else float(f @ self._vers_objet(cote)),
                "serrage": float(self.force_serrage.get(cote, 0.0)),
                "face_commandee": fc[k], "face_mesuree": fm[k],
                "couples": {j: float(brut[j][0]) for j in self._BRAS_COTE[cote] if j in brut},
                "angles": {j: float(angles[j]) for j in self._BRAS_COTE[cote] if j in angles}}
        return sortie

    def _mains_modele(self, reperes: bool = False) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Mains (points de contact ; `reperes` : les repères des mains)
        calculées par la cinématique sur les angles MESURÉS (repère du
        bassin), ou None si les angles sont illisibles."""
        try:
            brut = self.mesures.positions()
            angles = {j: float(v) for j, v in brut.items()}
        except Exception:                 # noqa: BLE001
            return None
        if not all(math.isfinite(v) for v in angles.values()):
            return None
        g, d = (self.mesures.modele if reperes else self._modele).mains({**self._fixes, **angles})
        return g, d

    def _contacts_bassin(self) -> Tuple[np.ndarray, np.ndarray]:
        """Points de contact des mains MESURÉES par le robot (repère du
        bassin) : chaque main mesurée, plus le décalage repère-contact du
        modèle sur les angles mesurés (le robot ne mesure que le repère)."""
        mg, md = self._mains_bassin()
        c, rep = self._mains_modele(), self._mains_modele(reperes=True)
        if c is None or rep is None:
            return mg, md
        return mg + (c[0] - rep[0]), md + (c[1] - rep[1])

    def _interieur(self, mod: Tuple[np.ndarray, np.ndarray]) -> float:
        """Écart (m) entre les faces intérieures des mains `mod` (repère du
        bassin), le long de la normale de l'objet planifié."""
        lacet = self._plan["lacet"]
        n = np.array([-math.sin(lacet), math.cos(lacet), 0.0])
        return float((mod[0] - mod[1]) @ n) - 2 * self.r.demi_main_m

    def etat_visible(self) -> dict:
        """Ce que la prise vise et mesure, repère du bassin, pour une vue de
        diagnostic (lecture seule, aucun effet sur les décisions) : phase,
        objet, centre visé, cibles des mains (ouverture, contact, borne de
        fermeture), mains mesurées (cinématique), contacts estimés."""
        v = {"phase": self.etat.name, "objet": self.objet, "plan": None, "cibles": {},
             "mains": None, "contacts": dict(self.contacts_estimes),
             "forces": dict(self.forces), "serrage": dict(self.force_serrage),
             "recalage": self.recalage}
        try:
            v["mains"] = self._mains_modele()
            plan = self._plan
            if plan:
                v["plan"] = {"centre": plan["centre"], "lacet": plan["lacet"],
                             "largeur": plan["largeur"]}
                modele, fixes = self._modele, self._fixes
                v["cibles"]["ouverture"] = plan["cibles"]
                v["cibles"]["contact"] = modele.mains({**fixes, **plan["q_contact"]})
                if self._cible_ferm:
                    q = {**fixes, **self._q, **self._cible_ferm.get("left", {}),
                         **self._cible_ferm.get("right", {})}
                    v["cibles"]["borne"] = modele.mains(q)
        except Exception:                 # noqa: BLE001 — une vue, jamais une panne
            pass
        return v

    def objet_entre_les_mains(self) -> bool:
        """Signal réel, pour vérifier une prise sans voir l'objet : les faces
        intérieures des mains (cinématique sur les angles mesurés) restent
        écartées d'au moins la largeur de l'objet, moins `vide_m`. Un objet
        qui a glissé laisse les mains se refermer sur le vide."""
        mod = self._mains_modele() if self._plan else None
        return mod is not None and self._interieur(mod) >= self._plan["largeur"] - self.r.vide_m

    def diagnostic_mains(self, objet: Optional[Objet] = None) -> str:
        """Position des deux mains (mesurées, et calculées sur les angles
        mesurés) et celle de l'objet visé, repère du bassin."""
        def f(v):
            return "?" if v is None else "(" + ", ".join(f"{x:.3f}" for x in v) + ")"
        try:
            mg, md = self._mains_bassin()
        except Exception:                 # noqa: BLE001
            mg = md = None
        mod = self._mains_modele() if self.mesures is not None else None
        centre = None
        objet = objet if objet is not None else self.objet
        if objet is not None:
            try:
                centre = self._vers_bassin(objet.centre)
            except Exception:             # noqa: BLE001
                centre = None
        cibles = self._plan["cibles"] if self._plan else (None, None)
        return (f"main G {f(mg)}, main D {f(md)} ; modèle G {f(mod and mod[0])}, "
                f"D {f(mod and mod[1])} ; cibles G {f(cibles[0])}, D {f(cibles[1])} ; "
                f"objet visé {f(centre)}, écart des mains "
                f"{'?' if mg is None else f'{float(np.linalg.norm(mg - md)):.3f}'} m "
                f"(repère du bassin)")

    def _journaliser_echec(self, raison: str, objet: Optional[Objet] = None) -> None:
        if self._journal is None or self.mesures is None:
            return
        try:
            self._journal(f"prise ratée : {raison} — {self.diagnostic_mains(objet)}"
                          + (f" ; {self._texte_forces()}" if self._plan else ""))
        except Exception:                 # noqa: BLE001 — le journal ne doit rien casser
            pass

    def _efforts(self) -> Tuple[Optional[Dict[str, float]], str]:
        """Couples des roulis (exigés) et de toutes les autres articulations
        des bras que le robot mesure."""
        tau, raison = efforts_valides(self.mesures, self._ROULIS.values(), self.r.age_max_effort_s,
                                      optionnels=self._BRAS_COTE["left"] + self._BRAS_COTE["right"])
        if tau is not None:
            self.effort_max_mesure = max([self.effort_max_mesure]
                                         + [abs(v) for v in tau.values()])
        return tau, raison

    def _limite_effort(self, j: str) -> float:
        c = self.mesures.couple_max.get(j)
        lim = self.r.effort_max_nm
        return min(lim, self.r.part_couple_garde * c) if c else lim

    def _effort_excessif(self, tau: Dict[str, float]) -> str:
        """« articulation à X N·m » si une articulation des bras dépasse sa
        limite, sinon ""."""
        for j, v in tau.items():
            if abs(v) > self._limite_effort(j):
                return f"{j} à {abs(v):.1f} N·m, limite {self._limite_effort(j):.1f}"
        return ""

    def _noter(self, msg: str) -> None:
        if self._journal is not None:
            try:
                self._journal(msg)
            except Exception:             # noqa: BLE001 — le journal ne doit rien casser
                pass

    @property
    def _modele(self):
        """Le modèle des bras dont les « mains » sont les points de contact
        (ReglagesPrise.contact_main_m) : celui de toute la géométrie de la
        prise. Les repères des mains (self.mesures.modele) ne servent qu'à
        comparer le modèle aux mains mesurées par le robot."""
        m = self.mesures.modele
        if getattr(self, "_modele_cache", (None, None))[0] is not m:
            self._modele_cache = (m, m.au_contact(self._bout))
        return self._modele_cache[1]

    @property
    def _bout(self) -> float:
        """Où les mains touchent, le long de la main : ce que déclare le
        robot (MesuresBras.contact_main_m), sinon la valeur du G1."""
        v = getattr(self.mesures, "contact_main_m", None)
        return self.r.contact_main_m if v is None else float(v)

    def _chaine(self, cote: str):
        return self._modele.gauche if cote == "left" else self._modele.droite

    def _commandables(self, cote: str) -> Tuple[str, ...]:
        """Articulations du bras `cote` que la prise commande."""
        c = self.mesures.commandables
        base = (f"{cote}_shoulder_pitch_joint", f"{cote}_shoulder_roll_joint",
                f"{cote}_elbow_joint")
        if c is None:
            return base
        return base + tuple(j for j in self._BRAS_COTE[cote] if j in c and j not in base)

    def _angles_mesures(self) -> Optional[Dict[str, float]]:
        try:
            brut = {j: float(v) for j, v in self.mesures.positions().items()}
        except Exception:                 # noqa: BLE001
            return None
        if not all(math.isfinite(v) for v in brut.values()):
            return None
        return {**self._fixes, **brut}

    def _gravite_bassin(self) -> np.ndarray:
        """La pesanteur dans le repère du bassin (centrale inertielle, par
        vers_bassin), ou verticale si le bassin est supposé droit."""
        vb, defaut = self.mesures.vers_bassin, np.array([0.0, 0.0, -9.81])
        if vb is None:
            return defaut
        try:
            g = np.asarray(vb(np.array([0.0, 0.0, -1.0])), float) - np.asarray(vb(np.zeros(3)),
                                                                                float)
        except Exception:                 # noqa: BLE001
            return defaut
        n = float(np.linalg.norm(g))
        return 9.81 * g / n if n > 1e-9 and np.all(np.isfinite(g)) else defaut

    def _force_main(self, cote: str, tau: Dict[str, float], angles: Dict[str, float],
                    g: np.ndarray, depuis_fermeture: bool = True) -> Optional[np.ndarray]:
        """Force (N, repère du bassin) que la main `cote` exerce sur ce
        qu'elle touche : F = (J Jᵀ + λ² I)⁻¹ J Δτ, pseudo-inverse amortie de
        Jᵀ, J la jacobienne de la main (angles mesurés) et Δτ la hausse des
        couples de toutes les articulations mesurées du bras depuis la
        fermeture (ou depuis zéro), le poids des bras retiré (URDF) et les
        biais des couples aussi (calibration des couples)."""
        joints = [j for j in self._BRAS_COTE[cote]
                  if j in tau and (not depuis_fermeture or j in self._tau0)]
        if not joints:
            return None
        chaine = self._chaine(cote)
        # Au point de contact (ReglagesPrise.contact_main_m), pas au repère
        # de la main : 35 % de trop sinon (essai GPU du 5 novembre 2026).
        jac = chaine.jacobienne(angles, joints)
        grav = chaine.couples_gravite(angles, joints, g)
        # Couples corrigés de leur biais, puis de leur gain (calibration de
        # charge ; 1 sans elle).
        b, k = self.biais_couples, self.gains_couples
        d = np.array([(tau[j] - b.get(j, 0.0)) / k.get(j, 1.0) for j in joints]) - grav
        if depuis_fermeture:
            d -= np.array([(self._tau0[j] - b.get(j, 0.0)) / k.get(j, 1.0) - self._g0.get(j, 0.0)
                           for j in joints])
        lam = self.r.amorti_force_m
        return np.linalg.solve(jac @ jac.T + lam * lam * np.eye(3), jac @ d)

    def _ecart_cg(self, contacts, centre) -> str:
        """Où passe la ligne qui joint les points de contact `contacts`
        (repère du bassin) par rapport au centre de gravité `centre` : "" si
        elle y passe, aux tolérances de tolerances_cg près, et au-dessus de
        lui d'au moins marge_pendule_m (prise en pendule), sinon pourquoi.
        Retient aussi l'écart (self.ligne_cg), pour le journal et le banc."""
        prof, haut = ecart_ligne_cg(contacts[0], contacts[1], centre)
        self.ligne_cg = (prof, haut)
        tp, th = tolerances_cg(self.r)
        mp = self.r.marge_pendule_m
        if abs(prof) <= tp and mp <= haut <= th:
            return ""
        return (f"ligne des mains à {100 * prof:+.1f} cm du centre de gravité en profondeur (au "
                f"plus {100 * tp:.1f}) et {100 * haut:+.1f} cm en hauteur (de {100 * mp:.1f} à "
                f"{100 * th:.1f} au-dessus : prise en pendule) : levé, le carton tournerait "
                f"autour d'elle")

    def _vers_objet(self, cote: str) -> np.ndarray:
        """Direction (repère du bassin) dans laquelle la main `cote` serre
        l'objet planifié : sa normale, vers son centre."""
        lacet = self._plan["lacet"]
        n = np.array([-math.sin(lacet), math.cos(lacet), 0.0])
        return -n if cote == "left" else n

    def _mesurer_forces(self, tau: Dict[str, float]) -> Optional[Dict[str, float]]:
        """Serrage de chaque main (N, vers l'objet), moyenne glissante de
        `filtre_force` lectures ; None si les angles sont illisibles."""
        angles = self._angles_mesures()
        if angles is None:
            return None
        g = self._gravite_bassin()
        for cote, j in self._ROULIS.items():
            # Hausse du couple du roulis depuis la fermeture, poids du bras
            # retiré (le bras qui ferme change déjà son couple de ~1 N·m).
            if j in tau and j in self._tau0:
                gj = float(self._chaine(cote).couples_gravite(angles, [j], g)[0])
                self._hausse_roulis[cote] = abs(tau[j] - gj - (self._tau0[j]
                                                                - self._g0.get(j, 0.0)))
        for cote in ("left", "right"):
            f = self._force_main(cote, tau, angles, g)
            self.forces[cote] = f
            if f is not None:
                self._hist_force[cote].append(float(f @ self._vers_objet(cote)))
            h = self._hist_force[cote]
            self.force_serrage[cote] = float(np.mean(h)) if h else 0.0
        self.force_max_mesuree = max([self.force_max_mesuree] + list(self.force_serrage.values()))
        return dict(self.force_serrage)

    def _avancer(self, q: Dict[str, float], cote: str, d: float,
                 direction: Optional[np.ndarray] = None) -> Optional[Dict[str, float]]:
        """La main COMMANDÉE `cote` avance de `d` (m ; négatif : recule) le
        long de `direction` (par défaut, vers l'objet) : cinématique inverse
        différentielle amortie sur toutes les articulations commandées du
        bras. None si elle sort des bornes de l'URDF."""
        chaine = self._chaine(cote)
        joints = self._commandables(cote)
        base = {**self._fixes, **q}
        u = self._vers_objet(cote) if direction is None else np.asarray(direction, float)
        cible = chaine.main(base) + d * u
        for _ in range(4):
            e = cible - chaine.main(base)
            if float(np.linalg.norm(e)) < 2e-5:
                break
            jac = chaine.jacobienne(base, joints)
            dq = jac.T @ np.linalg.solve(jac @ jac.T + 1e-6 * np.eye(3), e)
            for j, v in zip(joints, dq):
                base[j] = base.get(j, 0.0) + float(v)
        if not chaine.dans_les_bornes({j: base[j] for j in joints}):
            return None
        sortie = dict(q)
        sortie.update({j: base[j] for j in joints})
        return sortie

    def _direction_serrage(self, q: Dict[str, float], cote: str) -> np.ndarray:
        """Objet tenu : de la main `cote` vers l'autre (consigne)."""
        g, d = self._modele.mains({**self._fixes, **q})
        u = (d - g) / max(1e-9, float(np.linalg.norm(d - g)))
        return u if cote == "left" else -u

    def _reste_avant_borne(self, q: Dict[str, float], k: int, cote: str) -> float:
        """De combien (m) la face COMMANDÉE de la main peut encore avancer
        avant largeur/2 - penetration_max_m."""
        sens = 1.0 if k == 0 else -1.0
        main = self._chaine(cote).main({**self._fixes, **q})
        face = self._faces((main, main))[k]
        return sens * (face - self._borne(k, cote))

    def _decalage(self, cote: str) -> float:
        """Écart (m) entre la face mesurée et la face commandée de la main
        `cote` au début de la fermeture, avant tout contact (bras qui
        s'affaissent, modèle un peu différent du bras), borné à 3 cm."""
        return max(-0.03, min(0.03, self._ecart_face0.get(cote, 0.0)))

    def _borne(self, k: int, cote: str) -> float:
        """Borne de la face COMMANDÉE : la main, là où elle est vraiment (la
        consigne plus son décalage), jamais à plus de penetration_max_m dans
        la face attendue : à largeur/2 du centre perçu pendant la fermeture
        (celle de la seconde main suit la face touchée par la première), la
        face mesurée au toucher pendant le serrage (_recaler)."""
        sens = 1.0 if k == 0 else -1.0
        return self._attendue[cote] - sens * self.r.penetration_max_m - self._decalage(cote)

    def _texte_forces(self) -> str:
        """Force estimée par main et couple de chaque articulation des bras."""
        def f(v):
            return "?" if v is None else "(" + ", ".join(f"{x:+.1f}" for x in v) + ")"
        parts = [f"forces estimées à la main : G {f(self.forces['left'])} N (serrage "
                 f"{self.force_serrage['left']:.1f} N), D {f(self.forces['right'])} N (serrage "
                 f"{self.force_serrage['right']:.1f} N)"]
        try:
            brut = self.mesures.efforts()
            parts.append("couples : " + ", ".join(
                f"{j.replace('_joint', '')} {float(brut[j][0]):+.2f}"
                for c in ("left", "right") for j in self._BRAS_COTE[c] if j in brut))
        except Exception:                 # noqa: BLE001
            pass
        return " ; ".join(parts)

    def _mains_bassin(self) -> Tuple[np.ndarray, np.ndarray]:
        g, d = self._mains()
        return self._vers_bassin(g), self._vers_bassin(d)

    def _vers_placement(self, q: Dict[str, float], duree: float) -> None:
        self._q_de, self._q_vers = dict(self._q), dict(q)
        self._aller(Etat.PLACEMENT)
        self._t_phase = duree

    def _debut_fermeture(self) -> bool:
        tau, raison = self._efforts()
        if tau is None:
            self._echouer(f"serrage refusé — {raison}")
            return False
        self._tau0 = dict(tau)
        # Lacet d'épaule et poignets : commandés pendant la fermeture et le
        # serrage (plus figés à leur valeur mesurée du plan), depuis leur
        # consigne actuelle.
        for cote in self._ROULIS:
            for j in self._commandables(cote):
                if j not in self._q:
                    self._libres0[j] = float(self._bras.get(j, self._fixes.get(j, 0.0)))
                    self._q[j] = self._libres0[j]
                    self._fixes.pop(j, None)
        # Bruit de la force estimée, bras immobiles, avant de fermer (sous-
        # phase « bruit ») ; τ0 est la moyenne de ces lectures.
        self._sous, self._echantillons, self._ecart_face0 = "bruit", [], {}
        self._hist_force = {c: deque(maxlen=max(1, self.r.filtre_force)) for c in self._ROULIS}
        self._prec_serrage = {}
        self._contact = {"left": None, "right": None}
        self._compte_contact = {"left": 0, "right": 0}
        self._delta = {"left": 0.0, "right": 0.0}
        self._lam = {"left": 0.0, "right": 0.0}
        self._q_ferm0 = dict(self._q)
        # Faces attendues de l'objet, le long de sa normale (repère du bassin).
        self._attendue = {"left": self._plan["largeur"] / 2.0,
                          "right": -self._plan["largeur"] / 2.0}
        self._cible_ferm = {cote: self._cible_fermeture(k, cote)
                            for k, cote in enumerate(self._ROULIS)}
        self.contacts_estimes = {"left": None, "right": None}
        self._regler_raideur(True)
        self._aller(Etat.FERMETURE)
        return True

    def _cible_fermeture(self, k: int, cote: str) -> Dict[str, float]:
        """Où va la consigne d'un bras en fermant : le seul roulis, jusqu'à
        ce que la face COMMANDÉE soit à penetration_max_m dans la face
        attendue de l'objet ; si le roulis n'y suffit pas dans ses bornes,
        tout le bras, vers q_vide (cinématique inverse complète, compensée)."""
        plan, r = self._plan, self.r
        j = self._ROULIS[cote]
        sens = 1.0 if k == 0 else -1.0
        borne = self._borne(k, cote)
        chaine = self._chaine(cote)
        base = self._consigne_complete()

        def face(roulis):
            q2 = dict(base)
            q2[j] = roulis
            main = chaine.main(q2)
            return self._faces((main, main))[k]
        lo_b, hi_b = chaine.bornes[j]
        depart, fin = base[j], (lo_b + 0.02 if self._FERME[cote] < 0 else hi_b - 0.02)
        if sens * (face(fin) - borne) > 0:                 # le roulis n'y suffit pas
            return {jj: v for jj, v in plan["q_vide"].items() if jj.startswith(cote + "_")}
        a, b = depart, fin                                  # face(a) au-delà, face(b) en deçà
        for _ in range(40):
            m = (a + b) / 2.0
            a, b = (m, b) if sens * (face(m) - borne) > 0 else (a, m)
        return {j: b}

    def _pas_placement(self) -> None:
        # Les mains ne doivent rien toucher en allant aux faces : un effort
        # au-delà de la limite, c'est un objet plus grand ou ailleurs.
        tau, raison = self._efforts()
        if tau is None:
            self._echouer(f"serrage refusé — {raison}")
            return
        trop = self._effort_excessif(tau)
        if trop:
            self._echouer(f"prise refusée — contact inattendu en plaçant les mains ({trop}) : "
                          f"l'objet est plus grand que détecté, ou ailleurs")
            return
        if self._hors_consigne0 is None and not self._poser_bras():
            return
        q, fini = self._interp_q(self._t_phase or self.r.duree_placement_s)
        self._ecrire_q(q)
        if not fini:
            return
        plan, r = self._plan, self.r
        # Les mains vues par le robot et celles que donne la cinématique
        # sur les angles mesurés doivent concorder (leurs repères : le robot
        # ne mesure qu'eux), et leurs points de contact être au milieu des
        # faces : sinon, le modèle ne décrit pas le bras, et serrer serait
        # serrer à côté.
        rep = self._mains_modele(reperes=True)
        mod = self._mains_modele()
        if mod is None or rep is None:
            self._echouer("serrage refusé — angles des bras illisibles : position des "
                          "mains inconnue")
            return
        desaccord = max(float(np.linalg.norm(a - b)) for a, b in zip(self._mains_bassin(), rep))
        mg, md = self._contacts_bassin()
        mesurees = (mg, md)
        if desaccord > r.ecart_modele_max_m:
            self._echouer(f"prise refusée — mains mesurées à {100 * desaccord:.1f} cm du modèle "
                          f"des bras (limite {100 * r.ecart_modele_max_m:.1f}) : cinématique à "
                          f"recalibrer")
            return
        axe = np.array([math.cos(plan["lacet"]), math.sin(plan["lacet"]), 0.0])
        erreurs = [m - c for m, c in zip((mg, md), plan["cibles"])]
        erreurs_mod = [m - c for m, c in zip(mod, plan["cibles"])]
        # En profondeur : vers l'arrière (loin du robot), `tol_profondeur_m` ;
        # vers la face avant, au plus avance_max (7 cm derrière elle au moins).
        arriere = max(0.0, max(float(e @ axe) for e in erreurs + erreurs_mod))
        avant = max(0.0, max(-float(e @ axe) for e in erreurs + erreurs_mod))
        prof = max(arriere, avant)
        haut = max(abs(float(e[2])) for e in erreurs + erreurs_mod)
        avance_max = r.avance_max(self.objet.profondeur)
        # Prise en pendule : au serrage, la ligne des mains doit passer à
        # tolerances_cg près du centre de gravité en profondeur, et la fermeture
        # l'en écarte encore (bras affaissés de 0,25 rad : 0,7 cm) ; les mains
        # placées à la moitié au plus, sinon corrigées avant de fermer.
        tol_prof = min(r.tol_profondeur_m, tolerances_cg(r)[0] / 2.0)
        if arriere <= tol_prof and avant <= min(avance_max, tol_prof) and haut <= r.tol_hauteur_m:
            self._debut_fermeture()
            return
        if self._corrections >= r.corrections_max:
            if avant > avance_max:
                # Les mains restent trop près du bord avant : prendre là,
                # c'est prendre au bord. Le robot doit se rapprocher.
                self._echouer(f"prise refusée — mains à {100 * (self.objet.profondeur / 2 - avant):.1f}"
                              f" cm derrière la face avant (il en faut "
                              f"{100 * min(r.recul_face_min_m, self.objet.profondeur / 2):.0f}) après "
                              f"{self._corrections} corrections : hors de portée, se rapprocher")
                return
            self._echouer(f"prise refusée — mains mal placées après {self._corrections} "
                          f"corrections : {100 * prof:.1f} cm en profondeur, {100 * haut:.1f} cm "
                          f"en hauteur")
            return
        # Correction : viser plus loin de l'erreur mesurée (bras qui
        # s'affaissent sous leur poids, modèle un peu différent du bras),
        # en CUMULANT : la visée d'avant, moins l'écart qui reste. Puis
        # revérifier. La compensation est bornée.
        self._corrections += 1
        self.stats["corrections_placement"] = self.stats.get("corrections_placement", 0) + 1
        visee = self._visee if self._visee is not None else list(plan["cibles"])
        visee = [v - (m - c) for v, m, c in zip(visee, mesurees, plan["cibles"])]
        comp = [v - c for v, c in zip(visee, plan["cibles"])]
        trop = max(float(np.linalg.norm(c)) for c in comp)
        if trop > r.compensation_max_m:
            self._echouer(f"prise refusée — mains {100 * trop:.1f} cm loin de leur cible malgré "
                          f"la compensation (limite {100 * r.compensation_max_m:.0f}) : bras "
                          f"qui s'affaissent trop")
            return
        q, _ = resoudre(self._modele, visee[0], visee[1], self._graine(self._q))
        bornes = self._bornes_fermeture(comp)
        if q is None or bornes is None:
            self._echouer("prise refusée — hors de portée des bras pour corriger les mains")
            return
        self._visee = visee
        plan["q_contact"], plan["q_vide"] = bornes
        plan["q_place"] = q                # la reprise repartira de la visée compensée
        self._vers_placement(q, r.duree_placement_s / 2.0)

    def _bornes_fermeture(self, comp) -> Optional[Tuple[Dict[str, float], Dict[str, float]]]:
        """Angles du contact attendu et de la borne de fermeture, pour des
        mains visées avec la compensation `comp` (par main) : ceux du plan
        sont calculés pour des bras qui ne s'affaissent pas."""
        plan, r = self._plan, self.r
        sortie = []
        for ecart in (r.demi_main_m, r.demi_main_m - r.penetration_max_m):
            cg, cd = cibles_mains(plan.get("prise", plan["centre"]), plan["largeur"],
                                  plan["lacet"], ecart)
            q, _ = resoudre(self._modele, cg + comp[0], cd + comp[1], self._graine(self._q))
            if q is None:
                return None
            sortie.append(q)
        return sortie[0], sortie[1]

    def _hors_consigne(self, angles: Dict[str, float]) -> Dict[str, float]:
        """De combien (m) chaque main mesurée (cinématique sur `angles`) est
        en dehors de sa main commandée (cinématique sur la consigne), le long
        de la normale de l'objet, vers l'extérieur ; négatif vers l'objet."""
        try:
            mes = self._modele.mains({**self._fixes, **angles})
            cmd = self._modele.mains(self._consigne_complete())
        except Exception:                 # noqa: BLE001 — angles illisibles : rien à dire
            return {}
        return {c: float((m - k) @ -self._vers_objet(c))
                for c, m, k in zip(("left", "right"), mes, cmd)}

    def _retards_ouverture(self, angles: Dict[str, float]) -> Dict[str, float]:
        """Retard (m) de chaque main sur sa consigne, vers l'extérieur, gagné
        depuis le début du placement, où les mains grandes ouvertes ne
        touchent rien : l'affaissement des bras sous leur poids (et sa
        compensation) y est déjà, il s'annule ; reste ce qui retient la main."""
        maintenant = self._hors_consigne(angles)
        return {c: v - (self._hors_consigne0 or {}).get(c, 0.0) for c, v in maintenant.items()}

    def _poser_bras(self) -> bool:
        """Au début du placement : attend que les bras soient posés (voir
        ReglagesPrise.duree_pose_s), puis prend la référence du contrôle
        géométrique d'appui. Vrai une fois prise ; le geste de placement ne
        commence qu'ensuite."""
        r, t = self.r, self._horloge()
        angles = self._angles_mesures()
        h = {} if angles is None else self._hors_consigne(angles)
        a = self._attente_pose
        if a is None:
            a = self._attente_pose = {"debut": t, "stable": t, "h": h}
        elif not h or any(abs(h.get(c, 0.0) - a["h"].get(c, 0.0)) > r.stabilite_pose_m
                          for c in set(h) | set(a["h"])):
            a.update(stable=t, h=h)       # les bras bougent encore
        if t - a["debut"] > r.pose_max_s:
            self._attente_pose = None
            self._echouer(f"prise refusée — bras toujours en mouvement après "
                          f"{r.pose_max_s:.0f} s : pas de référence pour le contrôle d'appui")
            return False
        self._t0 = t                      # le placement n'a pas encore commencé
        if t - a["stable"] < r.duree_pose_s or t - self._t_raideur < r.duree_raidissement_s:
            return False
        self._hors_consigne0, self._attente_pose = h, None
        return True

    # ---- calibration des couples (au démarrage, bras en l'air) -----------
    def echantillon_calibration_couples(self, posture: str) -> bool:
        """Un relevé, bras immobiles dans `posture` : couples estimés et
        pesanteur du modèle sur les angles mesurés. Faux si une mesure
        manque (rien n'est relevé)."""
        if self.mesures is None:
            return False
        if not self._fixes:
            self._fixes = self._articulations_fixes()
        tau, _ = self._efforts()
        angles = self._angles_mesures()
        if tau is None or angles is None:
            return False
        g = self._gravite_bassin()
        grav: Dict[str, float] = {}
        for cote in self._ROULIS:
            js = [j for j in self._BRAS_COTE[cote] if j in tau]
            grav.update(zip(js, (float(v) for v in self._chaine(cote).couples_gravite(angles, js, g))))
        self.calibration_couples.ajouter(posture, tau, grav)
        return True

    def terminer_calibration_couples(self) -> str:
        """Biais trouvés (désormais retirés des couples dans la force à la
        main) ; rend le bilan, aussi écrit au journal."""
        biais = self.calibration_couples.resoudre()
        self.biais_couples = {j: b for j, (b, _, _) in biais.items()}
        postures = len({p for p, _ in self.calibration_couples.releves})
        texte = CalibrationCouples.bilan(biais, postures)
        self.couples_fiables, pentes = self.calibration_couples.fiabilite()
        if self.couples_fiables is not None:
            detail = ", ".join(f"{j} {v:.2f}" for j, v in sorted(pentes.items()))
            texte += ("\n" + (f"couples estimés fiables : ils suivent la pesanteur du modèle "
                               f"(pente {detail} ; attendu 1)" if self.couples_fiables else
                               f"couples estimés NON FIABLES : ils ne suivent pas la pesanteur du "
                               f"modèle (pente {detail} ; attendu 1"
                               + (", près de -1 : signe inversé ?" if all(
                                   -1.5 <= v <= -0.5 for v in pentes.values()) else "")
                               + ") — la force à la main ne décide plus ni du contact ni du "
                               "serrage (géométrie et retard du roulis) ; elle reste au "
                               "diagnostic"))
        self.stats["couples_fiables"] = self.couples_fiables
        for ligne in texte.splitlines():
            self._noter(ligne)
        self.stats["biais_couples_nm"] = {j: round(b, 2) for j, b in self.biais_couples.items()}
        self.stats["residu_couples_nm"] = round(max((v[1] for v in biais.values()), default=0.0), 3)
        return texte

    def _mesurer_bruit(self, tau: Dict[str, float]) -> bool:
        """Sous-phase « bruit » : lit les couples, bras immobiles, pendant
        `duree_bruit_s` ; puis τ0 (leur moyenne), le bruit de la force à
        chaque main et le seuil de contact. Vrai une fois fini."""
        r = self.r
        if self._horloge() - self._t_raideur < r.duree_raidissement_s:
            # Gains changés il y a peu : le bras bouge encore, la fenêtre
            # du zéro repart plus tard (voir ReglagesPrise.duree_raidissement_s).
            self._echantillons, self._t0 = [], self._horloge()
            return False
        angles = self._angles_mesures()
        if angles is not None:
            self._echantillons.append((dict(tau), angles))
        if self._horloge() - self._t0 < r.duree_bruit_s or len(self._echantillons) < 3:
            return False
        joints = set.intersection(*(set(t) for t, _ in self._echantillons))
        self._tau0 = {j: float(np.mean([t[j] for t, _ in self._echantillons])) for j in joints}
        a0 = {j: float(np.mean([a[j] for _, a in self._echantillons]))
              for j in self._echantillons[0][1]}
        g = self._gravite_bassin()
        self._g0 = {}
        for cote in self._ROULIS:
            js = [j for j in self._BRAS_COTE[cote] if j in self._tau0]
            if js:
                self._g0.update(zip(js, self._chaine(cote).couples_gravite(a0, js, g)))
        for cote in self._ROULIS:
            lus = [self._force_main(cote, t, a, g) for t, a in self._echantillons]
            lus = [float(f @ self._vers_objet(cote)) for f in lus if f is not None]
            self.bruit_force[cote] = float(np.std(lus)) if len(lus) > 1 else 0.0
        filtre = max(self.bruit_force.values(), default=0.0) / math.sqrt(max(1, r.filtre_force))
        self._seuil_contact = max(r.force_contact_n, r.sigmas_contact * filtre)
        self.stats["bruit_force_n"] = {c: round(v, 2) for c, v in self.bruit_force.items()}
        self._noter(f"bruit de la force estimée au repos : G {self.bruit_force.get('left', 0):.2f}"
                    f" N, D {self.bruit_force.get('right', 0):.2f} N par lecture ({len(lus)} "
                    f"lectures) ; seuil de contact {self._seuil_contact:.1f} N")
        # Filet avant de fermer : la force ABSOLUE à chaque main (poids du bras
        # retiré par le modèle), pas celle depuis ce zéro, qu'une main déjà en
        # appui fausserait (le contact serait vu trop tard).
        appui = []
        for cote, nom in (("left", "G"), ("right", "D")):
            f = self._force_main(cote, self._tau0, a0, g, depuis_fermeture=False)
            seuil = max(r.force_appui_n, r.sigmas_appui * self.bruit_force.get(cote, 0.0))
            if f is not None:
                # Le long de la normale, vers l'objet : c'est ainsi qu'une main
                # repose sur sa face (un biais des couples porte aussi ailleurs).
                self.force_avant_fermeture[cote] = float(f @ self._vers_objet(cote))
                appui.append((nom, self.force_avant_fermeture[cote], seuil))
        self.stats["force_avant_fermeture_n"] = {c: round(v, 2) for c, v in
                                                 self.force_avant_fermeture.items()}
        self._noter("force absolue à la main avant de fermer : " + ", ".join(
            f"{nom} {f:.1f} N (seuil {seuil:.1f})" for nom, f, seuil in appui))
        # … et en géométrie : la main mesurée retenue en dehors de sa consigne,
        # le long de la normale de l'objet, à la pose d'ouverture.
        retards = self._retards_ouverture(a0)
        self.stats["retard_ouverture_cm"] = {c: round(100 * v, 1) for c, v in retards.items()}
        self._noter("main mesurée hors de sa consigne à la pose d'ouverture (vers l'extérieur) : "
                    + ", ".join(f"{'G' if c == 'left' else 'D'} {100 * v:+.1f} cm"
                                for c, v in retards.items())
                    + f" (au plus {100 * r.retard_appui_m:.0f})")
        trop = [(nom, f, seuil) for nom, f, seuil in appui if f > seuil]
        if trop and self.couples_fiables is False:
            # Couples non fiables : cette force n'est que du bruit ; la
            # géométrie (juste après) juge seule.
            self._noter("force avant de fermer non jugée : couples estimés non fiables")
            trop = []
        if trop:
            nom, f, seuil = trop[0]
            self._echouer(f"prise refusée — main {nom} déjà en appui sur l'objet avant de fermer "
                          f"({f:.1f} N, seuil {seuil:.1f} N) : la perception le voyait ailleurs")
            return True
        bloquees = [(c, v) for c, v in retards.items() if v > r.retard_appui_m]
        if bloquees:
            c, v = bloquees[0]
            self._echouer(f"prise refusée — main {'G' if c == 'left' else 'D'} déjà en appui sur "
                          f"l'objet avant de fermer (retenue à {100 * v:.1f} cm de sa consigne, "
                          f"au plus {100 * r.retard_appui_m:.0f}) : la perception le voyait "
                          f"ailleurs")
            return True
        self._sous, self._echantillons = "", []
        self._t0 = self._horloge()
        return True

    def _pas_fermeture(self, dt: float) -> None:
        r = self.r
        tau, raison = self._efforts()
        if tau is None:
            self._echouer(f"serrage refusé — {raison}")
            return
        if self._sous == "bruit":
            self._mesurer_bruit(tau)
            return
        forces = self._mesurer_forces(tau)
        if not self._ecart_face0:
            mod0 = self._mains_modele()
            if mod0 is not None:
                fm0 = self._faces(mod0)
                fc0 = self._faces(self._modele.mains(self._consigne_complete()))
                self._ecart_face0 = {c: fm0[k] - fc0[k] for k, c in enumerate(self._ROULIS)}
                # Bornes de fermeture refaites pour ce décalage.
                self._cible_ferm = {c: self._cible_fermeture(k, c)
                                    for k, c in enumerate(self._ROULIS)}
        qm, raison = positions_valides(self.mesures, self._ROULIS.values())
        if qm is None:
            self._echouer(f"serrage refusé — {raison}")
            return
        # Vide : décidé d'après les MAINS (cinématique sur tous les angles
        # mesurés), jamais d'après le seul angle du roulis.
        mod = self._mains_modele()
        if mod is None:
            self._echouer("serrage refusé — angles des bras illisibles : position des mains "
                          "inconnue")
            return
        plan = self._plan
        interieur = self._interieur(mod)
        if interieur < plan["largeur"] - r.vide_m:
            self._echouer(f"prise refusée — rien entre les mains : serrage dans le vide évité "
                          f"(faces intérieures des mains à {interieur:.3f} m l'une de l'autre, "
                          f"l'objet attendu en ferait {plan['largeur']:.2f} ; "
                          f"{self._diagnostic_fermeture(mod)})")
            return
        # Faces intérieures des mains, COMMANDÉES (cinématique sur la
        # consigne complète) et MESURÉES (sur les angles mesurés), le long
        # de la normale de l'objet ; face attendue de l'objet : ±largeur/2.
        faces_c = self._faces(self._modele.mains(self._consigne_complete()))
        faces_m = self._faces(mod)
        q = dict(self._q)
        en_butee = 0
        for k, (cote, j) in enumerate(self._ROULIS.items()):
            if self._contact[cote] is not None:
                continue
            s = self._FERME[cote]
            sens = 1.0 if k == 0 else -1.0       # la face gauche descend, la droite monte
            attendue = self._attendue[cote]
            hausse = (self._hausse_roulis[cote] if forces is not None
                      else abs(tau[j] - self._tau0[j]))
            retard = s * (self._q[j] - qm[j])
            # Retard pris DEPUIS le début de la fermeture : bras affaissés
            # (tangage incliné), la main mesurée est déjà à côté de la
            # commandée avant tout contact (constaté sur le robot simulé : 2 cm).
            retard_face = sens * ((faces_m[k] - faces_c[k]) - self._ecart_face0.get(cote, 0.0))
            force = forces.get(cote, 0.0) if forces is not None else 0.0
            signal = signal_contact(r, hausse, retard, force, self._seuil_contact, retard_face,
                                    abs(faces_m[k] - attendue), self.couples_fiables is not False)
            if signal is not None:
                # Contact probable : la consigne n'avance plus, on le
                # confirme au pas suivant (jamais d'élan dans l'objet).
                self._compte_contact[cote] += 1
                if self._compte_contact[cote] >= 2:
                    self._contact[cote] = self._q[j]
                    self.contacts_estimes[cote] = {"t": self._horloge(), "signal": signal}
                    # L'objet est maintenant situé : la face opposée est
                    # attendue à une largeur de celle-ci (une main touche
                    # avant l'autre quand l'objet est un peu décalé), et la
                    # borne de l'autre main suit.
                    autre = "right" if cote == "left" else "left"
                    if self._contact[autre] is None:
                        self._attendue[autre] = faces_m[k] - sens * plan["largeur"]
                        self._cible_ferm[autre] = self._cible_fermeture(1 - k, autre)
                        self._q_ferm0.update({jj: self._q[jj] for jj in self._cible_ferm[autre]})
                        self._lam[autre] = 0.0
                continue
            self._compte_contact[cote] = 0
            # Borne de la consigne : la face COMMANDÉE ne va jamais au-delà de
            # largeur/2 - penetration_max_m (la cible de fermeture l'y amène
            # en lam = 1).
            borne = self._borne(k, cote)
            if self._lam[cote] >= 1.0 or sens * (faces_c[k] - borne) <= 1e-4:
                en_butee += 1
                continue
            # Vers la cible de fermeture (le roulis, ou tout le bras si le
            # roulis n'y suffit pas) ; près du contact attendu, petits pas :
            # le dépassement d'un pas (avant que le contact soit vu) reste
            # petit, même objet raide.
            cible = self._cible_ferm[cote]
            course = max(abs(v - self._q_ferm0[jj]) for jj, v in cible.items())
            pres = sens * (faces_c[k] + self._decalage(cote) - attendue) <= r.zone_contact_m
            pas = (min(r.vitesse_contact * min(dt, 0.05), r.pas_contact_max_rad) if pres
                   else r.vitesse_fermeture * min(dt, 0.05))
            self._lam[cote] = 1.0 if course < 1e-9 else min(1.0, self._lam[cote] + pas / course)
            for jj, v in cible.items():
                q[jj] = self._q_ferm0[jj] + self._lam[cote] * (v - self._q_ferm0[jj])
        self._ecrire_q(q)
        libres = sum(1 for v in self._contact.values() if v is None)
        if libres and en_butee == libres:
            self._echouer(f"prise refusée — fermeture au bout de sa course sans contact : faces "
                          f"intérieures des mains à {interieur:.3f} m l'une de l'autre, l'objet "
                          f"attendu en ferait {plan['largeur']:.2f} ; "
                          f"{self._diagnostic_fermeture(mod)}")
            return
        if all(v is not None for v in self._contact.values()):
            mg, md = self._contacts_bassin()
            ecart = float(np.linalg.norm((mg - md)[:2]))
            attendu = self._plan["largeur"] + 2 * r.demi_main_m
            if ecart > attendu + r.ecart_largeur_max_m:
                self._echouer(f"prise refusée — contact avec les mains à {ecart:.2f} m l'une "
                              f"de l'autre, l'objet attendu en ferait {attendu:.2f} : ce n'est "
                              f"pas lui")
                return
            if not self._recaler(faces_m):
                return
            self._ecart_mains = ecart
            self._t_phase = None
            self._delta = {"left": 0.0, "right": 0.0}
            self._plafond = {}
            self._delta_max = {c: max(0.0, self._reste_avant_borne(self._q, k, c))
                               for k, c in enumerate(self._ROULIS)}
            self._aller(Etat.SERRAGE)
        elif self._horloge() - self._t0 > 4.0:
            self._echouer("prise refusée — rien entre les mains : serrage dans le vide évité "
                          "(fermeture trop longue)")

    def _recaler(self, faces_m: Tuple[float, float]) -> bool:
        """Recalage au toucher, avant de serrer : les faces intérieures
        MESURÉES des deux mains au contact sont les faces du carton. Le
        centre du plan est déplacé à leur milieu, le long de la normale, et
        la borne du serrage de chaque main est prise depuis sa face mesurée ;
        au-delà de `recalage_max_m`, la prise est refusée (la perception le
        voyait ailleurs). Le lacet n'est pas observable par deux mains plates
        à la même profondeur : seule la largeur mesurée en borne l'écart
        (|Δ| = arccos(largeur / écart des faces)), donné au journal."""
        plan, r = self._plan, self.r
        decalage = (faces_m[0] + faces_m[1]) / 2.0
        ecart = faces_m[0] - faces_m[1]
        d_lacet = math.degrees(math.acos(min(1.0, plan["largeur"] / ecart))) if ecart > 0 else 0.0
        n = np.array([-math.sin(plan["lacet"]), math.cos(plan["lacet"]), 0.0])
        avant = np.asarray(plan["centre"], dtype=float)
        self.recalage = {"avant": avant, "apres": avant + decalage * n, "decalage_m": decalage,
                         "largeur_mesuree_m": ecart, "ecart_lacet_max_deg": d_lacet}
        self.stats["recalage_m"] = round(decalage, 4)
        texte = (f"recalage au toucher : carton à {100 * decalage:+.1f} cm de la perception le "
                 f"long de sa normale, largeur mesurée {ecart:.3f} m (lacet à au plus "
                 f"{d_lacet:.0f}° de la perception)")
        if abs(decalage) > r.recalage_max_m:
            self._echouer(f"prise refusée — {texte} : au-delà de {100 * r.recalage_max_m:.0f} cm, "
                          f"la perception le voyait ailleurs")
            return False
        self._noter(texte)
        plan["centre"] = avant + decalage * n
        # Bornes du serrage depuis les faces MESURÉES au toucher (comptées
        # depuis le centre recalé), et non depuis le centre estimé : aucune
        # main ne va plus de penetration_max_m dans la face qu'elle touche,
        # quelle que soit l'erreur de la perception.
        self._attendue = {"left": faces_m[0] - decalage, "right": faces_m[1] - decalage}
        # Une main déjà au-delà (sa fermeture visait la face perçue) recule
        # jusqu'à sa borne avant de serrer.
        q = dict(self._q)
        for k, cote in enumerate(self._ROULIS):
            reste = self._reste_avant_borne(q, k, cote)
            if reste < 0.0:
                q = self._avancer(q, cote, reste) or q
        if q != self._q:
            self._ecrire_q(q)
        return True

    def _pas_serrage_mesure(self, dt: float) -> None:
        """Serrage : la face COMMANDÉE de chaque main avance vers l'objet
        (cinématique inverse sur tout le bras, petits pas), jusqu'à la force
        visée à la main, jamais au-delà de largeur/2 - penetration_max_m ;
        une articulation du bras au-delà de sa limite de couple fait
        desserrer. Couples non fiables : jusqu'à la borne, et le serrage se
        juge en géométrie (voir ReglagesPrise.retard_serrage_m)."""
        r = self.r
        geo = self.couples_fiables is False
        tau, raison = self._efforts()
        if tau is None:
            self._echouer(f"serrage refusé — {raison}")
            return
        forces = self._mesurer_forces(tau)
        if forces is None:
            self._echouer("serrage refusé — angles des bras illisibles : force inconnue")
            return
        q = dict(self._q)
        prets = True
        for k, cote in enumerate(self._ROULIS):
            f = forces[cote]
            trop = [j for j in self._BRAS_COTE[cote] if j in tau
                    and abs(tau[j]) > self._limite_effort(j)]
            if trop:
                # Au-dessus d'une limite : on desserre, jamais plus fort, et
                # cette main ne serre plus (on jugera la force obtenue).
                d = -2 * r.pas_serrage_max_m
                self.stats["serrages_limites"] = self.stats.get("serrages_limites", 0) + 1
                self._plafond[cote] = trop[0]
                prets = False
            elif (geo or f < r.force_serrage_n) and not self._plafond.get(cote):
                # Jusqu'à la borne de la face commandée, et de la consigne
                # comptée (_delta_max) : la cinématique inverse s'arrête à
                # 20 µm près par pas, la somme des pas pourrait la dépasser.
                reste = min(self._reste_avant_borne(q, k, cote),
                            self._delta_max.get(cote, 0.0) - self._delta[cote])
                if reste <= r.pas_serrage_min_m:
                    continue                      # au bout : on juge ce qu'on a
                if geo:
                    # Rien ne mesure la force : jusqu'à la borne, au plus petit pas.
                    d = min(r.pas_serrage_min_m, reste)
                else:
                    raideur = self._raideur(cote, f)
                    # Pas réduit pour arriver à la force visée sans la dépasser ;
                    # le plus petit tant que la raideur n'est pas mesurée.
                    pas = (min(r.pas_serrage_max_m, max(r.pas_serrage_min_m,
                                                        (r.force_serrage_n - f) / raideur))
                           if raideur > 0.0 else r.pas_serrage_min_m)
                    d = min(pas, reste)
            else:
                continue
            q2 = self._avancer(q, cote, d)
            if q2 is None:
                if d > 0:
                    self._plafond[cote] = "bornes de l'URDF"
                continue                          # bornes du bras : on juge ce qu'on a
            prets = False
            q = q2
            self._delta[cote] += d
        self._ecrire_q(q)
        # Jamais un serrage sans fin : au-delà de `duree_serrage_max_s`, on
        # juge la force obtenue.
        if not prets and self._horloge() - self._t0 < r.duree_serrage_max_s:
            self._t_phase = None
            return
        if geo:
            retards, texte = self._retenues()
            if not retards or not all(v[2] for v in retards.values()):
                self._echouer(f"prise refusée — serrage insuffisant ({texte}) : l'objet cède ou "
                              f"glisse, on n'insiste pas (force estimée non fiable, non jugée)")
                return
            if self._t_phase is None:
                self._t_phase, self._retards_maintien = self._horloge(), retards
            elif self._horloge() - self._t_phase >= r.duree_maintien_s:
                # Main bloquée : sa face mesurée n'a pas bougé (consigne fixe).
                glisse = [(c, abs(v[3] - self._retards_maintien[c][3]))
                          for c, v in retards.items()
                          if abs(v[3] - self._retards_maintien[c][3]) > r.glissement_serrage_m]
                if glisse:
                    c, g_ = glisse[0]
                    self._echouer(f"prise refusée — serrage insuffisant (main "
                                  f"{'G' if c == 'left' else 'D'} déplacée de {100 * g_:.1f} cm "
                                  f"pendant le maintien, au plus "
                                  f"{100 * r.glissement_serrage_m:.1f}) : l'objet cède ou glisse, "
                                  f"on n'insiste pas")
                    return
                self._noter(f"serrage jugé en géométrie (couples non fiables) : {texte}")
                self._q_serre = dict(self._q)
                self._lever_de_quelques_cm()
            return
        faible = min(forces.values())
        if faible < r.force_min_n:
            limites = ", ".join(f"main {'G' if c == 'left' else 'D'} arrêtée par "
                                + (j if j.startswith("bornes") else f"la limite de couple de {j}")
                                for c, j in self._plafond.items() if j)
            self._echouer(f"prise refusée — serrage insuffisant ({faible:.1f} N à la main au plus "
                          f"serré, {r.force_min_n:.0f} au moins"
                          f"{' ; ' + limites if limites else ''}) : l'objet cède ou glisse, on "
                          f"n'insiste pas")
            return
        if self._t_phase is None:
            self._t_phase = self._horloge()
        elif self._horloge() - self._t_phase >= r.duree_maintien_s:
            self._q_serre = dict(self._q)
            self._lever_de_quelques_cm()

    def _retenues(self) -> Tuple[Dict[str, Tuple[float, float, bool, float]], str]:
        """Serrage en géométrie (couples non fiables), par main : (retard,
        écart à la face touchée, retenue, face mesurée) et le texte du
        jugement. Retard :
        la face mesurée en arrière de la commandée, le long de la normale,
        depuis le début de la fermeture (comme le contact) ; retenue s'il
        atteint `retard_serrage_m`, face mesurée à moins de
        `tol_face_contact_m` de la face touchée (pas un objet qui s'enfonce).
        Rien entre les mains : la face mesurée suit la commandée, pas de
        retard, pas de retenue."""
        r = self.r
        mod = self._mains_modele()
        if mod is None:
            return {}, "angles des bras illisibles : retard inconnu"
        fc = self._faces(self._modele.mains(self._consigne_complete()))
        fm = self._faces(mod)
        sortie, parts = {}, []
        for k, cote in enumerate(self._ROULIS):
            sens = 1.0 if k == 0 else -1.0
            retard = sens * ((fm[k] - fc[k]) - self._ecart_face0.get(cote, 0.0))
            ecart = abs(fm[k] - self._attendue.get(cote, fm[k]))
            tenue = main_retenue(r, retard, ecart)
            sortie[cote] = (retard, ecart, tenue, fm[k])
            parts.append(f"main {'G' if k == 0 else 'D'} retenue à {100 * retard:.1f} cm de sa "
                         f"consigne (au moins {100 * r.retard_serrage_m:.0f}), à {100 * ecart:.1f} "
                         f"cm de la face touchée (au plus {100 * r.tol_face_contact_m:.1f})")
        return sortie, " ; ".join(parts)

    def _raideur(self, cote: str, force: float) -> float:
        """Raideur mesurée (N/m) au serrage : hausse de la force à la main
        sur l'avance de la face commandée depuis la dernière mesure ; 0 si
        elle n'est pas encore connue."""
        prec = self._prec_serrage.get(cote)
        if prec is None:
            self._prec_serrage[cote] = (self._delta[cote], force)
            return 0.0
        if self._delta[cote] - prec[0] < 1e-3:
            return self._raideur_vue.get(cote, 0.0)
        self._prec_serrage[cote] = (self._delta[cote], force)
        self._raideur_vue[cote] = max(0.0, (force - prec[1]) / (self._delta[cote] - prec[0]))
        return self._raideur_vue[cote]

    def _garder_sous_la_limite(self, dt: float) -> None:
        """Objet tenu : un roulis au-delà de sa limite d'effort desserre un
        peu à chaque pas (jamais plus fort que la limite, même pour tenir).
        Effort illisible : rien ne change ici, et aucun serrage de plus
        (voir _surveiller_inclinaison)."""
        tau, raison = self._efforts()
        if tau is None:
            # Jamais « tout va bien » : la façade et l'app le voient.
            if not self.raison.startswith("effort illisible"):
                self.stats["efforts_illisibles"] = self.stats.get("efforts_illisibles", 0) + 1
            self.raison = f"effort illisible pendant le port ({raison}) : serrage figé"
            return
        if self.raison.startswith("effort illisible"):
            self.raison = ""
        pas = 2 * self.r.pas_serrage_max_m
        if self.etat is Etat.TENU and self._port0 is not None:
            # Port figé : la main au-delà de sa limite recule, et l'autre du même
            # pas, le long de la normale (à travers l'objet, le serrage est le
            # même des deux côtés : reculer d'un seul côté fait glisser la paire,
            # essai GPU T1 du 19 novembre 2026, 17 h 16).
            trop = [c for c in self._ROULIS
                    if any(abs(tau[j]) > self._limite_effort(j) for j in self._BRAS_COTE[c]
                           if j in tau) and self._delta[c] > -self.r.penetration_max_m]
            if not trop:
                return
            # Autant de serrage rendu qu'en reculant la seule main fautive de
            # `pas` : chacune de la moitié.
            avant = dict(self._delta)
            for cote in self._ROULIS:
                self._delta[cote] -= pas / 2.0
            self.stats["serrages_limites"] = self.stats.get("serrages_limites", 0) + len(trop)
            q = self._q_port()
            if q is None:
                # Sécurité d'abord : la main au-delà de sa limite recule quand même.
                self._delta, q = avant, dict(self._q)
                for cote in trop:
                    self._delta[cote] -= pas
                    q2 = self._avancer(q, cote, -pas, self._direction_serrage(q, cote))
                    q = q2 if q2 is not None else q
            self._ecrire_q(q)
            self._q_tenu = dict(q)
            return
        q = None
        for cote in self._ROULIS:
            trop = any(abs(tau[j]) > self._limite_effort(j) for j in self._BRAS_COTE[cote]
                       if j in tau)
            if trop and self._delta[cote] > -self.r.penetration_max_m:
                self._delta[cote] -= pas
                self.stats["serrages_limites"] = self.stats.get("serrages_limites", 0) + 1
                if self.etat is Etat.TENU:      # consigne fixe : on la corrige ici
                    q = dict(self._q) if q is None else q
                    q2 = self._avancer(q, cote, -pas, self._direction_serrage(q, cote))
                    q = q2 if q2 is not None else q
        if q is not None:
            self._ecrire_q(q)
            self._q_tenu = dict(q)

    def _q_mains(self, centre_mains: np.ndarray, ecart: float) -> Optional[Dict[str, float]]:
        """Bras qui mettent les mains de part et d'autre de `centre_mains`
        (repère du bassin), à `ecart` l'une de l'autre, le serrage gardé :
        les mains COMMANDÉES au serrage suivent, en translation, le milieu
        des mains mesurées (leur avance dans l'objet, le serrage, reste la
        même en cartésien, quelle que soit la posture), plus la variation
        du serrage. Le lacet d'épaule et les poignets restent ceux du
        serrage : la cinématique inverse ne résout que tangage, roulis et
        coude, les autres angles pris dans sa graine. Depuis le retour
        (_rigide), les deux mains suivent l'objet comme un seul corps rigide,
        orientation comprise (_q_rigide), ou None : la cinématique d'avant,
        partie des angles du serrage, leur ferait tourner d'un coup."""
        if self._rigide is not None:
            return self._q_rigide(centre_mains, ecart)
        cg, cd = self._modele.mains({**self._fixes, **self._q_serre})
        t = np.asarray(centre_mains, dtype=float) - self._c_serre
        e = np.array([0.0, (ecart - self._ecart_mains) / 2.0, 0.0])
        q, _ = resoudre(self._modele, cg + t + e, cd + t - e,
                        # 0,5 mm près : l'écart des mains EST le serrage (3 mm, sur un
                        # objet raide, feraient varier l'effort de plusieurs N·m).
                        self._graine(self._q_serre), tol=0.0005)
        if q is None:
            return None
        sortie = dict(self._q_serre)
        sortie.update(q)
        for cote in self._ROULIS:
            d = self._delta[cote] - self._delta_serre[cote]
            if abs(d) > 1e-6:
                sortie = self._avancer(sortie, cote, d, self._direction_serrage(sortie, cote))
                if sortie is None:
                    return None
        return sortie

    def _poses(self, q: Dict[str, float]) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
        """Pose (point de contact, orientation 3 × 3) de chaque main pour
        les angles `q`, complétés des articulations fixes (repère du bassin)."""
        return {c: self._chaine(c).pose({**self._fixes, **q}) for c in self._ROULIS}

    def _vers_pose(self, q: Dict[str, float], cote: str, cible: np.ndarray,
                   orientation: np.ndarray) -> Optional[Dict[str, float]]:
        """Angles du bras `cote` qui mettent sa main (point de contact) en
        `cible`, avec l'orientation `orientation`, en partant de `q` :
        moindres carrés amortis sur les articulations que la prise commande
        (la pose entière : six au moins). None hors des tolérances
        (tol_rigide_m, tol_rigide_deg) ou des bornes de l'URDF."""
        r = self.r
        chaine, joints = self._chaine(cote), self._commandables(cote)
        echelle = 0.1                     # m par radian : rotation et position comparables
        base = {**self._fixes, **q}
        for _ in range(12):
            p, rot = chaine.pose(base)
            e = np.concatenate([cible - p, echelle * _rotation_vecteur(orientation @ rot.T)])
            if float(np.linalg.norm(e[:3])) < 0.1 * r.tol_rigide_m and float(
                    np.linalg.norm(e[3:])) < 0.1 * echelle * math.radians(r.tol_rigide_deg):
                break
            jac = chaine.jacobienne(base, joints, rotation=True)
            jac[3:] *= echelle
            dq = jac.T @ np.linalg.solve(jac @ jac.T + 1e-6 * np.eye(6), e)
            for j, v in zip(joints, np.clip(dq, -0.2, 0.2)):
                base[j] += float(v)
        p, rot = chaine.pose(base)
        tourne = _rotation_vecteur(orientation @ rot.T)
        if float(np.linalg.norm(cible - p)) > r.tol_rigide_m \
                or math.degrees(float(np.linalg.norm(tourne))) > r.tol_rigide_deg \
                or not chaine.dans_les_bornes({j: base[j] for j in joints}):
            return None
        sortie = dict(q)
        sortie.update({j: base[j] for j in joints})
        return sortie

    def _q_rigide(self, centre: np.ndarray, ecart: Optional[float] = None,
                  graine: Optional[Dict[str, float]] = None,
                  delta: Optional[Dict[str, float]] = None,
                  rotation: Optional[np.ndarray] = None) -> Optional[Dict[str, float]]:
        """Bras qui portent les deux mains COMMANDÉES comme un seul corps
        rigide avec l'objet : chaque main à sa pose du début du retour
        (_rigide), translatée de `centre` moins le centre de départ, la même
        translation pour les deux, orientation inchangée (l'objet ne roule ni
        ne tangue), plus la variation du serrage (`delta`, l'actuel par
        défaut ; et de l'écart des mains) le long de la ligne des mains de
        départ. `rotation` (3 × 3, la correction de dérive du port) tourne
        l'ensemble autour du centre de départ. None si un bras n'y arrive pas."""
        c0, poses, delta0, ecart0 = self._rigide
        delta = self._delta if delta is None else delta
        t = np.asarray(centre, dtype=float) - c0
        de = 0.0 if ecart is None else (ecart - ecart0) / 2.0
        rc = np.eye(3) if rotation is None else np.asarray(rotation, dtype=float)
        q = dict(self._q if graine is None else graine)
        for cote in self._ROULIS:
            p, rot, u = poses[cote]
            q = self._vers_pose(q, cote, c0 + rc @ (p - c0) + t
                                + (delta[cote] - delta0[cote] - de) * (rc @ u), rc @ rot)
            if q is None:
                return None
        return q

    def _chemin_rigide(self, de: np.ndarray, vers: np.ndarray, plateau: Optional[float],
                       graine: Optional[Dict[str, float]] = None, pas_m: Optional[float] = None
                       ) -> Optional[Dict[str, float]]:
        """Angles des bras au bout du chemin rigide de l'objet tenu, de `de`
        à `vers` (son centre), si TOUT le chemin passe : chaque étape
        (pas_chemin_rigide_m) atteinte aux tolérances du corps rigide, aucune
        articulation qui saute de plus de saut_rigide_rad d'une étape à
        l'autre (un bras replié contre le corps change de solution en chemin :
        la consigne saute, le garde-fou la refuse), avant-bras jamais plus près
        du plateau qu'ils survolent encore que marge_plateau_m, ni qu'au départ
        s'ils en partent plus près (en haut du levage) ; None sinon. Le chemin
        part des angles actuels, ou de `graine` (la fin d'un chemin précédent),
        par étapes de `pas_m` (pas_chemin_rigide_m par défaut)."""
        r = self.r
        n = max(1, math.ceil(float(np.linalg.norm(vers - de)) / (pas_m or r.pas_chemin_rigide_m)))
        prec = self._q if graine is None else graine

        # Bord de la table supposé en chemin : à 15 cm du bassin au plus loin
        # (l'ancien bord_min_m : jamais moins prudent qu'avant), au vrai bord
        # s'il est plus près (robot pieds contre la table, bord_min_m).
        plan = getattr(self, "_plan", None)
        vrai = float(plan["bord"]) if plan and "bord" in plan else r.bord_min_m
        bord = max(r.bord_min_m, min(0.15, vrai))
        cap_bord = float(plan.get("cap_bord", 0.0)) if plan else 0.0

        def garde(q):
            return math.inf if plateau is None else garde_plateau(
                self._modele, self._graine(q), plateau, bord, r.rayon_avant_bras_m, cap_bord)
        plancher = min(r.marge_plateau_m, garde(prec)) - 1e-4
        for k in range(1, n + 1):
            q = self._q_rigide(de + (k / n) * (vers - de), graine=prec)
            if q is None or max(abs(v - prec.get(j, v)) for j, v in q.items()) > r.saut_rigide_rad \
                    or garde(q) < plancher:
                return None
            prec = q
        return prec

    def _rotation_estimee(self) -> Optional[Tuple[float, float]]:
        """Roulis (autour de l'axe avant) et tangage (autour de la ligne des
        mains) de l'objet tenu depuis le début du retour (degrés), estimés sur
        les mains MESURÉES : il tourne avec les paumes qui le serrent
        (rotation moyenne des deux mains), et roule si la ligne des mains
        penche (leurs hauteurs). Les couples ne s'y prêtent pas : le couple
        de basculement passe sous leur bruit (voir _surveiller_rotation)."""
        angles = self._angles_mesures()
        if angles is None or self._poses_mesurees0 is None:
            return None
        mes = self._poses(angles)
        rv = sum(_rotation_vecteur(mes[c][1] @ self._poses_mesurees0[c][1].T)
                 for c in self._ROULIS) / 2.0
        ligne, ligne0 = (mes["left"][0] - mes["right"][0],
                         self._poses_mesurees0["left"][0] - self._poses_mesurees0["right"][0])
        penche = math.atan2(ligne[2], ligne[1]) - math.atan2(ligne0[2], ligne0[1])
        roulis = max((float(rv[0]), penche), key=abs)
        return math.degrees(roulis), math.degrees(float(rv[1]))

    # ---- port : pose figée, surveillance des mains mesurées ------------------

    def _poses_torse(self, angles: Dict[str, float]) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
        """Poses des mains pour les angles `angles`, la taille à sa valeur
        figée (articulations fixes) : repère du torse. La marche fait tourner
        la taille ; les bras, eux, sont commandés dans ce repère."""
        return self._poses({j: v for j, v in angles.items() if not j.startswith("waist_")})

    def _figer_port(self) -> None:
        """Entrée en TENU (début du port, ou fin d'un geste du port) : la
        posture commandée devient la graine des bras du port (_q_port), la
        position mesurée commune des mains la référence de leur dérive ; au
        début du port seulement, les paumes mesurées et la ligne des mains,
        références de la rotation (_surveiller_mains_port)."""
        angles = self._angles_mesures()
        if self._rigide is None or angles is None:
            return
        mes = self._poses_torse(angles)
        if self._port0 is None:
            self._port0 = {"ori0": {c: mes[c][1] for c in self._ROULIS},
                           "ligne0": mes["left"][0] - mes["right"][0]}
            self.rotation_port, self.rotation_port_max, self.derive_port = None, 0.0, None
            self.resserrages_ce_port = self.corrections_ce_port = 0
            self._alarme_rotation = False
        self._port0.update(q=dict(self._q), pos0=(mes["left"][0] + mes["right"][0]) / 2.0)
        self._correction_port = self._correction_visee = (np.zeros(3), np.zeros(3))
        self._hist_derive.clear()
        self._t_correction = None

    def _q_port(self, delta: Optional[Dict[str, float]] = None,
                correction: Optional[Tuple[np.ndarray, np.ndarray]] = None
                ) -> Optional[Dict[str, float]]:
        """Bras du port : la pose figée des deux mains (corps rigide, _rigide)
        au centre porté, le serrage `delta` (l'actuel par défaut) le long de
        la normale, et la correction de dérive (translation et vecteur
        rotation communs ; l'actuelle par défaut). Calculés depuis la posture
        figée (graine), jamais depuis la consigne du moment : le même serrage
        donne les mêmes bras, rien ne dérive d'un resserrage à l'autre."""
        t, rv = self._correction_port if correction is None else correction
        centre = np.asarray(self._centre_porte, dtype=float) + t
        rot = _matrice_rotation(rv)
        graine = self._port0["q"] if self._port0 is not None else None
        q = self._q_rigide(centre, self._ecart_mains, graine=graine, delta=delta, rotation=rot)
        if q is None and graine is not None:
            q = self._q_rigide(centre, self._ecart_mains, delta=delta, rotation=rot)
        return q

    def _surveiller_mains_port(self, corriger: bool = True) -> None:
        """Pendant tout le port, les mains MESURÉES (repère du torse) :
          - roulis et tangage (autour de la ligne des mains) du carton, lacet
            de la paire, depuis le début du port : rotation moyenne des
            paumes, et ligne des mains qui penche ou tourne (comme
            _rotation_estimee au retour) ; au-delà de rotation_ramener_max_deg,
            base arrêtée (penche) et mains ramenées vers la pose figée ; au-delà
            de rotation_max_deg, à reposer (la façade s'arrête et le signale) ;
          - leur dérive commune depuis la pose figée (moyenne glissante de
            fenetre_derive_s : le balancement de la marche s'y annule) :
            au-delà de derive_port_m ou derive_port_deg, ramenées vers elle
            (`corriger` : au repos, pas pendant un geste).
        Ce que ces mains ne voient pas : un carton qui tourne entre les
        paumes, elles immobiles (la caméra de tête le voit quand il est en
        entier dans l'image : _surveiller_rotation)."""
        r = self.r
        if self._port0 is None or self._port_interrompu:
            return
        angles = self._angles_mesures()
        if angles is None:
            return
        mes = self._poses_torse(angles)
        ori0, l0 = self._port0["ori0"], self._port0["ligne0"]
        rv = sum(_rotation_vecteur(mes[c][1] @ ori0[c].T) for c in self._ROULIS) / 2.0
        ligne = mes["left"][0] - mes["right"][0]

        def ecart(a: float, b: float) -> float:
            return math.remainder(a - b, 2.0 * math.pi)
        roulis = max((float(rv[0]), ecart(math.atan2(ligne[2], ligne[1]),
                                          math.atan2(l0[2], l0[1]))), key=abs)
        lacet = max((float(rv[2]), ecart(math.atan2(-ligne[0], ligne[1]),
                                         math.atan2(-l0[0], l0[1]))), key=abs)
        rot = (math.degrees(roulis), math.degrees(float(rv[1])), math.degrees(lacet))
        self.rotation_port = rot
        pire = max(abs(v) for v in rot)
        self.rotation_port_max = max(self.rotation_port_max, pire)
        texte = (f"carton tourné de {rot[0]:+.1f}° en roulis, {rot[1]:+.1f}° en tangage "
                 f"(autour de la ligne des mains) et {rot[2]:+.1f}° en lacet depuis le début "
                 f"du port, sur les mains mesurées")
        if pire > r.rotation_max_deg:
            self._interrompre_port(f"{texte} (au plus {r.rotation_max_deg:g}°)")
            return
        t = self._horloge()
        pos = (mes["left"][0] + mes["right"][0]) / 2.0
        self._hist_derive.append((t, pos - self._port0["pos0"], rv))
        while t - self._hist_derive[0][0] > r.fenetre_derive_s:
            self._hist_derive.popleft()
        dp = sum(h[1] for h in self._hist_derive) / len(self._hist_derive)
        dr = sum(h[2] for h in self._hist_derive) / len(self._hist_derive)
        self.derive_port = (float(np.linalg.norm(dp)), math.degrees(float(np.linalg.norm(dr))))
        plein = t - self._hist_derive[0][0] >= 0.9 * r.fenetre_derive_s
        alarme = pire > r.rotation_ramener_max_deg
        if alarme and not self._alarme_rotation:
            self.stats["alarmes_rotation_port"] = self.stats.get("alarmes_rotation_port", 0) + 1
            self._noter(f"port : {texte} (au plus {r.rotation_ramener_max_deg:g}°) : base "
                        f"arrêtée, mains ramenées vers leur pose figée")
        elif self._alarme_rotation and not alarme:
            self._noter(f"port : carton revenu à {pire:.1f}° : la marche peut reprendre")
            self.penche = self._t_resserrage is not None or (
                self.inclinaison is not None and self.inclinaison > r.inclinaison_max_deg)
        self._alarme_rotation = alarme
        if corriger and self._approcher_correction():
            pass                          # correction en cours : jugée une fois finie
        elif corriger and (self._t_correction is None
                           or t - self._t_correction >= r.fenetre_derive_s):
            if alarme:
                self._corriger_derive(pos - self._port0["pos0"], rv)
            elif plein and (self.derive_port[0] > r.derive_port_m
                            or self.derive_port[1] > r.derive_port_deg):
                self._corriger_derive(dp, dr)
        if alarme:
            self.penche = True

    def _corriger_derive(self, dp: np.ndarray, dr: np.ndarray) -> None:
        """Les mains mesurées ont dérivé de `dp` (m) et `dr` (vecteur
        rotation) depuis leur pose figée : la consigne commune visée est
        déplacée d'autant en sens inverse (correction totale bornée par
        correction_port_max_m et correction_port_max_deg), le serrage gardé ;
        la consigne la rejoint pas à pas (_approcher_correction)."""
        r = self.r
        t0, r0 = self._correction_visee
        t1, r1 = t0 - dp, r0 - dr
        if float(np.linalg.norm(t1)) > r.correction_port_max_m \
                or math.degrees(float(np.linalg.norm(r1))) > r.correction_port_max_deg:
            if not self._port0.get("borne"):
                self._port0["borne"] = True
                self._noter(f"port : mains dérivées de {100 * float(np.linalg.norm(dp)):.1f} cm et "
                            f"{math.degrees(float(np.linalg.norm(dr))):.1f}°, au-delà de la "
                            f"correction permise ({100 * r.correction_port_max_m:g} cm, "
                            f"{r.correction_port_max_deg:g}°) : non corrigées")
            return
        if self._q_port(correction=(t1, r1)) is None:
            return
        self._correction_visee = (t1, r1)
        self._hist_derive.clear()
        self.corrections_ce_port += 1
        self.stats["corrections_derive_port"] = self.stats.get("corrections_derive_port", 0) + 1
        self._noter(f"port : mains dérivées de {100 * float(np.linalg.norm(dp)):.1f} cm et "
                    f"{math.degrees(float(np.linalg.norm(dr))):.1f}° de leur pose figée : "
                    f"ramenées vers elle")

    def _approcher_correction(self) -> bool:
        """La consigne commune rejoint la correction visée, au pas du serrage
        (pas_serrage_max_m par pas en translation, et la rotation qui déplace
        une main d'autant) : jamais un saut de consigne, qui ferait une
        pointe de couple. Vrai tant qu'elle n'y est pas."""
        (t0, r0), (t1, r1) = self._correction_port, self._correction_visee
        dt_, dr_ = t1 - t0, r1 - r0
        nt, nr = float(np.linalg.norm(dt_)), float(np.linalg.norm(dr_))
        if nt < 1e-9 and nr < 1e-9:
            return False
        pas = self.r.pas_serrage_max_m
        pas_r = pas / max(0.05, (self._ecart_mains or 0.24) / 2.0)
        t2 = t1 if nt <= pas else t0 + dt_ * (pas / nt)
        r2 = r1 if nr <= pas_r else r0 + dr_ * (pas_r / nr)
        q = self._q_port(correction=(t2, r2))
        if q is None:
            self._correction_visee = self._correction_port     # n'y arrive pas : on s'arrête là
            return False
        self._correction_port = (t2, r2)
        self._ecrire_q(q)
        self._q_tenu = dict(q)
        fini = float(np.linalg.norm(t1 - t2)) < 1e-9 and float(np.linalg.norm(r1 - r2)) < 1e-9
        if fini:
            self._t_correction = self._horloge()
            self._hist_derive.clear()
        return not fini

    def _interrompre_port(self, raison: str) -> None:
        """Le port s'arrête pour de bon, le carton tenu : la façade arrête la
        base et le signale (penche, a_reposer)."""
        self.stats["port_interrompu"] = self.stats.get("port_interrompu", 0) + 1
        self._port_interrompu = True
        self.penche = self.a_reposer = True
        self.raison = f"{raison} : à reposer"
        self._noter(f"transport interrompu : {raison}")
        self._fin_port("interrompu")

    def _fin_port(self, issue: str) -> None:
        """Fin du port (déposé, reposé ou interrompu) : au journal, une fois,
        ce qu'il a coûté."""
        if self._port0 is None:
            return
        self._noter(f"fin du port ({issue}) : {self.resserrages_ce_port} resserrage(s), base "
                    f"arrêtée ; {self.corrections_ce_port} correction(s) de la dérive des mains ; "
                    f"carton tourné de {self.rotation_port_max:.1f}° au plus (mains mesurées)")
        self.stats["resserrages_dernier_port"] = self.resserrages_ce_port
        self._port0 = None

    def _lever_de_quelques_cm(self) -> None:
        # Avant de lever : la ligne des points de contact MESURÉS (angles
        # mesurés) passe-t-elle par le centre de gravité ? Sinon, levé, le
        # carton tournerait autour d'elle (essai GPU du 5 novembre 2026).
        angles = self._angles_mesures()
        if angles is None:
            self._echouer("prise refusée — angles des bras illisibles avant de lever")
            return
        ecart = self._ecart_cg(self._modele.mains({**self._fixes, **angles}),
                               self._plan["centre"])
        self._noter(f"ligne des mains au serrage : {100 * self.ligne_cg[0]:+.1f} cm du centre de "
                    f"gravité en profondeur, {100 * self.ligne_cg[1]:+.1f} cm en hauteur "
                    f"(au plus {100 * tolerances_cg(self.r)[0]:.1f} et "
                    f"{100 * tolerances_cg(self.r)[1]:.1f})")
        if ecart:
            self._echouer(f"prise refusée — {ecart}")
            return
        self._hist_charge.clear()
        self._poids_ref, self._t_sans_poids, self.appui_torse = None, None, None
        r = self.r
        self.serrage_vise, self.serrage_requis, self.charge = None, None, None
        self.poids_carton, self.poids_tare, self._t_attente_serrage = None, None, None
        self._bruit_pesee = None
        self._poids_retour.clear()
        self._poids_statique = None
        self._lectures_pesee, self._lecture_charge, self._angles_pesee = [], None, None
        self._releves_pesee = []
        self._vitesse_pesee, self._ecartees_pesee = 0.0, 0
        self._rigide, self._resserrage_force = None, False
        self._t_serrage_bloque, self._record_serrage = None, {}
        self._delta_max_poids = {c: self._delta_max.get(c, 0.0) + r.penetration_poids_max_m
                                 - r.penetration_max_m for c in self._ROULIS}
        mod0 = self._mains_modele()
        self._ecart_faces0 = None if mod0 is None else self._interieur(mod0)
        self.marge_frottement = self.marge_courte = None
        mg, md = self._contacts_bassin()
        # Milieu des mains mesurées au serrage : la référence des gestes avec l'objet.
        centre = self._c_serre = (mg + md) / 2.0
        self._dz_prise = self._hauteur_contacts() - float(self.objet.centre[2])
        self._delta_serre = dict(self._delta)
        self._centre_porte = centre + np.array([0.0, 0.0, self.r.hauteur_test_m])
        q = self._q_mains(self._centre_porte, self._ecart_mains)
        if q is None:
            self._echouer("prise refusée — impossible de lever l'objet (hors de portée)")
            return
        self._q_de, self._q_vers = dict(self._q), q
        self._c_de, self._c_vers = centre, self._centre_porte
        self._aller(Etat.LEVAGE)

    def _peser(self, tau: Dict[str, float]) -> Union[bool, str]:
        """Une lecture de la pesée (ReglagesPrise.lectures_pesee) : la
        dernière lecture de la charge, gardée si aucune articulation des bras
        n'a bougé de plus de vitesse_pesee_max_rad_s depuis la lecture
        d'avant. Une main qui passe sous force_min_n (le serrage qui confirme
        la prise) écourte la pesée, au moins filtre_force × 3 lectures : on ne
        reste pas au serrage de prise pendant qu'une main se relâche. True une
        fois le carton pesé (poids_carton, poids_tare, au journal), False tant
        qu'on lit, ou la raison de le reposer (pas assez de lectures bras
        immobiles en duree_pesee_max_s)."""
        r = self.r
        t = self._horloge()
        forces = self._mesurer_forces(tau)
        relache = bool(forces) and min(forces.values()) < r.force_min_n
        lecture, self._lecture_charge = self._lecture_charge, None
        angles = self._angles_mesures()
        if angles is not None:
            bras = {j: float(angles[j]) for c in self._ROULIS for j in self._BRAS_COTE[c]
                    if j in angles}
            avant, self._angles_pesee = self._angles_pesee, (t, bras)
            if lecture is not None and avant is not None and t > avant[0]:
                v = max((abs(q - avant[1][j]) for j, q in bras.items() if j in avant[1]),
                        default=0.0) / (t - avant[0])
                if v <= r.vitesse_pesee_max_rad_s:
                    self._lectures_pesee.append(lecture)
                    self._vitesse_pesee = max(self._vitesse_pesee, v)
                    if self._calibration_en_cours is not None:
                        self._releves_pesee.append((dict(tau), dict(angles),
                                                    self._gravite_bassin()))
                else:
                    self._ecartees_pesee += 1
        n, n_min = len(self._lectures_pesee), max(1, r.filtre_force * 3)
        duree = t - self._t_attente_serrage
        # Calibration de charge : la masse de référence pesée plus longtemps.
        n_vise, duree_max = ((r.lectures_calibration_charge, r.duree_calibration_charge_max_s)
                             if self._calibration_en_cours is not None
                             else (r.lectures_pesee, r.duree_pesee_max_s))
        # Bras commandés immobiles qui bougent quand même : le carton bouge
        # entre les mains (campagne de cartons variés : un carton de 1,27 kg
        # glissait de 12 mm/s au serrage de prise, l'épaule broutait). Décidé
        # dès que les lectures écartées passent les immobiles, sans le laisser
        # glisser jusqu'à duree_pesee_max_s.
        bouge = self._ecartees_pesee >= n_min and self._ecartees_pesee > n
        ecourtee = relache and n >= n_min and n < n_vise
        if n < n_vise and duree < duree_max and not ecourtee and not bouge:
            return False
        if n < n_min:
            return (f"pesée impossible : les bras ne se posent pas en haut du levage ({n} "
                    f"lecture(s) immobiles en {duree:.1f} s, au moins {n_min} ; "
                    f"{self._ecartees_pesee} à plus de {r.vitesse_pesee_max_rad_s:g} rad/s) : le "
                    f"carton bouge entre les mains (trop lourd ou trop glissant pour le serrage "
                    f"de prise, il n'a pas suivi)")
        self.poids_carton, incertitude = poids_pese([v[0] for v in self._lectures_pesee],
                                                    r.part_ecartee_pesee)
        self._bruit_pesee = incertitude * math.sqrt(n)        # par lecture, bras immobiles
        tares = [v[2] for v in self._lectures_pesee if v[2] is not None]
        self.poids_tare = poids_pese(tares, r.part_ecartee_pesee)[0] if tares else None
        self._noter(f"poids du carton : {self.poids_carton:.1f} N (moyenne tronquée de {n} "
                    f"lectures en {duree:.1f} s, bras immobiles : au plus "
                    f"{self._vitesse_pesee:.3f} rad/s"
                    + (f", {self._ecartees_pesee} écartée(s)" if self._ecartees_pesee else "")
                    + (f" ; écourtée, une main sous {r.force_min_n:g} N "
                       f"({min(forces.values()):.1f})" if ecourtee else "")
                    + f" ; à ±{2 * incertitude:.2f} N près, deux écarts types ; en haut du "
                    f"levage, avant de serrer plus)"
                    + ("" if self.poids_tare is None else
                       f" ; taré depuis la fermeture : {self.poids_tare:.1f} N")
                    + f" ; {self._etat_calibration_charge()}")
        return True

    def _etat_calibration_charge(self) -> str:
        """Pour le journal de la pesée : ce qui corrige (ou non) les gains des coudes."""
        if self._calibration_en_cours is not None:
            return "masse de référence de la calibration de charge, couples sans correction de gain"
        if self.calibration_charge is None:
            return "sans calibration de charge (gains des coudes non corrigés)"
        return (f"gains des coudes de la calibration de charge du "
                f"{str(self.calibration_charge.get('date', '?'))[:10]}")

    def _abandonner_calibration(self, raison: str) -> None:
        """Calibration de charge en cours qui n'aboutit pas : gains d'avant rendus."""
        if self._calibration_en_cours is None:
            return
        self.gains_couples, self.calibration_charge = self._calibration_en_cours["avant"]
        self._calibration_en_cours = None
        self._noter(f"calibration de charge non faite ({raison}) : gains d'avant gardés")

    def _gains_coudes(self, releves: list,
                      poids_n: float) -> Union[Dict[str, Tuple[float, float, float]], str]:
        """Gains des couples estimés des coudes d'après les relevés
        (couples, angles, pesanteur) de la pesée d'une masse connue de
        `poids_n` (N), son poids partagé moitié-moitié entre les mains
        (prise.gain_coude) : {coude : (gain, poids à cette main en N,
        incertitude du gain, deux écarts types)}, ou pourquoi c'est
        impossible. Les couples pris tels quels (gains_couples vides, comme
        pendant la calibration)."""
        par_main = {c: [] for c in self._ROULIS}
        parts = {c: [] for c in self._ROULIS}
        for tau, angles, g in releves:
            haut = -g / max(1e-9, float(np.linalg.norm(g)))
            for c in self._ROULIS:
                j = f"{c}_elbow_joint"
                f1 = self._force_main(c, tau, angles, g, depuis_fermeture=False)
                # Sans le couple du coude (ramené à son biais) : ce qu'il porte.
                f0 = self._force_main(c, {**tau, j: self.biais_couples.get(j, 0.0)}, angles, g,
                                      depuis_fermeture=False) if j in tau else None
                if f1 is None or f0 is None:
                    return f"couple du coude {'gauche' if c == 'left' else 'droit'} illisible"
                par_main[c].append(float(f1 @ haut))
                parts[c].append(float((f1 - f0) @ haut))
        sortie = {}
        for c in self._ROULIS:
            w, part = float(np.mean(par_main[c])), float(np.mean(parts[c]))
            k = gain_coude(w, part, poids_n)
            # Incertitude : celle du poids à cette main (écart type sur la
            # racine du nombre de lectures), ramenée au gain (dk = k² dw / part).
            dw = float(np.std(par_main[c])) / math.sqrt(len(par_main[c]))
            sortie[f"{c}_elbow_joint"] = (k, w, 2.0 * k * k * dw / part if part > 1e-6
                                          else math.nan)
        return sortie

    def _finir_calibration_charge(self) -> None:
        """La masse de référence pesée (calibrer_charge) : le gain du couple
        estimé de chaque coude (prise.gain_coude, son poids partagé
        moitié-moitié entre les mains), vérifié, appliqué, passé à
        `enregistrer` ; puis la masse est reposée là où elle a été prise."""
        r, cal = self.r, self._calibration_en_cours
        poids, releves = 9.81 * cal["masse_kg"], self._releves_pesee
        if len(releves) < r.lectures_pesee:
            self._reposer(f"calibration de charge impossible : {len(releves)} lectures bras "
                          f"immobiles, au moins {r.lectures_pesee}")
            return
        mesure = self._gains_coudes(releves, poids)
        if isinstance(mesure, str):
            self._reposer(f"calibration de charge impossible : {mesure}")
            return
        gains = {j: v[0] for j, v in mesure.items()}
        details = [f"coude {'gauche' if j.startswith('left') else 'droit'} {k:.3f} à ±{dk:.3f} près "
                   f"({w:.2f} N à cette main pour {poids / 2:.2f})"
                   for j, (k, w, dk) in sorted(mesure.items())]
        hors = [f"{j.split('_')[0]} {k:.3f}" for j, k in gains.items()
                if not r.gain_coude_min <= k <= r.gain_coude_max]
        if hors:
            self._reposer(f"calibration de charge refusée : gain(s) du coude {', '.join(hors)} "
                          f"hors de [{r.gain_coude_min:g}, {r.gain_coude_max:g}] — masse de "
                          f"référence décentrée entre les mains, ou mal déclarée "
                          f"({cal['masse_kg']:g} kg) ? ; gains d'avant gardés")
            return
        fiche = {"date": datetime.now(timezone.utc).isoformat(timespec="seconds"),
                 "masse_kg": cal["masse_kg"],
                 "gains_couples": {j: round(k, 4) for j, k in gains.items()},
                 "lectures": len(releves), "poids_pese_n": round(float(self.poids_carton), 3),
                 "poids_par_main_n": {j.split("_")[0]: round(v[1], 3) for j, v in mesure.items()}}
        self.gains_couples, self.calibration_charge = dict(gains), fiche
        self._calibration_en_cours = None
        enregistree = "enregistrée au profil du robot"
        if cal["enregistrer"] is not None:
            try:
                cal["enregistrer"](fiche)
            except Exception as e:        # noqa: BLE001 — appliquée quand même : on le dit
                enregistree = f"NON ENREGISTRÉE au profil du robot ({e}) : à refaire au redémarrage"
        else:
            enregistree = "appliquée, pas enregistrée (aucun profil donné)"
        self.stats["calibration_charge"] = fiche["date"]
        self._noter(f"calibration de charge du {fiche['date']} : {cal['masse_kg']:g} kg de "
                    f"référence, {len(releves)} lectures bras immobiles, {poids:.2f} N partagés "
                    f"moitié-moitié entre les mains ; gains des couples estimés : "
                    + " ; ".join(details) + f" ; {enregistree} ; masse reposée")
        self._calibration_finie = True
        self._redescendre()

    def _pas_levage_mesure(self) -> None:
        r, p = self.r, self.p
        tau, raison = self._efforts()
        if tau is None:
            self._echouer(f"levage interrompu — {raison}")
            return
        q, fini = self._interp_c(r.duree_levage_test_s)
        self._ecrire_q(q)
        tourne = self._surveiller_rotation(tau)
        if tourne:
            self._reposer(tourne)
            return
        if not fini:
            return
        if self._t_attente_serrage is not None:
            # Prise confirmée, en haut : on pèse le carton, bras immobiles,
            # sur lectures_pesee lectures depuis la confirmation (_peser,
            # moyenne tronquée), puis on serre d'après ce poids, avant de
            # ramener ; au plus duree_serrage_poids_max_s bloqué.
            if self.poids_carton is None and self.charge is not None:
                pesee = self._peser(tau)
                if isinstance(pesee, str):
                    self._reposer(pesee)
                    return
                if not pesee:
                    return
                pese = self.poids_carton if self.poids_tare is None else self.poids_tare
                if min(self.poids_carton, pese) < r.poids_min_n:
                    # Il ne pèse pas sur les mains : resté sur la table, elles ont
                    # glissé le long de ses faces (ReglagesPrise.poids_min_n). La
                    # plus petite des deux pesées : le biais des couples faisait
                    # lire 1,7 N à un carton de 1,4 kg resté sur la table, bras
                    # tendus (0,0 N taré) ; serré pour 1,7 N, il glissait au retour.
                    self._reposer(f"le carton ne pèse pas sur les mains "
                                  f"({min(self.poids_carton, pese):.1f} N, au moins "
                                  f"{r.poids_min_n:g}) : il n'a pas quitté la table, les mains "
                                  f"ont glissé le long de ses faces (trop lourd ou trop glissant "
                                  f"pour le serrage de prise)")
                    return
                if self._poids_ref is not None and \
                        self.poids_carton < r.part_poids_min * self._poids_ref:
                    # Pesé bien moins lourd que porté pendant le levage : il
                    # retombe sur la table entre les mains (la règle du port,
                    # part_poids_min, dès la pesée). Constaté sur la campagne de
                    # cartons variés : 2,0 N pesés pour 9,5 portés au levage
                    # (8,4 N vrais, frottement 0,65) ; serré pour 2 N, il a fini
                    # par glisser entre les mains.
                    self._reposer(f"le carton ne pèse plus sur les mains que "
                                  f"{self.poids_carton:.1f} N, {self._poids_ref:.1f} au levage "
                                  f"(au moins {r.part_poids_min:.0%}) : il retombe entre elles "
                                  f"(trop lourd ou trop glissant pour le serrage de prise, il "
                                  f"n'a pas suivi)")
                    return
                if pese > 9.81 * r.masse_max_kg:
                    # Au-delà de la charge pour laquelle la prise est vérifiée
                    # (ReglagesPrise.masse_max_kg) : reposé aussitôt pesé. Taré :
                    # sans calibration des couples, leur biais faisait peser un
                    # carton de 1 kg de 0,77 à 1,23 kg (0,92 à 1,05 taré).
                    self._reposer(f"trop lourd : {pese / 9.81:.2f} kg pesés, au plus "
                                  f"{r.masse_max_kg:g} kg (masse_max_kg, la charge pour laquelle la "
                                  f"prise est vérifiée) ; ni serré plus, ni porté")
                    return
                if self._calibration_en_cours is not None:
                    # Calibration de charge : la masse de référence pesée, on la
                    # repose, ni serrée plus, ni portée.
                    self._finir_calibration_charge()
                    return
            serre = self._serrer_selon_poids(tau)
            if isinstance(serre, str):
                self._reposer(serre)
                return
            if serre is True:
                if self.serrage_requis is not None:
                    self._noter(
                        f"serrage d'après le poids : G {self.force_serrage['left']:.1f} N, D "
                        f"{self.force_serrage['right']:.1f} N par main pour "
                        f"{self.poids_carton:.1f} N de poids (requis "
                        f"{self.serrage_requis:.1f}, visé {self.serrage_vise:.1f})")
                self.stats["prises"] += 1
                self._lever_exemption()
                self._ramener()
            elif self._t_serrage_bloque is not None and \
                    self._horloge() - self._t_serrage_bloque >= r.duree_serrage_poids_max_s:
                self._reposer(self._trop_lourd())
            return
        forces = self._mesurer_forces(tau)
        if self.couples_fiables is False:
            # Force estimée non fiable : mains toujours retenues par l'objet.
            retards, _ = self._retenues()
            tient = bool(retards) and all(v[2] for v in retards.values())
        else:
            tient = forces is not None and min(forces.values()) >= r.force_min_n
        confirme = self._verifier() and tient
        if not confirme and self._horloge() - self._t0 - r.duree_levage_test_s \
                < p.attente_confirmation_s:
            return
        if confirme:
            self._t_attente_serrage = self._horloge()
            self._hist_charge.clear()
            # La pesée ne compte que les lectures qui suivent la confirmation.
            self._lectures_pesee, self._lecture_charge, self._angles_pesee = [], None, None
            self._releves_pesee = []
            self._vitesse_pesee, self._ecartees_pesee = 0.0, 0
            return
        self._essai += 1
        if self._essai >= r.essais_max:
            serre = min((self.force_serrage or {}).values(), default=0.0)
            self._echouer(f"prise NON confirmée — l'objet n'a pas suivi ({self._essai} essais) : il "
                          f"glisse entre les mains serrées à {serre:.1f} N, trop lourd ou trop "
                          f"glissant pour ce serrage")
            return
        # Reposer, desserrer, recommencer : deux phases, puis le placement.
        self.stats["reprises"] = self.stats.get("reprises", 0) + 1
        self._q_de, self._q_vers, self._sous = dict(self._q), dict(self._q_serre), "descendre"
        self._aller(Etat.REPOSE)

    def _serrer_selon_poids(self, tau: Dict[str, float]):
        """En haut du levage : le serrage requis d'après le poids pesé
        (poids_carton), securite_serrage × poids / (2 frottement_serrage), visé avec
        marge_serrage_n de plus (ReglagesPrise), monté peu à peu, la face
        commandée jusqu'à `penetration_poids_max_m` dans la face touchée,
        seulement si aucune articulation des deux bras n'est à moins de 0,3
        N·m de son plafond (plafonds inchangés). True : visé atteint, ou
        requis atteint et plus moyen d'avancer ; None : en cours ; une raison
        (str) : le carton cède sous le serrage. Depuis quand plus
        le serrage ne progresse plus (aucune main plus loin qu'elle n'est
        déjà allée : une avance que le garde-plafond défait aussitôt, couples
        bruités, n'en est pas une) : _t_serrage_bloque. Sans poids pesé
        (couples non fiables) : le serrage d'avant, True."""
        r = self.r
        if self.couples_fiables is False or self.poids_carton is None:
            return True
        mod = self._mains_modele()
        if mod is not None and self._ecart_faces0 is not None \
                and self._ecart_faces0 - self._interieur(mod) > r.cede_max_m:
            return (f"le carton cède sous le serrage (mains rapprochées de "
                    f"{100 * (self._ecart_faces0 - self._interieur(mod)):.1f} cm, au plus "
                    f"{100 * r.cede_max_m:.0f})")
        poids = max(0.0, self.poids_carton)
        self.serrage_requis = r.securite_serrage * poids / (2.0 * r.frottement_serrage)
        self.serrage_vise = self.serrage_requis + r.marge_serrage_n
        forces = self._mesurer_forces(tau)
        if forces is None:
            return None
        manque = [c for c in self._ROULIS if forces[c] < self.serrage_vise]
        avance = False
        if self._marge_plafonds(tau):
            for cote in manque:
                if self._delta[cote] + r.pas_serrage_max_m <= self._delta_max_poids[cote]:
                    self._delta[cote] += r.pas_serrage_max_m
                    self._garder_borne(cote)
                    avance = True
        progres = False
        for cote in self._ROULIS:
            if self._delta[cote] > self._record_serrage.get(cote, -math.inf) + 1e-9:
                self._record_serrage[cote] = self._delta[cote]
                progres = progres or avance
        if not manque or progres:
            self._t_serrage_bloque = None
        elif self._t_serrage_bloque is None:
            self._t_serrage_bloque = self._horloge()
        # Visé : le requis plus une marge ; bloqué au-dessus du requis, il suffit.
        suffit = not avance and min(forces.values()) >= self.serrage_requis
        return True if not manque or suffit else None

    def _garder_borne(self, cote: str) -> None:
        """La borne du serrage (_delta_max, celle que la correction
        d'inclinaison ne dépasse pas) suit le serrage d'après le poids : sans
        quoi cette correction, plafonnée à l'ancienne borne, desserrerait."""
        self._delta_max[cote] = max(self._delta_max.get(cote, 0.0), self._delta[cote])

    def _marge_plafonds(self, tau: Dict[str, float]) -> bool:
        """Serrer plus ? Seulement si AUCUNE articulation des deux bras n'est
        à moins de 0,3 N·m de son plafond (_limite_effort, inchangé) : le
        serrage d'une main charge aussi l'autre, à travers l'objet."""
        return all(abs(tau[j]) <= self._limite_effort(j) - 0.3
                   for c in self._ROULIS for j in self._BRAS_COTE[c] if j in tau)

    def _trop_lourd(self) -> str:
        """Pourquoi le serrage visé d'après le poids n'est pas atteint."""
        r = self.r
        poids = self.poids_carton or 0.0
        obtenus = ", ".join(f"main {'G' if c == 'left' else 'D'} "
                            f"{self.force_serrage.get(c, 0.0):.1f} N" for c in self._ROULIS)
        return (f"trop lourd pour ces mains (serrage insuffisant dans leurs plafonds) : il "
                f"faudrait {self.serrage_requis:.1f} N par main pour {poids:.1f} N de poids "
                f"(frottement {r.frottement_serrage:.1f}, sécurité {r.securite_serrage:g}) ; "
                f"obtenus : {obtenus}, plafonds de couple et borne de "
                f"{100 * r.penetration_poids_max_m:.0f} cm atteints")

    def _reposer(self, raison: str) -> None:
        """L'objet tenu tourne : on ne marche pas avec, on le repose
        doucement là où il a été pris (le milieu des mains au serrage, en
        ligne droite, serrage gardé), on desserre, puis la prise échoue avec
        la raison (l'app recommence, ou signale)."""
        self.stats["reposes_rotation"] = self.stats.get("reposes_rotation", 0) + 1
        self._raison_repose = raison
        self._noter(f"objet reposé : {raison}")
        self._redescendre()

    def _redescendre(self) -> None:
        """Le geste de _reposer : l'objet redescend là où il a été pris, en
        ligne droite, serrage gardé ; puis les mains se desserrent (REPOSE)."""
        self._fin_port("reposé")
        depart = self._c_actuel if self._c_actuel is not None else self._c_serre
        self._c_de, self._c_vers = np.array(depart, dtype=float), np.array(self._c_serre)
        self._q_de, self._q_vers, self._sous = dict(self._q), dict(self._q_serre), "redescendre"
        self._aller(Etat.REPOSE)

    def _pas_repose(self) -> None:
        r, p = self.r, self.p
        if self._sous == "redescendre":
            q, fini = self._interp_c(r.duree_levage_test_s)
        else:
            duree = r.duree_levage_test_s if self._sous == "descendre" else p.duree_ouverture_s
            q, fini = self._interp_q(duree)
        self._ecrire_q(q)
        if not fini:
            return
        if self._sous in ("descendre", "redescendre"):
            self._q_de, self._q_vers, self._sous = dict(self._q), dict(self._plan["q_ouvert"]), \
                "desserrer"
            self._aller(Etat.REPOSE)
            return
        if self._raison_repose:
            raison, self._raison_repose = self._raison_repose, ""
            self._echouer(f"prise refusée — {raison} : carton reposé sur la table")
            return
        if self._calibration_finie:
            # Masse de référence reposée, mains desserrées : comme après un
            # dépôt, gains normaux, puis retour progressif à la posture de port.
            self._calibration_finie = False
            self.objet, self._q_tenu = None, None
            self._lever_exemption()
            self._regler_raideur(False)
            self._q_de = dict(self._q)
            self._q_vers = {**self._libres0, **posture_bras(p.roulis_ouvert, p.tangage_port, p.coude)}
            self._aller(Etat.LACHER)
            return
        self._corrections, self._delta = 0, {"left": 0.0, "right": 0.0}
        self._vers_placement(self._plan.get("q_place", self._plan["q_ouvert"]), 0.0)

    def _ramener(self, plus_pres: float = 0.0) -> None:
        """Vers le buste : la face arrière de l'objet à `distance_torse_m`
        du bassin, un peu plus haut ; au plus près que les bras permettent."""
        r = self.r
        prof = self.objet.profondeur if self.objet is not None else 0.2
        depart = self._centre_porte
        cible = None
        # Le milieu des points de contact : le centre de l'objet ; la face
        # arrière visée `appui_torse_m` au-delà du buste attendu, l'arrêt se
        # fait au toucher (_pas_ramener). Avant-bras toujours au-dessus du
        # plateau qu'ils survolent encore.
        plateau = None
        if self._plan is not None and self.objet is not None:
            plateau = float(self._plan["centre"][2]) - self.objet.hauteur / 2.0
        # Les deux mains, comme un seul corps rigide avec l'objet : leurs
        # poses commandées au départ, la direction du serrage, l'écart.
        depart = np.array(depart, dtype=float)
        poses = self._poses(self._q)
        u = poses["right"][0] - poses["left"][0]
        u = u / max(1e-9, float(np.linalg.norm(u)))
        # La pose entière d'une main (position et orientation) demande six
        # articulations commandées au moins (le tangage du poignet).
        tenable = all(len(self._commandables(c)) >= 6 for c in self._ROULIS)
        self._rigide = (depart, {"left": (*poses["left"], u), "right": (*poses["right"], -u)},
                        dict(self._delta), self._ecart_mains) if tenable else None
        for k in range(12 if tenable else 0):
            x = max(r.distance_torse_m - r.appui_torse_m - plus_pres + prof / 2.0
                    + 0.02 * k, 0.05)
            if x >= float(depart[0]):
                break
            c = np.array([x, float(depart[1]), float(depart[2]) + r.hausse_transport_m])
            q = self._chemin_rigide(depart, c, plateau)
            if q is not None and self._redressable(c, q):
                cible = (c, q)
                break
        if cible is None:
            # Pas plus près sans tordre l'objet : au moins plus haut ; sinon,
            # porté là où le levage l'a laissé. Torse penché, jamais plus haut :
            # bras au bout de leur portée, les mains y lâchaient le carton
            # (robot simulé : table de 70 cm, torse à 5,7°, « mains qui ne
            # serrent plus » pendant le retour, sans que l'OS le voie à temps).
            penche = float(self._articulations_fixes().get(self.TAILLE, 0.0)) > 1e-3
            c = depart + np.array([0.0, 0.0, r.hausse_transport_m])
            q = self._chemin_rigide(depart, c, plateau) if tenable and not penche else None
            cible = (c, q) if q is not None else (depart, dict(self._q))
            pourquoi = ("aucun chemin plus près du buste où les mains restent un seul corps "
                        f"rigide avec le carton (orientation à {r.tol_rigide_deg:g}° près, sans "
                        "saut, avant-bras au-dessus du plateau)" if tenable else
                        f"{min(len(self._commandables(c_)) for c_ in self._ROULIS)} articulations "
                        "commandées par bras : la pose d'une main ne se tient pas (6 au moins, "
                        "tangage du poignet compris)")
            self._noter(f"retour : {pourquoi} : " + ("seulement plus haut" if q is not None else
                                                     "porté là où le levage l'a laissé"))
        self._c_de, self._c_vers = np.array(depart, dtype=float), cible[0]
        self._centre_porte = cible[0]
        self._q_de, self._q_vers = dict(self._q), cible[1]
        # Le carton ramené peut-il toucher le torse ? Sinon, pas de traction
        # vers le robot (essai GPU du 5 novembre 2026, 21 h 57 : « contre le
        # torse » à 3 N, alors que les bras ne l'y amenaient pas) : il est
        # rapproché en position, et rien ne s'arrête sur un effort.
        self._torse_possible = float(cible[0][0]) - prof / 2.0 <= r.distance_torse_m
        # Au port qui suit : serrage de référence repris à l'arrivée.
        self._serrage_port_ref, self._delta_port, self._t_resserrage = None, None, None
        self._resserrage_marge = False
        self._aller(Etat.RAMENER)

    def _pas_ramener(self) -> None:
        tau, _ = self._efforts()
        derniere = self._hist_charge[-1] if self._hist_charge else None
        tourne = self._surveiller_rotation(tau)
        if tourne:
            self._reposer(tourne)
            return
        if self._hist_charge and self._hist_charge[-1] is not derniere:
            # Base arrêtée (on ne marche qu'au port), bras qui arrivent en
            # posture de port : les fin_retour_s dernières lectures du poids.
            t_ = self._horloge()
            self._poids_retour.append((t_, self._hist_charge[-1][0]))
            while t_ - self._poids_retour[0][0] > self.r.fin_retour_s + 1e-9:
                self._poids_retour.popleft()
        q, fini = self._interp_c(self.r.duree_ramener_s)
        appui = self._appui_torse()
        # En corps rigide, les paumes commandées ne tournent pas : une rotation
        # mesurée est anormale. Sans lui (bras à moins de six articulations),
        # seule une remontée après une dépose manquée bouge ici, comme avant.
        self.rotation_carton = self._rotation_estimee() if self._rigide is not None else None
        tourne = self.rotation_carton is not None and \
            max(abs(v) for v in self.rotation_carton) > self.r.rotation_ramener_max_deg
        if tourne:
            # L'objet tourne entre les mains : on arrête le geste là, et l'on
            # resserre au port (_garder_serrage) avant de marcher.
            self.stats["retours_arretes"] = self.stats.get("retours_arretes", 0) + 1
            self._noter(f"retour arrêté : carton tourné de {self.rotation_carton[0]:+.1f}° "
                        f"en roulis (autour de l'axe avant) et {self.rotation_carton[1]:+.1f}° "
                        f"en tangage (autour de la ligne des mains), estimés sur les mains "
                        f"mesurées (au plus {self.r.rotation_ramener_max_deg:g}°) : on resserre")
            self._resserrage_force = True
            fini = True
        elif appui and self.appui_torse is None:
            # Contre le torse : on s'arrête là (troisième appui), sans
            # pousser plus.
            self.appui_torse = self.charge[1] if self.charge else 0.0
            self._noter(f"carton contre le torse : {appui}")
            fini = True
        else:
            self._ecrire_q(q)
        if fini:
            # Là où les mains sont arrivées : la référence des gestes qui
            # suivent (serrage au port, dépose), le geste fini ou arrêté.
            c = self._c_actuel if self._c_actuel is not None else self._centre_porte
            self._centre_porte = np.array(c, dtype=float)
            if self.appui_torse is None and not tourne:
                prof = self.objet.profondeur if self.objet is not None else 0.2
                self._noter(f"carton ramené sans toucher le torse : face arrière à "
                            f"{100 * (float(c[0]) - prof / 2.0):.0f} cm du bassin, buste attendu "
                            f"à {100 * self.r.distance_torse_m:.0f} ; plus près, les bras n'y "
                            f"arrivent pas sans changer de posture en chemin : rapproché en "
                            f"position, sans effort vers le robot, tenu par les mains seules"
                            + ("" if self._torse_possible else " (torse hors d'atteinte)"))
            self._q_tenu = dict(self._q)
            self._centre_transport = np.array(self._c_actuel if self._c_actuel is not None
                                              else self._centre_porte, dtype=float)
            self._t_sans_poids = None
            lus = [w for _, w in self._poids_retour]
            if len(lus) >= max(1, self.r.filtre_force * 3):
                self._poids_statique = float(np.median(lus))
                self._noter(f"poids lu base arrêtée, à la fin du retour : "
                            f"{self._poids_statique:.1f} N (médiane de {len(lus)} lectures en "
                            f"{self.r.fin_retour_s:g} s)"
                            + (f", pesé {self.poids_carton:.1f}" if self.poids_carton else ""))
            self.etat = Etat.TENU

    def _appui_torse(self) -> str:
        """Le carton ramené touche-t-il le torse ? Les mains le tirent vers
        le robot et le torse le repousse : la force des mains vers le robot
        passe `force_torse_n` (couples fiables) ; ou les mains, retenues,
        sont en retard de `retard_torse_m` sur leur consigne. "" sinon."""
        r = self.r
        if not self._torse_possible:
            return ""
        if self.charge is not None and self.charge[1] >= r.force_torse_n:
            return (f"mains qui tirent de {self.charge[1]:.1f} N vers le robot (au moins "
                    f"{r.force_torse_n:.0f})")
        retard = self._retard_vers_robot()
        if retard is not None and retard >= r.retard_torse_m:
            return f"mains retenues à {100 * retard:.1f} cm de leur consigne vers le robot"
        return ""

    def _serrage_port(self, angles: Dict[str, float]) -> float:
        """Au port, le serrage vu par la géométrie (m) : de combien les mains
        mesurées sont, à elles deux, retenues par l'objet en dehors des mains
        commandées, le long de la normale. La taille, que la marche fait
        tourner, est prise mesurée des deux côtés. Le balancement de
        l'ensemble mains et carton (essai GPU du 5 novembre 2026, 21 h 57 :
        ±2,5 cm, faces mesurées G et D décalées ensemble, leur écart constant
        à 2 mm près) retient autant d'une main qu'il en rend à l'autre : il
        s'annule dans la somme ; une main qui se desserre, ou une consigne
        desserrée, la fait baisser."""
        taille = {j: v for j, v in angles.items() if j.startswith("waist_")}
        cmd = self._faces(self._modele.mains({**self._fixes, **taille, **self._q}))
        mes = self._faces(self._modele.mains({**self._fixes, **angles}))
        return (mes[0] - cmd[0]) + (cmd[1] - mes[1])

    def _garder_serrage(self) -> None:
        """Au port, les mains gardent leur serrage (ReglagesPrise, port) : la
        consigne reste celle du serrage ; si les mains s'écartent de plus de
        `ecart_face_port_m` de leur place au serrage, ou si la marge de
        frottement μ (N_G + N_D) / poids passe sous `securite_serrage`, la
        base s'arrête (`penche`) et l'on resserre, bras sous leurs plafonds
        (inchangés), consigne jusqu'à `penetration_poids_max_m` ; l'écart, à
        `hysteresis_ecart_m` sous ce seuil, la marge au-dessus de
        `securite_reprise` ; sans retour en `duree_resserrage_s`, on s'arrête
        pour de bon : à reposer.
        Constaté sur GPU (essai T1 du 5 novembre 2026, 21 h 57) : une pointe
        de couple du poignet, à la marche, avait desserré la consigne de la
        main droite de 5 mm (_garder_sous_la_limite), sans jamais la
        resserrer ; et le serrage, fait pour 8 N, laissait une marge de
        frottement de 0,9 : le carton a glissé."""
        r = self.r
        if self.etat is not Etat.TENU or self._port_interrompu or self.couples_fiables is False:
            return
        angles = self._angles_mesures()
        tau, _ = self._efforts()
        if angles is None or tau is None:
            return
        serrage = self._serrage_port(angles)
        if self._serrage_port_ref is None:
            self._serrage_port_ref, self._delta_port = serrage, dict(self._delta)
            self._hist_normales.clear()
            self._t_normales.clear()
            self._poids_port.clear()
            self._ecartements.clear()
            self._ecart_debut = None
            return
        ecart = self._serrage_port_ref - serrage
        g = self._gravite_bassin()
        forces = [self._force_main(c, tau, angles, g, depuis_fermeture=False)
                  for c in self._ROULIS]
        if all(f is not None for f in forces):
            t_ = self._horloge()
            self._hist_normales.append(sum(float(f @ self._vers_objet(c))
                                           for c, f in zip(self._ROULIS, forces)))
            self._t_normales.append(t_)
            while t_ - self._t_normales[0] > r.fenetre_marge_s + 1e-9:
                self._t_normales.popleft()
                self._hist_normales.popleft()
            haut = -g / max(1e-9, float(np.linalg.norm(g)))
            self._poids_port.append((t_, sum(float(f @ haut) for f in forces)))
            while t_ - self._poids_port[0][0] > r.fenetre_marge_s + 1e-9:
                self._poids_port.popleft()
        # Marge sur la dernière foulée (dès filtre_force × 3 lectures), et sur
        # les filtre_force × 3 dernières lectures (0,3 s) : le plancher. Son
        # poids, jamais lu en marchant (securite_serrage couvre déjà les
        # charges de la marche) : avec une calibration de charge valide, la
        # pesée seule ; sans, le plus grand de la pesée et du poids lu base
        # arrêtée à la fin du retour (essai GPU du 20 novembre 2026, 21 h 37 :
        # pesé 8,4 N pour 9,8 ; une pesée trop légère rendait la marge
        # optimiste). Sans retour : la médiane du poids lu sur la foulée.
        n_court = max(1, r.filtre_force * 3)
        pese = self.poids_carton or self._poids_ref
        if self.poids_carton and self.calibration_charge is not None:
            poids = pese
            sur_poids = f"sur les {poids:.1f} N pesés, calibration de charge"
        elif self._poids_statique is not None:
            poids = max(pese, self._poids_statique) if pese else self._poids_statique
            sur_poids = (f"sur les {poids:.1f} N lus base arrêtée à la fin du retour, plus que "
                         f"les {pese:.1f} pesés" if not pese or poids > pese else
                         f"sur les {poids:.1f} N pesés")
        else:
            lu = float(np.median([w for _, w in self._poids_port])) \
                if len(self._poids_port) >= n_court else None
            poids = max(pese, lu) if pese and lu is not None else pese
            sur_poids = (f"sur les {poids:.1f} N lus sur la foulée, plus que les "
                         f"{pese:.1f} pesés" if pese and poids > pese else
                         f"sur les {poids:.1f} N pesés" if pese else "")
        if not self.poids_carton and poids:
            sur_poids = f"pas de pesée : sur le plus grand poids lu, {poids:.1f} N"
        plein = len(self._hist_normales) >= n_court
        juge = bool(poids and poids > 0.5 and plein)
        self.marge_frottement = (r.frottement_serrage * float(np.mean(self._hist_normales))
                                 / poids if juge else None)
        self.marge_courte = (r.frottement_serrage
                             * float(np.mean(list(self._hist_normales)[-n_court:])) / poids
                             if juge else None)
        # Hystérésis : la base s'arrête sous securite_serrage ; arrêtée pour la
        # marge, elle ne repart qu'au-dessus de securite_reprise (jamais moins).
        seuil = max(r.securite_reprise, r.securite_serrage) if self._resserrage_marge \
            else r.securite_serrage
        glisse = self.marge_courte is not None and self.marge_courte < r.securite_glissement
        # Base arrêtée (resserrage) : la marche ne fait plus varier le serrage,
        # la reprise se juge sur 0,3 s ; sur la foulée, elle ne verrait l'effet
        # du resserrage qu'au bout de 1 s, l'échéance de duree_resserrage_s.
        arretee = self._t_resserrage is not None
        marge = self.marge_courte if arretee else self.marge_frottement
        sur = "sur 0,3 s, base arrêtée" if arretee else "sur la dernière foulée"
        faible = glisse or (marge is not None and marge < seuil)
        # Hystérésis des écartements, comme celle de la marge : arrêtée pour un
        # écartement (au-delà de ecart_face_port_m), la base ne repart qu'une
        # fois le resserrage revenu sous ecart_face_port_m - hysteresis_ecart_m.
        bas_ecart = r.ecart_face_port_m - r.hysteresis_ecart_m
        ecartees = ecart > r.ecart_face_port_m or (
            self._t_resserrage is not None and self._ecart_debut is not None
            and ecart >= bas_ecart)
        # Carton qui a tourné au retour (_pas_ramener) : on resserre jusqu'au
        # serrage visé d'après le poids, et d'au moins marge_serrage_n à
        # chaque main, tant que les bras le permettent.
        if self._resserrage_force is True and plein:
            # Demandé au retour (True) : la cible (somme des deux forces, N).
            self._resserrage_force = False if self.serrage_vise is None else max(
                2.0 * self.serrage_vise,
                float(np.mean(self._hist_normales)) + 2.0 * r.marge_serrage_n)
        cible = self._resserrage_force if isinstance(self._resserrage_force, float) else None
        force = cible is not None and float(np.mean(self._hist_normales)) < cible
        if cible is not None and not force:
            self._resserrage_force = False
        # La consigne garde le serrage : ce qu'un plafond a desserré (pointe
        # de couple à la marche) est rendu dès que les deux bras ont de la
        # marge ; au-delà, seulement pour retrouver le serrage perdu.
        a_rendre = [c for c in self._ROULIS if self._delta[c] < self._delta_port[c] - 1e-6]
        # Un carton qui tourne entre les paumes les écarte, encore et encore
        # (essai GPU du 20 novembre 2026, 21 h 37 : 8 fois, de 5 à 13 mm,
        # chaque fois resserrées, jusqu'à 50°). Un écartement : les mains qui
        # passent au-delà de ecart_face_port_m ; il ne finit, et le suivant ne
        # peut compter, que sous bas_ecart : un écart qui oscille autour du
        # seuil (bruit de mesure) n'en fait qu'un.
        if ecart < bas_ecart:
            self._ecart_debut = None
        elif self._ecart_debut is None and ecart > r.ecart_face_port_m:
            t0 = self._horloge()
            self._ecart_debut = ecart
            self._ecartements.append(t0)
            while t0 - self._ecartements[0] > r.fenetre_ecartements_s:
                self._ecartements.popleft()
            if len(self._ecartements) >= r.ecartements_max:
                self._interrompre_port(
                    f"carton qui tourne entre les paumes : mains écartées "
                    f"{len(self._ecartements)} fois en {r.fenetre_ecartements_s:g} s "
                    f"({1000 * ecart:.0f} mm cette fois)")
                return
        elif self._ecart_debut is None:
            pass                                # sous le seuil, aucun écartement en cours
        elif self._horloge() - self._ecartements[-1] <= r.delai_croissance_s:
            # Le temps que le resserrage agisse : l'écart de référence suit
            # le plus grand vu (la main glisse encore, la consigne la suit).
            self._ecart_debut = max(self._ecart_debut, ecart)
        elif ecart > self._ecart_debut + r.croissance_ecart_m and self._t_resserrage is not None:
            self._interrompre_port(
                f"carton qui tourne entre les paumes : mains écartées de "
                f"{1000 * self._ecart_debut:.0f} puis {1000 * ecart:.0f} mm malgré le "
                f"resserrage")
            return
        if ecartees or faible or force:
            t = self._horloge()
            pourquoi = " ; ".join(
                ([f"mains écartées de {1000 * ecart:.0f} mm de leur serrage (au plus "
                  f"{1000 * r.ecart_face_port_m:g}, sous {1000 * bas_ecart:g} pour repartir)"]
                 if ecartees else [])
                + ([f"marge de frottement sur 0,3 s {self.marge_courte:.2f} (au moins "
                    f"{r.securite_glissement:g} : le serrage ne porte plus le carton au "
                    f"frottement prudent, un vrai desserrage)"] if glisse else [])
                + ([f"marge de frottement {marge:.2f} {sur} "
                    f"(au moins {r.securite_serrage:g}" + (f", {seuil:g} pour repartir"
                                                           if seuil > r.securite_serrage else "")
                    + f" ; {sur_poids})"]
                   if faible and not glisse else [])
                + ([f"carton tourné au retour, serrage visé {cible / 2.0:.1f} N par main"]
                   if force else []))
            if self._t_resserrage is None:
                self._t_resserrage, self._t_bloque = t, None
                self.stats["resserrages_port"] = self.stats.get("resserrages_port", 0) + 1
                self.resserrages_ce_port += 1
                self._noter(f"port : {pourquoi} : base arrêtée, on resserre")
            self._resserrage_marge = self._resserrage_marge or faible
            self.penche = True
            # Les bras ne peuvent serrer plus (une articulation à moins de 0,3 N·m
            # de son plafond, ou une main à sa borne) : securite_reprise est hors
            # de leur portée. La base repart dès que la marge tient
            # securite_serrage, jamais en dessous (comme avant l'hystérésis : à
            # leurs plafonds, la marge oscille autour de ce qu'ils tiennent).
            bloques = not self._marge_plafonds(tau) or any(
                self._delta[c] + r.pas_serrage_max_m
                > self._delta_max_poids.get(c, r.penetration_poids_max_m)
                for c in self._ROULIS)
            if bloques:
                self._t_bloque = t
            # De même pour les écartements : à leurs plafonds, la base repart
            # l'écart revenu sous ecart_face_port_m (comme avant l'hystérésis) ;
            # l'écartement, lui, reste en cours.
            par_marge = self._resserrage_marge and marge is not None \
                and marge >= r.securite_serrage
            if r.reprise_aux_plafonds and bloques and ecart <= r.ecart_face_port_m \
                    and not force and not glisse and (par_marge or ecartees and not faible):
                self._t_resserrage, self._resserrage_marge = None, False
                self._repartir_foulee()
                self.penche = self.inclinaison is not None \
                    and self.inclinaison > r.inclinaison_max_deg
                self.stats["reprises_aux_plafonds"] = \
                    self.stats.get("reprises_aux_plafonds", 0) + 1
                self._noter("port : serrage retrouvé (" + (
                    f"marge de frottement {marge:.2f}, {sur}, au moins "
                    f"{r.securite_serrage:g} ; {seuil:g}" if par_marge else
                    f"mains à {1000 * ecart:.0f} mm de leur serrage, au plus "
                    f"{1000 * r.ecart_face_port_m:g} ; sous {1000 * bas_ecart:g}")
                    + " hors de portée des bras, à leurs plafonds), la marche peut reprendre")
                return
            if t - self._t_resserrage >= r.duree_resserrage_s and not (ecartees or faible):
                # Visé pas atteint dans les plafonds, mais la marge suffit.
                self._resserrage_force = False
            elif t - self._t_resserrage >= r.duree_resserrage_s:
                self.stats["port_interrompu"] = self.stats.get("port_interrompu", 0) + 1
                self._port_interrompu = True
                self.a_reposer = True
                if not ecartees and not force and not glisse \
                        and marge is not None and marge >= r.securite_serrage:
                    # Au-dessus du seuil pour tenir, sous celui pour repartir :
                    # les bras n'y arrivent pas (plafonds, bornes du serrage).
                    self.raison = (f"marge de frottement {marge:.2f} {sur} après "
                                   f"{r.duree_resserrage_s:g} s de resserrage : au moins "
                                   f"{r.securite_serrage:g} pour tenir, mais {seuil:g} pour "
                                   f"repartir, que les bras n'atteignent pas : à reposer")
                else:
                    self.raison = (f"serrage perdu au port malgré le resserrage ({pourquoi}) : "
                                   f"à reposer")
                self._noter(f"transport interrompu : {self.raison}")
                self._fin_port("interrompu")
                return
            a_rendre = list(self._ROULIS)
        elif self._t_resserrage is not None and \
                self._horloge() - self._t_resserrage < r.delai_reprise_s:
            self.penche = True                    # pas avant le délai minimal
        elif self._t_resserrage is not None:
            self._t_resserrage, self._resserrage_marge = None, False
            self._repartir_foulee()
            self.penche = self.inclinaison is not None and self.inclinaison > r.inclinaison_max_deg
            self._noter("port : serrage retrouvé" + (
                f" (marge de frottement {marge:.2f}, {sur}, au-dessus de {seuil:g})"
                if marge is not None else "") + ", la marche peut reprendre")
        if not a_rendre or not self._marge_plafonds(tau):
            return
        if self._port0 is not None:
            # Port figé : les deux mains ensemble, du même pas, le long de la
            # normale ; les bras recalculés depuis la posture figée (_q_port),
            # rien d'autre ne bouge. Une main à sa borne : aucune n'avance.
            if any(self._delta[c] + r.pas_serrage_max_m
                   > self._delta_max_poids.get(c, r.penetration_poids_max_m)
                   for c in self._ROULIS):
                return
            avant = dict(self._delta)
            for cote in self._ROULIS:
                self._delta[cote] += r.pas_serrage_max_m
            q = self._q_port()
            if q is None:
                self._delta = avant
                return
            for cote in self._ROULIS:
                self._garder_borne(cote)
            self._ecrire_q(q)
            self._q_tenu = dict(q)
            return
        a_rendre = [c for c in a_rendre if self._delta[c] + r.pas_serrage_max_m
                    <= self._delta_max_poids.get(c, r.penetration_poids_max_m)]
        q = None
        if self._rigide is not None:
            # Corps rigide : le serrage le long de la ligne des mains, les
            # mains gardent leur orientation.
            avant = dict(self._delta)
            for cote in a_rendre:
                self._delta[cote] += r.pas_serrage_max_m
            q = self._q_rigide(self._centre_porte, self._ecart_mains)
            if q is None:
                self._delta = avant
        if q is None:
            q = dict(self._q)
            for cote in a_rendre:
                q2 = self._avancer(q, cote, r.pas_serrage_max_m, self._direction_serrage(q, cote))
                if q2 is not None:
                    q, self._delta[cote] = q2, self._delta[cote] + r.pas_serrage_max_m
        for cote in a_rendre:
            self._garder_borne(cote)
        self._ecrire_q(q)
        self._q_tenu = dict(q)

    def _repartir_foulee(self) -> None:
        """La base repart : la foulée repart des filtre_force × 3 dernières
        lectures (0,3 s), celles qui ont justifié la reprise. Encore pleine des
        lectures d'avant le resserrage, elle arrêterait la base aussitôt."""
        n = max(1, self.r.filtre_force * 3)
        while len(self._hist_normales) > n:
            self._hist_normales.popleft()
            self._t_normales.popleft()

    def _retard_vers_robot(self) -> Optional[float]:
        """De combien (m) le milieu des mains MESURÉES est en avant (loin du
        robot) du milieu des mains COMMANDÉES : retenu par ce que l'objet
        touche derrière lui (le torse). None si les angles sont illisibles."""
        mod = self._mains_modele()
        if mod is None:
            return None
        cmd = self._modele.mains(self._consigne_complete())
        return float((mod[0] + mod[1])[0] - (cmd[0] + cmd[1])[0]) / 2.0

    def _charge(self, tau: Optional[Dict[str, float]]) -> Optional[Tuple[float, float]]:
        """Ce que portent les mains, d'après les couples : (poids en N,
        poussée vers le robot en N) de l'objet tenu, la somme des forces que
        les mains exercent sur lui (au point de contact, poids des bras et
        biais retirés), vers le haut et vers le robot ; moyenne glissante. Le
        serrage, le long de la ligne des mains, s'y annule. None : couples
        illisibles ou non fiables."""
        if tau is None or self.couples_fiables is False:
            return None
        angles = self._angles_mesures()
        if angles is None:
            return None
        g = self._gravite_bassin()
        forces = [self._force_main(c, tau, angles, g, depuis_fermeture=False)
                  for c in ("left", "right")]
        if any(f is None for f in forces):
            return None
        haut = -g / max(1e-9, float(np.linalg.norm(g)))
        vers_robot = np.array([-1.0, 0.0, 0.0])
        # Et tarée : la hausse des couples depuis le zéro d'avant la fermeture
        # (_tau0), où leur biais s'annule (poids_tare).
        tare = [self._force_main(c, tau, angles, g) for c in ("left", "right")] \
            if self._tau0 else []
        self._hist_charge.append((sum(float(f @ haut) for f in forces),
                                  sum(float(f @ vers_robot) for f in forces),
                                  sum(float(f @ haut) for f in tare)
                                  if tare and all(f is not None for f in tare) else None))
        self._lecture_charge = self._hist_charge[-1]     # pour la pesée, comptée une fois
        self.charge = (float(np.mean([v[0] for v in self._hist_charge])),
                       float(np.mean([v[1] for v in self._hist_charge])))
        return self.charge

    def _surveiller_rotation(self, tau: Optional[Dict[str, float]]) -> str:
        """Objet tenu : tourne-t-il autour de la ligne des mains ? La
        caméra de tête, quand elle le voit (en entier, ou son dessus en
        partie), au-delà de `rotation_vue_max_deg` (jamais plus que
        `rotation_max_deg`). "" s'il tient, sinon pourquoi.

        Les couples ne la voient pas : le couple de basculement qu'une
        rotation fait passer par les mains est plafonné à leur frottement en
        rotation (couple_retenu, 0,16 N·m serrées à 8 N), sous le bruit des
        couples estimés (0,3 N·m par articulation) ; estimé quand même (effort
        6D sur 6 articulations), il valait 0,5 à 2,7 N·m pour un carton
        centré sur le robot simulé. La rotation se prévient (ligne des mains
        par le centre de gravité, torse en troisième appui) ; ses suites se
        voient sur les couples (poids perdu pendant le port :
        _surveiller_port)."""
        r = self.r
        lire = self.mesures.rotation
        v = None
        if lire is not None:
            try:
                v = lire()
            except Exception:             # noqa: BLE001 — caméra en panne : sans elle
                v = None
        self.rotation_vue = None if v is None or not math.isfinite(float(v)) else float(v)
        seuil_vue = min(r.rotation_max_deg, r.rotation_vue_max_deg)
        if self.rotation_vue is not None and abs(self.rotation_vue) > seuil_vue:
            return (f"carton tourné de {self.rotation_vue:+.0f}° autour de la ligne des mains, "
                    f"il tourne entre les paumes (caméra de tête ; au plus {seuil_vue:g}°)")
        if self._charge(tau) is not None:
            # Poids de référence du port : le plus grand porté depuis le levage.
            self._poids_ref = max(self._poids_ref or 0.0, self.charge[0])
        return ""

    def _surveiller_port(self) -> None:
        """Pendant le transport (TENU) : un objet qui tourne, ou qui ne
        pèse plus sur les mains (il glisse, ou repose ailleurs) : `a_reposer`,
        la façade arrête la base et le signale (comme l'objet penché)."""
        if self._port_interrompu:
            # Tenu jusqu'à ce qu'on le repose (le contrôle d'inclinaison,
            # objet redevenu droit, ne doit pas relancer la marche).
            self.penche = self.a_reposer = True
            return
        tau, _ = self._efforts()
        tourne = self._surveiller_rotation(tau)
        if not tourne and self.charge is not None and \
                len(self._hist_charge) == self._hist_charge.maxlen:
            # Les filtre_force dernières lectures (0,1 s) : vu en 0,3 s au plus.
            r, t = self.r, self._horloge()
            poids = float(np.mean([v[0] for v in
                                   list(self._hist_charge)[-max(1, r.filtre_force):]]))
            lu = self._niveau_debut_port(t, self._hist_charge[-1][0], poids)
            if self._t_sans_poids is not None and self._niveau_chute is not None:
                # Une chute en cours : le niveau d'avant elle, même s'il se remplit
                # encore (lâché dans la première seconde du port, ses lectures
                # l'abaisseraient : jamais vu, prise normale).
                lu = self._niveau_chute
            niveau, incertain = lu if lu is not None else (None, 0.0)
            pese = self.poids_carton
            if pese:
                # Depuis le niveau du début du port, dans la même posture (pas
                # encore lu : voir _niveau_debut_port). Encore jeune (quelques
                # lectures), plus son incertitude.
                perte = niveau is not None and niveau - poids > \
                    (1.0 - r.part_poids_min) * pese + r.sigmas_niveau_port * incertain
            else:
                # Sans pesée : le plus grand poids lu depuis le levage, comme avant.
                perte = self._poids_ref is not None and poids < r.part_poids_min * self._poids_ref
            if perte:
                if self._t_sans_poids is None:
                    self._t_sans_poids, self._niveau_chute = t, lu
                if t - self._t_sans_poids >= r.duree_perte_s:
                    tourne = ("le carton ne pèse plus sur les mains (" + (
                        f"{poids:.1f} N, {niveau:.1f} au début du port : {niveau - poids:.1f} "
                        f"de moins, au plus {100 * (1.0 - r.part_poids_min):.0f} % des "
                        f"{pese:.1f} pesés" if pese else
                        f"{poids:.1f} N, {self._poids_ref:.1f} au levage")
                        + ") : il glisse ou s'est échappé")
            else:
                self._t_sans_poids = self._niveau_chute = None
        if tourne:
            self._interrompre_port(tourne)

    def _niveau_debut_port(self, t: float, lecture: float,
                           poids: float) -> Optional[Tuple[float, float]]:
        """Le poids lu au début du port, dans sa posture : (la moyenne des
        lectures de sa première foulée (fenetre_marge_s), son incertitude :
        leur écart-type sur la racine de leur nombre), en N ; tant qu'elle se
        remplit, sans les recentes_niveau_s les plus récentes (une chute en
        cours n'abaisse pas sa propre référence), puis figée. `poids` : le
        poids lu sur 0,1 s à ce pas. Pas encore lu (ses 0,4 premières
        secondes) : le plus bas de la pesée et du poids lu en entrant au port ;
        None sans pesée."""
        r, lues = self.r, self._niveau_port
        if self._niveau_port_fige is not None:
            return self._niveau_port_fige
        if not lues:
            self._poids_entree = poids
        if not lues or t - lues[0][0] <= r.fenetre_marge_s + 1e-9:
            lues.append((t, lecture))
        anciennes = [w for tt, w in lues if tt <= t - r.recentes_niveau_s + 1e-9]
        if len(anciennes) < max(2, r.filtre_force):
            # Un lâcher dans ces 0,4 s serait sinon vu tard, ou jamais (ses
            # lectures dans le niveau). Ni la pesée seule (autre posture : un
            # carton de 0,22 kg pesé 2,0 N est lu -0,2 N en entrant au port en
            # pendule, campagne n° 105), ni l'entrée seule (fin du geste, en
            # mouvement) : le plus bas des deux, à l'incertitude d'une moyenne
            # de filtre_force lectures.
            if not self.poids_carton:
                return None
            return (min(self.poids_carton, self._poids_entree),
                    (self._bruit_pesee or 0.0) / math.sqrt(max(1, r.filtre_force)))
        # Leur écart-type, jamais moins que celui d'une lecture à la pesée, bras
        # immobiles (sur quelques lectures, il peut tomber au plus bas par hasard).
        ecart = max(float(np.std(anciennes, ddof=1)), self._bruit_pesee or 0.0)
        lu = (float(np.mean(anciennes)), ecart / math.sqrt(len(anciennes)))
        if len(anciennes) == len(lues) and t - lues[0][0] > r.fenetre_marge_s:
            self._niveau_port_fige = lu
        return lu

    def _surveiller_inclinaison(self) -> None:
        """Pendant le transport : au-delà de `inclinaison_max_deg`, `penche`
        (la façade arrête la base) et une correction (serrer un peu plus,
        ramener plus près du buste) ; si l'objet reste penché, `a_reposer` :
        on ne marche pas avec, la façade le signale."""
        lire = self.mesures.inclinaison
        if lire is None:
            return
        try:
            v = lire()
        except Exception:                 # noqa: BLE001
            v = None
        self.inclinaison = None if v is None or not math.isfinite(float(v)) else float(v)
        if self.inclinaison is None:
            return
        self.inclinaison_max = max(self.inclinaison_max, self.inclinaison)
        if self.inclinaison <= self.r.inclinaison_max_deg:
            self.penche = self.a_reposer = False
            return
        self.penche = True
        if self._corrections_inclinaison >= 1:
            self.a_reposer = True
            self.raison = (f"objet penché de {self.inclinaison:.0f}° malgré la correction : "
                           f"à reposer")
            return
        self._corrections_inclinaison += 1
        self.stats["corrections_inclinaison"] = self.stats.get("corrections_inclinaison", 0) + 1
        # Serrer un peu plus, seulement avec un effort lisible et de la
        # marge sous la limite (2 N·m) ; sinon, ramener plus près suffit.
        tau, _ = self._efforts()
        if tau is not None and all(abs(v) + 2.0 <= self._limite_effort(j)
                                   for j, v in tau.items()):
            for cote in self._delta:
                self._delta[cote] = min(self._delta_max.get(cote, 0.0), self._delta[cote] + 0.002)
        self._ramener(plus_pres=0.02)

    def _pas_mesure(self, dt: float) -> bool:
        """Un pas de la prise mesurée ; faux si l'état relève du code commun."""
        e = self.etat
        if e is not Etat.TENU:
            self._port_actif = False      # la prochaine entrée en TENU refige la pose
        if e is Etat.PLACEMENT:
            self._pas_placement()
        elif e is Etat.FERMETURE:
            self._pas_fermeture(dt)
        elif e is Etat.SERRAGE:
            self._pas_serrage_mesure(dt)
        elif e is Etat.LEVAGE:
            self._garder_sous_la_limite(dt)
            self._pas_levage_mesure()
        elif e is Etat.REPOSE:
            self._pas_repose()
        elif e is Etat.RAMENER:
            self._garder_sous_la_limite(dt)
            self._pas_ramener()
        elif e is Etat.TENU:
            if not self._port_actif:
                self._figer_port()
                self._port_actif = True
                # Nouvelle posture (début du port, ou fin d'un geste) : nouveau niveau.
                self._niveau_port, self._niveau_port_fige, self._niveau_chute = [], None, None
            if self._tangage_vise is not None:
                self._pas_tangage()
            self._garder_sous_la_limite(dt)
            self._surveiller_inclinaison()
            self._surveiller_port()
            self._garder_serrage()
            self._surveiller_mains_port()
        elif e in (Etat.MONTEE, Etat.DESCENTE, Etat.POSE, Etat.OUVERTURE):
            if e is not Etat.OUVERTURE:
                self._garder_sous_la_limite(dt)
            if e is Etat.MONTEE:
                # Carton monté au-dessus du plateau : toujours porté, surveillé ;
                # tourné de plus de rotation_max_deg, le geste s'arrête là, tenu.
                self._surveiller_mains_port(corriger=False)
                if self._port_interrompu:
                    self._t_phase = None
                    self.etat = Etat.TENU
                    return True
            self._pas_apres_prise(dt)
        else:
            return False
        return True

    def _centre_tenu(self) -> np.ndarray:
        mg, md = self._contacts_bassin()
        return (mg + md) / 2.0

    def _pas_apres_prise(self, dt: float) -> None:
        """Montée, descente, pose et ouverture, objet tenu en prise mesurée :
        les mains suivent un point (repère du bassin), le serrage gardé.
        Avant un plateau, l'objet repart devant, là où il a été pris."""
        p, r = self.p, self.r
        x_prise = float(self._plan["centre"][0]) if self._plan else 0.29
        # Hauteur des points de contact au-dessus du centre de l'objet, à la
        # prise : les mains (_centre_porte) sont là, pas à son centre.
        dz = self._dz_prise or 0.0
        if self.etat is Etat.MONTEE:
            haut = (self._surface - float(self.mesures.bassin_z())
                    + (self.objet.hauteur / 2.0 if self.objet else 0.1) + dz)
            visee = np.array([x_prise, float(self._centre_porte[1]),
                              haut + p.marge_franchissement])

            def atteinte(v):
                # Corps rigide : tout le chemin, vérifié au premier pas ; ensuite
                # _interp_c s'arrête là où la pose ne se tiendrait plus.
                if self._rigide is None:
                    return self._q_mains(v, self._ecart_mains)
                if self._t_phase is None:
                    return self._chemin_rigide(np.array(self._centre_porte, dtype=float), v, None)
                return self._q
            q = atteinte(visee)
            if q is None:
                visee[2] = haut + p.marge_franchissement_min
                q = atteinte(visee)
            if q is None:
                self.stats["franchissements_impossibles"] = \
                    self.stats.get("franchissements_impossibles", 0) + 1
                self.raison = (f"plateau à {self._surface:.2f} m trop haut pour y passer "
                               f"l'objet : hors de portée des bras")
                self.etat = Etat.TENU
                return
            if self._t_phase is None:
                self._q_de, self._q_vers, self._t_phase = dict(self._q), q, self._horloge()
                self._c_de = np.array(self._centre_porte, dtype=float)
                self._c_vers, self._c_actuel = visee, None
                self._t0 = self._horloge()
            qi, fini = self._interp_c(self.r.duree_ramener_s)
            self._ecrire_q(qi)
            if fini and self._horloge() - self._t0 >= self.r.duree_ramener_s \
                    + p.stabilisation_montee_s or (fini and self.degage_au_dessus(self._surface)):
                self._centre_porte = np.array(self._c_vers, dtype=float)   # là où le geste est allé
                self._t_phase = None
                fond = self._fond_objet()
                if fond >= self._surface + p.marge_franchissement_min:
                    self.stats["franchissements"] = self.stats.get("franchissements", 0) + 1
                    self._franchi = self._surface
                else:
                    self.raison = (f"plateau à {self._surface:.2f} m trop haut pour y passer "
                                   f"l'objet : fond du carton à {fond:.2f} m")
                self.etat = Etat.TENU
        elif self.etat is Etat.DESCENTE:
            if self._rigide is not None and self._t_phase is None:
                # Corps rigide : tout le chemin, devant puis jusqu'au plateau,
                # vérifié avant de bouger ; sinon l'objet reste tenu là où il
                # est. Arrêtée en chemin, une descente ne remonte pas forcément
                # (pose qui ne se tient plus là où le retour passe, coude en butée).
                self._t_phase = self._horloge()
                q, pourquoi = self._depose_possible(self._surface)
                if q is None:
                    self.stats["deposes_impossibles"] += 1
                    self.raison = (f"dépose impossible : plateau à {self._surface:.2f} m, "
                                   f"{pourquoi} (chemin et ouverture vérifiés avant de "
                                   f"descendre, l'objet reste tenu là où il est)")
                    self.etat = Etat.TENU
                    return
            # D'abord devant (sans saut : 0,15 m/s), puis vers le bas (5 cm/s).
            c = np.array(self._centre_porte, dtype=float)
            pas = min(dt, 0.05)
            if abs(c[0] - x_prise) > 1e-4:
                c[0] += max(-0.15 * pas, min(0.15 * pas, x_prise - c[0]))
            else:
                c[2] -= 0.05 * pas
            q = self._q_mains(c, self._ecart_mains)
            if q is None or c[2] < -0.3:
                self.stats["deposes_impossibles"] += 1
                self.raison = (f"dépose impossible : plateau à {self._surface:.2f} m hors "
                               f"d'atteinte des bras")
                self._remonter_tenu()
                return
            self._centre_porte = c
            self._ecrire_q(q)
            if self._fond_objet() <= self._surface + p.jeu_depose:
                ouvertes = self._q_ouverture()
                if ouvertes is None:
                    # Vérifiée avant de descendre à quelques mm près, l'ouverture
                    # sur place ne se fait pas là où sont les mains : _ouvrir
                    # rouvrirait les roulis, elles remonteraient en s'ouvrant et
                    # lâcheraient l'objet de haut (3,2 cm au-dessus d'un plateau
                    # de 71,5 cm). Il est remonté, tenu.
                    self.stats["deposes_impossibles"] += 1
                    self.raison = (f"dépose impossible : plateau à {self._surface:.2f} m, les "
                                   f"mains ne s'y ouvriraient pas sur place (vu en bas) : "
                                   f"l'objet remonté, tenu")
                    self._remonter_tenu()
                    return
                self.stats["deposes_plateau"] += 1
                self._ouvrir(ouvertes)
        elif self.etat is Etat.POSE:
            qi, fini = self._interp_q(p.duree_pose_s)
            self._ecrire_q(qi)
            if fini:
                self._ouvrir()
        elif self.etat is Etat.OUVERTURE:
            qi, fini = self._interp_q(p.duree_ouverture_s)
            self._ecrire_q(qi)
            if fini:
                self.stats["poses"] += 1
                self.objet, self._q_tenu = None, None
                self._regler_raideur(False)              # objet déposé : gains normaux
                # Retour progressif à la posture de port (jamais un saut).
                self._q_de = dict(self._q)
                self._q_vers = {**self._libres0,
                                **posture_bras(p.roulis_ouvert, p.tangage_port, p.coude)}
                self._aller(Etat.LACHER)

    def _remonter_tenu(self) -> None:
        """On ne lâche pas l'objet de haut : on le remonte contre le buste,
        en le tenant (RAMENER, puis TENU)."""
        self._c_de = np.array(self._centre_porte, dtype=float)
        self._c_vers = self._centre_porte = self._centre_transport
        self._q_de, self._q_vers = dict(self._q), dict(self._q_tenu)
        self._c_actuel = None             # pas encore bougé dans ce geste
        self._aller(Etat.RAMENER)

    def _q_ouverture(self) -> Optional[Dict[str, float]]:
        """Bras qui écartent les mains de `jeu_ouverture_m` de plus que le
        contact, sur place (autour des mains mesurées), serrage relâché ;
        None si les bras n'y arrivent pas. Ne change rien."""
        delta, self._delta = self._delta, {"left": 0.0, "right": 0.0}
        try:
            return self._q_mains(self._centre_tenu(),
                                 (self._ecart_mains or 0.33) + 2 * self.r.jeu_ouverture_m)
        finally:
            self._delta = delta

    def _ouvrir(self, q: Optional[Dict[str, float]] = None) -> None:
        """Les mains s'écartent de `jeu_ouverture_m` de plus que le contact
        (`q` : les bras ouverts, déjà calculés par _q_ouverture) ; sinon, les
        roulis rouverts de 0,3 rad."""
        q = self._q_ouverture() if q is None else q
        self._fin_port("déposé")
        self._delta = {"left": 0.0, "right": 0.0}
        q_vers = dict(self._q)
        if q is not None:
            q_vers = q
        else:
            for cote, j in self._ROULIS.items():
                q_vers[j] = self._q[j] - self._FERME[cote] * 0.3
        self._q_de, self._q_vers = dict(self._q), q_vers
        self._aller(Etat.OUVERTURE)

    def distance_de_prise(self, objet: Objet, d_min: float = 0.25, tangage: float = 0.0,
                          cap: Optional[float] = None, abaissement: float = 0.0
                          ) -> Tuple[Optional[float], str]:
        """Prise mesurée : à quelle distance (m) du centre de l'objet, le long
        du cap de prise, arrêter le bassin pour que les mains atteignent le
        milieu de ses faces et que les avant-bras passent au-dessus du
        plateau ; la plus courte à partir de `d_min`. (None, raison) s'il
        n'y en a aucune, ou si l'objet est trop large pour glisser les mains
        ouvertes autour de lui, de face comme ici (ouverture_presaisie).
        `tangage` : torse penché d'autant (rad, taille), voir plan_penche ;
        `abaissement` : bassin abaissé d'autant (m, jambes pliées), voir
        plan_posture. `cap` : cap du robot à la prise (rad, repère monde), face à une face de
        l'objet (son lacet, à un quart de tour près). Si la normale du bord est
        connue (Objet.lacet_bord), le bord est vu en biais : la garde des
        avant-bras le suit, et les pieds, qui avancent alors plus vers lui
        (avancee_pieds), n'en sont jamais plus près. None : face à la table,
        comme avant."""
        pose, raison = self.ouverture_presaisie(objet, de_face=True)
        if pose is None:
            return None, raison
        z = float(objet.centre[2]) - float(self.mesures.bassin_z()) + float(abaissement)
        port = self._port_penche(tangage)
        # Penchées, les épaules avancent d'environ 0,45 m × sin(tangage) : la
        # recherche va d'autant plus loin (droit : jusqu'à 0,45 m, comme avant).
        loin = 0.45 + 0.45 * math.sin(max(0.0, tangage))
        cap_bord = 0.0
        if cap is not None and objet.lacet_bord is not None:
            cap_bord = math.remainder(objet.lacet_bord - cap, 2.0 * math.pi)
        debord, marge = self._pieds_au_bord(cap_bord)
        dz = hauteur_prise(self.r, objet.hauteur) or 0.0
        depuis = d_min
        while True:
            d, raison = distance_ideale(self.mesures.modele, objet.largeur, objet.profondeur,
                                        objet.hauteur, z, port, bord_m=objet.bord_m,
                                        reglages=replace(self.r, contact_main_m=self._bout),
                                        distances=np.arange(depuis, loin, 0.005),
                                        cap_bord=cap_bord, debord_pieds=debord,
                                        marge_pieds=marge)
            if d is None or not dz:
                return d, raison
            # Prise en pendule : d'où, aussi, les mains toucheront assez haut
            # après la fermeture (le roulis seul les fait descendre en arc).
            q_o, _, touche = self._ouvert_pendule(np.array([d, 0.0, z + dz]), objet.largeur,
                                                  0.0, port)
            if q_o is None:
                # Plus loin, moins encore : la prise (_planifier) le refuserait.
                return None, (f"hors de portée des bras : à {d:.2f} m, la plus courte que la "
                              f"table permet, les mains ouvertes n'atteignent pas ses faces "
                              f"{100 * dz:.0f} cm au-dessus de son centre (prise en pendule)")
            if touche is None or touche - z >= self.r.marge_pendule_m + 0.01:
                return d, raison
            depuis = d + 0.005
            if depuis >= loin:
                return None, (f"prise en pendule impossible : à toute distance, en fermant (le "
                              f"roulis seul), les mains toucheraient moins de "
                              f"{100 * (self.r.marge_pendule_m + 0.01):.0f} cm au-dessus du "
                              f"centre de gravité")

    def _pieds_au_bord(self, cap_bord: float) -> Tuple[float, float]:
        """(debord, marge) des pieds pour un bord dont la normale fait
        l'angle `cap_bord` avec l'avant du robot : de combien ils avancent de
        plus vers lui qu'en face (avancee_pieds), et, au vrai bord, l'erreur
        d'approche de la marche (position, et cap qui avance un pied)."""
        from axis_os.services.locomotion.equilibre import avancee_pieds
        r, a = self.r, avancee_pieds(cap_bord)
        pire = max(avancee_pieds(cap_bord + e) for e in (-r.erreur_cap_rad, r.erreur_cap_rad))
        return a - avancee_pieds(0.0), r.erreur_approche_m + max(0.0, pire - a)

    # ---- Planificateur de prise : évaluer une prise SANS bouger -----------

    _NOMS_ARTICULATIONS = {
        "shoulder_pitch_joint": "épaule (tangage)", "shoulder_roll_joint": "épaule (roulis)",
        "shoulder_yaw_joint": "épaule (lacet)", "elbow_joint": "coude",
        "wrist_roll_joint": "poignet (roulis)", "wrist_pitch_joint": "poignet (tangage)",
        "wrist_yaw_joint": "poignet (lacet)"}

    def _nom_articulation(self, j: str) -> str:
        cote, _, reste = j.partition("_")
        nom = self._NOMS_ARTICULATIONS.get(reste, reste)
        return f"{nom} {'gauche' if cote == 'left' else 'droit'}"

    def evaluer_prise(self, objet: Objet, pose: Tuple[float, float, float],
                      masse_kg: Optional[float] = None, chemins: bool = False,
                      contexte: Optional[dict] = None,
                      posture: Tuple[float, float] = (0.0, 0.0)) -> list:
        """Les contrôles de la prise de `objet` (ses faces serrées : celles
        de part et d'autre du cap), le bassin à `pose` (x, y, cap ; repère de
        la pièce), SANS bouger : ceux de la prise, aux mêmes seuils, dans
        l'ordre du moins cher au plus cher. Rend les marges
        (planificateur.Marge) ; le premier refus arrête.
          - pieds : derrière le bord, erreur d'approche comprise (règle du bord) ;
          - largeur : entre les paumes ouvertes en grand, jeu_ouverture_m de chaque côté ;
          - portée : mains ouvertes, au contact et au-delà (comme _planifier) ;
          - garde des avant-bras au-dessus du plateau (marge_plateau_m) ;
          - ligne des mains par le centre de gravité (tolerances_cg) ;
          - couples des bras pour le serrage visé d'après le poids estimé
            (`masse_kg`, masse_max_kg par défaut : on ne le pèse qu'une fois
            soulevé) et ce poids, contre leurs plafonds (_limite_effort) ;
          - équilibre, le carton aux mains (BornesPosture.marge_min_m) ;
          - `chemins` : levage, retour vers le buste et dépose à sa place, en
            corps rigide (chemins vérifiés, ouverture en bas), comme ils se
            feraient.
        `contexte` : ce qui ne change pas d'une candidate à l'autre (écart des
        paumes ouvertes en grand, longueur des bras) et la dernière solution
        des bras, graine de la suivante (planifier_prise)."""
        from axis_os.services.locomotion.equilibre import jambes, modele_g1
        from axis_os.services.locomotion.service import BornesPosture
        from axis_os.services.manipulation.planificateur import Marge
        r = self.r
        masse = r.masse_max_kg if masse_kg is None else float(masse_kg)
        def repere(x, y, cap):
            """L'objet vu du bassin posé en (x, y, cap) : son centre, le lacet
            de ses faces serrées, le cap du bord, le bassin au bord."""
            c, s_ = math.cos(cap), math.sin(cap)
            dx, dy = float(objet.centre[0]) - x, float(objet.centre[1]) - y
            centre = np.array([c * dx + s_ * dy, -s_ * dx + c * dy,
                               float(objet.centre[2]) - float(self.mesures.bassin_z())
                               + abaissement])
            lacet = (objet.lacet - cap + math.pi / 4) % (math.pi / 2) - math.pi / 4
            cap_bord = (math.remainder(objet.lacet_bord - cap, 2.0 * math.pi)
                        if objet.lacet_bord is not None else 0.0)
            bord = (float(centre[0]) * math.cos(cap_bord) + float(centre[1]) * math.sin(cap_bord)
                    - objet.profondeur / 2.0 - objet.bord_m)
            return centre, lacet, cap_bord, bord
        x, y, cap = (float(v) for v in pose)
        # Posture de la marche (tangage du torse, abaissement du bassin) : le
        # bassin plus bas, l'objet d'autant plus haut dans son repère ; la
        # taille penchée dans les angles des bras (plan_posture).
        tangage, abaissement = (float(v) for v in posture)
        centre, lacet, cap_bord, bord = repere(x, y, cap)
        marges = []

        def note(nom, relative, texte):
            marges.append(Marge(nom, float(relative), texte))
            return relative >= 0.0

        debord, marge = self._pieds_au_bord(cap_bord)
        limite = r.bord_min_m + debord + marge
        if not note("pieds", (bord - limite) / r.erreur_approche_m,
                    f"bassin à {100 * bord:.1f} cm du bord, il en faut {100 * limite:.1f} pour "
                    f"que les pieds restent derrière (erreur d'approche comprise)"):
            return marges
        ctx = {} if contexte is None else contexte
        face = objet.largeur * abs(math.cos(lacet)) + objet.profondeur * abs(math.sin(lacet))
        if "libre" not in ctx:
            ctx["libre"] = self._ouverture(math.inf)[1]
        libre = ctx["libre"]
        besoin = face + 2.0 * r.jeu_ouverture_m
        if not note("largeur", (libre - besoin) / r.jeu_ouverture_m,
                    f"{100 * face:.0f} cm entre les mains : ouvertes en grand, elles laissent "
                    f"{100 * libre:.0f} cm, il en faut {100 * besoin:.0f} (jeu compris)"):
            return marges
        fixes = self._articulations_fixes()
        if tangage:
            fixes[self.TAILLE] = tangage
        graine = {**fixes, **self._q}
        if "bras" not in ctx:
            # Épaule et longueur de chaque bras (épaule-coude-poignet-contact) :
            # au-delà, aucune cinématique inverse n'y arrive.
            ctx["bras"] = {}
            for cote in self._ROULIS:
                pts = self._chaine(cote).points(graine)
                ctx["bras"][cote] = (pts["epaule"], sum(
                    float(np.linalg.norm(pts[b] - pts[a]))
                    for a, b in (("epaule", "coude"), ("coude", "poignet"), ("poignet", "main"))))
        dz = hauteur_prise(r, objet.hauteur)
        if not note("pendule", -1.0 if dz is None else (dz - r.marge_pendule_m) / r.marge_pendule_m,
                    f"carton de {100 * objet.hauteur:.0f} cm de haut : mains "
                    + (f"visées {100 * dz:.0f} cm au-dessus de son centre" if dz is not None else
                       f"à moins de {100 * r.marge_pendule_m:.0f} cm au-dessus de son centre "
                       f"(sous son arête de {100 * r.bord_face_m:.0f} cm) : trop bas pour une "
                       f"prise en pendule")):
            return marges
        centre_cg, centre = centre, centre + np.array([0.0, 0.0, dz])
        cibles = cibles_mains(centre, objet.largeur, lacet, r.demi_main_m)
        q_o = q_c = q_v = None
        if all(float(np.linalg.norm(c_ - ctx["bras"][cote][0])) <= ctx["bras"][cote][1]
               for cote, c_ in zip(("left", "right"), cibles)):
            # Au contact d'abord (graine : la candidate voisine), puis ouvertes
            # et au-delà, depuis lui.
            q_c, _ = resoudre(self._modele, *cibles, ctx.get("graine", graine))
            if q_c is not None:
                depuis = {**graine, **q_c}
                q_o, _ = resoudre(self._modele, *cibles_mains(
                    centre, objet.largeur, lacet, r.demi_main_m + r.jeu_ouverture_m), depuis)
                q_v, _ = resoudre(self._modele, *cibles_mains(
                    centre, objet.largeur, lacet, r.demi_main_m - r.penetration_max_m), depuis)
                # Graine des suivantes : seulement d'une candidate à portée. Celle
                # d'une candidate refusée (épaule à +0,2 rad, bras retournés)
                # égarait toutes les suivantes (pick() du SDK, carton à 0,34 m).
                if q_o is not None and q_v is not None:
                    ctx["graine"] = depuis
        if not note("portée", math.inf if None not in (q_o, q_c, q_v) else -1.0,
                    f"objet à {centre[0]:.2f} m devant le bassin et {centre[2]:+.2f} m en "
                    f"hauteur : les mains " + ("atteignent le milieu de ses faces"
                                               if None not in (q_o, q_c, q_v)
                                               else "n'atteignent pas le milieu de ses faces")):
            return marges
        # Où les mains toucheront, la fermeture (le roulis seul) les faisant
        # descendre en arc : au-dessus du centre de gravité d'au moins
        # marge_pendule_m, 1 cm de marge (_planifier, _ouvert_pendule).
        _, _, touche = self._ouvert_pendule(centre, objet.largeur, lacet,
                                            {**graine, **(q_o or {})})
        haut_touche = None if touche is None else touche - float(centre_cg[2])
        besoin_touche = r.marge_pendule_m + 0.01
        if not note("fermeture", 1.0 if haut_touche is None else
                    (haut_touche - besoin_touche) / besoin_touche,
                    "fermeture par tout le bras (le roulis n'y suffit pas)" if haut_touche is None
                    else f"en fermant, les mains toucheront {100 * haut_touche:.1f} cm au-dessus "
                         f"du centre de gravité (au moins {100 * besoin_touche:.0f})"):
            return marges
        # Les mains à la hauteur de l'objet avant de serrer (règle de Kenzo) :
        # plus haut que lui, les jambes doivent descendre (plan_posture). Après
        # la portée : un carton trop bas reste « à portée » pour la campagne.
        haut_mains = self._abaissement_hauteur(objet) - abaissement
        if not note("mains", -haut_mains / 0.01 if haut_mains > 0.0 else math.inf,
                    f"mains {100 * haut_mains:.0f} cm au-dessus de l'objet : plier les jambes"
                    if haut_mains > 0.0 else "mains à la hauteur de l'objet"):
            return marges
        qq = {**fixes, **q_c}
        plateau = float(centre_cg[2]) - objet.hauteur / 2.0
        garde = garde_plateau(self._modele, qq, plateau, max(r.bord_min_m, bord),
                              r.rayon_avant_bras_m, cap_bord)
        # Aux bornes de l'erreur d'approche de la marche (erreur_approche_m,
        # erreur_cap_rad) : le robot arrive à quelques cm et degrés de cette
        # pose, et la prise refait sa cinématique là où il est. Campagne de
        # cartons : 8 prises refusées à la prise, avant-bras à 2,5–2,9 cm.
        pire = garde
        if garde >= r.marge_plateau_m:
            c, s_ = math.cos(cap), math.sin(cap)
            ea, ecap = r.erreur_approche_m, r.erreur_cap_rad
            for ex, ey, ec in ((ea, 0.0, 0.0), (-ea, 0.0, 0.0), (0.0, ea, 0.0), (0.0, -ea, 0.0),
                               (0.0, 0.0, ecap), (0.0, 0.0, -ecap)):
                ce, la, cb, bo = repere(x + c * ex - s_ * ey, y + s_ * ex + c * ey, cap + ec)
                qe, _ = resoudre(self._modele, *cibles_mains(ce, objet.largeur, la, r.demi_main_m),
                                 qq)
                if qe is not None:
                    pire = min(pire, garde_plateau(self._modele, {**fixes, **qe}, plateau,
                                                   max(r.bord_min_m, bo), r.rayon_avant_bras_m,
                                                   cb))
        if not note("garde", (pire - r.marge_plateau_m) / r.marge_plateau_m,
                    f"avant-bras à {100 * garde:.1f} cm du plateau"
                    + (f", {100 * pire:.1f} au pire de l'erreur d'approche "
                       f"(±{100 * r.erreur_approche_m:g} cm, "
                       f"±{math.degrees(r.erreur_cap_rad):g}°)" if pire < garde - 1e-4 else "")
                    + f" (au moins {100 * r.marge_plateau_m:.0f})"):
            return marges
        mg, md = self._modele.mains(qq)
        prof, haut = ecart_ligne_cg(mg, md, centre_cg)
        tp, th = tolerances_cg(r)
        mp = r.marge_pendule_m
        if not note("ligne", min((tp - abs(prof)) / tp, (th - haut) / th, (haut - mp) / mp),
                    f"ligne des mains à {100 * prof:+.1f} cm du centre de gravité en profondeur "
                    f"(au plus {100 * tp:.1f}) et {100 * haut:+.1f} cm en hauteur (de "
                    f"{100 * mp:.1f} à {100 * th:.1f} au-dessus : prise en pendule)"):
            return marges
        poids = 9.81 * masse
        vise = r.securite_serrage * poids / (2.0 * r.frottement_serrage) + r.marge_serrage_n
        n = np.array([-math.sin(lacet), math.cos(lacet), 0.0])
        pire = None
        for cote, sens in (("left", -1.0), ("right", 1.0)):
            chaine, joints = self._chaine(cote), list(self._BRAS_COTE[cote])
            # τ = τ_gravité + Jᵀ F, F la force de la main sur l'objet (serrage
            # vers lui, et la moitié de son poids portée), comme _force_main.
            f = sens * vise * n + np.array([0.0, 0.0, poids / 2.0])
            tau = (chaine.couples_gravite(qq, joints, (0.0, 0.0, -9.81))
                   + chaine.jacobienne(qq, joints).T @ f)
            for j, t in zip(joints, tau):
                lim = self._limite_effort(j)
                rel = (lim - abs(float(t))) / lim
                if pire is None or rel < pire[0]:
                    pire = (rel, j, abs(float(t)), lim)
        if not note("couples", pire[0],
                    f"{self._nom_articulation(pire[1])} à {pire[2]:.2f} N·m pour {pire[3]:.2f} "
                    f"(serrage de {vise:.1f} N par main pour {masse:.1f} kg estimés)"):
            return marges
        e = modele_g1().equilibre({**jambes(-abaissement), **qq}, [(masse, tuple(centre_cg))])
        mmin = BornesPosture().marge_min_m
        if not note("équilibre", (e.marge - mmin) / mmin, f"{e.texte()} (au moins "
                                                          f"{100 * mmin:.0f} cm)"):
            return marges
        if (tangage or abaissement) and self._posture is not None:
            # La posture elle-même, par la marche, en essai : bornes, gardien
            # de proximité, équilibre (le même modèle, l'objet aux mains).
            rp = self._posture(tangage, abaissement, angles=qq,
                               charges=[(masse, tuple(centre_cg))], essai=True)
            if not note("posture", math.inf if rp.ok else -1.0, rp.reason or ""):
                return marges
        if chemins:
            ok, texte = self._essai_chemins(objet, centre, lacet, cap_bord, bord, q_c, fixes,
                                                 dz=dz, tangage=tangage)
            note("chemins", math.inf if ok else -1.0, texte)
        return marges

    def _essai_chemins(self, objet: Objet, centre: np.ndarray, lacet: float, cap_bord: float,
                       bord: float, q_c: Dict[str, float], fixes: Dict[str, float],
                       dz: float = 0.0, tangage: float = 0.0, depose: bool = True
                       ) -> Tuple[bool, str]:
        """Levage, retour vers le buste (comme _ramener) et dépose à la même
        place (comme la descente : chemin vérifié, ouverture des mains en
        bas), en corps rigide, SANS bouger : l'état de la prise posé le temps
        de l'essai, puis rendu tel qu'il était. `centre` : le milieu des
        points de contact, `dz` au-dessus du centre de l'objet (prise en
        pendule). Torse penché de `tangage` : il doit pouvoir se redresser,
        l'objet tenu là où le retour l'a amené (on ne marche que droit).
        `depose` False : sans la dépose à sa place (plan_penche : on pose
        ailleurs, debout)."""
        r, p = self.r, self.p
        if not all(len(self._commandables(c_)) >= 6 for c_ in self._ROULIS):
            return True, "chemins non vérifiables : sans tangage du poignet, pas de corps rigide"
        noms = ("_plan", "_rigide", "_q", "_delta", "_ecart_mains", "objet", "_centre_porte",
                "_fixes", "ligne_cg")
        avant = {nom: getattr(self, nom, None) for nom in noms}
        try:
            self._fixes, self._q = dict(fixes), dict(q_c)
            # Mains au contact, comptées serrées au plus (bornes du serrage, au
            # toucher puis d'après le poids) : pour s'ouvrir en bas, elles
            # doivent s'écarter d'autant de plus. Prudent.
            serre = r.penetration_max_m + r.penetration_poids_max_m
            self._delta = {"left": serre, "right": serre}
            self._ecart_mains = objet.largeur + 2.0 * r.demi_main_m
            self.objet = objet
            self._plan = {"centre": centre - np.array([0.0, 0.0, dz]), "prise": centre,
                          "bord": bord, "cap_bord": cap_bord, "lacet": lacet}
            poses = self._poses(q_c)
            u = poses["right"][0] - poses["left"][0]
            u = u / max(1e-9, float(np.linalg.norm(u)))
            self._rigide = (np.array(centre, dtype=float),
                            {"left": (*poses["left"], u), "right": (*poses["right"], -u)},
                            dict(self._delta), self._ecart_mains)
            plateau = float(centre[2]) - dz - objet.hauteur / 2.0
            leve = np.array(centre, dtype=float) + np.array([0.0, 0.0, r.hauteur_test_m])
            q = self._chemin_rigide(np.array(centre, dtype=float), leve, plateau)
            if q is None:
                return False, "chemin des bras : levage impossible, les mains ne restent pas un corps rigide"
            porte, q_porte = leve, q
            for k in range(12):
                xk = max(r.distance_torse_m - r.appui_torse_m + objet.profondeur / 2.0
                         + 0.02 * k, 0.05)
                if xk >= float(leve[0]):
                    break
                ck = np.array([xk, float(leve[1]), float(leve[2]) + r.hausse_transport_m])
                qk = self._chemin_rigide(leve, ck, plateau, graine=q)
                if qk is not None:
                    porte, q_porte = ck, qk
                    break
            # Torse penché : il se redresse l'objet tenu là où le retour l'amène,
            # et là où le levage l'a laissé (_ramener y reste s'il ne trouve
            # rien de plus près : robot simulé, table de 70 cm, torse à 5,7°).
            if tangage > 1e-3 and not all(self._redressable(c_, q_, t0=tangage)
                                          for c_, q_ in ((porte, q_porte), (leve, q))):
                return False, ("chemin des bras : le torse ne se redresserait pas, l'objet tenu là "
                               "où le retour ou le levage l'amène (on ne marche que droit)")
            if not depose:
                return True, "levage et retour vers le buste : chemins vérifiés"
            devant = np.array([float(centre[0]), float(porte[1]), float(porte[2])])
            bas = np.array([float(centre[0]), float(porte[1]),
                            plateau + p.jeu_depose + objet.hauteur / 2.0 + dz])
            qd = self._chemin_rigide(porte, devant, None, graine=q_porte, pas_m=0.002)
            qd = None if qd is None else self._chemin_rigide(devant, bas, None, graine=qd,
                                                             pas_m=0.002)
            if qd is None:
                return False, "dépose impossible ensuite : la reposer à sa place, hors d'atteinte"
            ecart = self._ecart_mains + 2.0 * r.jeu_ouverture_m
            relache = {c_: 0.0 for c_ in self._ROULIS}
            if not any(self._q_rigide(bas + np.array([ddx, ddy, 0.0]), ecart, graine=qd,
                                      delta=relache) is not None
                       for ddx in (0.0, -0.005, 0.005) for ddy in (0.0, -0.005, 0.005)):
                return False, ("dépose impossible ensuite : les mains ne s'ouvriraient pas sur "
                               "place, à sa place")
            return True, ("levage, retour vers le buste "
                          + ("" if porte is not leve else "(seulement plus haut) ")
                          + "et dépose à sa place : chemins vérifiés")
        finally:
            for nom, v in avant.items():
                setattr(self, nom, v)

    # Postures que le planificateur essaie quand aucune prise ne passe debout
    # (postures=True) : les jambes d'abord (descendre), puis le torse (aller
    # plus loin), dans les bornes de la marche (BornesPosture).
    POSTURES_PLAN = tuple((0.0, a) for a in (0.025, 0.05, 0.075, 0.10)) + tuple(
        (t, 0.0) for t in (0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35))

    def planifier_prise(self, coins, bord, masse_kg: Optional[float] = None,
                        pas_m: Optional[float] = None, postures: bool = False):
        """Planificateur de prise (voir planificateur.py) : d'après les 8
        coins de la boîte (repère de la pièce) et le bord de la table qui la
        porte (planificateur.BordTable), toutes les candidates (2 paires de
        faces × positions de base), évaluées SANS bouger (evaluer_prise) ; les
        chemins, pour les meilleures seulement, jusqu'à ce que l'une passe.
        Rend un PlanPrise : la retenue, ou la raison de la meilleure.
        `postures` : si aucune ne passe debout, les candidates refusées pour
        la portée ou la garde des avant-bras sont réévaluées penché ou jambes
        pliées (POSTURES_PLAN, la première posture qui donne une prise, avec
        ses chemins, redressement compris) ; Candidate.posture la porte."""
        from axis_os.services.manipulation import planificateur as P
        boite = P.boite_depuis_coins(coins)
        candidates, contexte = [], {}
        for paire in P.paires(boite, bord):
            contexte.pop("graine", None)
            for x, y, cap, d, s in P.positions(boite, paire, pas_m or P.PAS_M):
                cand = P.Candidate(paire, x, y, cap, d, s)
                cand.marges = self.evaluer_prise(self._objet_paire(boite, paire, bord),
                                                 (x, y, cap), masse_kg, contexte=contexte)
                candidates.append(cand)
        for cand in sorted((c for c in candidates if c.ok), key=lambda c: -c.score)[
                :P.COMPLETES_MAX]:
            contexte.pop("graine", None)
            cand.marges = self.evaluer_prise(self._objet_paire(boite, cand.paire, bord),
                                             (cand.x, cand.y, cand.cap), masse_kg, chemins=True,
                                             contexte=contexte)
            cand.complete = True
            if cand.ok:
                break
        choisie = P.choisir(candidates)
        if choisie is None and postures and self._posture is not None:
            choisie = self._planifier_posture(P, boite, bord, candidates, masse_kg)
        plan = P.PlanPrise(boite, bord, candidates, choisie)
        if plan.choisie is None:
            m = plan.meilleure
            refus = None if m is None else m.refus
            plan.raison = ("aucune prise candidate" if m is None else
                           f"même la meilleure prise : {self._QUOI_REFUS.get(refus.nom, '')}"
                           f"{refus.texte}" if refus is not None else
                           f"même la meilleure prise : chemins non vérifiés ({m.explication()})")
        self._noter(plan.resume())
        return plan

    def _planifier_posture(self, P, boite, bord, candidates, masse_kg):
        """Aucune prise debout : les candidates refusées pour la portée, la
        garde des avant-bras ou des mains plus haut que l'objet, réévaluées
        dans chaque posture de POSTURES_PLAN (jambes, puis torse) ; la
        première qui passe, chemins compris."""
        a_revoir = [c for c in candidates if c.refus is not None
                    and c.refus.nom in ("portée", "garde", "mains")]
        for posture in self.POSTURES_PLAN:
            contexte, essais = {}, []
            for c in a_revoir:
                cand = P.Candidate(c.paire, c.x, c.y, c.cap, c.d, c.s, posture=posture)
                cand.marges = self.evaluer_prise(self._objet_paire(boite, c.paire, bord),
                                                 (c.x, c.y, c.cap), masse_kg, contexte=contexte,
                                                 posture=posture)
                essais.append(cand)
            for cand in sorted((c for c in essais if c.ok), key=lambda c: -c.score)[
                    :P.COMPLETES_MAX]:
                contexte.pop("graine", None)
                cand.marges = self.evaluer_prise(self._objet_paire(boite, cand.paire, bord),
                                                 (cand.x, cand.y, cand.cap), masse_kg,
                                                 chemins=True, contexte=contexte,
                                                 posture=posture)
                cand.complete = True
                if cand.ok:
                    candidates.append(cand)
                    return cand
        return None

    # Ce qu'un refus du planificateur veut dire, devant sa raison (les mots
    # du rapport de campagne et des codes de refus du SDK).
    _QUOI_REFUS = {"mains": "trop bas pour les mains : ",
                   "pieds": "hors de portée sans que les pieds passent le bord : ",
                   "largeur": "trop large pour les mains : ",
                   "portée": "hors de portée des bras : ",
                   "couples": "trop lourd pour ces mains, là où il est : ",
                   "équilibre": "équilibre : "}

    def _objet_paire(self, boite, paire, bord) -> Objet:
        """L'objet tel que la prise le voit, serré par cette paire de faces :
        largeur entre les mains, lacet du cap, distance au bord."""
        distance = bord.au_dela(boite.centre[0], boite.centre[1])
        return Objet(tuple(boite.centre), paire.largeur, paire.profondeur, boite.hauteur,
                     lacet=paire.cap, bord_m=distance - paire.profondeur / 2.0,
                     lacet_bord=bord.normale)

    TAILLE = "waist_pitch_joint"
    # Tangages du torse essayés pour atteindre plus loin (rad), du plus petit
    # au plus grand ; le service de marche borne et vérifie l'équilibre.
    TANGAGES_ESSAYES = (0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35)

    def _port_penche(self, tangage: float) -> Dict[str, float]:
        """Bras en posture de port, articulations fixes mesurées, la taille
        penchée de `tangage` s'il n'est pas nul (repère du bassin)."""
        port = {**self._articulations_fixes(),
                **posture_bras(self.p.roulis_ouvert, self.p.tangage_port, self.p.coude)}
        if tangage:
            port[self.TAILLE] = float(tangage)
        return port

    def plan_penche(self, objet: Objet, cap: Optional[float] = None
                    ) -> Tuple[Optional[float], float, str]:
        """(distance de prise, tangage du torse qu'il y faut, raison) : droit
        si les bras l'atteignent ainsi (tangage 0) ; sinon le plus petit
        tangage (TANGAGES_ESSAYES) qui le leur permet et que le service de
        marche accepte (consigne de posture, en essai : bornes, gardien de
        proximité, équilibre). L'équilibre se juge bras à la prise, objet de
        masse_max_kg en son centre : on ne le pèse qu'une fois soulevé. La
        raison d'un succès penché porte la marge d'équilibre ; d'un échec,
        le dernier refus de la marche s'il y en a un. `cap` : voir
        distance_de_prise."""
        d, raison = self.distance_de_prise(objet, cap=cap)
        if d is not None or self._posture is None:
            return d, 0.0, raison
        dernier = ""
        z = float(objet.centre[2]) - float(self.mesures.bassin_z())
        # La portée croît avec le tangage : hors de portée au plus penché (pas
        # seulement avant-bras trop près du plateau), rien de moins n'y suffit.
        t_max = self.TANGAGES_ESSAYES[-1]
        d_max, r_max = self.distance_de_prise(objet, tangage=t_max, cap=cap)
        if d_max is None and "avant-bras" not in r_max:
            return None, 0.0, (f"{raison} ; même penché de {math.degrees(t_max):.0f}° : {r_max}")
        for t in self.TANGAGES_ESSAYES:
            d_t, _ = self.distance_de_prise(objet, tangage=t, cap=cap)
            if d_t is None:
                continue
            port = self._port_penche(t)
            # Les mains visées au-dessus du centre (prise en pendule).
            zm = z + (hauteur_prise(self.r, objet.hauteur) or 0.0)
            q, _ = resoudre(self.mesures.modele, *cibles_mains(
                np.array([d_t - self._bout, 0.0, zm]), objet.largeur, 0.0, self.r.demi_main_m),
                port)
            r = self._posture(t, 0.0, angles={**port, **(q or {})},
                              charges=[(self.r.masse_max_kg, (d_t, 0.0, z))], essai=True)
            if r.ok:
                # Les chemins, comme le planificateur : levage, retour, dépose,
                # et le torse qui se redresse l'objet tenu (on ne marche que droit).
                cap_bord = (math.remainder(objet.lacet_bord - cap, 2.0 * math.pi)
                            if cap is not None and objet.lacet_bord is not None else 0.0)
                fixes = {**self._articulations_fixes(), self.TAILLE: t}
                prise = np.array([d_t, 0.0, zm])
                q_c, _ = resoudre(self._modele, *cibles_mains(prise, objet.largeur, 0.0,
                                                             self.r.demi_main_m),
                                  {**fixes, **self._q})
                ok, texte = (False, "hors de portée des bras") if q_c is None else \
                    self._essai_chemins(objet, prise, 0.0, cap_bord,
                                        d_t * math.cos(cap_bord) - objet.profondeur / 2.0
                                        - objet.bord_m, q_c, fixes, dz=zm - z, tangage=t,
                                        depose=False)
                if ok:
                    return d_t, t, f"penché de {math.degrees(t):.0f}° : {r.reason}"
                # Plus penché, le torse a plus à redresser : rien de mieux.
                dernier = f"penché de {math.degrees(t):.0f}°, {texte}"
                break
            dernier = r.reason
            if "équilibre" in r.reason:
                break               # plus penché, le centre de masse n'avance que plus
        return None, 0.0, raison + (f" ; penché : {dernier}" if dernier else "")

    # Bassins abaissés essayés (m, jambes pliées), du plus petit au plus grand.
    # Les jambes d'abord pour descendre, le torse pour aller plus loin ; le
    # service de marche borne (BornesPosture : torse 0,35 rad, bassin 10 cm,
    # validés au banc lancer_posture.sh) et vérifie l'équilibre (2 cm).
    ABAISSEMENTS_ESSAYES = (0.025, 0.05, 0.075, 0.10)

    def plan_posture(self, objet: Objet, cap: Optional[float] = None, d_min: float = 0.25
                     ) -> Tuple[Optional[float], float, float, str]:
        """(distance de prise, tangage du torse, abaissement du bassin, raison) :
        droit si les bras l'atteignent ainsi ; sinon les jambes d'abord (le
        plus petit abaissement qui suffit : objet trop bas), puis le torse
        penché (plan_penche : objet trop loin). Chaque posture passe par la
        consigne de posture de la marche, en essai (bornes, gardien de
        proximité, équilibre, objet de masse_max_kg aux mains). La prise se
        calcule ensuite sur la taille et le bassin MESURÉS (_planifier : les
        articulations fixes et la hauteur du bassin lues), jamais sur la
        consigne : la marche ne suit la taille qu'en partie (65 à 95 %, banc
        de posture du 20 novembre 2026)."""
        d, raison = self.distance_de_prise(objet, d_min=d_min, cap=cap)
        # Mains plus haut que l'objet (règle de Kenzo : à sa hauteur, à 2 cm
        # près, avant de serrer) : les jambes doivent descendre d'au moins ça.
        besoin = self._abaissement_hauteur(objet)
        if (d is not None and besoin <= 0.0) or self._posture is None:
            return (d if besoin <= 0.0 else None), 0.0, 0.0, (
                raison if besoin <= 0.0 else f"mains {100 * besoin:.0f} cm au-dessus de l'objet, "
                                             f"et pas de consigne de posture")
        if d is None and besoin <= 0.0:
            besoin = 0.0
        z = float(objet.centre[2]) - float(self.mesures.bassin_z())
        dz = hauteur_prise(self.r, objet.hauteur) or 0.0
        dernier = ""
        for a in self.ABAISSEMENTS_ESSAYES:
            if a < besoin:
                continue
            d_a, _ = self.distance_de_prise(objet, d_min=d_min, cap=cap, abaissement=a)
            if d_a is None:
                continue
            port = self._port_penche(0.0)
            q, _ = resoudre(self.mesures.modele, *cibles_mains(
                np.array([d_a - self._bout, 0.0, z + a + dz]), objet.largeur, 0.0,
                self.r.demi_main_m), port)
            r = self._posture(0.0, a, angles={**port, **(q or {})},
                              charges=[(self.r.masse_max_kg, (d_a, 0.0, z + a))], essai=True)
            if r.ok:
                return d_a, 0.0, a, f"jambes pliées, bassin abaissé de {100 * a:.1f} cm : {r.reason}"
            dernier = r.reason
        if besoin > 0.0:
            return None, 0.0, 0.0, (f"objet trop bas : mains {100 * besoin:.0f} cm au-dessus de "
                                    f"lui, jambes pliées jusqu'à "
                                    f"{100 * self.ABAISSEMENTS_ESSAYES[-1]:.0f} cm"
                                    + (f" : {dernier}" if dernier else " sans prise possible"))
        d_t, t, raison_t = self.plan_penche(objet, cap=cap)
        if d_t is not None and d_t >= d_min:
            return d_t, t, 0.0, raison_t
        return None, 0.0, 0.0, raison_t + (f" ; jambes pliées : {dernier}" if dernier else "")

    def _abaissement_hauteur(self, objet: Objet) -> float:
        """De combien (m) les mains de la pré-saisie (sa hauteur fixe, d'où
        prendre part : _ouverture) dépasseront le dessus de l'objet (2 cm de
        tolérance de la règle de Kenzo, 1 cm de marge), le robot droit : ce
        que les jambes doivent descendre ; 0 si elles sont à sa hauteur.
        Calculé sur le modèle des bras, pas sur les mains du moment (au
        démarrage, les bras ne sont pas encore en posture)."""
        if self.mesures is None:
            return 0.0
        p = self.p
        g, d = self._modele.mains({**self._articulations_fixes(),
                                   **posture_bras(p.roulis_presaisie, p.tangage_port, p.coude)})
        mains_z = float(self.mesures.bassin_z()) + (float(g[2]) + float(d[2])) / 2.0
        dessus = float(objet.centre[2]) + objet.hauteur / 2.0 + 0.02
        return max(0.0, mains_z - dessus + 0.01)

    def changer_tangage(self, tangage: float) -> ServiceResult:
        """Objet tenu (TENU, prise mesurée en corps rigide) : pencher ou
        redresser le torse sans que l'objet bouge, ni en position ni en
        orientation (repère du bassin). Tout le chemin de la taille est
        vérifié d'avance (les bras le tiennent à chaque centième de radian) ;
        puis la consigne de posture de la marche (bornes, gardien, équilibre
        avec l'objet pesé) ; à chaque pas ensuite, les bras suivent la taille
        mesurée. Les bras n'y arrivant plus en chemin : la taille s'arrête où
        elle est, l'objet tenu."""
        if self.etat is not Etat.TENU or self._rigide is None:
            return ServiceResult(ok=False, reason=f"tangage refusé : objet tenu en corps rigide "
                                                  f"exigé (état {self.etat.name})")
        if self._posture is None:
            return ServiceResult(ok=False, reason="tangage refusé : pas de consigne de posture")
        t0 = float(self._articulations_fixes().get(self.TAILLE, 0.0))
        fixes, q = dict(self._fixes), dict(self._q)
        n = max(1, math.ceil(abs(tangage - t0) / 0.01))
        try:
            for k in range(1, n + 1):
                self._fixes = {**fixes, self.TAILLE: t0 + (k / n) * (tangage - t0)}
                q = self._q_rigide(self._centre_porte, graine=q)
                if q is None:
                    return ServiceResult(ok=False, reason=(
                        f"tangage refusé : les bras ne tiennent plus l'objet en place, torse à "
                        f"{math.degrees(t0 + (k / n) * (tangage - t0)):.0f}° (chemin vérifié "
                        f"avant de bouger, l'objet reste tenu là où il est)"))
        finally:
            self._fixes = fixes
        masse = (self.poids_tare if self.poids_tare is not None else
                 self.poids_carton if self.poids_carton is not None else
                 9.81 * self.r.masse_max_kg) / 9.81
        r = self._posture(tangage, 0.0, angles={**fixes, **q},
                          charges=[(max(0.0, masse), tuple(self._centre_porte))])
        if not r.ok:
            return ServiceResult(ok=False, reason=f"tangage refusé — {r.reason}")
        self._tangage_vise = float(tangage)
        self._noter(f"tangage du torse, objet tenu : {r.reason}")
        return ServiceResult(ok=True, reason=r.reason)

    def _masse_tenue(self) -> Tuple[float, Tuple[float, float, float]]:
        """(masse en kg, centre dans le repère du bassin) de l'objet tenu, pour
        l'équilibre : son poids pesé (sinon masse_max_kg), sous les mains de
        la hauteur de prise."""
        masse = (self.poids_tare if self.poids_tare is not None else
                 self.poids_carton if self.poids_carton is not None else
                 9.81 * self.r.masse_max_kg) / 9.81
        c = np.array(self._centre_porte, dtype=float) - np.array([0.0, 0.0, self._dz_prise or 0.0])
        return max(0.0, masse), tuple(float(v) for v in c)

    def changer_abaissement(self, abaissement: float) -> ServiceResult:
        """Objet tenu (TENU, prise mesurée en corps rigide) : plier ou déplier
        les jambes. Le bassin descend ou remonte, l'objet avec lui (il ne
        bouge pas dans le repère du bassin : les bras ne bougent pas). Par la
        consigne de posture de la marche (bornes, gardien, équilibre avec
        l'objet pesé) ; le torse reste où il est (taille MESURÉE)."""
        if self.etat is not Etat.TENU or self._rigide is None:
            return ServiceResult(ok=False, reason=f"abaissement refusé : objet tenu en corps "
                                                  f"rigide exigé (état {self.etat.name})")
        if self._posture is None:
            return ServiceResult(ok=False, reason="abaissement refusé : pas de consigne de posture")
        # Le torse reste où il est, droit sous 1 mrad : penché d'un bruit de
        # mesure, la marche le croirait penché et refuserait de partir.
        t = float(self._articulations_fixes().get(self.TAILLE, 0.0))
        t = t if t > 1e-3 else 0.0
        masse, centre = self._masse_tenue()
        r = self._posture(t, float(abaissement), angles={**self._fixes, **self._q},
                          charges=[(masse, centre)])
        if not r.ok:
            return ServiceResult(ok=False, reason=f"abaissement refusé — {r.reason}")
        self._noter(f"jambes, objet tenu : {r.reason}")
        return ServiceResult(ok=True, reason=r.reason)

    def _depose_possible(self, surface: float, abaissement: float = 0.0
                         ) -> Tuple[Optional[Dict[str, float]], str]:
        """Le chemin de la dépose sur un plateau à `surface` (m, monde), sans
        bouger : devant, puis vers le bas, en corps rigide, et l'ouverture des
        mains en bas, à quelques mm près ; le bassin abaissé de `abaissement`
        (m) de plus qu'il ne l'est (jambes pliées : le plateau d'autant plus
        haut dans le repère du bassin). (angles en bas, "") ou (None, pourquoi)."""
        p = self.p
        x_prise = float(self._plan["centre"][0]) if self._plan else 0.29
        de = np.array(self._centre_porte, dtype=float)
        devant = np.array([x_prise, de[1], de[2]])
        bas = np.array([x_prise, de[1], surface + p.jeu_depose - float(self.mesures.bassin_z())
                        + float(abaissement)
                        + (self.objet.hauteur / 2.0 if self.objet else 0.1) + (self._dz_prise or 0.0)])
        # Au pas de la descente (1 à 2,5 mm par tour) : un coude qui tourne
        # vite sans à-coup (0,16 rad au dernier cm vers 70 cm) n'y passe pas
        # pour un saut ; un changement de solution, si (0,32 rad en 2 mm).
        q = self._chemin_rigide(de, devant, None, pas_m=0.002)
        q = None if q is None else self._chemin_rigide(devant, bas, None, graine=q, pas_m=0.002)
        # En bas, les mains doivent aussi pouvoir s'ouvrir sur place, comme
        # _ouvrir le fera (serrage relâché, jeu_ouverture_m de chaque côté) :
        # sinon il rouvre les roulis et elles remontent en s'ouvrant (2,4 cm
        # vers un plateau de 71 cm, carton lâché ~3 cm au-dessus de lui).
        # _ouvrir part des mains mesurées, à quelques mm de leur consigne
        # (bras qui cèdent sous la charge : 3 à 5 mm en avant et de côté,
        # campagne de cartons variés), et l'ouverture d'un carton tourné en
        # dépend (12° : possible 4 mm plus près du robot, pas à la consigne) :
        # refus seulement si aucune de ces places ne le permet.
        ecart = (self._ecart_mains or 0.33) + 2 * self.r.jeu_ouverture_m
        relache = {c: 0.0 for c in self._ROULIS}
        ouvre = q is not None and any(
            self._q_rigide(bas + np.array([dx, dy, 0.0]), ecart, graine=q,
                           delta=relache) is not None
            for dx in (0.0, -0.005, 0.005) for dy in (0.0, -0.005, 0.005))
        if ouvre:
            return q, ""
        return None, ("hors d'atteinte des bras" if q is None else
                      "les mains ne pourraient pas s'y ouvrir sur place, elles remonteraient en "
                      "s'ouvrant")

    def abaissement_pour_deposer(self, hauteur_surface: float) -> Tuple[Optional[float], str]:
        """Objet tenu en corps rigide : de combien plier les jambes (m) pour le
        poser sur un plateau à `hauteur_surface` (m, monde), SANS bouger :
        0 si debout suffit ; sinon le plus petit abaissement
        (ABAISSEMENTS_ESSAYES) d'où le chemin de la dépose et l'ouverture
        passent (_depose_possible), que la marche accepte l'objet aux mains
        (consigne de posture, en essai : bornes, gardien, équilibre), et qui
        ne pose pas l'objet sur le plateau en descendant. (None, raison)
        sinon."""
        if self.etat is not Etat.TENU:
            return None, f"rien en main (état {self.etat.name})"
        if self._rigide is None or self.mesures is None:
            return 0.0, ""                   # sans corps rigide : debout, comme avant
        q, pourquoi = self._depose_possible(hauteur_surface)
        if q is not None:
            return 0.0, ""
        if self._posture is None:
            return None, f"dépose impossible : plateau à {hauteur_surface:.2f} m, {pourquoi}"
        masse, centre = self._masse_tenue()
        t = max(0.0, float(self._articulations_fixes().get(self.TAILLE, 0.0)))
        dernier = pourquoi
        for a in self.ABAISSEMENTS_ESSAYES:
            if self._fond_objet() - a < hauteur_surface + self.p.jeu_depose:
                dernier = "jambes pliées, l'objet toucherait le plateau"
                break
            q, pourquoi = self._depose_possible(hauteur_surface, a)
            if q is None:
                dernier = pourquoi
                continue
            r = self._posture(t, a, angles={**self._fixes, **self._q}, charges=[(masse, centre)],
                              essai=True)
            if r.ok:
                return a, f"jambes pliées, bassin abaissé de {100 * a:.1f} cm : {r.reason}"
            dernier = r.reason
        return None, (f"dépose impossible : plateau à {hauteur_surface:.2f} m, {dernier} (debout "
                      f"et jambes pliées jusqu'à {100 * self.ABAISSEMENTS_ESSAYES[-1]:.0f} cm)")

    def _redressable(self, centre: np.ndarray, graine: Dict[str, float],
                     t0: Optional[float] = None) -> bool:
        """Torse penché (taille MESURÉE) : l'objet tenu en `centre` (repère du
        bassin) le reste-t-il pendant que le torse se redresse
        (changer_tangage(0) : les bras le tiennent à chaque centième de
        radian) ? Toujours vrai droit. Prise en pendule, mains plus haut : le
        retour vers le buste allait au plus près que les bras permettent, d'où
        le torse ne se redressait plus (torse à 2°)."""
        if t0 is None:
            t0 = float(self._articulations_fixes().get(self.TAILLE, 0.0))
        if t0 <= 1e-3:
            return True
        fixes, q = dict(self._fixes), dict(graine)
        n = max(1, math.ceil(t0 / 0.01))
        try:
            for k in range(1, n + 1):
                self._fixes = {**fixes, self.TAILLE: t0 * (1.0 - k / n)}
                q = self._q_rigide(centre, graine=q)
                if q is None:
                    return False
        finally:
            self._fixes = fixes
        return True

    @property
    def tangage_taille(self) -> Optional[float]:
        """Tangage de la taille MESURÉ (rad), None s'il n'est pas lu."""
        v = self._articulations_fixes().get(self.TAILLE)
        return None if v is None else float(v)

    @property
    def tangage_en_cours(self) -> bool:
        """Un changement de tangage, objet tenu, n'est pas fini."""
        return self._tangage_vise is not None

    def _pas_tangage(self) -> None:
        """TENU, tangage en cours : les bras suivent la taille mesurée, l'objet
        gardé là où il est (repère du bassin)."""
        mesure = self._articulations_fixes()
        self._fixes = mesure
        q = self._q_rigide(self._centre_porte, graine=self._q)
        t = float(mesure.get(self.TAILLE, 0.0))
        if q is None:
            self._posture(max(0.0, t), 0.0)     # la taille s'arrête où elle est
            self.raison = (f"tangage arrêté à {math.degrees(t):.0f}° : les bras ne tiennent plus "
                           f"l'objet en place, il reste tenu")
            self._noter(self.raison)
            self._tangage_vise = None
            return
        self._ecrire_q(q)
        self._q_tenu = dict(q)
        if abs(t - self._tangage_vise) < 1e-3:
            self._tangage_vise = None

    def mains_a_hauteur(self, objet: Objet) -> Tuple[bool, str]:
        """Les mains sont-elles à la hauteur de l'objet (entre son dessous et
        son dessus, à 2 cm près) ? C'est la condition de hauteur de la règle
        de Kenzo (voir entre_les_mains), qui borne l'exemption du gardien."""
        g, d = self._mains()
        og, od = self._decalages_contact()
        mains_z = (float(g[2]) + float(og[2]) + float(d[2]) + float(od[2])) / 2.0
        dessous = float(objet.centre[2]) - objet.hauteur / 2.0 - 0.02
        dessus = float(objet.centre[2]) + objet.hauteur / 2.0 + 0.02
        if not (dessous < mains_z < dessus):
            return False, (f"mains à {mains_z:.2f} m de haut, objet entre "
                           f"{dessous + 0.02:.2f} et {dessus - 0.02:.2f} m")
        return True, ""

    # ---- API offerte aux apps --------------------------------------------

    def ouverture_presaisie(self, objet: Objet, de_face: bool = False
                            ) -> Tuple[Optional[Tuple[float, float]], str]:
        """Roulis et tangage d'épaule de la pré-saisie pour cet objet (voir
        Postures, roulis_presaisie_max) : ses mains ouvertes laissent entre les
        paumes sa largeur vue de face plus jeu_ouverture_m de chaque côté, à
        la hauteur de la pré-saisie fixe. (None, raison) s'il est trop large
        pour les mains ouvertes en grand. Son lacet, vu du robot tel qu'il est
        tourné ; `de_face` : sans lacet (approche pas encore faite)."""
        r = self.r
        a = 0.0 if de_face else self._lacet_relatif(objet)
        face = objet.largeur * abs(math.cos(a)) + objet.profondeur * abs(math.sin(a))
        pose, libre = self._ouverture(face + 2.0 * r.jeu_ouverture_m)
        if pose is not None:
            return pose, ""
        return None, (f"trop large pour les mains : {face:.2f} m vu de face (lacet "
                      f"{math.degrees(a):+.0f}°) ; ouvertes en grand, elles laissent "
                      f"{libre:.2f} m entre les paumes, il faut {100 * r.jeu_ouverture_m:.0f} cm "
                      f"de jeu de chaque côté pour les glisser autour")

    def _ouverture(self, besoin: float) -> Tuple[Optional[Tuple[float, float]], float]:
        """(roulis, tangage) d'épaule de la pré-saisie qui laissent `besoin`
        (m) entre les paumes, à la hauteur de la pré-saisie fixe, ou None si
        elles n'y arrivent pas ouvertes en grand ; et l'écart obtenu."""
        p, r = self.p, self.r
        fixes = self._articulations_fixes()

        def mains(roulis, tangage):
            g, d = self._modele.mains({**fixes, **posture_bras(roulis, tangage, p.coude)})
            return (float(g[2]) + float(d[2])) / 2.0, float(g[1] - d[1]) - 2.0 * r.demi_main_m
        z0, _ = mains(p.roulis_presaisie, p.tangage_port)
        libre = 0.0
        roulis, tangage = p.roulis_presaisie, p.tangage_port
        while True:
            for _ in range(6):                    # même hauteur des mains (sécante)
                z, libre = mains(roulis, tangage)
                if abs(z - z0) < 0.002:
                    break
                pente = (mains(roulis, tangage + 0.01)[0] - z) / 0.01
                if abs(pente) < 1e-6:
                    break
                tangage -= (z - z0) / pente
            if libre >= besoin:
                return (roulis, tangage), libre
            if roulis >= p.roulis_presaisie_max:
                return None, libre
            roulis = min(p.roulis_presaisie_max, roulis + 0.02)

    def presaisir(self, objet: Optional[Objet] = None) -> ServiceResult:
        """Ouvre les bras en grand avant l'approche finale, robot à
        l'arrêt. Non bloquant : suivre l'état jusqu'à PRET. `objet` (prise
        mesurée) : ouverts juste assez pour lui (ouverture_presaisie),
        refusé s'il est trop large ; sinon, roulis_presaisie, comme avant."""
        if not self._demarre:
            return ServiceResult(ok=False, reason="service non démarré")
        if self.etat is Etat.PRET:
            return ServiceResult(ok=True)
        if self.etat is not Etat.LIBRE:
            return ServiceResult(ok=False, reason=f"occupé : {self.etat.name}")
        pose = (self.p.roulis_presaisie, self.p.tangage_port)
        if objet is not None and self.mesures is not None:
            pose, raison = self.ouverture_presaisie(objet)
            if pose is None:
                return ServiceResult(ok=False, reason=f"prise refusée — {raison}")
        self._roulis_presaisie, self._tangage_presaisie = pose
        self._aller(Etat.PRESAISIE)
        return ServiceResult(ok=True)

    @sous_verrou
    def calibrer_charge(self, objet: Objet, masse_kg: float,
                        enregistrer: Optional[Callable[[dict], None]] = None) -> ServiceResult:
        """Calibration de charge, une fois par robot (première mise en route :
        docs/PREMIERE_MISE_EN_ROUTE.md). `objet` est une masse de référence de
        `masse_kg` connue, charge centrée (un carton lesté en son milieu),
        posée là où l'on prend un carton. Elle est prise comme un carton
        (prendre : la prise de T1), levée et pesée bras immobiles, plus
        longtemps (ReglagesPrise.lectures_calibration_charge), puis reposée là
        où elle a été prise, ni serrée plus, ni portée. Son poids, partagé
        moitié-moitié entre les deux mains, donne le gain du couple estimé de
        chaque coude (prise.gain_coude), appliqué aussitôt (gains_couples) et
        passé à `enregistrer` avec sa date (le profil du robot :
        profil_robot.enregistrer). Refus ou échec : les gains d'avant restent.
        Non bloquant, comme prendre : fini quand l'état revient à LIBRE
        (calibration_charge à jour), ou passe à ECHEC (raison)."""
        r = self.r
        if not r.masse_reference_min_kg <= masse_kg <= r.masse_max_kg:
            return ServiceResult(ok=False, reason=(
                f"calibration de charge refusée — masse de référence de {masse_kg:g} kg : entre "
                f"{r.masse_reference_min_kg:g} et {r.masse_max_kg:g} kg (masse_reference_min_kg, "
                f"masse_max_kg, la charge pour laquelle la prise est vérifiée)"))
        if self.mesures is None or self.couples_fiables is False:
            return ServiceResult(ok=False, reason=(
                "calibration de charge refusée — il faut les couples estimés des bras, et "
                "fiables (calibration des couples)"))
        avant = (dict(self.gains_couples), self.calibration_charge)
        self._calibration_en_cours = {"masse_kg": float(masse_kg), "enregistrer": enregistrer,
                                      "avant": avant}
        self.gains_couples = {}           # la masse pesée sur les couples bruts
        res = self.prendre(objet)
        if not res.ok:
            self.gains_couples, self.calibration_charge = avant
            self._calibration_en_cours = None
            return ServiceResult(ok=False, reason=f"calibration de charge : {res.reason}")
        self._noter(f"calibration de charge : masse de référence de {masse_kg:g} kg, prise comme "
                    f"un carton")
        return res

    @sous_verrou
    def appliquer_calibration_charge(self, cal: Optional[dict]) -> str:
        """La calibration de charge du profil du robot (la valeur
        « calibration_charge » de profil_robot.lire_profil), ou None. Sans
        elle, la pesée se fait sans corriger les gains des coudes, comme
        avant, et l'OS le dit ; illisible, ou des gains hors de
        [gain_coude_min, gain_coude_max] : pas appliquée non plus (dit aussi).
        Rend le texte, aussi écrit au journal."""
        r = self.r
        self.gains_couples, self.calibration_charge = {}, None
        gains = None
        if cal is not None:
            try:
                gains = {str(j): float(v) for j, v in cal["gains_couples"].items()}
                date, masse = str(cal["date"]), float(cal["masse_kg"])
            except (KeyError, TypeError, ValueError, AttributeError):
                gains = None
        if cal is None:
            texte = ("pas de calibration de charge dans le profil du robot : pesée sans corriger "
                     "les gains des coudes, comme avant ; à faire une fois, à la première mise en "
                     "route (docs/PREMIERE_MISE_EN_ROUTE.md)")
        elif not gains or any(not j.endswith("_elbow_joint") or
                              not r.gain_coude_min <= v <= r.gain_coude_max
                              for j, v in gains.items()):
            texte = (f"calibration de charge du profil du robot NON APPLIQUÉE (illisible, ou gains "
                     f"hors de [{r.gain_coude_min:g}, {r.gain_coude_max:g}]) : pesée sans corriger "
                     f"les gains des coudes, comme avant ; à refaire")
        else:
            self.gains_couples, self.calibration_charge = gains, dict(cal)
            texte = (f"calibration de charge du {date} ({masse:g} kg de référence) : gains des "
                     f"couples estimés des coudes, "
                     + ", ".join(f"{'gauche' if j.startswith('left') else 'droit'} {v:.3f}"
                                 for j, v in sorted(gains.items())))
        self.stats["calibration_charge"] = (self.calibration_charge or {}).get("date")
        self._noter(texte)
        return texte

    @sous_verrou
    def prendre(self, objet: Objet) -> ServiceResult:
        """Lance la prise de l'objet. Non bloquant : suivre la progression
        par `etat`, ou par le résultat de tick(). Refusée si l'objet n'est
        pas entre les mains, ou si le gardien refuse l'exemption."""
        if not self._demarre:
            return ServiceResult(ok=False, reason="service non démarré")
        if self.etat not in (Etat.LIBRE, Etat.PRET):
            return ServiceResult(ok=False, reason=f"occupé : {self.etat.name}")
        # Prise mesurée : les bras iront eux-mêmes aux faces de l'objet ; on
        # exige les mains à sa hauteur (condition de l'exemption, inchangée)
        # et qu'il soit à portée. En boucle ouverte : entre les mains.
        ok, raison = (self.mains_a_hauteur(objet) if self.mesures is not None
                      else self.entre_les_mains(objet))
        if not ok:
            self.stats["refus"] += 1
            return ServiceResult(ok=False, reason=f"prise refusée — {raison}")
        if self.mesures is not None:
            if self.calibration_refus:
                self.stats["refus"] += 1
                return ServiceResult(ok=False, reason=f"prise refusée — {self.calibration_refus}")
            self._init_mesure()
            ok, raison = self._planifier(objet)
            if not ok:
                self.stats["refus"] += 1
                self._journaliser_echec(f"prise refusée — {raison}", objet)
                return ServiceResult(ok=False, reason=f"prise refusée — {raison}")
            tau, raison = self._efforts()
            if tau is None:
                self.stats["refus"] += 1
                return ServiceResult(ok=False, reason=f"serrage refusé — {raison}")
            trop = self._effort_excessif(tau)
            angles = self._angles_mesures()
            if not trop and angles is not None:
                g = self._gravite_bassin()
                for cote, nom in (("left", "G"), ("right", "D")):
                    f = self._force_main(cote, tau, angles, g, depuis_fermeture=False)
                    if f is not None and float(np.linalg.norm(f)) > self.r.force_deja_n:
                        trop = f"main {nom} poussée à {float(np.linalg.norm(f)):.0f} N"
            if trop:
                self.stats["refus"] += 1
                return ServiceResult(ok=False, reason=f"serrage refusé — bras déjà en effort "
                                                      f"({trop})")
        if self._gardien is not None:
            # Exemption : le volume de l'objet, élargi de 3 cm de chaque
            # côté et de 10 cm de plus vers le haut, où il monte quand on le
            # soulève ; pendant 15 s au plus.
            cx, cy, cz = objet.centre
            m = 0.03
            dx, dy, dz = (objet.profondeur / 2 + m, objet.largeur / 2 + m,
                          objet.hauteur / 2 + m)
            ok_e, r_e = self._gardien.exempter(
                (cx - dx, cx + dx, cy - dy, cy + dy, cz - dz, cz + dz + 0.10),
                duree_s=15.0, maintenant=self._horloge(),
                motif="manipulation : prise")
            if not ok_e:
                self.stats["refus"] += 1
                return ServiceResult(ok=False, reason=f"exemption refusée — {r_e}")
        self.objet, self.raison, self._franchi = objet, "", None
        if self.mesures is not None:
            # Référence du contrôle géométrique d'appui : prise au début du
            # placement, bras posés (_poser_bras), pas maintenant.
            self._hors_consigne0, self._attente_pose = None, None
            # Gains raidis dès le placement : le bras se pose avec eux, et le
            # zéro des couples, avant de fermer, se prend bras immobile.
            self._regler_raideur(True)
            self._vers_placement(self._plan["q_ouvert"], self.r.duree_placement_s)
            return ServiceResult(ok=True)
        # Le serrage part de l'ouverture où sont les bras : large après une
        # pré-saisie, normale sinon.
        self._roulis_depart_serrage = (self._roulis_presaisie if self.etat is Etat.PRET
                                       else self.p.roulis_ouvert)
        self._aller(Etat.SERRAGE)
        return ServiceResult(ok=True)

    def _hauteur_contacts(self) -> float:
        """Hauteur moyenne des points de contact des mains mesurées (m,
        repère monde) : leur repère, plus le décalage vers le contact."""
        g, d = self._mains()
        og, od = self._decalages_contact()
        return (float(g[2]) + float(og[2]) + float(d[2]) + float(od[2])) / 2.0

    def _fond_objet(self) -> float:
        """Hauteur estimée du fond de l'objet : en prise mesurée, celle des
        points de contact, moins leur hauteur au-dessus de son centre à la
        prise (_dz_prise), moins sa demi-hauteur ; sinon, hauteur moyenne des
        deux mains moins la demi-hauteur (rien en main : celle des mains).
        Pourquoi : avec les mains du G1 (contact 11 cm le long de la main),
        les repères des mains donnaient le fond de 2 cm trop haut (carton de
        13 cm : enfoncé d'autant dans la table à la dépose) à 3 cm trop bas
        (carton de 30 cm : « déjà sous le plateau », pas reposé sur sa table ;
        campagne de cartons variés, axis_os/testing/cartons.py)."""
        h = self.objet.hauteur if self.objet is not None else 0.0
        if self.objet is not None and self.mesures is not None and self._dz_prise is not None:
            return self._hauteur_contacts() - self._dz_prise - h / 2.0
        g, d = self._mains()
        return (float(g[2]) + float(d[2])) / 2.0 - h / 2.0

    def degage_au_dessus(self, hauteur_surface: float) -> bool:
        """L'objet porté passe-t-il au-dessus d'un plateau à cette hauteur,
        avec la marge pleine (`marge_franchissement`) ?"""
        return self._fond_objet() >= hauteur_surface + self.p.marge_franchissement

    def peut_franchir(self, hauteur_surface: float) -> bool:
        """Vrai si l'objet est dégagé avec la marge pleine, ou si la
        dernière montée, terminée, a franchi CE plateau avec la marge
        réduite (`marge_franchissement_min`)."""
        return (self.degage_au_dessus(hauteur_surface)
                or (self.etat is Etat.TENU and self._franchi is not None
                    and abs(self._franchi - hauteur_surface) < 1e-6))

    def franchir(self, hauteur_surface: float) -> ServiceResult:
        """Soulève l'objet tenu au-dessus d'un plateau avant d'avancer vers
        lui, comme le fait quelqu'un qui pose un carton sur une table : sans
        cela, le carton bute contre le bord. Non bloquant : état MONTEE,
        puis TENU. Si l'objet est déjà assez haut, il n'y a rien à faire.
        Si le bras atteint sa limite sans dégager le plateau, l'état revient
        à TENU et `raison` dit pourquoi (peut_franchir() reste faux)."""
        if self.etat is not Etat.TENU:
            return ServiceResult(ok=False, reason="rien en main")
        if self.degage_au_dessus(hauteur_surface):
            return ServiceResult(ok=True, value="déjà au-dessus")
        self._surface, self._tangage_depart = hauteur_surface, self._tangage_courant
        self.raison, self._franchi, self._t_limite = "", None, None
        self._t_phase = None
        self._aller(Etat.MONTEE)
        return ServiceResult(ok=True)

    @sous_verrou
    def poser(self, hauteur_surface: Optional[float] = None) -> ServiceResult:
        """Sans plateau : redescend à hauteur de port et ouvre les bras.

        Avec `hauteur_surface` (en mètres, repère monde) : descend jusqu'à
        ce que le fond de l'objet effleure le plateau, puis ouvre, pour ne
        jamais lâcher l'objet de haut. Refusé si le fond de l'objet n'est
        pas déjà à au moins `jeu_depose` au-dessus du plateau (plateau trop
        haut : voir franchir()).
        """
        if self.etat is not Etat.TENU:
            return ServiceResult(ok=False, reason="rien en main")
        self.raison, self._franchi = "", None
        if hauteur_surface is None:
            self._surface, self._tangage_ouverture = None, self.p.tangage_port
            if self.mesures is not None and self._q_serre is not None:
                # Prise mesurée : l'objet revient là où il a été pris.
                self._q_de, self._q_vers = dict(self._q), dict(self._q_serre)
            self._aller(Etat.POSE)
            return ServiceResult(ok=True)
        fond = self._fond_objet()
        if fond < hauteur_surface + self.p.jeu_depose:
            return ServiceResult(ok=False, reason=(
                f"plateau trop haut ({hauteur_surface:.2f} m) : le fond de "
                f"l'objet est déjà à {fond:.2f} m"))
        self._surface = hauteur_surface
        self._tangage_depart = self._tangage_courant    # depuis la posture actuelle
        self._t_phase = None
        self._aller(Etat.DESCENTE)
        return ServiceResult(ok=True)

    @sous_verrou
    def reinitialiser(self, large: bool = False) -> None:
        """Après un échec : rouvre les bras en posture de port et lève
        l'exemption.

        large=True : ouverture grande (celle de la pré-saisie), pour lâcher
        un objet coincé entre les mains. Pourquoi : constaté sur GPU, lors
        d'un essai de 30 minutes de la démonstration de tri de colis
        (docs/demo_tri_usine.md). Après une prise ratée sur les coins d'un
        carton, les mains rouvertes à 33 cm le tenaient encore ; le robot a
        reculé en le tirant de 11 cm, jusqu'à le faire dépasser du bord de
        la table, puis s'est bloqué devant lui.

        Le geste est progressif (état LACHER, sur la durée d'ouverture),
        jamais un saut. Pourquoi : constaté sur GPU, le saut de « serré »
        (-0.40 rad) à « grand ouvert » (0.32 rad) dépassait l'écart maximal
        entre consigne et position (0.625 rad) admis par le JointSafetyGuard,
        le garde-fou qui contrôle chaque commande d'articulation
        (axis_os/hal/safety/limits.py). Résultat : 570 consignes refusées,
        bras restés serrés, carton tiré hors de la table et tombé."""
        self._abandonner_calibration("prise réinitialisée")
        self._calibration_finie = False
        self._lever_exemption()
        self.objet, self._franchi = None, None
        self._lacher_de = (self._roulis_courant, self._tangage_courant)
        # Grande : au moins celle de la dernière pré-saisie (un carton plus large).
        self._lacher_vers = (max(self.p.roulis_presaisie, self._roulis_presaisie) if large
                             else self.p.roulis_ouvert,
                             self.p.tangage_port)
        # Depuis la consigne réelle (le coude et chaque roulis peuvent avoir
        # bougé seuls en prise mesurée) ; posture_bras est linéaire, donc en
        # boucle ouverte c'est exactement le geste d'avant.
        self._q_de = dict(self._q)
        self._q_vers = {**self._libres0, **posture_bras(*self._lacher_vers, self.p.coude)}
        self._init_mesure()
        self._regler_raideur(False)
        self._aller(Etat.LACHER)

    def resynchroniser(self) -> None:
        """Reprend la main sur les bras après un autre écrivain.

        Cas visé : le VLA (modèle vision-langage-action, une IA qui commande
        les gestes à partir de ce qu'elle voit et d'une instruction) a tenu
        les bras, puis les a ramenés en posture de port, mains vides. Le
        service reprend la main sans saut : il considère les bras là où ils
        sont vraiment (roulis ouvert, tangage de port), passe à l'état
        LIBRE et lève l'exemption. Pourquoi : sans cela, le geste suivant
        partirait d'une posture périmée, d'où un saut de consigne que le
        JointSafetyGuard refuserait."""
        self._lever_exemption()
        self.objet, self._franchi, self.raison = None, None, ""
        self._roulis_courant, self._tangage_courant = self.p.roulis_ouvert, self.p.tangage_port
        self._q = posture_bras(self.p.roulis_ouvert, self.p.tangage_port, self.p.coude)
        self._libres0 = {}
        self._tangage_ouverture = self.p.tangage_port
        self._init_mesure()
        self._regler_raideur(False)
        self.etat = Etat.LIBRE

    # ---- boucle : un pas à chaque appel de tick() ------------------------

    # États où les bras bougent : ceux qu'un arrêt de sécurité doit figer.
    _EN_MOUVEMENT = (Etat.PRESAISIE, Etat.MONTEE, Etat.SERRAGE, Etat.LEVAGE, Etat.POSE,
                     Etat.OUVERTURE, Etat.DESCENTE, Etat.LACHER, Etat.PLACEMENT, Etat.FERMETURE,
                     Etat.REPOSE, Etat.RAMENER)

    @sous_verrou
    def tick(self) -> ServiceResult:
        """Un pas de la machine à états, à appeler régulièrement : écrit la
        consigne des bras pour ce pas. Rend l'état dans `value` et, s'il y
        en a une, la raison d'un échec ou d'une limite atteinte dans
        `reason`. `ok` n'est faux qu'en état ECHEC."""
        p = self.p
        maintenant = self._horloge()
        dt = 0.0 if self._dernier_tick is None else maintenant - self._dernier_tick
        self._dernier_tick = maintenant
        if self.raideur_active and self.mesures is not None \
                and self.mesures.arret_urgence is not None:
            try:
                urgence = bool(self.mesures.arret_urgence())
            except Exception:             # noqa: BLE001 — illisible : on rend les gains
                urgence = True
            if urgence:
                self._regler_raideur(False)      # arrêt d'urgence : gains normaux aussitôt
                self.stats["raideur_rendue_urgence"] = \
                    self.stats.get("raideur_rendue_urgence", 0) + 1
        if self.etat in self._EN_MOUVEMENT and self._arret_securite is not None \
                and self._arret_securite():
            # Gel : on n'écrit rien, donc les bras restent où ils sont, et on
            # décale l'origine du geste de la durée du gel pour qu'il
            # reprenne sans à-coup, là où il s'était arrêté.
            self._t0 += dt
            self.stats["gels_securite"] += 1
            return ServiceResult(ok=True, value=self.etat,
                                 reason="bras figés : arrêt de sécurité en cours")
        if self.mesures is not None and self._pas_mesure(dt):
            pass
        elif self.etat is Etat.LACHER:
            q, fini = self._interp_q(p.duree_ouverture_s)
            self._ecrire_q(q)
            if fini:
                # Lacet d'épaule et poignets revenus à leur consigne d'avant :
                # la prise suivante les relira mesurés.
                for j in self._libres0:
                    self._q.pop(j, None)
                self._libres0 = {}
                self.etat = Etat.LIBRE
        elif self.etat is Etat.PRESAISIE:
            # Plus grande, l'ouverture va au plus à vitesse_fermeture.
            duree = max(p.duree_presaisie_s,
                        (self._roulis_presaisie - p.roulis_ouvert) / self.r.vitesse_fermeture)
            r, fini = self._interp(p.roulis_ouvert, self._roulis_presaisie, duree)
            tg, _ = self._interp(p.tangage_port, self._tangage_presaisie, duree)
            self._ecrire(r, tg)
            if fini:
                self.etat = Etat.PRET
        elif self.etat is Etat.SERRAGE:
            r, fini = self._interp(self._roulis_depart_serrage, p.roulis_serre,
                                   p.duree_serrage_s)
            self._ecrire(r, p.tangage_port)
            if fini:
                self._aller(Etat.LEVAGE)
        elif self.etat is Etat.LEVAGE:
            t, fini = self._interp(p.tangage_port, p.tangage_leve, p.duree_levage_s)
            self._ecrire(p.roulis_serre, t)
            if fini:
                confirme = self._verifier()
                if not confirme and (self._horloge() - self._t0 - p.duree_levage_s
                                     < p.attente_confirmation_s):
                    pass                     # le bras chargé n'a pas encore rejoint
                elif confirme:
                    self.stats["prises"] += 1
                    # L'objet a quitté la table : l'exemption n'a plus lieu
                    # d'être. Le carton porté est couvert par le volume de
                    # port du gardien (la zone devant le torse, à hauteur
                    # des bras, où il ignore ce que le robot porte).
                    self._lever_exemption()
                    self.etat = Etat.TENU
                else:
                    self.stats["prises_ratees"] += 1
                    self.raison = "prise NON confirmée — l'objet n'a pas suivi"
                    self._lever_exemption()
                    # Pas de saut vers la posture de port (0.6 rad d'un coup,
                    # à la limite du JointSafetyGuard) : on reste en ECHEC,
                    # et c'est reinitialiser() qui rouvre progressivement.
                    self.etat = Etat.ECHEC
        elif self.etat is Etat.POSE:
            t, fini = self._interp(p.tangage_leve, p.tangage_port, p.duree_pose_s)
            self._ecrire(p.roulis_serre, t)
            if fini:
                self._aller(Etat.OUVERTURE)
        elif self.etat is Etat.MONTEE:
            # Montée à vitesse constante, plafonnée à `tangage_haut`, jusqu'à
            # ce que l'objet dégage le plateau avec la marge pleine.
            maintenant = self._horloge()
            t = max(p.tangage_haut,
                    self._tangage_depart - p.vitesse_montee * (maintenant - self._t0))
            self._ecrire(p.roulis_serre, t)
            if self.degage_au_dessus(self._surface):
                self.stats["franchissements"] = self.stats.get("franchissements", 0) + 1
                self._franchi = self._surface
                self.etat = Etat.TENU
            elif t <= p.tangage_haut:
                # La consigne a atteint sa limite : on laisse au bras chargé
                # le temps d'y arriver (`stabilisation_montee_s`), puis on
                # conclut avec la marge réduite.
                if self._t_limite is None:
                    self._t_limite = maintenant
                elif maintenant - self._t_limite >= p.stabilisation_montee_s:
                    fond = self._fond_objet()
                    if fond >= self._surface + p.marge_franchissement_min:
                        self.stats["franchissements"] = \
                            self.stats.get("franchissements", 0) + 1
                        self.stats["franchissements_justes"] = \
                            self.stats.get("franchissements_justes", 0) + 1
                        self._franchi = self._surface
                    else:
                        self.stats["franchissements_impossibles"] = \
                            self.stats.get("franchissements_impossibles", 0) + 1
                        self.raison = (
                            f"plateau à {self._surface:.2f} m trop haut pour y passer "
                            f"l'objet : bras au plus haut, fond du carton à "
                            f"{fond:.2f} m (il faut {self._surface + p.marge_franchissement_min:.2f})")
                    self.etat = Etat.TENU
        elif self.etat is Etat.DESCENTE:
            # Descente à vitesse constante, arrêtée dès que le fond de
            # l'objet, estimé par les mains, est à `jeu_depose` du plateau.
            t = self._tangage_depart + p.vitesse_descente * (self._horloge() - self._t0)
            if t >= p.tangage_bas:
                # Bras au plus bas sans que l'objet touche : le plateau est
                # hors d'atteinte. On ne lâche pas l'objet de haut : on
                # remonte en le tenant, et on le dit.
                self.stats["deposes_impossibles"] += 1
                self.raison = (f"dépose impossible : plateau à "
                               f"{self._surface:.2f} m hors d'atteinte des bras")
                self._ecrire(p.roulis_serre, p.tangage_leve)
                self.etat = Etat.TENU
            else:
                self._ecrire(p.roulis_serre, t)
                if self._fond_objet() <= self._surface + p.jeu_depose:
                    self._tangage_ouverture = t
                    self.stats["deposes_plateau"] += 1
                    self._aller(Etat.OUVERTURE)
        elif self.etat is Etat.OUVERTURE:
            r, fini = self._interp(p.roulis_serre, p.roulis_ouvert, p.duree_ouverture_s)
            self._ecrire(r, self._tangage_ouverture)
            if fini:
                self.stats["poses"] += 1
                self.objet = None
                # Mains ouvertes, l'objet est posé : retour en posture de port.
                self._ecrire(p.roulis_ouvert, p.tangage_port)
                self._tangage_ouverture = p.tangage_port
                self.etat = Etat.LIBRE
        return ServiceResult(ok=self.etat is not Etat.ECHEC, value=self.etat,
                             reason=self.raison or None)
