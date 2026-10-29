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
from dataclasses import dataclass
from enum import Enum, auto
from typing import Callable, Dict, Optional, Tuple

import numpy as np

from axis_os.hal.core.types import HALStatus
from axis_os.services.base import IService, ServiceHealth, ServiceResult, sous_verrou
from axis_os.store.sdk.manifest import Capability

from .prise import (
    MesuresBras, ReglagesPrise, cibles_mains, distance_ideale, efforts_valides, garde_plateau,
    positions_valides, resoudre,
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
    sa face avant au bord du plateau qui le porte (0 : au bord, comme sur
    les tables de tri) ; le bord n'est jamais pris à moins de
    ReglagesPrise.bord_min_m du bassin."""
    centre: Tuple[float, float, float]
    largeur: float
    profondeur: float
    hauteur: float
    lacet: float = 0.0
    bord_m: float = 0.0


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
                 journal: Optional[Callable[[str], None]] = None):
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
        """
        self._bras = bras
        self._mains = mains
        self._vers_robot = vers_robot
        self._verifier = verifier_prise
        self._gardien = gardien
        self._horloge = horloge
        self.p = postures
        self._journal = journal
        # Raison du refus de toute prise mesurée (calibration des mains
        # ratée, voir prise.CalibrationMains) ; vide : prise permise.
        self.calibration_refus = ""
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
        """Écrit une consigne articulation par articulation (prise mesurée)."""
        self._q = {j: float(v) for j, v in q.items()}
        self._tangage_courant = self._q.get("left_shoulder_pitch_joint", self._tangage_courant)
        self._roulis_courant = self._q.get("left_shoulder_roll_joint", self._roulis_courant)
        self._bras.update(self._q)

    def _interp_q(self, duree: float) -> Tuple[Dict[str, float], bool]:
        """Consigne qui va linéairement de `_q_de` à `_q_vers` en `duree`."""
        f = min(1.0, (self._horloge() - self._t0) / duree) if duree > 0 else 1.0
        return ({j: self._q_de[j] + f * (self._q_vers[j] - self._q_de[j]) for j in self._q_vers},
                f >= 1.0)

    def _interp_c(self, duree: float) -> Tuple[Dict[str, float], bool]:
        """Objet tenu : le milieu des mains va en ligne droite de `_c_de` à
        `_c_vers` en `duree`, le serrage gardé à chaque pas (une
        interpolation des angles le relâcherait en chemin)."""
        f = min(1.0, (self._horloge() - self._t0) / duree) if duree > 0 else 1.0
        q = self._q_mains(self._c_de + f * (self._c_vers - self._c_de), self._ecart_mains)
        if q is None:
            return self._interp_q(duree)
        return q, f >= 1.0

    def _interp(self, a: float, b: float, duree: float) -> Tuple[float, bool]:
        """Valeur qui va linéairement de a à b en `duree` secondes, depuis
        le début du geste en cours (self._t0), et vrai une fois `duree`
        écoulée."""
        f = min(1.0, (self._horloge() - self._t0) / duree)
        return a + f * (b - a), f >= 1.0

    def _aller(self, etat: Etat) -> None:
        """Passe dans `etat`, et note l'instant de départ du geste."""
        self.etat, self._t0 = etat, self._horloge()

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
        Rend (True, "") ou (False, raison du refus).
        """
        g, d = self._mains()
        gr, dr = self._vers_robot(g), self._vers_robot(d)
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

    # ---- prise mesurée (voir l'en-tête du module et prise.py) --------------

    # Roulis d'épaule de chaque bras, et sens de la fermeture (le bras
    # gauche se referme quand son roulis diminue, le droit quand il croît).
    _ROULIS = {"left": "left_shoulder_roll_joint", "right": "right_shoulder_roll_joint"}
    _FERME = {"left": -1.0, "right": 1.0}

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
        self._q_ref: Optional[Dict[str, float]] = None
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

    def _vers_bassin(self, p_monde) -> np.ndarray:
        """Point du repère monde -> repère du bassin (x, y du robot ; z
        au-dessus du bassin)."""
        p = np.asarray(p_monde, dtype=float)
        if self.mesures.vers_bassin is not None:
            return np.asarray(self.mesures.vers_bassin(p), dtype=float)
        r = self._vers_robot(p)
        return np.array([float(r[0]), float(r[1]), float(p[2]) - float(self.mesures.bassin_z())])

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
        r, modele = self.r, self.mesures.modele
        try:
            centre = self._vers_bassin(objet.centre)
            lacet = self._lacet_relatif(objet)
        except Exception as exc:          # noqa: BLE001 — pose ou hauteur illisible
            return False, f"position du robot illisible ({type(exc).__name__})"
        if not np.all(np.isfinite(centre)) or not math.isfinite(lacet):
            return False, "position de l'objet ou du robot non finie"
        self._fixes = self._articulations_fixes()
        q_ouvert, _ = resoudre(modele, *cibles_mains(centre, objet.largeur, lacet,
                                                    r.demi_main_m + r.jeu_ouverture_m),
                               self._graine(self._q))
        q_contact, _ = resoudre(modele, *cibles_mains(centre, objet.largeur, lacet,
                                                     r.demi_main_m), self._graine(self._q))
        q_vide, _ = resoudre(modele, *cibles_mains(centre, objet.largeur, lacet,
                                                  r.demi_main_m - r.penetration_max_m),
                             self._graine(self._q))
        if q_ouvert is None or q_contact is None or q_vide is None:
            return False, (f"hors de portée des bras : objet à {centre[0]:.2f} m devant le "
                           f"bassin et {centre[2]:+.2f} m en hauteur, les mains n'atteignent "
                           f"pas le milieu de ses faces")
        plateau = float(centre[2]) - objet.hauteur / 2.0
        bord = float(centre[0]) - objet.profondeur / 2.0 - objet.bord_m
        garde = garde_plateau(modele, self._graine(q_contact), plateau, max(r.bord_min_m, bord),
                              r.rayon_avant_bras_m)
        if garde < r.marge_plateau_m:
            return False, (f"avant-bras à {100 * garde:.1f} cm du plateau (il faut "
                           f"{100 * r.marge_plateau_m:.0f}) : changer la distance à la table")
        self._plan = {"centre": centre, "lacet": lacet, "q_ouvert": q_ouvert,
                      "q_contact": q_contact, "q_vide": q_vide, "largeur": objet.largeur,
                      "garde": garde,
                      "cibles": cibles_mains(centre, objet.largeur, lacet,
                                             r.demi_main_m + r.jeu_ouverture_m)}
        return True, ""

    def _echouer(self, raison: str) -> None:
        """Échec de la prise mesurée : on ne bouge plus (la consigne reste
        celle du dernier pas), on lève l'exemption ; reinitialiser() rouvre."""
        self.stats["prises_ratees"] += 1
        self.raison = raison
        self._lever_exemption()
        self.etat = Etat.ECHEC
        self._journaliser_echec(raison)

    def _mains_modele(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Mains calculées par la cinématique sur les angles MESURÉS
        (repère du bassin), ou None si les angles sont illisibles."""
        try:
            brut = self.mesures.positions()
            angles = {j: float(v) for j, v in brut.items()}
        except Exception:                 # noqa: BLE001
            return None
        if not all(math.isfinite(v) for v in angles.values()):
            return None
        g, d = self.mesures.modele.mains({**self._fixes, **angles})
        return g, d

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
            self._journal(f"prise ratée : {raison} — {self.diagnostic_mains(objet)}")
        except Exception:                 # noqa: BLE001 — le journal ne doit rien casser
            pass

    def _efforts(self) -> Tuple[Optional[Dict[str, float]], str]:
        tau, raison = efforts_valides(self.mesures, self._ROULIS.values(), self.r.age_max_effort_s)
        if tau is not None:
            self.effort_max_mesure = max([self.effort_max_mesure]
                                         + [abs(v) for v in tau.values()])
        return tau, raison

    def _limite_effort(self, j: str) -> float:
        c = self.mesures.couple_max.get(j)
        lim = self.r.effort_max_nm
        return min(lim, self.r.part_couple_garde * c) if c else lim

    def _effort_excessif(self, tau: Dict[str, float]) -> str:
        """« articulation à X N·m » si un roulis dépasse sa limite, sinon ""."""
        for j, v in tau.items():
            if abs(v) > self._limite_effort(j):
                return f"{j} à {abs(v):.1f} N·m, limite {self._limite_effort(j):.1f}"
        return ""

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
        self._prec_serrage = {}
        self._contact = {"left": None, "right": None}
        self._compte_contact = {"left": 0, "right": 0}
        self._delta = {"left": 0.0, "right": 0.0}
        self._aller(Etat.FERMETURE)
        return True

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
        q, fini = self._interp_q(self._t_phase or self.r.duree_placement_s)
        self._ecrire_q(q)
        if not fini:
            return
        plan, r = self._plan, self.r
        mg, md = self._mains_bassin()
        # Les mains vues par le robot et celles que donne la cinématique
        # sur les angles mesurés doivent concorder, et toutes être au milieu
        # des faces : sinon, le modèle ne décrit pas le bras, et serrer
        # serait serrer à côté.
        mod = self._mains_modele()
        if mod is None:
            self._echouer("serrage refusé — angles des bras illisibles : position des "
                          "mains inconnue")
            return
        desaccord = max(float(np.linalg.norm(a - b)) for a, b in zip((mg, md), mod))
        if desaccord > r.ecart_modele_max_m:
            self._echouer(f"prise refusée — mains mesurées à {100 * desaccord:.1f} cm du modèle "
                          f"des bras (limite {100 * r.ecart_modele_max_m:.1f}) : cinématique à "
                          f"recalibrer")
            return
        axe = np.array([math.cos(plan["lacet"]), math.sin(plan["lacet"]), 0.0])
        erreurs = [m - c for m, c in zip((mg, md), plan["cibles"])]
        prof = max(abs(float(e @ axe)) for e in erreurs + [m - c for m, c in zip(mod,
                                                                                   plan["cibles"])])
        haut = max(abs(float(e[2])) for e in erreurs)
        if prof <= r.tol_profondeur_m and haut <= r.tol_hauteur_m:
            self._debut_fermeture()
            return
        if self._corrections >= r.corrections_max:
            self._echouer(f"prise refusée — mains mal placées après {self._corrections} "
                          f"corrections : {100 * prof:.1f} cm en profondeur, {100 * haut:.1f} cm "
                          f"en hauteur")
            return
        # Correction : viser plus loin de l'erreur mesurée (le modèle et le
        # bras réel diffèrent un peu), puis revérifier.
        self._corrections += 1
        self.stats["corrections_placement"] = self.stats.get("corrections_placement", 0) + 1
        cg, cd = (c - e for c, e in zip(plan["cibles"], erreurs))
        q, _ = resoudre(self.mesures.modele, cg, cd, self._graine(self._q))
        if q is None:
            self._echouer("prise refusée — hors de portée des bras pour corriger les mains")
            return
        self._vers_placement(q, r.duree_placement_s / 2.0)

    def _pas_fermeture(self, dt: float) -> None:
        r = self.r
        tau, raison = self._efforts()
        if tau is None:
            self._echouer(f"serrage refusé — {raison}")
            return
        qm, raison = positions_valides(self.mesures, self._ROULIS.values())
        if qm is None:
            self._echouer(f"serrage refusé — {raison}")
            return
        q = dict(self._q)
        for cote, j in self._ROULIS.items():
            if self._contact[cote] is not None:
                continue
            s = self._FERME[cote]
            hausse = abs(tau[j] - self._tau0[j])
            retard = s * (self._q[j] - qm[j])
            if hausse >= r.hausse_contact_nm or retard >= r.retard_contact_rad:
                # Contact probable : la consigne n'avance plus, on le
                # confirme au pas suivant (jamais d'élan dans l'objet).
                self._compte_contact[cote] += 1
                if self._compte_contact[cote] >= 2:
                    self._contact[cote] = self._q[j]
                continue
            self._compte_contact[cote] = 0
            limite = self._plan["q_vide"][j]
            if s * (limite - qm[j]) <= 0.005:
                # La main elle-même (mesurée) a passé la largeur attendue.
                self._echouer("prise refusée — rien entre les mains : serrage dans le vide "
                              "évité (aucun contact à la largeur attendue)")
                return
            # Près du contact attendu, on ralentit : le dépassement d'un pas
            # (avant que le contact soit vu) reste petit, même objet raide.
            loin = s * (self._plan["q_contact"][j] - self._q[j]) > r.zone_contact_rad
            pas = s * (r.vitesse_fermeture * min(dt, 0.05) if loin
                       else min(r.vitesse_contact * min(dt, 0.05), r.pas_contact_max_rad))
            q[j] = self._q[j] + pas if s * (limite - (self._q[j] + pas)) > 0 else limite
        self._ecrire_q(q)
        if all(v is not None for v in self._contact.values()):
            mg, md = self._mains_bassin()
            ecart = float(np.linalg.norm((mg - md)[:2]))
            attendu = self._plan["largeur"] + 2 * r.demi_main_m
            if ecart > attendu + r.ecart_largeur_max_m:
                self._echouer(f"prise refusée — contact avec les mains à {ecart:.2f} m l'une "
                              f"de l'autre, l'objet attendu en ferait {attendu:.2f} : ce n'est "
                              f"pas lui")
                return
            self._ecart_mains = ecart
            self._t_phase = None
            self._aller(Etat.SERRAGE)
        elif self._horloge() - self._t0 > 4.0:
            self._echouer("prise refusée — rien entre les mains : serrage dans le vide évité "
                          "(fermeture trop longue)")

    def _pas_serrage_mesure(self, dt: float) -> None:
        r = self.r
        tau, raison = self._efforts()
        if tau is None:
            self._echouer(f"serrage refusé — {raison}")
            return
        q = dict(self._q)
        prets = True
        for cote, j in self._ROULIS.items():
            s, hausse = self._FERME[cote], abs(tau[j] - self._tau0[j])
            if abs(tau[j]) > self._limite_effort(j):
                # Au-dessus de la limite : on desserre, jamais plus fort, au
                # besoin en deçà du contact (objet très raide).
                self._delta[cote] = max(-r.serrage_max_rad,
                                        self._delta[cote] - 2 * r.vitesse_serrage * min(dt, 0.05))
                self.stats["serrages_limites"] = self.stats.get("serrages_limites", 0) + 1
                prets = False
            elif hausse < r.effort_serrage_nm and self._delta[cote] < r.serrage_max_rad:
                pas = r.vitesse_serrage * min(dt, 0.05)
                raideur = self._raideur(cote, hausse)
                # Pas réduit pour arriver à l'effort visé sans le dépasser ;
                # petit tant que la raideur n'est pas mesurée.
                pas = min(pas, max(0.002, (r.effort_serrage_nm - hausse) / raideur)
                          if raideur > 0.0 else 0.004)
                self._delta[cote] = min(r.serrage_max_rad, self._delta[cote] + pas)
                prets = False
            q[j] = self._contact[cote] + s * self._delta[cote]
        self._ecrire_q(q)
        if not prets:
            self._t_phase = None
            return
        faible = min(abs(tau[j] - self._tau0[j]) for j in self._ROULIS.values())
        if faible < r.effort_min_nm:
            self._echouer(f"prise refusée — serrage insuffisant ({faible:.1f} N·m au plus "
                          f"serré) : l'objet cède ou glisse, on n'insiste pas")
            return
        if self._t_phase is None:
            self._t_phase = self._horloge()
        elif self._horloge() - self._t_phase >= r.duree_maintien_s:
            self._q_serre = dict(self._q)
            self._lever_de_quelques_cm()

    def _ik_mains(self, centre_mains: np.ndarray, ecart: float, depart: Dict[str, float]
                  ) -> Optional[Dict[str, float]]:
        # Objet tenu : 0,5 mm près, car l'écart des mains EST le serrage (un
        # écart de 3 mm, sur un objet raide, ferait varier l'effort de plusieurs N·m).
        demi = np.array([0.0, ecart / 2.0, 0.0])
        q, _ = resoudre(self.mesures.modele, centre_mains + demi, centre_mains - demi,
                        self._graine(depart), tol=0.0005)
        return q

    def _raideur(self, cote: str, hausse: float) -> float:
        """Raideur mesurée (N·m/rad) entre deux pas du serrage : hausse du
        couple sur l'avance de la consigne ; 0 si elle n'est pas encore
        connue."""
        prec = self._prec_serrage.get(cote)
        self._prec_serrage[cote] = (self._delta[cote], hausse)
        if prec is None or self._delta[cote] - prec[0] < 1e-4:
            return 0.0
        return max(0.0, (hausse - prec[1]) / (self._delta[cote] - prec[0]))

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
        pas = 2 * self.r.vitesse_serrage * min(dt, 0.05)
        q = None
        for cote, j in self._ROULIS.items():
            if abs(tau[j]) > self._limite_effort(j) and self._delta[cote] > -self.r.serrage_max_rad:
                self._delta[cote] -= pas
                self.stats["serrages_limites"] = self.stats.get("serrages_limites", 0) + 1
                if self.etat is Etat.TENU:      # consigne fixe : on la corrige ici
                    q = dict(self._q) if q is None else q
                    q[j] -= self._FERME[cote] * pas
        if q is not None:
            self._ecrire_q(q)
            self._q_tenu = dict(q)

    def _q_mains(self, centre_mains: np.ndarray, ecart: float) -> Optional[Dict[str, float]]:
        """Bras qui mettent les mains de part et d'autre de `centre_mains`
        (repère du bassin), à `ecart` l'une de l'autre, le serrage gardé :
        aux consignes serrées s'ajoute le déplacement que la cinématique
        donne depuis la prise (le serrage mesuré reste le même, même si le
        modèle diffère un peu du bras réel), et la variation du serrage."""
        q = self._ik_mains(centre_mains, ecart, self._q_ref)
        if q is None:
            return None
        sortie = {j: self._q_serre[j] + q[j] - self._q_ref[j] for j in q}
        for cote, j in self._ROULIS.items():
            sortie[j] += self._FERME[cote] * (self._delta[cote] - self._delta_serre[cote])
        return sortie

    def _lever_de_quelques_cm(self) -> None:
        mg, md = self._mains_bassin()
        centre = (mg + md) / 2.0
        # La géométrie de la prise, mains mesurées, sert de référence.
        self._q_ref = self._ik_mains(centre, self._ecart_mains, self._q)
        if self._q_ref is None:
            self._echouer("prise refusée — impossible de lever l'objet (hors de portée)")
            return
        self._delta_serre = dict(self._delta)
        self._centre_porte = centre + np.array([0.0, 0.0, self.r.hauteur_test_m])
        q = self._q_mains(self._centre_porte, self._ecart_mains)
        if q is None:
            self._echouer("prise refusée — impossible de lever l'objet (hors de portée)")
            return
        self._q_de, self._q_vers = dict(self._q), q
        self._c_de, self._c_vers = centre, self._centre_porte
        self._aller(Etat.LEVAGE)

    def _pas_levage_mesure(self) -> None:
        r, p = self.r, self.p
        tau, raison = self._efforts()
        if tau is None:
            self._echouer(f"levage interrompu — {raison}")
            return
        q, fini = self._interp_c(r.duree_levage_test_s)
        self._ecrire_q(q)
        if not fini:
            return
        confirme = self._verifier() and min(abs(tau[j] - self._tau0[j])
                                            for j in self._ROULIS.values()) >= r.effort_min_nm
        if not confirme and self._horloge() - self._t0 - r.duree_levage_test_s \
                < p.attente_confirmation_s:
            return
        if confirme:
            self.stats["prises"] += 1
            self._lever_exemption()
            self._ramener()
            return
        self._essai += 1
        if self._essai >= r.essais_max:
            self._echouer(f"prise NON confirmée — l'objet n'a pas suivi ({self._essai} essais)")
            return
        # Reposer, desserrer, recommencer : deux phases, puis le placement.
        self.stats["reprises"] = self.stats.get("reprises", 0) + 1
        self._q_de, self._q_vers, self._sous = dict(self._q), dict(self._q_serre), "descendre"
        self._aller(Etat.REPOSE)

    def _pas_repose(self) -> None:
        r, p = self.r, self.p
        duree = r.duree_levage_test_s if self._sous == "descendre" else p.duree_ouverture_s
        q, fini = self._interp_q(duree)
        self._ecrire_q(q)
        if not fini:
            return
        if self._sous == "descendre":
            self._q_de, self._q_vers, self._sous = dict(self._q), dict(self._plan["q_ouvert"]), \
                "desserrer"
            self._aller(Etat.REPOSE)
            return
        self._corrections, self._delta = 0, {"left": 0.0, "right": 0.0}
        self._vers_placement(self._plan["q_ouvert"], 0.0)

    def _ramener(self, plus_pres: float = 0.0) -> None:
        """Vers le buste : la face arrière de l'objet à `distance_torse_m`
        du bassin, un peu plus haut ; au plus près que les bras permettent."""
        r = self.r
        prof = self.objet.profondeur if self.objet is not None else 0.2
        depart = self._centre_porte
        cible = None
        for k in range(12):
            x = max(r.distance_torse_m - plus_pres + prof / 2.0 + 0.02 * k, 0.05)
            if x >= float(depart[0]):
                break
            c = np.array([x, float(depart[1]), float(depart[2]) + r.hausse_transport_m])
            q = self._q_mains(c, self._ecart_mains)
            if q is not None:
                cible = (c, q)
                break
        if cible is None:
            c = depart + np.array([0.0, 0.0, r.hausse_transport_m])
            q = self._q_mains(c, self._ecart_mains)
            cible = (c, q if q is not None else dict(self._q))
        self._c_de, self._c_vers = np.array(depart, dtype=float), cible[0]
        self._centre_porte = cible[0]
        self._q_de, self._q_vers = dict(self._q), cible[1]
        self._aller(Etat.RAMENER)

    def _pas_ramener(self) -> None:
        q, fini = self._interp_c(self.r.duree_ramener_s)
        self._ecrire_q(q)
        if fini:
            self._q_tenu = dict(self._q)
            self._centre_transport = np.array(self._centre_porte, dtype=float)
            self.etat = Etat.TENU

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
        if tau is not None and all(abs(tau[j]) + 2.0 <= self._limite_effort(j)
                                   for j in self._ROULIS.values()):
            for cote in self._delta:
                self._delta[cote] = min(self.r.serrage_max_rad, self._delta[cote] + 0.05)
        self._ramener(plus_pres=0.02)

    def _pas_mesure(self, dt: float) -> bool:
        """Un pas de la prise mesurée ; faux si l'état relève du code commun."""
        e = self.etat
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
            self._garder_sous_la_limite(dt)
            self._surveiller_inclinaison()
        elif e in (Etat.MONTEE, Etat.DESCENTE, Etat.POSE, Etat.OUVERTURE):
            if e is not Etat.OUVERTURE:
                self._garder_sous_la_limite(dt)
            self._pas_apres_prise(dt)
        else:
            return False
        return True

    def _centre_tenu(self) -> np.ndarray:
        mg, md = self._mains_bassin()
        return (mg + md) / 2.0

    def _pas_apres_prise(self, dt: float) -> None:
        """Montée, descente, pose et ouverture, objet tenu en prise mesurée :
        les mains suivent un point (repère du bassin), le serrage gardé.
        Avant un plateau, l'objet repart devant, là où il a été pris."""
        p, r = self.p, self.r
        x_prise = float(self._plan["centre"][0]) if self._plan else 0.29
        if self.etat is Etat.MONTEE:
            haut = (self._surface - float(self.mesures.bassin_z())
                    + (self.objet.hauteur / 2.0 if self.objet else 0.1))
            visee = np.array([x_prise, float(self._centre_porte[1]),
                              haut + p.marge_franchissement])
            q = self._q_mains(visee, self._ecart_mains)
            if q is None:
                visee[2] = haut + p.marge_franchissement_min
                q = self._q_mains(visee, self._ecart_mains)
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
                self._c_vers = visee
                self._t0 = self._horloge()
            qi, fini = self._interp_c(self.r.duree_ramener_s)
            self._ecrire_q(qi)
            if fini and self._horloge() - self._t0 >= self.r.duree_ramener_s \
                    + p.stabilisation_montee_s or (fini and self.degage_au_dessus(self._surface)):
                self._centre_porte = visee
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
                # On ne lâche pas l'objet de haut : on le remonte contre le
                # buste, en le tenant (RAMENER, puis TENU).
                self._c_de = np.array(self._centre_porte, dtype=float)
                self._c_vers = self._centre_porte = self._centre_transport
                self._q_de, self._q_vers = dict(self._q), dict(self._q_tenu)
                self._aller(Etat.RAMENER)
                return
            self._centre_porte = c
            self._ecrire_q(q)
            if self._fond_objet() <= self._surface + p.jeu_depose:
                self.stats["deposes_plateau"] += 1
                self._ouvrir()
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
                # Retour progressif à la posture de port (jamais un saut).
                self._q_de = dict(self._q)
                self._q_vers = posture_bras(p.roulis_ouvert, p.tangage_port, p.coude)
                self._aller(Etat.LACHER)

    def _ouvrir(self) -> None:
        """Les mains s'écartent de `jeu_ouverture_m` de plus que le contact."""
        c = self._centre_tenu()
        self._delta = {"left": 0.0, "right": 0.0}
        q = self._q_mains(c, (self._ecart_mains or 0.33) + 2 * self.r.jeu_ouverture_m)
        q_vers = dict(self._q)
        if q is not None:
            q_vers = q
        else:
            for cote, j in self._ROULIS.items():
                q_vers[j] = self._q[j] - self._FERME[cote] * 0.3
        self._q_de, self._q_vers = dict(self._q), q_vers
        self._aller(Etat.OUVERTURE)

    def distance_de_prise(self, objet: Objet, d_min: float = 0.25
                          ) -> Tuple[Optional[float], str]:
        """Prise mesurée : à quelle distance (m) du centre de l'objet, le long
        du cap de prise, arrêter le bassin pour que les mains atteignent le
        milieu de ses faces et que les avant-bras passent au-dessus du
        plateau ; la plus courte à partir de `d_min`. (None, raison) s'il
        n'y en a aucune."""
        z = float(objet.centre[2]) - float(self.mesures.bassin_z())
        port = {**self._articulations_fixes(),
                **posture_bras(self.p.roulis_ouvert, self.p.tangage_port, self.p.coude)}
        return distance_ideale(self.mesures.modele, objet.largeur, objet.profondeur,
                               objet.hauteur, z, port, bord_m=objet.bord_m, reglages=self.r,
                               distances=np.arange(d_min, 0.45, 0.005))

    def mains_a_hauteur(self, objet: Objet) -> Tuple[bool, str]:
        """Les mains sont-elles à la hauteur de l'objet (entre son dessous et
        son dessus, à 2 cm près) ? C'est la condition de hauteur de la règle
        de Kenzo (voir entre_les_mains), qui borne l'exemption du gardien."""
        g, d = self._mains()
        mains_z = (float(g[2]) + float(d[2])) / 2.0
        dessous = float(objet.centre[2]) - objet.hauteur / 2.0 - 0.02
        dessus = float(objet.centre[2]) + objet.hauteur / 2.0 + 0.02
        if not (dessous < mains_z < dessus):
            return False, (f"mains à {mains_z:.2f} m de haut, objet entre "
                           f"{dessous + 0.02:.2f} et {dessus - 0.02:.2f} m")
        return True, ""

    # ---- API offerte aux apps --------------------------------------------

    def presaisir(self) -> ServiceResult:
        """Ouvre les bras en grand avant l'approche finale, robot à
        l'arrêt. Non bloquant : suivre l'état jusqu'à PRET."""
        if not self._demarre:
            return ServiceResult(ok=False, reason="service non démarré")
        if self.etat is Etat.PRET:
            return ServiceResult(ok=True)
        if self.etat is not Etat.LIBRE:
            return ServiceResult(ok=False, reason=f"occupé : {self.etat.name}")
        self._aller(Etat.PRESAISIE)
        return ServiceResult(ok=True)

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
            self._vers_placement(self._plan["q_ouvert"], self.r.duree_placement_s)
            return ServiceResult(ok=True)
        # Le serrage part de l'ouverture où sont les bras : large après une
        # pré-saisie, normale sinon.
        self._roulis_depart_serrage = (self.p.roulis_presaisie if self.etat is Etat.PRET
                                       else self.p.roulis_ouvert)
        self._aller(Etat.SERRAGE)
        return ServiceResult(ok=True)

    def _fond_objet(self) -> float:
        """Hauteur estimée du fond de l'objet : hauteur moyenne des deux
        mains, moins la demi-hauteur de l'objet (rien en main : la hauteur
        des mains)."""
        g, d = self._mains()
        h = self.objet.hauteur if self.objet is not None else 0.0
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
        self._lever_exemption()
        self.objet, self._franchi = None, None
        self._lacher_de = (self._roulis_courant, self._tangage_courant)
        self._lacher_vers = (self.p.roulis_presaisie if large else self.p.roulis_ouvert,
                             self.p.tangage_port)
        # Depuis la consigne réelle (le coude et chaque roulis peuvent avoir
        # bougé seuls en prise mesurée) ; posture_bras est linéaire, donc en
        # boucle ouverte c'est exactement le geste d'avant.
        self._q_de = dict(self._q)
        self._q_vers = posture_bras(*self._lacher_vers, self.p.coude)
        self._init_mesure()
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
        self._tangage_ouverture = self.p.tangage_port
        self._init_mesure()
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
                self.etat = Etat.LIBRE
        elif self.etat is Etat.PRESAISIE:
            r, fini = self._interp(p.roulis_ouvert, p.roulis_presaisie, p.duree_presaisie_s)
            self._ecrire(r, p.tangage_port)
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
