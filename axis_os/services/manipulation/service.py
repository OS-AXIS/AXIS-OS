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
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Callable, Dict, Optional, Tuple

import numpy as np

from axis_os.hal.core.types import HALStatus
from axis_os.services.base import IService, ServiceHealth, ServiceResult
from axis_os.store.sdk.manifest import Capability


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
    vers l'avant du robot."""
    centre: Tuple[float, float, float]
    largeur: float
    profondeur: float
    hauteur: float


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
                 arret_securite: Optional[Callable[[], bool]] = None):
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
        """
        self._bras = bras
        self._mains = mains
        self._vers_robot = vers_robot
        self._verifier = verifier_prise
        self._gardien = gardien
        self._horloge = horloge
        self.p = postures
        self.etat = Etat.LIBRE
        self.objet: Optional[Objet] = None
        self.raison = ""
        self._t0 = 0.0
        self._demarre = False
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
        self._bras.update(posture_bras(roulis, tangage, self.p.coude))

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
        doivent être entre sa face avant et sa face arrière, à 2 cm près.
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

    def prendre(self, objet: Objet) -> ServiceResult:
        """Lance la prise de l'objet. Non bloquant : suivre la progression
        par `etat`, ou par le résultat de tick(). Refusée si l'objet n'est
        pas entre les mains, ou si le gardien refuse l'exemption."""
        if not self._demarre:
            return ServiceResult(ok=False, reason="service non démarré")
        if self.etat not in (Etat.LIBRE, Etat.PRET):
            return ServiceResult(ok=False, reason=f"occupé : {self.etat.name}")
        ok, raison = self.entre_les_mains(objet)
        if not ok:
            self.stats["refus"] += 1
            return ServiceResult(ok=False, reason=f"prise refusée — {raison}")
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
        self._aller(Etat.MONTEE)
        return ServiceResult(ok=True)

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
        self._tangage_ouverture = self.p.tangage_port
        self.etat = Etat.LIBRE

    # ---- boucle : un pas à chaque appel de tick() ------------------------

    # États où les bras bougent : ceux qu'un arrêt de sécurité doit figer.
    _EN_MOUVEMENT = (Etat.PRESAISIE, Etat.MONTEE, Etat.SERRAGE, Etat.LEVAGE, Etat.POSE, Etat.OUVERTURE,
                     Etat.DESCENTE, Etat.LACHER)

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
        if self.etat is Etat.LACHER:
            r, _ = self._interp(self._lacher_de[0], self._lacher_vers[0], p.duree_ouverture_s)
            t, fini = self._interp(self._lacher_de[1], self._lacher_vers[1],
                                   p.duree_ouverture_s)
            self._ecrire(r, t)
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
