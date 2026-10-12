"""
AXIS-OS — Robot Capabilities
================================
Système déclaratif par lequel un IRobotHAL (le point d'entrée d'AXIS-OS
vers le matériel d'un robot, défini dans interfaces.py ; HAL signifie
Hardware Abstraction Layer, la couche d'abstraction matérielle) annonce
ce qu'il possède réellement : membres articulés (jambes, bras), roues,
caméra, etc.

Pourquoi : le reste d'AXIS-OS (les apps, JointSafetyGuard qui est le
garde-fou de chaque articulation, PlayStoreRuntime qui installe et lance
les apps du Play Store, la boutique d'apps d'AXIS-OS) doit pouvoir
vérifier qu'un robot possède une capacité avant de tenter de l'utiliser,
plutôt que de supposer implicitement « c'est un G1 avec des bras et des
jambes » partout dans le code.

Origine : une revue extérieure du code, le 2026-09-06, a classé ce point
en troisième priorité. Elle relevait que normalized_robot.py (et plus
largement toute la couche HAL) est une bonne abstraction, mais reste
implicitement centrée sur la morphologie du G1, l'humanoïde de référence
d'AXIS-OS. Rien ne permettait alors de représenter proprement un robot à
roues (pas de jambes), un drone (ni jambes ni bras, mais un contrôle de
vol) ou un bras robotique fixe (aucune jambe, jamais de locomotion).

Principes :
  - Une capacité est un fait déclaré par le driver du robot (le code qui
    pilote ce modèle précis), pas une déduction. C'est cohérent avec le
    reste d'AXIS-OS, où chaque fabricant fournit son propre driver (voir
    conventions.py et robot.py pour le G1) : c'est ce driver qui sait ce
    que son robot possède réellement, AXIS-OS ne devine jamais.
  - L'absence d'une capacité n'est jamais une erreur en soi. Un robot à
    roues qui n'a pas ARMS n'est pas « cassé », il a simplement une autre
    morphologie. C'est à l'app, ou à son manifeste (le fichier qui décrit
    l'app : identité, permissions, matériel requis ; voir
    axis_os/store/sdk/manifest.py), de vérifier ce dont elle a besoin,
    pas à AXIS-OS d'imposer une capacité universelle.
  - Ce module est purement déclaratif et informatif : il ne bloque rien
    lui-même. Il donne aux couches supérieures (PlayStoreRuntime, la
    section HardwareRequirements du manifeste d'une app) un moyen fiable
    de vérifier une compatibilité avant de démarrer une mission, plutôt
    que de découvrir en cours de route qu'un robot n'a pas ce qu'une app
    suppose. Ce qui est branché aujourd'hui est décrit dans la note
    d'intégration en fin de fichier.

Licence : Apache 2.0 (open-source)
"""

from __future__ import annotations
from enum import Enum, auto
from typing import Dict, FrozenSet, Iterable, Mapping


class CapabilityStatus(Enum):
    """
    État d'une capacité déclarée pendant le fonctionnement du robot,
    distinct de sa simple présence.

    Pourquoi deux informations : un robot peut posséder physiquement une
    caméra (déclarée dans son CapabilitySet) et avoir, à un instant donné,
    cette caméra débranchée. Les confondre empêcherait de distinguer « ce
    robot n'a pas de caméra » de « ce robot a une caméra, mais elle est
    hors service en ce moment ».

    Origine : proposition de la même revue extérieure du code
    (2026-09-06), dans sa discussion sur le lien entre capacités et Play
    Store : capacités obligatoires (required) ou facultatives (optional)
    d'une app, et manière de les confronter à celles du robot.
    """
    # Présente et fonctionnelle.
    AVAILABLE    = auto()
    # Non fonctionnelle actuellement, sans raison précisée.
    UNAVAILABLE  = auto()
    # Fonctionne, mais pas pleinement (par exemple une caméra de secours
    # en basse résolution).
    DEGRADED     = auto()
    # Matériel absent ou débranché alors qu'il est normalement présent.
    DISCONNECTED = auto()
    # Désactivée volontairement (configuration, sécurité, permission
    # refusée).
    DISABLED     = auto()
    # État non déterminé : traité comme indisponible, par prudence.
    UNKNOWN      = auto()


class RobotCapability(Enum):
    """
    Une catégorie de fonctionnalité physique : ce robot peut-il, en
    principe, faire ce type de chose ?

    La granularité est volontairement large. Une capacité ne décrit pas
    une articulation ou un capteur individuel : c'est le rôle de
    get_joint_definitions() et des méthodes get_imu() et
    get_force_torque() d'IRobotHAL. On ne répond pas ici à « quel est
    l'état exact de telle articulation ».
    """
    # Locomotion
    LEGS           = auto()  # membres articulés pour marcher (humanoïde, bipède)
    WHEELS         = auto()  # locomotion à roues
    TRACKS         = auto()  # locomotion à chenilles
    FLIGHT_CONTROL = auto()  # contrôle de vol (drone, quadricoptère)

    # Manipulation
    ARMS           = auto()  # membres articulés pour manipuler
    GRIPPER        = auto()  # organe de préhension (pince, main)

    # Perception
    CAMERA         = auto()
    LIDAR          = auto()  # laser qui mesure les distances autour du robot
    IMU            = auto()  # centrale inertielle : accélérations, rotation
    FORCE_TORQUE   = auto()  # capteur de force et de couple
    MICROPHONE     = auto()

    # Autre
    SPEAKER        = auto()  # sortie audio (annonces vocales, alertes)


class CapabilitySet:
    """
    Ensemble immuable des capacités d'un robot donné.

    Pourquoi immuable : les capacités physiques d'un robot ne changent pas
    en cours d'exécution, contrairement à son état, sa position ou sa
    santé. Rien dans AXIS-OS ne devrait donc avoir besoin de modifier un
    CapabilitySet après sa création par le driver. Même un changement
    d'état (set_status()) produit un nouvel ensemble.

    Usage typique dans un driver (par exemple robot.py pour le G1) :

        class UnitreeG1HAL(IRobotHAL):
            def get_capabilities(self) -> CapabilitySet:
                return CapabilitySet.from_iterable([
                    RobotCapability.LEGS, RobotCapability.ARMS,
                    RobotCapability.GRIPPER, RobotCapability.IMU,
                ])

    Usage typique côté vérification, dans une app ou dans
    PlayStoreRuntime (voir la note d'intégration en fin de fichier pour
    ce qui est branché automatiquement) :

        if not robot.get_capabilities().has(RobotCapability.ARMS):
            raise ValueError("Cette app nécessite un robot avec des bras.")
    """

    def __init__(
        self,
        capabilities: Iterable[RobotCapability] = (),
        statuses: Mapping[RobotCapability, CapabilityStatus] | None = None,
    ):
        """
        `capabilities` : ce que le robot possède physiquement. Cette liste
        est fixe, déclarée une fois par le driver (voir la docstring de la
        classe).

        `statuses` : état actuel de chaque capacité déclarée. Toute
        capacité de `capabilities` absente de `statuses` est considérée
        AVAILABLE par défaut. Ainsi, un driver qui ne gère pas encore les
        pannes en cours de fonctionnement n'a rien à changer : has() se
        comporte exactement comme avant (présente = utilisable).
        """
        self._capabilities: FrozenSet[RobotCapability] = frozenset(capabilities)
        given = dict(statuses) if statuses else {}
        self._statuses: Dict[RobotCapability, CapabilityStatus] = {
            cap: given.get(cap, CapabilityStatus.AVAILABLE)
            for cap in self._capabilities
        }

    @classmethod
    def from_iterable(cls, capabilities: Iterable[RobotCapability]) -> "CapabilitySet":
        return cls(capabilities)

    @classmethod
    def empty(cls) -> "CapabilitySet":
        """
        Ensemble vide : valeur renvoyée par défaut pour un driver qui
        n'implémente pas encore get_capabilities() (voir l'implémentation
        par défaut d'IRobotHAL, dans interfaces.py).

        Un ensemble vide signifie « capacités inconnues ou non
        déclarées », pas « ce robot n'a physiquement aucune capacité ».
        Une app qui interroge has() sur un ensemble vide obtient toujours
        False : c'est le comportement prudent par défaut (refuser plutôt
        que supposer).
        """
        return cls()

    def status_of(self, capability: RobotCapability) -> CapabilityStatus:
        """État actuel d'une capacité. Une capacité jamais déclarée
        renvoie UNKNOWN, jamais AVAILABLE : on ne suppose jamais qu'un
        composant non déclaré fonctionne."""
        if capability not in self._capabilities:
            return CapabilityStatus.UNKNOWN
        return self._statuses[capability]

    def set_status(self, capability: RobotCapability, status: CapabilityStatus) -> "CapabilitySet":
        """Renvoie un nouveau CapabilitySet où l'état d'une capacité déjà
        déclarée est mis à jour. L'instance existante n'est jamais
        modifiée (CapabilitySet reste immuable, voir la docstring de la
        classe).

        Une capacité que le driver n'a jamais déclarée lève une erreur,
        plutôt que d'être ajoutée en silence : l'état ne peut changer que
        pour du matériel qui existe."""
        if capability not in self._capabilities:
            raise ValueError(
                f"{capability.name} n'a jamais été déclarée pour ce robot — "
                "impossible de changer son état runtime."
            )
        new_statuses = dict(self._statuses)
        new_statuses[capability] = status
        return CapabilitySet(self._capabilities, new_statuses)

    def has(self, capability: RobotCapability) -> bool:
        """Vrai si la capacité est déclarée ET actuellement AVAILABLE.

        C'est la vérification stricte à faire avant d'agir maintenant
        (par exemple juste avant d'utiliser la caméra). Pour savoir
        seulement si le robot possède physiquement la capacité, quel que
        soit son état actuel, voir is_declared()."""
        return self.status_of(capability) == CapabilityStatus.AVAILABLE

    def is_declared(self, capability: RobotCapability) -> bool:
        """Vrai si le robot possède physiquement cette capacité, quel que
        soit son état actuel."""
        return capability in self._capabilities

    def has_all(self, capabilities: Iterable[RobotCapability]) -> bool:
        return all(self.has(c) for c in capabilities)

    def has_any(self, capabilities: Iterable[RobotCapability]) -> bool:
        return any(self.has(c) for c in capabilities)

    def missing_from(self, capabilities: Iterable[RobotCapability]) -> FrozenSet[RobotCapability]:
        """Renvoie les capacités demandées qui ne sont pas actuellement
        AVAILABLE, qu'elles soient non déclarées ou déclarées mais en
        panne. Sert à produire un message d'erreur précis plutôt qu'un
        simple booléen : « il manque CAMERA et LIDAR », pas seulement
        « incompatible »."""
        return frozenset(c for c in capabilities if not self.has(c))

    def __iter__(self):
        return iter(self._capabilities)

    def __len__(self) -> int:
        return len(self._capabilities)

    def __contains__(self, capability: RobotCapability) -> bool:
        return self.has(capability)

    def __repr__(self) -> str:
        parts = sorted(f"{c.name}={self._statuses[c].name}" for c in self._capabilities)
        return f"CapabilitySet({{{', '.join(parts)}}})"


# Note d'intégration : où ce module est branché.
#
# À sa création, ce fichier définissait RobotCapability et CapabilitySet
# de façon autonome et testée, ainsi que l'implémentation par défaut de
# get_capabilities() dans IRobotHAL (qui renvoie CapabilitySet.empty()).
# Deux choses avaient alors été volontairement laissées de côté, pour
# limiter le travail à la construction du composant lui-même :
#
#   1. UnitreeG1HAL (robot.py) n'implémentait pas encore
#      get_capabilities() avec ses vraies capacités et héritait donc de
#      l'ensemble vide. Il fallait d'abord lister précisément ce que le
#      G1 possède, en cohérence avec conventions.py et robot.py.
#      C'est fait depuis : le G1 déclare LEGS, ARMS, GRIPPER, IMU,
#      FORCE_TORQUE, CAMERA, MICROPHONE et SPEAKER (voir
#      UnitreeG1HAL.get_capabilities() pour les justifications).
#
#   2. Aucune vérification automatique n'était branchée dans
#      PlayStoreRuntime ni dans la section HardwareRequirements du
#      manifeste, alors que le système était prêt pour ce genre de
#      vérification. Le comportement à adopter quand une app démarre sur
#      un robot qui n'a pas les capacités qu'elle exige (refus au
#      démarrage ? simple avertissement ?) relevait d'une décision de
#      produit à prendre consciemment ; l'intégration ne devait pas être
#      ajoutée à la hâte.
#      Cette décision a été prise depuis, et la vérification se fait à
#      l'installation : PlayStoreRuntime.install() refuse d'installer une
#      app dont une capacité de HardwareRequirements.required manque (au
#      sens de has()), et affiche seulement un avertissement pour une
#      capacité de HardwareRequirements.optional absente (voir
#      axis_os/store/runtime/play_store.py et manifest.py).
#
# Limite connue : aucun driver ne met encore à jour l'état des capacités
# en cours de fonctionnement (set_status() n'est appelé nulle part dans
# axis_os/). Toute capacité déclarée reste donc AVAILABLE.
