"""
AXIS-OS — Robot Capabilities
================================
Système déclaratif permettant à un IRobotHAL d'annoncer CE QU'IL A
RÉELLEMENT (joints articulés, roues, bras, caméra, etc.), pour que le
reste d'AXIS-OS (apps, JointSafetyGuard, PlayStoreRuntime) puisse
vérifier qu'un robot possède une capacité AVANT de tenter de
l'utiliser, plutôt que de supposer implicitement "c'est un G1 avec
des bras et des jambes" partout dans le code.

Origine : revue externe du 2026-09-06, priorité 3. Point soulevé :
normalized_robot.py (et plus largement toute la stack HAL) est une
bonne abstraction, mais reste implicitement centrée sur la morphologie
du G1 — rien ne permettrait aujourd'hui de représenter proprement un
robot à roues (pas de jambes), un drone (pas de jambes ni de bras,
mais un contrôle de vol), ou un bras robotique fixe (pas de jambes du
tout, jamais de locomotion).

Principe :
  - Une capacité est un FAIT déclaré par le driver HAL du robot, pas
    une déduction — cohérent avec le reste d'AXIS-OS où chaque
    fabricant fournit son propre driver (voir conventions.py,
    robot.py pour le G1) : c'est ce driver qui sait ce que SON robot
    possède réellement, AXIS-OS ne devine jamais.
  - L'absence d'une capacité n'est jamais une erreur en soi — un
    robot à roues qui n'a pas ARMS n'est pas "cassé", il est
    simplement d'une autre morphologie. C'est à l'app ou au manifest
    de vérifier ce dont elle a besoin, pas à AXIS-OS de forcer une
    capacité universelle.
  - Ce système est purement DÉCLARATIF et INFORMATIF dans cette
    version — il ne bloque rien automatiquement (voir note
    d'intégration en fin de fichier). Il donne aux couches
    supérieures (PlayStoreRuntime, HardwareRequirements du manifest
    d'une app) un moyen fiable de VÉRIFIER une compatibilité avant de
    démarrer une mission, plutôt que de découvrir en cours de route
    qu'un robot n'a pas ce qu'une app suppose.

Licence : Apache 2.0 (open-source)
"""

from __future__ import annotations
from enum import Enum, auto
from typing import Dict, FrozenSet, Iterable, Mapping


class CapabilityStatus(Enum):
    """
    État runtime d'une capacité déclarée — distinct de sa simple
    PRÉSENCE. Un robot peut posséder physiquement une caméra
    (déclarée dans son CapabilitySet) tout en ayant, à un instant
    donné, cette caméra débranchée : ce ne sont pas la même
    information, et les confondre empêcherait de distinguer "ce
    robot n'a pas de caméra" de "ce robot a une caméra mais elle est
    hors service maintenant".

    Origine : proposition de revue externe du 2026-09-06 sur la
    connexion Capabilities ↔ Play Store — voir discussion Required/
    Optional/Capability Resolution.
    """
    AVAILABLE    = auto()  # présente et fonctionnelle
    UNAVAILABLE  = auto()  # non fonctionnelle actuellement (raison non précisée)
    DEGRADED     = auto()  # fonctionne, mais pas pleinement (ex: caméra basse résolution en secours)
    DISCONNECTED = auto()  # matériel absent/débranché alors qu'il est normalement présent
    DISABLED     = auto()  # désactivée volontairement (config, sécurité, permission refusée)
    UNKNOWN      = auto()  # état non déterminé — traité comme indisponible par prudence


class RobotCapability(Enum):
    """
    Granularité volontairement large — une capacité représente une
    CATÉGORIE de fonctionnalité physique, pas un joint ou capteur
    individuel (ça, c'est le rôle de get_joint_definitions() et des
    méthodes get_imu()/get_force_torque() déjà existantes sur
    IRobotHAL). Le but ici est de répondre à la question "ce robot
    peut-il, en principe, faire ce type de chose", pas "quel est
    l'état exact de tel joint précis".
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
    LIDAR          = auto()
    IMU            = auto()
    FORCE_TORQUE   = auto()
    MICROPHONE     = auto()

    # Autre
    SPEAKER        = auto()  # sortie audio (annonces vocales, alertes)


class CapabilitySet:
    """
    Ensemble immuable des capacités d'un robot donné. Immuable
    volontairement — les capacités PHYSIQUES d'un robot ne changent
    pas en cours d'exécution (contrairement à son état, sa position,
    sa santé), donc rien dans AXIS-OS ne devrait jamais avoir besoin
    de modifier un CapabilitySet après sa création par le driver HAL.

    Usage typique dans un driver HAL (ex: robot.py pour le G1) :

        class UnitreeG1HAL(IRobotHAL):
            def get_capabilities(self) -> CapabilitySet:
                return CapabilitySet.from_iterable([
                    RobotCapability.LEGS, RobotCapability.ARMS,
                    RobotCapability.GRIPPER, RobotCapability.IMU,
                ])

    Usage typique côté vérification (app ou PlayStoreRuntime, non
    branché automatiquement dans cette version — voir note en fin de
    fichier) :

        if not robot.get_capabilities().has(RobotCapability.ARMS):
            raise ValueError("Cette app nécessite un robot avec des bras.")
    """

    def __init__(
        self,
        capabilities: Iterable[RobotCapability] = (),
        statuses: Mapping[RobotCapability, CapabilityStatus] | None = None,
    ):
        """
        `capabilities` : ce que le robot POSSÈDE physiquement (fixe,
        déclaré une fois par le driver HAL — voir docstring de classe).
        `statuses` : état runtime actuel de chaque capacité déclarée.
        Toute capacité de `capabilities` absente de `statuses` est
        considérée AVAILABLE par défaut — un driver qui ne gère pas
        encore les pannes runtime n'a rien à changer, has() continue
        de se comporter exactement comme avant (présent = utilisable).
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
        Ensemble vide — valeur de retour par défaut pour un driver HAL
        qui n'implémente pas encore get_capabilities() (voir
        l'implémentation par défaut ajoutée à IRobotHAL). Un
        CapabilitySet vide signifie "capacités inconnues/non
        déclarées", PAS "ce robot n'a physiquement aucune capacité" —
        une app qui vérifie has() sur un ensemble vide obtient
        toujours False, ce qui est le comportement prudent par défaut
        (refuser plutôt que supposer).
        """
        return cls()

    def status_of(self, capability: RobotCapability) -> CapabilityStatus:
        """État runtime actuel. Une capacité jamais déclarée renvoie
        UNKNOWN (jamais AVAILABLE) — on ne suppose jamais qu'un
        composant non déclaré fonctionne."""
        if capability not in self._capabilities:
            return CapabilityStatus.UNKNOWN
        return self._statuses[capability]

    def set_status(self, capability: RobotCapability, status: CapabilityStatus) -> "CapabilitySet":
        """Retourne un NOUVEAU CapabilitySet avec le statut runtime
        mis à jour pour une capacité déjà déclarée — ne modifie
        jamais l'instance existante (CapabilitySet reste immuable,
        voir docstring de classe). Lever une erreur plutôt que
        d'ajouter silencieusement une capacité que le driver HAL n'a
        jamais déclarée physiquement : le statut runtime ne peut
        changer que pour du matériel qui existe."""
        if capability not in self._capabilities:
            raise ValueError(
                f"{capability.name} n'a jamais été déclarée pour ce robot — "
                "impossible de changer son état runtime."
            )
        new_statuses = dict(self._statuses)
        new_statuses[capability] = status
        return CapabilitySet(self._capabilities, new_statuses)

    def has(self, capability: RobotCapability) -> bool:
        """Présent ET actuellement AVAILABLE. C'est la vérification
        stricte à utiliser avant d'agir maintenant (ex: juste avant
        d'utiliser la caméra) — pour savoir seulement si le robot
        possède physiquement la capacité (indépendamment de son état
        runtime), voir is_declared()."""
        return self.status_of(capability) == CapabilityStatus.AVAILABLE

    def is_declared(self, capability: RobotCapability) -> bool:
        """Le robot possède physiquement cette capacité, quel que
        soit son état runtime actuel."""
        return capability in self._capabilities

    def has_all(self, capabilities: Iterable[RobotCapability]) -> bool:
        return all(self.has(c) for c in capabilities)

    def has_any(self, capabilities: Iterable[RobotCapability]) -> bool:
        return any(self.has(c) for c in capabilities)

    def missing_from(self, capabilities: Iterable[RobotCapability]) -> FrozenSet[RobotCapability]:
        """Retourne les capacités demandées qui ne sont pas
        actuellement AVAILABLE (non déclarées OU déclarées mais en
        panne) — utile pour un message d'erreur précis plutôt qu'un
        simple booléen ("il manque CAMERA et LIDAR", pas juste
        "incompatible")."""
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


# NOTE D'INTÉGRATION — ce fichier définit RobotCapability et
# CapabilitySet de façon autonome et testée, ainsi qu'une
# implémentation par défaut de get_capabilities() ajoutée à IRobotHAL
# (retourne CapabilitySet.empty()). Deux choses restent volontairement
# NON faites dans cette session :
#
#   1. UnitreeG1HAL (robot.py) n'implémente PAS encore
#      get_capabilities() avec ses vraies capacités (LEGS, ARMS,
#      GRIPPER, IMU, etc.) — il hérite donc de l'implémentation par
#      défaut vide. Ajouter ça correctement demande de lister
#      précisément ce que le G1 possède, cohérent avec le travail déjà
#      fait sur conventions.py/robot.py, mais non fait ici pour
#      limiter la portée de cette session à la construction du
#      composant lui-même.
#
#   2. Aucune vérification automatique n'est branchée dans
#      PlayStoreRuntime ou HardwareRequirements du manifest d'une app
#      — le système est prêt à être utilisé pour ce genre de
#      vérification, mais l'intégration réelle (qu'est-ce qui se passe
#      exactement si une app démarre sur un robot qui n'a pas les
#      capacités qu'elle requiert : refus au démarrage ? avertissement
#      seulement ?) est une décision de produit qui mérite d'être
#      prise consciemment, pas ajoutée à la volée en fin de session.
