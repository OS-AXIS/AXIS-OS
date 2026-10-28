"""
AXIS-OS — Service de locomotion : faire marcher le robot sur consigne de
vitesse.

Troisième service système (un service démarré par le système et partagé
par toutes les apps, voir axis_os/services/base.py). Il rend la marche
accessible à une app (un programme installé sur le robot) par une simple
consigne de vitesse, « avance à 0.5 m/s », « tourne à 0.6 rad/s », sans
que l'app ait à connaître le robot.

Universel par construction. Pourquoi : un même système doit faire avancer
des robots qui se déplacent de façons très différentes. Le service ne sait
donc rien de la manière dont un robot se déplace : il délègue à un
ILocomotionAdapter, fourni par le driver (le pilote propre à un modèle de
robot). Pour un bipède, l'adaptateur est une politique de marche (un
réseau de neurones entraîné qui calcule les consignes des articulations
des jambes) ; pour une base à roues, une cinématique différentielle (le
calcul de la vitesse de chaque roue à partir de la vitesse voulue). Même
principe que les groupes d'articulations : le cadre est commun, la
connaissance du robot est dans son driver.

Trois exigences de sécurité, chacune testée dans
test/test_locomotion_service.py :

1. Échéance de fraîcheur. Pourquoi : une app qui cesse de rafraîchir sa
   consigne (parce qu'elle a planté, par exemple) ne doit pas laisser le
   robot marcher indéfiniment. Passé command_timeout_s sans nouvelle
   consigne, la consigne retombe à zéro. Même principe que
   SetpointFollower, le suiveur de consignes articulaires à échéance de
   fraîcheur (axis_os/services/policy/setpoint_follower.py).

2. S'arrêter n'est pas couper. Pourquoi : sur un bipède, cesser d'envoyer
   des consignes ou couper le couple des moteurs, c'est tomber. Arrêter
   veut donc dire continuer à faire tourner l'adaptateur avec une vitesse
   nulle : le robot s'immobilise en gardant son équilibre. C'est
   l'adaptateur qui sait ce qu'est un arrêt sûr pour son robot ; le
   service lui demande seulement « vitesse zéro ».

3. Après une chute, plus rien. Pourquoi : une politique de marche qui
   continue à commander un robot à terre le fait se débattre. Dès qu'une
   chute est détectée, le service cesse d'envoyer des consignes et le
   signale. La suite (amortissement, arrêt d'urgence) relève du système de
   sécurité, pas de la marche.

Toutes les consignes passent par `send_targets`, qui doit être la méthode
gardée du HAL (la couche d'abstraction du matériel, entre le logiciel et
les moteurs) : elle soumet chaque consigne à JointSafetyGuard, le
garde-fou de chaque articulation (axis_os/hal/safety/limits.py). Un
service ne commande jamais un actionneur directement.

Si un gardien de proximité est fourni (proximite.py), il plafonne en plus
chaque consigne selon la distance aux obstacles : voir LocomotionService.
Puis le gardien de zones (axis_os/services/navigation/zones.py) refuse
d'entrer dans une zone interdite et plafonne la vitesse en zone lente, et la
protection thermique (axis_os/services/health/thermal.py) réduit la vitesse
d'un robot qui chauffe. Chacun ne peut que réduire la consigne.
"""
from __future__ import annotations

import math
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Callable, Dict, Optional, Tuple

from axis_os.hal.core.types import HALStatus
from axis_os.services.base import IService, ServiceHealth, ServiceResult
from axis_os.store.sdk.manifest import Capability


@dataclass(frozen=True)
class Twist:
    """Consigne de vitesse, dans le repère du robot (« twist » est le nom
    usuel, en robotique, d'un couple vitesse linéaire et vitesse angulaire).

    vx : vitesse vers l'avant, en m/s.
    vy : vitesse vers la gauche, en m/s.
    wz : vitesse de rotation autour de l'axe vertical, en rad/s.
    """
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    @property
    def is_zero(self) -> bool:
        """Vrai si les trois composantes sont nulles."""
        return self.vx == 0.0 and self.vy == 0.0 and self.wz == 0.0


@dataclass(frozen=True)
class BaseState:
    """État du robot dont la locomotion a besoin, lu à chaque pas de
    commande.

    quat_wxyz    : orientation du corps, sous forme de quaternion (w, x, y,
                   z), une représentation usuelle des rotations en 3D.
    ang_vel_body : vitesse angulaire du corps, en rad/s, dans le repère du
                   corps (voir ci-dessous).
    joint_pos    : position de chaque articulation, indexée par son nom.
    joint_vel    : vitesse de chaque articulation, indexée par son nom.
    timestamp_ns : instant de la lecture, en nanosecondes (par défaut,
                   l'horloge monotone du système, qui ne recule jamais).

    `ang_vel_body` porte son repère dans son nom. Pourquoi : le repère
    d'une vitesse angulaire ne se devine pas, et une erreur de repère fait
    tomber le robot. C'est le repère du corps, comme le rend une vraie
    centrale inertielle (IMU, le capteur qui mesure les rotations du
    robot). Une source qui la lit dans le repère du monde, comme le
    simulateur Isaac Sim de NVIDIA, doit la convertir avant de construire
    cet objet. Mesuré en simulation : avec cette erreur de repère, le G1
    (l'humanoïde Unitree G1) tombe après 103° de virage (voir
    docs/validation_locomotion.md, section 5).
    """
    quat_wxyz: Tuple[float, float, float, float]
    ang_vel_body: Tuple[float, float, float]
    joint_pos: Dict[str, float]
    joint_vel: Dict[str, float]
    timestamp_ns: int = field(default_factory=time.monotonic_ns)


class ILocomotionAdapter(ABC):
    """Traduit une consigne de vitesse en consignes articulaires, d'une
    façon propre à un modèle de robot. Fourni par le driver de ce robot.

    Exemples dans le dépôt, pour l'Unitree G1 : GR00TWBCAdapter, sur la
    politique GR00T Whole-Body Control de NVIDIA
    (axis_os/hal/drivers/unitree_g1/gr00t_wbc_policy.py), et
    G1LocomotionAdapter, sur la politique de marche d'Unitree
    (axis_os/hal/drivers/unitree_g1/locomotion_policy.py).
    """

    @property
    @abstractmethod
    def control_dt(self) -> float:
        """Période de commande attendue par l'adaptateur, en secondes :
        LocomotionService.tick() doit être appelé à ce rythme."""

    @property
    @abstractmethod
    def max_twist(self) -> Twist:
        """Vitesses maximales acceptées. Pourquoi une limite : au-delà, un
        adaptateur appris (une politique entraînée) sort de son domaine
        d'entraînement et extrapole, sans le signaler. set_velocity() borne
        chaque consigne à ces valeurs."""

    # Remet l'adaptateur dans son état initial ; appelé par start().
    @abstractmethod
    def reset(self) -> None: ...

    @abstractmethod
    def step(self, state: BaseState, twist: Twist) -> Dict[str, float]:
        """Un pas de commande : rend une consigne par articulation, indexée
        par son nom. Une vitesse nulle doit donner l'arrêt sûr de ce robot
        (immobile et en équilibre pour un bipède)."""

    # Vrai si le robot est à terre, d'après son état : le service cesse
    # alors toute consigne (exigence 3, en tête du module).
    @abstractmethod
    def is_fallen(self, state: BaseState) -> bool: ...


class LocomotionState(Enum):
    """État du service.

    STOPPED : arrêté, aucune consigne envoyée.
    RUNNING : en marche, une consigne est envoyée à chaque tick().
    FALLEN  : chute détectée ; plus aucune consigne tant que le service
              n'est pas redémarré par start().
    """
    STOPPED = auto()
    RUNNING = auto()
    FALLEN = auto()


class LocomotionService(IService):
    """Service système de locomotion : reçoit les consignes de vitesse (des
    apps, de la navigation) et fait marcher le robot.

    tick() est la boucle de commande, à appeler tous les
    adapter.control_dt. À chaque pas, le service lit l'état du robot,
    vérifie qu'il n'est pas tombé, prend la consigne en cours (ou zéro si
    elle a expiré), la fait plafonner par le gardien de proximité s'il y en
    a un, la fait traduire par l'adaptateur en consignes articulaires, puis
    transmet celles-ci au HAL gardé.
    """

    def __init__(
        self,
        adapter: ILocomotionAdapter,
        read_state: Callable[[], BaseState],
        send_targets: Callable[[Dict[str, float]], Dict[str, HALStatus]],
        command_timeout_s: float = 0.5,
        clock: Callable[[], float] = time.monotonic,
        gardien=None,
        zones=None,
        thermique=None,
    ):
        """
        adapter           : traduction vitesse -> articulations, fournie par
                            le driver (ILocomotionAdapter).
        read_state        : fonction sans argument qui rend l'état actuel du
                            robot (BaseState).
        send_targets      : fonction qui transmet les consignes articulaires
                            et rend un statut par articulation. Ce doit être
                            la méthode gardée du HAL (voir l'en-tête du
                            module).
        command_timeout_s : durée de validité d'une consigne, en secondes ;
                            passé ce délai sans rafraîchissement, la vitesse
                            retombe à zéro.
        clock             : horloge, en secondes (time.monotonic par défaut,
                            une horloge qui ne recule jamais) ; les tests en
                            fournissent une simulée. Le gardien est évalué
                            sur cette horloge : les horodatages du lidar
                            doivent l'utiliser aussi.
        gardien           : GardienProximite (proximite.py), facultatif.
                            S'il est fourni, chaque consigne de vitesse,
                            d'où qu'elle vienne (une app, la navigation ou
                            tout autre émetteur), est plafonnée par lui
                            avant d'atteindre les jambes. Aucune méthode ne
                            permet de le désactiver : une app ne choisit pas
                            la sécurité des humains autour du robot. Son
                            dernier verdict reste lisible dans
                            dernier_verdict.
        zones             : GardienZones (axis_os/services/navigation/
                            zones.py), facultatif : zones interdites et
                            lentes de la carte du site, appliquées à toute
                            consigne après le gardien de proximité. Dernier
                            verdict dans dernier_verdict_zones.
        thermique         : objet qui rend facteur_marche() dans [0, 1]
                            (ThermalService, axis_os/services/health/
                            thermal.py) : la vitesse est multipliée par ce
                            facteur quand le robot chauffe.
        """
        self._adapter = adapter
        self._read_state = read_state
        self._send = send_targets
        self._timeout = command_timeout_s
        self._clock = clock
        self._gardien = gardien
        self._zones = zones
        self._thermique = thermique
        self.dernier_verdict = None
        self.dernier_verdict_zones = None
        # Consigne en cours et instant de sa réception (None : aucune).
        self._twist = Twist()
        self._twist_t: Optional[float] = None
        self.state = LocomotionState.STOPPED
        # Compteurs : pas de commande exécutés (ticks), consignes non nulles
        # expirées (stale), consignes bornées à max_twist (clamped),
        # consignes articulaires refusées par le HAL (refused), chutes
        # (falls), consignes non finies refusées (non_finies).
        self.stats = {"ticks": 0, "stale": 0, "clamped": 0,
                      "refused": 0, "falls": 0, "non_finies": 0}

    # ---- Interface IService, commune à tous les services système --------

    @property
    def service_id(self) -> str:
        return "locomotion"

    @property
    def capability(self) -> Capability:
        """Capacité fournie : WALK, marcher (voir
        axis_os/store/sdk/manifest.py)."""
        return Capability.WALK

    def start(self) -> HALStatus:
        """Démarre le service : réinitialise l'adaptateur et part d'une
        consigne nulle. C'est aussi la seule façon de reprendre la marche
        après une chute (état FALLEN)."""
        self._adapter.reset()
        self._twist, self._twist_t = Twist(), None
        self.state = LocomotionState.RUNNING
        return HALStatus.OK

    def stop(self) -> None:
        """Arrête le service. À distinguer de halt() : après stop(), tick()
        n'envoie plus aucune consigne, alors que halt() continue de faire
        tourner l'adaptateur à vitesse nulle."""
        self.state = LocomotionState.STOPPED

    def health(self) -> ServiceHealth:
        """État de santé : en défaut après une chute ; sinon sain, que le
        service soit en marche ou non."""
        if self.state is LocomotionState.FALLEN:
            return ServiceHealth(running=False, healthy=False,
                                 detail="chute détectée — consignes suspendues")
        return ServiceHealth(running=self.state is LocomotionState.RUNNING,
                             healthy=True, detail=self.state.name)

    @property
    def max_twist(self) -> Twist:
        """Vitesse maximale acceptée sur chaque composante (celle de
        l'adaptateur) : au-delà, une consigne est bornée."""
        return self._adapter.max_twist

    @property
    def is_guarded(self) -> bool:
        """Vrai si un gardien de proximité plafonne les consignes. Lecture
        seule. (Nom choisi pour ne pas ressembler à un réglage du gardien :
        test/test_proximite.py vérifie qu'aucune méthode publique de ce
        service ne parle de le régler ou de le désactiver.)"""
        return self._gardien is not None

    @property
    def is_moving(self) -> bool:
        """Vrai si une consigne de marche non nulle, encore valide, est en
        cours. Lecture seule."""
        return (self._twist_t is not None and not self._twist.is_zero
                and self._clock() - self._twist_t <= self._timeout)

    # ---- API des apps ----------------------------------------------------
    # Appelée par le SDK (la bibliothèque fournie aux apps), par exemple par
    # ctx.walk().

    def set_velocity(self, vx: float = 0.0, vy: float = 0.0,
                     wz: float = 0.0) -> ServiceResult:
        """Nouvelle consigne de vitesse (voir Twist). Non bloquant : elle
        est appliquée par les appels suivants de tick().

        Elle doit être rafraîchie avant command_timeout_s, sinon le robot
        s'arrête (en équilibre). Chaque composante est bornée par
        adapter.max_twist. Une valeur non finie (NaN ou infinie) est
        refusée et remet la consigne à zéro. Refusée aussi si le service
        n'est pas en marche.

        Rend un ServiceResult : ok=True avec la consigne retenue (value),
        accompagnée d'une raison si elle a été bornée ; ok=False avec la
        raison du refus.
        """
        if self.state is not LocomotionState.RUNNING:
            return ServiceResult(ok=False,
                                 reason=f"service {self.state.name}")
        # Une consigne non finie est refusée, et le robot s'arrête en
        # équilibre. Pourquoi : un audit de sécurité (octobre 2026) a montré
        # que le bornage ci-dessous, max(-m, min(m, v)), transforme NaN
        # (« not a number », valeur numérique invalide) en +m : min(m, nan)
        # rend m, car toute comparaison avec NaN est fausse. Un ctx.walk(nan)
        # venu d'une app, ou un connecteur VLA (un modèle d'IA
        # vision-langage-action qui décide des mouvements) laissant passer
        # un NaN, faisait marcher le robot à pleine vitesse vers l'avant ;
        # seul le gardien de proximité freinait, et seulement près d'un
        # obstacle. Une valeur infinie n'a pas davantage de sens : elle est
        # refusée aussi, plutôt que ramenée à la vitesse maximale. La
        # consigne est remise à zéro, et pas seulement ignorée : la consigne
        # précédente, peut-être rapide, ne doit pas continuer jusqu'à son
        # échéance. Testé dans test/test_locomotion_non_finie.py.
        if not all(math.isfinite(v) for v in (vx, vy, wz)):
            self._twist, self._twist_t = Twist(), self._clock()
            self.stats["non_finies"] += 1
            return ServiceResult(
                ok=False,
                reason=f"consigne non finie ({vx}, {vy}, {wz}) refusée — "
                       f"arrêt en équilibre")
        # Bornage de chaque composante à max_twist (voir ILocomotionAdapter).
        m = self._adapter.max_twist
        c = Twist(max(-m.vx, min(m.vx, vx)),
                  max(-m.vy, min(m.vy, vy)),
                  max(-m.wz, min(m.wz, wz)))
        self._twist, self._twist_t = c, self._clock()
        if c != Twist(vx, vy, wz):
            self.stats["clamped"] += 1
            return ServiceResult(
                ok=True, value=c,
                reason=f"consigne bornée à ({c.vx}, {c.vy}, {c.wz}) — "
                       f"au-delà, l'adaptateur sort de son domaine")
        return ServiceResult(ok=True, value=c)

    def halt(self) -> ServiceResult:
        """Arrêt en équilibre : vitesse nulle, l'adaptateur continue de
        tourner et tient le robot debout. Ce n'est pas un arrêt d'urgence
        (e-stop), qui relève du système de sécurité."""
        return self.set_velocity(0.0, 0.0, 0.0)

    # ---- Boucle de commande ----------------------------------------------

    def _effective_twist(self) -> Twist:
        """Consigne à appliquer maintenant : la dernière reçue, ou zéro si
        aucune n'a été reçue ou si elle a expiré (plus vieille que
        command_timeout_s). Une consigne non nulle qui expire est comptée
        dans stats["stale"]."""
        if self._twist_t is None:
            return Twist()
        if self._clock() - self._twist_t > self._timeout:
            if not self._twist.is_zero:
                self.stats["stale"] += 1
            self._twist, self._twist_t = Twist(), None
            return Twist()
        return self._twist

    def tick(self) -> ServiceResult:
        """Un pas de commande, à appeler tous les adapter.control_dt.

        Rend ok=False si le robot est à terre ou si le service est arrêté.
        Sinon ok=True, avec la vitesse effectivement appliquée (value) et,
        le cas échéant, le nombre de consignes articulaires refusées par le
        HAL.
        """
        if self.state is LocomotionState.FALLEN:
            return ServiceResult(ok=False, reason="robot à terre — aucune consigne")
        if self.state is not LocomotionState.RUNNING:
            return ServiceResult(ok=False, reason="service arrêté")

        st = self._read_state()
        # Chute : plus aucune consigne, ni ce pas-ci ni les suivants
        # (exigence 3, en tête du module).
        if self._adapter.is_fallen(st):
            self.state = LocomotionState.FALLEN
            self.stats["falls"] += 1
            return ServiceResult(
                ok=False, reason="chute détectée — plus aucune consigne "
                                 "de marche envoyée ; relève du système de "
                                 "sécurité")

        twist = self._effective_twist()
        if self._gardien is not None:
            v = self._gardien.evaluer(self._clock())
            self.dernier_verdict = v
            if v.facteur <= 0.0:
                # Arrêt en équilibre : consigne nulle, la politique de marche
                # continue de tenir le robot debout. Jamais une coupure des
                # moteurs.
                twist = Twist()
            elif v.facteur < 1.0:
                # Ralentissement : les trois composantes sont multipliées
                # par le même facteur, la direction du mouvement est gardée.
                twist = Twist(twist.vx * v.facteur, twist.vy * v.facteur,
                              twist.wz * v.facteur)
        if self._zones is not None and not twist.is_zero:
            vz = self._zones.limiter(twist.vx, twist.vy, twist.wz)
            self.dernier_verdict_zones = vz
            # Jamais plus vite que la consigne déjà retenue.
            twist = Twist(math.copysign(min(abs(vz.vx), abs(twist.vx)), twist.vx),
                          math.copysign(min(abs(vz.vy), abs(twist.vy)), twist.vy),
                          math.copysign(min(abs(vz.wz), abs(twist.wz)), twist.wz))
        if self._thermique is not None and not twist.is_zero:
            try:
                f = float(self._thermique.facteur_marche())
            except Exception:                  # noqa: BLE001 — dans le doute, arrêt
                f = 0.0
            f = f if math.isfinite(f) else 0.0
            f = max(0.0, min(1.0, f))
            if f < 1.0:
                twist = Twist(twist.vx * f, twist.vy * f, twist.wz * f)
        targets = self._adapter.step(st, twist)
        statuses = self._send(targets)
        # Consignes articulaires refusées par le HAL gardé (statut autre que
        # OK) : comptées et signalées dans le résultat.
        refused = sum(1 for s in statuses.values() if s != HALStatus.OK)
        self.stats["refused"] += refused
        self.stats["ticks"] += 1
        return ServiceResult(ok=True, value=twist,
                             reason=(f"{refused} consigne(s) refusée(s)"
                                     if refused else None))
