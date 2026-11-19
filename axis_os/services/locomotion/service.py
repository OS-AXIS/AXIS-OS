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

Posture (LocomotionService.posture) : torse penché en avant, bassin abaissé,
par la consigne de posture de la marche GR00T WBC, pour que les bras
atteignent plus loin. Pour l'OS seulement (le service de manipulation) :
aucune API du SDK n'y donne accès. Bornée (BornesPosture), robot immobile,
zone dégagée selon le gardien de proximité (seul le retour vers la posture
droite est permis sinon), centre de masse au-dessus des pieds avec une marge
écrite (equilibre.py) ; refusée sinon. Penché, le robot ne marche pas.
"""
from __future__ import annotations

import math
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Callable, Dict, Optional, Sequence, Tuple

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
    pas_physique : en simulation, numéro du dernier pas de physique dont
                   cet état est le résultat ; None sur un vrai robot. Voir
                   LocomotionService.tick : aucune décision (chute,
                   consigne) n'est prise sur un état que la physique n'a
                   pas encore mis à jour depuis le démarrage du service.

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
    pas_physique: Optional[int] = None


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

    @property
    def bande_morte(self) -> Optional[float]:
        """Vitesse (m/s, rad/s en rotation) sous laquelle ce robot ne bouge
        plus vraiment : la bande morte de sa marche, propre à chaque robot.
        Lecture seule. None : non déclarée (l'app de tri garde alors son
        plancher, 0.06 m/s)."""
        return None


@dataclass(frozen=True)
class BornesPosture:
    """Bornes de la consigne de posture (LocomotionService.posture).
    Domaine d'entraînement de la politique GR00T WBC non documenté : valeurs
    prudentes, à confirmer sur GPU. La taille du G1 tangue de 0,52 rad au
    plus ; la marge d'équilibre est celle du modèle statique (equilibre.py,
    sans recul des hanches, donc prudent)."""
    tangage_max_rad: float = 0.35          # torse penché en avant, au plus
    abaissement_max_m: float = 0.10        # bassin abaissé, au plus
    vitesse_tangage_rad_s: float = 0.2     # la consigne y va peu à peu
    vitesse_abaissement_m_s: float = 0.05
    # Centre de masse à 2 cm au moins du bord de l'appui : décision de revue,
    # gardée jusqu'à la validation sur GPU (banc lancer_posture.sh).
    marge_min_m: float = 0.02


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
        bornes_posture: BornesPosture = BornesPosture(),
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
        self.bornes_posture = bornes_posture
        # Posture : consigne retenue et consigne en cours, (tangage du torse
        # en rad, abaissement du bassin en m) ; la seconde va vers la
        # première peu à peu (tick). Dernier bilan d'équilibre accepté.
        self._posture_cible: Tuple[float, float] = (0.0, 0.0)
        self._posture: Tuple[float, float] = (0.0, 0.0)
        self.equilibre_posture = None
        self._modele_corps = None
        # Motif d'interdiction de toute marche (verrouiller()), ou None.
        self.verrou: Optional[str] = None
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
                      "refused": 0, "falls": 0, "non_finies": 0,
                      "attentes_physique": 0}
        # Pas de physique au premier tick après start() (simulation) ; None
        # une fois la physique avancée : à partir de là, chaque état compte.
        self._pas_depart: Optional[int] = None
        self._physique_avancee = False

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
        consigne nulle, robot droit. C'est aussi la seule façon de reprendre
        la marche après une chute (état FALLEN)."""
        self._adapter.reset()
        self._twist, self._twist_t = Twist(), None
        # Droit, comme l'adaptateur réinitialisé : jamais de retour à une
        # posture d'avant l'arrêt, que rien n'a vérifiée de nouveau.
        self._posture_cible = self._posture = (0.0, 0.0)
        self.equilibre_posture = None
        self._pas_depart, self._physique_avancee = None, False
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
    def bande_morte(self) -> Optional[float]:
        """Bande morte de la marche déclarée par l'adaptateur (voir
        ILocomotionAdapter.bande_morte), ou None. Lecture seule."""
        return getattr(self._adapter, "bande_morte", None)

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
        if self.verrou and not (vx == 0.0 and vy == 0.0 and wz == 0.0):
            return ServiceResult(ok=False, reason=f"marche interdite : {self.verrou}")
        if self.penche and not (vx == 0.0 and vy == 0.0 and wz == 0.0):
            return ServiceResult(ok=False, reason=f"marche refusée : robot penché "
                                                  f"({self._texte_posture()}), à redresser d'abord")
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

    def verrouiller(self, motif: Optional[str]) -> None:
        """Interdit toute marche (`motif`), ou la permet de nouveau (None).
        Verrouillé, le robot s'arrête en équilibre, et aucune consigne reçue
        ensuite ne le fait repartir, d'où qu'elle vienne. Cas visé : batterie
        critique (EnergyService). Constaté : le dernier walk() d'une app,
        reçu entre l'arrêt et la mort de l'app, faisait encore avancer le
        robot de ~2 cm jusqu'au pas suivant du service d'énergie."""
        self.verrou = motif
        if motif:
            self._twist, self._twist_t = Twist(), self._clock()

    def halt(self) -> ServiceResult:
        """Arrêt en équilibre : vitesse nulle, l'adaptateur continue de
        tourner et tient le robot debout. Ce n'est pas un arrêt d'urgence
        (e-stop), qui relève du système de sécurité."""
        return self.set_velocity(0.0, 0.0, 0.0)

    # ---- Posture : torse penché, bassin abaissé (pour l'OS seulement) ----

    @property
    def posture_disponible(self) -> bool:
        """L'adaptateur prend-il une consigne de posture (GR00T WBC), avec
        l'URDF de son robot pour le modèle d'équilibre ?"""
        a = self._adapter
        return (bool(getattr(a, "posture_disponible", False))
                and callable(getattr(a, "posture", None))
                and getattr(a, "urdf_equilibre", None) is not None)

    @property
    def penche(self) -> bool:
        """Hors de la posture droite (consigne retenue ou en cours)."""
        return max(abs(self._posture_cible[0]), abs(self._posture[0]),
                   self._posture_cible[1], self._posture[1]) > 1e-4

    @property
    def posture_actuelle(self) -> Tuple[float, float]:
        """(tangage du torse en rad, abaissement du bassin en m) en cours."""
        return self._posture

    @property
    def posture_atteinte(self) -> bool:
        return all(abs(a - b) < 1e-6 for a, b in zip(self._posture, self._posture_cible))

    def _texte_posture(self, posture: Optional[Tuple[float, float]] = None) -> str:
        t, a = posture or self._posture_cible
        return f"torse penché de {math.degrees(t):.0f}°, bassin abaissé de {100 * a:.0f} cm"

    def posture(self, tangage: float = 0.0, abaissement: float = 0.0,
                angles: Optional[Dict[str, float]] = None,
                charges: Sequence[Tuple[float, Sequence[float]]] = (),
                essai: bool = False) -> ServiceResult:
        """Consigne de posture : torse penché en avant de `tangage` (rad),
        bassin abaissé de `abaissement` (m) ; (0, 0) : droit. Pour l'OS (le
        service de manipulation) : aucune API du SDK n'y mène.

        `angles` : angles prévus des bras (sinon les mesurés) ; `charges` :
        (masse en kg, position dans le repère du bassin), le carton porté.
        `essai` : vérifie sans rien changer.

        Refusée : service arrêté, adaptateur sans posture, valeur non finie
        ou hors des bornes (BornesPosture), robot qui marche, gardien de
        proximité qui arrête le robot (personne ou obstacle inexpliqué trop
        près, ou pas de balayage : seul le retour vers la posture droite
        reste permis, il éloigne le torse de ce qui est devant), centre de
        masse à moins de marge_min_m du bord de l'appui des pieds. La
        consigne est appliquée peu à peu par tick() ; ok=True rend le bilan
        d'équilibre (value), la marge écrite dans reason."""
        b = self.bornes_posture
        if self.state is not LocomotionState.RUNNING:
            return ServiceResult(ok=False, reason=f"posture refusée : service {self.state.name}")
        if not self.posture_disponible:
            return ServiceResult(ok=False, reason="posture refusée : cette marche ne prend pas "
                                                  "de consigne de posture")
        if not all(math.isfinite(v) for v in (tangage, abaissement)):
            return ServiceResult(ok=False, reason=f"posture refusée : valeur non finie "
                                                  f"({tangage}, {abaissement})")
        if not (0.0 <= tangage <= b.tangage_max_rad and 0.0 <= abaissement <= b.abaissement_max_m):
            return ServiceResult(ok=False, reason=(
                f"posture refusée : torse penché de 0 à {b.tangage_max_rad:.2f} rad, bassin "
                f"abaissé de 0 à {100 * b.abaissement_max_m:.0f} cm (demandé {tangage:.2f} rad, "
                f"{100 * abaissement:.0f} cm)"))
        if self.is_moving:
            return ServiceResult(ok=False, reason="posture refusée : le robot marche")
        vers_droit = tangage <= self._posture[0] + 1e-9 and abaissement <= self._posture[1] + 1e-9
        if self._gardien is not None and not vers_droit:
            v = self._gardien.evaluer(self._clock())
            if v.facteur <= 0.0:
                return ServiceResult(ok=False, reason=f"posture refusée : gardien de proximité "
                                                      f"({v.raison})")
        from axis_os.services.locomotion.equilibre import TAILLE, ModeleCorps, jambes
        if self._modele_corps is None:
            self._modele_corps = ModeleCorps(self._adapter.urdf_equilibre)
        try:
            mesures = dict(self._read_state().joint_pos)
        except Exception as exc:               # noqa: BLE001 — sans état, pas de posture
            return ServiceResult(ok=False, reason=f"posture refusée : état du robot illisible "
                                                  f"({type(exc).__name__})")
        valeurs = {**mesures, **(angles or {}), **jambes(-abaissement), TAILLE: tangage}
        e = self._modele_corps.equilibre(valeurs, charges)
        texte = f"{self._texte_posture((tangage, abaissement))} : {e.texte()}"
        if e.marge < b.marge_min_m:
            return ServiceResult(ok=False, value=e, reason=(
                f"posture refusée, équilibre : {texte} (au moins {100 * b.marge_min_m:.0f} cm)"))
        if not essai:
            self._posture_cible = (float(tangage), float(abaissement))
            self.equilibre_posture = e
        return ServiceResult(ok=True, value=e, reason=f"posture : {texte}")

    def _pas_posture(self, gardien_arrete: bool) -> None:
        """Un pas de la posture en cours vers la consigne (tick) ; le gardien
        qui arrête le robot fige ce qui s'éloigne de la posture droite."""
        b, dt = self.bornes_posture, self._adapter.control_dt
        (t, a), (tc, ac) = self._posture, self._posture_cible
        if gardien_arrete:
            tc, ac = min(tc, t), min(ac, a)
        t += max(-b.vitesse_tangage_rad_s * dt, min(b.vitesse_tangage_rad_s * dt, tc - t))
        a += max(-b.vitesse_abaissement_m_s * dt, min(b.vitesse_abaissement_m_s * dt, ac - a))
        change = (t, a) != self._posture
        self._posture = (t, a)
        if change or self.penche:
            self._adapter.posture(hauteur=self._adapter.hauteur_debout - a, rpy=(0.0, t, 0.0))

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
        if not self._physique_avancee and st.pas_physique is not None:
            # Simulation : tant que la physique n'a pas avancé depuis le
            # démarrage, l'état lu n'en est pas le résultat (constaté sur
            # GPU, 2026-10-28 : chute déclarée à t = 0, avant le premier
            # pas). On n'en tire rien, et on n'envoie rien. Dès qu'un pas a
            # eu lieu, la détection de chute s'applique à chaque état.
            if self._pas_depart is None:
                self._pas_depart = st.pas_physique
            if st.pas_physique <= self._pas_depart:
                self.stats["attentes_physique"] += 1
                return ServiceResult(ok=False, reason="état pas encore mis à jour par la "
                                                      "physique — aucune consigne")
            self._physique_avancee = True
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
        if self.verrou or self.penche:
            twist = Twist()
        v = None
        if self._gardien is not None:
            v = self._gardien.evaluer(self._clock())
            self.dernier_verdict = v
        if self.posture_disponible and (self.penche or self._posture != self._posture_cible):
            self._pas_posture(gardien_arrete=v is not None and v.facteur <= 0.0)
        if v is not None:
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
