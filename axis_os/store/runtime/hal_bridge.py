"""
AXIS-OS — HAL Command Bridge
==============================
Pattern Command/Response Queue — inspiré de ROS2 (publisher/subscriber, RPC services).

Résout la limite du SandboxRunner : un process enfant isolé ne peut pas
avoir d'accès direct au HAL (RobotContext non picklable, contient des threads).

Solution : le process enfant envoie des COMMANDES SÉRIALISABLES (dicts)
via une Queue. Le process PARENT (qui a le vrai accès HAL) les exécute
réellement et renvoie le résultat par une autre Queue.

                Process ENFANT (app isolée)
                        │
                command_queue.put({"action": "move_arms", ...})
                        │
                Process PARENT (vrai HAL)
                        │
                exécute réellement sur RobotContext
                        │
                response_queue.put({"status": "ok", ...})
                        │
                Process ENFANT reçoit le résultat

Aucun objet HAL ne traverse jamais la frontière process — seulement
des dicts JSON-compatibles. C'est le même principe que les messages
ROS2 (.msg) : données pures, jamais d'objets vivants.

Licence : Propriétaire — AXIS-OS
"""

from __future__ import annotations
import multiprocessing as mp
import queue
import threading
import time
from dataclasses import dataclass, asdict
from typing import Optional, Dict, Any, Callable

from axis_os.hal.core.types import (
    ActuatorCommand, ActuatorFeedback, ActuatorState, ControlMode, HALStatus,
    IMUData, Vector3, Quaternion, JointGroup, infer_joint_group,
)
from axis_os.store.sdk.app_base import (
    RobotContext, PermissionError as SandboxPermissionError, EstopActiveError
)
from axis_os.store.sdk.manifest import Permission


# ─────────────────────────────────────────────────────────────────
#  MESSAGES — dicts purs, toujours picklables
# ─────────────────────────────────────────────────────────────────

@dataclass
class HALCommand:
    """Commande envoyée par le process enfant. 100% sérialisable."""
    action: str                      # "move_arms" | "move_legs" | "get_state" | "log"
    payload: Dict[str, Any]
    request_id: int


@dataclass
class HALResponse:
    """Réponse renvoyée par le process parent."""
    request_id: int
    success:    bool
    data:       Any = None
    error:      Optional[str] = None


# ─────────────────────────────────────────────────────────────────
#  CÔTÉ PARENT — exécute réellement sur le HAL
# ─────────────────────────────────────────────────────────────────

def _serialize_feedback(fb: ActuatorFeedback) -> Dict[str, Any]:
    """ActuatorFeedback -> dict transportable par multiprocessing.Queue."""
    return {
        "joint_id": fb.joint_id, "state": fb.state.name,
        "position": fb.position, "velocity": fb.velocity,
        "torque": fb.torque, "current": fb.current,
        "temperature": fb.temperature, "voltage": fb.voltage,
        "error_code": fb.error_code, "timestamp_ns": fb.timestamp_ns,
    }


def _deserialize_feedback(jid: str, d: Dict[str, Any]) -> ActuatorFeedback:
    """dict -> ActuatorFeedback réel.

    Ferme une divergence de TYPE DE RETOUR entre les deux chemins :
    RobotContext.get_full_state() rendait des ActuatorFeedback
    (fb.position) tandis que HALBridgeClient rendait des dicts
    (fb["position"]). Une app écrite contre la documentation de
    RobotContext fonctionnait en test direct et plantait UNIQUEMENT en
    sandbox — donc uniquement sur le vrai robot, au pire moment pour
    s'en apercevoir. Constaté réellement (AttributeError: 'dict' object
    has no attribute 'position').

    Tolère les dicts partiels d'une version antérieure du protocole
    (champs absents -> valeurs neutres) plutôt que de lever : une app
    qui tourne ne doit pas mourir sur une incompatibilité de version du
    pont.
    """
    return ActuatorFeedback(
        joint_id=d.get("joint_id", jid),
        state=ActuatorState[d["state"]] if "state" in d else ActuatorState.IDLE,
        position=float(d.get("position", 0.0)),
        velocity=float(d.get("velocity", 0.0)),
        torque=float(d.get("torque", 0.0)),
        current=float(d.get("current", 0.0)),
        temperature=float(d.get("temperature", 0.0)),
        voltage=float(d.get("voltage", 0.0)),
        error_code=int(d.get("error_code", 0)),
    )


class HALBridgeServer:
    """
    Tourne dans le process PARENT. Écoute les commandes du process
    enfant sandboxé et les exécute sur le vrai RobotContext.

    Usage :
        server = HALBridgeServer(ctx, command_queue, response_queue)
        server.start()          # lance un thread d'écoute
        # ... le process enfant tourne ...
        server.stop()
    """

    def __init__(self, ctx: RobotContext,
                 command_queue: mp.Queue, response_queue: mp.Queue):
        self._ctx     = ctx
        self._cmd_q   = command_queue
        self._resp_q  = response_queue
        self._running = False
        self._thread: Optional[Any] = None
        # Trouvé nécessaire par un audit de sécurité (M3) : la
        # facturation reposait uniquement sur result.success, construit
        # par l'app elle-même dans le process ENFANT — une app
        # malveillante retourne toujours success=True sans jamais
        # bouger le robot, et déclenche quand même la facturation
        # client. Ce compteur, tenu côté PARENT (donc hors de portée de
        # l'app), enregistre chaque commande move_arms/move_legs qui a
        # RÉELLEMENT abouti (au moins un HALStatus.OK dans le résultat)
        # — une preuve indépendante que PlayStoreRuntime peut exiger en
        # plus de result.success avant de facturer (voir
        # _run_mission_locked()).
        self.successful_motor_commands = 0

    @property
    def had_any_motor_success(self) -> bool:
        return self.successful_motor_commands > 0

    def start(self) -> None:
        import threading
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                        name="hal_bridge_server")
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def _loop(self) -> None:
        while self._running:
            try:
                raw = self._cmd_q.get(timeout=0.1)
            except queue.Empty:
                continue
            if raw is None:  # poison pill
                break

            # Trouvé par un audit de sécurité (E8) : HALCommand(**raw)
            # vivait hors de tout try/except — raw vient du process
            # ENFANT (côté non fiable de la frontière de sécurité, une
            # app potentiellement bogguée ou malveillante). Un message
            # malformé (clé manquante, clé en trop) levait un TypeError
            # non rattrapé qui tuait ce thread daemon SILENCIEUSEMENT :
            # plus aucune commande n'était servie ensuite, l'app ne
            # voyait que des TimeoutError sans jamais savoir pourquoi,
            # et aucune alerte ne remontait nulle part.
            try:
                cmd = HALCommand(**raw)
            except TypeError as exc:
                print(f"  [SECURITY] Message HAL bridge malformé ignoré "
                      f"(thread reste vivant) : {exc} | raw={raw!r}")
                request_id = raw.get("request_id") if isinstance(raw, dict) else None
                if request_id is not None:
                    self._resp_q.put(asdict(HALResponse(
                        request_id, False,
                        error=f"MALFORMED_COMMAND: {exc}",
                    )))
                continue

            response = self._handle(cmd)
            self._resp_q.put(asdict(response))

    def _handle(self, cmd: HALCommand) -> HALResponse:
        try:
            if cmd.action == "get_joint_groups":
                # Groupes DÉCLARÉS par le driver, transmis en noms d'enum
                # (str) : le routage bras/jambes ne doit pas dépendre de
                # mots-clés anglais devinés côté client, et JointGroup
                # n'est pas sérialisable tel quel sans imposer l'import
                # côté process enfant.
                out = {}
                try:
                    for jdef in self._ctx._robot.get_joint_definitions():
                        g = (jdef.group if jdef.group is not None
                             else infer_joint_group(jdef.joint_id))
                        out[jdef.joint_id] = g.name
                except Exception:       # noqa: BLE001
                    pass
                return HALResponse(cmd.request_id, True, data=out)

            if cmd.action == "move_arms":
                commands = self._deserialize_commands(cmd.payload["commands"])
                result = self._ctx.move_arms(commands)
                # Compteur indépendant — voir __init__ (M3).
                if any(status == HALStatus.OK for status in result.values()):
                    self.successful_motor_commands += 1
                return HALResponse(cmd.request_id, True,
                                   data={k: v.name for k, v in result.items()})

            elif cmd.action == "move_legs":
                commands = self._deserialize_commands(cmd.payload["commands"])
                result = self._ctx.move_legs(commands)
                if any(status == HALStatus.OK for status in result.values()):
                    self.successful_motor_commands += 1
                return HALResponse(cmd.request_id, True,
                                   data={k: v.name for k, v in result.items()})

            elif cmd.action == "move_all":
                # Trouvé manquant côté pont par un audit externe : sans
                # cette action, waist_yaw_joint restait inatteignable
                # par toute app sandboxée — ni un bras ni une jambe,
                # ce joint n'est routable que par move_all(). L'appel
                # self._ctx.move_all() ci-dessous EST la vérification
                # de sécurité : self._ctx est le vrai RobotContext
                # (voir __init__ de cette classe), donc
                # _require(Permission.ACTUATORS_WRITE_ALL) s'exécute
                # réellement ici, côté PROCESS PARENT — jamais dans le
                # process enfant non fiable où tourne l'app sandboxée.
                # Voir test_context_parity.py section 2 pour la
                # vérification explicite que ce garde ne peut pas être
                # contourné depuis le client.
                commands = self._deserialize_commands(cmd.payload["commands"])
                result = self._ctx.move_all(commands)
                if any(status == HALStatus.OK for status in result.values()):
                    self.successful_motor_commands += 1
                return HALResponse(cmd.request_id, True,
                                   data={k: v.name for k, v in result.items()})

            elif cmd.action == "get_full_state":
                state = self._ctx.get_full_state()
                # Sérialise TOUS les champs d'ActuatorFeedback, pas un
                # sous-ensemble : le client reconstruit désormais de vrais
                # ActuatorFeedback (voir _deserialize_feedback), et une
                # reconstruction partielle mentirait silencieusement sur
                # current/voltage/error_code en les remplaçant par des
                # valeurs par défaut plausibles.
                data = {jid: _serialize_feedback(fb) for jid, fb in state.items()}
                return HALResponse(cmd.request_id, True, data=data)

            elif cmd.action == "get_joint_state":
                # Même remarque que move_all ci-dessus : self._ctx.
                # get_joint_state() applique _require(Permission.
                # ACTUATORS_READ) côté serveur, avant même de lire le
                # feedback réel.
                joint_id = cmd.payload.get("joint_id", "")
                fb = self._ctx.get_joint_state(joint_id)
                if fb is None:
                    return HALResponse(cmd.request_id, False,
                                       error=f"Joint '{joint_id}' introuvable")
                return HALResponse(cmd.request_id, True, data={
                    "position": fb.position, "velocity": fb.velocity,
                    "torque": fb.torque, "temperature": fb.temperature,
                    "state": fb.state.name,
                })

            elif cmd.action == "get_imu":
                imu = self._ctx.get_imu()
                if imu is None:
                    return HALResponse(cmd.request_id, False, error="IMU absente")
                data = imu.read()
                if data is None:
                    return HALResponse(cmd.request_id, False, error="IMU ne répond pas")
                return HALResponse(cmd.request_id, True, data={
                    "acc": [data.linear_acceleration.x, data.linear_acceleration.y,
                            data.linear_acceleration.z],
                    "gyro": [data.angular_velocity.x, data.angular_velocity.y,
                             data.angular_velocity.z],
                    "orientation": [data.orientation.x, data.orientation.y,
                                    data.orientation.z, data.orientation.w],
                    "timestamp_ns": data.timestamp_ns,
                })

            elif cmd.action == "log":
                self._ctx.log(cmd.payload.get("message", ""))
                return HALResponse(cmd.request_id, True)

            elif cmd.action == "emergency_stop":
                # Voir RobotContext.emergency_stop() (correctif M11,
                # audit de sécurité) pour le contexte complet.
                status = self._ctx.emergency_stop()
                return HALResponse(cmd.request_id, True, data={"status": status.name})

            elif cmd.action == "get_recent_alerts":
                # Voir WatchdogSystem.get_recent_alerts() pour le
                # contexte complet — self._ctx (le vrai RobotContext)
                # n'expose pas cette méthode directement (elle vit sur
                # WatchdogSystem, pas RobotContext) ; _require(
                # Permission.WATCHDOG_READ) est donc appliqué ICI,
                # explicitement, plutôt que délégué à self._ctx comme
                # les autres actions — seule exception à ce pattern
                # dans ce fichier, documentée pour qu'un futur ajout
                # d'action n'oublie pas ce garde en copiant le
                # mauvais exemple.
                if Permission.WATCHDOG_READ not in self._ctx._manifest.permissions:
                    return HALResponse(cmd.request_id, False,
                                       error="Permission refusée : WATCHDOG_READ "
                                             "non déclarée")
                since_index = cmd.payload.get("since_index", 0)
                alerts = self._ctx._watchdog.get_recent_alerts(since_index)
                return HALResponse(cmd.request_id, True, data={
                    "alerts": [
                        {"kind": a.kind.name, "severity": a.severity.name,
                         "message": a.message, "joint_id": a.joint_id}
                        for a in alerts
                    ],
                    "next_index": since_index + len(alerts),
                })

            else:
                return HALResponse(cmd.request_id, False,
                                   error=f"Action inconnue : {cmd.action}")

        except SandboxPermissionError as exc:
            return HALResponse(cmd.request_id, False, error=f"Permission refusée : {exc}")
        except EstopActiveError as exc:
            # Propagé distinctement — un e-stop survenu pendant qu'une
            # app sandboxée exécute sa mission doit être identifiable
            # côté enfant comme une interruption de sécurité, pas une
            # erreur applicative générique (voir HALBridgeClient._call).
            return HALResponse(cmd.request_id, False, error=f"ESTOP_ACTIVE:{exc}")
        except Exception as exc:
            return HALResponse(cmd.request_id, False, error=f"{type(exc).__name__}: {exc}")

    @staticmethod
    def _deserialize_commands(raw: Dict[str, dict]) -> Dict[str, ActuatorCommand]:
        return {
            jid: ActuatorCommand(
                mode=ControlMode[c["mode"]],
                position=c.get("position"), velocity=c.get("velocity"),
                torque=c.get("torque"), current=c.get("current"),
                # stiffness/damping — trouvés perdus à la traversée du
                # pont par un audit de sécurité (M7) : ni sérialisés ni
                # désérialisés, rendant le mode IMPEDANCE annoncé mais
                # non fonctionnel une fois une app sandboxée (ce qui
                # est TOUJOURS le cas pour run_mission()/on_start()/
                # on_stop() depuis le correctif C4).
                stiffness=c.get("stiffness"), damping=c.get("damping"),
            )
            for jid, c in raw.items()
        }


# ─────────────────────────────────────────────────────────────────
#  CÔTÉ ENFANT — proxy que l'app sandboxée utilise
# ─────────────────────────────────────────────────────────────────

class _IMUProxy:
    """
    Proxy local retourné par HALBridgeClient.get_imu(), compatible
    avec le contrat IIMU (interfaces.py) — expose .read() -> Optional
    [IMUData], au lieu de retourner directement un dict brut. Voir
    HALBridgeClient.get_imu() pour le contexte complet du correctif.
    """

    def __init__(self, client: "HALBridgeClient"):
        self._client = client

    def read(self) -> Optional[IMUData]:
        raw = self._client._call("get_imu", {})
        if raw is None:
            return None
        acc = raw["acc"]
        gyro = raw["gyro"]
        orientation = raw["orientation"]
        return IMUData(
            linear_acceleration=Vector3(x=acc[0], y=acc[1], z=acc[2]),
            angular_velocity=Vector3(x=gyro[0], y=gyro[1], z=gyro[2]),
            orientation=Quaternion(
                x=orientation[0], y=orientation[1],
                z=orientation[2], w=orientation[3],
            ),
            timestamp_ns=raw["timestamp_ns"],
        )


class HALBridgeClient:
    """
    Tourne dans le process ENFANT (sandboxé). Remplace RobotContext
    pour les apps isolées — même API de surface, mais chaque appel
    part en IPC vers le vrai HAL côté parent.

    L'app ne voit AUCUNE différence avec un RobotContext normal
    pour les opérations supportées.
    """

    def __init__(self, command_queue: mp.Queue, response_queue: mp.Queue,
                 timeout_s: float = 5.0, alert_poll_interval_s: float = 0.5):
        self._cmd_q    = command_queue
        self._resp_q   = response_queue
        self._timeout  = timeout_s
        self._req_id   = 0
        self._local_log: list = []
        # Voir subscribe_alerts() pour le contexte complet — polling
        # en arrière-plan, seule adaptation possible d'un vrai
        # callback à travers la frontière de process.
        self._alert_poll_thread: Optional[threading.Thread] = None
        self._alert_poll_stop = threading.Event()
        self._alert_poll_interval_s = alert_poll_interval_s
        self.__groups_cache: Optional[Dict[str, str]] = None

    def move_arms(self, commands: Dict[str, ActuatorCommand]) -> Dict[str, HALStatus]:
        """
        Retourne de vrais HALStatus, pas les chaînes brutes — bug
        trouvé en corrigeant C4 (sandboxing de on_start/on_stop) :
        le serveur sérialise déjà HALStatus.name en chaîne pour le
        transport IPC (nécessaire, les enums ne sont pas garantis
        picklables tels quels selon le contexte), mais rien côté
        client ne les reconstruisait. Une app qui teste
        `status.value` ou `status == HALStatus.OK` sur le résultat
        (comme package_sorting.py) échouait avec une AttributeError,
        jamais exercé avant que on_start() soit réellement sandboxé.
        """
        raw = self._call("move_arms", {"commands": self._serialize(commands)})
        return {jid: HALStatus[name] for jid, name in raw.items()}

    def move_legs(self, commands: Dict[str, ActuatorCommand]) -> Dict[str, HALStatus]:
        raw = self._call("move_legs", {"commands": self._serialize(commands)})
        return {jid: HALStatus[name] for jid, name in raw.items()}

    def move_all(self, commands: Dict[str, ActuatorCommand]) -> Dict[str, HALStatus]:
        """
        Équivalent de RobotContext.move_all() — trouvé manquant côté
        pont par un audit externe. Sans cette méthode, waist_yaw_joint
        restait inatteignable par toute app sandboxée : ce n'est ni un
        bras ni une jambe (voir le routage par mots-clés dans
        move_arms/move_legs), donc seul move_all() peut l'atteindre.
        La vérification de permission (ACTUATORS_WRITE_ALL) a lieu
        côté SERVEUR (voir HALBridgeServer._handle(), action
        "move_all") — jamais ici, dans ce process client non fiable.
        """
        raw = self._call("move_all", {"commands": self._serialize(commands)})
        return {jid: HALStatus[name] for jid, name in raw.items()}

    def _joint_groups(self) -> Dict[str, str]:
        """Groupes déclarés par le HAL, récupérés une fois via le pont.

        Les noms d'enum transitent en str : JointGroup n'est pas
        sérialisable tel quel à travers multiprocessing.Queue sans
        imposer l'import côté enfant.
        """
        if self.__groups_cache is None:
            try:
                self.__groups_cache = self._call("get_joint_groups", {})
            except Exception:       # noqa: BLE001 — serveur ancien
                self.__groups_cache = {}
        return self.__groups_cache

    def get_full_state(self) -> Dict[str, ActuatorFeedback]:
        """Identique à RobotContext.get_full_state(), Y COMPRIS LE TYPE
        DE RETOUR : de vrais ActuatorFeedback, pas des dicts sérialisés.
        Le transport par dict reste interne au pont."""
        raw = self._call("get_full_state", {})
        return {jid: _deserialize_feedback(jid, d) for jid, d in raw.items()}

    def get_joint_state(self, joint_id: str) -> Optional[Dict[str, Any]]:
        """
        Équivalent de RobotContext.get_joint_state() — retourne un dict
        brut (position/velocity/torque/temperature/state), pas un
        ActuatorFeedback réel, cohérent avec get_full_state() côté
        sandbox (voir sa docstring de classe : même API de surface,
        pas nécessairement les mêmes types internes). None si le
        joint n'existe pas — le serveur signale ce cas précis par un
        RuntimeError générique (voir _call(), aucune exception dédiée
        pour ce cas), donc capturé ici spécifiquement plutôt que
        toute exception, pour ne jamais masquer un vrai TimeoutError
        ou EstopActiveError qui doivent, eux, remonter normalement.
        """
        try:
            d = self._call("get_joint_state", {"joint_id": joint_id})
            return _deserialize_feedback(joint_id, d) if d else None
        except RuntimeError:
            return None

    def move_to_pose(
        self,
        target: Dict[str, float],
        duration_s: float = 2.0,
        step_s: float = 0.05,
    ) -> bool:
        """
        Équivalent de RobotContext.move_to_pose() (voir sa docstring
        pour le contexte complet — correctif suivant C5, audit de
        sécurité) — DIFFÈRE uniquement dans la lecture du feedback :
        get_full_state() retourne désormais de vrais ActuatorFeedback
        des deux côtés du pont (fb.position)
        comme côté RobotContext non sandboxé. Toute app doit pouvoir
        appeler move_to_pose() identiquement, sandboxée ou non — c'est
        exactement le rôle de HALBridgeClient (même API de surface que
        RobotContext, voir sa docstring de classe) ; réimplémenté ici
        plutôt que factorisé pour ne pas introduire de dépendance
        croisée entre process parent et enfant sur ce détail interne.
        """
        import time as _time

        state = self.get_full_state()
        current: Dict[str, float] = {}
        for jid in target:
            fb = state.get(jid)
            if fb is None:
                self.log(f"move_to_pose: joint '{jid}' introuvable — abandon.")
                return False
            current[jid] = fb.position

        n_steps = max(1, int(duration_s / step_s))
        for step in range(1, n_steps + 1):
            fraction = step / n_steps
            step_commands: Dict[str, ActuatorCommand] = {}
            for jid, target_pos in target.items():
                interpolated = current[jid] + (target_pos - current[jid]) * fraction
                step_commands[jid] = ActuatorCommand(
                    mode=ControlMode.POSITION, position=interpolated,
                )

            # Même routage par groupe DÉCLARÉ que RobotContext, pas de
            # devinement par mots-clés anglais. Côté client du pont, on
            # n'a pas get_joint_definitions() sous la main : on demande
            # les groupes au serveur, qui les tient du HAL.
            groups = self._joint_groups()
            arm_cmds = {jid: c for jid, c in step_commands.items()
                        if groups.get(jid) == JointGroup.MANIPULATION.name}
            leg_cmds = {jid: c for jid, c in step_commands.items()
                        if groups.get(jid) == JointGroup.LOCOMOTION.name}
            unrouted = set(step_commands) - set(arm_cmds) - set(leg_cmds)
            if unrouted:
                self.log(f"move_to_pose: joint(s) non routable(s) (ni bras ni "
                        f"jambe) : {sorted(unrouted)} — abandon.")
                return False

            results: Dict[str, HALStatus] = {}
            if arm_cmds:
                results.update(self.move_arms(arm_cmds))
            if leg_cmds:
                results.update(self.move_legs(leg_cmds))

            failed = [jid for jid, s in results.items() if s != HALStatus.OK]
            if failed:
                self.log(f"move_to_pose: pas {step}/{n_steps} échoué pour "
                        f"{failed} — abandon (interpolation trop agressive, "
                        f"e-stop, ou autre violation de sécurité).")
                return False

            _time.sleep(step_s)

        return True

    def get_imu(self) -> "Optional[_IMUProxy]":
        """
        Retourne un proxy exposant `.read() -> Optional[IMUData]`,
        cohérent avec le contrat `IIMU` réel (voir interfaces.py) —
        bug trouvé par un audit de sécurité, révélé en corrigeant C4
        (sandboxing de on_start/on_stop) : cette méthode retournait
        directement un dict brut, alors que les apps existantes
        (package_sorting.py notamment) appellent `ctx.get_imu().read()`
        exactement comme avec le vrai RobotContext non sandboxé. Avant
        ce correctif, ce chemin n'avait jamais été exercé en usage réel
        (on_start n'était jamais sandboxé), donc l'incohérence était
        restée invisible.

        L'appel réseau réel (`_call("get_imu", {})`) n'a lieu qu'au
        moment où `.read()` est appelé sur le proxy, pas à la
        construction du proxy — cohérent avec IIMU.read(), qui peut
        retourner None si le capteur ne répond pas au moment précis
        de l'appel plutôt qu'une seule fois au début de la mission.
        """
        return _IMUProxy(self)

    def log(self, message: str) -> None:
        self._local_log.append(message)
        self._call("log", {"message": message})

    def subscribe_alerts(self, callback) -> None:
        """
        Équivalent de RobotContext.subscribe_alerts() — trouvé
        manquant côté pont par un audit externe. Adaptation nécessaire
        et non triviale : un vrai Callable Python ne peut PAS
        traverser une multiprocessing.Queue (non sérialisable), donc
        le process enfant sandboxé ne peut structurellement pas
        recevoir un vrai callback synchrone du Watchdog comme le fait
        RobotContext.subscribe_alerts() (délégation directe à
        WatchdogSystem.subscribe(), même process). Implémenté ici par
        POLLING en arrière-plan (WatchdogSystem.get_recent_alerts(),
        voir sa docstring pour le contexte complet) — même signature
        publique (callback appelé à chaque nouvelle alerte) que
        RobotContext, mais latence de polling (self._alert_poll_interval_s)
        au lieu d'un vrai temps réel synchrone. Documenté explicitement
        plutôt que silencieusement approximé : une app qui a besoin
        d'une latence d'alerte garantie ne doit pas tourner sandboxée.

        Un seul callback actif à la fois côté sandbox (contrairement
        à RobotContext, qui accumule les abonnés sur
        WatchdogSystem._subscribers) — suffisant pour l'usage prévu
        (une app sandboxée, un seul point d'écoute), et évite de
        multiplier les threads de polling par abonnement.
        """
        if self._alert_poll_thread is not None:
            return  # déjà abonné — no-op, cohérent avec un seul callback actif
        self._alert_poll_thread = threading.Thread(
            target=self._alert_poll_loop, args=(callback,), daemon=True,
            name="hal-bridge-alert-poll",
        )
        self._alert_poll_thread.start()

    def _alert_poll_loop(self, callback) -> None:
        since_index = 0
        while not self._alert_poll_stop.is_set():
            try:
                raw = self._call("get_recent_alerts", {"since_index": since_index})
            except (TimeoutError, RuntimeError):
                break  # pont fermé (voir HALBridgeServer.stop()) — arrêt propre
            except (OSError, EOFError, BrokenPipeError):
                # Le process parent (ou le process courant en fin de
                # script) a détruit les Queue sous ce thread daemon —
                # trouvé en testant ce mécanisme : un script qui se
                # termine pendant qu'un abonnement est actif ferme les
                # multiprocessing.Queue avant que ce thread daemon ne
                # s'arrête proprement. Arrêt silencieux ici plutôt
                # qu'un traceback bruyant en fin de process — ce n'est
                # pas une erreur applicative, juste l'ordre normal de
                # destruction des ressources en fin de vie du process.
                break
            for alert_dict in raw["alerts"]:
                try:
                    callback(alert_dict)
                except Exception:
                    pass  # un callback défaillant ne doit jamais tuer ce thread
            since_index = raw["next_index"]
            self._alert_poll_stop.wait(self._alert_poll_interval_s)

    def emergency_stop(self) -> HALStatus:
        """Voir RobotContext.emergency_stop() (correctif M11, audit de
        sécurité) — même surface d'API, transmise via le pont IPC."""
        raw = self._call("emergency_stop", {})
        return HALStatus[raw["status"]]

    def get_mission_log(self) -> list:
        """
        Contrairement aux autres méthodes, le journal de mission est
        tenu localement dans le process ENFANT plutôt que redemandé au
        parent à chaque fois : chaque log() envoyé est aussi accumulé
        ici, pour rendre HALBridgeClient utilisable en remplacement
        direct de RobotContext sans changer une ligne dans les apps
        existantes (PackageSorting, SecurityPatrol, WarehouseOps
        appellent toutes ctx.get_mission_log() en fin de mission).
        """
        return list(self._local_log)

    def _call(self, action: str, payload: Dict[str, Any]) -> Any:
        self._req_id += 1
        req_id = self._req_id
        self._cmd_q.put(asdict(HALCommand(action=action, payload=payload,
                                          request_id=req_id)))
        deadline = time.monotonic() + self._timeout
        while time.monotonic() < deadline:
            try:
                raw = self._resp_q.get(timeout=0.1)
            except queue.Empty:
                continue
            resp = HALResponse(**raw)
            if resp.request_id != req_id:
                continue  # réponse d'un autre appel — ignorer (best-effort)
            if not resp.success:
                if resp.error and resp.error.startswith("ESTOP_ACTIVE:"):
                    raise EstopActiveError(resp.error[len("ESTOP_ACTIVE:"):])
                raise RuntimeError(f"HAL bridge erreur [{action}]: {resp.error}")
            return resp.data
        raise TimeoutError(f"HAL bridge timeout sur '{action}' (> {self._timeout}s)")

    @staticmethod
    def _serialize(commands: Dict[str, ActuatorCommand]) -> Dict[str, dict]:
        return {
            jid: {"mode": c.mode.name, "position": c.position,
                  "velocity": c.velocity, "torque": c.torque,
                  "current": c.current, "stiffness": c.stiffness,
                  "damping": c.damping}
            for jid, c in commands.items()
        }
