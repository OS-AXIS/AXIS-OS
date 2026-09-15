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
import time
from dataclasses import dataclass, asdict
from typing import Optional, Dict, Any, Callable

from axis_os.hal.core.types import (
    ActuatorCommand, ControlMode, HALStatus, IMUData, Vector3, Quaternion,
)
from axis_os.store.sdk.app_base import (
    RobotContext, PermissionError as SandboxPermissionError, EstopActiveError
)


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

            elif cmd.action == "get_full_state":
                state = self._ctx.get_full_state()
                data = {
                    jid: {"position": fb.position, "velocity": fb.velocity,
                          "torque": fb.torque, "temperature": fb.temperature,
                          "state": fb.state.name}
                    for jid, fb in state.items()
                }
                return HALResponse(cmd.request_id, True, data=data)

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
                 timeout_s: float = 5.0):
        self._cmd_q    = command_queue
        self._resp_q   = response_queue
        self._timeout  = timeout_s
        self._req_id   = 0
        self._local_log: list = []

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

    def get_full_state(self) -> Dict[str, Any]:
        return self._call("get_full_state", {})

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
