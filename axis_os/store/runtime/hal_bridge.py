"""
AXIS-OS — Pont de commandes vers le HAL (HAL Command Bridge)
=============================================================
Modèle commande/réponse par files de messages, inspiré de ROS2 (le
logiciel de communication de référence en robotique) : ses services
RPC (appels de procédure à distance) et son modèle
publication/abonnement.

Le problème : une app du Play Store (le magasin d'apps d'AXIS-OS) tourne
dans un processus enfant isolé, le sandbox lancé par SandboxRunner, pour
qu'un plantage, une boucle infinie ou un dépassement de quota ne touche
pas l'OS. On ne peut pas transmettre à ce processus le HAL (la couche
d'abstraction matérielle, le seul chemin prévu vers les moteurs) :
RobotContext, l'objet par lequel une app commande le robot et qui
vérifie ses permissions, ne se sérialise pas avec pickle (le format
Python qui sérialise des objets), car il contient des threads.

La solution : le processus enfant envoie des commandes faites de
simples données (des dicts) sur le pont. Le processus parent, seul à
détenir le vrai HAL, les exécute et renvoie le résultat par le même pont.
Cet échange forme l'IPC (la communication entre deux processus) de l'app.

                Processus ENFANT (app isolée)
                        │
                canal.envoyer({"type": "appel", "action": "move_arms", ...})
                        │
                Processus PARENT (vrai HAL)
                        │
                exécute la commande sur RobotContext
                        │
                canal.envoyer({"type": "reponse", "success": True, ...})
                        │
                Processus ENFANT reçoit le résultat

Les messages ne doivent contenir que des données simples, compatibles
JSON (le format texte standard d'échange de données : dicts, listes,
nombres, chaînes), jamais un objet HAL vivant : c'est le même principe
que les messages ROS2 (.msg).

Le transport : le pont passe uniquement par canal_ipc.py, qui n'échange
que des octets JSON (trames de taille bornée, JSON strict). Autrefois, des
files de multiprocessing sérialisaient chaque message avec pickle, et
désérialiser un pickle peut exécuter du code dans le processus qui le lit,
ici celui de l'OS : une app malveillante pouvait ainsi faire tourner son
propre code dans l'OS (étape 0 de docs/isolation_apps.md). Ce transport a
été retiré, essais compris. Chaque commande reçue est en outre vérifiée
par le schéma de son action (SCHEMAS_ACTIONS) avant d'être exécutée :
champ inconnu, absent ou du mauvais type, nombre non fini -> refus.

Licence : Apache 2.0, comme tout le dépôt (voir LICENSE).
"""

from __future__ import annotations
import math
import os
import threading
import time
from dataclasses import dataclass, asdict
from typing import Optional, Dict, Any, Callable

from axis_os.hal.core.types import (
    ActuatorCommand, ActuatorFeedback, ActuatorState, ControlMode, HALStatus,
    IMUData, Vector3, Quaternion, JointGroup, infer_joint_group, ORIGINE_IA,
)
from axis_os.store.sdk.app_base import (
    RobotContext, PermissionError as SandboxPermissionError, EstopActiveError
)
from axis_os.store.sdk.lifecycle import (
    CONFIRMATION_MAX_S, PROGRESS_MESSAGE_MAX, QUESTION_MAX, EventPoller, MissionCancelled,
)
from axis_os.store.sdk.animation import Animation, load_animation
from axis_os.store.sdk.manifest import Permission
from axis_os.store.sdk.stream import (
    MAX_JOINTS, StreamAck, StreamMetrics, TargetStream, stream_params, stream_target,
)
from axis_os.store.sdk.results import (
    EXCEPTIONS_PAR_CODE, ActionResult, AxisRefusal, InvalidArgumentError, ObjectSpec,
    RefusalCode, RobotInfo, SafetyStatus, object_from_target, refused, require_language,
    require_number, require_text, surface_height_from_target,
)
from axis_os.store.runtime.canal_ipc import (
    Booleen, Canal, CanalCorrompu, CanalFerme, Chaine, Dictionnaire, Entier, JsonLibre, Liste,
    MessageRefuse,
    Nombre, Objet, UnParmi,
)


# ─────────────────────────────────────────────────────────────────
#  MESSAGES — des données simples (dicts), toujours sérialisables
# ─────────────────────────────────────────────────────────────────

@dataclass
class HALCommand:
    """Commande envoyée par le processus enfant. Entièrement
    sérialisable : elle ne contient que des données simples."""
    # Nom de l'action demandée, par exemple "move_arms" ; la liste
    # complète des actions est dans HALBridgeServer._handle().
    action: str
    payload: Dict[str, Any]
    request_id: int


@dataclass
class HALResponse:
    """Réponse renvoyée par le processus parent, rattachée à sa commande
    par request_id."""
    request_id: int
    success:    bool
    data:       Any = None
    error:      Optional[str] = None


# ─────────────────────────────────────────────────────────────────
#  CÔTÉ PARENT — exécute les commandes sur le vrai HAL
# ─────────────────────────────────────────────────────────────────

def _serialize_detection(d) -> Dict[str, Any]:
    """Detection (un objet repéré par la perception : étiquette,
    confiance, boîte englobante, direction, position 3D) -> dict
    transportable par le pont."""
    def _v(v):
        return None if v is None else [v.x, v.y, v.z]
    return {"label": d.label, "confidence": d.confidence,
            "bbox": list(d.bbox), "direction": _v(d.direction),
            "position_3d": _v(d.position_3d), "timestamp_ns": d.timestamp_ns,
            "frame": getattr(d, "frame", "base")}


def _deserialize_detection(d: Dict[str, Any]):
    """dict -> vraie Detection.

    Pourquoi : comme pour ActuatorFeedback (voir _deserialize_feedback),
    une app sandboxée doit recevoir le même type qu'une app lancée en
    direct. Sinon, une app écrite d'après la documentation
    (`det.position_3d.x`) fonctionne en test et plante en sandbox,
    c'est-à-dire uniquement sur le vrai robot.
    """
    from axis_os.hal.core.types import Vector3
    from axis_os.services.perception.service import Detection

    def _v(raw):
        return None if raw is None else Vector3(raw[0], raw[1], raw[2])

    return Detection(
        label=d.get("label", ""), confidence=float(d.get("confidence", 0.0)),
        bbox=tuple(d.get("bbox", (0, 0, 0, 0))),
        direction=_v(d.get("direction")), position_3d=_v(d.get("position_3d")),
        timestamp_ns=int(d.get("timestamp_ns", 0)),
        frame="world" if d.get("frame") == "world" else "base",
    )


def _serialize_feedback(fb: ActuatorFeedback) -> Dict[str, Any]:
    """ActuatorFeedback (l'état mesuré d'un moteur : position, vitesse,
    couple, courant, température, tension, code d'erreur) -> dict
    transportable par le pont."""
    return {
        "joint_id": fb.joint_id, "state": fb.state.name,
        "position": fb.position, "velocity": fb.velocity,
        "torque": fb.torque, "current": fb.current,
        "temperature": fb.temperature, "voltage": fb.voltage,
        "error_code": fb.error_code, "timestamp_ns": fb.timestamp_ns,
    }


def _deserialize_feedback(jid: str, d: Dict[str, Any]) -> ActuatorFeedback:
    """dict -> vrai ActuatorFeedback.

    Pourquoi : les deux chemins (app lancée en direct, app en sandbox)
    ne rendaient pas le même type. RobotContext.get_full_state() rendait
    des ActuatorFeedback (fb.position), tandis que HALBridgeClient
    rendait des dicts (fb["position"]). Une app écrite d'après la
    documentation de RobotContext fonctionnait en test direct et
    plantait uniquement en sandbox, donc uniquement sur le vrai robot, au
    pire moment pour s'en apercevoir. Le cas a été constaté réellement
    (AttributeError: 'dict' object has no attribute 'position').

    Un dict partiel, venu d'une version antérieure du protocole, est
    accepté : les champs absents prennent des valeurs neutres plutôt que
    de lever une exception, car une app en cours d'exécution ne doit pas
    mourir d'une incompatibilité de version du pont.

    timestamp_ns n'est pas repris : l'objet reconstruit porte l'heure de
    sa reconstruction (valeur par défaut d'ActuatorFeedback), pas celle
    de la mesure.
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


# ─────────────────────────────────────────────────────────────────
#  SCHÉMAS — ce que chaque action accepte, et rien d'autre
# ─────────────────────────────────────────────────────────────────

_VIDE = Objet({})
_COMMANDE = Objet({"mode": UnParmi(*ControlMode.__members__)}, facultatifs={
    **{champ: Nombre() for champ in
       ("position", "velocity", "torque", "current", "stiffness", "damping")},
    # Marque d'une consigne venue d'un modèle d'IA (voir
    # RobotContext._exiger_pilotage_ia).
    "origine": UnParmi(ORIGINE_IA),
})
_COMMANDES = Objet({"commands": Dictionnaire(_COMMANDE, max_len=128)})

SCHEMAS_ACTIONS: Dict[str, Objet] = {
    "now": _VIDE,
    "remember": Objet({"key": Chaine(256)}, facultatifs={"value": JsonLibre()}),
    "recall": Objet({"key": Chaine(256)}, facultatifs={"default": JsonLibre()}),
    "listen": Objet({}, facultatifs={"duration_s": Nombre(0, 120)}),
    "declare_intent": Objet({"name": Chaine(128),
                             "keywords": Liste(Liste(Chaine(64), 32), 16)},
                            # Le type documenté d'IntentDeclaration.params :
                            # des chaînes courtes.
                            facultatifs={"params": Dictionnaire(Chaine(256), 32),
                                         "description": Chaine(1024)}),
    "look_for": Objet({}, facultatifs={"query": Chaine(512), "frame": Chaine(64)}),
    "describe_scene": _VIDE,
    "walk": Objet({}, facultatifs={"vx": Nombre(), "vy": Nombre(), "wz": Nombre(),
                                   "origine": UnParmi(ORIGINE_IA)}),
    "stop_walking": _VIDE,
    "get_joint_groups": _VIDE,
    "move_arms": _COMMANDES,
    "move_legs": _COMMANDES,
    "move_all": _COMMANDES,
    "get_full_state": _VIDE,
    "get_joint_state": Objet({"joint_id": Chaine(128)}),
    "get_imu": _VIDE,
    "log": Objet({}, facultatifs={"message": Chaine(4096)}),
    "emergency_stop": _VIDE,
    "get_recent_alerts": Objet({}, facultatifs={"since_index": Entier(0, 2 ** 31)}),
    "robot_info": _VIDE,
    "has_capability": Objet({"name": Chaine(64)}),
    "safety_status": _VIDE,
    # Bornes larges ici ; les bornes exactes sont celles de RobotContext
    # (require_number), vérifiées à nouveau côté OS.
    "go_to": Objet({"x": Nombre(-1e4, 1e4), "y": Nombre(-1e4, 1e4),
                    "timeout_s": Nombre(0, RobotContext.MAX_ACTION_S)},
                   facultatifs={"theta": Nombre(-2 * math.pi, 2 * math.pi)}),
    "pick": Objet({"center": Liste(Nombre(), 3), "size": Liste(Nombre(), 3),
                   "timeout_s": Nombre(0, RobotContext.MAX_ACTION_S)}),
    "place": Objet({"timeout_s": Nombre(0, RobotContext.MAX_ACTION_S)},
                   facultatifs={"surface_height_m": Nombre(0, 2.5)}),
    "rest_pose": Objet({}, facultatifs={"duration_s": Nombre(0, 30)}),
    "should_stop": _VIDE,
    "report_progress": Objet({"fraction": Nombre(0, 1),
                              "message": Chaine(PROGRESS_MESSAGE_MAX)}),
    "subscribe_event": Objet({"kind": UnParmi("estop", "alert", "low_battery")},
                             facultatifs={"threshold_percent": Nombre(1, 99)}),
    "poll_events": Objet({}, facultatifs={"since_index": Entier(0, 2 ** 31)}),
    "ask_confirmation": Objet({"question": Chaine(QUESTION_MAX),
                               "timeout_s": Nombre(0, CONFIRMATION_MAX_S)}),
    # Bornes exactes : stream_params() / stream_target(), côté OS.
    "stream_open": Objet({"kind": UnParmi("arms", "walk"), "deadman_ms": Nombre(),
                          "max_rate_hz": Nombre(), "max_age_ms": Nombre(),
                          "max_speed": Nombre()}),
    "stream_push": Objet({"stream_id": Entier(0, 2 ** 31), "deadman": Booleen(),
                          "t": Nombre()},
                         facultatifs={"joints": Dictionnaire(Nombre(), MAX_JOINTS),
                                      "velocity": Liste(Nombre(), 3)}),
    "stream_resume": Objet({"stream_id": Entier(0, 2 ** 31)}),
    "stream_metrics": Objet({"stream_id": Entier(0, 2 ** 31)}),
    "stream_close": Objet({"stream_id": Entier(0, 2 ** 31)}),
    # Exactement un de « name » et « animation » (vérifié par le serveur) ;
    # le contenu de l'animation est vérifié par load_animation() puis par
    # check_animation(), côté OS.
    "play_animation": Objet({"wait": Booleen()},
                            facultatifs={"name": Chaine(64), "animation": JsonLibre(),
                                         "timeout_s": Nombre(0, RobotContext.ANIMATION_TIMEOUT_MAX_S)}),
    "stop_animation": _VIDE,
    "speak": Objet({"text": Chaine(RobotContext.SPEAK_MAX_CHARS), "language": Chaine(8)}),
    "get_setting": Objet({"name": Chaine(32)}),
}

# Actions longues dont le délai est borné par le temps qui reste au hook
# (voir HALBridgeServer.traiter) : champ du délai, valeur minimale.
_DELAIS_BORNES = {"go_to": ("timeout_s", 0.1), "pick": ("timeout_s", 0.1),
                  "place": ("timeout_s", 0.1), "ask_confirmation": ("timeout_s", 0.1),
                  "play_animation": ("timeout_s", 0.1)}
# Délai par défaut quand l'app n'en donne pas (avant la borne).
_DELAIS_DEFAUT = {"play_animation": RobotContext.ANIMATION_TIMEOUT_MAX_S}

_ENVELOPPE = Objet({"action": Chaine(64), "payload": Dictionnaire(JsonLibre(), 64),
                    "request_id": Entier(0, 2 ** 53)},
                   facultatifs={"type": UnParmi("appel")})


def _sans_controle(texte: str) -> str:
    """Remplace les caractères de contrôle (séquences d'échappement du
    terminal, sauts de ligne) par « ? » : une ligne de journal venue d'une
    app ne doit pas pouvoir réécrire ce qu'affiche le terminal de l'OS."""
    return "".join(c if c.isprintable() else "?" for c in texte)


class HALBridgeServer:
    """
    Tourne dans le processus PARENT. Écoute les commandes du processus
    enfant (l'app sandboxée) et les exécute sur le vrai RobotContext.

    Les permissions sont donc vérifiées ici, dans le parent, par le vrai
    RobotContext, et jamais dans le processus de l'app, qui n'est pas
    fiable. Cela arrête une app boguée ; face à une app malveillante,
    voir la limite connue décrite en tête de ce fichier.

    Usage normal : SandboxRunner.run_app() lit les trames de l'app et
    appelle traiter() pour chacune. Avec un `canal` (essais, ou tout autre
    appelant qui tient lui-même une extrémité du pont) :
        server = HALBridgeServer(ctx, canal=Canal(sock))
        server.start()          # lance un thread qui sert le canal
        server.stop()
    """

    def __init__(self, ctx: RobotContext, canal: Optional[Canal] = None):
        """`canal` : extrémité OS du pont JSON, servie par start() ; None
        quand SandboxRunner.run_app() appelle traiter() lui-même."""
        self._ctx     = ctx
        self._canal   = canal
        self._running = False
        self._thread: Optional[Any] = None
        # Pourquoi ce compteur : un audit de sécurité a montré que la
        # facturation reposait uniquement sur result.success, une valeur
        # construite par l'app elle-même dans le processus enfant. Une
        # app malveillante pouvait renvoyer success=True sans jamais
        # bouger le robot, et déclencher quand même la facturation du
        # client. Ce compteur est tenu côté parent, hors de portée de
        # l'app : il enregistre chaque commande move_arms, move_legs ou
        # move_all qui a réellement abouti (au moins un HALStatus.OK dans
        # le résultat). C'est une preuve indépendante que PlayStoreRuntime
        # (la partie de l'OS qui installe, lance et facture les apps)
        # exige en plus de result.success avant de facturer (voir
        # _run_mission_locked() dans play_store.py).
        self.successful_motor_commands = 0

    @property
    def had_any_motor_success(self) -> bool:
        return self.successful_motor_commands > 0

    def start(self) -> None:
        if self._canal is None:
            raise ValueError("HALBridgeServer.start() : aucun canal à servir")
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                        name="hal_bridge_server")
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def _loop(self) -> None:
        """Sert le canal : chaque appel reçoit sa réponse ; un message
        refusé (JSON invalide, type inattendu) est signalé et ignoré, et le
        thread reste vivant. Un canal fermé ou corrompu arrête la boucle."""
        while self._running:
            try:
                msg = self._canal.recevoir(timeout=0.1)
            except TimeoutError:
                continue
            except MessageRefuse as exc:
                print(f"  [SECURITY] Message HAL bridge refusé (le pont continue) : "
                      f"{_sans_controle(str(exc))[:300]}")
                continue
            except (CanalFerme, CanalCorrompu):
                break
            if msg.get("type") != "appel":
                print(f"  [SECURITY] Message HAL bridge de type inattendu refusé "
                      f"(le pont continue) : {str(msg.get('type'))[:40]!r}")
                continue
            reponse = self.traiter(msg)
            if reponse is None:
                continue
            reponse["type"] = "reponse"
            try:
                self._canal.envoyer(reponse)
            except (TypeError, ValueError) as exc:
                self._canal.envoyer({"type": "reponse", "request_id": reponse.get("request_id"),
                                     "success": False, "data": None,
                                     "error": f"réponse non transmissible : {exc}"[:500]})
            except CanalFerme:
                break

    def traiter(self, raw: Any, reste_s: Optional[float] = None) -> Optional[Dict[str, Any]]:
        """Une commande reçue de l'app (déjà décodée) -> la réponse à lui
        renvoyer, ou None si le message ne permet même pas de répondre
        (aucun request_id lisible).

        raw vient du côté non fiable de la frontière de sécurité (une app
        peut-être boguée ou malveillante). L'enveloppe puis le contenu
        sont vérifiés par schéma (SCHEMAS_ACTIONS) ; un message refusé est
        signalé sur la console (étiquette [SECURITY]) et l'app reçoit une
        erreur MALFORMED_COMMAND. Historique : un audit de sécurité avait
        montré qu'un message malformé (clé manquante ou en trop) levait un
        TypeError non rattrapé, qui tuait le thread du serveur sans bruit ;
        plus aucune commande n'était servie et aucune alerte ne remontait.
        """
        request_id = raw.get("request_id") if isinstance(raw, dict) else None
        if isinstance(request_id, bool) or not isinstance(request_id, int):
            request_id = None
        try:
            enveloppe = _ENVELOPPE.verifier(raw, "commande")
            schema = SCHEMAS_ACTIONS.get(enveloppe["action"])
            if schema is not None:
                schema.verifier(raw["payload"], f"commande.payload[{enveloppe['action']}]")
        except MessageRefuse as exc:
            print(f"  [SECURITY] Message HAL bridge refusé (le pont continue) : "
                  f"{_sans_controle(str(exc))[:300]}")
            if request_id is None:
                return None
            return asdict(HALResponse(request_id, False, error=f"MALFORMED_COMMAND: {exc}"))
        payload = raw["payload"]
        if raw["action"] == "listen" and reste_s is not None:
            # Une écoute ne dure jamais plus que ce qui reste au hook (voir
            # SandboxRunner.run_app) : ce thread de l'OS doit rendre la main
            # avant l'échéance du quota.
            # La moitié du temps restant au plus : l'autre moitié est laissée
            # à la transcription, qui suit la capture dans ce même thread.
            # Une transcription plus longue reste possible (voir
            # SECURITY.md, limite n° 6).
            duree = payload.get("duration_s")
            duree = 3.0 if duree is None else duree
            payload = dict(payload, duration_s=max(0.0, min(duree, (reste_s - 0.5) / 2)))
        borne = _DELAIS_BORNES.get(raw["action"])
        if borne is not None and reste_s is not None:
            # Une action longue rend la main avant l'échéance du quota du
            # hook. On ne fait que RÉDUIRE le délai demandé : un délai
            # invalide (0) reste invalide et sera refusé par RobotContext.
            champ, mini = borne
            demande = payload.get(champ)
            demande = _DELAIS_DEFAUT.get(raw["action"], 2.0) if demande is None else demande
            payload = dict(payload, **{champ: min(demande, max(mini, reste_s - 0.5))})
        cmd = HALCommand(action=raw["action"], payload=payload,
                         request_id=raw["request_id"])
        self._reste_s = reste_s
        return asdict(self._handle(cmd))

    def _handle(self, cmd: HALCommand) -> HALResponse:
        try:
            if cmd.action == "now":
                return HALResponse(cmd.request_id, True, data=self._ctx.now())

            if cmd.action == "remember":
                return HALResponse(cmd.request_id, True,
                                   data=self._ctx.remember(
                                       cmd.payload["key"],
                                       cmd.payload.get("value")))

            if cmd.action == "recall":
                return HALResponse(cmd.request_id, True,
                                   data=self._ctx.recall(
                                       cmd.payload["key"],
                                       cmd.payload.get("default")))

            if cmd.action == "listen":
                intent = self._ctx.listen(cmd.payload.get("duration_s", 3.0))
                return HALResponse(cmd.request_id, True, data=(
                    None if intent is None else
                    {"name": intent.name, "params": dict(intent.params),
                     "confidence": intent.confidence,
                     "utterance": intent.utterance}))

            if cmd.action == "declare_intent":
                p = cmd.payload
                return HALResponse(cmd.request_id, True,
                                   data=self._ctx.declare_intent(
                                       p["name"], p["keywords"],
                                       p.get("params"), p.get("description", "")))

            if cmd.action == "look_for":
                dets = self._ctx.look_for(cmd.payload.get("query"),
                                          cmd.payload.get("frame", "base"))
                return HALResponse(cmd.request_id, True,
                                   data=[_serialize_detection(d) for d in dets])

            if cmd.action == "describe_scene":
                return HALResponse(cmd.request_id, True,
                                   data=self._ctx.describe_scene())

            if cmd.action == "walk":
                p = cmd.payload
                return HALResponse(cmd.request_id, True,
                                   data=self._ctx.walk(p.get("vx", 0.0),
                                                       p.get("vy", 0.0),
                                                       p.get("wz", 0.0),
                                                       origine=p.get("origine")))

            if cmd.action == "stop_walking":
                return HALResponse(cmd.request_id, True,
                                   data=self._ctx.stop_walking())

            if cmd.action == "get_joint_groups":
                # Groupes d'articulations (bras, jambes…) tels que
                # DÉCLARÉS par le driver (le pilote propre à chaque
                # robot) ; à défaut de déclaration, le groupe est déduit
                # du nom (infer_joint_group). JointGroup est une enum (un
                # type Python à valeurs nommées) ; chaque groupe est
                # transmis sous forme de chaîne, le nom de sa valeur, pour
                # deux raisons : le routage bras/jambes ne doit pas
                # dépendre de mots-clés anglais devinés côté client, et
                # JointGroup ne traverse pas le pont tel quel sans obliger
                # le processus enfant à l'importer.
                # Même permission que get_full_state, que move_to_pose
                # appelle de toute façon juste avant.
                self._ctx._require(Permission.ACTUATORS_READ)
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
                # Preuve de mouvement réel pour la facturation : voir
                # __init__.
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
                # Un audit externe a relevé que cette action manquait au
                # pont : sans elle, aucune app sandboxée ne pouvait
                # atteindre waist_yaw_joint (la rotation de la taille),
                # qui n'est ni un bras ni une jambe et n'est routable que
                # par move_all().
                # L'appel self._ctx.move_all() ci-dessous EST la
                # vérification de sécurité : self._ctx est le vrai
                # RobotContext (voir __init__), donc
                # _require(Permission.ACTUATORS_WRITE_ALL) s'exécute ici,
                # dans le processus parent, et jamais dans le processus
                # enfant non fiable où tourne l'app. La section 1 de
                # test/test_hal_bridge_extended.py vérifie que ce garde ne
                # peut pas être contourné depuis le client.
                commands = self._deserialize_commands(cmd.payload["commands"])
                result = self._ctx.move_all(commands)
                if any(status == HALStatus.OK for status in result.values()):
                    self.successful_motor_commands += 1
                return HALResponse(cmd.request_id, True,
                                   data={k: v.name for k, v in result.items()})

            elif cmd.action == "get_full_state":
                state = self._ctx.get_full_state()
                # Tous les champs d'ActuatorFeedback sont sérialisés, pas
                # un sous-ensemble : le client reconstruit de vrais
                # ActuatorFeedback (voir _deserialize_feedback), et une
                # reconstruction partielle mentirait sans bruit sur
                # current, voltage et error_code en les remplaçant par des
                # valeurs par défaut plausibles.
                data = {jid: _serialize_feedback(fb) for jid, fb in state.items()}
                return HALResponse(cmd.request_id, True, data=data)

            elif cmd.action == "get_joint_state":
                # Comme pour move_all ci-dessus :
                # self._ctx.get_joint_state() applique
                # _require(Permission.ACTUATORS_READ) côté serveur, avant
                # même de lire l'état mesuré du moteur (le « feedback »).
                # Seul un sous-ensemble des champs est renvoyé (voir
                # HALBridgeClient.get_joint_state).
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
                self._ctx.log(_sans_controle(cmd.payload.get("message") or ""))
                return HALResponse(cmd.request_id, True)

            elif cmd.action == "emergency_stop":
                # Arrêt d'urgence (e-stop) demandé par l'app elle-même. La
                # permission EMERGENCY_STOP est vérifiée par le vrai
                # RobotContext ; le contexte complet (un audit de sécurité
                # externe avait relevé qu'aucune app ne pouvait déclencher
                # un e-stop) est dans la docstring de
                # RobotContext.emergency_stop().
                status = self._ctx.emergency_stop()
                return HALResponse(cmd.request_id, True, data={"status": status.name})

            elif cmd.action == "get_recent_alerts":
                # Seule exception, dans ce fichier, à la règle « déléguer
                # la vérification de permission à self._ctx » :
                # get_recent_alerts() appartient à WatchdogSystem (le
                # chien de garde qui surveille les moteurs et émet des
                # alertes), pas à RobotContext. Le contrôle de
                # Permission.WATCHDOG_READ est donc fait ICI,
                # explicitement : la permission doit figurer dans le
                # manifeste de l'app (la fiche où l'app déclare les droits
                # qu'elle demande). C'est signalé pour qu'une future action
                # du même genre, écrite en copiant les autres, n'oublie
                # pas ce contrôle. Contexte complet : docstring de
                # WatchdogSystem.get_recent_alerts().
                if not self._ctx._manifest.has_permission(Permission.WATCHDOG_READ):
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

            elif cmd.action == "robot_info":
                return HALResponse(cmd.request_id, True,
                                   data=self._ctx.robot_info().to_json())

            elif cmd.action == "has_capability":
                return HALResponse(cmd.request_id, True,
                                   data=self._ctx.has_capability(cmd.payload["name"]))

            elif cmd.action == "safety_status":
                return HALResponse(cmd.request_id, True,
                                   data=self._ctx.safety_status().to_json())

            elif cmd.action == "speak":
                return HALResponse(cmd.request_id, True, data=self._ctx.speak(
                    cmd.payload["text"], language=cmd.payload["language"]).to_json())

            elif cmd.action == "get_setting":
                return HALResponse(cmd.request_id, True,
                                   data=self._ctx.get_setting(cmd.payload["name"]))

            elif cmd.action == "play_animation":
                p = cmd.payload
                if (p.get("name") is None) == (p.get("animation") is None):
                    raise InvalidArgumentError("play_animation : un nom OU une animation")
                cible = p["name"] if p.get("name") is not None else p["animation"]
                if not isinstance(cible, (str, dict)):
                    raise InvalidArgumentError("animation : un objet au format axis-animation")
                res = self._ctx.play_animation(cible, wait=p["wait"], timeout_s=p.get("timeout_s"))
                if res.status == "PLAYED":
                    # Geste joué jusqu'au bout : mouvement réel (facturation).
                    # Lancé seulement (PLAYING), il peut être arrêté aussitôt.
                    self.successful_motor_commands += 1
                return HALResponse(cmd.request_id, True, data=res.to_json())

            elif cmd.action == "stop_animation":
                return HALResponse(cmd.request_id, True, data=self._ctx.stop_animation().to_json())

            elif cmd.action == "stream_open":
                p = cmd.payload
                flux = self._ctx.stream_targets(
                    p["kind"], deadman_ms=p["deadman_ms"], max_rate_hz=p["max_rate_hz"],
                    max_age_ms=p["max_age_ms"], max_speed=p["max_speed"])
                return HALResponse(cmd.request_id, True, data={
                    "stream_id": flux._id, "joints": list(flux.joints),
                    "limits": flux.limits, "now": self._ctx.now()})

            elif cmd.action == "stream_push":
                p = cmd.payload
                moteur = self._ctx._flux_moteur(p["stream_id"])
                cible = stream_target(moteur.kind, p.get("joints") if moteur.kind == "arms"
                                      else p.get("velocity"))
                ack = self._ctx._flux_push(p["stream_id"], cible, p["deadman"], p["t"])
                # Une vitesse nulle ne fait rien bouger : pas une preuve.
                if ack.accepted and (moteur.kind == "arms" or any(cible)):
                    # Consigne acceptée : le robot la suit (facturation, voir
                    # __init__).
                    self.successful_motor_commands += 1
                return HALResponse(cmd.request_id, True,
                                   data=dict(ack.to_json(), now=self._ctx.now()))

            elif cmd.action == "stream_resume":
                return HALResponse(cmd.request_id, True, data=self._ctx._flux_resume(
                    cmd.payload["stream_id"]).to_json())

            elif cmd.action == "stream_metrics":
                return HALResponse(cmd.request_id, True, data=self._ctx._flux_metrics(
                    cmd.payload["stream_id"]).to_json())

            elif cmd.action == "stream_close":
                self._ctx._flux_close(cmd.payload["stream_id"])
                return HALResponse(cmd.request_id, True, data=None)

            elif cmd.action == "should_stop":
                # Le motif seul (estop, human, os) : check_cancelled() du
                # client lève la même exception qu'en direct.
                self._ctx._require(Permission.MISSION_LIFECYCLE)
                return HALResponse(cmd.request_id, True,
                                   data={"reason": self._ctx._motif_arret()})

            elif cmd.action == "report_progress":
                p = cmd.payload
                self._ctx.report_progress(p["fraction"], p["message"])
                return HALResponse(cmd.request_id, True, data=None)

            elif cmd.action == "subscribe_event":
                p = cmd.payload
                return HALResponse(cmd.request_id, True, data=self._ctx._peut_abonner(
                    p["kind"], p.get("threshold_percent")))

            elif cmd.action == "poll_events":
                # Rien à scruter sans au moins une permission d'événement.
                if not any(self._ctx._manifest.has_permission(p) for p in (
                        Permission.MISSION_LIFECYCLE, Permission.WATCHDOG_READ,
                        Permission.SYSTEM_INFO)):
                    raise SandboxPermissionError(
                        "poll_events exige MISSION_LIFECYCLE, WATCHDOG_READ ou SYSTEM_INFO")
                return HALResponse(cmd.request_id, True,
                                   data=self._ctx._etat_evenements(cmd.payload.get("since_index")))

            elif cmd.action == "ask_confirmation":
                p = cmd.payload
                return HALResponse(cmd.request_id, True, data=self._ctx.ask_confirmation(
                    p["question"], timeout_s=p["timeout_s"]).to_json())

            elif cmd.action in ("go_to", "pick", "place", "rest_pose"):
                p = cmd.payload
                if cmd.action == "go_to":
                    res = self._ctx.go_to(p["x"], p["y"], p.get("theta"),
                                          timeout_s=p["timeout_s"])
                elif cmd.action == "pick":
                    res = self._ctx.pick(ObjectSpec(tuple(p["center"]), tuple(p["size"])),
                                         timeout_s=p["timeout_s"])
                elif cmd.action == "place":
                    res = self._ctx.place(p.get("surface_height_m"), timeout_s=p["timeout_s"])
                else:
                    duree = p.get("duration_s")
                    duree = 2.0 if duree is None else duree
                    reste = getattr(self, "_reste_s", None)
                    # Durée effective (vitesse bornée, voir
                    # RobotContext._duree_repos) : c'est elle qu'il faut avoir.
                    duree = self._ctx._duree_repos(duree)
                    if reste is not None and duree > reste - 0.5:
                        # Pas assez de temps restant au hook : refus, plutôt
                        # qu'un geste raccourci, donc plus rapide.
                        self._ctx._require(Permission.ACTUATORS_READ)
                        self._ctx._require(Permission.ACTUATORS_WRITE_ARMS)
                        return HALResponse(cmd.request_id, True, data=refused(
                            "TIMEOUT", RefusalCode.TIMEOUT,
                            f"il reste {max(0.0, reste):.1f} s au hook, le geste en "
                            f"demande {duree:.1f}").to_json())
                    res = self._ctx.rest_pose(duration_s=duree)
                if res.ok:
                    # Le robot a réellement bougé (arrivé, objet tenu ou
                    # posé, bras au repos) : preuve de mouvement pour la
                    # facturation, comme une commande motrice réussie (voir
                    # __init__).
                    self.successful_motor_commands += 1
                return HALResponse(cmd.request_id, True, data=res.to_json())

            else:
                return HALResponse(cmd.request_id, False,
                                   error=f"Action inconnue : {cmd.action}")

        except SandboxPermissionError as exc:
            # Le TYPE de l'erreur doit traverser le pont, pas seulement
            # son message : d'où ce préfixe, comme ESTOP_ACTIVE
            # ci-dessous. Sans lui, une permission manquante devenait un
            # RuntimeError générique, que les méthodes du client
            # avalaient en renvoyant False. Une app sandboxée ne
            # distinguait plus « je n'ai pas le droit » de « ce robot n'a
            # pas de jambes », alors qu'en direct elle recevait une
            # PermissionError.
            return HALResponse(cmd.request_id, False,
                               error=f"PERMISSION_DENIED:{exc}")
        except EstopActiveError as exc:
            # Propagé à part : un e-stop survenu pendant la mission d'une
            # app sandboxée doit être reconnu côté enfant comme une
            # interruption de sécurité, et non comme une erreur
            # applicative ordinaire (voir HALBridgeClient._call).
            return HALResponse(cmd.request_id, False, error=f"ESTOP_ACTIVE:{exc}")
        except AxisRefusal as exc:
            # Tout autre refus structuré (InvalidArgumentError, annulation…)
            # traverse le pont avec son code, que le client reconvertit en
            # la même exception (voir HALBridgeClient._call).
            return HALResponse(cmd.request_id, False,
                               error=f"{exc.code.value}:{exc.message}")
        except Exception as exc:
            return HALResponse(cmd.request_id, False, error=f"{type(exc).__name__}: {exc}")

    @staticmethod
    def _deserialize_commands(raw: Dict[str, dict]) -> Dict[str, ActuatorCommand]:
        return {
            jid: ActuatorCommand(
                mode=ControlMode[c["mode"]],
                position=c.get("position"), velocity=c.get("velocity"),
                torque=c.get("torque"), current=c.get("current"),
                # stiffness (raideur) et damping (amortissement) : un
                # audit de sécurité a montré qu'ils se perdaient à la
                # traversée du pont, ni sérialisés ni désérialisés. Le
                # mode IMPEDANCE (le moteur se comporte comme un ressort
                # amorti) était donc annoncé mais inopérant dès qu'une
                # app tournait en sandbox, c'est-à-dire toujours pour
                # run_mission(), on_start() et on_stop() : un correctif
                # antérieur, lui aussi issu d'un audit de sécurité
                # externe, fait passer ces trois méthodes par le sandbox
                # (voir CHANGELOG.md).
                stiffness=c.get("stiffness"), damping=c.get("damping"),
                origine=c.get("origine"),
            )
            for jid, c in raw.items()
        }


# ─────────────────────────────────────────────────────────────────
#  CÔTÉ ENFANT — le substitut de RobotContext utilisé par l'app
# ─────────────────────────────────────────────────────────────────

def _origine_connue(origine: Optional[str]) -> None:
    """Même refus qu'en direct (RobotContext._exiger_origine) : une origine
    inconnue lève ValueError avant tout appel, sandboxé ou non."""
    if origine is not None and origine != ORIGINE_IA:
        raise ValueError(f"origine inconnue : {str(origine)[:40]!r}")


class _IMUProxy:
    """
    Objet local renvoyé par HALBridgeClient.get_imu(). Il respecte le
    contrat IIMU (l'interface d'une IMU, la centrale inertielle qui
    mesure accélérations, vitesses de rotation et orientation ; voir
    axis_os/hal/core/interfaces.py) : il expose .read() ->
    Optional[IMUData] au lieu de rendre directement un dict brut. Le
    contexte de ce correctif est dans la docstring de
    HALBridgeClient.get_imu().
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
    Tourne dans le processus ENFANT (l'app sandboxée). Remplace
    RobotContext pour les apps isolées : même API publique, mais chaque
    appel part par IPC vers le vrai HAL, côté parent.

    Pour les opérations prises en charge, l'app ne voit AUCUNE
    différence avec un RobotContext normal ; les rares écarts restants
    sont documentés méthode par méthode (par exemple subscribe_alerts()).
    test/test_context_parity.py vérifie que les deux API publiques ne
    divergent pas.
    """

    def __init__(self, canal: Canal, timeout_s: float = 5.0,
                 alert_poll_interval_s: float = 0.5, *,
                 tolerance_lenteur: Optional[float] = None):
        """`canal` : le pont JSON vers l'OS (dans le processus d'une app,
        voir hote_app.py)."""
        if canal is None:
            raise ValueError("HALBridgeClient : un canal est nécessaire")
        self._canal    = canal
        # Un seul appel à la fois sur le canal : le thread d'abonnement aux
        # alertes et celui de la mission ne doivent pas lire la réponse
        # l'un de l'autre.
        self._verrou_appel = threading.Lock()
        if tolerance_lenteur is not None:
            self.TOLERANCE_LENTEUR = float(tolerance_lenteur)
        self._timeout  = timeout_s
        self._req_id   = 0
        self._local_log: list = []
        # Abonnement aux alertes par interrogation périodique (polling)
        # en arrière-plan : seule façon de reproduire un vrai callback
        # (une fonction de l'app appelée à chaque alerte) à travers la
        # frontière entre processus. Voir subscribe_alerts().
        self._alert_poll_thread: Optional[threading.Thread] = None
        self._alert_poll_stop = threading.Event()
        self._alert_poll_interval_s = alert_poll_interval_s
        self._evenements: Optional[EventPoller] = None
        self._decalage_horloge = 0.0
        self.__groups_cache: Optional[Dict[str, str]] = None

    def move_arms(self, commands: Dict[str, ActuatorCommand]) -> Dict[str, HALStatus]:
        """
        Renvoie de vrais HALStatus, pas les chaînes brutes du transport.

        Le serveur convertit chaque HALStatus en son nom (une chaîne) pour
        le transport IPC, ce qui est nécessaire : selon le contexte, une
        enum n'est pas garantie sérialisable par pickle telle quelle. Mais
        le client ne reconstruisait pas les HALStatus. Une app qui
        testait `status.value` ou `status == HALStatus.OK` sur le
        résultat (comme package_sorting.py) échouait : AttributeError
        dans le premier cas, comparaison toujours fausse dans le second.
        Le bug est apparu quand un audit de sécurité a fait passer
        on_start() et on_stop() par le sandbox : ce chemin n'avait
        jamais été exercé auparavant.
        """
        raw = self._call("move_arms", {"commands": self._serialize(commands)})
        return {jid: HALStatus[name] for jid, name in raw.items()}

    def move_legs(self, commands: Dict[str, ActuatorCommand]) -> Dict[str, HALStatus]:
        raw = self._call("move_legs", {"commands": self._serialize(commands)})
        return {jid: HALStatus[name] for jid, name in raw.items()}

    def move_all(self, commands: Dict[str, ActuatorCommand]) -> Dict[str, HALStatus]:
        """
        Équivalent de RobotContext.move_all(), ajouté au pont après qu'un
        audit externe a relevé son absence. Sans cette méthode, aucune app
        sandboxée ne pouvait atteindre waist_yaw_joint : ce n'est ni un
        bras ni une jambe (move_arms et move_legs routent selon le groupe
        déclaré de chaque articulation), donc seul move_all() peut le
        commander.

        La permission ACTUATORS_WRITE_ALL est vérifiée côté serveur (voir
        HALBridgeServer._handle(), action "move_all"), et jamais ici, dans
        ce processus client non fiable.
        """
        raw = self._call("move_all", {"commands": self._serialize(commands)})
        return {jid: HALStatus[name] for jid, name in raw.items()}

    def now(self) -> float:
        """Heure du robot (et non l'heure réelle), demandée au noyau
        d'AXIS-OS (le processus parent) : en simulation, l'app sandboxée
        n'a pas d'autre moyen de connaître l'heure simulée."""
        return float(self._call("now", {}))

    def sleep(self, secondes: float) -> None:
        """Attend une durée exprimée en temps du robot (et non en temps
        réel), en redemandant l'heure du robot au noyau, avec une pause
        de 5 ms entre deux demandes."""
        fin = self.now() + max(0.0, secondes)
        while self.now() < fin:
            time.sleep(0.005)

    def remember(self, key: str, value) -> bool:
        try:
            return bool(self._call("remember", {"key": key, "value": value}))
        except RuntimeError as exc:
            self.log(f"remember indisponible : {exc}")
            return False

    def recall(self, key: str, default=None):
        try:
            return self._call("recall", {"key": key, "default": default})
        except RuntimeError:
            return default

    def listen(self, duration_s: float = 3.0):
        """Identique à RobotContext.listen(), y compris le type renvoyé :
        un vrai Intent (l'intention reconnue dans ce que l'utilisateur a
        dit), pas un dict sérialisé. La durée d'écoute allonge le délai
        d'attente de la réponse (voir _delai())."""
        try:
            raw = self._call("listen", {"duration_s": duration_s},
                             duree_nominale_s=duration_s)
        except RuntimeError as exc:
            self.log(f"listen indisponible : {exc}")
            return None
        if raw is None:
            return None
        from axis_os.services.speech.service import Intent
        return Intent(name=raw["name"], params=raw.get("params", {}),
                      confidence=float(raw.get("confidence", 0.0)),
                      utterance=raw.get("utterance", ""))

    def declare_intent(self, name: str, keywords, params=None,
                       description: str = "") -> bool:
        try:
            return bool(self._call("declare_intent", {
                "name": name, "keywords": [list(g) for g in keywords],
                "params": params or {}, "description": description}))
        except RuntimeError:
            return False

    def look_for(self, query: Optional[str] = None,
                 frame: str = "base") -> list:
        """Identique à RobotContext.look_for(), y compris le type renvoyé :
        de vraies Detection, pas des dicts sérialisés."""
        try:
            raw = self._call("look_for", {"query": query, "frame": frame})
        except RuntimeError as exc:
            self.log(f"look_for indisponible : {exc}")
            return []
        return [_deserialize_detection(d) for d in (raw or [])]

    def describe_scene(self) -> str:
        try:
            return self._call("describe_scene", {})
        except RuntimeError as exc:
            return f"perception indisponible : {exc}"

    def walk(self, vx: float = 0.0, vy: float = 0.0, wz: float = 0.0, *,
             origine: Optional[str] = None) -> bool:
        _origine_connue(origine)
        try:
            return bool(self._call("walk", {"vx": vx, "vy": vy, "wz": wz,
                                            "origine": origine}))
        except RuntimeError as exc:
            self.log(f"walk refusé : {exc}")
            return False

    def stop_walking(self) -> bool:
        try:
            return bool(self._call("stop_walking", {}))
        except RuntimeError:
            return False

    def _joint_groups(self) -> Dict[str, str]:
        """Groupes d'articulations déclarés par le HAL, demandés une seule
        fois au serveur puis gardés en cache.

        Ils arrivent sous forme de noms (des chaînes) : seules des données
        JSON traversent le pont, et JointGroup n'en est pas une.
        """
        if self.__groups_cache is None:
            try:
                self.__groups_cache = self._call("get_joint_groups", {})
            except Exception:       # noqa: BLE001 — serveur ancien
                self.__groups_cache = {}
        return self.__groups_cache

    def get_full_state(self) -> Dict[str, ActuatorFeedback]:
        """Identique à RobotContext.get_full_state(), y compris le type
        renvoyé : de vrais ActuatorFeedback, pas des dicts sérialisés. Le
        transport par dict reste interne au pont."""
        raw = self._call("get_full_state", {})
        return {jid: _deserialize_feedback(jid, d) for jid, d in raw.items()}

    def get_joint_state(self, joint_id: str) -> Optional[Dict[str, Any]]:
        """
        Équivalent de RobotContext.get_joint_state(). Renvoie un vrai
        ActuatorFeedback, reconstruit par _deserialize_feedback() comme
        dans get_full_state() ; l'annotation de type (un dict) date d'une
        version antérieure qui renvoyait le dict brut. Pour cette action,
        le serveur n'envoie que position, velocity, torque, temperature
        et state : current, voltage et error_code valent donc 0 dans
        l'objet renvoyé.

        Renvoie None si l'articulation n'existe pas. Le serveur signale ce
        cas par une simple erreur, que _call() transforme en RuntimeError
        générique (il n'existe pas d'exception dédiée). Seul RuntimeError
        est donc rattrapé ici, et non toute exception, pour ne jamais
        masquer un vrai TimeoutError ou EstopActiveError, qui doivent
        remonter normalement.
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
        *,
        origine: Optional[str] = None,
    ) -> bool:
        """
        Équivalent de RobotContext.move_to_pose(), dont la docstring donne
        le contexte complet. En bref : un audit de sécurité externe a fait
        borner l'écart de position accepté en une seule commande
        (position_delta_max) ; une cible lointaine ne peut donc plus être
        atteinte d'un coup, et move_to_pose() y va par petits pas
        interpolés depuis la position mesurée (voir CHANGELOG.md).

        Toute app doit pouvoir appeler move_to_pose() de la même façon,
        sandboxée ou non : c'est le rôle même de HALBridgeClient (même
        API publique que RobotContext). La lecture du feedback est
        désormais identique des deux côtés du pont, car get_full_state()
        renvoie de vrais ActuatorFeedback (fb.position). Seule différence
        restante : le routage bras/jambes demande les groupes
        d'articulations au serveur (voir _joint_groups()).

        Le code est réécrit ici plutôt que partagé avec RobotContext, pour
        ne pas créer de dépendance croisée entre processus parent et
        processus enfant sur ce détail interne.
        """
        import time as _time

        _origine_connue(origine)
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
                # La marque d'origine part avec chaque pas : l'OS exige
                # ACTUATORS_AI_DRIVEN dès le premier (move_arms/move_legs).
                step_commands[jid] = ActuatorCommand(
                    mode=ControlMode.POSITION, position=interpolated, origine=origine,
                )

            # Même routage par groupe DÉCLARÉ que RobotContext, sans
            # deviner à partir de mots-clés anglais. Côté client du pont,
            # get_joint_definitions() n'est pas disponible : on demande
            # donc les groupes au serveur, qui les tient du HAL.
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
        Renvoie un proxy (un objet local qui relaie chaque lecture vers le
        serveur) exposant `.read() -> Optional[IMUData]`, comme le contrat
        `IIMU` réel (voir axis_os/hal/core/interfaces.py).

        Pourquoi : un audit de sécurité a révélé que cette méthode
        renvoyait directement un dict brut, alors que les apps existantes
        (package_sorting.py notamment) appellent `ctx.get_imu().read()`,
        exactement comme avec le vrai RobotContext hors sandbox. Le défaut
        est apparu quand on_start() et on_stop() sont passés par le
        sandbox : auparavant, ce chemin n'avait jamais été exercé en usage
        réel (on_start ne tournait pas en sandbox), et l'incohérence était
        restée invisible.

        L'appel au serveur (`_call("get_imu", {})`) n'a lieu qu'au moment
        où `.read()` est appelé sur le proxy, et non à sa construction.
        C'est cohérent avec IIMU.read(), qui peut renvoyer None si le
        capteur ne répond pas au moment précis de l'appel : la mesure est
        prise à chaque lecture, pas une seule fois en début de mission.
        """
        return _IMUProxy(self)

    def log(self, message: str) -> None:
        self._local_log.append(message)
        # Le pont refuse une ligne de plus de 4 096 caractères : la fin
        # d'une ligne trop longue est coupée ici plutôt que de faire
        # échouer l'app.
        self._call("log", {"message": str(message)[:4096]})

    def subscribe_alerts(self, callback) -> None:
        """
        Équivalent de RobotContext.subscribe_alerts(), ajouté au pont
        après qu'un audit externe a relevé son absence.

        L'adaptation n'est pas triviale. Une fonction Python (le callback)
        ne peut pas traverser le pont, où ne passent que des données JSON.
        Le processus enfant ne peut donc pas recevoir un
        vrai callback synchrone du Watchdog, comme le fait RobotContext
        en déléguant à WatchdogSystem.subscribe() dans le même processus.
        Ici, un thread d'arrière-plan interroge périodiquement le serveur
        (polling, par WatchdogSystem.get_recent_alerts() ; voir sa
        docstring). La signature publique est la même (le callback est
        appelé à chaque nouvelle alerte), mais avec la latence de
        l'interrogation (self._alert_poll_interval_s, 0,5 s par défaut)
        au lieu d'un appel immédiat. C'est documenté plutôt qu'approché
        en silence : une app qui a besoin d'une latence d'alerte garantie
        ne doit pas tourner en sandbox.

        Autre différence : le callback reçoit un dict (kind, severity,
        message, joint_id ; kind et severity sous forme de noms), et non
        l'objet alerte du Watchdog.

        Un seul callback est actif à la fois côté sandbox (RobotContext,
        lui, accumule les abonnés dans WatchdogSystem._subscribers). Cela
        suffit pour l'usage prévu (une app sandboxée, un seul point
        d'écoute) et évite un thread d'interrogation par abonnement. Un
        second appel est sans effet.

        Sans la permission WATCHDOG_READ, RobotContext lève PermissionError
        dès l'abonnement. Ici, l'abonnement est accepté, mais le serveur
        refuse la première interrogation : le thread s'arrête alors sans
        prévenir l'app, et le callback n'est jamais appelé.
        """
        if self._alert_poll_thread is not None:
            return  # déjà abonné : sans effet, un seul callback actif
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
                # Pont fermé (voir HALBridgeServer.stop()), ou erreur
                # renvoyée par le serveur, par exemple un refus de la
                # permission WATCHDOG_READ : arrêt propre du thread.
                break
            except (OSError, EOFError, BrokenPipeError):
                # Le pont a été fermé sous ce thread, par le processus
                # parent ou par le processus courant en fin de script : un
                # script qui se termine pendant qu'un abonnement est actif
                # libère ses ressources avant que ce thread « daemon »
                # (arrêté d'office avec le processus) ne s'arrête
                # proprement. On s'arrête alors sans bruit : ce n'est pas
                # une erreur applicative.
                break
            for alert_dict in raw["alerts"]:
                try:
                    callback(alert_dict)
                except Exception:
                    pass  # un callback qui plante ne doit pas tuer ce thread
            since_index = raw["next_index"]
            self._alert_poll_stop.wait(self._alert_poll_interval_s)

    def emergency_stop(self) -> HALStatus:
        """Équivalent de RobotContext.emergency_stop(), transmis par le
        pont IPC. Ajouté après qu'un audit de sécurité externe
        (docs/audit_externe_2026-09-14.md) a relevé qu'aucune app ne
        pouvait déclencher elle-même un arrêt d'urgence ; contexte complet
        dans la docstring de RobotContext.emergency_stop()."""
        raw = self._call("emergency_stop", {})
        return HALStatus[raw["status"]]

    # ---- État, capacités et diagnostic (voir RobotContext) -------------

    def robot_info(self) -> RobotInfo:
        return RobotInfo.from_json(self._call("robot_info", {}))

    def has_capability(self, name: str) -> bool:
        # Même refus qu'en direct, avant d'envoyer : le schéma du pont
        # refuserait un nom qui n'est pas un texte.
        return bool(self._call("has_capability", {"name": require_text("name", name, 64)}))

    def safety_status(self) -> SafetyStatus:
        return SafetyStatus.from_json(self._call("safety_status", {}))

    # ---- Services de haut niveau (voir RobotContext) ---------------------
    # Les arguments sont validés ici comme en direct, avant l'envoi : une
    # app reçoit la même InvalidArgumentError dans les deux cas. L'OS les
    # valide à nouveau de son côté.

    MAX_ACTION_S = RobotContext.MAX_ACTION_S
    api_version: str = RobotContext.api_version

    def go_to(self, x: float, y: float, theta: Optional[float] = None, *,
              timeout_s: float) -> ActionResult:
        payload = {"x": require_number("x", x, mini=-1e4, maxi=1e4),
                   "y": require_number("y", y, mini=-1e4, maxi=1e4),
                   "timeout_s": require_number("timeout_s", timeout_s, mini=0.0,
                                               maxi=self.MAX_ACTION_S, strict_min=True)}
        if theta is not None:
            payload["theta"] = require_number("theta", theta, mini=-2 * math.pi,
                                              maxi=2 * math.pi)
        return ActionResult.from_json(self._call("go_to", payload,
                                                 duree_nominale_s=payload["timeout_s"]))

    def pick(self, target, *, size_m=None, timeout_s: float = 15.0) -> ActionResult:
        spec = object_from_target(target, size_m)
        delai = require_number("timeout_s", timeout_s, mini=0.0, maxi=self.MAX_ACTION_S,
                               strict_min=True)
        return ActionResult.from_json(self._call("pick", {
            "center": list(spec.center), "size": list(spec.size), "timeout_s": delai},
            duree_nominale_s=delai))

    def place(self, target=None, *, timeout_s: float = 15.0) -> ActionResult:
        hauteur = surface_height_from_target(target)
        delai = require_number("timeout_s", timeout_s, mini=0.0, maxi=self.MAX_ACTION_S,
                               strict_min=True)
        return ActionResult.from_json(self._call("place", {
            "surface_height_m": hauteur, "timeout_s": delai}, duree_nominale_s=delai))

    # ---- Parole et réglages (voir RobotContext) ----------------------------

    SPEAK_MAX_CHARS = RobotContext.SPEAK_MAX_CHARS

    def speak(self, text: str, *, language: str = "en") -> ActionResult:
        texte = require_text("text", text, self.SPEAK_MAX_CHARS)
        # Une durée de parole est proportionnelle au texte : environ 15
        # caractères par seconde.
        return ActionResult.from_json(self._call(
            "speak", {"text": texte, "language": require_language(language)},
            duree_nominale_s=len(texte) / 15.0))

    def get_setting(self, name: str):
        return self._call("get_setting", {"name": require_text("name", name, 32)})

    # ---- Animations (voir RobotContext.play_animation) ---------------------

    ANIMATION_TIMEOUT_MAX_S = RobotContext.ANIMATION_TIMEOUT_MAX_S

    def play_animation(self, animation, *, wait: bool = True,
                       timeout_s: Optional[float] = None) -> ActionResult:
        if not isinstance(wait, bool):
            raise InvalidArgumentError("wait doit être un booléen")
        payload: Dict[str, Any] = {"wait": wait}
        if timeout_s is not None:
            payload["timeout_s"] = require_number("timeout_s", timeout_s, mini=0.0,
                                                  maxi=self.ANIMATION_TIMEOUT_MAX_S,
                                                  strict_min=True)
        if isinstance(animation, str):
            payload["name"] = require_text("name", animation, 64)
        elif isinstance(animation, Animation):
            payload["animation"] = animation.to_json()
        elif isinstance(animation, dict):
            # Format vérifié ici aussi : la même InvalidArgumentError qu'en
            # direct, avant l'envoi.
            payload["animation"] = load_animation(animation).to_json()
        else:
            raise InvalidArgumentError("animation : un nom d'animation du robot, un dict au "
                                       "format axis-animation, ou une Animation")
        attente = (timeout_s or self.ANIMATION_TIMEOUT_MAX_S) if wait else 0.0
        return ActionResult.from_json(self._call("play_animation", payload,
                                                 duree_nominale_s=attente))

    def stop_animation(self) -> ActionResult:
        return ActionResult.from_json(self._call("stop_animation", {}))

    # ---- Flux de consignes (voir RobotContext.stream_targets) -------------
    # L'heure de chaque consigne est estimée ici en heure du robot, recalée
    # à chaque réponse (l'OS y joint son heure) : elle sert à l'OS à jeter
    # les consignes trop vieilles.

    def stream_targets(self, kind: str, *, deadman_ms: float = 200.0,
                       max_rate_hz: float = 50.0, max_age_ms: float = 150.0,
                       max_speed: Optional[float] = None) -> TargetStream:
        params = stream_params(kind, deadman_ms, max_rate_hz, max_age_ms, max_speed)
        t0 = time.monotonic()
        rep = self._call("stream_open", params)
        self._recaler_horloge(rep["now"], t0)
        return TargetStream(self, rep["stream_id"], params["kind"], rep)

    def _recaler_horloge(self, maintenant_os: float, t0: float) -> None:
        self._decalage_horloge = float(maintenant_os) - (t0 + time.monotonic()) / 2.0

    def _flux_push(self, stream_id: int, cible, deadman: bool, t: Optional[float]):
        if t is None:
            t = time.monotonic() + self._decalage_horloge
        payload = {"stream_id": stream_id, "deadman": deadman, "t": t}
        if isinstance(cible, dict):
            payload["joints"] = cible
        else:
            payload["velocity"] = list(cible)
        t0 = time.monotonic()
        rep = self._call("stream_push", payload)
        self._recaler_horloge(rep["now"], t0)
        return StreamAck.from_json(rep)

    def _flux_resume(self, stream_id: int) -> ActionResult:
        return ActionResult.from_json(self._call("stream_resume", {"stream_id": stream_id}))

    def _flux_metrics(self, stream_id: int) -> StreamMetrics:
        return StreamMetrics.from_json(self._call("stream_metrics", {"stream_id": stream_id}))

    def _flux_close(self, stream_id: int) -> None:
        self._call("stream_close", {"stream_id": stream_id})

    # ---- Cycle de vie (voir RobotContext) ---------------------------------

    def _motif_arret(self) -> Optional[str]:
        motif = self._call("should_stop", {})["reason"]
        if motif == "estop" and self._evenements is not None:
            self._evenements.notify_estop()
        return motif

    def should_stop(self) -> bool:
        return self._motif_arret() is not None

    def check_cancelled(self) -> None:
        motif = self._motif_arret()
        if motif == "estop":
            raise EstopActiveError("arrêt d'urgence actif")
        if motif is not None:
            raise MissionCancelled(f"arrêt demandé ({motif})")

    def report_progress(self, fraction: float, message: str = "") -> None:
        self._call("report_progress", {
            "fraction": require_number("fraction", fraction, mini=0.0, maxi=1.0),
            "message": require_text("message", message, PROGRESS_MESSAGE_MAX,
                                    allow_empty=True)})

    def _abonner(self, kind: str, callback, threshold: Optional[float] = None) -> bool:
        """Vérifie le droit auprès de l'OS, puis abonne `callback` au thread
        des événements de ce processus (créé au premier abonnement)."""
        if not callable(callback):
            raise InvalidArgumentError("callback doit être une fonction")
        payload = {"kind": kind}
        if threshold is not None:
            payload["threshold_percent"] = require_number("threshold_percent", threshold,
                                                          mini=1.0, maxi=99.0)
        if not self._call("subscribe_event", payload):
            return False
        if self._evenements is None:
            self._evenements = EventPoller(
                lambda i: self._call("poll_events", {} if i is None else {"since_index": i}),
                self.log)
        self._evenements.add(kind, callback, threshold)
        return True

    def on_estop(self, callback) -> None:
        self._abonner("estop", callback)

    def on_alert(self, callback) -> None:
        self._abonner("alert", callback)

    def on_low_battery(self, callback, *, threshold_percent: float = 20.0) -> bool:
        if not callable(callback):
            raise InvalidArgumentError("callback doit être une fonction")
        if not self._abonner("low_battery", callback, threshold_percent):
            self.log("on_low_battery : batterie inconnue sur ce robot — pas d'abonnement")
            return False
        return True

    def ask_confirmation(self, question: str, *, timeout_s: float = 30.0) -> ActionResult:
        q = require_text("question", question, QUESTION_MAX)
        delai = require_number("timeout_s", timeout_s, mini=0.0, maxi=CONFIRMATION_MAX_S,
                               strict_min=True)
        return ActionResult.from_json(self._call("ask_confirmation", {
            "question": q, "timeout_s": delai}, duree_nominale_s=delai))

    def rest_pose(self, *, duration_s: float = 2.0) -> ActionResult:
        duree = require_number("duration_s", duration_s, mini=0.5, maxi=30.0)
        return ActionResult.from_json(self._call("rest_pose", {"duration_s": duree},
                                                 duree_nominale_s=duree))

    def get_mission_log(self) -> list:
        """
        Contrairement aux autres méthodes, le journal de mission est tenu
        localement, dans le processus enfant, au lieu d'être redemandé au
        parent : chaque message envoyé par log() est aussi gardé ici. Le
        but est que HALBridgeClient remplace RobotContext sans changer
        une ligne des apps existantes (PackageSorting, SecurityPatrol et
        WarehouseOps appellent toutes ctx.get_mission_log() en fin de
        mission).
        """
        return list(self._local_log)

    # Un appel qui bloque par nature (écouter 2 s, par exemple) peut
    # durer bien plus longtemps en temps réel (le « temps mural », celui
    # d'une horloge au mur) qu'en temps du robot, quand la simulation
    # tourne au ralenti. Tolérance par défaut : une simulation jusqu'à
    # 60 fois plus lente que le temps réel. Réglable par la variable
    # d'environnement AXIS_OS_BRIDGE_TOLERANCE pour les cas extrêmes.
    TOLERANCE_LENTEUR = float(os.environ.get("AXIS_OS_BRIDGE_TOLERANCE", "60"))

    def _delai(self, duree_nominale_s: float = 0.0) -> float:
        """Délai d'attente maximal d'un appel, en temps mural (réel).

        Les 5 s de base (timeout_s) servent à détecter un noyau d'AXIS-OS
        mort, pour les appels censés répondre aussitôt. Un appel qui
        bloque par conception, comme listen() qui écoute pendant toute sa
        durée, reçoit en plus sa durée annoncée multipliée par la
        tolérance de lenteur.

        Piège constaté sans cela : listen(2.0), dans une simulation
        ralentie par le rendu 1080p, dépassait 5 s réelles ; le pont
        concluait à un noyau mort et l'app plantait. C'était la troisième
        dépendance à l'horloge murale trouvée sur ce chemin, après la
        logique des apps et le quota de temps du sandbox.
        """
        return self._timeout + max(0.0, duree_nominale_s) * self.TOLERANCE_LENTEUR

    def _call(self, action: str, payload: Dict[str, Any],
              duree_nominale_s: float = 0.0) -> Any:
        with self._verrou_appel:
            self._req_id += 1
            req_id = self._req_id
            delai = self._delai(duree_nominale_s)
            resp = self._appel_canal(action, payload, req_id, delai)
        if not resp.success:
            # Un refus structuré redevient la même exception qu'en direct
            # (PermissionError, EstopActiveError, InvalidArgumentError…) :
            # une app écrite pour RobotContext se comporte de la même façon,
            # sandboxée ou non.
            code, sep, message = (resp.error or "").partition(":")
            classe = EXCEPTIONS_PAR_CODE.get(code) if sep else None
            if classe is EstopActiveError and self._evenements is not None:
                self._evenements.notify_estop()
            if classe is not None:
                raise classe(message)
            raise RuntimeError(f"HAL bridge erreur [{action}]: {resp.error}")
        return resp.data

    def _appel_canal(self, action: str, payload: Dict[str, Any],
                     req_id: int, delai: float) -> HALResponse:
        try:
            self._canal.envoyer({"type": "appel", "action": action,
                                 "payload": payload, "request_id": req_id})
        except (TypeError, ValueError) as exc:
            # Seules des données JSON passent le pont : un objet Python
            # (une instance, un ensemble, un tableau numpy) est refusé ici,
            # avant même de partir.
            raise RuntimeError(f"HAL bridge erreur [{action}]: valeur non "
                               f"transmissible en JSON : {exc}") from None
        except CanalFerme as exc:
            raise TimeoutError(f"HAL bridge fermé pendant '{action}' : {exc}") from None
        deadline = time.monotonic() + delai
        while True:
            reste = deadline - time.monotonic()
            if reste <= 0:
                raise TimeoutError(f"HAL bridge timeout sur '{action}' (> {delai:.1f}s)")
            try:
                msg = self._canal.recevoir(timeout=reste)
            except TimeoutError:
                continue
            except CanalFerme as exc:
                raise TimeoutError(f"HAL bridge fermé pendant '{action}' : {exc}") from None
            if msg.get("type") != "reponse" or msg.get("request_id") != req_id:
                continue
            return HALResponse(request_id=req_id, success=msg.get("success") is True,
                               data=msg.get("data"), error=msg.get("error"))

    @staticmethod
    def _serialize(commands: Dict[str, ActuatorCommand]) -> Dict[str, dict]:
        return {
            jid: {"mode": c.mode.name, "position": c.position,
                  "velocity": c.velocity, "torque": c.torque,
                  "current": c.current, "stiffness": c.stiffness,
                  "damping": c.damping, "origine": getattr(c, "origine", None)}
            for jid, c in commands.items()
        }
