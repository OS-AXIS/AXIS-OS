"""
AXIS-OS — SDK des apps : refus structurés et résultats
=======================================================
Quand l'OS refuse un appel d'une app, l'app reçoit un CODE stable (une
valeur de RefusalCode, la même d'une version à l'autre) et un message
lisible. Deux façons de le recevoir :

  - une exception, pour ce qui est une erreur de l'app ou un refus de
    sécurité : AxisRefusal et ses sous-classes (PermissionError et
    EstopActiveError d'app_base.py en héritent, InvalidArgumentError,
    MissionCancelled), chacune avec `.code` et `.message` ;
  - un résultat, pour une issue normale d'une action qui peut échouer sans
    que ce soit une faute (un chemin bloqué, un délai dépassé) :
    ActionResult, avec `ok`, `status`, `code` et `message`.

La liste des codes est documentée dans docs/API_REFERENCE.md ; un code
n'est jamais renommé ni réutilisé pour autre chose.

Les mêmes objets traversent le pont JSON (axis_os/store/runtime/
hal_bridge.py) : une app sandboxée reçoit le même type et le même code
qu'une app lancée en direct.

Licence : Apache 2.0 (open-source — SDK public)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, Optional


class RefusalCode(str, Enum):
    """Raison stable d'un refus ou d'un échec. La valeur est le nom : c'est
    elle qui traverse le pont et qu'une app peut comparer."""
    PERMISSION_DENIED = "PERMISSION_DENIED"     # permission absente du manifeste
    ESTOP_ACTIVE = "ESTOP_ACTIVE"               # arrêt d'urgence actif
    SAFETY_LIMIT = "SAFETY_LIMIT"               # garde-fou d'articulation, limite de sécurité
    PROXIMITY_GUARD = "PROXIMITY_GUARD"         # gardien de proximité : personne ou obstacle
    CAPABILITY_MISSING = "CAPABILITY_MISSING"   # matériel ou service absent du robot
    QUOTA_EXCEEDED = "QUOTA_EXCEEDED"           # quota de l'app atteint
    INVALID_ARGUMENT = "INVALID_ARGUMENT"       # NaN, hors bornes, mauvais type
    TIMEOUT = "TIMEOUT"                         # délai dépassé
    CANCELLED = "CANCELLED"                     # arrêt demandé (humain, OS, e-stop)
    BUSY = "BUSY"                               # le service fait déjà autre chose
    BLOCKED = "BLOCKED"                         # pas de chemin, obstacle infranchissable
    OUT_OF_REACH = "OUT_OF_REACH"               # objet hors des mains, plateau hors d'atteinte
    FAILED = "FAILED"                           # l'action a été tentée et a échoué
    HUMAN_DECLINED = "HUMAN_DECLINED"           # l'humain a répondu non
    HUMAN_UNAVAILABLE = "HUMAN_UNAVAILABLE"     # aucun humain pour répondre
    STALE_TARGET = "STALE_TARGET"               # consigne de flux trop ancienne
    RATE_LIMITED = "RATE_LIMITED"               # consignes de flux trop rapprochées
    OUT_OF_BOUNDS = "OUT_OF_BOUNDS"             # consigne de flux hors des bornes
    STREAM_STOPPED = "STREAM_STOPPED"           # flux arrêté : reprise explicite exigée


class AxisRefusal(Exception):
    """Refus de l'OS, levé comme exception. `code` est une RefusalCode,
    `message` l'explication lisible. Chaque sous-classe fixe son code."""
    code: RefusalCode = RefusalCode.FAILED

    def __init__(self, message: str = "", code: Optional[RefusalCode] = None):
        super().__init__(message)
        self.message = str(message)
        if code is not None:
            self.code = RefusalCode(code)


class InvalidArgumentError(AxisRefusal, ValueError):
    """Argument refusé avant tout effet : valeur non finie (NaN, infinie),
    hors de ses bornes, ou d'un mauvais type. Hérite aussi de ValueError,
    pour qu'un code qui attrapait ValueError continue de le faire."""
    code = RefusalCode.INVALID_ARGUMENT


# Préfixe d'erreur du pont -> classe d'exception reconstruite côté app.
# PERMISSION_DENIED et ESTOP_ACTIVE sont enregistrés par app_base.py, qui
# définit leurs classes.
EXCEPTIONS_PAR_CODE: Dict[str, type] = {
    RefusalCode.INVALID_ARGUMENT.value: InvalidArgumentError,
}


@dataclass
class ActionResult:
    """Issue d'une action qui peut échouer normalement (navigation, prise,
    parole…). `ok` dit si elle a abouti ; `status` nomme l'issue
    (« ARRIVED », « BLOCKED »…, propre à chaque API) ; `code` est la
    RefusalCode d'un échec (None en cas de succès) ; `data` porte des
    valeurs propres à l'API. Vrai en booléen si et seulement si ok."""
    ok: bool
    status: str
    code: Optional[str] = None
    message: str = ""
    data: Dict[str, Any] = field(default_factory=dict)

    def __bool__(self) -> bool:
        return self.ok is True

    def to_json(self) -> Dict[str, Any]:
        return {"ok": self.ok, "status": self.status, "code": self.code,
                "message": self.message, "data": self.data}

    @classmethod
    def from_json(cls, d: Dict[str, Any]) -> "ActionResult":
        return cls(ok=d.get("ok") is True, status=str(d.get("status", "")),
                   code=d.get("code"), message=str(d.get("message", "")),
                   data=dict(d.get("data") or {}))


def refused(status: str, code: RefusalCode, message: str, **data) -> ActionResult:
    """ActionResult d'échec, avec son code."""
    return ActionResult(ok=False, status=status, code=RefusalCode(code).value,
                        message=message, data=data)


# ─── Validation des arguments, la même des deux côtés du pont ─────────────

def require_number(name: str, value: Any, *, mini: Optional[float] = None,
                   maxi: Optional[float] = None, strict_min: bool = False) -> float:
    """Rend `value` en float, ou lève InvalidArgumentError : pas un nombre
    (un booléen n'en est pas un), non fini, ou hors de [mini, maxi]
    (]mini, maxi] avec strict_min)."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise InvalidArgumentError(f"{name} doit être un nombre (reçu : "
                                   f"{type(value).__name__})")
    v = float(value)
    if not math.isfinite(v):
        raise InvalidArgumentError(f"{name} doit être un nombre fini (reçu : {v})")
    if mini is not None and (v <= mini if strict_min else v < mini):
        raise InvalidArgumentError(f"{name} = {v} : en dessous de la borne {mini}")
    if maxi is not None and v > maxi:
        raise InvalidArgumentError(f"{name} = {v} : au-dessus de la borne {maxi}")
    return v


def require_text(name: str, value: Any, max_len: int, *, allow_empty: bool = False) -> str:
    """Rend `value`, un texte, borné à `max_len` caractères et dont chaque
    caractère de contrôle (saut de ligne, séquence d'échappement du
    terminal) est remplacé par une espace ; lève InvalidArgumentError si ce
    n'est pas un texte ou s'il est vide sans `allow_empty`."""
    if not isinstance(value, str):
        raise InvalidArgumentError(f"{name} doit être un texte (reçu : "
                                   f"{type(value).__name__})")
    texte = "".join(c if c.isprintable() else " " for c in value[:max_len])
    if not allow_empty and not texte.strip():
        raise InvalidArgumentError(f"{name} ne doit pas être vide")
    return texte


# ─── État du robot et de sa sécurité (robot_info, safety_status) ─────────

# État d'une capacité matérielle, tel que l'app le lit.
AVAILABLE, DEGRADED, ABSENT = "available", "degraded", "absent"


@dataclass
class BatteryInfo:
    """Batterie, telle que le driver la mesure. `known` faux : le robot ne
    sait pas la lire, et `percent` est None (jamais une valeur inventée)."""
    known: bool
    percent: Optional[float] = None
    charging: Optional[bool] = None
    voltage: Optional[float] = None

    @classmethod
    def from_json(cls, d: Dict[str, Any]) -> "BatteryInfo":
        return cls(known=d.get("known") is True, percent=d.get("percent"),
                   charging=d.get("charging"), voltage=d.get("voltage"))


@dataclass
class RobotInfo:
    """Ce que l'app peut savoir du robot qui l'exécute (ctx.robot_info()).

    `capabilities` : chaque capacité matérielle (« legs », « camera »…)
    avec son état, AVAILABLE, DEGRADED ou ABSENT. `services` : chaque
    service système (« locomotion », « perception »…) avec AVAILABLE ou
    ABSENT. `limits` : limites utiles (vitesse de marche maximale en m/s,
    rotation maximale en rad/s, rapport de vitesse articulaire du
    manifeste…), None quand le robot n'en a pas l'usage."""
    model: str
    manufacturer: str
    os_version: str
    sdk_api_version: str
    profile: str
    capabilities: Dict[str, str]
    services: Dict[str, str]
    limits: Dict[str, Optional[float]]
    battery: BatteryInfo

    def to_json(self) -> Dict[str, Any]:
        return {"model": self.model, "manufacturer": self.manufacturer,
                "os_version": self.os_version, "sdk_api_version": self.sdk_api_version,
                "profile": self.profile, "capabilities": dict(self.capabilities),
                "services": dict(self.services), "limits": dict(self.limits),
                "battery": {"known": self.battery.known, "percent": self.battery.percent,
                            "charging": self.battery.charging,
                            "voltage": self.battery.voltage}}

    @classmethod
    def from_json(cls, d: Dict[str, Any]) -> "RobotInfo":
        return cls(model=str(d.get("model", "")), manufacturer=str(d.get("manufacturer", "")),
                   os_version=str(d.get("os_version", "")),
                   sdk_api_version=str(d.get("sdk_api_version", "")),
                   profile=str(d.get("profile", "")),
                   capabilities=dict(d.get("capabilities") or {}),
                   services=dict(d.get("services") or {}),
                   limits=dict(d.get("limits") or {}),
                   battery=BatteryInfo.from_json(d.get("battery") or {}))


@dataclass
class SafetyStatus:
    """État de sécurité au moment de l'appel (ctx.safety_status()).

    `proximity` : « clear » (rien ne gêne), « slowing » (le gardien de
    proximité ralentit le robot), « stopped » (il le tient à l'arrêt) ou
    « unavailable » (pas de gardien sur ce robot, ou pas encore de mesure).
    `fall_detected` : None si le robot ne marche pas. Les quotas restants
    sont None quand ils ne s'appliquent pas (pas de mémoire persistante,
    hors d'une mission)."""
    estop_active: bool
    proximity: str
    proximity_distance_m: Optional[float]
    proximity_reason: str
    fall_detected: Optional[bool]
    time_remaining_s: Optional[float]
    storage_keys_remaining: Optional[int]
    storage_bytes_remaining: Optional[int]

    def to_json(self) -> Dict[str, Any]:
        return {"estop_active": self.estop_active, "proximity": self.proximity,
                "proximity_distance_m": self.proximity_distance_m,
                "proximity_reason": self.proximity_reason,
                "fall_detected": self.fall_detected,
                "time_remaining_s": self.time_remaining_s,
                "storage_keys_remaining": self.storage_keys_remaining,
                "storage_bytes_remaining": self.storage_bytes_remaining}

    @classmethod
    def from_json(cls, d: Dict[str, Any]) -> "SafetyStatus":
        return cls(estop_active=d.get("estop_active") is True,
                   proximity=str(d.get("proximity", "unavailable")),
                   proximity_distance_m=d.get("proximity_distance_m"),
                   proximity_reason=str(d.get("proximity_reason", "")),
                   fall_detected=d.get("fall_detected"),
                   time_remaining_s=d.get("time_remaining_s"),
                   storage_keys_remaining=d.get("storage_keys_remaining"),
                   storage_bytes_remaining=d.get("storage_bytes_remaining"))


# ─── Objets à saisir (pick, place) ───────────────────────────────────────

@dataclass(frozen=True)
class ObjectSpec:
    """Un objet décrit par l'app, dans le repère monde (celui de la
    carte) : son centre (x, y, z), en mètres, et ses dimensions (largeur,
    prise entre les mains ; profondeur, vers l'avant du robot ; hauteur),
    en mètres. Pour un objet vu par look_for(..., frame="world"), passer
    plutôt la Detection elle-même à pick(), avec sa taille."""
    center: tuple
    size: tuple


# Bornes d'une description d'objet : un centre dans un site de 1 km de
# côté, entre 1 m sous le sol et 3 m de haut ; une taille entre 2 cm et
# 70 cm, l'exemption maximale du gardien de proximité autour d'un objet
# manipulé (ChampsProtection.exemption_cote_max).
CENTRE_MAX_M, Z_MIN_M, Z_MAX_M = 1000.0, -1.0, 3.0
TAILLE_MIN_M, TAILLE_MAX_M = 0.02, 0.70


def object_spec(center: Any, size: Any) -> ObjectSpec:
    """ObjectSpec validé, ou InvalidArgumentError."""
    if not isinstance(center, (list, tuple)) or len(center) != 3:
        raise InvalidArgumentError("center doit être un triplet (x, y, z) en mètres")
    if not isinstance(size, (list, tuple)) or len(size) != 3:
        raise InvalidArgumentError("size doit être un triplet (largeur, profondeur, "
                                   "hauteur) en mètres")
    x = require_number("center.x", center[0], mini=-CENTRE_MAX_M, maxi=CENTRE_MAX_M)
    y = require_number("center.y", center[1], mini=-CENTRE_MAX_M, maxi=CENTRE_MAX_M)
    z = require_number("center.z", center[2], mini=Z_MIN_M, maxi=Z_MAX_M)
    taille = tuple(require_number(f"size[{i}]", v, mini=TAILLE_MIN_M, maxi=TAILLE_MAX_M)
                   for i, v in enumerate(size))
    return ObjectSpec((x, y, z), taille)


def object_from_target(target: Any, size: Any = None) -> ObjectSpec:
    """L'objet à saisir, depuis une ObjectSpec ou une Detection vue dans le
    repère monde (avec sa position 3D), dont la taille est alors exigée :
    une image ne donne pas les dimensions d'un objet, et l'OS ne les
    invente pas."""
    if isinstance(target, ObjectSpec):
        return object_spec(target.center, target.size)
    p = getattr(target, "position_3d", None)
    if p is None or not hasattr(target, "frame"):
        raise InvalidArgumentError("cible : une ObjectSpec, ou une Detection avec sa "
                                   "position 3D (caméra de profondeur)")
    if target.frame != "world":
        raise InvalidArgumentError("détection dans le repère du robot : la chercher avec "
                                   "look_for(..., frame=\"world\")")
    if size is None:
        raise InvalidArgumentError("size exigé pour une Detection : (largeur, profondeur, "
                                   "hauteur) en mètres")
    return object_spec((p.x, p.y, p.z), size)


def surface_height_from_target(target: Any) -> Optional[float]:
    """Hauteur du plateau où poser (m, repère monde) : None (pas de
    plateau), un nombre, une ObjectSpec (sa face supérieure) ou une
    Detection vue dans le repère monde (la hauteur du point vu)."""
    if target is None:
        return None
    if isinstance(target, ObjectSpec):
        spec = object_from_target(target)
        target = spec.center[2] + spec.size[2] / 2.0
    elif not isinstance(target, (int, float)) or isinstance(target, bool):
        p = getattr(target, "position_3d", None)
        if p is None or getattr(target, "frame", None) != "world":
            raise InvalidArgumentError("cible : None, une hauteur (m), une ObjectSpec ou "
                                       "une Detection vue dans le repère « world »")
        target = p.z
    return require_number("hauteur du plateau", target, mini=0.0, maxi=2.5)
