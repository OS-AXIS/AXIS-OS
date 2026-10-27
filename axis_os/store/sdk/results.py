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
