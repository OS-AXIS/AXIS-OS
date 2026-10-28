"""
AXIS-OS — Santé matérielle : températures des moteurs, statut thermique du
robot, batterie détaillée, processeur (et carte graphique si présente).

Statut thermique, par articulation puis pour le robot (le pire) ; seuils de
JointSafetyGuard (seuils_thermiques : critique = le plus bas de 80 °C et de
temperature_max, bridage 15 °C plus bas) :
  normal     — sous « chaud » (critique − HOT_MARGIN_C) ;
  hot        — chaud, rien n'est réduit ;
  throttled  — bridé : la marche est ralentie (facteur_marche), et le
               garde-fou refuse vitesses et couples au-delà de la part permise ;
  critical   — au seuil critique, ou température mesurée non finie, ou
               mesure plus vieille que STALE_S : plus de marche
               (facteur_marche = 0), le garde-fou refuse toute commande à
               cette articulation, et l'OS arrête la mission en cours et
               refuse les suivantes (voir PlayStoreRuntime) ;
  unknown    — le driver ne mesure pas la température (rien n'est inventé).

tick() lit le HAL normalisé (à appeler par la boucle du robot) ; tout le
reste lit son dernier résultat. Un service qui n'a pas été tické depuis
STALE_S rend « critical » : un statut figé n'est pas une mesure.
"""
from __future__ import annotations

import math
import os
import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional

from axis_os.hal.safety.limits import facteur_thermique, seuils_thermiques

NORMAL, HOT, THROTTLED, CRITICAL, UNKNOWN = "normal", "hot", "throttled", "critical", "unknown"
THERMAL_STATES = (NORMAL, HOT, THROTTLED, CRITICAL, UNKNOWN)
_GRAVITE = {UNKNOWN: -1, NORMAL: 0, HOT: 1, THROTTLED: 2, CRITICAL: 3}


@dataclass(frozen=True)
class MotorReading:
    """Santé d'une articulation, telle que le driver la mesure (None :
    inconnue)."""
    joint: str
    temperature_c: Optional[float]
    critical_c: float
    margin_c: Optional[float]
    torque_nm: Optional[float]
    current_a: Optional[float]
    status: str


class ComputeProbe:
    """Température et charge du processeur, et de la carte graphique si
    elle existe ; None pour ce qui ne se mesure pas ici."""

    def __init__(self):
        try:
            import psutil
            psutil.cpu_percent(interval=None)    # la première lecture vaut toujours 0
        except Exception:                        # noqa: BLE001
            pass

    def read(self) -> dict:
        cpu_t = cpu_charge = None
        try:
            import psutil
            capteurs = getattr(psutil, "sensors_temperatures", lambda: {})() or {}
            for nom in ("coretemp", "k10temp", "cpu_thermal", "soc_thermal", "acpitz"):
                if capteurs.get(nom):
                    cpu_t = float(capteurs[nom][0].current)
                    break
            cpu_charge = float(psutil.cpu_percent(interval=None))
        except Exception:                    # noqa: BLE001 — psutil absent ou muet
            try:
                cpu_charge = 100.0 * os.getloadavg()[0] / (os.cpu_count() or 1)
            except (OSError, AttributeError):
                cpu_charge = None
        gpu = {"gpu_present": False, "gpu_temperature_c": None, "gpu_load_percent": None}
        try:
            import pynvml
            pynvml.nvmlInit()
            h = pynvml.nvmlDeviceGetHandleByIndex(0)
            gpu = {"gpu_present": True,
                   "gpu_temperature_c": float(pynvml.nvmlDeviceGetTemperature(h, 0)),
                   "gpu_load_percent": float(pynvml.nvmlDeviceGetUtilizationRates(h).gpu)}
        except Exception:                    # noqa: BLE001 — pas de carte NVIDIA lisible
            pass
        return {"cpu_temperature_c": cpu_t, "cpu_load_percent": cpu_charge, **gpu}


class HardwareHealthService:
    """Voir l'en-tête du module.

    hal   : NormalizedRobotHAL (get_full_state, get_joint_definitions,
            get_battery, measures_motor_temperature).
    clock : horloge monotone du robot (âge du dernier tick).
    """

    HOT_MARGIN_C = 25.0
    HYSTERESIS_C = 2.0
    STALE_S = 1.0
    TICK_MIN_S = 0.2
    service_id = "hardware_health"

    def __init__(self, hal, *, clock: Callable[[], float] = time.monotonic,
                 compute_probe: Optional[ComputeProbe] = None):
        self._hal = hal
        self._clock = clock
        self._sonde = compute_probe or ComputeProbe()
        self._verrou = threading.Lock()
        self._limites = {j.joint_id: j.limits for j in hal.get_joint_definitions()}
        self._moteurs: Dict[str, MotorReading] = {}
        self._statut = UNKNOWN
        self._facteur = 1.0
        self._dernier_tick: Optional[float] = None
        self._ecouteurs: List[Callable[[str, str], None]] = []

    def add_listener(self, fn: Callable[[str, str], None]) -> None:
        """fn(ancien, nouveau) à chaque changement du statut thermique
        (côté OS : arrêt de mission, notification)."""
        self._ecouteurs.append(fn)

    # ---- mesure ------------------------------------------------------------

    def _statut_de(self, temp: Optional[float], limits, avant: str = UNKNOWN) -> str:
        """Statut d'une articulation ; pour redescendre d'un statut, il faut
        passer HYSTERESIS_C sous son seuil (un capteur qui oscille au seuil
        ne fait pas entrer et sortir le robot de « critical » sans cesse)."""
        debut, critique = seuils_thermiques(limits)
        if temp is None:
            return UNKNOWN
        g, h = _GRAVITE[avant], self.HYSTERESIS_C
        if not math.isfinite(temp) or temp >= critique - (h if g >= 3 else 0.0):
            return CRITICAL
        if temp > debut - (h if g >= 2 else 0.0):
            return THROTTLED
        if temp >= critique - self.HOT_MARGIN_C - (h if g >= 1 else 0.0):
            return HOT
        return NORMAL

    def tick(self) -> str:
        """Relit les moteurs ; rend le statut thermique du robot."""
        t = self._clock()
        if self._dernier_tick is not None and t - self._dernier_tick < self.TICK_MIN_S:
            return self._statut
        mesure = bool(self._hal.measures_motor_temperature())
        try:
            etat = self._hal.get_full_state()
        except Exception:                    # noqa: BLE001 — HAL muet : danger
            etat = None
        moteurs, facteur, pire = {}, 1.0, UNKNOWN
        maintenant_ns = time.time_ns()
        for jid, limits in self._limites.items():
            fb = etat.get(jid) if etat is not None else None
            critique = seuils_thermiques(limits)[1]
            if not mesure:
                temp = None
            elif fb is None or (maintenant_ns - fb.timestamp_ns) / 1e9 > self.STALE_S:
                temp = math.nan              # mesure absente ou périmée : danger
            else:
                temp = float(fb.temperature)
            avant = self._moteurs[jid].status if jid in self._moteurs else UNKNOWN
            statut = self._statut_de(temp, limits, avant)
            couple = float(fb.torque) if fb is not None and math.isfinite(fb.torque) else None
            courant = (float(fb.current) if fb is not None and limits.current_is_measured
                       and math.isfinite(fb.current) else None)
            moteurs[jid] = MotorReading(
                jid, temp if temp is None or math.isfinite(temp) else None, critique,
                None if temp is None or not math.isfinite(temp) else round(critique - temp, 2),
                couple, courant, statut)
            if temp is not None:
                facteur = min(facteur, 0.0 if statut == CRITICAL
                              else facteur_thermique(temp, limits))
            if _GRAVITE[statut] > _GRAVITE[pire]:
                pire = statut
        with self._verrou:
            ancien = self._statut
            self._moteurs, self._statut, self._facteur = moteurs, pire, facteur
            self._dernier_tick = t
        if pire != ancien:
            for fn in list(self._ecouteurs):
                try:
                    fn(ancien, pire)
                except Exception:            # noqa: BLE001 — un écouteur ne casse pas la mesure
                    pass
        return pire

    def _perime(self) -> bool:
        return self._dernier_tick is None or self._clock() - self._dernier_tick > self.STALE_S

    # ---- lecture -------------------------------------------------------------

    def thermal_status(self) -> str:
        if self._hal.measures_motor_temperature() and self._perime():
            return CRITICAL
        return self._statut

    def facteur_marche(self) -> float:
        """Part de la vitesse de marche permise (LocomotionService)."""
        if not self._hal.measures_motor_temperature():
            return 1.0
        if self._perime():
            return 0.0
        return self._facteur

    def motors(self) -> List[MotorReading]:
        with self._verrou:
            return [self._moteurs[j] for j in sorted(self._moteurs)]

    def battery(self):
        """BatteryState mesuré, ou None (inconnue)."""
        try:
            return self._hal.get_battery()
        except Exception:                    # noqa: BLE001 — capteur en panne : inconnue
            return None

    def compute(self) -> dict:
        try:
            return self._sonde.read()
        except Exception:                    # noqa: BLE001
            return {"cpu_temperature_c": None, "cpu_load_percent": None, "gpu_present": False,
                    "gpu_temperature_c": None, "gpu_load_percent": None}
