"""
AXIS-OS — Énergie : retour à la station de charge, arrêt sûr sur batterie
critique.

Sous `return_percent` (réglable par l'administrateur), hors charge :
  1. plus aucune mission ni démarrage d'app (bloqueur du Play Store) ;
  2. la mission en cours est arrêtée : demande coopérative (should_stop),
     puis arrêt forcé (processus de l'app tué) après `grace_s` ;
  3. une fois aucune app en cours, l'OS conduit le robot à la station de
     charge de la carte du sens, par la navigation : planificateur, zones
     interdites et lentes, gardien de proximité, bridage thermique, tout
     s'applique. Échec (chemin bloqué, délai) : nouvel essai après
     `retry_s`, au plus `max_retries` fois, puis l'opérateur est prévenu ;
  4. à la station, la charge reprend ; les missions restent refusées
     jusqu'à `resume_percent`.
Sous `critical_percent`, hors charge : tout s'arrête sur place, en
équilibre (pas de marche vers la station : mieux vaut un robot arrêté
debout qu'une chute en route), l'app en cours est arrêtée de force, et
l'opérateur est prévenu (URGENT).

Batterie inconnue (le driver ne la mesure pas) : rien n'est inventé, le
service ne fait rien et le dit (state = « unknown »). Sous arrêt
d'urgence : aucune marche.
"""
from __future__ import annotations

import math
import threading
import time
from typing import Callable, Optional

from axis_os.services.navigation.service import EtatNavigation, Pose2D

NORMAL, UNKNOWN, RETURNING, DOCKED, CHARGING = "normal", "unknown", "returning", "docked", "charging"
CRITICAL, NO_STATION, RETURN_FAILED = "critical", "no_station", "return_failed"
BLOCKING_STATES = (RETURNING, DOCKED, CHARGING, CRITICAL, NO_STATION, RETURN_FAILED)


class EnergyService:
    """Voir l'en-tête du module.

    runtime    : PlayStoreRuntime (bloqueur de missions, arrêt des hooks,
                 notifications de l'OS).
    navigation : NavigationService ; locomotion : LocomotionService ;
    site_map   : SiteMap (la station est le lieu de type charging_station).
    battery    : fonction -> BatteryState ou None ; par défaut le HAL du
                 runtime.
    """

    service_id = "energy"
    TICK_MIN_S = 0.2

    def __init__(self, runtime, navigation, locomotion, site_map, *,
                 battery: Optional[Callable] = None, follow=None,
                 clock: Optional[Callable[[], float]] = None,
                 return_percent: float = 20.0, critical_percent: float = 8.0,
                 resume_percent: Optional[float] = None, grace_s: float = 5.0,
                 retry_s: float = 30.0, max_retries: int = 5,
                 return_timeout_s: float = 600.0):
        self._rt = runtime
        self._nav, self._loco, self._site, self._suivi = navigation, locomotion, site_map, follow
        self._batterie = battery or runtime._robot.get_battery
        self._clock = clock or runtime._clock
        self.set_thresholds(return_percent, critical_percent, resume_percent)
        self.grace_s, self.retry_s = float(grace_s), float(retry_s)
        self.max_retries, self.return_timeout_s = int(max_retries), float(return_timeout_s)
        self.state = UNKNOWN
        self.percent: Optional[float] = None
        self._en_route_depuis: Optional[float] = None
        self._prochain_essai = 0.0
        self.retries = 0
        self._dernier_tick: Optional[float] = None
        self._vus: dict = {}                  # app_id -> instant où l'arrêt a été demandé
        self._verrou = threading.RLock()
        self._fin = threading.Event()
        self._fil: Optional[threading.Thread] = None
        runtime.add_mission_blocker(self.blocked_reason)

    # ---- réglages de l'administrateur ----------------------------------------

    def set_thresholds(self, return_percent: float, critical_percent: float,
                       resume_percent: Optional[float] = None) -> None:
        """5 ≤ critique < retour ≤ 90, et reprise ≥ retour (par défaut
        retour + 10)."""
        reprise = return_percent + 10.0 if resume_percent is None else resume_percent
        for v in (return_percent, critical_percent, reprise):
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                raise ValueError(f"seuil de batterie invalide : {v!r}")
        if not (5.0 <= critical_percent < return_percent <= 90.0 and
                return_percent <= reprise <= 100.0):
            raise ValueError("seuils : 5 ≤ critique < retour ≤ 90, retour ≤ reprise ≤ 100")
        self.return_percent, self.critical_percent = float(return_percent), float(critical_percent)
        self.resume_percent = float(reprise)

    # ---- lecture ---------------------------------------------------------------

    def blocked_reason(self) -> Optional[str]:
        return {RETURNING: "batterie faible : retour à la station de charge",
                DOCKED: "à la station de charge, en attente de charge",
                CHARGING: "en charge",
                CRITICAL: "batterie critique : robot arrêté",
                NO_STATION: "batterie faible et aucune station de charge sur la carte",
                RETURN_FAILED: "batterie faible : retour à la station impossible",
                }.get(self.state)

    # ---- boucle ---------------------------------------------------------------

    def _notifier(self, titre: str, message: str, priorite: str) -> None:
        centre = self._rt._notifications
        if centre is not None:
            centre.from_os(titre, message, priorite)

    def _passer(self, etat: str) -> None:
        self.state = etat

    def _arreter_suivi(self, raison: str) -> None:
        suivi = self._suivi
        if suivi is not None and suivi.active:
            suivi.arreter(suivi.owner(), "CANCELLED", "CANCELLED", raison)

    def _echec(self, t: float) -> None:
        """Un essai de retour a échoué : nouvel essai plus tard, ou abandon."""
        self._en_route_depuis = None
        self._prochain_essai = t + self.retry_s
        self.retries += 1
        if self.retries > self.max_retries:
            self._passer(RETURN_FAILED)
            self._notifier("Retour à la station impossible",
                           f"{self.max_retries} essais : {self._nav.raison or 'délai dépassé'}",
                           "urgent")

    def tick(self) -> str:
        with self._verrou:
            t = self._clock()
            if self._dernier_tick is not None and t - self._dernier_tick < self.TICK_MIN_S:
                return self.state
            self._dernier_tick = t
            self._evaluer(t)
            if self.state in BLOCKING_STATES:
                self._balayer(t)
            else:
                self._vus.clear()
            if self.state == CRITICAL:
                # Un dernier walk() d'une app, entre l'arrêt et la mort de son
                # processus, ne doit pas faire repartir le robot.
                self._loco.halt()
            return self.state

    def _balayer(self, t: float) -> None:
        """Tant que les missions sont bloquées, tout hook d'app en cours (même
        démarré après coup : un on_stop lancé par l'humain…) est prié de
        s'arrêter, puis arrêté de force après grace_s (tout de suite sous
        batterie critique) : aucune app ne garde la main sur le robot."""
        grace = 0.0 if self.state == CRITICAL else self.grace_s
        actives = set(self._rt.running_apps())
        for app_id in set(self._vus) - actives:
            del self._vus[app_id]
        for app_id in actives:
            if app_id not in self._vus:
                self._vus[app_id] = t
                self._rt.request_stop(app_id, source="os")
            if t - self._vus[app_id] >= grace:
                self._rt.force_stop(app_id, self.blocked_reason() or "énergie")

    def _evaluer(self, t: float) -> None:
        """Lit la batterie et fait avancer l'état."""
        try:
            b = self._batterie()
        except Exception:                    # noqa: BLE001 — capteur en panne : inconnue
            b = None
        pct = None if b is None else float(b.percent)
        if pct is not None and not math.isfinite(pct):
            pct = None
        self.percent = pct
        en_charge = b is not None and b.charging is True
        if pct is None:
            if self.state == NORMAL:
                self._passer(UNKNOWN)
            return
        if en_charge:
            if self.state in (RETURNING, DOCKED, CRITICAL, NO_STATION, RETURN_FAILED):
                self._nav.annuler()
                self._passer(CHARGING)
            if self.state in (CHARGING, UNKNOWN) and pct >= self.resume_percent:
                self._passer(NORMAL)
            elif self.state == UNKNOWN:
                self._passer(CHARGING)
            return
        if pct < self.critical_percent and self.state != CRITICAL:
            self._arret_critique(pct)
            return
        if self.state in (NORMAL, UNKNOWN, CHARGING) and pct < self.return_percent:
            self._partir(pct, t)
        elif self.state in (UNKNOWN, CHARGING):
            self._passer(NORMAL)       # assez de charge, hors station
        if self.state in (RETURNING, NO_STATION):
            self._conduire(t)

    def _arret_critique(self, pct: float) -> None:
        self._passer(CRITICAL)
        self._arreter_suivi("batterie critique")
        self._nav.annuler()
        self._loco.halt()
        self._notifier("Batterie critique", f"{pct:.0f} % : robot arrêté en équilibre sur place, "
                                            f"à recharger", "urgent")

    def _partir(self, pct: float, t: float) -> None:
        self._passer(RETURNING)
        self.retries, self._en_route_depuis, self._prochain_essai = 0, None, t
        self._notifier("Batterie faible", f"{pct:.0f} % : mission arrêtée, retour à la station "
                                          f"de charge", "warning")

    def _conduire(self, t: float) -> None:
        """Conduit le robot à la station, une fois aucune app en cours."""
        if self._rt.running_apps() or self._rt._estop_actif():
            return                            # attendre la fin (forcée) des apps ; e-stop : rien
        station = self._site.current.station()
        if station is None:
            if self.state != NO_STATION:
                self._passer(NO_STATION)
                self._notifier("Batterie faible", "aucune station de charge sur la carte du site",
                               "urgent")
            return
        if self.state == NO_STATION:
            self._passer(RETURNING)
        if self._en_route_depuis is not None:
            etat = self._nav.etat
            if etat is EtatNavigation.ARRIVE:
                self._passer(DOCKED)
                return
            if etat is EtatNavigation.EN_ROUTE and t - self._en_route_depuis < self.return_timeout_s:
                return
            self._nav.annuler()               # échec, délai, ou navigation reprise par un autre
            self._echec(t)
            return
        if t < self._prochain_essai:
            return
        self._arreter_suivi("batterie faible : retour à la station")
        r = self._nav.aller_vers(Pose2D(station.x, station.y, station.heading))
        if r.ok:
            self._en_route_depuis = t
        else:
            self._echec(t)

    def start(self, period_s: float = 0.2) -> "EnergyService":
        def boucle():
            while not self._fin.wait(period_s):
                try:
                    self.tick()
                except Exception:            # noqa: BLE001 — le service ne meurt pas
                    pass
        self._fin.clear()
        self._fil = threading.Thread(target=boucle, name="energie", daemon=True)
        self._fil.start()
        return self

    def stop(self) -> None:
        self._fin.set()
        if self._fil is not None:
            self._fil.join(1.0)
