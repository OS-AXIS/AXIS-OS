"""
AXIS-OS — Tâches planifiées : l'OS lance la mission d'une app quand un de ses
déclencheurs tombe (AppManifest.triggers : chaque jour à HH:MM, à intervalle,
batterie sous un seuil, entrée dans une zone).

La mission passe par PlayStoreRuntime.run_mission, comme si un humain la
lançait : bac à sable, quotas, permissions, arrêt d'urgence, verrou de
mission. L'ordonnanceur n'a aucun droit de plus.

Règles :
  1. Seules les apps installées ET démarrées, avec SCHEDULED_MISSIONS dans
     leur manifeste, ont des déclencheurs actifs. Une app ne peut ni en
     ajouter ni en changer à l'exécution : ils viennent du manifeste, montré
     à l'installation.
  2. Une seule mission à la fois. Un déclencheur qui tombe pendant une
     mission (quelle qu'elle soit) est sauté (« busy ») : il n'attend pas
     dans une file, il retombera à sa prochaine occurrence.
  3. Deux déclencheurs dus au même pas : un seul est lancé, dans cet ordre :
     batterie, zone, heure fixe, intervalle ; à égalité, l'app_id puis le nom
     du déclencheur (ordre alphabétique). Les autres sont sautés
     (« collision »).
  4. Pas de rattrapage : une occurrence en retard de plus de GRACE_S
     (robot éteint, OS occupé) est notée « missed » et n'est jamais lancée
     plus tard ; un intervalle repart de maintenant.
  5. Rien n'est lancé pendant un arrêt d'urgence, après une chute, hors des
     heures permises par l'administrateur (set_allowed_hours), quand un
     bloqueur de mission le refuse (retour à la station de charge…), ni
     pour un déclencheur ou une app que l'humain a désactivés
     (set_enabled, set_paused).
  6. Un même déclencheur ne tombe jamais deux fois en moins de MIN_REARM_S
     (une zone franchie dix fois de suite ne lance pas dix missions) ; un
     déclencheur de batterie se réarme quand la charge remonte de
     BATTERY_HYSTERESIS au-dessus de son seuil.

Chaque décision est notée dans `journal`, lisible par l'administrateur.
"""
from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timedelta
from typing import Callable, Deque, Dict, List, Optional, Tuple

from axis_os.store.sdk.manifest import Permission, TRIGGER_INTERVAL_MIN_S

ORDRE_TYPES = {"battery_below": 0, "zone_enter": 1, "daily": 2, "interval": 3}


@dataclass(frozen=True)
class SchedulerEvent:
    """Une décision de l'ordonnanceur. `decision` : launched, finished,
    busy, collision, missed, estop, fallen, hours, blocked, disabled,
    rearm."""
    t: float
    app_id: str
    trigger: str
    decision: str
    detail: str = ""


class _Etat:
    """État d'un déclencheur d'une app."""

    def __init__(self, spec, maintenant: datetime, t: float):
        self.spec = spec
        self.dernier_lancement: Optional[float] = None
        self.prochain = t + spec.every_s if spec.kind == "interval" else None
        self.jour_traite = None
        if spec.kind == "daily":
            h, m = (int(v) for v in spec.at.split(":"))
            cible = maintenant.replace(hour=h, minute=m, second=0, microsecond=0)
            # Installée après l'heure du jour : la première occurrence est demain.
            if maintenant > cible + timedelta(seconds=MissionScheduler.GRACE_S):
                self.jour_traite = maintenant.date()
        self.arme = True                # batterie
        self.dedans: Optional[bool] = None  # zone (None : pas encore vu)


class MissionScheduler:
    """Voir l'en-tête du module.

    runtime    : le PlayStoreRuntime dont il lance les missions.
    clock      : horloge monotone du robot (intervalles, réarmement).
    wall_clock : heure locale du robot (déclencheurs « daily »).
    battery    : fonction -> charge en % mesurée, ou None (inconnue) ;
                 par défaut, la batterie du HAL du runtime.
    pose       : fonction -> (x, y) du robot, ou None ; par défaut, la
                 localisation du service de navigation du runtime.
    fallen     : fonction -> vrai après une chute ; par défaut, l'état du
                 service de locomotion du runtime.
    """

    GRACE_S = 120.0
    MIN_REARM_S = TRIGGER_INTERVAL_MIN_S
    BATTERY_HYSTERESIS = 5.0
    TICK_S = 1.0
    JOURNAL_MAX = 500

    def __init__(self, runtime, *, clock: Optional[Callable[[], float]] = None,
                 wall_clock: Callable[[], datetime] = datetime.now,
                 battery: Optional[Callable[[], Optional[float]]] = None,
                 pose: Optional[Callable[[], Optional[Tuple[float, float]]]] = None,
                 fallen: Optional[Callable[[], bool]] = None):
        self._rt = runtime
        self._clock = clock or runtime._clock
        self._mur = wall_clock
        self._batterie = battery or self._batterie_hal
        self._pose = pose or self._pose_navigation
        self._chute = fallen or self._chute_locomotion
        self._etats: Dict[Tuple[str, str], _Etat] = {}
        self._desactives: set = set()
        self._pause = False
        self._heures: Optional[Tuple[int, int]] = None
        self._verrou = threading.Lock()
        self._travail: Optional[threading.Thread] = None
        self._fil: Optional[threading.Thread] = None
        self._fin = threading.Event()
        self.journal: Deque[SchedulerEvent] = deque(maxlen=self.JOURNAL_MAX)

    # ---- sources par défaut ------------------------------------------------

    def _batterie_hal(self) -> Optional[float]:
        lire = getattr(self._rt._robot, "get_battery", None)
        b = lire() if lire is not None else None
        return None if b is None else float(b.percent)

    def _pose_navigation(self):
        nav = self._rt._navigation_service
        p = nav.pose() if nav is not None else None
        return None if p is None else (p.x, p.y)

    def _chute_locomotion(self) -> bool:
        loco = self._rt._locomotion_service
        return getattr(getattr(loco, "state", None), "name", "") == "FALLEN"

    # ---- réglages de l'humain et de l'administrateur ------------------------

    def set_enabled(self, app_id: str, enabled: bool, trigger: Optional[str] = None) -> None:
        """Active ou désactive les déclencheurs d'une app (tous, ou un seul)."""
        cle = (app_id, trigger)
        if enabled:
            self._desactives.discard(cle)
        else:
            self._desactives.add(cle)

    def set_paused(self, paused: bool) -> None:
        """Suspend (ou reprend) toutes les tâches planifiées du robot."""
        self._pause = bool(paused)

    def set_allowed_hours(self, start: Optional[str], end: Optional[str]) -> None:
        """Heures où une mission planifiée peut démarrer (« HH:MM »,
        heure locale ; la plage peut passer minuit : 20:00 à 06:00).
        None, None : à toute heure."""
        if start is None and end is None:
            self._heures = None
            return
        bornes = []
        for v in (start, end):
            try:
                h, m = (int(x) for x in str(v).split(":"))
            except ValueError:
                raise ValueError(f"heure « HH:MM » attendue (reçu {v!r})") from None
            if not (0 <= h <= 23 and 0 <= m <= 59):
                raise ValueError(f"heure « HH:MM » attendue (reçu {v!r})")
            bornes.append(h * 60 + m)
        self._heures = (bornes[0], bornes[1])

    def _heure_permise(self, maintenant: datetime) -> bool:
        if self._heures is None:
            return True
        d, f = self._heures
        mn = maintenant.hour * 60 + maintenant.minute
        return d <= mn < f if d <= f else (mn >= d or mn < f)

    # ---- boucle --------------------------------------------------------------

    def _apps_actives(self):
        for app_id, entree in list(self._rt._apps.items()):
            m = entree.manifest
            if entree.is_started and m.triggers and m.has_permission(Permission.SCHEDULED_MISSIONS):
                yield app_id, m

    def _noter(self, t, app_id, nom, decision, detail="") -> SchedulerEvent:
        e = SchedulerEvent(t, app_id, nom, decision, detail)
        self.journal.append(e)
        return e

    def busy(self) -> bool:
        """Vrai si une mission tourne (lancée par l'ordonnanceur ou non)."""
        return (self._travail is not None and self._travail.is_alive()) \
            or self._rt._mission_lock.locked()

    def tick(self) -> List[SchedulerEvent]:
        """Un pas : met à jour chaque déclencheur, lance au plus une mission
        (dans un fil à part) ; rend les décisions de ce pas."""
        with self._verrou:
            return self._tick()

    def _tick(self) -> List[SchedulerEvent]:
        t, maintenant = self._clock(), self._mur()
        try:
            pct = self._batterie()
        except Exception:                    # noqa: BLE001 — capteur en panne : inconnue
            pct = None
        if pct is not None and not math.isfinite(pct):
            pct = None
        try:
            ici = self._pose()
        except Exception:                    # noqa: BLE001 — localisation en panne
            ici = None
        carte = self._rt._site_map.current if self._rt._site_map is not None else None

        vus, dus = set(), []
        for app_id, m in self._apps_actives():
            for spec in m.triggers:
                cle = (app_id, spec.name)
                vus.add(cle)
                etat = self._etats.get(cle)
                if etat is None or etat.spec != spec:   # nouveau, ou changé par une mise à jour
                    etat = self._etats[cle] = _Etat(spec, maintenant, t)
                du = self._du(etat, t, maintenant, pct, ici, carte)
                if du == "missed":
                    self._noter(t, app_id, spec.name, "missed",
                                f"occurrence en retard de plus de {self.GRACE_S:g} s")
                elif du:
                    dus.append((app_id, spec, etat))
        for cle in set(self._etats) - vus:        # app arrêtée ou désinstallée
            del self._etats[cle]

        sortie: List[SchedulerEvent] = []
        dus.sort(key=lambda d: (ORDRE_TYPES[d[1].kind], d[0], d[1].name))
        lance = False
        for app_id, spec, etat in dus:
            motif = (("collision", "un autre déclencheur a été lancé à ce pas") if lance
                     else self._motif_refus(app_id, spec, etat, t, maintenant))
            if motif is not None:
                sortie.append(self._noter(t, app_id, spec.name, *motif))
                continue
            etat.dernier_lancement = t
            lance = True
            sortie.append(self._noter(t, app_id, spec.name, "launched", spec.describe()))
            self._travail = threading.Thread(target=self._executer, args=(app_id, spec.name),
                                             name=f"tache-{app_id}-{spec.name}", daemon=True)
            self._travail.start()
        return sortie

    def _du(self, etat: _Etat, t: float, maintenant: datetime, pct, ici, carte):
        """True si le déclencheur tombe à ce pas, « missed » s'il est tombé
        trop tard, False sinon. Met à jour son état dans tous les cas."""
        s = etat.spec
        if s.kind == "daily":
            if etat.jour_traite == maintenant.date():
                return False
            h, m = (int(v) for v in s.at.split(":"))
            cible = maintenant.replace(hour=h, minute=m, second=0, microsecond=0)
            if maintenant < cible:
                return False
            etat.jour_traite = maintenant.date()
            return True if (maintenant - cible).total_seconds() <= self.GRACE_S else "missed"
        if s.kind == "interval":
            if t < etat.prochain:
                return False
            retard = t - etat.prochain
            etat.prochain = t + s.every_s
            return True if retard <= self.GRACE_S else "missed"
        if s.kind == "battery_below":
            if pct is None:
                return False                 # inconnue : jamais de déclenchement inventé
            if pct >= s.percent + self.BATTERY_HYSTERESIS:
                etat.arme = True
            if etat.arme and pct < s.percent:
                etat.arme = False
                return True
            return False
        # zone_enter
        if ici is None or carte is None:
            return False
        zone = next((z for z in carte.zones if z.name == s.zone), None)
        dedans = zone is not None and zone.contains(*ici)
        avant, etat.dedans = etat.dedans, dedans
        return avant is False and dedans

    def _motif_refus(self, app_id, spec, etat, t, maintenant) -> Optional[Tuple[str, str]]:
        if self._pause or (app_id, None) in self._desactives \
                or (app_id, spec.name) in self._desactives:
            return "disabled", "désactivé par l'humain"
        if self._rt._estop_actif():
            return "estop", "arrêt d'urgence actif"
        try:
            tombe = bool(self._chute())
        except Exception:                    # noqa: BLE001 — dans le doute, pas de mission
            tombe = True
        if tombe:
            return "fallen", "chute détectée"
        if not self._heure_permise(maintenant):
            return "hours", "hors des heures permises par l'administrateur"
        bloque = self._rt.mission_blocked_reason()
        if bloque is not None:
            return "blocked", bloque
        if self.busy():
            return "busy", "une mission est déjà en cours"
        if etat.dernier_lancement is not None and t - etat.dernier_lancement < self.MIN_REARM_S:
            return "rearm", f"déjà lancé il y a moins de {self.MIN_REARM_S:g} s"
        return None

    def _executer(self, app_id: str, nom: str) -> None:
        try:
            r = self._rt.run_mission(app_id, {"trigger": nom})
            detail = "app non démarrée" if r is None else (
                "réussie" if r.success else f"échec : {(r.error_message or '')[:120]}")
        except Exception as exc:             # noqa: BLE001 — noté, jamais propagé
            detail = f"erreur : {type(exc).__name__}"
        self._noter(self._clock(), app_id, nom, "finished", detail)

    def wait_idle(self, timeout: float = 60.0) -> bool:
        """Attend la fin de la mission lancée (tests) ; False si elle dure."""
        fil = self._travail
        if fil is not None:
            fil.join(timeout)
            return not fil.is_alive()
        return True

    def start(self) -> "MissionScheduler":
        """Fait tourner tick() toutes les TICK_S secondes, dans un fil."""
        def boucle():
            while not self._fin.wait(self.TICK_S):
                try:
                    self.tick()
                except Exception as exc:     # noqa: BLE001 — l'ordonnanceur ne meurt pas
                    self._noter(self._clock(), "-", "-", "error", type(exc).__name__)
        self._fin.clear()
        self._fil = threading.Thread(target=boucle, name="ordonnanceur", daemon=True)
        self._fil.start()
        return self

    def stop(self) -> None:
        self._fin.set()
        if self._fil is not None:
            self._fil.join(2 * self.TICK_S)
