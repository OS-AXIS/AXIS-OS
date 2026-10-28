"""
Tâches planifiées (MissionScheduler) : déclencheurs du manifeste (heure fixe,
intervalle, batterie, zone), une seule mission à la fois, règle des
collisions, pas de rattrapage, rien pendant un arrêt d'urgence ou une chute,
heures permises, désactivation par l'humain ; puis une vraie mission lancée
dans le bac à sable du Play Store.
"""
import contextlib
import io
import sys
import threading
from datetime import datetime, timedelta
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.services.navigation.site_map import NAMED, Zone  # noqa: E402
from axis_os.store.runtime.play_store import PlayStoreRuntime  # noqa: E402
from axis_os.store.runtime.profil import Profil  # noqa: E402
from axis_os.store.runtime.scheduler import MissionScheduler  # noqa: E402
from axis_os.store.sdk.app_base import MissionResult, RobotApp  # noqa: E402
from axis_os.store.sdk.manifest import (  # noqa: E402
    PERMISSION_LEVELS, AppCategory, AppManifest, Permission, ProtectionLevel, TriggerSpec,
)
from axis_os.testing import SimulatedRobot  # noqa: E402

FAIL = 0


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


def manifeste(triggers, perms=None, app_id="com.test.ronde"):
    return AppManifest(app_id=app_id, name="Ronde", version="1.0.0", author="T",
                       author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
                       permissions=[Permission.SCHEDULED_MISSIONS] if perms is None else perms,
                       triggers=list(triggers))


class Monde:
    """Runtime factice : juste ce que lit l'ordonnanceur."""

    def __init__(self):
        self.t = 0.0
        self.mur = datetime(2026, 10, 28, 12, 0, 0)
        self.batterie = None
        self.pose = None
        self.tombe = False
        self.estop = False
        self.bloque = None
        self._clock = lambda: self.t
        self._apps = {}
        self._mission_lock = threading.Lock()
        self._robot = self._navigation_service = self._locomotion_service = None
        zone = Zone("quai", NAMED, ((0, 0), (2, 0), (2, 2), (0, 2)))
        self._site_map = SimpleNamespace(current=SimpleNamespace(zones=[zone]))
        self.lances = []
        self.retenir = None              # un Event : la mission dure tant qu'il n'est pas posé

    def installer(self, triggers, app_id="com.test.ronde", started=True, perms=None):
        self._apps[app_id] = SimpleNamespace(manifest=manifeste(triggers, perms, app_id),
                                             is_started=started)

    def _estop_actif(self):
        return self.estop

    def mission_blocked_reason(self):
        return self.bloque

    def run_mission(self, app_id, params):
        self.lances.append((app_id, params["trigger"]))
        if self.retenir is not None:
            self.retenir.wait(10)
        return MissionResult(success=True, mission_id="X", app_id=app_id, duration_s=0.0)

    def ordonnanceur(self):
        """L'ordonnanceur, après un premier pas (où il découvre les apps)."""
        s = MissionScheduler(self, wall_clock=lambda: self.mur,
                             battery=lambda: self.batterie, pose=lambda: self.pose,
                             fallen=lambda: self.tombe)
        s.tick()
        return s

    def avancer(self, s, sched):
        self.t += s
        self.mur += timedelta(seconds=s)
        evs = sched.tick()
        sched.wait_idle(5)
        return [(e.trigger, e.decision) for e in evs]


section("1. Déclencheurs dans le manifeste")
for spec, attendu in ((TriggerSpec("ronde", "daily", at="22:00"), True),
                      (TriggerSpec("ronde", "daily", at="25:00"), False),
                      (TriggerSpec("ronde", "interval", every_s=60), False),
                      (TriggerSpec("ronde", "interval", every_s=600), True),
                      (TriggerSpec("ronde", "battery_below", percent=2), False),
                      (TriggerSpec("ronde", "zone_enter", zone="quai"), True),
                      (TriggerSpec("ronde", "daily", at="22:00", every_s=600), False),
                      (TriggerSpec("Ronde!", "daily", at="22:00"), False)):
    erreurs = manifeste([spec]).validate()
    check(f"{spec.kind} {spec.at or spec.every_s or spec.percent or spec.zone} "
          f"{'accepté' if attendu else 'refusé'}", (not erreurs) == attendu, str(erreurs))
check("Déclencheurs sans SCHEDULED_MISSIONS : manifeste refusé",
      any("SCHEDULED_MISSIONS" in e for e in manifeste(
          [TriggerSpec("ronde", "daily", at="22:00")], perms=[]).validate()))
check("SCHEDULED_MISSIONS est DANGEROUS (confirmée par un humain)",
      PERMISSION_LEVELS[Permission.SCHEDULED_MISSIONS] is ProtectionLevel.DANGEROUS)
check("Texte montré à l'installation", TriggerSpec("ronde", "daily", at="22:00").describe()
      == "ronde : tous les jours à 22:00")


section("2. Heure fixe : à l'heure, une fois par jour, jamais en retard")
m = Monde()
m.mur = datetime(2026, 10, 28, 21, 59, 0)
m.installer([TriggerSpec("ronde", "daily", at="22:00")])
s = m.ordonnanceur()
check("21:59 : rien", m.avancer(0, s) == [] and m.lances == [])
check("22:00:30 : lancée, avec params {trigger: ronde}",
      m.avancer(90, s) == [("ronde", "launched")] and m.lances == [("com.test.ronde", "ronde")])
check("Le même soir : plus rien", all(m.avancer(60, s) == [] for _ in range(5)))
m.avancer(24 * 3600 - 6 * 60, s)
m.avancer(60, s)
check("Le lendemain à 22:00 : de nouveau", m.lances[-1] == ("com.test.ronde", "ronde")
      and len(m.lances) == 2, str(m.lances))
m = Monde()
m.mur = datetime(2026, 10, 28, 23, 0, 0)
m.installer([TriggerSpec("ronde", "daily", at="22:00")])
s = m.ordonnanceur()
m.avancer(1, s)
check("Installée à 23:00 : rien ce soir-là (pas de rattrapage)", m.lances == [])
m = Monde()
m.mur = datetime(2026, 10, 28, 21, 0, 0)
m.installer([TriggerSpec("ronde", "daily", at="22:00")])
s = m.ordonnanceur()
m.avancer(1, s)
check("OS endormi de 21:00 à 22:05 : « missed », rien lancé",
      m.avancer(65 * 60, s) == [] and m.lances == [] and s.journal[-1].decision == "missed")


section("3. Intervalle : pas de rafale après une longue absence")
m = Monde()
m.installer([TriggerSpec("tour", "interval", every_s=600)])
s = m.ordonnanceur()
m.avancer(1, s)
check("Avant 600 s : rien", m.avancer(500, s) == [])
check("À 600 s : lancé", m.avancer(100, s) == [("tour", "launched")])
m.avancer(3 * 3600, s)
check("Trois heures sans pas : « missed », aucune des 18 occurrences rattrapée",
      len(m.lances) == 1 and s.journal[-1].decision == "missed")
for _ in range(9):
    m.avancer(60, s)
m.avancer(60, s)
check("Puis une seule fois, 600 s plus tard", len(m.lances) == 2, str(m.lances))


section("4. Batterie sous un seuil : une fois par passage, inconnue = jamais")
m = Monde()
m.installer([TriggerSpec("recharge", "battery_below", percent=40)])
s = m.ordonnanceur()
m.batterie = 50
m.avancer(1, s)
m.batterie = 39
check("50 % → 39 % : lancé", m.avancer(1, s) == [("recharge", "launched")])
m.batterie = 30
check("Reste sous le seuil : plus rien", m.avancer(400, s) == [])
m.batterie = 42
m.avancer(400, s)
m.batterie = 38
check("Remonte à 42 % (moins que l'hystérésis) : pas réarmé", m.avancer(400, s) == [])
m.batterie = 46
m.avancer(1, s)
m.batterie = 39
check("Remonte à 46 % puis redescend : relancé", m.avancer(1, s) == [("recharge", "launched")])
m2 = Monde()
m2.installer([TriggerSpec("recharge", "battery_below", percent=40)])
s2 = m2.ordonnanceur()
check("Batterie inconnue : jamais de déclenchement",
      all(m2.avancer(1, s2) == [] for _ in range(5)) and m2.lances == [])
m2.batterie = float("nan")
check("Batterie NaN : inconnue, rien", m2.avancer(1, s2) == [])


section("5. Entrée dans une zone : pas de rafale au bord")
m = Monde()
m.installer([TriggerSpec("arrivee", "zone_enter", zone="quai")])
s = m.ordonnanceur()
m.pose = (1.0, 1.0)
check("Déjà dedans au premier pas : rien (ce n'est pas une entrée)", m.avancer(1, s) == [])
m.pose = (5.0, 5.0)
m.avancer(1, s)
m.pose = (1.0, 1.0)
check("Dehors → dedans : lancé", m.avancer(1, s) == [("arrivee", "launched")])
decisions = []
for _ in range(10):
    m.pose = (5.0, 5.0)
    m.avancer(1, s)
    m.pose = (1.0, 1.0)
    decisions += [d for _, d in m.avancer(1, s)]
check("Bord franchi dix fois en 20 s : aucune nouvelle mission (« rearm »)",
      len(m.lances) == 1 and set(decisions) == {"rearm"}, str(decisions))
m.pose = None
check("Position inconnue : rien", m.avancer(400, s) == [])


section("6. Une mission à la fois, collisions, arrêt d'urgence, chute, heures, humain")
m = Monde()
m.mur = datetime(2026, 10, 28, 21, 59, 30)
m.installer([TriggerSpec("ronde", "daily", at="22:00"),
             TriggerSpec("recharge", "battery_below", percent=40)])
m.installer([TriggerSpec("autre", "daily", at="22:00")], app_id="com.test.autre")
s = m.ordonnanceur()
m.batterie = 50
m.avancer(1, s)
m.batterie = 30
d = dict(m.avancer(40, s))
check("Trois dus au même pas : batterie lancée, les deux autres « collision »",
      d == {"recharge": "launched", "ronde": "collision", "autre": "collision"}
      and m.lances == [("com.test.ronde", "recharge")], str(d))
m = Monde()
m.installer([TriggerSpec("tour", "interval", every_s=300)])
s = m.ordonnanceur()
m.retenir = threading.Event()
m.t += 300
s.tick()                                 # lancée, retenue
m.t += 300
evs = s.tick()
check("Mission en cours : le suivant est sauté (« busy »), pas mis en file",
      [e.decision for e in evs] == ["busy"] and len(m.lances) == 1)
m.retenir.set()
s.wait_idle(5)
with m._mission_lock:
    m.t += 300
    check("Mission lancée par un humain en cours : « busy »",
          [e.decision for e in s.tick()] == ["busy"])
for attr, val, attendu in (("estop", True, "estop"), ("tombe", True, "fallen"),
                           ("bloque", "retour à la station de charge", "blocked")):
    m2 = Monde()
    m2.installer([TriggerSpec("tour", "interval", every_s=300)])
    s2 = m2.ordonnanceur()
    setattr(m2, attr, val)
    check(f"{attendu} : rien lancé", m2.avancer(300, s2) == [("tour", attendu)]
          and m2.lances == [])
m2 = Monde()
m2.installer([TriggerSpec("tour", "interval", every_s=300)])
s2 = m2.ordonnanceur()
s2._chute = lambda: 1 / 0
check("Détecteur de chute en panne : dans le doute, rien", m2.avancer(300, s2) == [("tour", "fallen")])
m2 = Monde()
m2.mur = datetime(2026, 10, 28, 2, 59, 0)
m2.installer([TriggerSpec("nuit", "daily", at="03:00")])
s2 = m2.ordonnanceur()
s2.set_allowed_hours("06:00", "23:00")
check("03:00 hors des heures permises (06:00–23:00) : « hours »",
      m2.avancer(60, s2) == [("nuit", "hours")] and m2.lances == [])
s2.set_allowed_hours("20:00", "06:00")
check("Plage qui passe minuit (20:00–06:00) : 03:00 permis", s2._heure_permise(m2.mur)
      and not s2._heure_permise(datetime(2026, 10, 28, 12, 0)))
for nom, f in (("app", lambda s_: s_.set_enabled("com.test.ronde", False)),
               ("déclencheur", lambda s_: s_.set_enabled("com.test.ronde", False, "tour")),
               ("tout", lambda s_: s_.set_paused(True))):
    m2 = Monde()
    m2.installer([TriggerSpec("tour", "interval", every_s=300)])
    s2 = m2.ordonnanceur()
    f(s2)
    check(f"Désactivé par l'humain ({nom}) : « disabled »",
          m2.avancer(300, s2) == [("tour", "disabled")] and m2.lances == [])
m2 = Monde()
m2.installer([TriggerSpec("tour", "interval", every_s=300)], started=False)
s2 = m2.ordonnanceur()
check("App non démarrée : aucun déclencheur actif", m2.avancer(300, s2) == [])
m2 = Monde()
m2.installer([TriggerSpec("tour", "interval", every_s=300)], perms=[Permission.LOCATION])
s2 = m2.ordonnanceur()
check("Manifeste sans SCHEDULED_MISSIONS : rien", m2.avancer(300, s2) == [])


section("7. Une vraie mission planifiée, dans le bac à sable")


class AppRonde(RobotApp):
    vu = None

    def manifest(self):
        return manifeste([TriggerSpec("ronde", "daily", at="22:00")])

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0,
                             metrics={"trigger": params.get("trigger")})


with SimulatedRobot() as robot:
    sortie = io.StringIO()
    with contextlib.redirect_stdout(sortie):
        rt = robot.make_runtime()
        ok = rt.install(AppRonde()) and rt.start_app("com.test.ronde")
    check("Installée ; la tâche planifiée est montrée à l'installation",
          ok and "Tâche planifiée : ronde : tous les jours à 22:00" in sortie.getvalue())
    mur = [datetime(2026, 10, 28, 21, 59, 50)]
    s = MissionScheduler(rt, wall_clock=lambda: mur[0])
    with contextlib.redirect_stdout(io.StringIO()):
        s.tick()
        mur[0] = datetime(2026, 10, 28, 22, 0, 5)
        evs = s.tick()
        s.wait_idle(60)
    fin = [e for e in s.journal if e.decision == "finished"]
    check("22:00 : mission lancée par le Play Store, dans son processus, et réussie",
          [e.decision for e in evs] == ["launched"] and fin and fin[-1].detail == "réussie",
          str(list(s.journal)))
    rt.add_mission_blocker(lambda: "retour à la station de charge")
    with contextlib.redirect_stdout(io.StringIO()):
        r = rt.run_mission("com.test.ronde", {})
    check("Bloqueur de l'OS : une mission lancée par un humain est refusée aussi",
          r is not None and not r.success and "station" in r.error_message)
    rt._bloqueurs.clear()
    with contextlib.redirect_stdout(io.StringIO()):
        robot.hal.emergency_stop_all()
        mur[0] = datetime(2026, 10, 29, 22, 0, 5)
        evs = s.tick()
    check("Arrêt d'urgence : la ronde du lendemain n'est pas lancée",
          [e.decision for e in evs] == ["estop"]
          and sum(e.decision == "launched" for e in s.journal) == 1, str(evs))


class AppSansConfirmation(AppRonde):
    pass


with SimulatedRobot() as robot:
    with contextlib.redirect_stdout(io.StringIO()):
        rt = PlayStoreRuntime(robot.hal, robot.watchdog, profil=Profil.DEVELOPPEMENT,
                              confirmer_permissions=lambda *a, **k: False)
        ok = rt.install(AppSansConfirmation())
    check("SCHEDULED_MISSIONS refusée par l'humain : installation refusée", not ok)

print(f"\n{'=' * 66}\n  {'TOUT PASSE' if FAIL == 0 else f'{FAIL} ÉCHEC(S)'}\n{'=' * 66}")
sys.exit(1 if FAIL else 0)
