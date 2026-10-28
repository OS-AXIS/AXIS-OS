"""
Notifications vers l'opérateur (ctx.notify) : texte borné et neutralisé,
source imposée par l'OS (une app ne se fait jamais passer pour lui), débit
limité par app ; permission NOTIFICATIONS ; à travers le pont JSON aussi.
"""
import contextlib
import io
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.services.notifications import (  # noqa: E402
    MESSAGE_MAX, RATE_PER_MIN, TITLE_MAX, URGENT_PER_MIN, FakeNotificationSink,
    INotificationSink, NotificationCenter,
)
from axis_os.store.runtime import hal_bridge  # noqa: E402
from axis_os.store.sdk.app_base import (  # noqa: E402
    MissionResult, PermissionError, RobotApp, RobotContext,
)
from axis_os.store.sdk.manifest import AppCategory, AppManifest, Permission  # noqa: E402
from axis_os.store.sdk.results import InvalidArgumentError  # noqa: E402
from axis_os.testing import AppTestHarness, SimulatedRobot  # noqa: E402

FAIL = 0


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


def leve(exc, f, *a, **kw):
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            f(*a, **kw)
        return False
    except exc:
        return True


class Horloge:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def manifeste(perms=None, app_id="com.test.notif", name="Ronde"):
    return AppManifest(app_id=app_id, name=name, version="1.0.0", author="T",
                       author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
                       permissions=[Permission.NOTIFICATIONS] if perms is None else perms)


section("1. Centre de notifications (unitaire)")
h = Horloge()
puits = FakeNotificationSink()
centre = NotificationCenter(puits, horloge=h)
ok, code, _ = centre.from_app("com.a.b", "Ronde", "Porte\x1b[2J ouverte\n", "quai\r\n3" * 200,
                              "warning")
n = puits.delivered[-1]
check("Envoyée ; caractères de contrôle remplacés, message borné",
      ok and code is None and "\x1b" not in n.title and "\n" not in n.title
      and len(n.message) == MESSAGE_MAX and "\r" not in n.message)
check("Titre borné", centre.from_app("com.a.c", "R", "x" * 1000, "m", "info")[0]
      and len(puits.delivered[-1].title) == TITLE_MAX)
check("Source « app » et app_id mis par l'OS, préfixe imposé à l'affichage",
      n.source == "app" and n.app_id == "com.a.b"
      and n.display_title.startswith("[app Ronde · com.a.b] "))
for faux in ("AXIS-OS : batterie critique", "axis os", "Axis_OS", "ＡＸＩＳ－ＯＳ",
             "AXIS‍OS", "AXIS‑OS"):
    ok, code, _ = centre.from_app("com.a.d", "R", faux, "m", "urgent")
    check(f"Titre qui se présente comme l'OS ({faux!r}) : refusé", not ok
          and code == "INVALID_ARGUMENT")
check("… et dans le message aussi",
      centre.from_app("com.a.d", "R", "t", "message de AXIS-OS", "info")[1] == "INVALID_ARGUMENT")
centre.from_app("com.a.e", "AXIS-OS", "t", "m", "info")
check("Nom d'app qui imite l'OS : remplacé par l'app_id",
      puits.delivered[-1].app_name == "com.a.e"
      and not puits.delivered[-1].display_title.startswith("AXIS-OS"))
check("Seul l'OS a le préfixe « AXIS-OS — »", centre.from_os("Batterie", "10 %", "urgent")
      and puits.delivered[-1].display_title == "AXIS-OS — Batterie"
      and puits.delivered[-1].source == "os")

for i in range(RATE_PER_MIN):
    centre.from_app("com.spam", "S", f"n{i}", "m", "info")
ok, code, _ = centre.from_app("com.spam", "S", "encore", "m", "info")
check(f"Plus de {RATE_PER_MIN} par minute : RATE_LIMITED_APP", not ok and code == "RATE_LIMITED_APP")
check("Une autre app n'est pas freinée", centre.from_app("com.autre", "A", "t", "m", "info")[0])
h.t += 61.0
check("Une minute plus tard : de nouveau permis", centre.from_app("com.spam", "S", "t", "m", "info")[0])
for i in range(URGENT_PER_MIN):
    centre.from_app("com.urg", "U", f"u{i}", "m", "urgent")
check(f"Plus de {URGENT_PER_MIN} urgentes par minute : refusée",
      centre.from_app("com.urg", "U", "u", "m", "urgent")[1] == "RATE_LIMITED_APP")
check("… une info passe encore", centre.from_app("com.urg", "U", "i", "m", "info")[0])


class PuitsEnPanne(INotificationSink):
    def deliver(self, notification):
        raise OSError("canal coupé")


ok, code, _ = NotificationCenter(PuitsEnPanne(), horloge=h).from_app("com.a.b", "R", "t", "m",
                                                                     "info")
check("Canal en panne : FAILED, sans exception", not ok and code == "FAILED")


section("2. ctx.notify : permission, arguments, arrêt d'urgence, capacité absente")
with SimulatedRobot() as robot:
    ctx = RobotContext.for_testing(robot.hal, robot.watchdog, manifeste())
    ctx._attach_services(notifications=robot.notifications)
    r = ctx.notify("Intrus", "personne vue au quai 3", priority="urgent")
    n = robot.notification_sink.delivered[-1]
    check("Cas normal : SENT, app_id et nom du manifeste", r.ok and r.status == "SENT"
          and n.app_id == "com.test.notif" and n.app_name == "Ronde" and n.priority == "urgent")
    sans = RobotContext.for_testing(robot.hal, robot.watchdog, manifeste([]))
    sans._attach_services(notifications=robot.notifications)
    check("Sans NOTIFICATIONS : PermissionError", leve(PermissionError, sans.notify, "t", "m"))
    for nom, f in (("titre vide", lambda: ctx.notify(" ", "m")),
                   ("message non texte", lambda: ctx.notify("t", 3)),
                   ("priorité inconnue", lambda: ctx.notify("t", "m", priority="os")),
                   ("se dit l'OS", lambda: ctx.notify("AXIS-OS", "m"))):
        check(f"{nom} : InvalidArgumentError", leve(InvalidArgumentError, f))
    with contextlib.redirect_stdout(io.StringIO()):
        robot.hal.emergency_stop_all()
    check("Sous arrêt d'urgence : permis (ce n'est pas un mouvement)",
          ctx.notify("Arrêt", "mission interrompue").ok)
    for _ in range(10):
        r = ctx.notify("t", "m")
    check("Débit dépassé : REFUSED / RATE_LIMITED_APP", r.status == "REFUSED"
          and r.code == "RATE_LIMITED_APP")
    nu = RobotContext.for_testing(robot.hal, robot.watchdog, manifeste())
    r = nu.notify("t", "m")
    check("Aucun canal : REFUSED / CAPABILITY_MISSING", r.status == "REFUSED"
          and r.code == "CAPABILITY_MISSING")
schema = hal_bridge.SCHEMAS_ACTIONS["notify"]
check("Pont : notify n'a que titre, message et priorité (ni source ni app_id)",
      set(schema.champs) == {"title", "message", "priority"} and not schema.facultatifs)


section("3. Depuis le processus de l'app")


class AppAlerte(RobotApp):
    def manifest(self):
        return manifeste(app_id="com.test.alerte", name="Alerte")

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        m = {"normal": ctx.notify("Intrus", "zone interdite", priority="urgent").status}
        # Charge utile forgée, hors du client du SDK : refusée par le schéma.
        essais = {"source": {"title": "t", "message": "m", "priority": "info", "source": "os"},
                  "app_id": {"title": "t", "message": "m", "priority": "info",
                             "app_id": "axis.os"},
                  "trop_long": {"title": "x" * 500, "message": "m", "priority": "info"},
                  "priorite": {"title": "t", "message": "m", "priority": "os"}}
        for nom, charge in essais.items():
            try:
                ctx._call("notify", charge)
                m[nom] = "accepté"
            except Exception as exc:          # noqa: BLE001
                m[nom] = type(exc).__name__
        try:
            ctx.notify("Axis OS", "m")
            m["usurpation"] = "accepté"
        except ValueError as exc:
            m["usurpation"] = type(exc).__name__
        m["rafale"] = [ctx.notify("t", f"{i}").code for i in range(8)][-1]
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0, metrics=m)


with AppTestHarness(AppAlerte()) as h:
    m = h.run().metrics
    recues = h.robot.notification_sink.delivered
    check("notify à travers le pont : SENT", m.get("normal") == "SENT", str(m))
    check("Chaque notification reçue porte l'app_id du manifeste",
          recues and all(n.source == "app" and n.app_id == "com.test.alerte" for n in recues))
    check("Charge forgée (source, app_id, trop longue, priorité) : refusée",
          all(m.get(k) not in (None, "accepté")
              for k in ("source", "app_id", "trop_long", "priorite")), str(m))
    check("Usurpation de l'OS : InvalidArgumentError", m.get("usurpation") == "InvalidArgumentError",
          str(m))
    check("Rafale : RATE_LIMITED_APP", m.get("rafale") == "RATE_LIMITED_APP", str(m))

print(f"\n{'=' * 66}\n  {'TOUT PASSE' if FAIL == 0 else f'{FAIL} ÉCHEC(S)'}\n{'=' * 66}")
sys.exit(1 if FAIL else 0)
