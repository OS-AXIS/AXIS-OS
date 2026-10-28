"""
Sons de l'OS (play_sound : jeu fixe, volume plafonné, débit limité), état du
réseau (network_status, sans connexion ouverte), rapports de plantage (sans
donnée personnelle ni secret, lisibles par l'administrateur seulement).
"""
import contextlib
import io
import os
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.services.network import FakeNetworkProbe, SystemNetworkProbe  # noqa: E402
from axis_os.store.runtime import hal_bridge  # noqa: E402
from axis_os.store.sdk.app_base import (  # noqa: E402
    EstopActiveError, MissionResult, PermissionError, RobotApp, RobotContext,
)
from axis_os.store.sdk.manifest import (  # noqa: E402
    AppCategory, AppManifest, Permission, ResourceQuota,
)
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


def manifeste(perms, app_id="com.test.divers", **kw):
    return AppManifest(app_id=app_id, name="Divers", version="1.2.3", author="T",
                       author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
                       permissions=list(perms), **kw)


def contexte(robot, perms, sons=True, reseau=True):
    ctx = RobotContext.for_testing(robot.hal, robot.watchdog, manifeste(perms))
    ctx._attach_services(sounds=robot.sounds if sons else None,
                         network=robot.network if reseau else None)
    return ctx


section("1. play_sound")
with SimulatedRobot() as robot:
    ctx = contexte(robot, [Permission.AUDIO_OUTPUT])
    r = ctx.play_sound("beep")
    check("Cas normal : PLAYED", r.ok and r.status == "PLAYED" and robot.sounds.played[-1]
          == ("beep", 0.5))
    ctx.play_sound("alert", volume=1.0)
    check("Volume 1,0 plafonné à 0,8", robot.sounds.played[-1] == ("alert", 0.8))
    check("Sans AUDIO_OUTPUT : PermissionError",
          leve(PermissionError, contexte(robot, []).play_sound, "beep"))
    for nom, f in (("son inconnu", lambda: ctx.play_sound("/etc/passwd")),
                   ("volume négatif", lambda: ctx.play_sound("beep", volume=-0.1)),
                   ("volume NaN", lambda: ctx.play_sound("beep", volume=float("nan")))):
        check(f"{nom} : InvalidArgumentError", leve(InvalidArgumentError, f))
    r = contexte(robot, [Permission.AUDIO_OUTPUT], sons=False).play_sound("beep")
    check("Pas de haut-parleur : REFUSED / CAPABILITY_MISSING", r.code == "CAPABILITY_MISSING")
    for _ in range(25):
        r = ctx.play_sound("beep")
    check("Plus de 20 sons par minute : RATE_LIMITED_APP", r.code == "RATE_LIMITED_APP")
    with contextlib.redirect_stdout(io.StringIO()):
        robot.hal.emergency_stop_all()
    check("Sous arrêt d'urgence : EstopActiveError",
          leve(EstopActiveError, contexte(robot, [Permission.AUDIO_OUTPUT]).play_sound, "beep"))


section("2. network_status")
with SimulatedRobot() as robot:
    ctx = contexte(robot, [Permission.SYSTEM_INFO])
    for etat in ("online", "local_only", "offline"):
        robot.network.state = etat
        check(f"Sonde « {etat} » : rendu tel quel", ctx.network_status() == etat)
    robot.network.state = "connecté !"
    check("Valeur inconnue de la sonde : « unknown »", ctx.network_status() == "unknown")
    check("Sans sonde : « unknown »", contexte(robot, [Permission.SYSTEM_INFO],
                                                reseau=False).network_status() == "unknown")
    check("Sans SYSTEM_INFO : PermissionError",
          leve(PermissionError, contexte(robot, []).network_status))


def systeme(interfaces, route):
    d = tempfile.mkdtemp()
    for nom, etat in interfaces.items():
        os.makedirs(os.path.join(d, "net", nom))
        Path(d, "net", nom, "operstate").write_text(etat + "\n")
    Path(d, "route").write_text("Iface\tDestination\tGateway\n" + route)
    return SystemNetworkProbe(os.path.join(d, "net"), os.path.join(d, "route"))


check("Système : aucune interface active → offline",
      systeme({"lo": "unknown", "eth0": "down"}, "").status() == "offline")
check("Système : interface active, pas de route par défaut → local_only",
      systeme({"eth0": "up"}, "eth0\t0000A8C0\t00000000\n").status() == "local_only")
check("Système : route par défaut → online",
      systeme({"eth0": "up"}, "eth0\t00000000\t0100A8C0\n").status() == "online")
check("Système illisible → unknown",
      SystemNetworkProbe("/nulle/part", "/nulle/part").status() == "unknown")


section("3. Rapports de plantage")


class AppPlante(RobotApp):
    def manifest(self):
        return manifeste([], app_id="com.test.plante")

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        raise ValueError("client alice@example.com, jeton sk-1234567890abcdef, au quai 3")


class AppBloquee(RobotApp):
    def manifest(self):
        return manifeste([], app_id="com.test.bloquee", quota=ResourceQuota(max_runtime_s=2.0))

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        while True:
            pass


with AppTestHarness(AppPlante()) as h:
    h.run({"client": "alice@example.com"})
    rapports = h.runtime.crash_reports()
    texte = repr(rapports)
    check("Une app qui plante laisse un rapport : app, version, hook, cause, type d'exception",
          len(rapports) == 1 and rapports[0].app_id == "com.test.plante"
          and rapports[0].version == "1.2.3" and rapports[0].hook == "run_mission"
          and rapports[0].kind == "APPLICATION_ERROR" and rapports[0].exception_type == "ValueError",
          texte)
    check("Ni message, ni paramètres, ni secret dans le rapport",
          not any(s in texte for s in ("alice", "sk-123", "quai", "client")), texte)
with AppTestHarness(AppBloquee()) as h:
    h.run()
    rapports = h.runtime.crash_reports("com.test.bloquee")
    check("Une app bloquée (quota de durée) laisse un rapport TIMEOUT",
          len(rapports) == 1 and rapports[0].kind == "TIMEOUT"
          and rapports[0].quota_rule == "TIMEOUT", repr(rapports))
check("Aucune action du pont ne lit les rapports",
      not any("crash" in a or "report" in a and a != "report_progress"
              for a in hal_bridge.SCHEMAS_ACTIONS))

print(f"\n{'=' * 66}\n  {'TOUT PASSE' if FAIL == 0 else f'{FAIL} ÉCHEC(S)'}\n{'=' * 66}")
sys.exit(1 if FAIL else 0)
