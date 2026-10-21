"""
Un arrêt d'urgence INTERROMPT la mission en cours (SECURITY.md, ancienne
limite n° 6).

Avant : un e-stop refusait les commandes de mouvement, mais une mission qui
ne bougeait plus gardait le robot (et le verrou de mission) jusqu'à son
délai, rendait success=True et était facturée.

Vérifié ici, chaque fois sur un robot neuf (un e-stop ne se réarme pas) :
  1. mission qui attend après un mouvement : interrompue dans un délai
     borné, notée « interrompue par arrêt d'urgence », jamais facturée,
     verrou libéré, robot toujours arrêté (mission suivante refusée) ;
  2. mission qui avale EstopActiveError en boucle : même chose ;
  3. mission qui déclenche elle-même l'e-stop puis prétend avoir réussi :
     notée interrompue, pas facturée ;
  4. mission en plein calcul, sans appel au robot : son processus est tué ;
  5. mission bloquée dans un appel de l'OS (écoute) : son processus est tué
     et la mission rend la main (verrou compris) sans attendre la fin de
     l'appel ;
  6. on_start en cours : interrompu de même, l'app n'est pas démarrée ;
  7. SandboxRunner seul : état KILLED, cause ARRET_URGENCE, même si un
     programme lancé hors du groupe de processus survit ; sans arrêt
     d'urgence, rien ne change.
"""
import contextlib
import io
import os
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import psutil  # noqa: E402

from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer  # noqa: E402
from axis_os.hal.core.normalized_robot import NormalizedRobotHAL  # noqa: E402
from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL  # noqa: E402
from axis_os.hal.watchdog.system import WatchdogSystem  # noqa: E402
from axis_os.services.base import ServiceResult  # noqa: E402
from axis_os.store.runtime.play_store import (  # noqa: E402
    PlayStoreRuntime, accepter_tout_SIMULATION,
)
from axis_os.store.runtime.processus_app import reference_fonction  # noqa: E402
from axis_os.store.runtime.profil import Profil  # noqa: E402
from axis_os.store.runtime.sandbox import (  # noqa: E402
    SandboxErrorKind, SandboxRunner, SandboxState,
)
from axis_os.store.sdk.app_base import EstopActiveError, MissionResult, RobotApp  # noqa: E402
from axis_os.store.sdk.manifest import (  # noqa: E402
    AppCategory, AppManifest, Permission, PricingModel, ResourceQuota,
)

FAIL = 0
# Borne entre l'e-stop et le retour de run_mission : 20 ms de détection,
# 0,5 s de grâce, puis l'arrêt du processus ; large marge pour la CI.
BORNE_S = 3.0


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


def manifeste(app_id, perms):
    return AppManifest(app_id=app_id, name=app_id, version="1.0.0", author="T",
                       author_email="t@t.com", description="d",
                       category=AppCategory.UTILITIES, permissions=perms,
                       pricing_model=PricingModel.PAY_PER_MISSION, base_price_eur=1.0,
                       quota=ResourceQuota(max_runtime_s=20.0))


class AppQuiAttend(RobotApp):
    """Bouge, puis attend 15 s sans plus rien commander."""
    def manifest(self):
        return manifeste("com.test.attend", [Permission.ACTUATORS_WRITE_LEGS])

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        from axis_os.hal.core.types import ActuatorCommand, ControlMode
        ctx.move_legs({"left_hip_pitch_joint": ActuatorCommand(ControlMode.POSITION, position=0.02)})
        t0 = ctx.now()
        while ctx.now() - t0 < 15.0:
            ctx.sleep(0.05)
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0,
                             items_processed=1)


class AppQuiAvale(RobotApp):
    """Bouge en boucle et avale EstopActiveError."""
    def manifest(self):
        return manifeste("com.test.avale", [Permission.ACTUATORS_WRITE_LEGS])

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        import time as _t
        from axis_os.hal.core.types import ActuatorCommand, ControlMode
        fin = _t.monotonic() + 15.0
        while _t.monotonic() < fin:
            try:
                ctx.move_legs({"left_hip_pitch_joint":
                               ActuatorCommand(ControlMode.POSITION, position=0.02)})
            except EstopActiveError:
                pass
            _t.sleep(0.01)
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0)


class AppQuiSArrete(RobotApp):
    """Bouge, déclenche elle-même l'e-stop, puis prétend avoir réussi."""
    def manifest(self):
        return manifeste("com.test.sarrete", [Permission.ACTUATORS_WRITE_LEGS,
                                              Permission.EMERGENCY_STOP])

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        from axis_os.hal.core.types import ActuatorCommand, ControlMode
        ctx.move_legs({"left_hip_pitch_joint": ActuatorCommand(ControlMode.POSITION, position=0.02)})
        ctx.emergency_stop()
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0)


class AppQuiCalcule(RobotApp):
    """Calcule sans fin, sans jamais appeler le robot."""
    def manifest(self):
        return manifeste("com.test.calcule", [])

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        n = 0
        while True:
            n += 1


class AppQuiEcoute(RobotApp):
    """Reste bloquée dans une écoute servie par l'OS."""
    def manifest(self):
        return manifeste("com.test.ecoute", [Permission.SENSORS_MICROPHONE])

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        ctx.listen(5.0)
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0)


class AppQuiDemarreLentement(RobotApp):
    """on_start qui attend 15 s (initialisation longue)."""
    def manifest(self):
        return manifeste("com.test.demarrage", [])

    def on_start(self, ctx):
        t0 = ctx.now()
        while ctx.now() - t0 < 15.0:
            ctx.sleep(0.05)
        return True

    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0)


class ParoleLente:
    """Service de parole dont l'écoute dure 8 s, quoi qu'on lui demande."""
    registry = None

    def listen(self, duration_s=3.0, app_id=None):
        time.sleep(8.0)
        return ServiceResult(ok=True, value=None, reason="silence")


def pile(app, **services):
    with contextlib.redirect_stdout(io.StringIO()):
        robot = NormalizedRobotHAL(UnitreeG1HAL(f"g1_{app.manifest().app_id}"),
                                   build_g1_normalizer())
        robot.initialize_all()
        rt = PlayStoreRuntime(robot, WatchdogSystem(robot), profil=Profil.SIMULATION,
                              confirmer_permissions=accepter_tout_SIMULATION, **services)
        rt.install(app)
        rt.start_app(app.manifest().app_id)
    return robot, rt


def hotes_vivants():
    """Processus d'app (hote_app.py) encore vivants sous ce processus."""
    vivants = []
    for p in psutil.Process().children(recursive=True):
        try:
            if any("hote_app.py" in a for a in p.cmdline()) and p.status() != psutil.STATUS_ZOMBIE:
                vivants.append(p.pid)
        except psutil.Error:
            pass
    return vivants


def mission_avec_estop(robot, rt, app_id, delai_s=1.5, estop=True):
    """Lance la mission dans un thread, déclenche l'e-stop (chemin du HAL,
    comme le Supervisor) après `delai_s`, et rend (résultat, secondes entre
    l'e-stop et le retour)."""
    res = {}

    def lancer():
        with contextlib.redirect_stdout(io.StringIO()):
            res["r"] = rt.run_mission(app_id, {})
        res["fin"] = time.monotonic()

    t = threading.Thread(target=lancer)
    t.start()
    time.sleep(delai_s)
    t_estop = time.monotonic()
    if estop:
        with contextlib.redirect_stdout(io.StringIO()):
            robot.emergency_stop_all()
    t.join(timeout=30.0)
    return res.get("r"), res.get("fin", float("inf")) - t_estop


def verifier_interruption(nom, robot, rt, r, duree):
    check(f"{nom} : interrompue en moins de {BORNE_S:g} s après l'e-stop", duree < BORNE_S,
          f"{duree:.2f} s")
    check("… échec, notée « interrompue par arrêt d'urgence »",
          r is not None and not r.success and r.interrompue_par_arret_urgence
          and "arrêt d'urgence" in (r.error_message or ""), str(getattr(r, "error_message", r)))
    check("… jamais facturée", rt._billing == [] and rt.billing_summary()["total_gross_eur"] == 0)
    libre = rt._mission_lock.acquire(blocking=False)
    if libre:
        rt._mission_lock.release()
    check("… verrou de mission libéré", libre)
    with contextlib.redirect_stdout(io.StringIO()):
        suivante = rt.run_mission(r.app_id if r else "", {})
    check("… le robot reste à l'arrêt : mission suivante refusée",
          robot.estop_active and suivante is not None and not suivante.success
          and "E-STOP" in (suivante.error_message or ""))


section("1. Mission qui attend après un mouvement")
robot, rt = pile(AppQuiAttend())
r, duree = mission_avec_estop(robot, rt, "com.test.attend")
verifier_interruption("Mission qui attend", robot, rt, r, duree)

section("2. Mission qui avale EstopActiveError")
robot, rt = pile(AppQuiAvale())
r, duree = mission_avec_estop(robot, rt, "com.test.avale")
verifier_interruption("Mission qui avale l'arrêt", robot, rt, r, duree)

section("3. Mission qui déclenche l'e-stop puis prétend avoir réussi")
robot, rt = pile(AppQuiSArrete())
r, _ = mission_avec_estop(robot, rt, "com.test.sarrete", delai_s=0.0, estop=False)
check("L'app a bien déclenché l'e-stop", robot.estop_active)
check("… mission notée interrompue malgré success=True",
      r is not None and not r.success and r.interrompue_par_arret_urgence,
      str(getattr(r, "error_message", r)))
check("… et pas facturée", rt._billing == [])

section("4. Mission en plein calcul, sans appel au robot")
robot, rt = pile(AppQuiCalcule())
r, duree = mission_avec_estop(robot, rt, "com.test.calcule")
check(f"Interrompue en moins de {BORNE_S:g} s", duree < BORNE_S, f"{duree:.2f} s")
check("… notée interrompue", r is not None and r.interrompue_par_arret_urgence)
check("… son processus ne tourne plus", hotes_vivants() == [], str(hotes_vivants()))

section("5. Mission bloquée dans une écoute servie par l'OS")
robot, rt = pile(AppQuiEcoute(), speech_service=ParoleLente())
r, duree = mission_avec_estop(robot, rt, "com.test.ecoute")
check(f"Mission rendue (verrou compris) en moins de {BORNE_S:g} s, sans attendre la fin "
      f"de l'écoute de 8 s", duree < BORNE_S, f"{duree:.2f} s")
check("… notée interrompue, pas facturée",
      r is not None and r.interrompue_par_arret_urgence and rt._billing == [])
check("… son processus ne tourne plus", hotes_vivants() == [], str(hotes_vivants()))

section("6. on_start en cours")
with contextlib.redirect_stdout(io.StringIO()):
    robot = NormalizedRobotHAL(UnitreeG1HAL("g1_demarrage"), build_g1_normalizer())
    robot.initialize_all()
    rt = PlayStoreRuntime(robot, WatchdogSystem(robot), profil=Profil.SIMULATION,
                          confirmer_permissions=accepter_tout_SIMULATION)
    rt.install(AppQuiDemarreLentement())
res = {}


def demarrer():
    with contextlib.redirect_stdout(io.StringIO()):
        res["pret"] = rt.start_app("com.test.demarrage")
    res["fin"] = time.monotonic()


t = threading.Thread(target=demarrer)
t.start()
time.sleep(1.5)
t_estop = time.monotonic()
with contextlib.redirect_stdout(io.StringIO()):
    robot.emergency_stop_all()
t.join(timeout=30.0)
check(f"on_start interrompu en moins de {BORNE_S:g} s", res.get("fin", float("inf")) - t_estop
      < BORNE_S, f"{res.get('fin', float('inf')) - t_estop:.2f} s")
check("… et l'app n'est pas démarrée", res.get("pret") is False
      and not rt._apps["com.test.demarrage"].is_started)

section("7. SandboxRunner seul")
spec = reference_fonction(time.sleep)
t0 = time.monotonic()
with contextlib.redirect_stdout(io.StringIO()):
    res6 = SandboxRunner("t.arret", ResourceQuota(max_runtime_s=10.0)).run_app(
        spec, "fonction", args=[8], arret=lambda: True)
check("Arrêt d'urgence : processus arrêté vite", time.monotonic() - t0 < BORNE_S,
      f"{time.monotonic() - t0:.2f} s")
check("… état KILLED, cause ARRET_URGENCE",
      res6.state is SandboxState.KILLED and res6.error_kind is SandboxErrorKind.ARRET_URGENCE,
      f"{res6.state} {res6.error_kind}")
with contextlib.redirect_stdout(io.StringIO()):
    res7 = SandboxRunner("t.normal", ResourceQuota(max_runtime_s=10.0)).run_app(
        spec, "fonction", args=[0.2], arret=lambda: False)
check("Sans arrêt d'urgence : exécution normale", res7.success, str(res7))
# Un programme lancé par la fonction quitte le groupe de processus (setsid)
# et vit encore 6 s : il ne doit retenir ni le pont ni le retour.
t0 = time.monotonic()
with contextlib.redirect_stdout(io.StringIO()):
    res8 = SandboxRunner("t.descendant", ResourceQuota(max_runtime_s=10.0)).run_app(
        reference_fonction(os.system), "fonction", args=["setsid -f sleep 6; sleep 30"],
        arret=lambda: True)
check("Un descendant sorti du groupe ne retient pas le retour", time.monotonic() - t0 < BORNE_S
      and res8.state is SandboxState.KILLED, f"{time.monotonic() - t0:.2f} s, {res8.state}")

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
