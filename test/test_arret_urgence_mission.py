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
     l'appel ; tant que cet appel abandonné tourne, le micro est occupé :
     une autre écoute est refusée tout de suite, puis servie de nouveau ;
  6. on_start en cours : interrompu de même, l'app n'est pas démarrée ;
     on_stop, appelé sous e-stop, est interrompu de même : l'app est
     arrêtée sans attendre la fin de son quota ;
  7. SandboxRunner seul : état KILLED, cause ARRET_URGENCE, même si un
     programme lancé hors du groupe de processus survit ; sans arrêt
     d'urgence, rien ne change ;
  8. l'app reçoit l'arrêt : son prochain appel lève EstopActiveError, rien
     n'est écrit par un appel refusé, et seules les actions sans effet sur
     le robot (log, stop_walking…) restent servies ;
  9. un appel dont l'action n'est pas un texte (app boguée) est refusé par
     le pont, avec ou sans arrêt d'urgence, sans faire lever run_mission.
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

    def on_stop(self, ctx):
        ctx.listen(1.0)

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

    def __init__(self):
        self.entree = threading.Event()   # posé quand l'OS commence l'écoute

    def listen(self, duration_s=3.0, app_id=None):
        self.entree.set()
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


def mission_avec_estop(robot, rt, app_id, delai_s=1.5, estop=True, pret=None):
    """Lance la mission dans un thread, déclenche l'e-stop (chemin du HAL,
    comme le Supervisor) dès que `pret()` est vrai (l'app est dans l'état
    à tester ; 10 s au plus), ou après `delai_s` sans `pret`, et rend
    (résultat, secondes entre l'e-stop et le retour)."""
    res = {}

    def lancer():
        with contextlib.redirect_stdout(io.StringIO()):
            res["r"] = rt.run_mission(app_id, {})
        res["fin"] = time.monotonic()

    t = threading.Thread(target=lancer)
    t.start()
    atteint = None
    if pret is None:
        time.sleep(delai_s)
    else:
        fin = time.monotonic() + 10.0
        while not pret() and time.monotonic() < fin:
            time.sleep(0.01)
        atteint = pret()
    t_estop = time.monotonic()
    if estop:
        with contextlib.redirect_stdout(io.StringIO()):
            robot.emergency_stop_all()
    t.join(timeout=30.0)
    # Le redirect_stdout du thread vaut pour tout le processus : les check()
    # viennent après lui, et la sortie est rendue s'il ne s'est pas terminé.
    if t.is_alive():
        sys.stdout = sys.__stdout__
    if atteint is not None:
        check("(l'app a atteint l'état à tester avant l'e-stop)", atteint)
    return res.get("r"), res.get("fin", float("inf")) - t_estop


def a_bouge(robot):
    """Vrai dès que la mission a fait bouger la hanche (robot neuf à 0)."""
    return lambda: robot.get_full_state()["left_hip_pitch_joint"].position != 0.0


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
r, duree = mission_avec_estop(robot, rt, "com.test.attend", pret=a_bouge(robot))
verifier_interruption("Mission qui attend", robot, rt, r, duree)

section("2. Mission qui avale EstopActiveError")
robot, rt = pile(AppQuiAvale())
r, duree = mission_avec_estop(robot, rt, "com.test.avale", pret=a_bouge(robot))
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
parole = ParoleLente()
robot, rt = pile(AppQuiEcoute(), speech_service=parole)
r, duree = mission_avec_estop(robot, rt, "com.test.ecoute", pret=parole.entree.is_set)
check(f"Mission rendue (verrou compris) en moins de {BORNE_S:g} s, sans attendre la fin "
      f"de l'écoute de 8 s", duree < BORNE_S, f"{duree:.2f} s")
check("… notée interrompue, pas facturée",
      r is not None and r.interrompue_par_arret_urgence and rt._billing == [])
check("… son processus ne tourne plus", hotes_vivants() == [], str(hotes_vivants()))
t0 = time.monotonic()
with contextlib.redirect_stdout(io.StringIO()):
    rt.stop_app("com.test.ecoute")
check("Sous e-stop, l'écoute d'on_stop est refusée au lieu d'attendre",
      time.monotonic() - t0 < BORNE_S, f"{time.monotonic() - t0:.2f} s")


class AppQuiEcouteApres(RobotApp):
    """Écoute une fois, sur un autre robot sans e-stop, et rend la durée de
    l'appel et ce que dit le journal de mission."""
    def manifest(self):
        return manifeste("com.test.ecoute_apres", [Permission.SENSORS_MICROPHONE])

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        import time as _t
        t0 = _t.monotonic()
        ctx.listen(0.5)
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0,
                             metrics={"duree_s": _t.monotonic() - t0,
                                      "occupe": any("occupé" in l for l in ctx.get_mission_log())})


check("(l'écoute abandonnée tourne encore dans l'OS)", SandboxRunner._peripherique_occupe())
_, rt_apres = pile(AppQuiEcouteApres(), speech_service=ParoleLente())
with contextlib.redirect_stdout(io.StringIO()):
    r = rt_apres.run_mission("com.test.ecoute_apres", {})
check("Tant qu'elle tourne, le micro est occupé : une autre écoute, sans e-stop, est refusée "
      "tout de suite au lieu d'attendre", r is not None and r.metrics.get("occupe") is True
      and r.metrics.get("duree_s", 99.0) < 1.0, str(getattr(r, "metrics", r)))
for fil in list(SandboxRunner._abandonnes):
    fil.join(timeout=15.0)


class ParoleRapide:
    registry = None

    def listen(self, duration_s=3.0, app_id=None):
        return ServiceResult(ok=True, value=None, reason="silence")


_, rt_apres = pile(AppQuiEcouteApres(), speech_service=ParoleRapide())
with contextlib.redirect_stdout(io.StringIO()):
    r = rt_apres.run_mission("com.test.ecoute_apres", {})
check("Une fois l'écoute abandonnée finie, l'écoute est de nouveau servie",
      r is not None and r.metrics.get("occupe") is False, str(getattr(r, "metrics", r)))

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
if t.is_alive():
    sys.stdout = sys.__stdout__
check(f"on_start interrompu en moins de {BORNE_S:g} s", res.get("fin", float("inf")) - t_estop
      < BORNE_S, f"{res.get('fin', float('inf')) - t_estop:.2f} s")
check("… et l'app n'est pas démarrée", res.get("pret") is False
      and not rt._apps["com.test.demarrage"].is_started)

class AppQuiDemarreVite(RobotApp):
    """on_start immédiat : il répond avant la fin du délai de grâce."""
    def manifest(self):
        return manifeste("com.test.demarre_vite", [])

    def on_start(self, ctx):
        ctx.log("prête")
        return True

    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0)


robot, rt = pile(AppQuiDemarreVite())
rt._apps["com.test.demarre_vite"].is_started = False
with contextlib.redirect_stdout(io.StringIO()):
    robot.emergency_stop_all()
    pret = rt.start_app("com.test.demarre_vite")
check("Sous e-stop, une app qui répond vite à on_start n'est pas démarrée non plus",
      pret is False and not rt._apps["com.test.demarre_vite"].is_started)
robot, rt = pile(AppQuiDemarreVite())
rt._apps["com.test.demarre_vite"].is_started = False
vrai_hook = rt._executer_hook


def hook_puis_estop(*a, **kw):
    resultat = vrai_hook(*a, **kw)     # on_start répond, sans e-stop...
    robot.emergency_stop_all()         # ... qui arrive juste après
    return resultat


rt._executer_hook = hook_puis_estop
with contextlib.redirect_stdout(io.StringIO()):
    pret = rt.start_app("com.test.demarre_vite")
check("E-stop juste après un on_start réussi : l'app n'est pas démarrée",
      pret is False and not rt._apps["com.test.demarre_vite"].is_started)



class AppQuiSArreteLentement(RobotApp):
    """on_stop qui attend 15 s (rangement long)."""
    def manifest(self):
        return manifeste("com.test.arret_lent", [])

    def on_start(self, ctx): return True

    def on_stop(self, ctx):
        t0 = ctx.now()
        while ctx.now() - t0 < 15.0:
            ctx.sleep(0.05)

    def run_mission(self, ctx, params):
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0)


robot, rt = pile(AppQuiSArreteLentement())
with contextlib.redirect_stdout(io.StringIO()):
    robot.emergency_stop_all()
    t0 = time.monotonic()
    rt.stop_app("com.test.arret_lent")
check(f"Sous e-stop, on_stop est interrompu : stop_app rend la main en moins de {BORNE_S:g} s",
      time.monotonic() - t0 < BORNE_S, f"{time.monotonic() - t0:.2f} s")
check("… et l'app est arrêtée", not rt._apps["com.test.arret_lent"].is_started)

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
# et vit encore 6 s : il ne doit retenir ni le pont ni le retour. L'arrêt
# n'est déclenché qu'une fois ce programme lancé (marqueur).
import tempfile  # noqa: E402
marqueur = Path(tempfile.mkdtemp(prefix="axis_descendant_")) / "lance"
vu = {}


def arret_descendant():
    if marqueur.exists():
        vu.setdefault("t", time.monotonic())
        return True
    return False


sortie = io.StringIO()
with contextlib.redirect_stdout(sortie):
    res8 = SandboxRunner("t.descendant", ResourceQuota(max_runtime_s=10.0)).run_app(
        reference_fonction(os.system), "fonction",
        args=[f"setsid -f sleep 6; touch {marqueur}; sleep 30"], arret=arret_descendant)
duree8 = time.monotonic() - vu.get("t", float("-inf"))
check("(le descendant existait : l'OS l'a signalé)", "garde sa sortie ouverte" in sortie.getvalue(),
      sortie.getvalue()[-300:])
check("Un descendant sorti du groupe ne retient pas le retour", duree8 < BORNE_S
      and res8.state is SandboxState.KILLED, f"{duree8:.2f} s, {res8.state}")
def sans_veilleur():
    """Lance une fonction de 20 s alors que le veilleur ne peut pas démarrer
    (plus de threads disponibles) ; rend l'exception levée. (Une fonction et
    non du code de niveau module : le processus d'une app de ce script n'en
    reprend que les définitions.)"""
    import threading as vrai_threading
    import axis_os.store.runtime.sandbox as module_sandbox

    class ThreadVeilleurImpossible(vrai_threading.Thread):
        def start(self):
            if self.name.startswith("arret-urgence"):
                raise RuntimeError("can't start new thread")
            super().start()

    class ThreadingVuParSandbox:
        """threading tel que le voit sandbox.py seul : le module partagé
        n'est pas modifié pour les autres threads du processus."""
        Thread = ThreadVeilleurImpossible

        def __getattr__(self, nom):
            return getattr(vrai_threading, nom)

    module_sandbox.threading = ThreadingVuParSandbox()
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            SandboxRunner("t.veilleur", ResourceQuota(max_runtime_s=10.0)).run_app(
                reference_fonction(time.sleep), "fonction", args=[20], arret=lambda: False)
        return None
    except RuntimeError as exc:
        return exc
    finally:
        module_sandbox.threading = vrai_threading


leve = sans_veilleur()
time.sleep(0.5)
check("Veilleur impossible à démarrer : l'erreur remonte telle quelle, et le processus de "
      "l'app est quand même arrêté", leve is not None and "start new thread" in str(leve)
      and hotes_vivants() == [], f"{leve!r} {hotes_vivants()}")

section("8. L'app reçoit l'arrêt ; seules les actions sans effet restent servies")


class AppQuiRecoitLArret(RobotApp):
    """Bouge, puis enchaîne des appels sans effet sur le robot jusqu'à
    recevoir l'arrêt ; essaie alors d'écrire en mémoire et d'arrêter la
    marche, et rend aussitôt ce qu'elle a vu."""
    def manifest(self):
        return manifeste("com.test.recoit", [Permission.ACTUATORS_WRITE_LEGS,
                                             Permission.STORAGE_PERSISTENT,
                                             Permission.LOCOMOTION_WALK])

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        import time as _t
        from axis_os.hal.core.types import ActuatorCommand, ControlMode
        ctx.move_legs({"left_hip_pitch_joint": ActuatorCommand(ControlMode.POSITION, position=0.02)})
        fin, recu = _t.monotonic() + 15.0, False
        while _t.monotonic() < fin and not recu:
            try:
                ctx.now()
            except EstopActiveError:
                recu = True
            _t.sleep(0.01)
        try:
            ecrit = ctx.remember("apres_arret", 1)
        except EstopActiveError:
            ecrit = "refusé"
        marche = ctx.stop_walking()
        ctx.log("arrêt reçu")
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0,
                             metrics={"estop_recu": recu, "remember": ecrit,
                                      "stop_walking": marche})


class LocomotionFactice:
    def __init__(self):
        self.arrets = 0

    def halt(self):
        self.arrets += 1
        return ServiceResult(ok=True, value=None)


from axis_os.services.memory.service import MemoryService  # noqa: E402
import tempfile  # noqa: E402
memoire = MemoryService(Path(tempfile.mkdtemp(prefix="axis_estop_")))
with contextlib.redirect_stdout(io.StringIO()):
    memoire.start()
loco = LocomotionFactice()
robot, rt = pile(AppQuiRecoitLArret(), memory_service=memoire, locomotion_service=loco)
r, duree = mission_avec_estop(robot, rt, "com.test.recoit", pret=a_bouge(robot))
m = getattr(r, "metrics", None) or {}
check("L'app a reçu l'arrêt à son appel suivant (EstopActiveError) et a rendu elle-même son "
      "résultat, avant d'être tuée", m.get("estop_recu") is True
      and duree < SandboxRunner.DELAI_ARRET_URGENCE_S + 0.3, f"{m} {duree:.2f} s")
check("… son écriture en mémoire, après l'arrêt, est refusée", m.get("remember") == "refusé"
      and not memoire.app_load("com.test.recoit", "apres_arret").ok, str(m))
check("… stop_walking reste servi (il ne fait qu'arrêter)", m.get("stop_walking") is True
      and loco.arrets >= 1, f"{m} arrêts={loco.arrets}")
check("… mission notée interrompue, pas facturée",
      r is not None and r.interrompue_par_arret_urgence and rt._billing == [])

section("9. Appel dont l'action n'est pas un texte")


class AppMessageMalforme(RobotApp):
    """App boguée : envoie au pont un appel dont l'action est une liste, et
    rend la dernière erreur reçue. Avec APRES_ARRET, bouge d'abord, puis
    recommence jusqu'à l'arrêt d'urgence."""
    APRES_ARRET = False

    def manifest(self):
        return manifeste("com.test.malforme", [Permission.ACTUATORS_WRITE_LEGS])

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        import time as _t
        from axis_os.hal.core.types import ActuatorCommand, ControlMode
        if self.APRES_ARRET:
            ctx.move_legs({"left_hip_pitch_joint":
                           ActuatorCommand(ControlMode.POSITION, position=0.02)})
        erreur, fin, n = "", _t.monotonic() + 15.0, 0
        while _t.monotonic() < fin:
            n += 1
            erreur = ctx._appel_canal(["listen"], {}, 1000 + n, 2.0).error or ""
            if not self.APRES_ARRET or erreur.startswith("ESTOP_ACTIVE"):
                break
            _t.sleep(0.02)
        return MissionResult(success=False, mission_id="", app_id="", duration_s=0.0,
                             metrics={"erreur": erreur[:80]})


class AppMessageMalformeApresArret(AppMessageMalforme):
    APRES_ARRET = True


robot, rt = pile(AppMessageMalforme())
try:
    with contextlib.redirect_stdout(io.StringIO()):
        r, leve = rt.run_mission("com.test.malforme", {}), None
except TypeError as exc:
    r, leve = None, exc
check("Sans e-stop : run_mission rend un résultat, sans lever", leve is None and r is not None,
      repr(leve))
check("… et l'app reçoit le refus du schéma du pont (MALFORMED_COMMAND)",
      str(getattr(r, "metrics", {}).get("erreur", "")).startswith("MALFORMED_COMMAND"),
      str(getattr(r, "metrics", r)))
robot, rt = pile(AppMessageMalformeApresArret())
r, duree = mission_avec_estop(robot, rt, "com.test.malforme", pret=a_bouge(robot))
check("Après l'e-stop : un résultat rendu (pas d'exception), noté interrompu",
      r is not None and r.interrompue_par_arret_urgence and duree < BORNE_S,
      f"{getattr(r, 'error_message', r)} {duree:.2f} s")

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
