"""
SDK, partie 3 : cycle de vie d'une mission.

Vérifié ici, en direct (RobotContext) puis depuis le processus de l'app
(pont JSON) :
  1. should_stop() / check_cancelled() : arrêt demandé par un humain ou
     par l'OS, fin proche du quota, arrêt d'urgence ; MissionCancelled
     (CANCELLED) ou EstopActiveError ; une action longue (go_to) rend la
     main avec CANCELLED ; l'arrêt forcé (quota) reste en place pour une
     app qui ignore la demande ;
  2. report_progress() : bornée (fraction dans [0, 1], message coupé et
     neutralisé, historique borné), lue par l'OS (get_progress) ;
  3. on_estop(), on_alert(), on_low_battery() : callbacks appelés dans le
     processus de l'app ; batterie inconnue : pas d'abonnement ;
  4. ask_confirmation() : sans humain, refus ; oui, non, pas de réponse ;
     seule une réponse True explicite confirme ; le nom affiché vient du
     manifeste ;
  5. permission manquante, arguments invalides, arrêt d'urgence, pour
     chaque API ;
  6. pont : schémas stricts.
"""
import contextlib
import io
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.watchdog.events import AlertKind, AlertSeverity, WatchdogAlert  # noqa: E402
from axis_os.store.runtime.hal_bridge import HALBridgeServer  # noqa: E402
from axis_os.store.sdk.app_base import (  # noqa: E402
    EstopActiveError, MissionResult, RobotApp, RobotContext,
)
from axis_os.store.sdk.lifecycle import (  # noqa: E402
    PROGRESS_HISTORY_MAX, PROGRESS_MESSAGE_MAX, IHumanInterface, MissionCancelled,
)
from axis_os.store.sdk.manifest import (  # noqa: E402
    PERMISSION_LEVELS, AppCategory, AppManifest, Permission, ProtectionLevel, ResourceQuota,
)
from axis_os.store.sdk.results import AxisRefusal, RefusalCode  # noqa: E402
from axis_os.testing import AppTestHarness, ScriptedHuman, SimulatedRobot  # noqa: E402

FAIL = 0


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


def silencieux(f, *a, **kw):
    with contextlib.redirect_stdout(io.StringIO()):
        return f(*a, **kw)


def leve(code, f, *a, **kw):
    """Vrai si l'appel lève un AxisRefusal de ce code."""
    try:
        silencieux(f, *a, **kw)
        return False
    except AxisRefusal as exc:
        return exc.code is code


def attendre(cond, delai=3.0):
    fin = time.monotonic() + delai
    while time.monotonic() < fin:
        if cond():
            return True
        time.sleep(0.02)
    return cond()


def manifeste(perms, app_id="com.test.cycle", name="Cycle de vie"):
    return AppManifest(app_id=app_id, name=name, version="1.0.0", author="T",
                       author_email="t@t.com", description="d",
                       category=AppCategory.UTILITIES, permissions=list(perms))


def contexte(robot, perms=None, **kw):
    toutes = [Permission.MISSION_LIFECYCLE, Permission.HUMAN_CONFIRMATION,
              Permission.WATCHDOG_READ, Permission.SYSTEM_INFO, Permission.NAVIGATION_GOTO,
              Permission.LOCOMOTION_WALK, Permission.ACTUATORS_WRITE_LEGS]
    ctx = RobotContext.for_testing(robot.hal, robot.watchdog,
                                   manifeste(toutes if perms is None else perms, **kw))
    ctx._set_clock(robot.clock)
    ctx._attach_services(locomotion=robot.locomotion, navigation=robot.navigation)
    return ctx


def alerte(robot, message="moteur chaud"):
    robot.watchdog._emit(WatchdogAlert(kind=AlertKind.TEMPERATURE_WARNING,
                                       severity=AlertSeverity.WARNING,
                                       joint_id="left_elbow_joint", message=message,
                                       value=71.0, threshold=70.0))


class HumainEnPanne(IHumanInterface):
    def confirm(self, app_id, app_name, question, timeout_s):
        raise ConnectionError("écran débranché")


class HumainAmbigu(IHumanInterface):
    """Rend une valeur « vraie » qui n'est pas True : jamais une
    confirmation."""
    def __init__(self, valeur):
        self.valeur = valeur

    def confirm(self, app_id, app_name, question, timeout_s):
        return self.valeur


section("0. Permissions déclarées")
check("MISSION_LIFECYCLE et HUMAN_CONFIRMATION sont NORMAL (rien ne bouge)",
      PERMISSION_LEVELS[Permission.MISSION_LIFECYCLE] is ProtectionLevel.NORMAL
      and PERMISSION_LEVELS[Permission.HUMAN_CONFIRMATION] is ProtectionLevel.NORMAL)

section("1. should_stop / check_cancelled en direct")
with SimulatedRobot() as robot:
    ctx = contexte(robot)
    check("Rien de demandé : should_stop() faux, check_cancelled() sans effet",
          not ctx.should_stop() and ctx.check_cancelled() is None)
    ctx._demander_arret("human")
    check("Arrêt demandé par un humain : should_stop() vrai", ctx.should_stop())
    check("… check_cancelled() lève MissionCancelled (code CANCELLED)",
          leve(RefusalCode.CANCELLED, ctx.check_cancelled))
    try:
        ctx.check_cancelled()
    except MissionCancelled as exc:
        check("… qui dit la source de la demande", "human" in str(exc), str(exc))

    ctx = contexte(robot)
    ctx._set_deadline(ctx.now() + 1.0)
    check("Moins de 2 s restantes au hook : should_stop() vrai (source os)",
          ctx.should_stop() and ctx._motif_arret() == "os")
    ctx = contexte(robot)
    ctx._fermer()
    check("Hook terminé : should_stop() vrai", ctx.should_stop())
    try:
        ctx._demander_arret("app")
        source_libre = True
    except ValueError:
        source_libre = False
    check("Source d'arrêt inconnue refusée (seulement human / os)", not source_libre)

    ctx = contexte(robot)
    threading.Timer(0.8, lambda: ctx._demander_arret("human")).start()
    r = silencieux(ctx.go_to, 3.0, 0.0, timeout_s=20.0)
    check("Arrêt demandé pendant go_to : l'action rend la main, CANCELLED, robot arrêté",
          r.code == RefusalCode.CANCELLED.value
          and robot.navigation.etat.name == "INACTIF", str(r))

with SimulatedRobot() as robot:
    ctx = contexte(robot)
    silencieux(robot.hal.emergency_stop_all)
    check("Arrêt d'urgence : should_stop() vrai", ctx.should_stop())
    check("… check_cancelled() lève EstopActiveError (ESTOP_ACTIVE)",
          leve(RefusalCode.ESTOP_ACTIVE, ctx.check_cancelled))
    try:
        ctx.check_cancelled()
    except EstopActiveError:
        check("… c'est bien l'EstopActiveError du SDK", True)

section("2. report_progress en direct")
with SimulatedRobot() as robot:
    ctx = contexte(robot)
    ctx.report_progress(0.25, "un quart")
    p = ctx._progression[-1]
    check("Progression notée : fraction, message, heure du robot",
          p.fraction == 0.25 and p.message == "un quart" and p.timestamp > 0, str(p))
    ctx.report_progress(1, "x" * 5000)
    check(f"Message coupé à {PROGRESS_MESSAGE_MAX} caractères",
          len(ctx._progression[-1].message) == PROGRESS_MESSAGE_MAX)
    ctx.report_progress(0.5, "ligne\n\x1b[2Jeffacée")
    check("Caractères de contrôle neutralisés", "\n" not in ctx._progression[-1].message
          and "\x1b" not in ctx._progression[-1].message, repr(ctx._progression[-1].message))
    for i in range(200):
        ctx.report_progress(i / 200)
    check(f"Historique borné à {PROGRESS_HISTORY_MAX}",
          len(ctx._progression) == PROGRESS_HISTORY_MAX)
    for label, args in [("fraction NaN", (float("nan"),)), ("fraction 1.5", (1.5,)),
                        ("fraction -0.1", (-0.1,)), ("fraction texte", ("0.5",)),
                        ("fraction booléen", (True,)), ("message nombre", (0.5, 42))]:
        check(f"{label} : INVALID_ARGUMENT",
              leve(RefusalCode.INVALID_ARGUMENT, ctx.report_progress, *args))
    silencieux(robot.hal.emergency_stop_all)
    ctx.report_progress(0.9, "arrêtée")
    check("Sous arrêt d'urgence, l'app peut encore dire où elle en est (rien ne bouge)",
          ctx._progression[-1].message == "arrêtée")

section("3. Événements en direct")
with SimulatedRobot(battery_percent=50.0) as robot:
    ctx = contexte(robot)
    vus = {"estop": 0, "alertes": [], "batterie": []}
    alerte(robot, "ancienne alerte")
    ctx.on_estop(lambda: vus.__setitem__("estop", vus["estop"] + 1))
    ctx.on_alert(lambda a: vus["alertes"].append(a))
    check("on_low_battery : batterie mesurée, abonnement accepté",
          ctx.on_low_battery(lambda pct: vus["batterie"].append(pct), threshold_percent=20) is True)
    time.sleep(0.3)
    check("Une alerte antérieure à l'abonnement n'est pas rejouée", vus["alertes"] == [],
          str(vus["alertes"]))
    alerte(robot, "moteur chaud")
    check("on_alert : la nouvelle alerte arrive (kind, severity, message, joint_id)",
          attendre(lambda: len(vus["alertes"]) == 1)
          and vus["alertes"][0]["kind"] == "TEMPERATURE_WARNING"
          and vus["alertes"][0]["joint_id"] == "left_elbow_joint", str(vus["alertes"]))
    robot.set_battery(15.0)
    check("on_low_battery : passage sous 20 % -> callback(15.0)",
          attendre(lambda: vus["batterie"] == [15.0]), str(vus["batterie"]))
    robot.set_battery(12.0)
    time.sleep(0.4)
    check("… une seule fois par passage sous le seuil", vus["batterie"] == [15.0],
          str(vus["batterie"]))
    robot.set_battery(60.0)
    time.sleep(0.3)
    robot.set_battery(10.0)
    check("… et à nouveau après être remontée", attendre(lambda: vus["batterie"] == [15.0, 10.0]),
          str(vus["batterie"]))
    silencieux(robot.hal.emergency_stop_all)
    check("on_estop : callback appelé une fois", attendre(lambda: vus["estop"] == 1))
    time.sleep(0.4)
    check("… une seule fois", vus["estop"] == 1, str(vus["estop"]))
    ctx._fermer()

with SimulatedRobot() as robot:
    ctx = contexte(robot)
    check("Batterie inconnue : on_low_battery rend False (aucune charge inventée)",
          silencieux(ctx.on_low_battery, lambda pct: None) is False)
    for label, f, args, kw in [
            ("on_estop(texte)", ctx.on_estop, ("pas une fonction",), {}),
            ("on_alert(None)", ctx.on_alert, (None,), {}),
            ("seuil 0 %", ctx.on_low_battery, (print,), {"threshold_percent": 0}),
            ("seuil 150 %", ctx.on_low_battery, (print,), {"threshold_percent": 150}),
            ("seuil NaN", ctx.on_low_battery, (print,), {"threshold_percent": float("nan")})]:
        check(f"{label} : INVALID_ARGUMENT", leve(RefusalCode.INVALID_ARGUMENT, f, *args, **kw))

    def boom():
        raise RuntimeError("callback bogué")
    vus = []
    ctx.on_alert(lambda a: boom())
    ctx.on_alert(lambda a: vus.append(a["message"]))
    alerte(robot, "après le bogue")
    check("Un callback qui lève n'empêche pas les autres", attendre(lambda: vus == ["après le bogue"]),
          str(vus))
    ctx._fermer()

section("4. ask_confirmation en direct")
with SimulatedRobot() as robot:
    ctx = contexte(robot, name="Livreur")
    r = silencieux(ctx.ask_confirmation, "Je peux ouvrir la porte ?", timeout_s=1.0)
    check("Aucun humain : REFUSED/HUMAN_UNAVAILABLE (jamais tenu pour un oui)",
          not r.ok and r.code == RefusalCode.HUMAN_UNAVAILABLE.value, str(r))
    humain = ScriptedHuman([True, False, None])
    ctx._set_human_interface(humain)
    r = ctx.ask_confirmation("Je peux\n\x1b[31mouvrir la porte ?", timeout_s=2.0)
    check("Oui : CONFIRMED", r.ok and r.status == "CONFIRMED", str(r))
    check("L'humain voit l'app_id vérifié, le nom du manifeste et une question neutralisée",
          humain.questions[0][0] == "com.test.cycle" and humain.questions[0][1] == "Livreur"
          and "\x1b" not in humain.questions[0][2] and "\n" not in humain.questions[0][2],
          str(humain.questions[0]))
    r = ctx.ask_confirmation("Et maintenant ?", timeout_s=2.0)
    check("Non : DECLINED/HUMAN_DECLINED", r.status == "DECLINED"
          and r.code == RefusalCode.HUMAN_DECLINED.value, str(r))
    t0 = time.monotonic()
    r = ctx.ask_confirmation("Personne ?", timeout_s=0.5)
    check("Pas de réponse : TIMEOUT après le délai", r.status == "TIMEOUT"
          and r.code == RefusalCode.TIMEOUT.value and time.monotonic() - t0 >= 0.45, str(r))
    for valeur in ("oui", 1, [True]):
        ctx._set_human_interface(HumainAmbigu(valeur))
        r = ctx.ask_confirmation("Sûr ?", timeout_s=1.0)
        check(f"Réponse {valeur!r} (pas True) : jamais une confirmation", not r.ok, str(r))
    ctx._set_human_interface(HumainEnPanne())
    r = ctx.ask_confirmation("Écran ?", timeout_s=1.0)
    check("Interface en panne : REFUSED/HUMAN_UNAVAILABLE", r.code == RefusalCode.HUMAN_UNAVAILABLE.value,
          str(r))
    for label, args, kw in [("question vide", ("",), {}), ("question nombre", (42,), {}),
                            ("délai nul", ("ok ?",), {"timeout_s": 0}),
                            ("délai d'une heure", ("ok ?",), {"timeout_s": 3600}),
                            ("délai NaN", ("ok ?",), {"timeout_s": float("nan")})]:
        check(f"{label} : INVALID_ARGUMENT",
              leve(RefusalCode.INVALID_ARGUMENT, ctx.ask_confirmation, *args, **kw))
    ctx._set_human_interface(ScriptedHuman([True]))
    silencieux(robot.hal.emergency_stop_all)
    check("Sous arrêt d'urgence : ESTOP_ACTIVE (pas de question qui mènerait à bouger)",
          leve(RefusalCode.ESTOP_ACTIVE, ctx.ask_confirmation, "On repart ?", timeout_s=1.0))
    check("… et l'humain n'a pas été dérangé", len(ctx._humain.questions) == 0)

section("5. Permissions manquantes (en direct)")
with SimulatedRobot(battery_percent=50.0) as robot:
    sans = contexte(robot, perms=[Permission.NAVIGATION_GOTO])
    for label, f, args, kw in [
            ("should_stop sans MISSION_LIFECYCLE", sans.should_stop, (), {}),
            ("check_cancelled sans MISSION_LIFECYCLE", sans.check_cancelled, (), {}),
            ("report_progress sans MISSION_LIFECYCLE", sans.report_progress, (0.5,), {}),
            ("on_estop sans MISSION_LIFECYCLE", sans.on_estop, (print,), {}),
            ("on_alert sans WATCHDOG_READ", sans.on_alert, (print,), {}),
            ("on_low_battery sans SYSTEM_INFO", sans.on_low_battery, (print,), {}),
            ("ask_confirmation sans HUMAN_CONFIRMATION", sans.ask_confirmation, ("ok ?",), {})]:
        check(f"{label} : PERMISSION_DENIED",
              leve(RefusalCode.PERMISSION_DENIED, f, *args, **kw))


# ─── Depuis le processus de l'app (pont JSON) ────────────────────────────

class AppPatiente(RobotApp):
    """Travaille par petits pas, rapporte sa progression, et s'arrête
    proprement dès que l'arrêt est demandé."""
    def manifest(self):
        return AppManifest(app_id="com.test.patiente", name="Patiente", version="1.0.0",
                           author="T", author_email="t@t.com", description="d",
                           category=AppCategory.UTILITIES,
                           permissions=[Permission.MISSION_LIFECYCLE, Permission.WATCHDOG_READ,
                                        Permission.SYSTEM_INFO, Permission.HUMAN_CONFIRMATION])

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        geste = params.get("geste")
        if geste == "confirmer":
            r = ctx.ask_confirmation("Je commence ?", timeout_s=params.get("delai", 2.0))
            return MissionResult(success=r.ok, mission_id="", app_id="", duration_s=0.0,
                                 metrics={"status": r.status, "code": r.code})
        if geste == "evenements":
            vus = []
            ctx.on_alert(lambda a: vus.append("alerte " + a["kind"]))
            ctx.on_low_battery(lambda pct: vus.append(f"batterie {pct:g}"),
                               threshold_percent=20)
            fin = ctx.now() + 3.0
            while ctx.now() < fin:
                ctx.sleep(0.05)
            return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0,
                                 metrics={"vus": vus})
        if geste == "estop":
            ctx.on_estop(lambda: ctx.report_progress(1.0, "estop vu"))
        if geste == "invalide":
            issues = {}
            for nom, appel in [("fraction", lambda: ctx.report_progress(float("nan"))),
                               ("question", lambda: ctx.ask_confirmation("", timeout_s=1.0)),
                               ("callback", lambda: ctx.on_estop("pas une fonction"))]:
                try:
                    appel()
                    issues[nom] = "accepté"
                except ValueError as exc:
                    issues[nom] = type(exc).__name__
            return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0,
                                 metrics=issues)
        n = 0
        while n < 400:
            if geste != "ignorer" and ctx.should_stop():
                if geste == "lever":
                    ctx.check_cancelled()
                return MissionResult(success=False, mission_id="", app_id="", duration_s=0.0,
                                     error_message="arrêtée à la demande", items_processed=n)
            n += 1
            ctx.report_progress(n / 400, f"pas {n}")
            ctx.sleep(0.02)
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0,
                             items_processed=n)


class AppSansCycle(RobotApp):
    """N'a déclaré aucune des permissions du cycle de vie."""
    def manifest(self):
        return AppManifest(app_id="com.test.sans_cycle", name="Sans", version="1.0.0",
                           author="T", author_email="t@t.com", description="d",
                           category=AppCategory.UTILITIES, permissions=[Permission.ACTUATORS_READ])

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        appels = {"should_stop": ctx.should_stop,
                  "report_progress": lambda: ctx.report_progress(0.5),
                  "on_estop": lambda: ctx.on_estop(print),
                  "ask_confirmation": lambda: ctx.ask_confirmation("ok ?", timeout_s=1.0)}
        appels[params["api"]]()
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0)


class AppTetue(AppPatiente):
    """Ignore les demandes d'arrêt : l'arrêt forcé (quota) s'applique."""
    def manifest(self):
        m = super().manifest()
        m.app_id = "com.test.tetue"
        m.quota = ResourceQuota(max_runtime_s=3.0)
        return m


section("6. Depuis le processus de l'app")
with AppTestHarness(AppPatiente(), runtime={"human_interface": ScriptedHuman([True, False])},
                    battery_percent=50.0) as h:
    vu = {}

    def demander():
        vu["progression"] = h.runtime.get_progress(h.app_id)
        vu["demande"] = h.runtime.request_stop(h.app_id)
    threading.Timer(1.5, demander).start()
    res = h.run()
    check("request_stop : l'app le voit par should_stop() et s'arrête d'elle-même",
          res is not None and not res.success and res.error_message == "arrêtée à la demande"
          and vu.get("demande") is True and 0 < res.items_processed < 400,
          f"{getattr(res, 'error_message', None)} {getattr(res, 'items_processed', None)}")
    p = vu.get("progression")
    check("get_progress pendant la mission : la dernière progression de l'app",
          p is not None and 0 < p.fraction < 1 and p.message.startswith("pas "), str(p))
    check("request_stop sans hook en cours : False", h.runtime.request_stop(h.app_id) is False)

    threading.Timer(1.0, lambda: h.runtime.request_stop(h.app_id, source="os")).start()
    res = h.run({"geste": "lever"})
    check("check_cancelled() dans le processus : MissionCancelled, mission notée annulée",
          res is not None and not res.success and "annulée" in (res.error_message or "")
          and "os" in (res.error_message or ""), getattr(res, "error_message", None))

    res = h.run({"geste": "confirmer"})
    check("ask_confirmation, l'humain dit oui : CONFIRMED", res.success
          and res.metrics.get("status") == "CONFIRMED", str(res.metrics))
    res = h.run({"geste": "confirmer"})
    check("… l'humain dit non : DECLINED/HUMAN_DECLINED", not res.success
          and res.metrics.get("code") == "HUMAN_DECLINED", str(res.metrics))
    res = h.run({"geste": "confirmer", "delai": 0.5})
    check("… plus personne ne répond : TIMEOUT", res.metrics.get("status") == "TIMEOUT",
          str(res.metrics))
    res = h.run({"geste": "invalide"})
    check("Arguments invalides refusés dans le processus (InvalidArgumentError)",
          res.metrics == {"fraction": "InvalidArgumentError", "question": "InvalidArgumentError",
                          "callback": "InvalidArgumentError"}, str(res.metrics))

    threading.Timer(0.8, lambda: alerte(h.robot)).start()
    threading.Timer(1.6, lambda: h.robot.set_battery(12.0)).start()
    res = h.run({"geste": "evenements"})
    vus = (res.metrics or {}).get("vus") if res is not None else None
    check("on_alert dans le processus de l'app : callback appelé",
          vus is not None and "alerte TEMPERATURE_WARNING" in vus, str(vus))
    check("on_low_battery dans le processus de l'app : callback appelé une fois",
          vus is not None and vus.count("batterie 12") == 1, str(vus))

    threading.Timer(1.0, lambda: silencieux(h.robot.hal.emergency_stop_all)).start()
    res = h.run({"geste": "estop"})
    check("on_estop dans le processus de l'app : callback appelé avant l'arrêt du processus",
          h.runtime.get_progress(h.app_id).message == "estop vu",
          str(h.runtime.get_progress(h.app_id)))
    check("… et la mission est notée interrompue par l'arrêt d'urgence",
          res is not None and not res.success and res.interrompue_par_arret_urgence,
          getattr(res, "error_message", None))

with AppTestHarness(AppPatiente(), runtime={"human_interface": None}) as h:
    res = h.run({"geste": "confirmer"})
    check("Sans interface humaine, dans le processus : HUMAN_UNAVAILABLE",
          res.metrics.get("code") == "HUMAN_UNAVAILABLE", str(res.metrics))

with AppTestHarness(AppSansCycle()) as h:
    for api, perm in [("should_stop", "MISSION_LIFECYCLE"), ("report_progress", "MISSION_LIFECYCLE"),
                      ("on_estop", "MISSION_LIFECYCLE"), ("ask_confirmation", "HUMAN_CONFIRMATION")]:
        res = h.run({"api": api})
        check(f"{api} sans {perm}, depuis le processus : mission refusée",
              res is not None and not res.success and perm in (res.error_message or ""),
              getattr(res, "error_message", None))

with AppTestHarness(AppTetue()) as h:
    t0 = time.monotonic()
    res = h.run({"geste": "ignorer"})
    duree = time.monotonic() - t0
    check("App qui ignore should_stop : l'arrêt forcé (quota de 3 s) s'applique quand même",
          res is not None and not res.success and duree < 6.0,
          f"{getattr(res, 'error_message', None)} en {duree:.1f} s")

section("7. Pont : schémas")
with SimulatedRobot() as robot:
    serveur = HALBridgeServer(contexte(robot))

    def appel(action, payload):
        return silencieux(serveur.traiter, {"type": "appel", "action": action,
                                            "payload": payload, "request_id": 1})
    for label, action, payload in [
            ("report_progress : fraction 2", "report_progress", {"fraction": 2, "message": ""}),
            ("report_progress : message trop long", "report_progress",
             {"fraction": 0.5, "message": "x" * 5000}),
            ("subscribe_event : événement inconnu", "subscribe_event", {"kind": "reboot"}),
            ("ask_confirmation : délai d'un jour", "ask_confirmation",
             {"question": "ok ?", "timeout_s": 86400}),
            ("ask_confirmation : champ « answer » ajouté par l'app", "ask_confirmation",
             {"question": "ok ?", "timeout_s": 1, "answer": True}),
            ("should_stop : champ inconnu", "should_stop", {"force": True})]:
        rep = appel(action, payload)
        check(f"{label} : MALFORMED_COMMAND", rep is not None and not rep["success"]
              and "MALFORMED" in rep["error"], str(rep))

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
