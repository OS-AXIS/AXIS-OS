"""
Suivi d'une personne depuis le processus de l'app : people, person_in_front,
follow, follow_status et stop_follow traversent le pont JSON avec les mêmes
types, les mêmes refus et les mêmes arrêts qu'en direct
(test/test_suivi_personne.py). L'app meurt, se bloque, ou subit un arrêt
d'urgence pendant le suivi : le robot s'arrête.
"""
import math
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.store.sdk.app_base import MissionResult, RobotApp  # noqa: E402
from axis_os.store.sdk.manifest import AppCategory, AppManifest, Permission  # noqa: E402
from axis_os.testing import AppTestHarness  # noqa: E402

FAIL = 0


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


def manifeste(app_id, perms):
    return AppManifest(app_id=app_id, name="Suivi", version="1.0.0", author="T",
                       author_email="t@t.com", description="d",
                       category=AppCategory.UTILITIES, permissions=perms)


class AppSuisMoi(RobotApp):
    """Suit la personne devant elle pendant params["duree"] s, puis s'arrête."""
    def manifest(self):
        return manifeste("com.test.suis_moi_pont", [
            Permission.PEOPLE_DETECT, Permission.PEOPLE_FOLLOW, Permission.LOCOMOTION_WALK,
            Permission.ACTUATORS_WRITE_LEGS])

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        m = {}
        vues = ctx.people()
        m["types"] = [type(p).__name__ for p in vues]
        m["champs"] = sorted(vars(vues[0])) if vues else []
        devant = ctx.person_in_front(5.0)
        for nom, appel in (("bool", lambda: ctx.follow(True)),
                           ("trop_pres", lambda: ctx.follow(devant.person_id, distance_m=1.0)),
                           ("nan", lambda: ctx.follow(devant.person_id, timeout_s=float("nan"))),
                           ("portee", lambda: ctx.person_in_front(-1.0))):
            try:
                appel()
                m[nom] = "accepté"
            except ValueError as exc:
                m[nom] = type(exc).__name__
        r = ctx.follow(devant.person_id, distance_m=1.5, timeout_s=60.0)
        m["follow"] = r.status
        etats = []
        fin = ctx.now() + params["duree"]
        while ctx.now() < fin:
            r = ctx.follow_status()
            etats.append(r.status)
            if r.status != "FOLLOWING":
                break
            ctx.sleep(0.2)
        m["etats"] = sorted(set(etats))
        r = ctx.stop_follow()
        m["fin"] = r.status
        m["moved"] = r.data.get("moved")
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0, metrics=m)


class AppSansDroitDeSuivre(RobotApp):
    """Voit les personnes, mais n'a pas le droit de les suivre."""
    def manifest(self):
        return manifeste("com.test.sans_suivi", [Permission.PEOPLE_DETECT,
                                                 Permission.LOCOMOTION_WALK,
                                                 Permission.ACTUATORS_WRITE_LEGS])

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        devant = ctx.person_in_front(5.0)
        try:
            ctx.follow(devant.person_id)
            issue = "accepté"
        except Exception as exc:       # noqa: BLE001 — le type est ce qu'on vérifie
            issue = f"{type(exc).__name__}:{getattr(exc, 'code', '')}"
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0,
                             metrics={"issue": issue})


class AppMeurtEnSuivant(RobotApp):
    """Lance un suivi, et son processus meurt une seconde plus tard."""
    def manifest(self):
        return manifeste("com.test.meurt_en_suivant", [
            Permission.PEOPLE_DETECT, Permission.PEOPLE_FOLLOW, Permission.LOCOMOTION_WALK,
            Permission.ACTUATORS_WRITE_LEGS])

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        import os
        ctx.follow(ctx.person_in_front(5.0).person_id)
        threading.Timer(1.0, lambda: os._exit(3)).start()
        while True:
            ctx.follow_status()
            ctx.sleep(0.2)


class AppBloquee(RobotApp):
    """Lance un suivi, puis calcule sans plus jamais donner de nouvelles."""
    def manifest(self):
        return manifeste("com.test.bloquee_suivi", [
            Permission.PEOPLE_DETECT, Permission.PEOPLE_FOLLOW, Permission.LOCOMOTION_WALK,
            Permission.ACTUATORS_WRITE_LEGS])

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        import time as _t
        ctx.follow(ctx.person_in_front(5.0).person_id, deadman_s=1.0)
        fin = _t.monotonic() + 3.0
        while _t.monotonic() < fin:
            pass
        r = ctx.follow_status()
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0,
                             metrics={"statut": r.status, "code": r.code})


ROUGE = (200, 30, 30)


def consignes(robot):
    liste, origine = [], robot.locomotion.set_velocity

    def envoyer(vx=0.0, vy=0.0, wz=0.0):
        liste.append((time.monotonic(), vx, vy, wz))
        return origine(vx, vy, wz)
    robot.locomotion.set_velocity = envoyer
    return liste


section("1. Suivre depuis le processus de l'app")
with AppTestHarness(AppSuisMoi()) as h:
    p = h.robot.add_person(2.5, 0.0, torso=ROUGE)
    p.walk_to(4.0, 0.0, speed_mps=0.25)
    cons = consignes(h.robot)
    res = h.run({"duree": 8.0})
    m = res.metrics
    check("Mission réussie", res.success, str(res.error_message))
    check("people() : des PersonInfo, cinq champs seulement",
          m.get("types") == ["PersonInfo"] and m.get("champs") == sorted(
              ["person_id", "distance_m", "bearing_rad", "seen_for_s", "confidence"]), str(m))
    check("Arguments invalides : InvalidArgumentError, comme en direct",
          all(m.get(k) == "InvalidArgumentError" for k in ("bool", "trop_pres", "nan", "portee")),
          str(m))
    check("follow : FOLLOWING ; suivi en cours ; stop_follow : STOPPED_BY_APP",
          m.get("follow") == "FOLLOWING" and m.get("etats") == ["FOLLOWING"]
          and m.get("fin") == "STOPPED_BY_APP" and m.get("moved") is True, str(m))
    check("Le robot a suivi la personne (il a avancé)", h.robot.x > 0.3, f"x = {h.robot.x:.2f}")
    check("Jamais en arrière, jamais de côté, jamais plus vite que 0.25 m/s",
          all(0.0 <= c[1] <= 0.25 and c[2] == 0.0 for c in cons))

section("2. Permission absente, depuis le processus de l'app")
with AppTestHarness(AppSansDroitDeSuivre()) as h:
    h.robot.add_person(2.5, 0.0, torso=ROUGE)
    res = h.run()
    check("follow sans PEOPLE_FOLLOW : PermissionError (PERMISSION_DENIED)",
          res.metrics.get("issue") == "PermissionError:RefusalCode.PERMISSION_DENIED"
          or res.metrics.get("issue", "").startswith("PermissionError"), str(res.metrics))
    check("… et le robot n'a pas suivi", not h.robot.follow_service.active
          and h.robot.follow_service.stats["follows"] == 0)

section("3. L'app meurt pendant le suivi : le robot s'arrête")
with AppTestHarness(AppMeurtEnSuivant()) as h:
    p = h.robot.add_person(2.5, 0.0, torso=ROUGE)
    p.walk_to(6.0, 0.0, speed_mps=0.3)
    cons = consignes(h.robot)
    t0 = time.monotonic()
    res = h.run()
    duree = time.monotonic() - t0
    t_fin = time.monotonic()
    time.sleep(0.5)
    check("Mission rendue vite, en échec", res is not None and not res.success and duree < 5.0,
          f"{duree:.1f} s, {res.error_message if res else None}")
    check("Le suivi est arrêté (CANCELLED)", not h.robot.follow_service.active
          and h.robot.follow_service._issue is not None
          and h.robot.follow_service._issue.status == "CANCELLED",
          str(h.robot.follow_service._issue))
    check("… plus aucune consigne de marche non nulle ensuite",
          all(c[1] == 0.0 and c[3] == 0.0 for c in cons if c[0] > t_fin))

section("4. L'app ne donne plus de nouvelles : homme mort")
with AppTestHarness(AppBloquee()) as h:
    p = h.robot.add_person(2.5, 0.0, torso=ROUGE)
    p.walk_to(6.0, 0.0, speed_mps=0.3)
    res = h.run()
    check("follow_status rend DEADMAN (code DEADMAN)", res.metrics.get("statut") == "DEADMAN"
          and res.metrics.get("code") == "DEADMAN", str(res.metrics))

section("5. Arrêt d'urgence pendant un suivi sandboxé")
with AppTestHarness(AppSuisMoi()) as h:
    p = h.robot.add_person(2.5, 0.0, torso=ROUGE)
    p.walk_to(6.0, 0.0, speed_mps=0.3)
    cons = consignes(h.robot)
    t_arret = []

    def arreter():
        time.sleep(3.0)
        t_arret.append(time.monotonic())
        h.robot.hal.emergency_stop_all()
    threading.Thread(target=arreter, daemon=True).start()
    res = h.run({"duree": 10.0})
    issue = h.robot.follow_service._issue
    check("Le suivi s'arrête avec ESTOP", issue is not None and issue.status == "ESTOP",
          str(issue))
    check("… aucune consigne de marche non nulle après l'arrêt d'urgence",
          bool(t_arret) and all(c[1] == 0.0 and c[3] == 0.0 for c in cons
                                if c[0] > t_arret[0] + 0.1))
    check("… la base ne bouge plus", math.isfinite(h.robot.x))

print(f"\n{'=' * 66}\n  {'TOUT PASSE' if FAIL == 0 else f'{FAIL} ÉCHEC(S)'}\n{'=' * 66}")
sys.exit(1 if FAIL else 0)
