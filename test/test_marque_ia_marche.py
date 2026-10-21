"""
Marque « IA » sur walk() et move_to_pose() (SECURITY.md, limite n° 2).

Vérifié ici :
  1. une vitesse proposée par un modèle et transmise par les actions du SDK
     (GR00TAction.walk, LocomotionAction.walk) porte la marque d'origine IA :
     sans ACTUATORS_AI_DRIVEN, l'OS la refuse et le service de locomotion
     n'est jamais appelé ; avec la permission, elle passe ;
  2. une cible proposée par un modèle et rejointe par VLAAction.move_to_pose
     ou LocomotionAction.move_to_pose : refusée sans la permission, avant
     tout mouvement ; acceptée avec ;
  3. même règle depuis le processus de l'app (pont JSON) ;
  4. une origine inconnue est refusée ; un appel sans marque reste permis,
     c'est la partie que l'OS ne peut pas détecter (documentée).
"""
import contextlib
import io
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer  # noqa: E402
from axis_os.hal.core.normalized_robot import NormalizedRobotHAL  # noqa: E402
from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL  # noqa: E402
from axis_os.hal.watchdog.system import WatchdogSystem  # noqa: E402
from axis_os.services.base import ServiceResult  # noqa: E402
from axis_os.store.runtime.play_store import (  # noqa: E402
    PlayStoreRuntime, accepter_tout_SIMULATION,
)
from axis_os.store.runtime.profil import Profil  # noqa: E402
from axis_os.store.sdk.ai.connector import VLAAction  # noqa: E402
from axis_os.store.sdk.ai.gr00t_connector import GR00TAction  # noqa: E402
from axis_os.store.sdk.ai.locomotion_connector import LocomotionAction  # noqa: E402
from axis_os.store.sdk.app_base import (  # noqa: E402
    MissionResult, PermissionError as SdkPermissionError, RobotApp, RobotContext,
)
from axis_os.store.sdk.manifest import AppCategory, AppManifest, Permission  # noqa: E402

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


def refuse(f, *a, **kw):
    """Vrai si l'appel lève PermissionError (celle du SDK)."""
    try:
        silencieux(f, *a, **kw)
        return False
    except SdkPermissionError:
        return True


class Locomotion:
    """Service de locomotion factice : note chaque vitesse reçue."""
    def __init__(self):
        self.vitesses = []

    def set_velocity(self, vx=0.0, vy=0.0, wz=0.0):
        self.vitesses.append((vx, vy, wz))
        return ServiceResult(ok=True, value=(vx, vy, wz))

    def halt(self):
        return self.set_velocity(0.0, 0.0, 0.0)


with contextlib.redirect_stdout(io.StringIO()):
    ROBOT = NormalizedRobotHAL(UnitreeG1HAL("g1_marque_ia"), build_g1_normalizer())
    ROBOT.initialize_all()
WD = WatchdogSystem(ROBOT)
COUDE = "left_elbow_joint"
MARCHE = [Permission.LOCOMOTION_WALK, Permission.ACTUATORS_WRITE_LEGS]
BRAS = [Permission.ACTUATORS_READ, Permission.ACTUATORS_WRITE_ARMS]


def manifeste(perms, app_id="com.test.marque_ia"):
    return AppManifest(app_id=app_id, name="IA", version="1.0.0", author="T",
                       author_email="t@t.com", description="d",
                       category=AppCategory.UTILITIES, permissions=list(perms))


def contexte(perms):
    ctx = RobotContext.for_testing(ROBOT, WD, manifeste(perms))
    loco = Locomotion()
    ctx._attach_services(locomotion=loco)
    return ctx, loco


def position_coude():
    return ROBOT.get_full_state()[COUDE].position


section("1. walk() : la vitesse proposée par un modèle porte la marque")
for nom, action in [("GR00TAction.walk", GR00TAction(base_velocity=(0.2, 0.0, 0.1))),
                    ("LocomotionAction.walk", LocomotionAction(base_velocity=(0.2, 0.0, 0.1)))]:
    ctx, loco = contexte(MARCHE)
    check(f"{nom} sans ACTUATORS_AI_DRIVEN : refusé", refuse(action.walk, ctx))
    check("… le service de locomotion n'a jamais été appelé", loco.vitesses == [])
    ctx, loco = contexte(MARCHE + [Permission.ACTUATORS_AI_DRIVEN])
    check(f"{nom} avec ACTUATORS_AI_DRIVEN : vitesse transmise",
          silencieux(action.walk, ctx) and loco.vitesses == [(0.2, 0.0, 0.1)], str(loco.vitesses))

section("2. move_to_pose() : la cible proposée par un modèle porte la marque")
for nom, action in [("VLAAction.move_to_pose", VLAAction(target_angles={COUDE: 0.12})),
                    ("LocomotionAction.move_to_pose",
                     LocomotionAction(target_positions={COUDE: 0.12}))]:
    ctx, _ = contexte(BRAS)
    avant = position_coude()
    check(f"{nom} sans ACTUATORS_AI_DRIVEN : refusé",
          refuse(action.move_to_pose, ctx, duration_s=0.2, step_s=0.05))
    check("… avant tout mouvement", position_coude() == avant)
    ctx, _ = contexte(BRAS + [Permission.ACTUATORS_AI_DRIVEN])
    check(f"{nom} avec ACTUATORS_AI_DRIVEN : cible rejointe",
          silencieux(action.move_to_pose, ctx, duration_s=0.2, step_s=0.05) is True)


class AppModele(RobotApp):
    """Fait passer la sortie d'un « modèle » par les actions du SDK, sans
    déclarer ACTUATORS_AI_DRIVEN. (Le processus de l'app ne reprend du
    script que les définitions : pas BRAS ni MARCHE.)"""
    PERMISSIONS = [Permission.ACTUATORS_READ, Permission.ACTUATORS_WRITE_ARMS,
                   Permission.LOCOMOTION_WALK, Permission.ACTUATORS_WRITE_LEGS]
    APP_ID = "com.test.modele_non_declare"

    def manifest(self):
        return manifeste(self.PERMISSIONS, self.APP_ID)

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        if params.get("geste") == "marche":
            ok = GR00TAction(base_velocity=(0.2, 0.0, 0.0)).walk(ctx)
        else:
            ok = VLAAction(target_angles={COUDE: 0.1}).move_to_pose(ctx, 0.2, 0.05)
        return MissionResult(success=bool(ok), mission_id="", app_id="", duration_s=0.0)


class AppModeleDeclare(AppModele):
    PERMISSIONS = AppModele.PERMISSIONS + [Permission.ACTUATORS_AI_DRIVEN]
    APP_ID = "com.test.modele_declare"


section("3. Depuis le processus de l'app (pont JSON)")
loco = Locomotion()
rt = silencieux(PlayStoreRuntime, ROBOT, WD, locomotion_service=loco, profil=Profil.SIMULATION,
                confirmer_permissions=accepter_tout_SIMULATION)
for app in (AppModele(), AppModeleDeclare()):
    silencieux(rt.install, app)
    silencieux(rt.start_app, app.APP_ID)
for geste in ("marche", "pose"):
    res = silencieux(rt.run_mission, AppModele.APP_ID, {"geste": geste})
    check(f"{geste} : refusé sans ACTUATORS_AI_DRIVEN",
          res is not None and not res.success and "Permission" in (res.error_message or ""),
          getattr(res, "error_message", None))
check("… aucune vitesse n'a atteint le service de locomotion", loco.vitesses == [], str(loco.vitesses))
for geste in ("marche", "pose"):
    res = silencieux(rt.run_mission, AppModeleDeclare.APP_ID, {"geste": geste})
    check(f"{geste} : accepté avec ACTUATORS_AI_DRIVEN", res is not None and res.success,
          getattr(res, "error_message", None))

section("4. Origine inconnue ; appel sans marque (limite documentée)")
ctx, loco = contexte(MARCHE)
try:
    silencieux(ctx.walk, 0.1, origine="humain")
    check("walk(origine inconnue) : refusé", False)
except ValueError:
    check("walk(origine inconnue) : refusé", True)
check("walk() sans marque, sans ACTUATORS_AI_DRIVEN : permis (l'OS ne sait pas d'où "
      "vient le nombre)", silencieux(ctx.walk, 0.1) and loco.vitesses == [(0.1, 0.0, 0.0)])

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
