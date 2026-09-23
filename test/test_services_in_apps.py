"""
Accès des apps aux services : les permissions sont-elles APPLIQUÉES ?

Une permission déclarée mais non vérifiée est pire qu'absente : elle
donne l'illusion d'un contrôle. Ce test vérifie qu'une app sans la
permission ne peut PAS marcher ni voir.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.core.types import Vector3
from axis_os.services.base import ServiceResult
from axis_os.services.perception.service import Detection
from axis_os.services.speech.service import Intent
from axis_os.store.sdk.app_base import PermissionError as SandboxPermissionError
from axis_os.store.sdk.manifest import (
    AppCategory, AppManifest, Permission, ProtectionLevel,
    get_protection_level,
)

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


class FakePerception:
    def __init__(self):
        self.calls = 0

    def find(self, query=None):
        self.calls += 1
        return ServiceResult(ok=True, value=[
            Detection("a box", 0.3, (1, 2, 3, 4),
                      direction=Vector3(1, 0, 0),
                      position_3d=Vector3(1.3, 0, 0.3))])

    def describe_scene(self):
        return ServiceResult(ok=True, value="a box")


class FakeLocomotion:
    def __init__(self):
        self.velocities = []

    def set_velocity(self, vx=0.0, vy=0.0, wz=0.0):
        self.velocities.append((vx, vy, wz))
        return ServiceResult(ok=True, value=(vx, vy, wz))

    def halt(self):
        return self.set_velocity(0.0, 0.0, 0.0)


class FakeSpeech:
    def __init__(self):
        self.ecoutes = 0
        from axis_os.services.speech.service import IntentRegistry
        self.registry = IntentRegistry()

    def listen(self, duration_s=3.0):
        self.ecoutes += 1
        return ServiceResult(ok=True, value=Intent(
            "aller_vers_colis", {"cible": "a cardboard box"}, 1.0,
            "va vers le carton"))


def make_ctx(perms):
    from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL
    from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
    from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer
    from axis_os.hal.watchdog.system import WatchdogSystem
    from axis_os.store.sdk.app_base import RobotContext
    import contextlib, io

    with contextlib.redirect_stdout(io.StringIO()):
        hal = UnitreeG1HAL(backend="sim"); hal.initialize_all()
        robot = NormalizedRobotHAL(hal, build_g1_normalizer()); robot.initialize_all()
        wd = WatchdogSystem(robot)
    man = AppManifest(
        app_id="com.test.services", name="T", version="1.0.0", author="t",
        author_email="t@t.t", description="t", category=AppCategory.RESEARCH,
        permissions=list(perms))
    ctx = RobotContext.for_testing(robot, wd, man)
    p, l, sp = FakePerception(), FakeLocomotion(), FakeSpeech()
    ctx._attach_services(perception=p, locomotion=l, speech=sp)
    ctx._fake_speech = sp
    return ctx, p, l


section("1 — Niveaux de protection")

check("LOCOMOTION_WALK est DANGEROUS : l'utilisateur doit accepter qu'un "
      "robot se déplace seul chez lui",
      get_protection_level(Permission.LOCOMOTION_WALK)
      is ProtectionLevel.DANGEROUS)
check("SENSORS_CAMERA est NORMAL : voir n'est pas agir",
      get_protection_level(Permission.SENSORS_CAMERA)
      is ProtectionLevel.NORMAL)

section("2 — Sans permission, l'app ne peut PAS")

ctx, p, l = make_ctx([])
for nom, appel in [("listen", lambda: ctx.listen()),
                   ("declare_intent", lambda: ctx.declare_intent("x", [["y"]])),
                   ("look_for", lambda: ctx.look_for("a box")),
                   ("describe_scene", lambda: ctx.describe_scene()),
                   ("walk", lambda: ctx.walk(0.5)),
                   ("stop_walking", lambda: ctx.stop_walking())]:
    try:
        appel()
        ok = False
    except SandboxPermissionError:
        ok = True
    check(f"{nom}() sans permission : refusé", ok)

check("Le service de locomotion n'a JAMAIS été appelé", l.velocities == [])
check("Le service de perception n'a JAMAIS été appelé", p.calls == 0)
check("Le service de parole n'a JAMAIS été appelé",
      ctx._fake_speech.ecoutes == 0)

section("3 — Marcher exige les DEUX permissions")

ctx, p, l = make_ctx([Permission.LOCOMOTION_WALK])
try:
    ctx.walk(0.5)
    ok = False
except SandboxPermissionError:
    ok = True
check("LOCOMOTION_WALK seule ne suffit pas : ACTUATORS_WRITE_LEGS "
      "est aussi exigée (le robot bouge bien ses jambes)", ok)

ctx, p, l = make_ctx([Permission.ACTUATORS_WRITE_LEGS])
try:
    ctx.walk(0.5)
    ok = False
except SandboxPermissionError:
    ok = True
check("ACTUATORS_WRITE_LEGS seule ne suffit pas non plus : se DÉPLACER "
      "est un risque distinct de bouger une jambe", ok)

section("4 — Avec les permissions, tout fonctionne")

ctx, p, l = make_ctx([Permission.SENSORS_CAMERA, Permission.LOCOMOTION_WALK,
                      Permission.ACTUATORS_WRITE_LEGS])
dets = ctx.look_for("a box")
check("look_for() rend des Detection", len(dets) == 1
      and isinstance(dets[0], Detection))
check("La position 3D est accessible en attribut (det.position_3d.x)",
      abs(dets[0].position_3d.x - 1.3) < 1e-9)
check("describe_scene() répond", ctx.describe_scene() == "a box")
check("walk() transmet la consigne", ctx.walk(0.5, 0.0, 0.2)
      and l.velocities[-1] == (0.5, 0.0, 0.2))
check("stop_walking() demande une vitesse NULLE, sans couper les moteurs",
      ctx.stop_walking() and l.velocities[-1] == (0.0, 0.0, 0.0))

section("4b — Parole : une intention, jamais un mouvement")

ctx, p, l = make_ctx([Permission.SENSORS_MICROPHONE])
it = ctx.listen()
check("listen() rend un Intent", isinstance(it, Intent))
check("L'intention porte un NOM et des paramètres, pas des angles",
      it.name == "aller_vers_colis"
      and it.params.get("cible") == "a cardboard box")
check("Aucun champ de l'intention ne peut contenir une commande moteur",
      set(vars(it)) <= {"name", "params", "confidence", "utterance"})
check("declare_intent() enregistre le vocabulaire de l'app",
      ctx.declare_intent("trier", [["trie", "range"], ["colis"]]))
check("La même intention déclarée deux fois est refusée, sans planter",
      ctx.declare_intent("trier", [["trie"], ["colis"]]) is False)

section("5 — Robot sans le service : réponse claire, pas d'erreur obscure")

ctx, _, _ = make_ctx([Permission.SENSORS_CAMERA, Permission.LOCOMOTION_WALK,
                      Permission.ACTUATORS_WRITE_LEGS])
ctx._attach_services(perception=None, locomotion=None, speech=None)
check("look_for() sur un robot sans caméra : liste vide", ctx.look_for("x") == [])
check("walk() sur un robot sans jambes : False, pas d'exception",
      ctx.walk(0.5) is False)
check("describe_scene() le dit en clair",
      "aucune perception" in ctx.describe_scene())
ctx2, _, _ = make_ctx([Permission.SENSORS_MICROPHONE])
ctx2._attach_services(perception=None, locomotion=None, speech=None)
check("listen() sur un robot sans micro : None, pas d'exception",
      ctx2.listen() is None)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
