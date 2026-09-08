"""
AXIS-OS — Tests URDFLoader
============================
Valide le parseur URDF générique (axis_os/hal/core/urdf_loader.py) :
parsing correct, filtrage fixed/floating, valeurs par défaut sur
limites partielles, et rejet strict des cas ambigus (joint actionné
sans <limit>, type inconnu, XML cassé, xacro non développé) — c'est
précisément ce dernier point qui aurait empêché, structurellement,
le type d'écart 24 vs 37 joints découvert cette session sur le G1.
"""

import sys
import tempfile
from pathlib import Path
sys.path.insert(0, ".")

from axis_os.hal.core.urdf_loader import load_urdf, URDFParseError, URDFRobotModel
from axis_os.hal.core.types import JointType, JointDefinition

FAIL = 0


def check(condition: bool, description: str) -> None:
    global FAIL
    status = "✓" if condition else "✗"
    print(f"  [{status}] {description}")
    if not condition:
        FAIL += 1


SIMPLE_ARM_URDF = """<?xml version="1.0"?>
<robot name="test_arm">
  <joint name="shoulder_pitch" type="revolute">
    <parent link="torso"/>
    <child link="upper_arm"/>
    <limit lower="-1.57" upper="1.57" velocity="10.0" effort="50.0"/>
  </joint>
  <joint name="elbow" type="revolute">
    <parent link="upper_arm"/>
    <child link="forearm"/>
    <limit lower="0.0" upper="2.5" effort="30.0"/>
  </joint>
  <joint name="wrist_slide" type="prismatic">
    <parent link="forearm"/>
    <child link="wrist"/>
    <limit lower="0.0" upper="0.1" velocity="0.5" effort="20.0"/>
  </joint>
  <joint name="continuous_wheel" type="continuous">
    <parent link="base"/>
    <child link="wheel"/>
    <limit lower="-3.14159" upper="3.14159" velocity="15.0" effort="10.0"/>
  </joint>
  <joint name="gripper_fixed" type="fixed">
    <parent link="forearm"/>
    <child link="gripper_base"/>
  </joint>
  <joint name="base_floating" type="floating">
    <parent link="world"/>
    <child link="torso"/>
  </joint>
</robot>"""


def test_basic_parsing():
    print("\n--- Parsing de base (chaîne XML en mémoire) ---")
    model = load_urdf(SIMPLE_ARM_URDF)
    check(isinstance(model, URDFRobotModel), "load_urdf() retourne un URDFRobotModel")
    check(model.name == "test_arm", "Nom du robot correctement extrait")
    check(len(model.joints) == 6, f"6 joints bruts trouvés (obtenu: {len(model.joints)})")


def test_actuated_filtering():
    print("\n--- Filtrage joints actionnés (fixed/floating exclus) ---")
    model = load_urdf(SIMPLE_ARM_URDF)
    joints = model.get_joints()
    check(len(joints) == 4, f"4 joints actionnés (fixed+floating exclus), obtenu: {len(joints)}")
    ids = {j.joint_id for j in joints}
    check("gripper_fixed" not in ids, "Joint fixed correctement exclu")
    check("base_floating" not in ids, "Joint floating correctement exclu")
    check("shoulder_pitch" in ids and "elbow" in ids, "Joints revolute présents")
    check("wrist_slide" in ids, "Joint prismatic présent")
    check("continuous_wheel" in ids, "Joint continuous mappé vers REVOLUTE")

    wheel = next(j for j in joints if j.joint_id == "continuous_wheel")
    check(wheel.joint_type == JointType.REVOLUTE, "continuous -> JointType.REVOLUTE")

    wrist = next(j for j in joints if j.joint_id == "wrist_slide")
    from axis_os.hal.core.types import JointType as JT
    check(wrist.joint_type == JT.PRISMATIC, "prismatic -> JointType.PRISMATIC")


def test_partial_limits_use_defaults():
    print("\n--- Limites partielles -> defaults appliqués sélectivement ---")
    model = load_urdf(SIMPLE_ARM_URDF)
    joints = {j.joint_id: j for j in model.get_joints(default_vel_max=99.0, default_torque_max=88.0)}

    shoulder = joints["shoulder_pitch"]
    check(shoulder.limits.vel_max == 10.0, "vel_max du fichier respectée quand présente (pas le default)")
    check(shoulder.limits.torque_max == 50.0, "torque_max du fichier respectée quand présente")

    elbow = joints["elbow"]
    check(elbow.limits.vel_max == 99.0, "vel_max manquante -> default appliqué")
    check(elbow.limits.torque_max == 30.0, "torque_max présente malgré vel_max absente -> pas écrasée")
    check(elbow.limits.pos_min == 0.0, "pos_min=0.0 explicite préservé (pas confondu avec 'absent')")


def test_reject_missing_limit_on_actuated_joint():
    print("\n--- Rejet strict : joint actionné sans <limit> ---")
    bad = """<?xml version="1.0"?>
<robot name="bad">
  <joint name="no_limits" type="revolute">
    <parent link="a"/><child link="b"/>
  </joint>
</robot>"""
    try:
        load_urdf(bad).get_joints()
        check(False, "Aurait dû lever URDFParseError")
    except URDFParseError:
        check(True, "URDFParseError levée pour joint sans limites")


def test_reject_unknown_joint_type():
    print("\n--- Rejet strict : type de joint non supporté ---")
    bad = """<?xml version="1.0"?>
<robot name="bad">
  <joint name="mystery" type="planar">
    <parent link="a"/><child link="b"/>
    <limit lower="0" upper="1"/>
  </joint>
</robot>"""
    try:
        load_urdf(bad).get_joints()
        check(False, "Aurait dû lever URDFParseError")
    except URDFParseError:
        check(True, "URDFParseError levée pour type 'planar' inconnu")


def test_reject_malformed_xml():
    print("\n--- Rejet strict : XML cassé ---")
    try:
        load_urdf("<robot><joint></robot>")
        check(False, "Aurait dû lever URDFParseError")
    except URDFParseError:
        check(True, "URDFParseError levée pour XML mal formé")


def test_reject_missing_parent_child():
    print("\n--- Rejet strict : <parent>/<child> manquants ---")
    bad = """<?xml version="1.0"?>
<robot name="bad">
  <joint name="orphan" type="revolute">
    <limit lower="0" upper="1"/>
  </joint>
</robot>"""
    try:
        load_urdf(bad)
        check(False, "Aurait dû lever URDFParseError")
    except URDFParseError:
        check(True, "URDFParseError levée pour parent/child absents")


def test_reject_undeveloped_xacro():
    print("\n--- Rejet strict : xacro non développé ---")
    bad_xacro = """<?xml version="1.0"?>
<robot name="x" xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:include filename="$(find my_pkg)/foo.xacro"/>
</robot>"""
    try:
        load_urdf(bad_xacro)
        check(False, "Aurait dû lever URDFParseError")
    except URDFParseError:
        check(True, "URDFParseError levée pour xacro non développé (macros/$(find) détectés)")


def test_load_from_file_path():
    print("\n--- Chargement depuis un fichier disque ---")
    with tempfile.TemporaryDirectory() as tmpdir:
        path = Path(tmpdir) / "robot.urdf"
        path.write_text(SIMPLE_ARM_URDF)
        model = load_urdf(path)
        check(model.name == "test_arm", "Chargement par Path() fonctionne")
        model2 = load_urdf(str(path))
        check(model2.name == "test_arm", "Chargement par chemin str() fonctionne")


def test_empty_urdf_rejected():
    print("\n--- Rejet strict : URDF sans aucun joint ---")
    empty = '<?xml version="1.0"?><robot name="empty"></robot>'
    try:
        load_urdf(empty)
        check(False, "Aurait dû lever URDFParseError (aucun joint)")
    except URDFParseError:
        check(True, "URDFParseError levée pour URDF sans joint")


def test_output_compatible_with_existing_hal_format():
    print("\n--- Compatibilité avec le format JointDefinition existant ---")
    model = load_urdf(SIMPLE_ARM_URDF)
    joints = model.get_joints()
    check(all(isinstance(j, JointDefinition) for j in joints),
          "Tous les éléments retournés sont bien des JointDefinition")
    from axis_os.hal.drivers.unitree_g1.robot import _build_g1_joint_definitions
    reference_fields = set(_build_g1_joint_definitions()[0].__dataclass_fields__.keys())
    parsed_fields = set(joints[0].__dataclass_fields__.keys())
    check(reference_fields == parsed_fields,
          "Mêmes champs que le format actuel du G1 codé à la main (drop-in compatible)")


if __name__ == "__main__":
    print("=" * 60)
    print("AXIS-OS — Tests URDFLoader")
    print("=" * 60)

    test_basic_parsing()
    test_actuated_filtering()
    test_partial_limits_use_defaults()
    test_reject_missing_limit_on_actuated_joint()
    test_reject_unknown_joint_type()
    test_reject_malformed_xml()
    test_reject_missing_parent_child()
    test_reject_undeveloped_xacro()
    test_load_from_file_path()
    test_empty_urdf_rejected()
    test_output_compatible_with_existing_hal_format()

    print()
    if FAIL == 0:
        print("✓ Tous les tests passés.")
    else:
        print(f"  ✗ {FAIL} test(s) échoué(s).")
    sys.exit(0 if FAIL == 0 else 1)
