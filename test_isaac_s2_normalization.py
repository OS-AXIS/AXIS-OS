"""
Suite S2 — normalisation contre la physique réelle.
À exécuter DANS le conteneur Isaac Sim :
  docker exec isim-isaac-sim-1 /isaac-sim/python.sh /isaac-sim/axis_os/test_isaac_s2_normalization.py

Fait exactement ce que dit campagne_validation_isaac_axis_os.md §3 :
S2.1 (direction en position) et S2.2 (direction en couple), sur les 23
joints. C'est la suite qui aurait attrapé le bug de signe du couple sans
lire une ligne de code.
"""
import sys
sys.path.insert(0, "/isaac-sim")

from axis_os.hal.drivers.isaac_sim.bridge import IsaacSimBridge
from axis_os.hal.core.types import ActuatorCommand, ControlMode
from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer

INVERTED_JOINTS = {  # sign=-1 dans conventions.py — candidats au piège
    "right_hip_roll_joint", "right_hip_yaw_joint", "right_hip_pitch_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_wrist_roll_joint",
}


def main():
    bridge = IsaacSimBridge()
    bridge.setup()
    hal = bridge.build_hal()
    hal.initialize_all()
    normalizer = build_g1_normalizer()

    results = {"S2.1": [], "S2.2": []}

    print("\n=== S2.1 — direction en mode POSITION (+0.2 rad normalisé) ===")
    for joint_id in bridge._joint_index:
        before = bridge.read_feedback(joint_id).position
        cmd = ActuatorCommand(mode=ControlMode.POSITION, position=0.2)
        hal.send_joint_commands({joint_id: cmd})
        bridge.step(30)
        after = bridge.read_feedback(joint_id).position
        moved_positive = (after - before) > 0.01
        expected_positive = True  # +0.2 normalisé doit toujours donner +delta normalisé
        ok = moved_positive == expected_positive
        results["S2.1"].append((joint_id, ok, before, after))
        flag = "INVERSÉ" if joint_id in INVERTED_JOINTS else ""
        print(f"  {joint_id:32} avant={before:+.3f} après={after:+.3f} "
              f"{'OK' if ok else 'ÉCHEC ***'} {flag}")

    print("\n=== S2.2 — direction en mode TORQUE (+10 Nm normalisé) ===")
    print("    (relire les positions de départ, le robot a bougé au test précédent)")
    for joint_id in bridge._joint_index:
        before = bridge.read_feedback(joint_id).position
        cmd = ActuatorCommand(mode=ControlMode.TORQUE, torque=10.0)
        hal.send_joint_commands({joint_id: cmd})
        bridge.step(30)
        after = bridge.read_feedback(joint_id).position
        moved_positive = (after - before) > 0.005
        results["S2.2"].append((joint_id, moved_positive, before, after))
        flag = "INVERSÉ" if joint_id in INVERTED_JOINTS else ""
        print(f"  {joint_id:32} avant={before:+.3f} après={after:+.3f} "
              f"{'mouvement +' if moved_positive else 'mouvement - ***'} {flag}")

    print("\n=== S2.2 vs S2.1 : cohérence de signe ===")
    fail = 0
    s21 = {r[0]: r[1] for r in results["S2.1"]}
    s22 = {r[0]: r[1] for r in results["S2.2"]}
    for joint_id in bridge._joint_index:
        if s21[joint_id] != s22[joint_id]:
            print(f"  *** INCOHÉRENCE {joint_id} : position OK={s21[joint_id]} "
                  f"mais couple OK={s22[joint_id]} — même bug que C3, à corriger ***")
            fail += 1
    print(f"\n{23 - fail}/23 cohérents entre position et couple.")
    if fail:
        print("ÉCHEC — ne pas continuer vers S3 avant d'avoir corrigé.")
        sys.exit(1)
    print("S2 validée sur les 23 joints.")


if __name__ == "__main__":
    main()
