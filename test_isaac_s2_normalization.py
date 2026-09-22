"""
Suite S2 — normalisation contre la physique réelle. VERSION 3.

Historique des deux versions précédentes (voir git log) :
  v1 : position absolue au lieu de delta relatif, mode TORQUE jamais
       appliqué (bug du pont), base flottante sans tenue -> robot
       effondré. Aucune conclusion valable.
  v2 : delta relatif + couple réellement appliqué + fix_base=True.
       Toujours invalide : AUCUN gain PD sur les moteurs (l'URDF ne
       fournit ni stiffness ni damping — warning Isaac Sim confirmé
       sur les 23 joints à chaque import). Sans gain, une commande
       POSITION ne tient rien : chaque joint devient un pendule libre
       sous la gravité pendant tout le reste du test. 18/23
       "incohérences" observées, mais quasi certainement du bruit
       gravitationnel (la plupart des essais couple finissaient près
       des butées mécaniques, signature d'un bras qui tombe en butée,
       pas d'un couple commandé).

v3 (celle-ci) : gains de maintien explicites (set_holding_gains) pour
S2.1, gains à zéro pour S2.2 (set_pure_torque_mode), et reset_joint()
entre CHAQUE essai pour empêcher toute dérive de s'accumuler d'un
joint au suivant. Chaque joint est testé depuis une pose de référence
à 0.0 rad, isolément.

À exécuter DANS le conteneur Isaac Sim :
  docker exec isim-isaac-sim-1 /isaac-sim/python.sh /isaac-sim/axis_os/test_isaac_s2_normalization.py
"""
import sys
sys.path.insert(0, "/isaac-sim")

from axis_os.hal.drivers.isaac_sim.bridge import IsaacSimBridge, start_simulation_app
from axis_os.hal.core.types import ActuatorCommand, ControlMode

INVERTED_JOINTS = {  # sign=-1 dans conventions.py — candidats au piège
    "right_hip_roll_joint", "right_hip_yaw_joint", "right_hip_pitch_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_wrist_roll_joint",
}

DELTA = 0.15
SETTLE_STEPS = 40


def main():
    simulation_app = start_simulation_app(headless=True)
    bridge = IsaacSimBridge()
    # SUSPENDU : pieds 15 cm au-dessus du sol. Posé au sol avec de
    # vraies collisions, le plancher bloquait les chevilles et faisait
    # échouer S2 pour une raison sans rapport avec les conventions.
    bridge.setup(fix_base=True, ground_clearance=0.15)
    hal = bridge.build_hal()
    hal.initialize_all()

    results = {"S2.1": {}, "S2.2": {}}

    print(f"\n=== S2.1 — direction en mode POSITION (delta +{DELTA} rad, "
          f"gains de maintien actifs) ===")
    bridge.set_holding_gains()
    for joint_id in bridge._joint_index:
        bridge.reset_joint(joint_id, 0.0)   # pose de référence connue, isolée
        bridge.step(10)                     # laisser les gains stabiliser le zéro
        before = bridge.read_position(joint_id)
        target = before + DELTA
        cmd = ActuatorCommand(mode=ControlMode.POSITION, position=target)
        status = hal.send_joint_commands({joint_id: cmd})
        bridge.step(SETTLE_STEPS)
        after = bridge.read_position(joint_id)
        rejected = status[joint_id].name != "OK"
        moved_positive = (after - before) > 0.01
        ok = moved_positive and not rejected
        results["S2.1"][joint_id] = ok
        flag = "INVERSÉ" if joint_id in INVERTED_JOINTS else ""
        tag = "REJETÉ" if rejected else ("OK" if ok else "ÉCHEC ***")
        print(f"  {joint_id:32} avant={before:+.3f} cible={target:+.3f} "
              f"après={after:+.3f}  {tag} {flag}")

    print(f"\n=== S2.2 — direction en mode TORQUE (+10 Nm, un seul joint "
          f"libéré à la fois, les 22 autres tenus) ===")
    bridge.set_holding_gains()   # tout tenu par défaut
    bridge.reset_all_joints(0.0)
    bridge.step(20)
    for joint_id in bridge._joint_index:
        bridge.hold_all_except(joint_id)  # SEUL ce joint passe en couple pur
        bridge.reset_joint(joint_id, 0.0)
        bridge.step(5)
        before = bridge.read_position(joint_id)
        cmd = ActuatorCommand(mode=ControlMode.TORQUE, torque=10.0)
        status = hal.send_joint_commands({joint_id: cmd})
        bridge.step(SETTLE_STEPS)
        after = bridge.read_position(joint_id)
        rejected = status[joint_id].name != "OK"
        moved_positive = (after - before) > 0.005
        results["S2.2"][joint_id] = moved_positive and not rejected
        flag = "INVERSÉ" if joint_id in INVERTED_JOINTS else ""
        tag = "REJETÉ" if rejected else ("mouvement +" if moved_positive else "mouvement - ***")
        print(f"  {joint_id:32} avant={before:+.3f} après={after:+.3f}  {tag} {flag}")
        # Neutraliser le couple ET remettre à zéro avant le joint suivant —
        # hold_all_except() du prochain tour le retiendra alors à 0.
        hal.send_joint_commands({joint_id: ActuatorCommand(mode=ControlMode.TORQUE, torque=0.0)})
        bridge.reset_joint(joint_id, 0.0)
        bridge.step(5)

    print("\n=== S2.2 vs S2.1 : cohérence de signe ===")
    fail = 0
    for joint_id in bridge._joint_index:
        s21, s22 = results["S2.1"][joint_id], results["S2.2"][joint_id]
        if s21 != s22:
            print(f"  *** INCOHÉRENCE {joint_id} : position OK={s21} mais couple OK={s22} ***")
            fail += 1

    n = len(bridge._joint_index)
    print(f"\n{n - fail}/{n} cohérents entre position et couple.")
    if fail:
        print("ÉCHEC — inspecter conventions.py pour ces joints précis.")
        simulation_app.close()
        sys.exit(1)
    print("S2 validée.")
    simulation_app.close()


if __name__ == "__main__":
    main()
