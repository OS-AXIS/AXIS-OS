"""
Suite S2 — normalisation contre la physique réelle. VERSION CORRIGÉE.

La première version (voir historique git) avait trois défauts de
méthodologie découverts en exécution réelle, PAS des régressions
d'AXIS-OS :
  1. Le pont n'appliquait jamais le couple à Isaac Sim (ArticulationAction
     vide en mode TORQUE) — corrigé dans bridge.py (apply_command accepte
     maintenant `effort`).
  2. Le test commandait une position ABSOLUE (0.2) plutôt qu'un DELTA
     relatif à la position courante — sur un G1 sans gains PD ni tenue,
     ça déclenchait le rejet légitime de position_delta_max (C5) plutôt
     que de tester un vrai mouvement.
  3. Base flottante (fix_base=False) : le G1 s'effondre sous la gravité
     entre l'import et la première commande, faute de couple de maintien
     — positions de départ incohérentes d'un joint à l'autre.

Cette version : fix_base=True (isole la convention de signe de la
dynamique corps entier — même principe que "robot suspendu" pour la
mise sous tension réelle), deltas relatifs, couple réellement appliqué.

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

DELTA = 0.15       # rad, delta RELATIF — sous position_delta_max pour tous les joints
SETTLE_STEPS = 40  # pas de physique laissés pour que le mouvement se stabilise


def main():
    simulation_app = start_simulation_app(headless=True)
    bridge = IsaacSimBridge()
    # fix_base=True : isole la convention de signe de la dynamique corps
    # entier. On teste "est-ce que +delta normalisé bouge dans le bon
    # sens", pas "est-ce que le G1 tient debout" — ce dernier point est
    # couvert par les suites S4/S5, avec fix_base=False.
    bridge.setup(fix_base=True)
    hal = bridge.build_hal()
    hal.initialize_all()

    results = {"S2.1": {}, "S2.2": {}}

    print(f"\n=== S2.1 — direction en mode POSITION (delta relatif +{DELTA} rad) ===")
    for joint_id in bridge._joint_index:
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

    print(f"\n=== S2.2 — direction en mode TORQUE (+10 Nm normalisé) ===")
    for joint_id in bridge._joint_index:
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
        # Neutraliser le couple avant le joint suivant pour ne pas
        # accumuler de rotation continue entre deux itérations.
        hal.send_joint_commands({joint_id: ActuatorCommand(mode=ControlMode.TORQUE, torque=0.0)})
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
        print("ÉCHEC — inspecter conventions.py pour ces joints précis avant de continuer.")
        simulation_app.close()
        sys.exit(1)
    print("S2 validée.")
    simulation_app.close()


if __name__ == "__main__":
    main()
