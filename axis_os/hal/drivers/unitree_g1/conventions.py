"""
AXIS-OS — Unitree G1 Joint Conventions
=========================================
Table des conventions d'angles bruts du Unitree G1.

Problème : le G1 a des angles asymétriques gauche/droite.
  left_knee  : raw > 0 = flexion ✓
  right_knee : raw < 0 = flexion ✗ (contre-intuitif)

Solution : sign=-1 sur les joints inversés.
  normalized = (raw - offset) × sign
  raw        = (normalized × sign) + offset

Joints inversés (9) : right_hip_roll/yaw/pitch, right_knee,
  right_ankle_pitch/roll, right_shoulder_roll/yaw, right_wrist_roll

Licence : Apache 2.0 (open-source)
"""

from __future__ import annotations
from dataclasses import dataclass


@dataclass(frozen=True)
class JointConvention:
    joint_id:        str
    sign:            float
    offset_rad:      float
    positive_label:  str
    negative_label:  str
    anatomical_note: str


G1_JOINT_CONVENTIONS: dict[str, JointConvention] = {

    # ── Jambe gauche ─────────────────────────────────────────────
    "left_hip_roll":    JointConvention("left_hip_roll",    +1.0, 0.0, "abduction",          "adduction",           "Naturel"),
    "left_hip_yaw":     JointConvention("left_hip_yaw",     +1.0, 0.0, "rotation_interne",    "rotation_externe",    "Naturel"),
    "left_hip_pitch":   JointConvention("left_hip_pitch",   +1.0, 0.0, "flexion",             "extension",           "Naturel"),
    "left_knee":        JointConvention("left_knee",        +1.0, 0.0, "flexion",             "extension",           "Naturel"),
    "left_ankle_pitch": JointConvention("left_ankle_pitch", +1.0, 0.0, "dorsiflexion",        "flexion_plantaire",   "Naturel"),
    "left_ankle_roll":  JointConvention("left_ankle_roll",  +1.0, 0.0, "eversion",            "inversion",           "Naturel"),

    # ── Jambe droite — inversions ─────────────────────────────────
    "right_hip_roll":    JointConvention("right_hip_roll",    -1.0, 0.0, "abduction",         "adduction",           "INVERSION G1"),
    "right_hip_yaw":     JointConvention("right_hip_yaw",     -1.0, 0.0, "rotation_interne",  "rotation_externe",    "INVERSION G1"),
    "right_hip_pitch":   JointConvention("right_hip_pitch",   -1.0, 0.0, "flexion",           "extension",           "INVERSION G1 CRITIQUE"),
    "right_knee":        JointConvention("right_knee",        -1.0, 0.0, "flexion",           "extension",           "INVERSION G1 — bug historique"),
    "right_ankle_pitch": JointConvention("right_ankle_pitch", -1.0, 0.0, "dorsiflexion",      "flexion_plantaire",   "INVERSION G1"),
    "right_ankle_roll":  JointConvention("right_ankle_roll",  -1.0, 0.0, "eversion",          "inversion",           "INVERSION G1"),

    # ── Torse ─────────────────────────────────────────────────────
    "waist_yaw":  JointConvention("waist_yaw",  +1.0, 0.0, "rotation_gauche",    "rotation_droite",    "Naturel"),
    "waist_roll": JointConvention("waist_roll", +1.0, 0.0, "inclinaison_gauche", "inclinaison_droite", "Naturel"),

    # ── Bras gauche ───────────────────────────────────────────────
    "left_shoulder_pitch": JointConvention("left_shoulder_pitch", +1.0, 0.0, "flexion_anterieure", "extension",         "Naturel"),
    "left_shoulder_roll":  JointConvention("left_shoulder_roll",  +1.0, 0.0, "abduction",          "adduction",         "Naturel"),
    "left_shoulder_yaw":   JointConvention("left_shoulder_yaw",   +1.0, 0.0, "rotation_externe",   "rotation_interne",  "Naturel"),
    "left_elbow":          JointConvention("left_elbow",          +1.0, 0.0, "flexion",            "extension",         "Naturel"),
    "left_wrist_roll":     JointConvention("left_wrist_roll",     +1.0, 0.0, "pronation",          "supination",        "Naturel"),

    # ── Bras droit — inversions partielles ───────────────────────
    "right_shoulder_pitch": JointConvention("right_shoulder_pitch", +1.0, 0.0, "flexion_anterieure", "extension",        "Même convention gauche"),
    "right_shoulder_roll":  JointConvention("right_shoulder_roll",  -1.0, 0.0, "abduction",          "adduction",        "INVERSION G1"),
    "right_shoulder_yaw":   JointConvention("right_shoulder_yaw",   -1.0, 0.0, "rotation_externe",   "rotation_interne", "INVERSION G1"),
    "right_elbow":          JointConvention("right_elbow",          +1.0, 0.0, "flexion",            "extension",        "Naturel"),
    "right_wrist_roll":     JointConvention("right_wrist_roll",     -1.0, 0.0, "pronation",          "supination",       "INVERSION G1"),
}


def get_convention(joint_id: str) -> JointConvention:
    if joint_id not in G1_JOINT_CONVENTIONS:
        raise KeyError(f"Joint '{joint_id}' inconnu dans les conventions G1.")
    return G1_JOINT_CONVENTIONS[joint_id]


def get_inverted_joints() -> list[str]:
    return [jid for jid, c in G1_JOINT_CONVENTIONS.items() if c.sign == -1.0]
