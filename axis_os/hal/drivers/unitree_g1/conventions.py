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

CORRIGÉ (2026-09-08) — cette table utilise désormais les IDs
officiels du fichier URDF Unitree (g1_23dof_rev_1_0.urdf, source :
github.com/unitreerobotics/unitree_ros, licence BSD 3-Clause — voir
axis_os/hal/drivers/unitree_g1/urdf/README.md), tous suffixés
"_joint" pour correspondre exactement à ce fichier, plutôt que les
noms approximatifs devinés lors de la session précédente.

Ce qui change concrètement par rapport à l'ancienne table :
  - Tous les IDs gagnent le suffixe "_joint" (ex: "left_knee" devient
    "left_knee_joint") pour correspondre au fichier officiel.
  - "waist_roll" est SUPPRIMÉ : la variante 23dof officielle du G1
    n'a qu'UN SEUL joint de taille (waist_yaw_joint) — l'ancienne
    table en supposait deux (waist_yaw + waist_roll), une hypothèse
    qui n'existe dans AUCUNE variante officielle à 23 DOF. (La
    variante 29dof, elle, a bien 3 joints de taille : waist_yaw +
    waist_roll + waist_pitch — non utilisée ici, voir urdf/README.md
    si un futur besoin justifie de migrer vers cette variante.)
  - "left_elbow"/"right_elbow" restent un SEUL joint chacun (pas de
    séparation pitch/roll) : c'était en fait déjà correct dans
    l'ancienne table — l'écart supposé sur le coude, documenté dans
    la session précédente comme "non corrigé", ne correspondait à
    AUCUNE variante officielle du G1 en réalité. C'est
    IsaacSimBackend.ISAAC_G1_JOINT_MAP (backend.py) qui avait tort
    sur ce point précis, pas cette table — à corriger dans backend.py
    séparément.
  - Tous les signes (+1.0/-1.0) et labels anatomiques sont conservés
    à l'identique de l'ancienne table : la comparaison joint par
    joint (22 des 23 joints se correspondent exactement en anatomie,
    seul le renommage "_joint" change) ne donne aucune raison de les
    revoir. Seule la clé (l'ID) change.

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
    "left_hip_roll_joint":    JointConvention("left_hip_roll_joint",    +1.0, 0.0, "abduction",          "adduction",           "Naturel"),
    "left_hip_yaw_joint":     JointConvention("left_hip_yaw_joint",     +1.0, 0.0, "rotation_interne",    "rotation_externe",    "Naturel"),
    "left_hip_pitch_joint":   JointConvention("left_hip_pitch_joint",   +1.0, 0.0, "flexion",             "extension",           "Naturel"),
    "left_knee_joint":        JointConvention("left_knee_joint",        +1.0, 0.0, "flexion",             "extension",           "Naturel"),
    "left_ankle_pitch_joint": JointConvention("left_ankle_pitch_joint", +1.0, 0.0, "dorsiflexion",        "flexion_plantaire",   "Naturel"),
    "left_ankle_roll_joint":  JointConvention("left_ankle_roll_joint",  +1.0, 0.0, "eversion",            "inversion",           "Naturel"),

    # ── Jambe droite — inversions ─────────────────────────────────
    "right_hip_roll_joint":    JointConvention("right_hip_roll_joint",    -1.0, 0.0, "abduction",         "adduction",           "INVERSION G1"),
    "right_hip_yaw_joint":     JointConvention("right_hip_yaw_joint",     -1.0, 0.0, "rotation_interne",  "rotation_externe",    "INVERSION G1"),
    "right_hip_pitch_joint":   JointConvention("right_hip_pitch_joint",   -1.0, 0.0, "flexion",           "extension",           "INVERSION G1 CRITIQUE"),
    "right_knee_joint":        JointConvention("right_knee_joint",        -1.0, 0.0, "flexion",           "extension",           "INVERSION G1 — bug historique"),
    "right_ankle_pitch_joint": JointConvention("right_ankle_pitch_joint", -1.0, 0.0, "dorsiflexion",      "flexion_plantaire",   "INVERSION G1"),
    "right_ankle_roll_joint":  JointConvention("right_ankle_roll_joint",  -1.0, 0.0, "eversion",          "inversion",           "INVERSION G1"),

    # ── Torse — un seul joint dans la variante 23dof officielle ───
    "waist_yaw_joint": JointConvention("waist_yaw_joint", +1.0, 0.0, "rotation_gauche", "rotation_droite", "Naturel"),

    # ── Bras gauche ───────────────────────────────────────────────
    "left_shoulder_pitch_joint": JointConvention("left_shoulder_pitch_joint", +1.0, 0.0, "flexion_anterieure", "extension",         "Naturel"),
    "left_shoulder_roll_joint":  JointConvention("left_shoulder_roll_joint",  +1.0, 0.0, "abduction",          "adduction",         "Naturel"),
    "left_shoulder_yaw_joint":   JointConvention("left_shoulder_yaw_joint",   +1.0, 0.0, "rotation_externe",   "rotation_interne",  "Naturel"),
    "left_elbow_joint":          JointConvention("left_elbow_joint",          +1.0, 0.0, "flexion",            "extension",         "Naturel"),
    "left_wrist_roll_joint":     JointConvention("left_wrist_roll_joint",     +1.0, 0.0, "pronation",          "supination",        "Naturel"),

    # ── Bras droit — inversions partielles ───────────────────────
    "right_shoulder_pitch_joint": JointConvention("right_shoulder_pitch_joint", +1.0, 0.0, "flexion_anterieure", "extension",        "Même convention gauche"),
    "right_shoulder_roll_joint":  JointConvention("right_shoulder_roll_joint",  -1.0, 0.0, "abduction",          "adduction",        "INVERSION G1"),
    "right_shoulder_yaw_joint":   JointConvention("right_shoulder_yaw_joint",   -1.0, 0.0, "rotation_externe",   "rotation_interne", "INVERSION G1"),
    "right_elbow_joint":          JointConvention("right_elbow_joint",          +1.0, 0.0, "flexion",            "extension",        "Naturel"),
    "right_wrist_roll_joint":     JointConvention("right_wrist_roll_joint",     -1.0, 0.0, "pronation",          "supination",       "INVERSION G1"),
}


def get_convention(joint_id: str) -> JointConvention:
    if joint_id not in G1_JOINT_CONVENTIONS:
        raise KeyError(f"Joint '{joint_id}' inconnu dans les conventions G1.")
    return G1_JOINT_CONVENTIONS[joint_id]


def get_inverted_joints() -> list[str]:
    return [jid for jid, c in G1_JOINT_CONVENTIONS.items() if c.sign == -1.0]
