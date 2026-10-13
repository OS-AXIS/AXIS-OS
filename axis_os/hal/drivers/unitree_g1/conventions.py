"""
AXIS-OS — Conventions d'articulation du Unitree G1
===================================================
Table des conventions d'angles bruts du Unitree G1 : pour chaque
articulation, le sens dans lequel un angle positif la fait bouger, et
son décalage d'origine.

Problème : sur le G1, les angles bruts des moteurs ne sont pas
symétriques entre la gauche et la droite. Par exemple :
  left_knee  : angle brut > 0 = flexion (intuitif)
  right_knee : angle brut < 0 = flexion (contre-intuitif)
Sans correction, une même consigne plierait un genou et tendrait
l'autre.

Solution : sign = -1 sur les articulations inversées, pour que le reste
d'AXIS-OS travaille en angles « normalisés », de même sens des deux
côtés (raw : angle brut du moteur ; offset : décalage d'origine) :
  normalized = (raw - offset) × sign
  raw        = (normalized × sign) + offset

Articulations inversées (9) : right_hip_roll/yaw/pitch, right_knee,
  right_ankle_pitch/roll, right_shoulder_roll/yaw, right_wrist_roll
(roll, yaw et pitch désignent les rotations de roulis, de lacet et de
tangage).

Correction du 2026-09-08 : cette table utilise désormais les identifiants
officiels du fichier URDF de Unitree, g1_23dof_rev_1_0.urdf (un URDF,
Unified Robot Description Format, est le fichier XML standard qui décrit
les articulations d'un robot ; source : github.com/unitreerobotics/
unitree_ros, licence BSD 3-Clause, voir
axis_os/hal/drivers/unitree_g1/urdf/README.md). Ils sont tous suffixés
"_joint" pour correspondre exactement à ce fichier, au lieu des noms
approximatifs devinés dans une version antérieure.

Ce qui change concrètement par rapport à l'ancienne table :
  - Tous les identifiants gagnent le suffixe "_joint" (par exemple,
    "left_knee" devient "left_knee_joint") pour correspondre au fichier
    officiel.
  - "waist_roll" est supprimé : la variante officielle 23 DOF (23 degrés
    de liberté, c'est-à-dire 23 articulations motorisées) du G1 n'a
    qu'une seule articulation de taille, waist_yaw_joint. L'ancienne
    table en supposait deux (waist_yaw et waist_roll), ce qui ne
    correspond à aucune variante officielle à 23 DOF. La variante
    29 DOF, elle, a bien trois articulations de taille (waist_yaw,
    waist_roll et waist_pitch) ; elle dispose désormais de sa propre
    table, G1_29DOF_JOINT_CONVENTIONS, plus bas dans ce fichier (voir
    aussi urdf/README.md).
  - "left_elbow" et "right_elbow" restent une seule articulation chacun
    (pas de séparation pitch/roll). C'était en fait déjà correct dans
    l'ancienne table : l'écart supposé sur le coude, noté auparavant
    comme « non corrigé », ne correspondait en réalité à aucune variante
    officielle du G1. C'est IsaacSimBackend.ISAAC_G1_JOINT_MAP
    (backend.py) qui avait tort sur ce point précis, pas cette table ;
    à corriger séparément dans backend.py.
  - Tous les signes (+1.0/-1.0) et toutes les étiquettes anatomiques
    sont conservés à l'identique : la comparaison articulation par
    articulation (22 des 23 se correspondent exactement en anatomie,
    seul le suffixe "_joint" change) ne donne aucune raison de les
    revoir. Seule la clé (l'identifiant) change.

Licence : Apache 2.0 (open-source)
"""

from __future__ import annotations
from dataclasses import dataclass


# Convention d'une articulation : sign (+1.0 ou -1.0), offset_rad
# (décalage d'origine, en radians), positive_label et negative_label (le
# mouvement anatomique obtenu par un angle normalisé positif ou négatif),
# anatomical_note (remarque libre ; « INVERSION G1 » marque les
# articulations dont le sens brut est inversé).
@dataclass(frozen=True)
class JointConvention:
    joint_id:        str
    sign:            float
    offset_rad:      float
    positive_label:  str
    negative_label:  str
    anatomical_note: str
    # verifiee : sens confirmé par la suite de validation S2, c'est-à-dire
    # par une mesure en simulation et non par un raisonnement (voir
    # docs/campagne_validation_isaac.md).
    # Faux par défaut depuis un audit de sécurité du 2026-10-10.
    # Pourquoi : une entrée ajoutée à la main sans mesure ne doit pas
    # pouvoir passer pour vérifiée. C'est au contraire « vérifiée » qui
    # doit être écrit, entrée par entrée, comme pour les 23 articulations
    # sur 23 du 23 DOF ci-dessous. Une entrée non vérifiée porte un signe
    # provisoire : le HAL (couche d'abstraction matérielle, voir robot.py)
    # la tolère en simulation, avec un avertissement, et la refuse sur un
    # vrai robot.
    # On ne commande jamais un moteur réel dont on ignore le sens.
    verifiee:        bool = False


# Table du G1 23 DOF : ses 23 articulations, toutes vérifiées par la
# suite S2.
G1_JOINT_CONVENTIONS: dict[str, JointConvention] = {

    # ── Jambe gauche ─────────────────────────────────────────────
    "left_hip_roll_joint":    JointConvention("left_hip_roll_joint",    +1.0, 0.0, "abduction",          "adduction",           "Naturel", verifiee=True),
    "left_hip_yaw_joint":     JointConvention("left_hip_yaw_joint",     +1.0, 0.0, "rotation_interne",    "rotation_externe",    "Naturel", verifiee=True),
    "left_hip_pitch_joint":   JointConvention("left_hip_pitch_joint",   +1.0, 0.0, "flexion",             "extension",           "Naturel", verifiee=True),
    "left_knee_joint":        JointConvention("left_knee_joint",        +1.0, 0.0, "flexion",             "extension",           "Naturel", verifiee=True),
    "left_ankle_pitch_joint": JointConvention("left_ankle_pitch_joint", +1.0, 0.0, "dorsiflexion",        "flexion_plantaire",   "Naturel", verifiee=True),
    "left_ankle_roll_joint":  JointConvention("left_ankle_roll_joint",  +1.0, 0.0, "eversion",            "inversion",           "Naturel", verifiee=True),

    # ── Jambe droite — inversions ─────────────────────────────────
    "right_hip_roll_joint":    JointConvention("right_hip_roll_joint",    -1.0, 0.0, "abduction",         "adduction",           "INVERSION G1", verifiee=True),
    "right_hip_yaw_joint":     JointConvention("right_hip_yaw_joint",     -1.0, 0.0, "rotation_interne",  "rotation_externe",    "INVERSION G1", verifiee=True),
    "right_hip_pitch_joint":   JointConvention("right_hip_pitch_joint",   -1.0, 0.0, "flexion",           "extension",           "INVERSION G1 CRITIQUE", verifiee=True),
    "right_knee_joint":        JointConvention("right_knee_joint",        -1.0, 0.0, "flexion",           "extension",           "INVERSION G1 — bug historique", verifiee=True),
    "right_ankle_pitch_joint": JointConvention("right_ankle_pitch_joint", -1.0, 0.0, "dorsiflexion",      "flexion_plantaire",   "INVERSION G1", verifiee=True),
    "right_ankle_roll_joint":  JointConvention("right_ankle_roll_joint",  -1.0, 0.0, "eversion",          "inversion",           "INVERSION G1", verifiee=True),

    # ── Torse — une seule articulation dans la variante officielle 23 DOF
    "waist_yaw_joint": JointConvention("waist_yaw_joint", +1.0, 0.0, "rotation_gauche", "rotation_droite", "Naturel", verifiee=True),

    # ── Bras gauche ───────────────────────────────────────────────
    "left_shoulder_pitch_joint": JointConvention("left_shoulder_pitch_joint", +1.0, 0.0, "flexion_anterieure", "extension",         "Naturel", verifiee=True),
    "left_shoulder_roll_joint":  JointConvention("left_shoulder_roll_joint",  +1.0, 0.0, "abduction",          "adduction",         "Naturel", verifiee=True),
    "left_shoulder_yaw_joint":   JointConvention("left_shoulder_yaw_joint",   +1.0, 0.0, "rotation_externe",   "rotation_interne",  "Naturel", verifiee=True),
    "left_elbow_joint":          JointConvention("left_elbow_joint",          +1.0, 0.0, "flexion",            "extension",         "Naturel", verifiee=True),
    "left_wrist_roll_joint":     JointConvention("left_wrist_roll_joint",     +1.0, 0.0, "pronation",          "supination",        "Naturel", verifiee=True),

    # ── Bras droit — inversions partielles ───────────────────────
    "right_shoulder_pitch_joint": JointConvention("right_shoulder_pitch_joint", +1.0, 0.0, "flexion_anterieure", "extension",        "Même convention gauche", verifiee=True),
    "right_shoulder_roll_joint":  JointConvention("right_shoulder_roll_joint",  -1.0, 0.0, "abduction",          "adduction",        "INVERSION G1", verifiee=True),
    "right_shoulder_yaw_joint":   JointConvention("right_shoulder_yaw_joint",   -1.0, 0.0, "rotation_externe",   "rotation_interne", "INVERSION G1", verifiee=True),
    "right_elbow_joint":          JointConvention("right_elbow_joint",          +1.0, 0.0, "flexion",            "extension",        "Naturel", verifiee=True),
    "right_wrist_roll_joint":     JointConvention("right_wrist_roll_joint",     -1.0, 0.0, "pronation",          "supination",       "INVERSION G1", verifiee=True),
}


def get_convention(joint_id: str) -> JointConvention:
    """Convention d'une articulation du G1 23 DOF ; lève KeyError si
    l'identifiant est inconnu."""
    if joint_id not in G1_JOINT_CONVENTIONS:
        raise KeyError(f"Joint '{joint_id}' inconnu dans les conventions G1.")
    return G1_JOINT_CONVENTIONS[joint_id]


def get_inverted_joints() -> list[str]:
    """Identifiants des articulations du 23 DOF dont le sens brut est
    inversé (sign = -1.0)."""
    return [jid for jid, c in G1_JOINT_CONVENTIONS.items() if c.sign == -1.0]


# ═════════════════════════════════════════════════════════════════════
#  G1 29 DOF
# ═════════════════════════════════════════════════════════════════════
#
# 23 articulations sont communes avec le 23 DOF. D'après la comparaison
# des deux URDF officiels (unitree_ros), 20 sont identiques en tout point
# (axe, origine, limites, liens) ; les 3 autres (lacet de la taille,
# roulis des poignets) ne diffèrent que par leur position dans la chaîne
# cinématique, parce que le 29 DOF intercale de nouvelles articulations
# derrière elles. Leurs axes et limites étant identiques, leur convention
# se transmet. À reconfirmer par la suite S2 sur le modèle 29 DOF, mais
# sans raison d'en douter.
#
# 6 articulations sont nouvelles, et personne n'a jamais vérifié leur
# sens. Leur signe est donc provisoire : celui de l'URDF brut (+1), avec
# des étiquettes qui ne prétendent à aucun sens anatomique. Les nommer
# « flexion » ou « inclinaison à gauche » sans mesure serait inventer :
# c'est exactement l'erreur qui avait laissé 9 articulations inversées
# sur le 23 DOF.

def _non_verifiee(joint_id: str, axe: str) -> JointConvention:
    """Convention provisoire d'une articulation nouvelle du 29 DOF : signe
    brut de l'URDF (+1), étiquettes neutres tirées de l'axe de rotation
    (x, y ou z) et verifiee=False."""
    return JointConvention(
        joint_id, +1.0, 0.0,
        f"sens_positif_urdf_{axe}", f"sens_negatif_urdf_{axe}",
        "NON VÉRIFIÉE — signe brut de l'URDF, sens anatomique inconnu. "
        "À établir par la suite S2 sur le G1 29 DOF.",
        verifiee=False,
    )


# Les six articulations propres au 29 DOF, dont le sens reste à vérifier.
G1_29DOF_NOUVELLES = (
    "waist_roll_joint", "waist_pitch_joint",
    "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_wrist_pitch_joint", "right_wrist_yaw_joint",
)

G1_29DOF_JOINT_CONVENTIONS: dict[str, JointConvention] = {
    **G1_JOINT_CONVENTIONS,
    "waist_roll_joint":        _non_verifiee("waist_roll_joint", "x"),
    "waist_pitch_joint":       _non_verifiee("waist_pitch_joint", "y"),
    "left_wrist_pitch_joint":  _non_verifiee("left_wrist_pitch_joint", "y"),
    "left_wrist_yaw_joint":    _non_verifiee("left_wrist_yaw_joint", "z"),
    "right_wrist_pitch_joint": _non_verifiee("right_wrist_pitch_joint", "y"),
    "right_wrist_yaw_joint":   _non_verifiee("right_wrist_yaw_joint", "z"),
}


def conventions_pour(variante: str) -> dict[str, JointConvention]:
    """Table de conventions d'une variante du G1 ("23dof" ou "29dof") ;
    lève ValueError pour toute autre valeur."""
    if variante == "23dof":
        return G1_JOINT_CONVENTIONS
    if variante == "29dof":
        return G1_29DOF_JOINT_CONVENTIONS
    raise ValueError(f"variante G1 inconnue : {variante!r} (23dof ou 29dof)")
