# Fichiers URDF officiels — Unitree G1

Source : https://github.com/unitreerobotics/unitree_ros
(dossier `robots/g1_description/`)

Licence : BSD 3-Clause (voir `LICENSE_UNITREE_ROS.txt`) — copyright
Unitree Robotics (HangZhou YuShu Technology Co., Ltd.), 2016-2022.
Usage commercial autorisé sous réserve de conservation de la notice
de copyright, sans caution ni garantie implicite du fabricant.

## Fichiers inclus

- `g1_23dof_rev_1_0.urdf` — 23 joints actionnés : jambes complètes,
  waist_yaw SEUL (pas de roll/pitch en taille), bras avec coude
  simple et wrist_roll seul.
- `g1_29dof_rev_1_0.urdf` — 29 joints actionnés : jambes complètes,
  waist à 3 DOF (yaw+roll+pitch), bras avec coude simple et poignet
  à 3 DOF (roll+pitch+yaw).

D'autres variantes existent dans le repo source (avec mains Inspire,
taille verrouillée, modes 10-18...) — ces deux-là couvrent le cas
d'usage principal d'AXIS-OS à ce stade. À récupérer depuis la source
si un futur besoin (mains articulées, verrouillage de taille) l'exige.

## Ce que la découverte de ces fichiers a corrigé

Le code précédent (`_build_g1_joint_definitions()` dans robot.py,
et la correction faite dans `backend.py` pendant la session Isaac
Sim du 2026-09-06) avait DEVINÉ la structure du G1 par observation
dans le simulateur, sans jamais avoir accès au vrai fichier source.
Résultat : NI le codage initial (24 joints, coude en pitch+roll
séparés, waist en yaw+roll) NI la "correction" (23 joints) ne
correspondaient exactement à la vraie structure officielle — les
deux étaient des suppositions raisonnables mais fausses sur des
points précis (notamment : le coude du G1 est un SEUL joint, pas
deux ; la variante 23dof n'a qu'un seul joint de taille, pas deux).

Utiliser ces fichiers directement via `urdf_loader.py` élimine cette
classe d'erreur : la source de vérité est désormais le fichier du
fabricant, jamais une retranscription manuelle.
