# Meshes du G1 (non versionnés)

Les fichiers `.STL` référencés par `g1_23dof_rev_1_0.urdf` et
`g1_29dof_rev_1_0.urdf` ne sont volontairement PAS commités dans ce
repo (106MB, trop volumineux pour Git).

## Récupération

```bash
cd /tmp
git clone --depth 1 --filter=blob:none --sparse https://github.com/unitreerobotics/unitree_ros.git
cd unitree_ros
git sparse-checkout set robots/g1_description/meshes
cp robots/g1_description/meshes/*.STL <chemin_vers_ce_dossier>/
```

Source officielle : github.com/unitreerobotics/unitree_ros,
licence BSD 3-Clause (voir LICENSE_UNITREE_ROS.txt dans le dossier
parent).

Uniquement nécessaire pour un import physique COMPLET dans Isaac
Sim (`URDFParseAndImportFile`) — pas requis pour le simple parsing
des joints/limites via `URDFParseFile` (utilisé par
`axis_os/hal/core/urdf_loader.py`), qui fonctionne sans ces
fichiers.
