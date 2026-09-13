# Isaac Sim 4.5 — Patches nécessaires pour le mode headless

Isaac Sim 4.5.0.0 contient plusieurs bugs qui font planter (segfault)
tout script lancé en `headless=True` sur un serveur sans affichage
réel. Ce document liste les correctifs appliqués lors de la session
de validation du 2026-09-13, pour éviter de les redécouvrir un par un
à chaque nouvelle instance.

## Contexte

Objectif de cette session : valider en simulation la vraie chaîne
cinématique du Unitree G1 (23 joints, variante 23dof) face à ce
qu'AXIS-OS a codé, puis tester `JointSafetyGuard` avec les vraies
limites articulaires extraites du fichier URDF officiel Unitree.
**Objectif atteint** — voir résultat en fin de document.

## Configuration serveur utilisée

- Instance AWS `g6.2xlarge` (8 vCPU, 32GB RAM, GPU NVIDIA L4 24GB VRAM)
- AMI Deep Learning Base with Single CUDA (Ubuntu 24.04)
- Isaac Sim 4.5.0.0, venv Python 3.10 (via PPA deadsnakes)
- Variable requise à chaque session : `export OMNI_KIT_ACCEPT_EULA=YES`

## Dépendances système à installer avant tout

```bash
sudo apt install -y libnvidia-gl-595 libsm6 libxext6 libxrender1 libglu1-mesa
```

## Conflit driver Vulkan (GPU dupliqué)

Symptôme : `Failed to create any GPU devices` — le GPU apparaît deux
fois dans le diagnostic Isaac Sim (deux `nvidia_icd.json` présents).

```bash
sudo mv /usr/share/vulkan/icd.d/nvidia_icd.json /usr/share/vulkan/icd.d/nvidia_icd.json.disabled
```

## Patches Python (à réappliquer sur toute nouvelle installation)

Isaac Sim gère un cache d'extensions à PLUSIEURS emplacements
simultanément (`~/isaac_env/lib/python3.10/site-packages/omni/data/Kit/...`
ET `~/.local/share/ov/data/exts/v2/...`) — toujours identifier le VRAI
chemin depuis le traceback d'erreur avant de patcher, jamais supposer.

Chaque patch suit le même principe : insérer `return None` juste
après la signature de la fonction, pour empêcher la construction
d'éléments UI qui plantent sans affichage réel (même quand le code
est protégé par un `try/except` — un segfault C++ contourne le
try/except Python).

1. `isaacsim.asset.importer.urdf-.../scripts/ui/ui_utils.py`
   → fonction `string_filed_builder`

2. `isaacsim/exts/isaacsim.simulation_app/simulation_app.py`
   → fonctions `_prepare_ui()` et `_wait_for_viewport()`

3. `omni.kit.material.library-.../material_library.py`
   → fonction `_refresh_create_menu()`

4. `isaacsim.asset.importer.urdf-.../scripts/ui/style.py`
   → variable `EXTENSION_FOLDER_PATH`, ajouter un fallback en dur
     (`get_extension_path_by_module()` retourne parfois `None`,
     intermittent — a fonctionné une fois puis replanté)

5. `omni.kit.widget.viewport-.../widget.py`
   → fonction imbriquée `stage_changed_notice`

Après ces 5 patches : `SimulationApp({'headless': True})` démarre
proprement de bout en bout ("Simulation App Startup Complete"), et
`URDFParseFile()` (parsing seul, sans import physique dans une scène)
fonctionne parfaitement.

Segfault résiduel uniquement à la fermeture (`app.close()`) —
contourné avec `os._exit(0)` en fin de script plutôt que `.close()`.

## Limite connue, NON résolue

`URDFParseAndImportFile()` (import physique complet dans une scène,
nécessaire pour simuler la physique réelle — gravité, collisions,
mouvement) plante TOUJOURS avec un segfault natif dans
`self._urdf_interface.import_robot()` (`commands.py` ligne ~202),
même après tous les patches ci-dessus, même avec un `dest_path`
explicite (fichier USD réel plutôt que stage en mémoire), même avec
`create_physics_scene=False`. C'est un appel direct à du C++ compilé,
pas patchable en Python.

Confirmé comme bug connu de la communauté, pas spécifique à ce
projet : voir github.com/isaac-sim/IsaacSim/discussions/726 (même
symptôme exact rapporté par un autre utilisateur, avec un robot
différent). Pas de solution universelle documentée à ce jour.

**Ce qui reste possible malgré cette limite** : tout ce qui ne
nécessite QUE le parsing (joints, limites, structure) — largement
suffisant pour valider une chaîne de sécurité robotique sans avoir
besoin d'une vraie simulation physique visuelle.

## Résultat obtenu malgré cette limite

Avec uniquement `URDFParseFile()` (fonctionnel) :

- 23 joints actionnés (REVOLUTE) du G1 confirmés — noms et limites
  exactement conformes à `g1_23dof_rev_1_0.urdf`
- `right_knee_joint` : lower=-0.0873, upper=2.8798, effort=139.0 Nm
  — confirme la correction de sécurité du couple max (88 Nm codé en
  dur avant, 139 Nm réel)
- `JointSafetyGuard` testé avec les vraies limites extraites d'Isaac
  Sim sur les 23 joints individuellement : chaque joint rejette
  correctement une commande de position hors limites
- NaN/Infinity : diagnostic correct confirmé (`kind=INVALID_VALUE`,
  pas de faux `POSITION_LIMIT` trompeur)

Voir aussi `axis_os/hal/drivers/unitree_g1/urdf/meshes/README.md`
pour la récupération des fichiers `.STL` (nécessaires uniquement pour
une éventuelle reprise future de l'import physique complet).
