# Isaac Lab + unitree_sim_isaaclab — état et point de blocage (2026-09-13)

**Statut : INSTALLATION RÉUSSIE, SIMULATION EN ÉCHEC.** Ce document
capture précisément où s'arrête la progression, avec toutes les
étapes de résolution déjà validées, pour reprendre directement à ce
point plutôt que de repartir de zéro.

## Objectif de ce chantier

Aller au-delà de la validation faite via `URDFParseFile()` (voir
`docs/isaac_sim_headless_patches.md`, qui reste pleinement acquise et
indépendante de ce document) : faire tourner une vraie simulation
physique du G1 dans Isaac Lab, pour un jour pouvoir y tester
`G1DDSActuator` comme s'il parlait à un vrai robot (le projet
[unitree_sim_isaaclab](https://github.com/unitreerobotics/unitree_sim_isaaclab)
adopte le même protocole DDS que le vrai matériel), et éventuellement
y valider un modèle VLA (Octo, GR00T...) avant tout déploiement réel.

## Ce qui fonctionne, confirmé, dans l'ordre

1. **Isaac Sim 4.5.0.0** — déjà acquis (voir
   `docs/isaac_sim_headless_patches.md`).

2. **Isaac Lab installé avec succès**, après résolution de 4
   problèmes de compatibilité en cascade :
   - `conda: command not found` → installer miniconda
     (`Miniconda3-latest-Linux-x86_64.sh`)
   - `Terms of Service have not been accepted` (nouvelle exigence
     Anaconda) →
     ```bash
     conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main
     conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r
     ```
   - `ModuleNotFoundError: No module named 'tomllib'` — Isaac Lab
     (branche `main`) importe `tomllib`, stdlib uniquement depuis
     Python 3.11, alors que `auto_setup_env.sh` configure Python
     3.10 pour Isaac Sim 4.5. Contourné en créant un module de
     redirection vers `tomli` :
     ```bash
     pip install tomli
     cat > <site-packages>/tomllib.py << 'EOF'
     from tomli import *
     from tomli import load, loads
     EOF
     ```
   - `ERROR: Package 'isaaclab' requires a different Python: 3.10.21 not in '>=3.12'`
     — la branche `main` d'Isaac Lab exige Python ≥3.12. **Cause
     racine identifiée** : `auto_setup_env.sh` contient une ligne
     `# git checkout "$ISAAC_LAB_COMMIT"` **commentée** (donc jamais
     exécutée) qui aurait dû figer Isaac Lab au bon commit pour
     Python 3.10. C'est un bug/oubli du script officiel Unitree, pas
     une erreur de notre part. Contourné manuellement :
     ```bash
     cd ~/IsaacLab
     git checkout 91ad4944f2b7fad29d52c04a5264a082bcaad71d
     ```
     (le commit suggéré, en commentaire, pour Isaac Sim 4.5 — les
     deux autres commits commentés dans le script,
     `v2.2.0` et `80094be3245aa5c8376a7464d29cb4412ea518f5`,
     correspondent respectivement à Isaac Sim 5.0 et 5.1, non testés)
   - `ModuleNotFoundError: No module named 'pkg_resources'` puis
     `No module named 'toml'` en cascade, dus à une version trop
     récente de `setuptools` (≥80) qui a retiré `pkg_resources`.
     Contourné en fixant une version antérieure :
     ```bash
     pip install "setuptools==70.0.0" toml
     pip install --editable . --no-build-isolation
     ```
   - Résultat final : `import isaaclab` fonctionne.

3. **`unitree_sdk2py` installé avec succès**, séparément (n'est PAS
   mentionné dans `requirements.txt` de `unitree_sim_isaaclab` ni
   installé par `auto_setup_env.sh` — vraie omission du projet
   officiel, à installer manuellement) :
   ```bash
   git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
   cd unitree_sdk2_python
   export CYCLONEDDS_HOME=~/unitree_sim_isaaclab/cyclonedds/install
   pip3 install -e .
   ```
   Résultat : `cyclonedds-0.10.2` (la bonne version exacte, pas de
   conflit), `unitree_sdk2py-1.0.1`. Les deux s'importent
   correctement.

4. **`teleimager`** (sous-module git de `unitree_sim_isaaclab`) —
   cloné via `git submodule update --init --recursive`, mais pas
   installé comme paquet par le script principal. Installé
   manuellement :
   ```bash
   cd teleimager && pip install -e . && cd ..
   ```

5. **Assets téléchargés avec succès** via `bash fetch_assets.sh`.

## Où ça bloque, précisément

Commande lancée :
```bash
export CYCLONEDDS_HOME=~/unitree_sim_isaaclab/cyclonedds/install
python sim_main.py --device cpu --enable_cameras \
    --task Isaac-PickPlace-Cylinder-G129-Dex1-Joint \
    --enable_dex1_dds --robot_type g129 --headless
```

Progression observée avant le crash : le pont DDS Unitree démarre
correctement (`MotionSwitcherClient`, canaux `lowcmd`/`lowstate`/`imu`
initialisés — confirme que `unitree_sdk2py` fonctionne bien dans ce
contexte). Le crash survient ENSUITE, dans la configuration de
l'environnement Isaac Lab lui-même :

```
KeyError: 'lin_vel'
  File ".../isaaclab/envs/mdp/observations.py", ...
  File ".../isaaclab/managers/observation_manager.py", ...
  File ".../isaaclab/envs/manager_based_env.py", line 1099, in ...
```

`base_lin_vel` / `imu_lin_acc` sont des fonctions **officielles**
d'Isaac Lab (confirmé via la documentation officielle et le code
source) — ce n'est donc pas un nom inventé par `unitree_sim_isaaclab`
qui aurait changé. L'hypothèse la plus probable : la tâche
`Isaac-PickPlace-Cylinder-G129-Dex1-Joint` (écrite par Unitree pour
une version d'Isaac Lab plus récente que celle qu'on a dû figer au
commit `91ad4944f`) suppose une structure de dictionnaire
d'observations légèrement différente de celle que ce commit fournit
réellement — un vrai conflit de version entre deux projets tiers
écrits à des moments différents, pas un bug de notre code.

## Piste non testée pour la prochaine fois

Les deux autres commits Isaac Lab suggérés (commentés) dans
`auto_setup_env.sh` correspondent à Isaac Sim 5.0/5.1, pas 4.5 :
- `ISAAC_LAB_COMMIT="v2.2.0"` (pour Isaac Sim 5.0)
- `ISAAC_LAB_COMMIT="80094be3245aa5c8376a7464d29cb4412ea518f5"` (5.1)

Reprendre en repartant sur Isaac Sim 5.x plutôt que 4.5 pourrait
donner une combinaison de versions réellement testée et cohérente
côté Unitree — mais c'est un chantier d'installation complet à
refaire depuis le début (nouveau `auto_setup_env.sh 5.1 <env>`), pas
une correction rapide sur l'installation actuelle.

## Condition de reprise

Cette piste (simulation physique complète + test d'un modèle VLA en
conditions proches du réel) est un vrai "nice to have" pour la
maturité du projet, mais n'est PAS bloquante pour la suite : la
validation de sécurité déjà acquise via `URDFParseFile()` (23 joints,
vraies limites, `JointSafetyGuard` testé) reste entièrement valide et
suffisante pour continuer le développement. À reprendre soit :
- en réessayant avec Isaac Sim 5.x (voir ci-dessus), soit
- une fois qu'un robot physique réel est disponible, ce qui rend
  cette étape de simulation optionnelle plutôt qu'obligatoire.

## Environnement de référence (pour reproduire exactement)

- Instance AWS `g6.2xlarge` (8 vCPU, 32GB RAM, GPU NVIDIA L4 24GB)
- Miniconda, environnement conda `unitree_sim_env` (Python 3.10)
- Isaac Sim 4.5.0.0, Isaac Lab au commit `91ad4944f2b7fad29d52c04a5264a082bcaad71d`
- `unitree_sdk2py` 1.0.1, `cyclonedds` 0.10.2
