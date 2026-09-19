# Isaac Lab + unitree_sim_isaaclab — état et point de blocage (2026-09-13)

**Statut : PERCÉE RÉELLE LE 2026-09-18 (voir tout en bas), sur une
approche DIFFÉRENTE de celle documentée ici.** Ce document capture
d'abord la tentative Isaac Lab + `unitree_sim_isaaclab` (2026-09-13,
toujours bloquée telle quelle), puis, en fin de fichier, une vraie
réussite obtenue le 2026-09-18 par un chemin plus simple : Isaac Sim
seul (pas Isaac Lab), via le conteneur Docker officiel NVIDIA, avec
notre propre `JointSafetyGuard` connecté en direct à un vrai G1
physiquement simulé.

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

---

## PERCÉE — Isaac Sim (sans Isaac Lab) via Docker officiel, JointSafetyGuard validé contre un vrai G1 physique (2026-09-18)

Chemin entièrement différent de la tentative ci-dessus — abandon
d'Isaac Lab et de `unitree_sim_isaaclab` (toujours bloqués comme
documenté plus haut, jamais repris), au profit d'**Isaac Sim seul**,
via le **conteneur Docker officiel NVIDIA** (`nvcr.io/nvidia/
isaac-sim:6.1.0`) plutôt qu'une installation native — beaucoup plus
simple à faire fonctionner, avec un vrai résultat obtenu en une
session, contrairement aux blocages en cascade rencontrés plus haut.

### Ce qui a été validé, pour de vrai

1. **Isaac Sim 6.1.0 complet** (pas headless, interface graphique
   complète) tourne dans un conteneur Docker, streamé vers un
   navigateur via WebRTC (port `8210`) — `docker compose` avec deux
   services (`isaac-sim`, `web-viewer`), tous deux `healthy`.
2. **Le G1 se charge et réagit correctement à la physique** — chute
   sous gravité, reste stable une fois posé sur un vrai plan de sol
   physique (`Create > Physics > Ground Plane`, nécessaire : la
   grille par défaut d'Isaac Sim n'a AUCUNE collision, un G1 posé
   dessus tombe à travers).
3. **Deux variantes du G1 trouvées dans le Nucleus cloud NVIDIA**
   (`https://omniverse-content-production.s3-us-west-2.amazonaws.com/
   Assets/Isaac/6.1/Isaac/Robots/Unitree/`) :
   - `G1/g1.usd` — 29 DOF avec mains articulées (doigts séparés).
   - `G1_23dof/g1.usd` — **37 DOF en réalité** (torse + bras/jambes +
     doigts individuels comme `left_five_joint`), pas 23 malgré le
     nom du dossier.
4. **Divergence de nommage trouvée entre le G1 NVIDIA et notre
   mapping AXIS-OS** (`G1_JOINT_ID_TO_MOTOR_INDEX`, construit à
   partir du vrai SDK Unitree) — **18 des 23 nos joints
   correspondent exactement** (mêmes noms), 5 diffèrent :
   - `torso_joint` (Isaac Sim) vs `waist_yaw_joint` (AXIS-OS)
   - `left_elbow_pitch_joint`/`right_elbow_pitch_joint` (Isaac Sim)
     vs `left_elbow_joint`/`right_elbow_joint` (AXIS-OS)
   - `left_wrist_roll_joint`/`right_wrist_roll_joint` : absents de
     ce modèle Isaac Sim (pas de rotation de poignet dans cette
     variante précise, ou nom différent non identifié)

   Pas une erreur de notre part — vraisemblablement une différence
   de convention entre le fichier URDF Unitree source utilisé pour
   construire `G1_JOINT_ID_TO_MOTOR_INDEX` et la conversion USD
   réalisée par NVIDIA pour Isaac Sim. **À corriger avant un usage
   plus poussé** : soit un dictionnaire de traduction de noms entre
   les deux conventions, soit vérifier si un fichier URDF/USD source
   différent du même G1 23dof officiel résout la divergence.

5. **`JointSafetyGuard` (le vrai code AXIS-OS, pas un mock) connecté
   en direct à Isaac Sim et testé avec succès sur un des 18 joints
   qui correspondent** (`right_knee_joint`) :
   - Une commande hors limites (`position=99.0`, limite réelle
     `2.8798`) a été **rejetée**, jamais transmise au robot.
   - Une commande valide (`position=1.0`) a été **acceptée**, puis
     réellement appliquée via `robot.apply_action()` — position
     finale mesurée après simulation : `0.9827 rad` (écart de
     `0.017 rad` avec la cible, cohérent avec l'inertie physique sur
     un nombre fini de pas de simulation, pas une anomalie).

   **Première fois depuis le début du projet que le vrai code de
   sécurité AXIS-OS est validé contre un G1 avec une vraie physique
   simulée** — jusqu'ici, tout avait été soit des tests unitaires en
   Python pur (acteurs mockés), soit bloqué par l'installation
   Isaac Lab documentée plus haut.

### Comment reproduire

```bash
# Dans ~/IsaacLab/IsaacSim (ou équivalent) — suppose docker-compose.yml
# déjà en place pour isaac-sim + web-viewer (nvcr.io/nvidia/isaac-sim:6.1.0)
docker compose -p isim up -d
docker compose -p isim ps   # vérifier "healthy" sur les deux services

# Accès web : http://<IP_PUBLIQUE_INSTANCE>:8210/
# (l'IP publique change à chaque redémarrage d'instance AWS sans IP
# élastique fixe — vérifier l'IP courante avant de se connecter)
```

Dans l'interface web : `Window > Robot Examples > Unitree > G1_23dof`
pour charger le bon fichier (pinces simples, pas de doigts articulés
visuellement — malgré les 37 DOF réels détectés par script).
`Create > Physics > Ground Plane` avant tout test de chute, la grille
seule n'a pas de collision.

Script de test end-to-end (AXIS-OS + Isaac Sim) :
```bash
docker cp ~/AXIS-OS/axis_os isim-isaac-sim-1:/isaac-sim/axis_os
# puis lancer un script Python qui importe
# axis_os.hal.safety.limits.JointSafetyGuard avec
# sys.path.insert(0, "/isaac-sim"), charge G1_23dof/g1.usd via
# isaacsim.core.api.robots.Robot, et applique une ActuatorCommand
# validée par le guard via robot.apply_action(ArticulationAction(...))
```

### Prochaines étapes possibles, non faites ce soir

- Résoudre la divergence de nommage sur les 5 joints manquants
  (torse, coudes, poignets) — dictionnaire de traduction, ou
  fichier source alternatif.
- Étendre le test à un cycle corps entier (23 joints, pas un seul)
  pour valider aussi `position_delta_max` (C5) en conditions
  physiques réelles, pas seulement en mocks.
- Reprendre la piste Isaac Lab documentée plus haut (toujours
  bloquée) — moins prioritaire maintenant qu'Isaac Sim seul donne
  déjà un vrai résultat exploitable pour la validation de sécurité.

---

## SUITE — import du vrai URDF officiel AXIS-OS, 23/23 joints validés (2026-09-18, session 2)

Reprend la percée ci-dessus et résout directement le point 4 (la
divergence de nommage à 5 joints) — pas en corrigeant un
dictionnaire de traduction comme envisagé, mais en éliminant la
divergence à la racine : **au lieu de charger le G1 générique de
NVIDIA, on importe directement notre propre fichier URDF officiel**
(`axis_os/hal/drivers/unitree_g1/urdf/g1_23dof_rev_1_0.urdf`, celui
que `G1_JOINT_ID_TO_MOTOR_INDEX` prend déjà comme référence) via
l'importateur URDF natif d'Isaac Sim.

### API réelle de l'importateur URDF, Isaac Sim 6.1.0

Différente de la commande `omni.kit.commands.execute("URDFParseFile",
...)` utilisée dans les versions/tutoriels précédents — cette
version expose directement `isaacsim.asset.importer.urdf.
URDFImporter` :

```python
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

config = URDFImporterConfig()
config.urdf_path = "/isaac-sim/g1_23dof_rev_1_0.urdf"
config.usd_path = "/isaac-sim/g1_axisos.usd"
config.fix_base = False
config.merge_fixed_joints = False
config.self_collision = False

importer = URDFImporter(config)
usd_path = importer.import_urdf()  # retourne le chemin du .usd généré
```

Pas de fichiers de maillage (`.STL`) — volontairement non committés
dans le dépôt (106MB, voir `axis_os/hal/drivers/unitree_g1/urdf/
meshes/README.md`). Pas bloquant : `URDFImporter` importe la
**structure physique** (joints, limites, masses) sans eux ; le robot
apparaît en formes géométriques simples, pas avec son apparence
visuelle réelle — suffisant pour valider la sécurité, pas pour un
rendu visuel fidèle.

### Résultat : 23/23, aucune divergence

```
=== Notre G1 (URDF officiel AXIS-OS) charge avec succes ===
Nombre de joints (DOF): 23
```

Exactement 23 — ni 29, ni 37 comme les variantes NVIDIA testées en
session 1. Test complet ensuite sur les 23 joints avec
`JointSafetyGuard` (une commande hors limites par joint, une
commande sûre par joint, appliquée réellement via
`robot.apply_action()`) :

```
=== RESULTATS ===
Joints presents et testes: 23/23
Commandes dangereuses rejetees: 23/23
Joints absents d'Isaac Sim: 0
```

**Chaque limite de sécurité réelle (`JointLimits`, une par joint,
définie dans `_build_g1_joint_definitions()`) validée contre le
vrai robot physique simulé — plus un sous-ensemble, le système de
sécurité complet.** Aucune commande dangereuse acceptée, aucune
commande valide refusée à tort, sur l'intégralité des 23 joints.

### En cours, non résolu ce soir : test d'une vraie app (package_sorting)

Tentative de connecter `PackageSortingApp` (via `PlayStoreRuntime`,
`NormalizedRobotHAL`, `UnitreeG1HAL`) à ce même G1 importé, en
interceptant `UnitreeG1HAL.send_joint_commands()` pour répercuter
chaque commande validée vers `robot.apply_action()` côté Isaac Sim
— un vrai "pont" entre notre pile logicielle complète et la
simulation physique, pas seulement `JointSafetyGuard` isolé.

Bloqué sur une erreur de ré-import :
```
AttributeError: 'NoneType' object has no attribute 'is_homogeneous'
```
survenue lors du second appel à `URDFImporter.import_urdf()` avec le
même `usd_path` déjà généré par la session précédente — probablement
un conflit avec le fichier `.usd` déjà présent sur disque dans le
conteneur (`/isaac-sim/g1_axisos.usd`), pas encore confirmé. Piste
de reprise immédiate : supprimer ce fichier avant de relancer
(`docker exec isim-isaac-sim-1 rm -f /isaac-sim/g1_axisos.usd`) puis
relancer le script de test app — non vérifié, la session s'est
arrêtée avant de pouvoir confirmer que ça résout l'erreur.

### Comment reproduire (import URDF officiel)

```bash
# Copier notre URDF officiel dans le conteneur (une fois par session
# de conteneur — perdu si le conteneur est recréé, pas juste redémarré)
docker cp ~/AXIS-OS/axis_os/hal/drivers/unitree_g1/urdf/g1_23dof_rev_1_0.urdf \
    isim-isaac-sim-1:/isaac-sim/g1_23dof_rev_1_0.urdf

# Script complet (import + test des 23 joints) — voir le script
# décrit ci-dessus, reproductible en l'écrivant dans un fichier .py
# local puis `docker cp` + `docker exec ... /isaac-sim/python.sh ...`
```

### Prochaines étapes, mises à jour

- **Immédiat** : débloquer le test `package_sorting` (supprimer
  `g1_axisos.usd` avant réimport, confirmer que l'erreur
  `is_homogeneous` disparaît).
- Étendre le "pont" HAL↔Isaac Sim (actuellement un simple
  intercepteur de fonction sur `send_joint_commands`, écrit pour
  cette session) en un vrai composant réutilisable, si d'autres apps
  doivent être testées de la même façon.
- Résoudre la divergence de nommage n'est plus nécessaire pour le G1
  23dof officiel (éliminée par l'import URDF direct) — reste
  pertinente uniquement si un test contre le G1 générique NVIDIA
  (29/37 DOF) est un jour repris pour une autre raison (comparaison
  visuelle, par exemple).
- Les points de la session précédente (rotation de clé, apparence
  visuelle avec maillages, reprise d'Isaac Lab) restent d'actualité,
  inchangés.

---

## SUITE — pile complète validée de bout en bout : deadlock PhysX résolu, mission package_sorting réussie (2026-09-19, session 3)

Reprend le point bloquant de la session précédente (`is_homogeneous`)
et va bien au-delà : `axis_os/hal/drivers/isaac_sim/bridge.py`
devient un vrai pont réutilisable (`IsaacSimBridge`), et une **vraie
app AXIS-OS complète** (`PackageSortingApp`, via `PlayStoreRuntime`)
tourne pour la première fois de bout en bout contre un G1
physiquement simulé — sandboxing réel compris, pas seulement
`JointSafetyGuard` isolé comme en session 2.

### Bug de fond trouvé et résolu : deadlock PhysX sur thread secondaire

Le premier pont (interception simple de `send_joint_commands`, écrit
en session 2) fonctionnait en usage mono-thread (scripts de test
directs), mais bloquait indéfiniment dès qu'une vraie app tournait
via `PlayStoreRuntime.start_app()` — sandbox multiprocess,
`HALBridgeServer` dans un thread du process parent.

Diagnostic confirmé par un test isolé et minimal :

```python
def call_step_from_thread():
    world.step(render=False)

t = threading.Thread(target=call_step_from_thread, daemon=True)
t.start()
t.join(timeout=15.0)
```

Résultat : le process entier reste bloqué indéfiniment, **même le
`join(timeout=15.0)` ne rend jamais la main** — signe que PhysX
bloque dans du code C sans relâcher le GIL, gelant tous les threads
Python du process, y compris celui qui attend son timeout. Pas un
problème de lenteur : un vrai deadlock structurel, sans version
« prudente avec timeout » possible depuis un thread secondaire.

### Solution : `MainThreadSimPump`, inversion de contrôle

Architecture retenue (voir `bridge.py` pour l'implémentation
complète et sa documentation) :
- **thread principal** : `MainThreadSimPump.run_forever()` — seul
  endroit où `world.step()`/`apply_action()` sont appelés.
- **thread « mission »** : `PlayStoreRuntime.start_app()`/
  `run_mission()`, lancé via `run_mission_in_thread()` — le thread
  principal ne doit jamais y entrer, sinon il ne peut plus vider la
  queue de la pompe (un deadlock différent, pas une correction).
- **thread du pont** : `HALBridgeServer._loop()` (inchangé, créé par
  AXIS-OS) — `send_joint_commands` patché dépose une requête
  (`submit()`, avec un timeout de 5s, jamais une attente infinie) et
  attend, sans jamais toucher directement une API Isaac Sim.
- **lecture aussi concernée** : `JointSafetyGuard` lit le feedback
  (`position_delta_max`) depuis le thread du pont — `read_feedback()`
  lit désormais un instantané publié par le thread principal à chaque
  pas (`read_all_raw()`), jamais l'API Isaac Sim directement.

Un seul `step()` par lot de commandes, pas un par commande —
cohérent avec la publication DDS groupée du driver réel (même
raisonnement déjà appliqué à `move_to_pose()` dans le SDK).

Cette architecture rapproche le pont du driver matériel réel plutôt
que de l'en éloigner : sur un vrai G1, la publication DDS se fait
aussi depuis une boucle dédiée alimentée par les threads applicatifs.

### Deux bugs supplémentaires trouvés en testant le pont contre une vraie app

- **Quota mémoire du sandbox systématiquement dépassé**
  (`VIOLATED (MEMORY_QUOTA)`) — `psutil` mesure `memory_info().rss`
  du sous-process, qui hérite par `fork()` de toute la mémoire déjà
  chargée par Isaac Sim dans le process parent (plusieurs Go). Pas un
  vrai dépassement causé par l'app — un artefact de mesure propre à
  ce contexte de test. 2048 Mo restait insuffisant, 8192 Mo suffit.
- **Un patch de méthode d'instance rend l'app non picklable** —
  `app.manifest = lambda: ...` fait échouer silencieusement la
  sérialisation que `SandboxRunner` doit faire pour transmettre l'app
  au sous-process (`CRASHED`/`SANDBOX_ERROR`, sans message clair).
  Corrigé par une vraie sous-classe définie au niveau module
  (`PackageSortingAppBigQuota`), qui reste parfaitement picklable —
  même famille de problème que celle déjà rencontrée dans
  `axis_os/store/runtime/packaging.py`.

### Résultat final : mission réussie de bout en bout

```
[T+004.68s] Position HOME — prêt.
[Mission] lot=5 | mode=normal
  Colis PKG-0001..0005 — 5/5 triés
  [SAFETY VIOLATION] joint=right_shoulder_pitch_joint | rule=POSITION_DELTA_LIMIT | value=0.8 | limit=0.625
  [SAFETY VIOLATION] joint=left_shoulder_roll_joint | rule=POSITION_DELTA_LIMIT | value=0.8 | limit=0.625
  [SAFETY VIOLATION] joint=left_shoulder_roll_joint | rule=POSITION_DELTA_LIMIT | value=0.8 | limit=0.625
[BILLING] total=2.00€ | commission=0.60€ (30%) | dev=1.40€ (70%)
[Mission] ✓ SUCCESS | durée=2.73s | items=5
```

**Premier test de bout en bout d'une vraie app AXIS-OS — logique
métier, sandboxing, `JointSafetyGuard`, facturation — contre un G1
physiquement simulé.** Les 3 violations `POSITION_DELTA_LIMIT`
détectées en cours de mission confirment que `JointSafetyGuard`
fonctionne réellement en conditions physiques (pas seulement contre
des valeurs mockées) et que la mission continue malgré une violation
individuelle plutôt que de s'arrêter brutalement — comportement
voulu, pas encore expliqué pourquoi `package_sorting` déclenche ces
mouvements trop rapides à ces moments précis (piste ouverte,
ci-dessous).

### Comment reproduire

```bash
docker exec -u root -it isim-isaac-sim-1 rm -rf /isaac-sim/axis_os
docker cp ~/AXIS-OS/axis_os isim-isaac-sim-1:/isaac-sim/axis_os
docker cp ~/AXIS-OS/axis_os/hal/drivers/unitree_g1/urdf/g1_23dof_rev_1_0.urdf \
    isim-isaac-sim-1:/isaac-sim/g1_23dof_rev_1_0.urdf
docker cp ~/AXIS-OS/deploy/isaac_sim_validation/test_s5_full_stack.py \
    isim-isaac-sim-1:/isaac-sim/test_s5_full_stack.py
docker exec -it isim-isaac-sim-1 timeout 200 /isaac-sim/python.sh /isaac-sim/test_s5_full_stack.py
```

⚠️ Piège rencontré deux fois cette session : si le conteneur a été
**recréé** (pas juste redémarré, ex. `docker compose up --build`),
tout ce qui a été copié via `docker cp` disparaît avec lui. Un
`docker cp` qui semble réussir peut aussi écraser un dossier
partiellement (constaté : un ancien `/isaac-sim/axis_os/axis_os/...`
imbriqué, laissant un vieux fichier actif) — en cas de doute,
toujours vérifier le nombre de lignes ou un `grep` sur une classe
récente plutôt que de faire confiance au message de succès de
`docker cp` seul :
```bash
docker exec -it isim-isaac-sim-1 grep -c "MainThreadSimPump" /isaac-sim/axis_os/hal/drivers/isaac_sim/bridge.py
```

### Prochaines étapes

- Étendre ce test aux deux autres apps de référence
  (`SecurityPatrolApp`, `WarehouseOpsApp`) pour confirmer que le pont
  fonctionne au-delà de `package_sorting`.
- Investiguer pourquoi `package_sorting` déclenche ces 3 violations
  `POSITION_DELTA_LIMIT` précises — vrai bug de l'app (mouvement trop
  brusque entre deux positions) ou limite trop stricte pour ce
  contexte de simulation particulier, pas encore déterminé.
- Un vrai G1 physique reste hors de portée de cette suite — tout ce
  qui précède valide la LOGIQUE de contrôle et de sécurité, jamais le
  vrai protocole DDS ni du matériel réel.

---

## SUITE — les 3 apps de référence validées contre Isaac Sim (2026-09-19, session 4)

Reprend les deux points ouverts de la session précédente.

### Point 1 — investigation des 3 violations POSITION_DELTA_LIMIT

Cause trouvée précisément (voir CHANGELOG.md, commits `501f8e6` et
`306e7cf`) : `move_to_pose()` (déjà utilisée dans `on_start()` pour
atteindre HOME au démarrage) n'avait jamais été appliquée aux
mouvements de TRAVAIL de la mission (`_sort()`) ni au retour final
vers HOME (`run_mission()`, `on_stop()`) — ces trois points
utilisaient encore `move_arms()` direct. `REACH_LEFT`/`REACH_RIGHT`
(0.8 rad) depuis HOME (0.0 rad) franchit `position_delta_max` (0.625)
en un seul appel — invisible en tests unitaires purs, révélé
uniquement en conditions physiques réelles.

Corrigé en remplaçant chaque `move_arms()` direct par
`move_to_pose()`, avec une constante dédiée
(`MOVE_DURATION_S = 0.3`, jamais réduite pour accélérer les tests —
voir la note importante ci-dessous sur la marge de sécurité réelle).

**Note de marge de sécurité, importante pour toute app future** :
`position_delta_max = torque_max / 40.0` dans le HAL simulé
(`UnitreeG1HAL._build_g1_joint_definitions()`) utilise une constante
UNIQUE de `kp=40` pour tous les joints, alors que le vrai driver
(`g1_dds_actuator.py::_DEFAULT_KP`) a un `kp` variable par moteur
(jusqu'à 100 sur certains genoux/chevilles) — un facteur 5 d'écart
entre bras (0.625 rad/cycle) et jambes (jusqu'à 3.475 rad/cycle). Un
mouvement validé comme sûr en simulation n'est donc pas
automatiquement sûr sur le vrai G1 sur ces joints précis — toute
future app doit garder cette marge, jamais la réduire pour accélérer
un test.

### Point 2 — les 3 apps de référence validées

`SecurityPatrolApp` et `WarehouseOpsApp` testées d'abord en Python
pur (sans Isaac Sim) sur 50 graines aléatoires chacune : **0
violation trouvée** — contrairement à `package_sorting`, leurs
mouvements (deltas de 0.05-0.08 rad sur les hanches, 0.4-0.6 rad sur
les bras depuis le repos) restent déjà sous les limites réelles sans
nécessiter `move_to_pose()`.

Confirmé ensuite contre Isaac Sim (`test_s5_security_patrol.py`,
`test_s5_warehouse_ops.py`, même architecture `MainThreadSimPump` que
`test_s5_full_stack.py`) :

```
SecurityPatrolApp : 7 checkpoints visités, 2 détections de personne
(badge invalide → alerte CRITICAL correcte, badge valide → accès
autorisé), facturation 70.00€, AUCUNE violation.

WarehouseOpsApp : 2 objets scannés et manipulés avec succès (couple
mesuré cohérent avec le poids réel), facturation 5.00€, AUCUNE
violation.
```

**Les 3 apps de référence du Play Store AXIS-OS tournent désormais de
bout en bout — sandboxing complet, `JointSafetyGuard`, facturation —
contre un G1 physiquement simulé, sans aucune violation de
sécurité.**

### Piège rencontré cette session, à ajouter à la liste déjà documentée

Un `git pull` peut échouer si un fichier a été modifié directement
dans le clone local (ex: via un `sed` appliqué en urgence en cours de
session précédente, jamais reporté dans le commit). Toujours vérifier
`git status --short` avant de investiguer un échec de synchronisation
— un `git checkout -- <fichier>` suivi d'un nouveau `git pull` suffit
si la version distante est la bonne référence à garder.

### Comment reproduire

```bash
docker cp ~/AXIS-OS/deploy/isaac_sim_validation/test_s5_security_patrol.py \
    isim-isaac-sim-1:/isaac-sim/test_s5_security_patrol.py
docker cp ~/AXIS-OS/deploy/isaac_sim_validation/test_s5_warehouse_ops.py \
    isim-isaac-sim-1:/isaac-sim/test_s5_warehouse_ops.py
docker exec -it isim-isaac-sim-1 timeout 200 /isaac-sim/python.sh /isaac-sim/test_s5_security_patrol.py
docker exec -it isim-isaac-sim-1 timeout 200 /isaac-sim/python.sh /isaac-sim/test_s5_warehouse_ops.py
```

### Prochaines étapes

- Un vrai G1 physique reste hors de portée de cette suite — tout ce
  qui précède valide la LOGIQUE de contrôle et de sécurité, jamais le
  vrai protocole DDS ni du matériel réel.
- Intégration d'un modèle VLA/VLM (NVIDIA Isaac GR00T N1.7, choisi
  après recherche — ouvert, licence Apache 2.0, validé sur le
  Unitree G1 par NVIDIA lui-même) pour remplacer les mouvements codés
  en dur des 3 apps de référence par de vraies décisions IA — chantier
  distinct et important, pas commencé, prochaine grande étape du
  projet une fois cette base de sécurité jugée suffisamment solide.

