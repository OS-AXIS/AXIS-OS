# Changelog AXIS-OS

Toutes les versions suivent [SemVer 2.0.0](https://semver.org/lang/fr/).
Voir `axis_os/__init__.py` pour la politique de version complète
(notamment : en 0.x, un changement de version MINEURE peut être
cassant — voir `SemVer.is_compatible_with()`).

Ce fichier documente les changements qui touchent le **contrat public**
du SDK (manifest, HAL, interfaces store) — pas chaque commit interne.

## [Non publié]

### Ajouts

- **`G1DDSActuator`** (`axis_os/hal/drivers/unitree_g1/g1_dds_actuator.py`) :
  premier driver pour le VRAI matériel G1 physique, via le SDK
  officiel Unitree (`unitree_sdk2py`, protocole DDS/Ethernet).
  Comble un trou identifié lors d'un audit — jusqu'ici, aucun chemin
  technique n'existait pour déployer AXIS-OS sur un G1 réel, seul
  `G1SimulatedActuator` existait. Le commentaire précédent dans
  `robot.py` ("remplacer par G1CANActuator") était lui-même erroné :
  le G1 communique par DDS, pas par CAN direct. Mapping exact des 23
  joints vers leurs index moteur officiels (`G1_JOINT_ID_TO_MOTOR_INDEX`,
  vérifié contre l'exemple officiel `g1_low_level_example.py`),
  intègre les points de sécurité documentés par le SDK officiel
  (désactivation du mode sport intégré, CRC obligatoire par commande).
  **⚠️ Non testé sur matériel physique** — écrit en suivant exactement
  le code source et les exemples officiels, à valider sur un vrai G1
  avant tout déploiement client.
- `test/test_g1_dds_actuator.py` — 19 tests avec un hub DDS mocké
  (mapping des 23 joints, aucun doublon d'index, commande dangereuse
  qui n'atteint jamais le hub réel, comportement de l'arrêt d'urgence).
- **Validation du prix minimum viable** (`AppManifest.validate()`) :
  comble un bug financier identifié lors d'un audit — avec un
  `base_price_eur` trop bas (ex: 0.01€), l'arrondi à 2 décimales de
  `store_price_eur` pouvait ramener `platform_commission_eur` à
  exactement 0.00€ — la plateforme ne touchant alors littéralement
  rien sur une app pourtant déclarée payante à 30% de commission. Le
  seuil minimum viable (0.02€ avec le taux actuel) est calculé
  dynamiquement à partir de `COMMISSION_RATE`, pas codé en dur — reste
  correct si le taux change. N'affecte jamais les apps `FREE`
  (`commission=0.0€` y est normal, pas un bug).
- `test/test_pricing_validation.py` — 16 tests (cas bug exact, seuil
  minimum valide, non-régression sur les prix déjà utilisés par les
  apps de référence, cohérence du taux de commission).
- **Verrou global de mission** (`PlayStoreRuntime.run_mission()`) :
  comble un trou identifié lors d'un audit — rien n'empêchait
  jusqu'ici deux missions concurrentes (même app appelée deux fois,
  ou deux apps différentes) d'envoyer des commandes contradictoires
  aux mêmes actionneurs physiques en même temps. Un seul robot = une
  seule mission active à la fois. Un appel concurrent est refusé
  immédiatement avec un `MissionResult` explicite (`success=False`),
  jamais mis en attente silencieuse. Le verrou est toujours relâché
  via `try/finally`, y compris si la mission échoue ou lève une
  exception — pas de deadlock possible.
- `test/test_mission_lock.py` — 8 tests (missions concurrentes même
  app / apps différentes, relâchement du verrou après succès et
  après échec).
- `docs/isaac_sim_headless_patches.md` — documentation des patches
  nécessaires pour faire tourner Isaac Sim 4.5 en mode headless sur
  un serveur sans affichage, issue de la session de validation du
  2026-09-13. Confirme les 23 joints REVOLUTE du G1 et leurs limites
  réelles via `URDFParseFile()` (parsing seul), avec `JointSafetyGuard`
  testé sur les 23 joints individuellement contre ces vraies données.
  Documente aussi une limite non résolue (segfault natif de
  `URDFParseAndImportFile()`, bug connu de la communauté Isaac Sim).
- `axis_os/hal/drivers/unitree_g1/urdf/meshes/README.md` — instructions
  de récupération des fichiers `.STL` du G1 (non versionnés, 106MB,
  source unitreerobotics/unitree_ros).
- `docs/ai_vs_script_authority_notes.md` — piste (non implémentée) de
  distinction entre commandes d'origine scriptée et commandes
  d'origine IA (VLACppConnector/OpenVLAConnector) dans le modèle de
  permissions, avec condition de reprise explicite.
- **`VLACppConnector`** (`axis_os/store/sdk/ai/connector.py`) : nouveau
  connecteur `IVLAConnector`, client ZeroMQ/Protobuf compatible avec
  le protocole exact du serveur d'inférence
  [vla.cpp](https://github.com/VinRobotics/vla.cpp) (licence Apache
  2.0) — un moteur C++ (basé sur llama.cpp) qui fait tourner des
  modèles VLA au format GGUF unifié (SmolVLA, π0, OpenVLA-OFT, GR00T
  N1.x...), benchmarké publiquement sur NVIDIA Jetson (le même type
  de module que le Jetson Orin NX du Unitree G1 EDU). Schéma protobuf
  copié tel quel depuis le dépôt source
  (`axis_os/store/sdk/ai/proto/vla.proto`), compilé avec `protoc`.
  Coexiste avec `OpenVLAConnector` (stub de développement) sans le
  remplacer — une app choisit le connecteur adapté à son déploiement.
  Testé de bout en bout (`test_vla_cpp_connector.py`) contre un vrai
  serveur ZeroMQ minimal parlant le protocole réel, pas un mock.
- **Signature Ed25519 des apps** (`axis_os/store/sdk/signing.py`) :
  comble le trou de sécurité identifié dès le début du projet — rien
  n'empêchait jusqu'ici un fichier .py quelconque de se faire passer
  pour une app AXIS-OS légitime. `AppManifest.code_signature` et
  `.developer_public_key` (optionnels, rétrocompatibles) permettent à
  un développeur de signer le code de son app ; `PlayStoreRuntime.
  install()` vérifie cette signature avant installation et refuse
  proprement si le code a été modifié après signature. Limite
  documentée : Trust On First Use, pas d'autorité de certification
  centrale (voir docstring du module).
- `docs/fleet_manager_design.md` complété avec une section sur SROS2
  (Secure ROS2 / DDS-Security) comme référence pour la sécurité
  réseau du futur Fleet Manager.
- `docs/sandboxing_hardening_notes.md` — pistes de renforcement futur
  du sandboxing (AppArmor/seccomp/Firejail, puis gVisor/Firecracker
  si nécessaire), non implémentées, avec justification de séquencement.
- `test_app_signing.py` — tests de bout en bout pour la signature des
  apps.
- **Chaînage de hash pour `StructuredLogger`**
  (`axis_os/hal/core/structured_log.py`) : chaque entrée de log inclut
  désormais `hash` et `prev_hash` (SHA-256), formant une chaîne
  vérifiable via `verify_chain()`. Motivation : l'EU AI Act impose des
  obligations de transparence pour les systèmes d'IA à risque dès août
  2026 — un journal d'incident de sécurité robotique n'a de valeur en
  investigation que si son intégrité peut être prouvée, pas seulement
  supposée (même logique que l'enregistreur de vol en aviation).
  Rétrocompatible : les logs écrits avant ce changement restent lisibles
  (`read_recent()` fonctionne toujours), `verify_chain()` signale
  honnêtement leur absence de chaînage plutôt que de planter.
- `test_structured_log_chain.py` — 19 tests (chaîne valide, détection
  de modification, détection de suppression de ligne, continuité entre
  redémarrages de process, rétrocompatibilité, thread-safety).

## [0.2.0] — 2026-09-08

### ⚠️ Changements cassants

- **`HardwareRequirements`** (`axis_os/store/sdk/manifest.py`) : les
  anciens champs booléens (`requires_arms`, `requires_legs`,
  `requires_imu`, `requires_force_torque`, `requires_camera`) sont
  **retirés**, remplacés par `required: list[RobotCapability]` et
  `optional: list[RobotCapability]`. Toute app qui utilisait l'ancienne
  syntaxe doit migrer vers la nouvelle.
  - Migration : `HardwareRequirements(requires_arms=True)` devient
    `HardwareRequirements(required=[RobotCapability.ARMS])`.
- **Identifiants de joints du G1** (`axis_os/hal/drivers/unitree_g1/`) :
  tous les `joint_id` gagnent le suffixe `_joint` (ex: `"left_knee"`
  devient `"left_knee_joint"`), pour correspondre au fichier URDF
  officiel Unitree désormais utilisé comme source de vérité (voir
  `axis_os/hal/core/urdf_loader.py`). Le nombre de joints du G1 passe
  de 24 (codés à la main, avec des erreurs) à 23 (chargés depuis le
  fichier officiel).
  - Découverte associée : les anciennes limites de couple max
    codées à la main étaient sous-estimées jusqu'à 37 % sur certains
    joints de hanche/genou (88 Nm au lieu de 139 Nm réels) — pas
    seulement un problème de nommage, une vraie correction de
    sécurité.
  - Le joint `waist_roll` n'existe plus : la variante 23dof officielle
    du G1 n'a qu'un seul joint de taille (`waist_yaw_joint`).

### Ajouts

- `axis_os/hal/core/urdf_loader.py` — parseur URDF générique, source
  de vérité pour les définitions de joints de tout futur robot
  supporté par AXIS-OS (pas seulement le G1).
- Fichiers URDF officiels du G1 (licence BSD 3-Clause, Unitree
  Robotics) sous `axis_os/hal/drivers/unitree_g1/urdf/`.
- `AppManifest.sdk_version_min` (optionnel, `None` par défaut —
  rétrocompatible) : permet à une app de déclarer la version minimale
  d'AXIS-OS qu'elle requiert. `PlayStoreRuntime.install()` vérifie
  cette compatibilité et refuse proprement l'installation, avec un
  message clair, plutôt que de risquer un échec silencieux ou confus
  plus loin dans le code.
- `axis_os.SemVer` — utilitaire de version/comparaison SemVer.
- `docs/fleet_manager_design.md` — conception cible (non implémentée)
  pour un futur Fleet Manager, basée sur un papier de recherche 2026
  (Federated Single-Agent Robotics, arXiv:2604.11028).

## [0.1.0] — sessions précédentes

Première version fonctionnelle : HAL (Watchdog, Safety, Sandbox,
Capabilities, ResourceManager, Supervisor), Play Store (permissions,
quotas, facturation), 3 apps de référence (package_sorting,
security_patrol, warehouse_ops). Pas de numéro de version global à
l'époque — reconstitué a posteriori comme point de départ 0.1.0.
