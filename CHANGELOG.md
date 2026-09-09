# Changelog AXIS-OS

Toutes les versions suivent [SemVer 2.0.0](https://semver.org/lang/fr/).
Voir `axis_os/__init__.py` pour la politique de version complète
(notamment : en 0.x, un changement de version MINEURE peut être
cassant — voir `SemVer.is_compatible_with()`).

Ce fichier documente les changements qui touchent le **contrat public**
du SDK (manifest, HAL, interfaces store) — pas chaque commit interne.

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
