# Changelog AXIS-OS

Toutes les versions suivent [SemVer 2.0.0](https://semver.org/lang/fr/).
Voir `axis_os/__init__.py` pour la politique de version complète
(notamment : en 0.x, un changement de version MINEURE peut être
cassant — voir `SemVer.is_compatible_with()`).

Ce fichier documente les changements qui touchent le **contrat public**
du SDK (manifest, HAL, interfaces store) — pas chaque commit interne.

## [Non publié]

### Corrections — suite au deuxième audit externe (2026-09-15)

Un second audit externe (Claude Opus), qui a recloné et exécuté le
code (pas seulement lu les messages de commit), a confirmé les 5
CRITIQUES, les 10 ÉLEVÉS et les 12 MOYEN précédents comme
correctement corrigés — et trouvé 3 points supplémentaires, dont un
**nouveau bug introduit par le correctif initial de C2 lui-même** :

- **`publish()` par actionneur → 22 messages DDS incohérents par
  cycle corps entier.** Le correctif initial de C2 (publier après
  chaque `send_command()`) résolvait le problème d'origine (aucune
  commande n'atteignait jamais le robot) mais en introduisait un
  nouveau : sur un cycle de 23 joints, 22 messages `LowCmd_`
  intermédiaires sont émis, chacun avec une partie des articulations
  déjà sur la nouvelle consigne et l'autre encore sur l'ancienne —
  `LowCmd_` transporte les 35 moteurs dans un seul message. Le
  contrôleur du G1, plus rapide que la boucle de commande logicielle,
  peut verrouiller une de ces poses incohérentes à kp=100 : sur un
  bipède en appui, un déséquilibre transitoire à chaque cycle.
  Corrigé : `send_command(cmd, defer_publish=True)` écrit dans le
  buffer sans publier ; `UnitreeG1HAL.send_joint_commands()` appelle
  `defer_publish=True` pour les 23 joints puis publie UNE SEULE fois
  à la fin. `emergency_stop()` continue de publier immédiatement et
  inconditionnellement, inchangé.
- **Aucun test n'assertait `publish()`.** Le correctif le plus
  critique de la série (C2) n'était gardé par aucun test — un
  refactor aurait pu le supprimer sans faire échouer un seul test.
  `test/test_g1_dds_actuator.py` vérifie désormais explicitement
  `hub.publish.called` à chaque point critique (commande normale,
  commande rejetée, e-stop, `defer_publish=True`), et un nouveau test
  dédié confirme qu'un cycle corps entier de 23 joints ne publie
  qu'une seule fois.
- **`CURRENT_OVERLOAD` surveillait une valeur qui n'existe pas.**
  `G1DDSActuator.get_feedback()` retourne toujours `current=0.0`
  (`MotorState_` ne l'expose pas côté G1 réel) — le check ajouté en
  M4 ne pouvait donc structurellement jamais se déclencher sur
  matériel réel, tout en donnant l'apparence d'une protection active
  (pire qu'absent : il rassure sans protéger). Nouveau
  `JointLimits.current_is_measured` (False par défaut) : le check
  n'est émis que si explicitement activé — vrai en simulation
  (`G1SimulatedActuator`, où le courant est dérivé du couple simulé),
  jamais sur `G1DDSActuator` tant qu'aucun vrai capteur n'y est
  branché.

Deux détails mineurs relevés au passage, également corrigés :
- L'horloge de facturation `SUBSCRIPTION` utilisait
  `time.monotonic()` — inadapté pour une fenêtre calendaire de 30
  jours (n'a de sens qu'en différence dans le même process, epoch
  arbitraire à chaque redémarrage). Remplacé par `time.time()`
  (horloge murale). État de facturation (`one_time_billed`,
  `last_subscription_bill_time_s`) documenté explicitement comme tenu
  en mémoire uniquement, perdu à tout redémarrage — persistance
  disque restant un chantier distinct.
- `JointLimits.accel_max` conservé tel quel (pas retiré) : un vrai
  fichier de test (`test_motion_state.py`) l'exerce explicitement ;
  le risque de régression dépassait le gain (juste du code mort selon
  l'audit, pas un danger).

### Corrections — audit externe, vague 3 (M1, M4-M7, M9, M12)

Complète la liste MOYEN priorité du même audit externe (Claude Opus,
2026-09-14) — seul M8 (déjà corrigé en vague 1) manquait à l'appel.
Testés via `test/test_audit_fixes_wave3.py`, 26/26 fichiers de test
passent, zéro régression :

- **M1** — 7 permissions déclarées dans l'enum `Permission`
  (`SENSORS_FORCE_TORQUE`, `SENSORS_CAMERA`, `SENSORS_MICROPHONE`,
  `FLEET_BROADCAST`, `NETWORK_CLOUD`, `NETWORK_LOCAL`) sans qu'aucune
  méthode de `RobotContext` ne les vérifie — documentées explicitement
  comme non appliquées dans l'enum lui-même plutôt que silencieusement
  trompeuses. Les retirer casserait la rétrocompatibilité des
  manifests existants ; les implémenter à vide (ex: une fausse API
  caméra) inventerait une fonctionnalité qui n'existe pas.
- **M4** — le courant mesuré (feedback) n'était jamais vérifié, seul
  `command.current` (l'entrée) l'était. `check_feedback_health()`
  détecte désormais aussi `CURRENT_OVERLOAD` (> 110% de
  `current_max`), déclenche un arrêt d'urgence comme
  `TORQUE_OVERLOAD` (voir E4, vague 1).
- **M5** — le backend Webots retournait une température moteur
  constante (`STUB_TEMP`) même en mode production (motors/sensors
  réellement liés), rendant le Watchdog structurellement aveugle à
  toute surchauffe réelle. Webots n'exposant pas nativement de
  capteur de température, applique désormais la même estimation
  corrélée au couple qu'en mode stub — imparfait mais honnête et
  variable avec l'activité réelle.
- **M6** — `WebotsBackend.connect()` masquait silencieusement (`except
  Exception: pass`) tout joint qui échoue à se lier, tout en
  retournant `True` — les commandes suivantes sur ce joint échouaient
  ensuite en silence. Chaque échec de liaison journalisé bruyamment
  à la connexion, visible avant le premier mouvement.
- **M7** — `stiffness`/`damping` n'étaient jamais validés (NaN et
  valeurs aberrantes passaient le guard) ni transportés par le pont
  HAL (`_serialize`/`_deserialize_commands` ne les incluaient pas, ni
  `current`) — rendant le mode `IMPEDANCE` non fonctionnel dès qu'une
  app tourne en sandbox (systématique depuis C4). Validation ajoutée
  (fini, non-négatif) ; les trois champs désormais transportés.
- **M9** — l'e-stop n'est jamais réarmable automatiquement
  (`_estop_fired` ne repasse jamais à `False`), documenté
  explicitement comme choix de sécurité assumé plutôt que comme un
  oubli — un réarmement automatique risquerait de relancer un
  mouvement sur un robot dont l'état physique réel n'a jamais été
  vérifié.
- **M12** — `requirments.txt` (faute de frappe), contenu périmé,
  supprimé — `requirements.txt` (orthographe correcte) reste la seule
  source de vérité.

### Corrections — audit externe, vague 2 (C5, E8-E10, M2-M3, M10-M11)

Suite du même audit externe (Claude Opus, 2026-09-14) — le dernier des
5 problèmes CRITIQUES, les 3 problèmes ÉLEVÉS restants, et 4 points de
la liste MOYEN priorité les plus significatifs. Testés individuellement
puis via `test/test_audit_fixes_wave2.py`, 25/25 fichiers de test
passent, zéro régression :

- **C5** (le dernier CRITIQUE) — le couple réel produit en contrôle de
  position n'était borné par rien : une commande parfaitement dans
  `[pos_min, pos_max]` mais brutale (loin de la position actuelle)
  peut commander, via le PD embarqué du moteur, un couple bien
  au-delà de `torque_max`. Nouveau champ `JointLimits.
  position_delta_max`, calculé comme `torque_max / kp` dans
  `G1DDSActuator`/`G1SimulatedActuator` et vérifié par
  `JointSafetyGuard` contre le feedback réel avant chaque commande.
- **E8** — le thread du pont HAL mourait silencieusement sur un
  message malformé venant du process enfant (`HALCommand(**raw)`
  hors de tout `try`). Message malformé désormais ignoré avec un log
  `[SECURITY]`, le thread reste vivant et continue de servir de
  vraies requêtes.
- **E9** — sans `psutil`, les quotas CPU/RAM du sandbox étaient
  silencieusement désactivés alors que `run()` continuait de
  rapporter des exécutions "réussies". `SandboxRunner(strict=True)`
  refuse désormais de démarrer sans `psutil` ; avertissement bruyant
  sinon.
- **E10** — un e-stop déclenché par le Supervisor (précisément quand
  le Watchdog est mort ou bloqué) ne positionnait jamais
  `watchdog.estop_fired`, laissant `PlayStoreRuntime.run_mission()`
  et les apps croire le robot disponible. Nouvel état
  `IRobotHAL.estop_active`, positionné par `emergency_stop_all()`,
  consulté en plus de `watchdog.estop_fired` partout où c'était
  nécessaire.
- **M2** — `_bill()` facturait le prix complet à chaque mission quel
  que soit `pricing_model` : une app `ONE_TIME` facturait à chaque
  mission au lieu d'une seule fois, une app `SUBSCRIPTION` facturait
  l'abonnement mensuel entier à chaque mission. Chaque modèle
  applique désormais sa vraie règle (`ONE_TIME` : une fois par
  installation ; `SUBSCRIPTION` : une fois par période de 30 jours).
- **M3** — la facturation reposait uniquement sur `result.success`,
  construit par l'app elle-même dans le process enfant — une app
  malveillante retourne toujours `success=True` sans jamais bouger
  le robot. `HALBridgeServer` compte désormais, côté process PARENT
  (hors de portée de l'app), les commandes motrices réellement
  abouties ; la facturation exige les deux signaux.
- **M10** — le watchdog ne surveillait jamais si la position mesurée
  restait dans `[pos_min, pos_max]` (distinct de `POSITION_DRIFT`,
  qui compare commande et feedback). Nouvelle alerte
  `AlertKind.POSITION_LIMIT_BREACH`, avec confirmation
  multi-échantillons comme les autres checks mécaniques.
- **M11** — aucune app ne pouvait déclencher elle-même un e-stop,
  malgré l'existence de `Permission.EMERGENCY_STOP` dans l'enum.
  `RobotContext.emergency_stop()` ajouté (requiert cette permission
  explicitement), branché sur le pont HAL pour rester accessible en
  sandbox.

### Corrections critiques et élevées (audit externe du 2026-09-14)

Un audit indépendant (Claude Opus) a trouvé 5 problèmes CRITIQUES
(pouvant blesser quelqu'un ou casser le matériel dès la première mise
sous tension) et plusieurs problèmes ÉLEVÉS. Corrigés, testés, zéro
régression sur les 24 fichiers de test existants :

- **C1** — `JointSafetyGuard` ne validait `command.position` que si
  `command.mode == POSITION`, alors que `G1DDSActuator` transmettait
  `q` au moteur quel que soit le mode. Une commande en mode TORQUE/
  VELOCITY avec un champ position resté à une valeur aberrante
  atteignait le moteur sans jamais être vérifiée. Position validée
  désormais dès qu'elle est non-None, indépendamment du mode.
- **C2** — `G1DDSHub.send_motor_command()` n'écrit que dans un buffer
  local ; sans `publish()`, aucune commande n'est jamais transmise au
  réseau DDS — y compris l'arrêt d'urgence, qui écrivait
  `kp=kd=0` dans un message jamais envoyé. `publish()` appelé
  désormais après chaque `send_command()` et immédiatement, sans
  condition, dans `emergency_stop()`.
- **C3** — Le couple/courant n'étaient jamais inversés selon le signe
  du joint en écriture (`command_to_raw`), alors que le feedback
  l'est déjà en lecture. Sur les 9 joints à `sign=-1` du G1, une
  commande de couple partait avec le signe inversé au moteur réel.
- **C4** — `on_start()`/`on_stop()` s'exécutaient hors sandbox, avec
  un accès HAL direct, aucun quota, aucun timeout — alors que
  `run_mission()` isole correctement l'app. Concret : `package_sorting.
  py` bouge physiquement le robot (`move_arms`) dans son `on_start()`.
  Sandboxés désormais via le même `SandboxRunner`/`HALBridgeServer`
  que `run_mission()`. A révélé au passage deux bugs préexistants du
  pont HAL (`get_imu()` retournait un dict brut sans `.read()`,
  `move_arms`/`move_legs` retournaient des chaînes au lieu de vrais
  `HALStatus`) — jamais exercés avant que ce chemin de code soit
  réellement utilisé pour la première fois.
- **E1** — La signature des apps était purement opt-in (une app
  malveillante omet simplement le champ). `PlayStoreRuntime` accepte
  désormais `require_signed_apps=True` (`False` par défaut,
  rétrocompatible) pour la rendre obligatoire. Signe aussi le fichier
  source complet du module désormais, plus seulement le texte de la
  classe (`inspect.getsourcefile` plutôt que `inspect.getsource`).
- **E2** — `_apply_velocity_cap` faisait `cmd.velocity *= ratio` EN
  PLACE sur les objets de l'app appelante — une app qui réutilise son
  dict de commandes à chaque cycle voyait sa vitesse décroître
  géométriquement (`ratio^n`). Retourne désormais un nouveau dict,
  vitesse plafonnée sans mutation.
- **E3** — Le tracker de mouvement enregistrait une commande même
  rejetée par `JointSafetyGuard`, malgré un commentaire affirmant le
  contraire — dangereux une fois une limite d'accélération basée sur
  l'historique branchée (raisonnerait sur une référence falsifiée).
  Enregistrement désormais conditionné à `HALStatus.OK`.
- **E4** — `check_feedback_health()` (détecte notamment
  `TORQUE_OVERLOAD`, signe qu'un moteur force contre un obstacle)
  n'était appelée nulle part en production. Appelée désormais avant
  chaque commande ; une surcharge de couple confirmée déclenche un
  arrêt d'urgence immédiat du joint concerné.
- **E5** — `UnitreeG1HAL` n'instanciait jamais `G1DDSActuator`, en dur
  sur le simulé sans option — le driver matériel réel était du code
  mort. `UnitreeG1HAL(backend="sim"|"dds", network_interface=...)`.
- **E6** — Aucun verrou sur `G1DDSHub`, partagé par les 23 actionneurs
  avec plusieurs écrivains concurrents possibles (mission, watchdog,
  callback DDS) — un e-stop et une commande de mouvement pouvaient
  s'entrelacer dans le même message DDS. `threading.RLock` ajouté.
- **E7** — `_ensure_low_level_mode()` ignorait le code de retour de
  `CheckMode()`/`ReleaseMode()` et bouclait sans limite. Statut
  désormais vérifié, boucle bornée (10 tentatives), retourne un bool
  explicite que l'appelant doit respecter avant de continuer.
- **M8** — L'offset de position était appliqué à la vitesse
  (`normalized_to_raw` utilisé pour les deux), sans effet tant que les
  offsets du G1 sont à 0.0 mais dangereux pour le premier robot qui en
  aurait un non nul. Vitesse traitée séparément, sign seul.

### Ajouts

- `docs/isaac_lab_unitree_sim_status.md` — état détaillé d'une
  tentative de faire tourner unitree_sim_isaaclab (simulation
  physique G1 via Isaac Lab, même protocole DDS que le matériel
  réel). Isaac Lab et unitree_sdk2py installés avec succès (4
  problèmes de compatibilité en cascade résolus et documentés), mais
  la simulation elle-même échoue sur un conflit de version entre
  Isaac Lab et le code de tâche d'unitree_sim_isaaclab
  (`KeyError: 'lin_vel'`). Non bloquant pour la suite — la validation
  de sécurité déjà acquise (23 joints, vraies limites,
  `JointSafetyGuard`) reste entièrement valide.
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
