# Changelog AXIS-OS

Toutes les versions suivent [SemVer 2.0.0](https://semver.org/lang/fr/).
Voir `axis_os/__init__.py` pour la politique de version complète
(notamment : en 0.x, un changement de version MINEURE peut être
cassant — voir `SemVer.is_compatible_with()`).

Ce fichier documente les changements qui touchent le **contrat public**
du SDK (manifest, HAL, interfaces store) — pas chaque commit interne.

## [Non publié]

### Sécurité, changements cassants — isolation des apps et profils d'exécution (2026-10-20)

Étapes 0 et 1 de `docs/isolation_apps.md`, et limites n° 1, 2 et 4 de
`SECURITY.md` (les parties qui restent ouvertes y sont décrites).

- **Pont IPC sans pickle.** Les échanges entre une app et l'OS sont des
  trames JSON (1 Mio au plus, JSON strict : ni NaN, ni infini, ni clé en
  double, ni entier démesuré, profondeur bornée), et chaque commande est
  vérifiée par le schéma de son action avant d'être exécutée : champ
  inconnu, absent ou du mauvais type -> `MALFORMED_COMMAND`. Une trame
  trop longue ou un flot de messages invalides arrête l'app
  (`SandboxErrorKind.PROTOCOL_VIOLATION`, nouveau).
- **Processus de l'app neuf, plus de fork.** Chaque hook (`manifest`,
  `on_start`, `run_mission`, `on_stop`) tourne dans `python -I` lancé avec
  `close_fds`, un environnement réduit à PATH et à la langue, et sa propre
  session ; il ne reçoit que le pont, recharge l'app depuis sa source
  (le texte vérifié pour une app signée), meurt avec l'OS, et tout son
  groupe de processus est tué à la fin de l'appel. `SandboxRunner.run()`
  passe aussi par ce processus.
- **Profil explicite obligatoire.** `PlayStoreRuntime(..., profil=Profil.X)`
  (`axis_os/store/runtime/profil.py`), jamais deviné. En `PRODUCTION` :
  signature obligatoire (`require_signed_apps=False` refusé), fichier local
  des clés de développeur approuvées obligatoire (`cles_approuvees`),
  `accepter_tout_SIMULATION` refusée, psutil exigé, et installation par
  `install_source()` seulement : le code d'une app n'est jamais exécuté
  dans le processus de l'OS. `SIMULATION` est refusé sur un robot réel.
- **`ACTUATORS_AI_DRIVEN` vérifiée à l'exécution** pour les consignes des
  actions d'IA du SDK (`ActuatorCommand.origine == "ia"`), aussi à travers
  `SetpointFollower.set_target(..., origine=ORIGINE_IA)`.
- **Correctifs de la revue du bac à sable** : la sortie d'une app est
  relayée par morceaux bornés (une ligne énorme ne fait plus grossir la
  mémoire de l'OS) et son relais s'arrête même si un processus lancé par
  l'app garde le tube ; le groupe de processus de l'app est tué avant
  d'être récolté ; le processus de l'OS n'est plus inspectable par ceux des
  apps (`prctl(PR_SET_DUMPABLE, 0)`, exigé en production) ; une écoute ne
  dure jamais plus que ce qui reste au hook ; le message de fin d'une app
  est vérifié et son texte d'erreur neutralisé ; l'état de facturation est
  relu avec son type ; `decrypt_verified_app_source()` donne aux apps
  chiffrées un chemin sans exécution vers `install_source()` ; le fichier
  des clés approuvées est lu sans suivre de lien symbolique ni bloquer sur
  un tube nommé, doit appartenir à root ou à l'utilisateur de l'OS et
  faire au plus 1 Mio.
- **Correctifs de la seconde revue** : le journal de mission tenu par l'OS
  est neutralisé (plus de caractère de contrôle, quelle que soit l'origine
  de la ligne), coupé à 1 000 caractères par ligne et borné à 1 million de
  caractères ; le relais de la sortie d'une app passe par poll (aucune
  limite sur le numéro de descripteur) et respecte sa borne ligne par
  ligne ; une écoute laisse la moitié du temps restant à la transcription ;
  le registre des intentions vocales est plafonné ; une date de
  facturation dans le futur est gardée (pas de refacturation) ; les
  erreurs de `validate()` sont neutralisées à l'affichage.

Ce qui casse pour un appelant :

- `PlayStoreRuntime` exige l'argument nommé `profil` ; `require_signed_apps`
  vaut désormais `None` par défaut (lu comme False hors production).
- Les paramètres de mission, l'état d'une instance d'app (attributs) et
  le résultat d'une mission (`metrics`) doivent être des données JSON :
  None, bool, nombres finis, chaînes, listes, dicts à clés textuelles (et
  énumérations, pour l'état). Un attribut qui n'en est pas une est recréé
  par le constructeur de l'app dans son processus ; un paramètre ou un
  résultat qui n'en est pas un fait échouer l'appel proprement.
- Une app définie dans un script lancé directement est rechargée sans le
  code de niveau module de ce script (seuls imports, définitions et
  constantes littérales sont repris) ; une classe définie dans une
  fonction ne peut plus tourner dans le bac à sable.
- `SandboxRunner.run()` n'accepte qu'une fonction désignée par son module
  et son nom (fonction de module, `time.sleep`, `random.random`), avec des
  arguments et un résultat JSON.
- En production, `install(objet)` et donc `update_app()` sont refusés ;
  `install_source()` refuse une app déjà installée (mise à jour depuis la
  source non prise en charge).
- Un `app_id` ne peut plus contenir « : » (réservé aux espaces internes de
  l'OS) ; par le pont, `listen()` est limité à 120 s et `get_joint_groups`
  exige `ACTUATORS_READ` (comme `get_full_state`, que `move_to_pose`
  appelle déjà) ; le journal de mission tenu par l'OS est plafonné à
  10 000 lignes et 1 million de caractères, 1 000 par ligne ; le
  processus de l'OS ne produit plus de vidage mémoire (core dump).
- Par le pont : une ligne de `log()` fait au plus 4 096 caractères (coupée
  par le client), et `declare_intent()` n'accepte que des `params` faits
  de chaînes (32 au plus, de 256 caractères) et au plus 16 groupes de 32
  mots-clés ; le registre des intentions refuse une 257e déclaration.

Tests : `test/test_profils_production.py` et
`test/test_isolation_durcissements.py` (nouveaux) ; la suite existante
passe, avec un profil ajouté à chaque construction de runtime.

### Sécurité, changement cassant — apps chiffrées : signature vérifiée avant toute exécution (2026-10-20)

Le chargeur d'apps chiffrées exécutait le code de niveau module de l'app
avant que sa signature ne soit vérifiée, et seule cette vérification,
faite plus tard par `install()`, établissait l'origine du code : le
chiffrement seul ne la prouve pas, puisque la clé qui sert à chiffrer est
publique.

- `load_app_from_encrypted_bytes()` et `load_app_from_encrypted_file()`
  exigent maintenant `code_signature` (la signature du développeur, faite
  avec `sign_app_code()`) et `developer_public_key` (la clé publique
  attendue du développeur, obtenue par un canal de confiance, jamais lue
  dans le paquet). La signature est vérifiée sur le code déchiffré avant
  toute compilation ou exécution : si elle est invalide ou malformée,
  `AppLoadError` est levée et aucune ligne de l'app n'est exécutée.

Test : `test/test_encrypted_app_packaging.py`, section 7.

### Changements cassants — permissions DANGEROUS confirmées à l'installation (2026-10-10)

Le niveau DANGEROUS promettait une confirmation humaine qu'`install()` ne
demandait jamais : une app qui déclarait `ACTUATORS_WRITE_ALL` ou
`ACTUATORS_AI_DRIVEN` s'installait sans que personne ait accepté qu'elle
fasse bouger le robot.

- `PlayStoreRuntime.install()`, et donc `update_app()`, refuse toute app
  qui déclare une permission DANGEROUS si le runtime n'a pas reçu de
  fonction `confirmer_permissions`, ou si cette fonction ne rend pas
  exactement `True`. Deux fonctions sont fournies : `confirmer_au_terminal`,
  qui pose la question à l'opérateur, et `accepter_tout_SIMULATION`, qui
  accepte tout, pour la simulation et les tests ; le constructeur refuse
  cette dernière avec un robot réel reconnu (aujourd'hui, le G1 en DDS).
- `AppManifest.validate()` refuse un manifeste qui exige
  `Capability.AI_POLICY` sans déclarer `ACTUATORS_AI_DRIVEN`. Il exige
  aussi que `permissions` et `capabilities_required` soient des listes de
  `Permission` et de `Capability`, rien d'autre, et refuse tout caractère
  de contrôle dans `app_id`, `name` et `author`, qui sont affichés au
  moment de la confirmation.

Test : `test/test_permissions_dangereuses.py`.

### Corrections — entrées non fiables refusées, après un audit de sécurité (2026-10-10)

- `LocomotionService` : une consigne de vitesse non finie (NaN ou infinie)
  est refusée et remet la consigne à zéro. Le bornage transformait NaN en
  vitesse maximale (`test/test_locomotion_non_finie.py`).
- `GardienProximite` : une pose, un horodatage ou un balayage invalide
  donne un arrêt.
- `SetpointFollower` : une cible non finie est ignorée, et une consigne
  horodatée dans le futur, ou avec un horodatage invalide, est tenue pour
  périmée.
- `VLACppConnector` : une action qui contient NaN ou inf est rejetée, et
  l'action est horodatée avant l'envoi de la requête plutôt qu'à la
  réception.
- `GR00TConnector` : taille des messages bornée (`taille_max_octets`,
  8 Mio par défaut) ; jeton d'accès facultatif (`api_token`). Changement
  cassant : seul un serveur sur la machine elle-même est accepté, sauf
  `autoriser_distant=True`, parce que ZeroMQ circule en clair ; pour un
  serveur distant, la voie prévue est un tunnel SSH.
- Détecteur OWL-ViT : poids épinglés par défaut (révision fixée, format
  safetensors). Changement cassant : sans hash de commit complet, le
  chargement est refusé, sauf `autoriser_non_epingle_SIMULATION=True`.

### Ajouts — connecteur GR00T N1.7 et VLAService (2026-10-06)

- `GR00TConnector` (`axis_os/store/sdk/ai/gr00t_connector.py`) : un
  connecteur VLA (interface `IVLAConnector`) vers le serveur de NVIDIA qui
  fait tourner un modèle GR00T N1.7, par ZeroMQ et msgpack-numpy.
- `VLAService` (`axis_os/services/policy/vla_service.py`) : fait tourner une
  politique VLA en mission sans jamais toucher un actionneur directement.
  Les bras passent par l'adaptateur et `JointSafetyGuard`, la base par
  `LocomotionService`.

### Ajouts — navigation, manipulation, gardien de proximité, G1 29 DOF (2026-09-25)

- `NavigationService` : chemin planifié par A* sur une carte déclarée, puis
  suivi par poursuite pure ; la position du robot vient d'une source
  injectée (en simulation, la pose lue dans la scène).
- `ManipulationService` : prendre et poser un objet, sans que l'app ait à
  connaître les angles des bras.
- `GardienProximite` (`axis_os/services/locomotion/proximite.py`) : passé
  à `LocomotionService` (paramètre `gardien`, facultatif), il plafonne
  chaque consigne de marche selon la distance aux obstacles vus par un
  lidar, jusqu'à l'arrêt.
- `UnitreeG1HAL(variante="29dof")` : la variante 29 DOF du G1, lue dans
  l'URDF officiel ; `"23dof"` reste la valeur par défaut. Pour cette
  variante, un adaptateur de marche GR00T Whole-Body Control
  (`GR00TWBCAdapter`).

### Ajouts — horloge du robot dans RobotContext (2026-09-24)

- `ctx.now()` et `ctx.sleep()` : l'heure du robot, simulée en simulation et
  réelle sur le robot. Une app les utilise à la place de `time.monotonic()`
  et de `time.sleep()`, pour se comporter de la même façon quand la
  simulation est ralentie.
- Correction : à travers le sandbox, une permission manquante lève
  désormais `PermissionError`, comme en direct. Elle devenait une erreur
  générique, que `walk()` avalait en rendant `False` : l'app ne pouvait pas
  savoir pourquoi elle avait été refusée.

### Ajouts — perception, marche et parole dans RobotContext (2026-09-23)

- `ctx.look_for()` et `ctx.describe_scene()`, qui exigent `SENSORS_CAMERA`,
  désormais appliquée.
- `ctx.walk()` et `ctx.stop_walking()`. `walk()` déclare une consigne de
  vitesse que l'OS fait appliquer par `LocomotionService` ; il exige la
  nouvelle permission `LOCOMOTION_WALK` (DANGEROUS) en plus
  d'`ACTUATORS_WRITE_LEGS`.
- `ctx.listen()` et `ctx.declare_intent()`, servis par le nouveau
  `SpeechService`, qui exigent `SENSORS_MICROPHONE`, désormais appliquée.
  Le G1 déclare son micro et son haut-parleur (`RobotCapability.MICROPHONE`
  et `SPEAKER`).
- Correction : `Capability.MEMORY_PERSISTENT` est disponible si et
  seulement si un service de mémoire est présent. Une app qui l'exigeait
  était jusque-là toujours refusée à l'installation.

### Ajouts — services de perception et de locomotion (2026-09-20 et 2026-09-22)

- `PerceptionService` (2026-09-20), avec l'interface `IObjectDetector` et un
  premier détecteur réel à vocabulaire ouvert, OWL-ViT.
- `LocomotionService` (2026-09-22) : une app pilote la marche par consigne
  de vitesse, sans connaître le robot ; le service délègue à un
  `ILocomotionAdapter` fourni pour chaque robot. Une consigne non
  rafraîchie à temps revient à zéro, et après une chute le service n'envoie
  plus aucune consigne.
- Changement cassant (2026-09-22) : `IMUData.angular_velocity` est exprimée
  dans le repère du capteur, et `G1LocomotionPolicy.build_observation()` et
  `step()` exigent `ang_vel_frame` (`"world"` ou `"body"`), sans valeur par
  défaut.

### Ajouts — HAL : groupes d'articulations, repères, caméra du G1 (2026-09-20)

- `JointGroup` (MANIPULATION, LOCOMOTION, OTHER) et `JointDefinition.group` :
  le driver déclare le groupe de chaque articulation, que le SDK devinait
  jusque-là à partir de mots-clés anglais. `infer_joint_group()` reste un
  repli pour un driver qui ne le déclare pas encore. Sur le G1, le torse est
  déclaré OTHER et reste accessible par `move_all()`.
- `axis_os/hal/core/frames.py` : arbre de repères et transformations
  rigides, sur la convention de TF (ROS), pour relier ce que voit la caméra
  au repère du robot.
- `G1RealSenseCamera` : driver de la caméra RealSense du G1 (interface
  `ICamera`).

### Ajouts — pilotage par une politique apprise (2026-09-19)

- `VLAAction.timestamp_ns` : l'instant où la politique a produit l'action
  (0 si elle n'est pas horodatée).
- `Permission.ACTUATORS_AI_DRIVEN` (DANGEROUS) et `Capability.AI_POLICY`.
  La permission n'est pas encore vérifiée à l'exécution (SECURITY.md,
  limite connue n° 2) ; depuis le 2026-10-10, elle est confirmée à
  l'installation (voir plus haut).
- `SetpointFollower` (`axis_os/services/policy/setpoint_follower.py`) :
  avance par petits pas vers la consigne d'une politique lente, avec une
  échéance de fraîcheur, et passe par les mêmes commandes que les apps,
  donc par `JointSafetyGuard`.
- Correction : à travers le sandbox, `get_full_state()` rend de vrais
  `ActuatorFeedback`, comme en direct, et non des dictionnaires.

### Documentation — les 3 apps de référence validées contre Isaac Sim (2026-09-19, session 4)

Complète `docs/isaac_lab_unitree_sim_status.md`. `SecurityPatrolApp`
et `WarehouseOpsApp` testées d'abord en Python pur sur 50 graines
aléatoires chacune (0 violation trouvée), puis confirmées contre
Isaac Sim (`deploy/isaac_sim_validation/test_s5_security_patrol.py`,
`test_s5_warehouse_ops.py`, même architecture `MainThreadSimPump`) :
SecurityPatrolApp (7 checkpoints, 2 détections de personne gérées
correctement, facturation 70.00€), WarehouseOpsApp (2 objets
manipulés avec succès, facturation 5.00€) — aucune violation de
sécurité dans les deux cas.

Les 3 apps de référence du Play Store AXIS-OS tournent désormais de
bout en bout — sandboxing complet, `JointSafetyGuard`, facturation —
contre un G1 physiquement simulé, sans aucune violation de sécurité.

### fix(package_sorting) — durée de mouvement restaurée à 0.3s, jamais sacrifiée pour la vitesse des tests

Correction d'une décision prise trop vite dans le commit précédent :
`MOVE_DURATION_S` avait été réduite de 0.3s à 0.15s pour faire passer
`test_heavy_integration.py` sous un timeout de test — un mauvais
compromis, signalé et corrigé : le temps d'exécution d'un test n'est
qu'un coût de développement, alors que réduire le nombre de pas
d'interpolation entame la seule vraie marge de sécurité disponible.

Raison supplémentaire de ne jamais grignoter cette marge :
`position_delta_max = torque_max / 40.0` dans le HAL simulé (voir
`UnitreeG1HAL._build_g1_joint_definitions()`) utilise une constante
UNIQUE de `kp=40` pour tous les joints, alors que le vrai driver
(`g1_dds_actuator.py::_DEFAULT_KP`) a un `kp` variable par moteur
(jusqu'à 100 sur certains genoux/chevilles) — un facteur 5 d'écart
entre bras (0.625 rad/cycle) et jambes (jusqu'à 3.475 rad/cycle). Un
mouvement validé comme sûr en simulation n'est donc pas
automatiquement sûr sur le vrai G1 : la vraie limite matérielle peut
être plus serrée que celle mesurée en sim sur les joints à `kp` élevé.

`MOVE_DURATION_S` restaurée à 0.3s. `test_heavy_integration.py`
(138s, 23/23) et le reste de la suite (43/43 fichiers au total)
passent toujours — le seul coût réel de cette correction est un
temps d'exécution de test plus long, jamais un vrai problème.

### fix(package_sorting) — mouvements de travail passés par move_to_pose(), 3 vraies violations éliminées

Trouvé en investiguant les 3 violations `POSITION_DELTA_LIMIT`
observées lors du premier test réussi contre Isaac Sim (voir
`docs/isaac_lab_unitree_sim_status.md`) : `move_to_pose()` (déjà
utilisée dans `on_start()` pour atteindre HOME au démarrage) n'avait
jamais été appliquée aux mouvements de TRAVAIL de la mission
(`_sort()`) ni au retour final vers HOME (`run_mission()`,
`on_stop()`) — ces trois points utilisaient encore `move_arms()`
direct. `REACH_LEFT`/`REACH_RIGHT` (0.8 rad) depuis HOME (0.0 rad),
puis `DEPOSIT_*` (0.8 rad sur un autre joint) depuis `REACH_*` :
chaque saut franchit `position_delta_max` (0.625) en un seul appel —
invisible en tests unitaires purs (jamais un vrai enchaînement de
positions successives), révélé uniquement en conditions physiques
réelles.

Premier essai (`duration_s=delay`, réutilisant le délai de tempo de
simulation de la mission comme durée de mouvement physique) a
lui-même révélé un second bug : `delay` vaut 0.02 à 0.08s selon
`speed_mode`, donnant `int(duration_s/step_s)=1` seul pas
d'interpolation avec le `step_s=0.05` par défaut de `move_to_pose()`
— revient exactement au saut direct qu'on cherche à éviter. Une
constante dédiée (`MOVE_DURATION_S=0.15`, 2 pas, marge de sécurité
confortable — 0.4 rad/pas contre une limite de 0.625) sépare
proprement les deux notions.

48/48 fichiers de test passent, aucune régression — y compris
`test_heavy_integration.py` (40 missions enchaînées), qui avait
d'abord dépassé son timeout avec une première valeur de
`MOVE_DURATION_S=0.3` plus prudente mais trop lente à l'échelle.

### fix(isaac) — quota mémoire du script de validation S5 corrigé (2026-09-19)

`deploy/isaac_sim_validation/test_s5_full_stack.py` : le quota mémoire
utilisé pour tester `PackageSortingApp` était resté à `2048` Mo dans
le fichier committé, alors que `8192` Mo s'était révélé nécessaire en
exécution réelle (voir la session documentée juste en dessous) —
perdu entre la correction en conteneur et le commit initial. Corrigé,
plus une vraie sous-classe picklable (`PackageSortingAppBigQuota`,
définie au niveau module) remplace le patch de méthode d'instance
fragile utilisé initialement.

### Documentation — pile complète validée de bout en bout dans Isaac Sim (2026-09-19, session 3)

Complète `docs/isaac_lab_unitree_sim_status.md` — résout le deadlock
PhysX identifié en fin de session précédente : `World.step()`/
`apply_action()` d'Isaac Sim ne sont pas thread-safe et bloquent dans
du code C sans relâcher le GIL, gelant tout le process (confirmé par
un test isolé — même `thread.join(timeout=15.0)` ne rend jamais la
main). `axis_os/hal/drivers/isaac_sim/bridge.py` : nouvelle classe
`MainThreadSimPump` — tous les appels Isaac Sim ont désormais lieu
exclusivement sur le thread principal, qui devient la boucle de
simulation pendant que la mission AXIS-OS tourne dans un thread
annexe. Corrige aussi la lecture (`JointSafetyGuard` lit un instantané
publié par le thread principal, jamais l'API Isaac Sim directement).

Premier test de bout en bout d'une vraie app AXIS-OS
(`PackageSortingApp`, via `PlayStoreRuntime` complet — sandboxing,
facturation, `JointSafetyGuard`) contre un G1 physiquement simulé :
mission réussie, 5/5 colis triés, 3 vraies violations
`POSITION_DELTA_LIMIT` détectées en cours de mission par
`JointSafetyGuard` en conditions physiques réelles, facturation
calculée correctement.

### Documentation — 23/23 joints validés contre notre propre URDF officiel dans Isaac Sim (2026-09-18, session 2)

Complète `docs/isaac_lab_unitree_sim_status.md` — résout directement
la divergence de nommage trouvée en session 1 (5 des 23 joints
différaient du G1 générique NVIDIA) en éliminant sa cause : au lieu
de charger le G1 de NVIDIA, import direct de notre propre fichier
URDF officiel (`g1_23dof_rev_1_0.urdf`) via `isaacsim.asset.
importer.urdf.URDFImporter` (API native d'Isaac Sim 6.1.0). Résultat
: exactement 23 DOF, correspondance parfaite avec
`G1_JOINT_ID_TO_MOTOR_INDEX` par construction.

`JointSafetyGuard` testé sur l'intégralité des 23 joints (pas un
sous-ensemble comme en session 1) : 23/23 commandes dangereuses
rejetées, 23/23 commandes valides acceptées et réellement appliquées
au robot physiquement simulé. Premier test complet du système de
sécurité AXIS-OS contre une vraie physique simulée, sur tous les
joints réels.

Tentative de connecter une vraie app (`PackageSortingApp`, via
`PlayStoreRuntime` complet) à ce même G1 — en cours, non résolue :
bloquée sur une erreur `AttributeError: 'NoneType' object has no
attribute 'is_homogeneous'` au second import URDF dans la même
session de conteneur, piste de reprise documentée (supprimer le
`.usd` généré avant réimport) mais pas encore vérifiée.

### Documentation — JointSafetyGuard validé contre un vrai G1 dans Isaac Sim (2026-09-18)

`docs/isaac_lab_unitree_sim_status.md` complété avec une vraie
percée, sur un chemin différent de la tentative Isaac Lab +
`unitree_sim_isaaclab` documentée précédemment (toujours bloquée,
inchangée) : Isaac Sim seul, via le conteneur Docker officiel NVIDIA,
avec `JointSafetyGuard` (le vrai code, pas un mock) connecté en
direct à un G1 physiquement simulé — une commande hors limites
rejetée, une commande valide acceptée et réellement appliquée au
robot, position finale mesurée après simulation cohérente avec la
cible demandée. Première fois depuis le début du projet que le code
de sécurité est validé contre une vraie physique simulée plutôt que
des acteurs mockés en tests unitaires.

Divergence de nommage trouvée et documentée entre le G1 du Nucleus
NVIDIA et `G1_JOINT_ID_TO_MOTOR_INDEX` (18/23 joints correspondent
exactement, 5 diffèrent — torse, coudes, poignets) — pas encore
corrigée, prochaine étape possible listée dans le document.

### Ajouts — mise à jour d'une app déjà installée (2026-09-16)

Comble un trou trouvé en testant le comportement réel d'`install()` :
rien n'empêchait d'écraser silencieusement une app déjà installée
(même `app_id`) — sans l'arrêter proprement si elle tournait, sans
vérifier le sens de la mise à jour, et en remettant à zéro l'état de
facturation (`one_time_billed`, `last_subscription_bill_time_s`)
comme pour une toute nouvelle installation.

- **`PlayStoreRuntime.update_app(new_app, allow_downgrade=False)`**
  — inspiré du cycle de vie réel d'Android (`PackageManager`) :
  - Refuse tout downgrade par défaut (`INSTALL_FAILED_VERSION_
    DOWNGRADE` d'Android — protection de sécurité connue, revenir à
    une ancienne version peut réintroduire une faille déjà
    corrigée). `allow_downgrade=True` lève ce refus explicitement.
  - Refuse la mise à jour si une mission tourne actuellement sur ce
    robot (`self._mission_lock` déjà pris, quelle que soit l'app
    concernée) — testé avec un vrai thread concurrent, pas
    seulement en théorie.
  - Arrête proprement l'ancienne version si elle est démarrée
    (réutilise `stop_app()` tel quel, mêmes garanties de
    sandboxing).
  - Réinstalle via `install()` — réutilise toutes les vérifications
    déjà en place (signature, permissions, capacités,
    `ProtectionLevel.SIGNATURE`) pour la nouvelle version ; une mise
    à jour n'est jamais un chemin plus permissif qu'une installation
    neuve.
  - Restaure l'état de facturation de l'ancienne entrée sur la
    nouvelle — une mise à jour n'est pas une réinstallation au sens
    commercial.
  - Ne redémarre jamais automatiquement l'app après mise à jour
    (cohérent avec `ACTION_MY_PACKAGE_REPLACED` d'Android, qui
    notifie sans redémarrer) — reste la responsabilité explicite de
    l'appelant.
- `test/test_app_update.py` — 6 sections, dont le cas le plus
  critique testé avec un vrai thread concurrent (mise à jour refusée
  pendant qu'une mission tourne réellement, pas seulement supposée).
  46/46 fichiers de test du projet passent, aucune régression.

### Ajouts — chargement d'apps depuis un fichier .axisapp chiffré (2026-09-16)

Complète le chiffrement HPKE (`encryption.py`, commit précédent) avec
le vrai chemin d'installation : `axis_os/store/runtime/packaging.py`.

- **`load_app_from_encrypted_bytes()`/`load_app_from_encrypted_file()`**
  — déchiffre un fichier `.axisapp` en mémoire, exécute le code
  déchiffré dans un module dédié pour en extraire la classe
  `RobotApp`, l'instancie, et retourne une instance prête pour
  `PlayStoreRuntime.install()`. Le code déchiffré n'est jamais
  réécrit en clair sur disque à aucune étape.
- Bug trouvé en testant le cycle complet de bout en bout (chiffrer
  une vraie app, la charger, l'installer via un vrai
  `PlayStoreRuntime`) : une classe créée par `exec()` dans un
  namespace transitoire n'est PAS picklable — `SandboxRunner`
  (correctif C4) pickle l'app pour la transmettre à son sous-process
  sandboxé, donc `on_start()`/`run_mission()` échouaient
  systématiquement pour toute app chargée ainsi. Corrigé en
  enregistrant chaque app chargée comme un vrai module dans
  `sys.modules` (nom unique par chargement), avec nettoyage garanti
  en cas d'échec (aucune fuite de module fantôme, vérifié
  explicitement).
- Limite de sécurité assumée et documentée, cohérente avec
  `signing.py` : le chargement dynamique exécute le code déchiffré
  avec les mêmes privilèges que le process appelant (comme un
  `import` normal) — le vrai sandboxing (process séparé, quotas) a
  lieu ensuite, dans `PlayStoreRuntime.install()`, inchangé.
- `test/test_encrypted_app_packaging.py` — 11 tests, dont le cycle
  complet à travers le sandboxing réel (`install`/`start_app`/
  `run_mission`, tous sandboxés), l'absence de fuite dans
  `sys.modules`, l'isolation entre deux apps chargées dans le même
  process, et le chargement depuis un vrai fichier `.axisapp`.
- 45/45 fichiers de test du projet passent, aucune régression.

### Ajouts — chiffrement des apps livrées au Play Store (2026-09-16)

Comble une demande explicite, distincte de la signature (`signing.py`,
inchangée) : une app, une fois livrée au Play Store, ne doit être
lisible que par AXIS-OS lui-même — pas par n'importe qui qui
obtiendrait le fichier publié.

- **`axis_os/store/sdk/encryption.py`** — `generate_platform_keypair()`,
  `encrypt_app_code()`/`decrypt_app_code()` (+ raccourcis fichier).
  HPKE (RFC 9180), cipher suite DHKEM(X25519, HKDF-SHA256) /
  HKDF-SHA256 / AES-256-GCM — la configuration recommandée par Google
  Tink pour la plupart des usages. Seule la clé privée plateforme
  (jamais distribuée) permet de déchiffrer ; n'importe qui peut
  chiffrer avec la clé publique, personne ne peut déchiffrer avec
  elle.
- Implémentation via la bibliothèque tierce `pyhpke`, conforme et
  testée contre les vecteurs de test officiels du RFC 9180 — pas une
  composition manuelle des primitives. Une première version composait
  X25519+HKDF+AES-GCM à la main (primitives individuellement
  correctes, mais un vrai risque d'erreur d'assemblage qu'aucune
  primitive isolément solide ne neutralise) ; remplacée après avoir
  confirmé que le module HPKE natif de `cryptography` (≥50.0.1)
  exige un backend AWS-LC/BoringSSL indisponible ici sans recompiler
  tout le paquet — `pyhpke` fonctionne avec l'installation OpenSSL
  standard.
- 23 tests (`test/test_app_encryption.py`), chaque propriété de
  sécurité vérifiée par exécution réelle, pas seulement documentée :
  contenu original absent de l'enveloppe chiffrée (mémoire et
  disque), seule la vraie clé privée déchiffre, toute altération du
  fichier chiffré est détectée (authenticité AEAD), deux chiffrements
  du même contenu sont non corrélables, robustesse aux entrées
  malformées, cycle complet via de vrais fichiers disque.
- 44/44 fichiers de test du projet passent au total, aucune
  régression.

### Ajouts — verrouillage RobotContext au vrai Play Store (2026-09-16)

Empêche qu'une app tourne accidentellement hors d'AXIS-OS : jusqu'ici,
rien n'empêchait quiconque disposant d'un `NormalizedRobotHAL`/
`WatchdogSystem` réel de construire son propre `RobotContext` avec un
`AppManifest` arbitraire — n'importe quelles permissions, sans jamais
passer par `install()` ni la vérification de signature.

- **`RobotContext` ne peut plus être construit directement** —
  `RobotContext(robot, watchdog, manifest)`, l'appel évident, lève
  désormais `UnauthorizedContextError`. Seul
  `PlayStoreRuntime._make_ctx()` détient un `_RuntimeSessionToken`
  valide.
- Le mécanisme a été attaqué contre lui-même avant d'être considéré
  fini : deux premières versions (comparaison de valeur, registre
  d'instances module-level) se sont révélées contournables, corrigées
  après des tests dédiés qui les cassaient délibérément. La version
  finale vérifie l'identité d'objet (`candidate is self`) — un jeton
  forgé par un attaquant, même comparé au vrai jeton d'un runtime
  existant, échoue toujours.
- **Limite assumée et documentée honnêtement** dans le code : en
  Python pur, aucune barrière n'est absolue contre un développeur qui
  lit le code source et écrit délibérément pour la contourner. Ce
  mécanisme empêche l'usage accidentel ou évident, pas un
  contournement actif et informé.
- **`RobotContext.for_testing()`** — classmethod explicite pour les
  tests unitaires qui ont besoin d'un contexte isolé sans
  `PlayStoreRuntime` complet, sans réintroduire le trou de sécurité.
  `test_context_parity.py` exclut explicitement cette méthode de la
  comparaison de surface (pas une méthode d'API runtime qu'une app
  appelle, un point d'entrée de construction réservé aux tests).
- Avertissement d'exécution (pas seulement documenté) quand
  `require_signed_apps=False` (le défaut, pour rester compatible
  développement) — visible à chaque démarrage de `PlayStoreRuntime`,
  pour qu'un déploiement de production ne l'oublie pas silencieusement.
- `test/test_context_lockdown.py` — 39/39 fichiers de test passent au
  total, aucune régression.

### Ajouts — persistance de la facturation via MemoryService (2026-09-16)

Referme le trou documenté depuis deux audits externes : l'état de
facturation (`InstalledApp.one_time_billed`,
`last_subscription_bill_time_s`) était tenu EN MÉMOIRE uniquement,
perdu à tout redémarrage du process — une app `ONE_TIME` refacturée
comme neuve, une fenêtre `SUBSCRIPTION` de 30 jours relancée à zéro.
`MemoryService` (écrit et testé dans une session précédente,
`axis_os/services/memory/service.py`) existait déjà pour combler
précisément ce trou — ce commit termine le branchement.

- **`PlayStoreRuntime` accepte un `memory_service: Optional[
  MemoryService] = None`** (nouveau paramètre du constructeur, `None`
  par défaut — rétrocompatible). Quand fourni : `install()` lit
  l'état de facturation persisté pour l'`app_id` installé (au lieu
  des valeurs par défaut `False`/`None`) ; `_bill()` réécrit chaque
  mise à jour via `MemoryService.app_store()` (cloisonné par
  `"billing::<app_id>"`, jamais accessible aux vraies apps
  elles-mêmes, dont les clés utilisent directement leur propre
  `app_id`). Sans `memory_service` configuré : comportement en
  mémoire strictement inchangé, testé explicitement en
  non-régression.
- **Critère de succès explicite confirmé** (`test/
  test_billing_persistence.py`) : une app `ONE_TIME`/`SUBSCRIPTION`
  n'est jamais refacturée après un vrai redémarrage — vérifié avec
  de nouvelles instances complètement séparées de
  `NormalizedRobotHAL`/`PlayStoreRuntime`/`MemoryService`, partageant
  uniquement le répertoire disque, exactement ce qu'un vrai
  redémarrage de process produirait.

### Ajouts — fin de l'architecture services système (2026-09-15, session 2/2)

Complète la session précédente (test de non-divergence, pont HAL,
manifest étendu, MemoryService) — dernière étape de l'ordre demandé :
interfaces perception, puis capacités réelles sur les 3 apps de
référence.

- **`ICamera`/`IMicrophone`/`ISpeaker`** (`axis_os/hal/core/
  interfaces.py`) + `CameraFrame`/`AudioChunk` (`types.py`). `ICamera`/
  `IMicrophone` héritent de `ISensor` (capteurs en lecture) ;
  `ISpeaker` n'en hérite pas (émet le monde, ne le perçoit pas).
  Aucune des trois n'expose de moyen de commander un actionneur —
  cohérent avec l'invariant déjà posé pour `IService`
  (`axis_os/services/base.py`).
- **Faux drivers** (`axis_os/hal/drivers/sim_common/sim_perception.py`)
  — pas de driver caméra réel pour le G1 (décision explicite, pas une
  limite technique). `FakeCamera` **rejoue de vraies images PPM
  depuis un dossier disque** (décodage P6 minimal, sans dépendance
  externe), en boucle — pas des données synthétiques générées en
  mémoire. `FakeMicrophone`/`FakeSpeaker` restent synthétiques
  (silence déterministe), faute de source d'enregistrements de
  référence dans ce dépôt.
- **Capacités réelles déclarées sur les 3 apps de référence** —
  dérivées de ce qu'elles font vraiment dans leur code, pas
  supposées : `PackageSortingApp` → `GRASP_SINGLE` (un seul bras à la
  fois, jamais les deux — voir `DEPOSIT_LEFT`/`DEPOSIT_RIGHT`) ;
  `SecurityPatrolApp` → `WALK`, `CAMERA_RGB` (`SPEECH_IN` explicitement
  PAS déclarée : le G1 ne déclare pas `RobotCapability.MICROPHONE`,
  et aucune ligne du fichier n'exerce réellement le micro) ;
  `WarehouseOpsApp` → `WALK`, `GRASP_SINGLE`, `CAMERA_RGB`.
- Bug trouvé et corrigé en testant l'installation réelle des 3 apps :
  `_available_capabilities()` ne dérivait jamais `WALK`/`GRASP_SINGLE`
  depuis `RobotCapability.LEGS`/`ARMS` — les 3 apps échouaient à
  s'installer avec leurs propres capacités fraîchement déclarées.
  Corrigé : ces deux `Capability` de mouvement DE BASE sont bien
  dérivées du matériel (les apps bougent déjà bras/jambes directement
  via `move_arms()`/`move_legs()`, un mouvement physique réel, pas
  hypothétique) — distinct d'un futur SERVICE intelligent de haut
  niveau (planification de marche, préhension guidée par vision),
  qui lui reste effectivement inexistant.
- `test/test_perception_interfaces.py`,
  `test/test_reference_apps_capabilities.py` — 37/37 fichiers de
  test passent au total, aucune régression.

### Ajouts — début de l'architecture services système (2026-09-15, session 1/2)

Suite à une conception externe (docs/architecture_services*.md, non
committée séparément — voir le document fourni), première partie de
l'implémentation dans l'ordre demandé : le test de non-divergence
d'abord, le contrat de service validé par une vraie implémentation
ensuite. La suite (interfaces caméra/micro/haut-parleur, capacités
sur les 3 apps de référence) reprend dans une session suivante.

- **Test de non-divergence** (`test/test_context_parity.py`) — écrit
  AVANT toute correction, a révélé que `RobotContext` exposait 11
  méthodes et `HALBridgeClient` seulement 8 : `get_joint_state`,
  `move_all`, `subscribe_alerts` manquantes côté sandbox. Sans lui,
  une app se comporterait différemment selon qu'elle tourne
  sandboxée ou non — c'est le test, pas une interface abstraite
  commune, qui empêche ce problème de revenir.
- **3 actions ajoutées au pont HAL** — `move_all` (sans elle,
  `waist_yaw_joint` restait inatteignable en sandbox, n'étant ni un
  bras ni une jambe), `get_joint_state`, `subscribe_alerts` (adapté
  par polling en arrière-plan, un vrai `Callable` ne pouvant pas
  traverser une `multiprocessing.Queue` — nouveau buffer
  `WatchdogSystem.get_recent_alerts()`). Vérifié explicitement que la
  permission `ACTUATORS_WRITE_ALL` pour `move_all` est contrôlée
  côté SERVEUR (`HALBridgeServer._handle`, via le vrai
  `RobotContext`), jamais côté client — le process enfant sandboxé
  est non fiable, sa propre vérification ne compterait pour rien.
- Bug préexistant corrigé au passage : `NormalizedRobotHAL` ne
  délégait jamais `get_motion_tracker()` à son `IRobotHAL`
  sous-jacent — révélé en testant `WatchdogSystem` avec
  `NormalizedRobotHAL` pour la première fois.
- **Manifest étendu** — `Capability` (WALK, GRASP_SINGLE/DUAL,
  CAMERA_RGB/DEPTH, SPEECH_IN/OUT, NAVIGATION_MAP,
  MEMORY_PERSISTENT), `IntentDecl`, `ProtectionLevel`
  (NORMAL/DANGEROUS/SIGNATURE) avec `PERMISSION_LEVELS`, et les deux
  champs `capabilities_required`/`intents` sur `AppManifest`.
  `PlayStoreRuntime.install()` refuse désormais une app dont une
  `Capability` manque sur ce robot (testé : refusé pour
  `CAMERA_DEPTH`, le G1 ne déclarant que `CAMERA` générique — mapping
  volontairement conservateur, `_available_capabilities()`), et
  refuse une app tierce déclarant une permission `SIGNATURE` (ex:
  nouvelle `ACTUATORS_RAW`, pour un futur niveau 1 du SDK) sauf si
  réellement signée par une nouvelle clé plateforme
  (`platform_public_key` sur `PlayStoreRuntime`, fail-safe : aucune
  permission SIGNATURE accordée si cette clé n'est pas configurée).
- Régression trouvée et corrigée en cours de route :
  `EMERGENCY_STOP`/`FLEET_BROADCAST` classées `SIGNATURE` cassaient
  les apps de référence existantes. `EMERGENCY_STOP` repassée en
  `DANGEROUS` (contredisait directement l'intention de M11 — permettre
  aux apps tierces de déclencher un e-stop, avec confirmation).
  `FLEET_BROADCAST` reste `SIGNATURE` ; sa déclaration prospective et
  jamais exercée dans `warehouse_ops` a été retirée à la place.
- **`axis_os/services/`** — nouveau package. `IService`/
  `ServiceResult`/`ServiceHealth` (`base.py`), et **`MemoryService`**,
  premier service système réellement implémenté (pas une coquille) :
  stockage clé-valeur persisté sur disque, `remember_place`/
  `recall_place`, mémoire épisodique naïve, `app_store`/`app_load`
  cloisonné par `app_id`. Corrige au passage, pour ce nouveau
  service, le bug relevé par un audit externe sur l'état de
  facturation perdu à chaque redémarrage (`PlayStoreRuntime` n'est
  pas encore branché dessus — chantier séparé). Critère de succès
  explicite confirmé par un vrai sous-process Python indépendant
  (pas une simulation en mémoire) : un lieu nommé survit à un
  redémarrage complet du process. Bug de collision de clé trouvé et
  corrigé en testant un cas limite (`app_id` contenant le séparateur
  utilisé en interne) — encodage JSON de la paire plutôt qu'une
  simple concaténation.

### Ajouts — 5 recommandations externes de renforcement (2026-09-15)

Suite au second audit externe, une liste de 5 recommandations
complémentaires (redondance, robustesse d'infrastructure) a été
implémentée dans l'ordre de priorité recommandé :

- **`RobotContext.move_to_pose()` / `HALBridgeClient.move_to_pose()`**
  (point pratique urgent, priorité 1) — comble un vrai blocage créé
  par le correctif C5 : `position_delta_max` protège correctement
  contre un saut de position dangereux, mais aucune app ne pouvait
  atteindre une cible légitime au-delà de cette limite en un seul
  appel. Concret : `package_sorting.on_start()` commande directement
  HOME (~0 rad) — si le G1 démarre bras repliés (~1.2 rad),
  `JointSafetyGuard` refuse à raison ce saut, et l'app ne démarrait
  jamais. `move_to_pose()` interpole progressivement depuis la
  position réelle mesurée, par petits pas restant sous
  `position_delta_max` à chaque itération. `package_sorting.on_start()`
  mis à jour pour l'utiliser — testé avec le scénario exact décrit.
- **Chien de garde systemd** (`_sd_notify_watchdog()` dans
  `WatchdogSystem`, `deploy/systemd/axis-os.service`) — si
  `_monitor_loop()` se fige complètement (deadlock, boucle infinie
  échappant au `try/except`), rien dans le process Python ne peut
  s'en apercevoir. `WatchdogSec=2s` + heartbeat envoyé après chaque
  cycle réussi : c'est l'init système (le noyau), pas AXIS-OS
  lui-même, qui détecte l'absence de notification et redémarre le
  service. Testé avec un vrai socket Unix datagram (protocole
  `sd_notify` réel), y compris en intégration avec `WatchdogSystem.
  start()`.
- **Moniteur de fraîcheur hors process** (`deploy/freshness_monitor/`,
  en C) — protège spécifiquement contre la mort du process principal
  AXIS-OS, la seule classe de panne que le chien de garde systemd
  détecte mais ne prévient pas immédiatement (délai de `WatchdogSec`
  + temps de redémarrage). Écrit en C, pas Python, pour ne partager
  aucun mode de défaillance avec l'interpréteur qu'il surveille.
  Intégration DDS réelle implémentée (pas un simple squelette) :
  schéma `LowCmd_`/`MotorCmd_` reconstruit en `.idl` (Unitree ne
  distribue pas de fichier `.idl` source, seulement des `.hpp` C++
  déjà générés — reconstruit ici en comparant deux générations
  officielles indépendantes, Python et C++, vérifiées identiques),
  compilé avec le vrai `idlc` de CycloneDDS. CRC32 copié à l'identique
  depuis l'algorithme officiel Unitree (`unitree_sdk2/include/
  unitree/dds_wrapper/common/crc.h`). Compile sans aucun avertissement,
  démon testé en conditions quasi réelles (abonnement DDS réel
  confirmé) — bloqué uniquement par le multicast désactivé dans
  l'environnement de développement (voir `deploy/freshness_monitor/
  README.md` pour le détail complet de cette limite, non liée au
  code lui-même).
- **Moniteur de sécurité indépendant**
  (`axis_os/hal/independent_monitor/`) — redondance DIVERSE (pas
  duplication) : lit le feedback directement depuis `IRobotHAL.
  get_full_state()`, sans jamais passer par `NormalizedRobotHAL` ni
  `KinematicNormalizer`, et compare aux limites lues directement
  depuis le fichier URDF officiel via `urdf_loader.py` — jamais via
  `robot.py`. Conçu précisément pour attraper des bugs de CONCEPTION
  du chemin de commande (comme C3, où `JointSafetyGuard` validait un
  couple déjà inversé par le normalizer) que la redondance de
  duplication ne peut pas attraper. Isolation architecturale vérifiée
  **statiquement** par analyse des imports du module (pas seulement
  par comportement observé) — un futur refactor qui réintroduirait le
  couplage ferait échouer ce test immédiatement.
- **Mutation testing** sur `axis_os/hal/safety/limits.py` (via
  `cosmic-ray` — `mutmut` 3.x incompatible avec le style de nos
  fichiers de test sans réécriture complète). Session partielle
  (59/305 mutants exécutés avant timeout) mais réelle : a trouvé 5
  vrais trous de couverture sur des cas limites exacts
  (`TORQUE_OVERLOAD`, `TEMPERATURE_CRITICAL`, `position_delta_max`,
  `accel_max`, validation de construction de `JointLimits`) —
  fermés dans `test/test_mutation_boundary_cases.py`. Voir
  `docs/mutation_testing_notes.md` pour le rapport complet et
  comment reprendre une session plus exhaustive.
- **`docs/hardware_estop_notes.md`** — point de fond, non codé :
  rien dans ce dépôt (y compris les 4 ajouts ci-dessus) n'est une
  fonction de sécurité au sens normatif (ISO 13849, IEC 61508). Un
  bouton d'arrêt d'urgence matériel, câblé, qui coupe l'alimentation
  de puissance des moteurs indépendamment de tout logiciel, reste la
  seule vraie garantie avant tout déploiement à proximité d'humains
  — AXIS-OS reste une couche de supervision intelligente au-dessus,
  jamais à sa place.

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
