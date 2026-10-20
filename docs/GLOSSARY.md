# Glossary — French terms in the AXIS-OS code

*Glossaire français → anglais des termes les plus fréquents du code.*

The code, its comments and most documents are written in French. This
table gives the English meaning of the French words you will meet most
often in class names, comments, docstrings and messages. Terms that are
already English in the code (HAL, watchdog, e-stop, sandbox, Play Store,
`JointSafetyGuard`…) are not repeated here.

## Words

| Français | English | Short meaning |
|----------|---------|---------------|
| arrêt d'urgence | emergency stop (e-stop) | Immediate stop of every motor, triggered by the watchdog, the supervisor or an app. |
| articulation | joint | One motorised axis of the robot (knee, elbow…). |
| bras | arm | — |
| butée | joint limit, end stop | Position an articulation must never pass. |
| capacité | capability | What the robot can do (walk, grasp…), declared by services and required by apps. |
| capteur | sensor | Camera, lidar, microphone, IMU… |
| carte | map | The 2D map used by the navigation service. |
| carton | cardboard box | The object handled in the parcel demos. |
| chemin | path | Route planned by the navigation service; also a file path. |
| chiffrement, chiffré | encryption, encrypted | How a `.axisapp` package keeps its code private. |
| clé (publique, privée) | key (public, private) | Cryptographic key for signing or encryption. |
| colis | parcel | A box to sort in the sorting app. |
| commande | command | A value sent to a motor (position, velocity, torque). |
| consigne | setpoint, target | The value a controller is asked to reach. |
| couple | torque | Rotational force of a motor, in N·m. |
| défaut | fault; also default | A hardware fault state, or a default value: read the context. |
| dépose | drop-off | Putting a parcel down at its station. |
| détection | detection | An object found in a camera image. |
| échec | failure | — |
| empreinte | hash, fingerprint | SHA-256 digest of a file or log entry; also a git commit hash. |
| épisode | episode | One recorded demonstration, used to train a VLA model. |
| facturation | billing | Computing what each mission costs. |
| façade | facade | Simple interface over several services (`FacadeServices`). |
| garde-fou | safety guard | The per-joint check every command passes (`JointSafetyGuard`). |
| gardien (de proximité) | (proximity) guard | Slows then stops the robot near an obstacle (`GardienProximite`). |
| intention | intent | What a spoken sentence asks for, recognised from keywords. |
| jambes | legs | — |
| journal | log | Persistent record of safety events (`logs/*.jsonl`). |
| manifeste | manifest | What an app declares: permissions, capabilities, price. |
| marche | walking | Locomotion (`walk()`, `LocomotionService`). |
| mémoire | memory | Data an app keeps between missions (`MemoryService`). |
| mission | mission | One run of an app's `run_mission()`. |
| moteur | motor | — |
| normalisé, normaliseur | normalised, normaliser | The common joint convention (positive = flexion) and the layer that converts to it. |
| parole | speech | Speech recognition service. |
| pilote | driver | The robot-specific HAL code (Unitree G1, simulation…). |
| plateforme | platform | AXIS-OS itself, as the owner of the platform keys. |
| pont | bridge | Link between two parts: app sandbox ↔ HAL, AXIS-OS ↔ Isaac Sim. |
| poste | station | A place where parcels are dropped off (`PosteDepose`). |
| posture | pose | A set of joint angles (`Postures`). |
| prise, saisie | grasp | Picking up an object. |
| profondeur | depth | Distance image from a depth camera. |
| quota | quota | CPU, memory or time limit of a sandboxed app. |
| repère | frame (of reference) | Coordinate system (world, robot body, camera…). |
| réflexe | scripted behaviour | A hand-written movement, as opposed to one proposed by a model. |
| retour (capteur) | feedback | What a motor reports: position, velocity, torque, temperature. |
| suiveur (de consignes) | setpoint follower | Moves joints step by step towards a target (`SetpointFollower`). |
| superviseur | supervisor | In the HAL: watches the watchdog. In the sorting app: turns operator requests into rules. |
| tri | sorting | The parcel-sorting app and demo. |
| vitesse | velocity, speed | — |

## French class names

| Class | English | Where |
|-------|---------|-------|
| `Carte` | Map | `axis_os/services/navigation/` |
| `Planificateur` | Planner | `axis_os/services/navigation/` |
| `GardienProximite` | ProximityGuard | `axis_os/services/locomotion/` |
| `Postures`, `Objet` | Poses, Object | `axis_os/services/manipulation/` |
| `Colis`, `PosteDepose` | Parcel, DropOffStation | `axis_os/apps/tri_colis/` |
| `FacadeServices` | ServicesFacade | `axis_os/apps/tri_colis/` |
| `Superviseur`, `Interpreteur`, `Ordre`, `Reponse` | Supervisor, Interpreter, Order, Reply | `axis_os/apps/tri_colis/` |
| `CollecteurPrises`, `EnregistreurEpisodes`, `EvaluateurPrises` | GraspCollector, EpisodeRecorder, GraspEvaluator | `axis_os/vla/` |
| `EtatVLA`, `EtatNavigation` | VLAState, NavigationState | `axis_os/services/` |
