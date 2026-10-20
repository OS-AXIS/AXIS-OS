# Security policy

*Version française plus bas.*

AXIS-OS is meant to drive robots that move near people. A flaw in it can
hurt someone, so we take reports seriously, and we would rather hear about a
problem privately before it is public.

## Supported versions

AXIS-OS has no release yet. Only the `main` branch is maintained; please
check that the problem still exists there.

## Reporting a vulnerability

**Please do not open a public issue.** Instead, either:

- write to **axis.project.dev@gmail.com**, with `[SECURITY]` at the start of
  the subject; or
- use GitHub's private reporting: tab **Security**, then **Report a
  vulnerability**.

Please include what you did, what happened, what you expected, the commit
you tested, and, if you can, a script or test that shows the problem. A
test in the style of `test/` ("the app tries, and fails") is the most
useful thing you can send.

The project is maintained by a very small team. We aim to acknowledge a
report within 7 days and to agree with you on a date before anything is
made public. You will be credited, unless you prefer not to be.

## What counts as a vulnerability

Anything that lets an app, an AI model's output, a network message or a
file read by the OS do something the design says it cannot. In particular:

- a motor command that reaches the hardware without going through
  `JointSafetyGuard`, or that the guard should have refused (invalid value,
  limit exceeded);
- a way to make the proximity guard, the watchdog or the emergency stop
  ignore a condition it is meant to act on;
- an app obtaining a permission it was not granted: a DANGEROUS permission
  without the user's confirmation, a SIGNATURE permission without the
  platform key;
- accepting an app with a forged or invalid signature, or reading an
  encrypted app without its key;
- running code in the OS process from outside it, or from an app (for
  example through the IPC bridge), except in the one accepted case: in
  simulation or development, an app installed as an already-loaded object
  was loaded into the OS process by the operator;
- an app installed under the PRODUCTION profile without a valid
  signature, with a key missing from the approved-keys file, or an app
  that changes the profile;
- a deployment script that runs downloaded code without checking it, or that
  can leak a token.

The known limits below are not new vulnerabilities, unless you find a way
around one of them that this page does not describe. Bugs without a safety
or security impact are welcome as ordinary issues.

## Known limits

These are weaknesses we know about and have not fixed yet. They are the
reason AXIS-OS is **not ready for a real robot**.

1. **App isolation against a malicious app is incomplete.** Each call to
   an app runs in a fresh Python interpreter, which inherits neither the
   OS memory, nor its HAL objects, nor its open connections: it only gets
   a socket to the OS, which carries only checked JSON data and no pickle
   objects any more. An app therefore can no longer get its code run in
   the OS process through this channel, nor reuse an OS connection to the
   motors. Under the PRODUCTION profile, an app's code is never loaded into
   the OS process, not even to read its manifest. Still open: the app
   process runs as the same user as the OS (it can read the OS files and
   those of other apps); it has network access (on a real G1 it could open
   its own DDS connection to the motors, outside `JointSafetyGuard`); its
   quotas are measured on its first process only, with no seccomp or
   cgroups. These are steps 2 to 4 of
   [docs/isolation_apps.md](docs/isolation_apps.md). **Until then, on a
   real robot, use the PRODUCTION profile and install only trusted apps
   whose code you have reviewed.**
2. **`ACTUATORS_AI_DRIVEN` is only partly checked at run time.** The
   permission tells the user that an AI model will drive the robot, and
   installing an app that declares it requires their confirmation. At run
   time, the commands produced by the SDK's AI actions (`VLAAction`,
   `GR00TAction`, `LocomotionAction`) carry an origin mark, and the OS
   refuses them to an app without the permission. But the OS cannot tell
   a number computed by a model from a hard-coded one: an app that builds
   its own commands from a model's output, without these actions, is not
   detected. The SDK's `SetpointFollower` passes the mark on when given it
   (`set_target(..., origine=ORIGINE_IA)`); `walk()` and `move_to_pose()`
   do not carry it.
3. **Some model weights, libraries and installers are not pinned.** OWL-ViT
   is loaded at a fixed revision and in the safetensors format. Whisper is
   loaded at the latest revision unless one is given, and `torch` (with
   `torchvision`) and `onnxruntime-gpu` are installed in their current
   version on the GPU machine, because the right version depends on its
   CUDA driver. On the same machine, Docker is installed with its official
   get.docker.com script, which publishes no checksum to check it against
   (`deploy/machine_gpu/installer.sh`). The NVIDIA Isaac-GR00T code, which
   fine-tunes the vision-language-action model and serves it during
   evaluation, is cloned at its latest commit, and no revision is fixed for
   the GR00T-N1.7-3B and Cosmos-Reason2-2B weights
   (`deploy/machine_gpu/entrainer_vla.sh`). A change, or a compromise, of
   those upstream files would be picked up without notice.
4. **Outside production, the only real robot recognised is the G1 over
   DDS.** `accepter_tout_SIMULATION` grants every DANGEROUS permission
   without asking, for simulation and tests. It is always refused under
   the PRODUCTION profile, and both the SIMULATION profile and this helper
   are refused with a recognised real robot. But the only real robot the
   OS recognises is the Unitree G1 driven over DDS: in simulation or
   development, a driver for another physical robot would not be detected.
   The profile is never guessed: choosing PRODUCTION on any real robot is
   the operator's job.
5. **Some permissions are declared but not enforced yet.** An app can
   declare `NETWORK_CLOUD`, `NETWORK_LOCAL`, `SENSORS_FORCE_TORQUE` and
   `FLEET_BROADCAST`, but no check depends on them at run time. The sandbox
   does not filter the network, so an app can open a network connection
   without `NETWORK_CLOUD` or `NETWORK_LOCAL`; `SENSORS_FORCE_TORQUE` and
   `FLEET_BROADCAST` relate to functions that `RobotContext` does not offer
   yet. Filtering the network is step 3 of
   [docs/isolation_apps.md](docs/isolation_apps.md).

6. **Some OS resources are not bounded per app yet.** An app's quotas are
   those of its manifest, with no ceiling set by the OS and no display at
   confirmation time. An e-stop refuses its commands but does not stop
   its mission, which keeps the robot until its time limit. Its persistent
   memory (`remember`) has no quota. A process started by the app that
   leaves its process group outlives the call: the OS reports it but
   cannot stop it (cgroups, step 4). Under the same user, an app can also
   send a signal to the OS process.
7. **App identity and life cycle in production.** An approved key can
   publish any `app_id` and any author name, and the confirmation shows
   the identity the app declares, not its signer. `install_source()` runs
   the signed code, without robot access, to read its manifest before the
   human confirmation. An installed app cannot be updated in production,
   and nothing prevents installing an older version after a restart.
8. **Voice intents are shared between apps.** The intent registry does not
   record which app declared what: an app can reserve another app's
   intent name in advance, and that app then receives the parameters the
   first one chose.

## Resolved limits

Weaknesses that earlier versions of this page listed, and what fixed them
(details in [CHANGELOG.md](CHANGELOG.md)):

- **Pickle on the IPC bridge.** A crafted object sent by an app could run
  code in the OS process when read. The bridge now carries only bounded,
  strictly decoded JSON frames, each checked against the schema of its
  action (`axis_os/store/runtime/canal_ipc.py`).
- **App process copied from the OS process (fork).** It inherited the OS
  memory, HAL objects and open connections, such as the DDS sockets of a
  real G1. Each call now runs in a fresh interpreter that receives only
  the bridge (`hote_app.py`, `processus_app.py`).
- **Signature optional everywhere.** Under the PRODUCTION profile it is
  mandatory, the developer key must be in the local approved-keys file,
  and `accepter_tout_SIMULATION` is refused (`axis_os/store/runtime/profil.py`).
- **`ACTUATORS_AI_DRIVEN` never checked at run time.** It now is for
  commands produced by the SDK's AI actions; see limit n° 2 for what
  remains.

---

# Politique de sécurité

AXIS-OS est fait pour piloter des robots qui bougent près des gens. Une
faille peut blesser quelqu'un : nous prenons les signalements au sérieux,
et nous préférons apprendre un problème en privé avant qu'il soit public.

## Versions maintenues

AXIS-OS n'a pas encore de version publiée. Seule la branche `main` est
maintenue : vérifiez que le problème y existe toujours.

## Signaler une faille

**N'ouvrez pas d'issue publique.** À la place :

- écrivez à **axis.project.dev@gmail.com**, avec `[SECURITY]` au début de
  l'objet ;
- ou utilisez le signalement privé de GitHub : onglet **Security**, puis
  **Report a vulnerability**.

Indiquez ce que vous avez fait, ce qui s'est passé, ce que vous attendiez,
le commit essayé et, si possible, un script ou un test qui montre le
problème. Un test dans le style de `test/` (« l'app essaie, et échoue ») est
ce qui nous aide le plus.

Le projet est maintenu par une toute petite équipe. Nous visons une
réponse sous 7 jours, et nous conviendrons avec vous d'une date avant toute
publication. Vous serez cité, sauf si vous préférez ne pas l'être.

## Ce qui compte comme une faille

Tout ce qui permet à une app, à la sortie d'un modèle d'IA, à un message
réseau ou à un fichier lu par l'OS de faire ce que la conception dit
impossible. En particulier :

- une commande moteur qui atteint le matériel sans passer par
  `JointSafetyGuard`, ou que le garde-fou aurait dû refuser (valeur
  invalide, limite dépassée) ;
- un moyen de faire ignorer au gardien de proximité, au watchdog ou à
  l'arrêt d'urgence une situation à laquelle il doit réagir ;
- une app qui obtient une permission qu'on ne lui a pas accordée : une
  permission DANGEROUS sans la confirmation de l'utilisateur, une
  permission SIGNATURE sans la clé plateforme ;
- une app acceptée avec une signature fausse ou invalide, ou une app
  chiffrée lue sans sa clé ;
- du code exécuté dans le processus de l'OS depuis l'extérieur, ou depuis
  une app (par exemple à travers le pont IPC), hors du seul cas assumé :
  en simulation ou en développement, une app installée comme objet déjà
  chargé l'a été par l'opérateur dans le processus de l'OS ;
- une app installée en profil PRODUCTION sans signature valide, avec une
  clé absente du fichier des clés approuvées, ou qui change de profil ;
- un script de déploiement qui exécute du code téléchargé sans le
  vérifier, ou qui peut laisser fuir un jeton.

Les limites connues ci-dessous ne sont pas de nouvelles failles, sauf si
vous trouvez un moyen de les contourner que cette page ne décrit pas. Les
bugs sans conséquence sur la sécurité sont bienvenus en issues ordinaires.

## Limites connues

Ce sont des faiblesses que nous connaissons et que nous n'avons pas encore
corrigées. C'est pour elles qu'AXIS-OS n'est **pas prêt pour un robot
réel**.

1. **L'isolation des apps face à une app malveillante est incomplète.**
   Chaque appel à une app tourne dans un interpréteur Python neuf, qui
   n'hérite ni de la mémoire de l'OS, ni de ses objets HAL, ni de ses
   connexions ouvertes : il ne reçoit qu'une socket vers l'OS, où ne
   passent que des données JSON vérifiées, et plus aucun objet pickle. Une
   app ne peut donc plus faire exécuter son code dans le processus de l'OS
   par ce canal, ni réutiliser une connexion de l'OS vers les moteurs. En
   profil PRODUCTION, le code d'une app n'est jamais chargé dans le
   processus de l'OS, pas même pour lire son manifeste. Restent ouverts :
   le processus de l'app tourne sous le même utilisateur que l'OS (il peut
   lire les fichiers de l'OS et ceux des autres apps) ; il a accès au
   réseau (sur un vrai G1, il pourrait ouvrir sa propre connexion DDS vers
   les moteurs, hors de `JointSafetyGuard`) ; ses quotas ne sont mesurés
   que sur son premier processus, sans seccomp ni cgroups. Ce sont les
   étapes 2 à 4 de [docs/isolation_apps.md](docs/isolation_apps.md). **D'ici
   là, sur un vrai robot, utilisez le profil PRODUCTION et n'installez que
   des apps de confiance, dont vous avez relu le code.**
2. **`ACTUATORS_AI_DRIVEN` n'est vérifiée qu'en partie à l'exécution.**
   Cette permission prévient l'utilisateur qu'un modèle d'IA pilotera le
   robot, et l'installation d'une app qui la déclare exige sa confirmation.
   À l'exécution, les consignes produites par les actions d'IA du SDK
   (`VLAAction`, `GR00TAction`, `LocomotionAction`) portent une marque
   d'origine, et l'OS les refuse à une app qui n'a pas la permission. Mais
   l'OS ne peut pas distinguer un nombre calculé par un modèle d'un nombre
   écrit en dur : une app qui fabrique elle-même ses consignes à partir
   d'un modèle, sans passer par ces actions, n'est pas détectée. Le
   `SetpointFollower` du SDK transmet la marque si on la lui donne
   (`set_target(..., origine=ORIGINE_IA)`) ; `walk()` et `move_to_pose()`
   ne la portent pas.
3. **Certains poids, bibliothèques et installeurs ne sont pas épinglés.**
   OWL-ViT est chargé à une révision fixe et au format safetensors. Whisper
   est chargé à sa dernière révision si on n'en donne pas, et `torch` (avec
   `torchvision`) et `onnxruntime-gpu` sont installés dans leur version du
   jour sur la machine GPU, parce que la bonne version dépend de son pilote
   CUDA. Sur la même machine, Docker est installé par son script officiel
   get.docker.com, qui ne publie aucune empreinte permettant de le vérifier
   (`deploy/machine_gpu/installer.sh`). Le code Isaac-GR00T de NVIDIA, qui
   affine le modèle vision-langage-action et le sert pendant l'évaluation,
   est cloné à son dernier commit, et aucune révision n'est fixée pour les
   poids GR00T-N1.7-3B et Cosmos-Reason2-2B
   (`deploy/machine_gpu/entrainer_vla.sh`). Une modification, ou une
   compromission, de ces fichiers en amont serait prise sans prévenir.
4. **Hors production, le seul robot réel reconnu est le G1 en DDS.**
   `accepter_tout_SIMULATION` accorde toutes les permissions DANGEROUS sans
   rien demander, pour la simulation et les tests. Elle est toujours
   refusée en profil PRODUCTION, et le profil SIMULATION comme cette
   fonction sont refusés sur un robot réel reconnu. Mais le seul robot réel
   que l'OS reconnaît est le Unitree G1 piloté en DDS : en simulation ou en
   développement, le driver d'un autre robot physique ne serait pas
   détecté. Le profil n'est jamais deviné : c'est à l'opérateur de choisir
   PRODUCTION sur tout vrai robot.
5. **Certaines permissions sont déclarées mais pas encore appliquées.** Une
   app peut déclarer `NETWORK_CLOUD`, `NETWORK_LOCAL`,
   `SENSORS_FORCE_TORQUE` et `FLEET_BROADCAST`, mais aucun contrôle n'en
   dépend à l'exécution. Le sandbox ne filtre pas le réseau : une app peut
   donc ouvrir une connexion réseau sans `NETWORK_CLOUD` ni
   `NETWORK_LOCAL` ; `SENSORS_FORCE_TORQUE` et `FLEET_BROADCAST` concernent
   des fonctions que `RobotContext` n'offre pas encore. Le filtrage du
   réseau est l'étape 3 de [docs/isolation_apps.md](docs/isolation_apps.md).

6. **Certaines ressources de l'OS ne sont pas encore bornées par app.** Les
   quotas d'une app sont ceux de son manifeste, sans plafond fixé par l'OS
   ni affichage au moment de la confirmation. Un e-stop refuse ses
   commandes mais n'interrompt pas sa mission, qui garde le robot jusqu'à
   son délai. Sa mémoire persistante (`remember`) n'a pas de quota. Un
   processus lancé par l'app qui quitte son groupe de processus survit à
   la fin de l'appel : l'OS le signale sans pouvoir l'arrêter (cgroups,
   étape 4). Sous le même utilisateur, une app peut aussi envoyer un
   signal au processus de l'OS.
7. **Identité et cycle de vie des apps en production.** Une clé approuvée
   peut publier n'importe quel `app_id` et n'importe quel nom d'auteur, et
   la confirmation affiche l'identité déclarée par l'app, pas son
   signataire. `install_source()` exécute le code signé, sans accès au
   robot, pour lire son manifeste avant la confirmation humaine. Une app
   installée ne peut pas être mise à jour en production, et rien
   n'empêche, après un redémarrage, d'installer une version plus ancienne.
8. **Les intentions vocales sont partagées entre apps.** Le registre des
   intentions ne retient pas quelle app a déclaré quoi : une app peut
   réserver d'avance le nom d'intention d'une autre, qui recevra alors les
   paramètres choisis par la première.

## Limites résolues

Faiblesses que de précédentes versions de cette page signalaient, et ce
qui les a corrigées (détails dans [CHANGELOG.md](CHANGELOG.md)) :

- **Pickle sur le pont IPC.** Un objet fabriqué par une app pouvait faire
  exécuter du code dans le processus de l'OS au moment de sa lecture. Le
  pont ne transporte plus que des trames JSON bornées et décodées
  strictement, chacune vérifiée par le schéma de son action
  (`axis_os/store/runtime/canal_ipc.py`).
- **Processus de l'app copié de celui de l'OS (fork).** Il héritait de la
  mémoire de l'OS, de ses objets HAL et de ses connexions ouvertes, comme
  les sockets DDS d'un vrai G1. Chaque appel tourne désormais dans un
  interpréteur neuf qui ne reçoit que le pont (`hote_app.py`,
  `processus_app.py`).
- **Signature facultative partout.** En profil PRODUCTION, elle est
  obligatoire, la clé du développeur doit figurer dans le fichier local
  des clés approuvées, et `accepter_tout_SIMULATION` est refusée
  (`axis_os/store/runtime/profil.py`).
- **`ACTUATORS_AI_DRIVEN` jamais vérifiée à l'exécution.** Elle l'est
  désormais pour les consignes produites par les actions d'IA du SDK ;
  voir la limite n° 2 pour ce qui reste.
