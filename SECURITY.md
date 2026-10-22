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
   the OS process, and runs nowhere before the human accepts: its manifest
   is declared next to the code and signed with it. Still open: the app
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
   detected. The mark also goes through `SetpointFollower`
   (`set_target(..., origine=ORIGINE_IA)`), and through `walk()` and
   `move_to_pose()` when called via the SDK's actions
   (`GR00TAction.walk(ctx)`, `LocomotionAction.walk(ctx)`,
   `VLAAction.move_to_pose(ctx)`, `LocomotionAction.move_to_pose(ctx)`); an
   app that copies a model's numbers by hand into `ctx.walk()` or
   `ctx.move_to_pose()` does not carry it.
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

6. **Some OS resources are not bounded per app yet.** The OS answers a
   few calls with a model inference (`listen` transcription, `look_for`,
   `describe_scene`) that can run past the hook's deadline by the length
   of one inference; after an emergency stop, such a call already under
   way runs to its end in the background (its result is discarded),
   microphone or camera included. A process started by the app that
   leaves its process group outlives the call: the OS reports it but
   cannot stop it (cgroups, step 4). Under the same user, an app can
   also send a signal to the OS process. The persistent-memory quota is
   the same for every app (1,000 keys, 1 MiB), and an app can only
   overwrite a key, not delete it.
7. **App identity: trust on first install.** In production, an `app_id` is
   bound to the key that first installed it on this robot (app registry):
   an approved key can therefore still claim an `app_id` before its real
   publisher, and changing an app's key means editing the registry by
   hand. The registry and the approved-keys file are protected only by
   their Unix permissions: while apps run as the same user as the OS
   (limit n° 1), an app can rewrite the registry, which the OS must be
   able to rewrite too. The registry does not keep the price: a new price
   is confirmed on an update, but after a restart an installation is not
   compared with the price of the version installed before.

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
  commands produced by the SDK's AI actions, including the walking speeds
  and poses they pass through `walk()` and `move_to_pose()`; see limit
  n° 2 for what remains.
- **Quotas with no ceiling, not shown at confirmation (former limit
  n° 6).** The OS sets a ceiling for each manifest quota, stricter under
  PRODUCTION (`PLAFONDS_QUOTAS`, `axis_os/store/runtime/profil.py`); a
  manifest above one, or with a NaN, infinite, zero or negative quota, is
  refused at install time, never lowered. The human confirming the
  permissions sees the quotas (`test/test_quotas_plafonds.py`).
- **Emergency stop with no effect on the running mission (former limit
  n° 6).** An e-stop now interrupts the mission: the app receives it
  (`EstopActiveError` on its next call), its process is killed half a
  second later, even in the middle of a computation, and the mission is
  recorded as "interrupted by emergency stop", never billed; the robot
  stays stopped. `on_start` and `on_stop` called under e-stop are
  interrupted the same way (`test/test_arret_urgence_mission.py`).
- **Persistent memory with no quota (former limit n° 6).** `remember` is
  limited to 1,000 keys and 1 MiB per app, with a clear refusal
  (`MemoryService.app_store`).
- **App identity and life cycle in production (former limit n° 7).** The
  manifest is declared next to the code and signed with it
  (`signing.sign_app_package`): the OS reads it without running a line of
  the app before the human agrees. An `app_id` is bound to the key that
  first installed it (persistent registry, mandatory in production):
  another key can neither install nor update it. The confirmation shows
  the signing key (fingerprint, name recorded in the approved-keys file),
  not only the declared author, and the price; a new version that
  changes the pricing model or raises the price must be accepted by the
  human. Updates go through `update_source()`: same key, strictly newer
  version; the newest installed version survives a restart, and that
  version number is reinstalled only with the same signed content (its
  SHA-256 is in the registry) (`test/test_identite_apps.py`). Trust on
  first install remains: limit n° 7.
- **Voice intents shared between apps (former limit n° 8).** Each app has
  its own intent space and its own cap: it only receives its own intents
  and cannot reserve another app's name (`test/test_intentions_par_app.py`).

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
   processus de l'OS, et ne s'exécute nulle part avant que l'humain ait
   accepté : son manifeste est déclaré à côté du code et signé avec lui.
   Restent ouverts :
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
   d'un modèle, sans passer par ces actions, n'est pas détectée. La
   marque passe aussi par `SetpointFollower`
   (`set_target(..., origine=ORIGINE_IA)`), et par `walk()` et
   `move_to_pose()` appelés à travers les actions du SDK
   (`GR00TAction.walk(ctx)`, `LocomotionAction.walk(ctx)`,
   `VLAAction.move_to_pose(ctx)`, `LocomotionAction.move_to_pose(ctx)`) ;
   une app qui recopie à la main les nombres d'un modèle dans `ctx.walk()`
   ou `ctx.move_to_pose()` ne la porte pas.
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

6. **Certaines ressources de l'OS ne sont pas encore bornées par app.**
   L'OS répond à certains appels par une inférence de modèle
   (transcription de `listen`, `look_for`, `describe_scene`), qui peut
   dépasser l'échéance du hook de la durée d'un calcul ; après un arrêt
   d'urgence, un tel appel déjà commencé va à son terme en arrière-plan
   (son résultat est jeté), micro ou caméra compris. Un processus lancé
   par l'app qui quitte son groupe de processus survit à la fin de
   l'appel : l'OS le signale sans pouvoir l'arrêter (cgroups, étape 4).
   Sous le même utilisateur, une app peut aussi envoyer un signal au
   processus de l'OS. Le quota de mémoire persistante est le même pour
   toutes les apps (1 000 clés, 1 Mio), et une app ne peut que réécrire
   une clé, pas l'effacer.
7. **Identité des apps : confiance à la première installation.** En
   production, un `app_id` est lié à la clé qui l'a installé la première
   fois sur ce robot (registre des apps) : une clé approuvée peut donc
   encore réserver un `app_id` avant son véritable éditeur, et changer la
   clé d'une app demande de modifier le registre à la main. Le registre et
   le fichier des clés approuvées ne sont protégés que par leurs droits
   Unix : tant que les apps tournent sous le même utilisateur que l'OS
   (limite n° 1), une app peut réécrire le registre, que l'OS doit pouvoir
   réécrire lui aussi. Le registre ne garde pas le prix : un nouveau prix
   est confirmé lors d'une mise à jour, mais après un redémarrage une
   installation n'est pas comparée au prix de la version installée
   avant.

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
  désormais pour les consignes produites par les actions d'IA du SDK, y
  compris les vitesses de marche et les poses qu'elles transmettent par
  `walk()` et `move_to_pose()` ; voir la limite n° 2 pour ce qui reste.
- **Quotas sans plafond, et invisibles à la confirmation (ancienne
  limite n° 6).** L'OS fixe un plafond pour chaque quota du manifeste,
  plus strict en PRODUCTION (`PLAFONDS_QUOTAS`,
  `axis_os/store/runtime/profil.py`) ; un manifeste qui en dépasse un, ou
  dont un quota est NaN, infini, nul ou négatif, est refusé à
  l'installation, jamais ramené. L'humain qui confirme les permissions
  voit les quotas (`test/test_quotas_plafonds.py`).
- **Arrêt d'urgence sans effet sur la mission en cours (ancienne limite
  n° 6).** Un e-stop interrompt désormais la mission : l'app le reçoit
  (`EstopActiveError` à son prochain appel), son processus est tué une
  demi-seconde plus tard, même en plein calcul, et la mission est notée
  « interrompue par arrêt d'urgence », jamais facturée ; le robot reste
  arrêté. `on_start` et `on_stop` appelés sous e-stop sont interrompus de
  même (`test/test_arret_urgence_mission.py`).
- **Mémoire persistante sans quota (ancienne limite n° 6).** `remember`
  est limitée à 1 000 clés et 1 Mio par app, avec un refus clair
  (`MemoryService.app_store`).
- **Identité et cycle de vie des apps en production (ancienne limite
  n° 7).** Le manifeste est déclaré à côté du code et signé avec lui
  (`signing.sign_app_package`) : l'OS le lit sans exécuter une ligne de
  l'app avant l'accord de l'humain. Un `app_id` est lié à la clé qui l'a
  installé la première fois (registre persistant, obligatoire en
  production) : une autre clé ne peut ni l'installer ni le mettre à jour.
  La confirmation montre la clé signataire (empreinte, nom inscrit dans le
  fichier des clés approuvées), pas seulement l'auteur déclaré, et le
  prix ; une nouvelle version qui change le modèle de prix ou augmente le
  prix doit être acceptée par l'humain. Les mises à jour passent par
  `update_source()` : même clé, version strictement plus récente ; la
  version la plus récente installée survit au redémarrage, et ce numéro
  ne se réinstalle qu'avec le même contenu signé (son SHA-256 est au
  registre) (`test/test_identite_apps.py`). Reste la confiance à la
  première installation : limite n° 7.
- **Intentions vocales partagées entre apps (ancienne limite n° 8).**
  Chaque app a son propre espace d'intentions et son propre plafond : elle
  ne reçoit que les siennes et ne peut pas réserver le nom d'une autre
  (`test/test_intentions_par_app.py`).
