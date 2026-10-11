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

- use GitHub's private reporting: tab **Security**, then
  **Report a vulnerability**; or
- write to **axis.project.dev@gmail.com**, with `[SECURITY]` at the start of
  the subject.

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
- running code in the OS process from outside it, or from an app, by a path
  other than the known limit n° 1 below;
- a deployment script that runs downloaded code without checking it, or that
  can leak a token.

The known limits below are not new vulnerabilities, unless you find a way
around one of them that this page does not describe. Bugs without a safety
or security impact are welcome as ordinary issues.

## Known limits

These are weaknesses we know about and have not fixed yet. They are the
reason AXIS-OS is **not ready for a real robot**.

1. **Apps are not isolated from a malicious app's point of view.** An app
   runs in its own process, but that process starts as a copy of the OS
   process: on a real G1 it inherits the open connections to the robot's
   motors (DDS). The channel between the app and the OS also exchanges
   Python objects (pickle), not plain data. A deliberately hostile app can
   therefore act outside the HAL, and so outside `JointSafetyGuard`. The
   sandbox does protect against buggy apps (crashes, infinite loops,
   quotas). The fix is planned in [docs/isolation_apps.md](docs/isolation_apps.md):
   its first step is a channel that only carries data, with no pickle.
   **Until then, install only trusted apps whose code you have reviewed.**
2. **`ACTUATORS_AI_DRIVEN` is confirmed at install time, but not checked at
   run time.** The permission tells the user that an AI model will drive
   the robot, and installing an app that declares it requires their
   explicit confirmation. But the OS cannot yet tell whether a given command
   comes from a model or from fixed code: an app that runs a model without
   declaring the permission is not stopped.
3. **Some model weights and libraries are not pinned.** OWL-ViT is loaded at
   a fixed revision and in the safetensors format. Whisper is loaded at the
   latest revision unless one is given, and `torch` and `onnxruntime-gpu`
   are installed in their current version on the GPU machine, because the
   right version depends on its CUDA driver. A change, or a compromise, of
   those upstream files would be picked up without notice.
4. **`accepter_tout_SIMULATION` only recognises one real robot.** This
   helper grants every DANGEROUS permission without asking, for simulation
   and tests. The Play Store refuses to use it with a real robot, but the
   only real robot it recognises today is the Unitree G1 driven over DDS. A
   driver for another physical robot would not be detected.

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

- utilisez le signalement privé de GitHub : onglet **Security**, puis
  **Report a vulnerability** ;
- ou écrivez à **axis.project.dev@gmail.com**, avec `[SECURITY]` au début
  de l'objet.

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
  une app, par un autre chemin que la limite connue n° 1 ci-dessous ;
- un script de déploiement qui exécute du code téléchargé sans le
  vérifier, ou qui peut laisser fuir un jeton.

Les limites connues ci-dessous ne sont pas de nouvelles failles, sauf si
vous trouvez un moyen de les contourner que cette page ne décrit pas. Les
bugs sans conséquence sur la sécurité sont bienvenus en issues ordinaires.

## Limites connues

Ce sont des faiblesses que nous connaissons et que nous n'avons pas encore
corrigées. C'est pour elles qu'AXIS-OS n'est **pas prêt pour un robot
réel**.

1. **Les apps ne sont pas isolées face à une app malveillante.** Une app
   tourne dans son propre processus, mais ce processus démarre comme une
   copie de celui de l'OS : sur un vrai G1, il hérite des connexions
   ouvertes vers les moteurs du robot (DDS). Le canal entre l'app et l'OS
   échange en outre des objets Python (pickle), pas de simples données.
   Une app délibérément hostile peut donc agir en dehors de la HAL, et donc
   de `JointSafetyGuard`. Le sandbox protège bien contre les apps buggées
   (plantage, boucle infinie, quotas). La correction est planifiée dans
   [docs/isolation_apps.md](docs/isolation_apps.md) : sa première étape est
   un canal qui ne transporte que des données, sans pickle. **D'ici là,
   n'installez que des apps de confiance, dont vous avez relu le code.**
2. **`ACTUATORS_AI_DRIVEN` est confirmée à l'installation, mais pas
   vérifiée à l'exécution.** Cette permission prévient l'utilisateur qu'un
   modèle d'IA pilotera le robot, et l'installation d'une app qui la
   déclare exige sa confirmation explicite. Mais l'OS ne sait pas encore
   dire si une commande donnée vient d'un modèle ou de code fixe : une app
   qui fait tourner un modèle sans déclarer la permission n'est pas
   arrêtée.
3. **Certains poids et bibliothèques ne sont pas épinglés.** OWL-ViT est
   chargé à une révision fixe et au format safetensors. Whisper est chargé
   à sa dernière révision si on n'en donne pas, et `torch` et
   `onnxruntime-gpu` sont installés dans leur version du jour sur la
   machine GPU, parce que la bonne version dépend de son pilote CUDA. Une
   modification, ou une compromission, de ces fichiers en amont serait
   prise sans prévenir.
4. **`accepter_tout_SIMULATION` ne reconnaît qu'un seul robot réel.** Cette
   fonction accorde toutes les permissions DANGEROUS sans rien demander,
   pour la simulation et les tests. Le Play Store refuse de l'utiliser avec
   un robot réel, mais le seul robot réel qu'il reconnaît aujourd'hui est
   le Unitree G1 piloté en DDS. Le driver d'un autre robot physique ne
   serait pas détecté.
