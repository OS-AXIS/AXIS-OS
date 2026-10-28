# AXIS-OS

[![Tests](https://github.com/OS-AXIS/AXIS-OS/actions/workflows/tests.yml/badge.svg)](https://github.com/OS-AXIS/AXIS-OS/actions/workflows/tests.yml)
[![Licence : Apache 2.0](https://img.shields.io/badge/licence-Apache%202.0-blue.svg)](LICENSE)

*English version: [README.md](README.md).*

AXIS-OS est un système d'exploitation open source pour robots humanoïdes.
Comme Android pour les téléphones, il sépare ce dont tout robot a besoin
(sécurité, services système, magasin d'applications avec permissions) de ce
qui est propre à un robot (un driver matériel), pour qu'un jour les mêmes
applications tournent sur des robots de fabricants différents. Aujourd'hui,
il pilote un Unitree G1 **en simulation seulement** (NVIDIA Isaac Sim) : il
n'a encore jamais commandé un robot réel.

## Le principe : l'IA propose, le code décide, l'OS protège

Les modèles d'IA sont bons pour percevoir et suggérer, et ils peuvent se
tromper d'une façon que personne n'a prévue. AXIS-OS ne laisse jamais la
sortie d'un modèle atteindre directement un moteur.

1. **L'IA propose.** Les modèles produisent des suggestions : où se trouve
   un carton (OWL-ViT et profondeur), ce qu'a dit un opérateur (Whisper),
   comment garder l'équilibre en marchant (GR00T Whole-Body Control), quelle
   règle changer (un modèle de langage).
2. **Le code décide.** Du code simple et relu, celui de l'app ou du
   service, accepte ou refuse chaque suggestion selon des règles écrites.
   Exemple : quand un opérateur dit « les colis bleus vont maintenant au
   poste C », le superviseur de tri traite la réponse du modèle de langage
   comme une donnée non fiable, et ne l'applique que si l'action, la
   couleur et le poste figurent dans une liste fermée. Il n'a aucun accès
   aux bras, aux jambes ni à la sécurité : même un ordre de mouvement
   « halluciné » n'a aucun moyen de s'exécuter.
3. **L'OS protège.** Chaque commande moteur passe par des contrôles que ni
   les apps ni les modèles ne maîtrisent : `JointSafetyGuard` rejette les
   valeurs invalides (NaN, infini) et tout dépassement des limites de
   position, de vitesse, de couple ou d'accélération de chaque
   articulation, sans jamais « corriger » une commande en silence ; le
   gardien de proximité, quand un lidar lui est branché (facultatif ;
   aujourd'hui dans la démo de tri seulement), ralentit puis arrête le robot
   dès que ce lidar voit un obstacle inexpliqué à proximité, qu'un modèle
   l'ait reconnu ou non comme une personne ; le watchdog déclenche un arrêt
   d'urgence quand une articulation franchit un seuil critique, et un
   superviseur distinct arrête le robot si le watchdog lui-même cesse de
   tourner ; les permissions limitent ce que chaque app a le droit de faire.

Ce dernier point vaut pour les apps qui respectent les règles. L'isolation
contre une app délibérément malveillante n'est pas terminée : voir
[Statut et limites](#statut-et-limites).

## Architecture

```mermaid
---
config:
  layout: dagre
  fontFamily: "-apple-system, BlinkMacSystemFont, 'SF Pro Text', 'Helvetica Neue', Helvetica, Arial, sans-serif"
  flowchart:
    wrappingWidth: 320
  themeCSS: ".node rect, .cluster rect { rx: 14px; ry: 14px; } .edgeLabel foreignObject { overflow: visible; } .labelBkg { background: none; } .labelBkg .edgeLabel { display: inline-block; padding: 0 8px; border-radius: 999px; background: #ffffff; border: 1px solid #d2d2d7; color: #3a3a3c; } .labelBkg .edgeLabel p { background: none; margin: 0; } .labelBkg .edgeLabel:empty { display: none; } .flowchart-link { stroke: #8e8e93; stroke-width: 1.3px; } .marker { fill: #8e8e93; stroke: #8e8e93; } .edge-animation-fast { stroke: #007aff; stroke-dasharray: 0 20 !important; stroke-width: 5px; } @media (prefers-reduced-motion: reduce) { .edge-animation-fast { animation: none; } }"
---
flowchart TB
    PS("<b>Play Store</b><br/>✓ signature, obligatoire en production<br/>✓ app_id lié à sa clé (production)<br/>✓ permissions, quotas plafonnés<br/>✓ confirmation des permissions DANGEROUS")

    A("<b>App tierce</b> · RobotApp<br/>processus neuf, en sandbox<br/>quotas de CPU, de mémoire et de temps<br/>arrêtée par un arrêt d'urgence<br/>⚠ isolation incomplète, voir SECURITY.md")

    subgraph OS["Processus d'AXIS-OS"]
        CTX("<b>RobotContext</b><br/>✓ permission de chaque appel<br/>✓ refus pendant un arrêt d'urgence<br/>✓ plafond de vitesse")
        SV("<b>Services système</b><br/>locomotion · navigation · perception<br/>manipulation · parole · mémoire · VLA<br/>personnes · suis-moi (sans reconnaissance faciale)")
        G("<b>Gardien de proximité</b> · lidar facultatif<br/>✓ ralentit, puis arrête la marche")
        N("<b>NormalizedRobotHAL</b><br/>✓ refuse les angles NaN ou infinis")
        D("<b>Driver du robot</b> · IRobotHAL<br/>Unitree G1 · Isaac Sim")
        J("<b>JointSafetyGuard</b> · chaque articulation<br/>✓ position · vitesse · couple<br/>✓ accélération · taille du pas")
        subgraph MON["Surveillance en parallèle"]
            W("<b>Watchdog</b><br/>✓ seuil fatal → arrêt d'urgence")
            SU("<b>Superviseur</b><br/>✓ watchdog figé → arrêt d'urgence")
            IM("<b>Moniteur indépendant</b><br/>✓ ses propres limites → arrêt d'urgence")
        end
    end

    R(["<b>Moteurs du robot</b>"])

    PS e1@-->|installe| A
    A e2@-->|"pont IPC · JSON vérifié"| CTX
    CTX e3@--> SV
    CTX e4@--> N
    SV e5@-->|marche| G
    G e6@--> N
    SV e7@--> N
    N e8@--> D
    D e9@--> J
    J e10@--> R
    SU -.->|surveille| W
    D -.-|"lisent les retours, peuvent tout arrêter"| MON

    e1@{ animate: true }
    e2@{ animate: true }
    e3@{ animate: true }
    e4@{ animate: true }
    e5@{ animate: true }
    e6@{ animate: true }
    e7@{ animate: true }
    e8@{ animate: true }
    e9@{ animate: true }
    e10@{ animate: true }
    classDef app fill:#eaf3ff,stroke:#007aff,stroke-width:1.5px,color:#1d1d1f
    classDef check fill:#eafaf0,stroke:#34c759,stroke-width:1.5px,color:#1d1d1f
    classDef watch fill:#fff5e6,stroke:#ff9500,stroke-width:1.5px,color:#1d1d1f
    classDef plain fill:#ffffff,stroke:#c7c7cc,stroke-width:1.5px,color:#1d1d1f
    classDef motor fill:#1d1d1f,stroke:#1d1d1f,color:#ffffff
    class A app
    class PS,CTX,G,N,J check
    class W,SU,IM watch
    class SV,D plain
    class R motor
    style OS fill:#f5f5f7,stroke:#d2d2d7,color:#1d1d1f
    style MON fill:#fffaf2,stroke:#ffd59e,color:#1d1d1f
```

En vert, les contrôles de sécurité (chaque ✓) : ni les apps ni les
modèles d'IA ne les commandent. En orange, ce qui surveille le robot en
parallèle et peut l'arrêter à tout moment. Le processus de l'app n'hérite
de rien de l'OS et n'échange avec lui que du JSON vérifié ; contre une app
malveillante, l'isolation reste incomplète (fichiers, réseau) : voir
[Statut et limites](#statut-et-limites).

Un service, c'est ce dont deux apps de métiers différents auraient toutes
les deux besoin (marcher, voir, saisir) ; ce qui est propre à un métier est
une app. La conception est expliquée dans
[docs/architecture_services.md](docs/architecture_services.md).

| Dossier | Contenu |
|---------|---------|
| `axis_os/hal/` | abstraction matérielle, garde-fou articulaire, watchdog, superviseur, drivers (Unitree G1, simulation) |
| `axis_os/services/` | services système partagés par toutes les apps |
| `axis_os/store/` | Play Store : manifeste et permissions, signature, sandbox, SDK des développeurs d'apps |
| `axis_os/apps/` | apps de référence (tri de colis, ronde de sécurité, inventaire…) |
| `axis_os/vla/` | enregistrement de démonstrations et évaluation d'un modèle vision-langage-action |
| `test/` | la suite de tests, qui tourne partout sans GPU |
| `sim/` | scripts de validation qui tournent dans Isaac Sim, sur une machine GPU |
| `deploy/` | installation de la machine GPU, service systemd, moniteur de fraîcheur matériel |
| `docs/` | notes de conception et rapports de validation |

## Ce qui marche

Tout ce qui suit a été mesuré **en simulation**, dans NVIDIA Isaac Sim 6.1
avec un Unitree G1. Chaque résultat, échecs compris, est décrit dans
`docs/`.

| Capacité | Preuve |
|----------|--------|
| Le même OS et les mêmes apps sur les versions 23 et 29 degrés de liberté du G1 | suite de tests |
| Marche, virage, et arrêt sûr quand une app plante en pleine marche | [validation_locomotion.md](docs/validation_locomotion.md) |
| Détection d'objets à vocabulaire ouvert (OWL-ViT) ; distance à la face visible d'un carton mesurée au centimètre | [validation_perception.md](docs/validation_perception.md) |
| Commande vocale ; les phrases hors sujet ne déclenchent rien | [validation_voix.md](docs/validation_voix.md) |
| Plusieurs apps tierces sur un même robot, arbitrées par l'OS | [validation_multiapp.md](docs/validation_multiapp.md) |
| Prendre un carton d'un kilo sur une table et l'emporter, sans une seule commande refusée par le garde-fou ; puis le trouver sans qu'on lui dise où il est (chacun montré par un déroulé réussi d'un script de test, pas encore dans une app ; le carton glisse lentement, 8 cm en 11 s) | [validation_port_carton.md](docs/validation_port_carton.md) |
| Une séance de tri de colis de 30 minutes avec des personnes qui circulent, répétée sur une simulation cinématique sans GPU : le robot ne s'est jamais approché d'une personne à moins de 0,75 m | [demo_tri_usine.md](docs/demo_tri_usine.md), `test/test_tri_pile_logicielle.py` |

Dans ces démonstrations, l'IA perçoit et tient l'équilibre ; les gestes des
bras sont encore programmés.

## Statut et limites

AXIS-OS est un logiciel de recherche. À lire avant de l'utiliser.

- **Simulation seulement. Pas prêt pour un robot réel.** Le driver du
  Unitree G1 parle le vrai protocole du robot (DDS), mais il n'a jamais été
  branché sur un robot physique. Un comportement sûr en simulation n'est
  pas automatiquement sûr sur le matériel.
- **L'isolation des apps n'est pas terminée.** Chaque app tourne dans un
  processus neuf, qui n'hérite de rien de l'OS et ne lui parle que par du
  JSON vérifié. Mais ce processus tourne encore sous le même utilisateur et
  a accès au réseau : une app hostile pourrait lire les fichiers de l'OS
  et, sur un vrai G1, ouvrir sa propre connexion DDS vers les moteurs.
  **Sur un vrai robot, utilisez le profil PRODUCTION et n'installez que des
  apps de confiance, dont vous avez relu le code.** Le plan est dans
  [docs/isolation_apps.md](docs/isolation_apps.md), le détail dans
  [SECURITY.md](SECURITY.md).
- **La saisie pilotée par un modèle vision-langage-action est en cours.**
  Un modèle GR00T N1.7 affiné sur des démonstrations simulées est évalué
  face aux réflexes programmés ; il n'est pas encore fiable.
- **Pas encore de localisation par capteurs** : en simulation, la position
  du robot est lue dans le simulateur. Pas de cartographie (SLAM), pas de
  mains à doigts, pas de gestion de flotte (seulement une note de
  conception).
- La démonstration de tri de 30 minutes a été répétée sur CPU ; son
  exécution complète dans Isaac Sim reste à faire.

## Lancer les tests (sans GPU)

La suite de tests vérifie la couche de sécurité, le Play Store, les
services et les apps de référence, sans GPU, sans simulateur et sans aucun
modèle d'IA. Elle prend environ 7 minutes sur une machine à 4 cœurs.

```sh
git clone https://github.com/OS-AXIS/AXIS-OS.git
cd AXIS-OS
python3 -m venv .venv
. .venv/bin/activate
pip install --require-hashes -r requirements-ci.txt
pytest test/
```

Python 3.12 ou plus récent (la CI teste 3.12 ; essayé aussi en local avec
3.13). Chaque fichier de `test/` est aussi un script autonome :
`python3 test/test_safety_v2.py` en lance un et affiche chaque
vérification. La même suite tourne à chaque push
(`.github/workflows/tests.yml`).

Les scripts de simulation demandent un GPU NVIDIA et le conteneur Isaac
Sim : voir [sim/README.md](sim/README.md) et `deploy/machine_gpu/`.

## Pour aller plus loin

- [Créer une app pour AXIS-OS, de A à Z](docs/CREER_UNE_APP.md), sans robot ni GPU
- [Référence des API du SDK](docs/API_REFERENCE.md) (en anglais) : chaque API, sa permission,
  ses codes de refus, un exemple ; [quatre apps d'exemple complètes](examples/), dont « suis-moi »
- [Ce qu'un robot doit avoir pour faire tourner AXIS-OS](docs/PREREQUIS_ROBOT.md)
- [Intégration de GR00T Whole-Body Control](docs/integration_gr00t_wbc.md)
- [Isolation des apps : constat et plan](docs/isolation_apps.md)
- [Glossaire français-anglais des termes du code](docs/GLOSSARY.md)

## Licence

Licence Apache 2.0 (voir [LICENSE](LICENSE)). Les fichiers tiers inclus
dans le dépôt sont listés dans [NOTICE](NOTICE). Aucun poids de modèle
n'est inclus ; les modèles qu'AXIS-OS peut utiliser, et leurs licences,
dont certaines ne sont pas libres, sont listés dans
[THIRD_PARTY.md](THIRD_PARTY.md).

## Contribuer et sécurité

Les contributions sont bienvenues : voir [CONTRIBUTING.md](CONTRIBUTING.md)
et le [code de conduite](CODE_OF_CONDUCT.md). Pour signaler une faille, ne
créez pas d'issue publique : suivez [SECURITY.md](SECURITY.md).

## Contact

axis.project.dev@gmail.com
