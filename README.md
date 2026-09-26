# AXIS-OS — Universal Humanoid Robot Operating System

> Un système d'exploitation open-source pour robots humanoïdes, conçu sur le
> modèle Android : un noyau universel indépendant du matériel, une couche de
> sécurité industrielle, et un système de distribution d'applications tierces.

---

## En une phrase

AXIS-OS sépare strictement **ce qui doit être universel** (sécurité, surveillance,
distribution d'applications) de **ce qui est spécifique à un robot donné**
(conventions articulaires, modèle d'équilibre), pour qu'un même OS puisse à
terme piloter des humanoïdes de fabricants différents sans réécrire son cœur.

---

## État réel du projet (honnête, pas un pitch)

Ce projet est au stade de **logiciel validé en simulation** (NVIDIA Isaac
Sim 6.1), **pas encore sur un robot physique**. Concrètement :

| Ce qui fonctionne en simulation | Ce qui n'existe pas encore |
|---|---|
| HAL, garde-fou articulaire, watchdog, sandbox, permissions, Play Store | Un robot physique branché |
| Unitree G1 **23 et 29 DOF**, même OS, mêmes apps | Navigation et cartographie (SLAM) |
| Marche, virage, arrêt sûr quand une app plante | Localisation par capteurs |
| Perception en vocabulaire ouvert, positions 3D à quelques cm | Un modèle vision-langage-action qui décide des gestes |
| Commande vocale — les phrases hors sujet ne déclenchent rien | Chaîne de démonstration et d'entraînement |
| **Saisie autonome** : trouver un carton, le prendre, l'emporter, le déposer | Mains à doigts |
| Plusieurs apps tierces sur un même robot, arbitrées par l'OS | Fleet Manager multi-robots |

**56 suites de tests automatisés** protègent chaque composant, et **16 suites
de validation** tournent dans Isaac Sim. Chaque résultat, **échecs compris**,
est documenté dans `docs/`.

### Documents clés

- **[Créer une app pour AXIS-OS, de A à Z](docs/CREER_UNE_APP.md)** — sans robot ni GPU
- [Ce qu'un robot doit avoir pour faire tourner AXIS-OS](docs/PREREQUIS_ROBOT.md)
- [Saisie et port autonomes d'un carton](docs/validation_port_carton.md)
- [Intégration de la politique GR00T Whole-Body Control](docs/integration_gr00t_wbc.md)
- [Plusieurs apps sur un même robot](docs/validation_multiapp.md)
- [Commande vocale](docs/validation_voix.md)
- [Prochaine démonstration : tri autonome en usine](docs/demo_tri_usine.md)

---

## Pourquoi ce projet existe

Aujourd'hui, chaque fabricant de robot humanoïde (Unitree, Agility Robotics,
Figure AI, Tesla) construit sa propre pile logicielle depuis zéro : sécurité,
gestion des applications, orchestration — tout est réinventé à chaque fois,
verrouillé dans l'écosystème du fabricant.

Android a résolu ce problème pour les téléphones il y a 15 ans : un noyau
commun, une interface matérielle standardisée (HAL), et un store
d'applications. AXIS-OS applique la même approche à la robotique humanoïde.

---

## Architecture

```
                    Applications (Play Store)
                            │
              SDK développeurs (RobotApp, RobotContext)
                            │
                    Sandbox (isolation processus)
                            │
              ┌─────────────┴─────────────┐
              │                           │
      JointSafetyGuard              WatchdogSystem
      (barrière avant HAL)          (surveillance continue)
              │                           │
              └─────────────┬─────────────┘
                            │
                  MotionStateTracker
              (historique commande/feedback)
                            │
                    IRobotHAL (interface universelle)
                            │
              ┌─────────────┴─────────────┐
              │                           │
      UnitreeG1HAL                 IsaacSimHAL / WebotsHAL
      (driver spécifique)          (backends de simulation)
```

Le principe directeur : **aucune commande n'atteint jamais le matériel sans
passer par les couches de sécurité**, quelle que soit son origine (application,
IA, ou modèle d'équilibre).

---

## Composants principaux

### Couche matérielle (HAL) — universelle
Interface commune (`IRobotHAL`) que chaque fabricant implémente pour son
robot. Le reste du système ne connaît jamais les particularités d'un modèle
donné (inversions d'articulations, offsets de calibration).

### Sécurité (`JointSafetyGuard`)
Dernière barrière avant toute commande moteur. Rejette explicitement les
valeurs invalides (NaN, infini), les dépassements de position/vitesse/couple,
et — avec un `MotionStateTracker` branché — les accélérations excessives.
Ne modifie jamais une commande : elle est acceptée ou rejetée, jamais
« corrigée » automatiquement.

### Surveillance (`WatchdogSystem`)
Thread indépendant qui surveille en continu température, communication, et
état des articulations. Déclenche un arrêt d'urgence global (e-stop)
idempotent en cas de condition critique, avec journal structuré de chaque
déclenchement (cause, valeurs, timestamp).

### Isolation applicative (`SandboxRunner`)
Chaque application tourne dans un processus séparé avec quotas CPU/RAM/temps.
Une application qui plante, boucle à l'infini, ou dépasse ses quotas est
arrêtée sans affecter le reste du système.

### Distribution d'applications (Play Store)
Système de manifeste déclaratif (permissions, matériel requis, quotas,
modèle de prix) et moteur d'exécution avec facturation automatique (30% de
commission plateforme, modèle inspiré des stores mobiles existants).

### Interfaces pour l'intelligence artificielle
- `IVLAConnector` : contrat pour brancher un modèle Vision-Language-Action
  (décision de haut niveau) dans une application.
- `IBalanceModel` : contrat pour brancher un modèle d'équilibre entraîné par
  simulation (spécifique à chaque robot, jamais dans le cœur de l'OS).

Les deux interfaces sont accompagnées d'implémentations de référence
(stubs déterministes) permettant de développer et tester sans dépendre d'un
modèle IA réel.

---

## Applications de démonstration

Deux applications complètes, écrites avec le SDK public, prouvent que le
système généralise au-delà d'un cas d'usage unique :

- **PackageSorting** — tri de colis, cas d'usage logistique classique.
- **SecurityPatrol** — ronde de sécurité nocturne autonome : génération de
  chemin aléatoire, détection de présence, vérification de badge, escalade
  d'alerte vers un serveur (jamais d'action d'urgence automatique — décision
  de sécurité volontaire).

---

## Suite de tests

```
221 tests automatisés répartis en 9 suites :

  test_axis_os.py             47 tests  — intégration HAL/Safety/Watchdog/Store
  test_sandbox.py              27 tests  — isolation, quotas, crash, timeout
  test_conventions.py          19 tests  — 24 articulations G1, round-trip
  test_watchdog_v2.py          17 tests  — e-stop, health state, faux positifs
  test_safety_v2.py            21 tests  — validation limites, fault injection
  test_motion_state.py         25 tests  — historique de mouvement, rate limiting
  test_security_patrol.py      23 tests  — application de sécurité complète
  test_balance_model.py        19 tests  — interface d'équilibre
  test_heavy_integration.py    23 vérifications — charge soutenue + pannes injectées
```

Le test d'intégration lourde fait tourner deux applications en alternance
sur 40+ missions, puis injecte des pannes réalistes (température critique,
perte de communication, donnée corrompue) en plein milieu de l'exécution
pour vérifier que le système reste cohérent sous stress, pas seulement dans
des conditions de test isolées et propres.

Exécution :
```bash
for f in test_*.py; do python3 "$f"; done
```

---

## Ce qui manque, honnêtement

Le principal obstacle technique n'est pas logiciel : **aucun robot
humanoïde ne tient debout sans un modèle d'équilibre entraîné par
apprentissage par renforcement en simulation physique** (approche utilisée
par Boston Dynamics, Figure AI, Unitree). Cette étape nécessite NVIDIA
Isaac Sim / Isaac Lab et une infrastructure GPU, actuellement en cours
d'acquisition.

Sans cette étape, AXIS-OS dispose d'une architecture de sécurité et de
distribution validée, mais d'aucune preuve qu'un robot physique peut
l'utiliser pour se déplacer de manière autonome.

---

## Pile technique

- Python 3.11+
- Aucune dépendance externe pour le cœur (HAL, Safety, Watchdog, Sandbox)
- `psutil` (optionnel) pour la surveillance CPU/RAM du sandbox
- Cible future : NVIDIA Isaac Sim / Isaac Lab pour la simulation physique et
  l'entraînement de modèles d'équilibre

---

## Licence

Cœur de l'OS (HAL, Safety, Watchdog) : Apache 2.0.
SDK applicatif : Apache 2.0.
Runtime Play Store (facturation) : propriétaire.

---

## Contact

Projet développé et maintenu de façon indépendante.
