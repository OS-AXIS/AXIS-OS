# Validation de la locomotion — 2026-09-22

Mesures réelles contre Isaac Sim 6.1.0, G1 23 DOF importé depuis l'URDF
officiel AXIS-OS, géométrie et collisions complètes. Reproductible via
`test_isaac_l1_locomotion.py`.

---

## 1. Résultat

**Le G1 marche dans Isaac Sim, à travers AXIS-OS, sous le contrôle de
`JointSafetyGuard`.** Première marche du projet dans une simulation
physique fidèle.

Politique : officielle Unitree (`unitree_rl_gym` @ 276801e,
`deploy/pre_train/g1/motion.pt`, BSD 3-Clause, SHA-256 vérifié au
chargement). Entraînée dans Isaac Gym, vérifiée dans MuJoCo, **jamais
dans Isaac Sim** avant ce test.

Protocole : 3 s debout (consigne nulle), puis 5 s de marche à 0.5 m/s.

| | A — politique seule | B — via le garde-fou |
|---|---|---|
| Chute | aucune | aucune |
| Distance en marche | **2.17 m** | **2.15 m** |
| Hauteur min. du bassin | 0.758 m | 0.757 m |
| Consignes refusées | — | 52 (chevilles) |

**Le coût du garde-fou sur la marche est de l'ordre de 1 %.** Les deux
phases atteignent ~87 % de la distance idéale (2.5 m) : l'écart vient de
la politique et de la simulation (accélération au départ, moteur de
contact différent de l'entraînement), pas d'AXIS-OS.

---

## 2. Trois pièges silencieux, traités avant le premier essai

Une politique entraînée est une clé : tout écart avec l'entraînement
fait tomber le robot, sans erreur. Chacun a été vérifié hors simulateur
(`test/test_g1_locomotion_policy.py`).

**Repère de la vitesse angulaire.** Isaac Sim la rend dans le repère du
monde, la politique l'attend dans celui du robot (convention MuJoCo).
Sans conversion, tout marche droit devant, puis le robot tombe dès qu'il
tourne. Conversion vérifiée sur un quart de tour.

**Convention des angles.** La politique travaille en angles bruts de
l'URDF, pas dans la convention normalisée d'AXIS-OS. Vérifié : axes,
origines et limites de position identiques sur les 12 articulations
entre l'URDF d'entraînement et celui d'AXIS-OS.

**Pas de temps.** Politique entraînée à 500 Hz de physique ; Isaac Sim
tourne par défaut à 60 Hz. `setup(physics_dt=0.002)`.

Et la gravité projetée est **identique** à la formule d'Unitree sur 1000
orientations aléatoires (écart 7.8 × 10⁻¹⁶).

---

## 3. Unitree se contredit sur les chevilles

Même nom de fichier, `g1_23dof_rev_1_0.urdf`, deux dépôts officiels,
deux valeurs d'effort pour la même articulation :

| Source | Cheville | Genou | Hanche |
|---|---|---|---|
| `unitree_ros` (source de vérité AXIS-OS) | **35 N·m** | 139 | 88 |
| `unitree_rl_gym` (entraînement de la politique) | **50 N·m** | 139 | 88 |

AXIS-OS garde **35 N·m**, la valeur prudente. **Résultat : suffisant pour
marcher droit à 0.5 m/s en simulation.** La contradiction ne bloque pas
la marche, mais reste à trancher par la fiche technique du moteur avant
tout essai réel — elle pourrait compter pour des allures plus rapides,
des virages serrés ou des perturbations.

---

## 4. La politique vise au-delà des butées

Le garde-fou a refusé 52 consignes, toutes des chevilles hors limites de
position : jusqu'à 0.326 rad pour un roulis limité à 0.262, soit 25 % de
dépassement ; un pas sur dix pour la cheville gauche.

Comportement connu des politiques apprises : à l'entraînement, viser
au-delà d'une butée ne coûte rien, la physique bloque l'articulation.

**Décision d'architecture :**
- `JointSafetyGuard` **reste strict** pour toutes les apps : un
  dépassement est refusé, jamais corrigé en silence.
- L'**adaptateur de locomotion** ramène ses propres consignes aux butées
  avant envoi — ce qui correspond exactement à ce que la politique a
  vécu à l'entraînement — et **compte** chaque bornage (`clamp_counts`).
- L'action **réinjectée** dans l'observation reste brute : à
  l'entraînement, le réseau voyait sa sortie non bornée.

Motivation : pas la performance (refuser ou borner, 1 % d'écart), mais
le **journal de sécurité**. Plus de six violations bénignes par seconde
de marche noieraient une violation réellement dangereuse. Un journal de
sécurité qui crie tout le temps n'avertit plus de rien.

**Résultat après bornage :**

| | A — politique seule | B — via le garde-fou |
|---|---|---|
| Chute | aucune | aucune |
| Distance en marche | 2.16 m | 2.16 m |
| Consignes bornées (adaptateur) | 59 | 53 |
| Consignes refusées (garde-fou) | — | **aucune** |

A et B sont désormais **identiques au centimètre** : avec le refus, B
perdait 2 cm ; avec le bornage, la marche sous garde-fou se comporte
exactement comme la politique seule. Les bornages existent toujours —
ils sont comptés là où ils sont compris, au lieu de remplir le journal
de sécurité.

---

## 5. Virage — expérience témoin

La conversion du repère de la vitesse angulaire avait été faite AVANT le
premier essai, sur la base d'un raisonnement. La ligne droite ne pouvait
pas la tester. `test_isaac_l2_turning.py` fait tourner le robot deux
fois (0.3 m/s en avançant, 0.6 rad/s), dont une fois en fournissant
délibérément à la politique la vitesse angulaire dans le repère du monde.

| | virage | chute | refus du garde-fou |
|---|---|---|---|
| **Avec** conversion (code réel) | 272° sur ~275° | aucune | 0 |
| **Sans** conversion (témoin) | 103° | **à t = 6.74 s** | 3 |

**La correction était indispensable — démontré par la mesure.**

Le point de chute est lui-même une confirmation : 103°, juste après le
quart de tour, là où la théorie prédit l'erreur maximale — à 90°, les
vitesses de roulis et de tangage sont intervertis. Le robot tient tant
que l'erreur est faible et tombe précisément quand elle devient totale.

> Un test qui ne peut pas détecter le bug qu'il prétend couvrir ne prouve
> rien. La phase témoin est ce qui donne sa valeur à la phase réelle.

## 6. Limites

- **Simulation seulement.** Sol plat, 5 à 8 s, une allure.
- **Pas de perturbation** : aucune poussée, aucun obstacle, aucun sol
  irrégulier.
- **Moteur de contact différent** de l'entraînement : un succès dans
  Isaac Sim ne garantit rien sur le matériel, où les chevilles, la
  latence réelle et la calibration des encodeurs entrent en jeu (voir
  `campagne_validation_isaac.md` §9).
