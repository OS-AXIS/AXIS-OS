# Latence du chemin de commande — mesures du 2026-09-19

Mesures réelles contre Isaac Sim 6.1.0, G1 23 DOF importé depuis l'URDF
officiel AXIS-OS. Reproductibles via `sim/validation/test_isaac_follower_latency.py` et
`sim/validation/test_s5_latency_probe.py`.

Ce document existe pour une raison précise : **il tranche par la mesure
une question d'architecture qui a été posée deux fois** — faut-il un
« chemin rapide » qui contournerait `JointSafetyGuard` pour tenir les
hautes fréquences ? La réponse est non, et ces chiffres le prouvent.

> **Mesures à refaire.** Elles datent de l'ancien pont, deux
> `multiprocessing.Queue` sérialisées par pickle. Depuis le 2026-10-20, le
> pont passe par une socket Unix et chaque message est encodé puis vérifié
> en JSON (voir `docs/isolation_apps.md`, étapes 0 et 1) : le surcoût de
> l'isolation a changé et n'est pas encore remesuré sur Isaac Sim.

---

## 1. Chemin direct — sans sandbox

`sim/validation/test_isaac_follower_latency.py`, 200 pas, budget 20 ms (50 Hz).

| | moyenne | max |
|---|---|---|
| cycle `SetpointFollower` | **5.41 ms** | 7.57 ms |

1600 commandes envoyées, **0 refusée**. Chemin complet : interpolation →
`move_arms()`/`move_legs()` → `JointSafetyGuard` → `apply_action` +
`step`.

Reproductible : 5.39 ms puis 5.41 ms sur deux exécutions.

## 2. Chemin sandboxé — pile complète

`sim/validation/test_s5_latency_probe.py` : sous-process → `Queue` → `HALBridgeServer`
(thread) → `RobotContext` → `NormalizedRobotHAL` → `JointSafetyGuard` →
`MainThreadSimPump` → thread principal → retour.

200 cycles, 4 joints par cycle, **0 commande refusée**.

| | moyenne | p50 | p95 | max |
|---|---|---|---|---|
| `get_full_state()` | 1.25 ms | 1.15 | 1.38 | 18.23 |
| `move_arms()` | 5.62 ms | 5.29 | 6.75 | 13.51 |
| **cycle complet** | **6.87 ms** | 6.49 | **7.96** | 31.33 |

Décomposition côté pompe :

| | moyenne |
|---|---|
| attente `submit` → appliqué | 4.85 ms |
| application des commandes | 0.48 ms |
| pas de physique Isaac Sim | 4.26 ms |

**Le pas de physique n'existe pas sur un robot réel** : la physique y
*est* le monde, elle ne coûte rien au logiciel. Sur le vrai G1, le
chemin sera donc plus court que ce qui est mesuré ici.

## 3. Le chiffre qui compte

> **Surcoût réel de l'isolation : +1.46 ms** (6.87 contre 5.41), soit
> +27 %.

Les deux `multiprocessing.Queue`, le thread `HALBridgeServer`, la
désérialisation des feedback et `JointSafetyGuard` sur **chaque**
commande : une milliseconde et demie au total.

**Conclusion : aucun arbitrage de sécurité n'est nécessaire.** Pas de
chemin rapide, pas de clamp pré-calculé, pas de vérification allégée
pour les hautes fréquences. Une politique apprise peut piloter une app
sandboxée à 50 Hz avec les deux tiers du budget en réserve.

---

## 4. Le piège qui a failli fausser cette conclusion

La première mesure donnait **87.28 ms** en sandbox, soit +1513 %. À ce
niveau une app plafonne à 11 Hz et la tentation d'un chemin rapide
devient forte.

C'était un bug dans `MainThreadSimPump.run_forever()` :
`idle_sleep_s=0.0` faisait boucler le thread principal à vide, sans
jamais céder le GIL — dont 4.26 ms par tick passés dans `bridge.step()`
en C. Le thread du pont et le sous-process ne progressaient qu'au bon
vouloir de l'ordonnanceur Python. Les ~82 ms n'étaient **ni du
transport ni de la physique**, mais de l'attente pure.

Correctif : la pompe attend sur `Queue.get(timeout=2ms)`, qui relâche le
GIL. 87.28 ms → 6.87 ms, facteur 13.

**Deux leçons de méthode :**

*Décomposer avant de conclure.* La prédiction initiale — « le pas de
physique explique les 82 ms » — était fausse : il n'en expliquait que
4.26. Sans l'instrumentation séparant transport et simulation, la
conclusion aurait été « le sandbox est trop lent », et une architecture
correcte aurait été dégradée pour rien.

*Se méfier d'un indice incohérent.* `get_full_state()` coûtait 30.88 ms
alors que c'est une pure lecture qui n'attend rien du thread principal.
Une lecture qui coûte 31 ms ne décrit pas du travail : elle décrit une
attente. C'est cet écart, pas le raisonnement, qui a mené à la cause.

---

## 5. Contexte des correctifs associés

Trois corrections de la même session encadrent ces mesures :

- **`SetpointFollower`** (`axis_os/services/policy/`) — interpolateur à
  la fréquence du HAL entre deux consignes lentes, avec échéance de
  fraîcheur : `HOLD` au-delà de 0.5 s, `DAMPING` au-delà de 1.5 s.
  Dérive mesurée après péremption : **0.0027 rad**, du bruit de solveur.
  Chaque commande produite passe par `JointSafetyGuard`.

- **Désérialisation du pont** — `HALBridgeClient.get_full_state()`
  rendait des dicts là où `RobotContext` rend des `ActuatorFeedback`.
  Une app écrite contre la doc plantait uniquement en sandbox, donc
  uniquement sur le vrai robot. Fermé côté pont ; `test_context_parity`
  vérifie désormais les types de retour, pas seulement les noms.

- **`ACTUATORS_AI_DRIVEN`** (`DANGEROUS`), **`Capability.AI_POLICY`**,
  **`VLAAction.timestamp_ns`** — prérequis du pilotage par politique
  apprise.
