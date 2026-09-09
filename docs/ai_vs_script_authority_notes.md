# AXIS-OS — Distinction script vs IA dans les permissions (piste, non implémenté)

**Statut : NON CODÉ.** Ce document capture une piste de renforcement
identifiée lors de l'audit du 2026-09-09, juste après l'ajout de
`VLACppConnector` — non implémentée, à reprendre quand la condition
ci-dessous sera remplie.

## Le constat

Le modèle de permissions actuel (`Permission`, `AppManifest.
has_permission()`, `RobotContext._require()`) est un vrai système à
base de capacités (capability-based security) — vérifié à CHAQUE
appel, pas seulement au démarrage (confirmé lors de cet audit :
`move_arms()`/`move_legs()`/`move_all()` appellent `_require()` à
chaque invocation).

Ce que ce modèle ne distingue PAS aujourd'hui : **qui**, à l'intérieur
d'une même app autorisée, est à l'origine d'une commande précise —
du code scripté écrit et relu par un développeur humain, ou une
décision produite par un modèle IA (via `VLACppConnector` ou
`OpenVLAConnector`, ajoutés dans cette même session). Une app qui a
`ACTUATORS_WRITE_ARMS` l'a pour toute sa durée de vie, que la
commande vienne d'une ligne de code fixe ou d'une inférence de
modèle — les deux traversent `JointSafetyGuard` avec la même autorité.

## Pourquoi ça pourrait devenir pertinent

Une source de recherche 2026 (Microsoft, "Defense in depth for
autonomous AI agents") formule le principe directement transposable :
*"no agent should have the same identity as the user... you need to
know if it was executed by the user, the agent acting on its own
behalf, or the agent acting on the user's behalf."* Appliqué ici : un
mouvement décidé par une IA qui "hallucine" une action inattendue
n'a, structurellement, pas de raison de bénéficier automatiquement de
la même autorité complète qu'un mouvement écrit et vérifié par un
humain dans le code de l'app.

Une piste complémentaire, de la même recherche : les permissions
d'AXIS-OS sont statiques (déclarées une fois, valables toute la durée
de vie de l'app) — un raffinement possible serait de les scoper à la
PHASE d'une mission (ex: `ACTUATORS_WRITE_ARMS` actif seulement
pendant une phase de manipulation explicite, pas pendant toute
l'exécution), réduisant la fenêtre d'exposition si l'app est
compromise ou si le modèle IA branché se comporte de façon inattendue.

## Ce que ça impliquerait concrètement (esquisse, non détaillée)

- Faire traverser une information d'origine ("script" vs "ia") depuis
  `IVLAConnector.predict_action()` jusqu'à `JointSafetyGuard` — un
  vrai changement architectural, pas une simple addition de champ.
- Potentiellement : des limites de sécurité DIFFÉRENTES (vitesse
  max, amplitude de mouvement) selon l'origine — plus prudentes pour
  une commande d'origine IA que pour une commande scriptée.
- Potentiellement : un scope de permission par phase de mission
  plutôt que par durée de vie complète de l'app.

## Pourquoi pas maintenant

- Aucune app de ce projet n'utilise encore RÉELLEMENT `VLACppConnector`
  ou `OpenVLAConnector` en production — le risque concret (une IA qui
  agit avec une autorité non distinguée) reste théorique tant qu'aucun
  vrai modèle n'est branché sur un vrai robot.
- C'est un changement architectural, pas une addition simple —
  mérite d'être conçu avec un vrai cas d'usage réel en main plutôt
  qu'en spéculatif.

## Condition de reprise

Dès qu'une app AXIS-OS branche réellement un VLA (Octo via
`VLACppConnector`, ou autre) sur un premier robot physique — c'est à
ce moment que le risque cesse d'être théorique et que la conception
de cette distinction doit se faire avec de vrais exemples de
commandes IA en main.

## Sources

Recherche menée le 2026-09-09 : "Defense in depth for autonomous AI
agents", Microsoft Security Blog, mai 2026 ; "How to Implement
Capability-Based Security", oneuptime.com, janvier 2026 ; "Capability
Minimization as a Safety Primitive" (RACG), arXiv 2606.13884.
