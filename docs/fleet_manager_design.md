# AXIS-OS — Fleet Manager (conception cible, non implémenté)

**Statut : NON CODÉ.** Ce document décrit l'architecture visée pour
une future version du Fleet Manager d'AXIS-OS. Aucune ligne de ce
design n'est encore implémentée — voir la section "Pourquoi ce n'est
pas codé maintenant" avant toute reprise de ce travail.

---

## Pourquoi ce n'est pas codé maintenant

Un Fleet Manager, par nature, ouvre une communication réseau entre
plusieurs robots et un serveur central. C'est exactement le sujet de
sécurité réseau (authentification, chiffrement) qui a été
explicitement mis de côté ailleurs dans ce projet, en attendant les
moyens de le traiter sérieusement. Construire cette pièce avant
d'avoir posé ces bases créerait une vraie surface d'attaque non
protégée, pour un besoin qui reste hypothétique tant qu'aucun client
réel n'a plusieurs robots à coordonner.

**Condition avant de reprendre ce chantier :**
1. Sécurité réseau de base posée (authentification robot ↔ serveur,
   chiffrement des communications).
2. Un premier cas d'usage réel qui le justifie (un client avec 2+
   robots à coordonner) — pas construit en spéculatif.

---

## Source de la conception

Ce design s'appuie sur un vrai papier de recherche récent :

> **Federated Single-Agent Robotics: Multi-Robot Coordination Without
> Intra-Robot Multi-Agent Fragmentation** (FSAR), arXiv:2604.11028
> (2026). https://arxiv.org/abs/2604.11028

Le choix de suivre ce papier plutôt que d'inventer une architecture
maison : ses résultats montrent des gains statistiquement
significatifs en "clarté des frontières de gouvernance" et en
"confinement de la récupération" par rapport à des architectures plus
fragmentées (agents multiples à l'intérieur d'un même robot) — et ses
5 objectifs de conception correspondent déjà, sans que ce soit
recherché intentionnellement, à des choix déjà faits dans AXIS-OS
(voir plus bas).

---

## Principe central : robot = agent unique gouverné, jamais fragmenté

> *"Chaque robot expose une surface de capacités gouvernée, plutôt
> qu'une société d'agents fragmentée en interne."*

Concrètement pour AXIS-OS : le Fleet Manager ne doit **jamais**
introduire de deuxième point de décision à l'intérieur d'un robot
(un "agent distant" qui déciderait à la place du robot). Chaque robot
reste piloté par son propre `PlayStoreRuntime` + `WatchdogSystem` +
`Supervisor` locaux — le Fleet Manager coordonne **entre** robots, il
ne remplace ni ne court-circuite **jamais** la prise de décision
locale de sécurité.

Ce principe est déjà respecté par construction dans AXIS-OS
aujourd'hui, précisément parce que le Fleet Manager n'existe pas
encore : chaque robot fonctionne de façon autonome. La règle est de
**préserver** cette autonomie une fois le Fleet Manager introduit, pas
de l'introduire après coup en espérant qu'elle survive.

---

## La relation d'autorité asymétrique (règle non négociable)

> *"La relation est asymétrique : le runtime local est autoritaire
> pour l'exécution ; la couche de fédération est autoritaire pour la
> coordination. Un runtime local peut fonctionner sans la couche de
> fédération (mode isolé), mais la couche de fédération ne peut pas
> exécuter de capacités sans les runtimes locaux."*

Traduit en règles concrètes pour l'implémentation future :

- Le Fleet Manager **propose** des tâches/missions à un robot — il ne
  les **impose** jamais en contournant `JointSafetyGuard`,
  `WatchdogSystem`, ou `ResourceManager`. Chaque commande reçue du
  Fleet Manager traverse exactement la même chaîne de sécurité qu'une
  commande venant d'une app locale.
- Le Fleet Manager **ne peut jamais** désactiver, court-circuiter, ou
  ignorer le `Supervisor`/`WatchdogSystem` d'un robot à distance. Un
  arrêt d'urgence local reste toujours prioritaire sur toute
  instruction reçue de la flotte.
- Un robot doit continuer à fonctionner **en mode isolé** dès que la
  connexion au Fleet Manager est perdue — jamais bloqué en attente
  d'instructions distantes pour des opérations déjà en cours ou
  déjà autorisées localement.

---

## Les 5 objectifs de conception (papier FSAR) appliqués à AXIS-OS

| # | Objectif (papier) | État chez AXIS-OS aujourd'hui | À garantir dans le futur Fleet Manager |
|---|---|---|---|
| 1 | Autosuffisance du runtime local | ✅ Déjà vrai (pas de Fleet Manager = pas de dépendance) | Ne jamais introduire de dépendance bloquante au Fleet Manager pour une opération déjà autorisée localement |
| 2 | Coordination de flotte sans absorber l'autonomie locale | ✅ Par construction (rien à absorber) | Le Fleet Manager reste un **coordinateur**, jamais un contrôleur qui retire la responsabilité de sécurité au robot |
| 3 | Chaque événement de coordination gouverné et auditable | ✅ `structured_log.py` existe déjà localement | Étendre le logging structuré aux événements de coordination (attribution de tâche, délégation, etc.) |
| 4 | Supporter des robots hétérogènes (capacités/policies/confiance différentes) | ✅ Système `Capabilities` déjà conçu pour ça (testé sur un robot à roues fictif) | Le registre de flotte doit interroger `get_capabilities()` de chaque robot, jamais supposer une morphologie uniforme |
| 5 | Dégrader proprement si la couche fédération est indisponible | ⚠️ À tester explicitement une fois le Fleet Manager introduit | Écrire un test de non-régression : robot déconnecté du Fleet Manager continue ses opérations en cours sans interruption ni comportement dégradé dangereux |

---

## Éléments d'architecture à reprendre du papier

### Registre de capacités partagé (ECM — Embodied Capability Module)

Chaque robot annonce ses capacités au Fleet Manager (réutilisation
directe de `CapabilitySet`/`RobotCapability` déjà existants dans
`axis_os/hal/core/capabilities.py`) — le Fleet Manager s'en sert pour
répartir intelligemment les tâches entre robots hétérogènes, sans
jamais supposer qu'un robot donné a telle ou telle capacité par
défaut.

### Deux couches distinctes, jamais fusionnées

```
┌─────────────────────────────────────────────┐
│         Fleet Federation Layer               │
│  (coordination : qui fait quoi, quand,       │
│   registre de capacités, délégation)          │
└───────────────────┬───────────────────────────┘
                     │  propose / coordonne
                     │  (jamais n'exécute directement)
┌───────────────────▼───────────────────────────┐
│         Local Runtime Layer (par robot)        │
│  PlayStoreRuntime + WatchdogSystem +           │
│  Supervisor + JointSafetyGuard + ResourceManager│
│  (AUTORITAIRE pour toute exécution physique)    │
└─────────────────────────────────────────────────┘
```

### Relations de coordination à modéliser (terminologie du papier)

- **Inter-robot capability request** — un robot (ou le Fleet Manager
  en son nom) demande à un autre robot d'effectuer une action liée à
  une capacité qu'il possède.
- **Authority delegation** — transfert temporaire et explicite
  d'autorité sur une tâche d'un robot à un autre (jamais implicite).
- **Layered recovery** — distinction claire entre une récupération
  gérée **localement** par un robot (son propre Watchdog/Supervisor)
  et une récupération qui nécessite une intervention **au niveau de
  la flotte** (ex : un robot en panne, ses tâches redistribuées).
- **Hierarchical human supervision** — un opérateur humain doit
  pouvoir superviser à la fois un robot individuel et la flotte
  entière, sans que l'un masque les alertes de l'autre.

---

## Ce que ce document ne couvre volontairement pas

- Le protocole réseau exact (WebSocket, HTTP, mTLS...) — dépend des
  décisions de sécurité réseau qui restent à prendre en amont.
- Le schéma de données précis du registre de capacités partagé —
  à concevoir une fois `Capabilities` éprouvé sur davantage de types
  de robots réels.
- Toute estimation de charge/scalabilité — prématuré sans premier
  déploiement réel à observer.
