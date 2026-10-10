# AXIS-OS — Isolation des apps : constat et plan

**Statut : CONSTAT ÉTABLI, CORRECTION NON CODÉE.** Rédigé à l'audit de
sécurité du 2026-10-10 (point I2). Complète
`docs/sandboxing_hardening_notes.md` (pistes techniques générales) par un
constat vérifié sur le code actuel et un plan ordonné.

Ce document décrit les faiblesses et leur correction, volontairement sans
mode opératoire : les essais qui les ont établies ne sont pas publiés.

## Le constat, en une phrase

**Aujourd'hui, une app malveillante PEUT commander un actionneur sans passer
par la HAL.** Le Play Store, les permissions et le pont IPC (`hal_bridge.py`)
protègent contre une app *buggée* ou *maladroite*, pas contre une app qui
cherche à les contourner.

La règle « aucune app ne commande un actionneur hors de la HAL » est donc
vraie pour le code coopératif, et fausse pour le code hostile. Tant que ce
plan n'est pas réalisé, **seules des apps dont on connaît et relit le code
doivent tourner sur un vrai robot** (voir « En attendant »).

## Ce qui a été vérifié

| # | Constat | Vérifié comment |
|---|---------|-----------------|
| 1 | Le processus de l'app est créé par `fork` (`sandbox.py`) : il hérite d'une COPIE de toute la mémoire du processus principal, objets HAL compris. | Essai : depuis une mission sandboxée, les objets HAL du processus principal sont accessibles. |
| 2 | Sur un vrai G1 (`backend="dds"`), cette copie détient les **mêmes connexions DDS ouvertes** vers le robot : une commande moteur envoyée depuis l'app partirait vers les moteurs sans passer par `JointSafetyGuard`. | Conséquence directe de (1) : `fork` duplique aussi les connexions ouvertes. Non essayé sur matériel. |
| 3 | L'app tourne sous le **même utilisateur** que l'OS : elle peut charger les mêmes bibliothèques que lui, ouvrir ses propres connexions réseau vers le robot, et lire les fichiers de l'OS et des autres apps (le cloisonnement des données par `app_id` est logique, pas imposé par le système). | Lecture du code (`sandbox.py`, `memory/service.py`). |
| 4 | Le réseau n'est pas filtré : `NETWORK_CLOUD` / `NETWORK_LOCAL` sont déclarées mais non appliquées (déjà documenté dans `manifest.py`). | Lecture du code. |
| 6 | **Le pont IPC transporte des objets PICKLE** (`HALBridgeServer` lit les commandes de l'app, `SandboxRunner` lit son résultat). Or désérialiser un pickle peut exécuter du code : un objet fabriqué par l'app ferait tourner du code de son choix DANS LE PROCESSUS DE L'OS, avec tous ses droits (modifier ses propres permissions après la confirmation, appeler la HAL directement). | Essai lors de la revue de l'audit : du code venu d'une mission s'est exécuté dans le processus de l'OS. |
| 5 | Ce qui tient déjà (contre une app buggée ; contre une app hostile, voir 6) : les hooks `on_start` / `run_mission` / `on_stop` passent tous par `SandboxRunner` + `HALBridgeServer` ; un `RobotContext` ne peut pas être fabriqué hors du runtime (jeton de session) ; les permissions SIGNATURE exigent la clé plateforme ; les quotas CPU/RAM/temps s'appliquent (si `psutil` est installé). | Tests existants (`test_context_lockdown.py`, `test_playstore_sandbox_isolation.py`, …). |

## Le plan, dans l'ordre

Chaque étape ferme une voie de contournement précise et vient avec son test
« l'app essaie, et échoue ». L'ordre suit le rapport gain/coût : les étapes 0
à 3 sont le **minimum avant d'installer sur un vrai robot une app dont on n'a
pas relu le code**.

### Étape 0 — Un pont qui ne désérialise que des DONNÉES

- Remplacer le transport par pickle (`mp.Queue`) par des octets JSON avec
  un schéma strict par action (`mp.Pipe`, `send_bytes` / `recv_bytes` avec
  taille maximale, `json.loads`, types de base seulement). Le parent
  reconstruit le `MissionResult` et le résultat des hooks à partir de
  champs typés, jamais d'un objet reçu.
- Ferme : constat 6. Indispensable AVANT les étapes suivantes : un pickle
  exécute son code dans le parent quel que soit le mode de création du
  processus de l'app (fork ou spawn) et quel que soit son utilisateur.
- Test : l'app envoie un objet qui tente d'exécuter du code à sa
  désérialisation -> rien ne s'exécute dans le parent, la commande est
  refusée.

### Étape 1 — Ne plus hériter de la mémoire : `spawn` au lieu de `fork`

- Démarrer l'app dans un interpréteur NEUF (`mp.get_context("spawn")`, ou
  mieux un processus dédié `python -I -m axis_os.store.runtime.hote_app`
  lancé avec `close_fds=True`), qui ne reçoit QUE les deux files du pont IPC.
- Le code de l'app y est rechargé depuis sa source signée
  (`packaging.py` sait déjà le faire par `exec`).
- Ferme : constats 1 et 2 (plus d'objet HAL ni de socket DDS hérités).
- Test : depuis une mission, aucun objet HAL n'est accessible, et les
  seuls descripteurs ouverts sont ceux du pont.
- Coût : moyen. Démarrage plus lent (~0,3 s par mission) ; les apps doivent
  être importables (déjà exigé par la signature).

### Étape 2 — Un utilisateur système dédié aux apps

- L'hôte d'app tourne sous un uid `axis-app` (sans shell, sans groupe), lancé
  par un petit assistant privilégié démarré par systemd ; `NoNewPrivileges`.
- `logs/`, `memory_store/`, les clés de signature et le code de l'OS ne sont
  pas lisibles par `axis-app` ; la mémoire de chaque app lui est transmise
  par le pont, plus par le disque.
- Ferme : constat 3 (lecture des données des autres apps et de l'OS).
- Test : lire un journal de l'OS depuis l'app -> accès refusé.

### Étape 3 — Pas de réseau par défaut : espace de noms réseau vide

- L'hôte d'app est créé dans un espace de noms réseau sans interface
  (`unshare(CLONE_NEWNET)`, ou `PrivateNetwork=yes` côté systemd) : plus de
  DDS, plus d'Internet. Le pont IPC passe par des descripteurs déjà ouverts,
  non par le réseau.
- `NETWORK_CLOUD` devient APPLIQUÉE, et passe alors DANGEROUS (elle est
  NORMAL aujourd'hui, donc accordée sans confirmation) : une app qui la
  déclare et l'a fait confirmer reçoit un accès sortant limité par un
  mandataire (proxy) à liste blanche — jamais l'interface DDS du robot.
- Ferme : constats 3 (DDS direct) et 4.
- Test : un envoi réseau vers le robot depuis l'app -> erreur « réseau
  inaccessible ».

### Étape 4 — Filtre d'appels système et limites du noyau

- seccomp (liste blanche) : pas de `ptrace`, `mount`, `socket(AF_PACKET)`,
  `bpf`, `keyctl`… ; `RLIMIT_AS`, `RLIMIT_NPROC`, `RLIMIT_NOFILE` ; quotas
  CPU/RAM par cgroups v2 au lieu de la surveillance par `psutil` (qui, sans
  `psutil`, n'applique rien).
- Outil le plus simple pour commencer : `bubblewrap` ou `Firejail`, qui
  combinent espaces de noms + seccomp sans écrire de profil à la main.
- Test : un `os.fork()` en boucle, une allocation de 4 Go, un `ptrace` -> tués
  ou refusés, l'OS continue.

### Étape 5 (si un vrai Play Store public existe) — isolation noyau

- gVisor ou Firecracker, comme décrit dans
  `docs/sandboxing_hardening_notes.md`. À ne considérer qu'après les étapes
  1 à 4, et sur décision explicite.

### En parallèle, côté robot

- DDS : un domaine DDS réservé à l'OS et, à terme, DDS Security
  (authentification des participants) : un participant inconnu ne doit pas
  pouvoir publier de commande moteur.
- Le chien de garde matériel et l'arrêt d'urgence physique restent la
  dernière barrière, indépendante de tout logiciel (voir
  `docs/hardware_estop_notes.md`).

## En attendant (règles d'exploitation, sans code)

1. Sur un vrai robot : `PlayStoreRuntime(require_signed_apps=True,
   platform_public_key=..., confirmer_permissions=confirmer_au_terminal)`,
   et n'installer que des apps signées dont le code a été relu.
2. `psutil` installé (sinon les quotas CPU/RAM ne s'appliquent pas — le
   runtime le signale au démarrage).
3. Les permissions DANGEROUS (dont `ACTUATORS_AI_DRIVEN`) exigent une
   confirmation explicite à l'installation (audit I1) : elle ne protège pas
   contre une app hostile, mais rend visible ce qu'une app honnête fera.

## Suivi

| Étape | Statut |
|-------|--------|
| Constat vérifié | fait (2026-10-10 ; constat 6 à la revue) |
| 0 — pont sans pickle | à faire |
| 1 — spawn, pas de mémoire héritée | à faire |
| 2 — utilisateur dédié | à faire |
| 3 — espace de noms réseau vide, NETWORK_* appliquées | à faire |
| 4 — seccomp, rlimits, cgroups | à faire |
| 5 — gVisor / Firecracker | sur décision |
