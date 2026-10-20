# AXIS-OS — Isolation des apps : constat et plan

**Statut : ÉTAPES 0 ET 1 FAITES (2026-10-20), ÉTAPES 2 À 4 À FAIRE.**
Rédigé à l'audit de sécurité du 2026-10-10 (point I2). Complète
`docs/sandboxing_hardening_notes.md` (pistes techniques générales) par un
constat vérifié sur le code et un plan ordonné.

Ce document décrit les faiblesses et leur correction, volontairement sans
mode opératoire : les essais qui les ont établies ne sont pas publiés.

## Le constat, en une phrase

**Avant le 2026-10-20, une app malveillante POUVAIT commander un
actionneur sans passer par la HAL** : son processus était une copie de
celui de l'OS (fork), et le pont IPC transportait des objets pickle. Les
étapes 0 et 1 ci-dessous ont fermé ces deux voies. Restent ouverts le
même utilisateur Unix, le réseau et les appels système (étapes 2 à 4) :
une app hostile peut encore lire les fichiers de l'OS et, sur un vrai G1,
ouvrir sa propre connexion DDS. Tant que ces étapes ne sont pas faites,
**seules des apps dont on connaît et relit le code doivent tourner sur un
vrai robot**, en profil PRODUCTION (voir « En attendant »).

## Ce qui a été vérifié

| # | Constat | Vérifié comment |
|---|---------|-----------------|
| 1 | *(Corrigé, étape 1.)* Le processus de l'app était créé par `fork` (`sandbox.py`) : il héritait d'une COPIE de toute la mémoire du processus principal, objets HAL compris. | Essai : depuis une mission sandboxée, les objets HAL du processus principal sont accessibles. |
| 2 | *(Corrigé, étape 1, pour les connexions héritées ; une connexion NEUVE reste possible tant que l'étape 3 n'est pas faite.)* Sur un vrai G1 (`backend="dds"`), cette copie détenait les **mêmes connexions DDS ouvertes** vers le robot : une commande moteur envoyée depuis l'app partirait vers les moteurs sans passer par `JointSafetyGuard`. | Conséquence directe de (1) : `fork` duplique aussi les connexions ouvertes. Non essayé sur matériel. |
| 3 | L'app tourne sous le **même utilisateur** que l'OS : elle peut charger les mêmes bibliothèques que lui, ouvrir ses propres connexions réseau vers le robot, et lire les fichiers de l'OS et des autres apps (le cloisonnement des données par `app_id` est logique, pas imposé par le système). | Lecture du code (`sandbox.py`, `memory/service.py`). |
| 4 | Le réseau n'est pas filtré : `NETWORK_CLOUD` / `NETWORK_LOCAL` sont déclarées mais non appliquées (déjà documenté dans `manifest.py`). | Lecture du code. |
| 5 | Ce qui tient déjà (contre une app buggée ; contre une app hostile, voir 6) : les hooks `on_start` / `run_mission` / `on_stop` passent tous par `SandboxRunner` + `HALBridgeServer` ; un `RobotContext` ne peut pas être fabriqué hors du runtime (jeton de session) ; les permissions SIGNATURE exigent la clé plateforme ; les quotas CPU/RAM/temps s'appliquent (si `psutil` est installé). | Tests existants (`test_context_lockdown.py`, `test_playstore_sandbox_isolation.py`, …). |
| 6 | *(Corrigé, étape 0.)* **Le pont IPC transportait des objets PICKLE** (`HALBridgeServer` lit les commandes de l'app, `SandboxRunner` lit son résultat). Or désérialiser un pickle peut exécuter du code : un objet fabriqué par l'app ferait tourner du code de son choix DANS LE PROCESSUS DE L'OS, avec tous ses droits (modifier ses propres permissions après la confirmation, appeler la HAL directement). | Essai lors de la revue de l'audit : du code venu d'une mission s'est exécuté dans le processus de l'OS. |

## Le plan, dans l'ordre

Chaque étape ferme une voie de contournement précise et vient avec son test
« l'app essaie, et échoue ». L'ordre suit le rapport gain/coût : les étapes 0
à 3 sont le **minimum avant d'installer sur un vrai robot une app dont on n'a
pas relu le code**.

### Étape 0 — Un pont qui ne désérialise que des DONNÉES (FAIT)

Fait le 2026-10-20 : `canal_ipc.py` (trames de 1 Mio au plus, JSON
strict, schémas), `hal_bridge.py` (un schéma par action, refus de tout
champ inattendu), `sandbox.py` (le résultat d'un hook est un JSON
revérifié, reconstruit champ par champ). Le plan d'origine :


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

### Étape 1 — Ne plus hériter de la mémoire : `spawn` au lieu de `fork` (FAIT)

Fait le 2026-10-20 : `processus_app.py` lance
`python -I hote_app.py` par `subprocess.Popen(close_fds=True,
pass_fds=(pont,), start_new_session=True)`, avec un environnement réduit
à PATH et à la langue ; `hote_app.py` recharge l'app depuis sa source
vérifiée. Le processus de l'app meurt avec l'OS (PR_SET_PDEATHSIG), et
tout son groupe de processus est tué à la fin de chaque appel. En profil
PRODUCTION, `PlayStoreRuntime.install_source()` lit même le manifeste
dans ce processus : le code d'une app ne s'exécute jamais dans celui de
l'OS. Le plan d'origine :


- Démarrer l'app dans un interpréteur NEUF (`mp.get_context("spawn")`, ou
  mieux un processus dédié `python -I -m axis_os.store.runtime.hote_app`
  lancé avec `close_fds=True`), qui ne reçoit QUE les extrémités du pont IPC
  (le canal de l'étape 0).
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
  0 à 4, et sur décision explicite.

### En parallèle, côté robot

- DDS : un domaine DDS réservé à l'OS et, à terme, DDS Security
  (authentification des participants) : un participant inconnu ne doit pas
  pouvoir publier de commande moteur.
- Le chien de garde matériel et l'arrêt d'urgence physique restent la
  dernière barrière, indépendante de tout logiciel (voir
  `docs/hardware_estop_notes.md`).

## En attendant (règles d'exploitation)

1. Sur un vrai robot : `PlayStoreRuntime(robot, watchdog,
   profil=Profil.PRODUCTION, cles_approuvees="/chemin/cles_approuvees.txt",
   platform_public_key=..., confirmer_permissions=confirmer_au_terminal)`,
   puis `install_source(...)` d'apps signées dont le code a été relu. Le
   profil PRODUCTION impose la signature, une clé approuvée, la
   confirmation humaine des permissions DANGEROUS et psutil.
2. Les permissions DANGEROUS (dont `ACTUATORS_AI_DRIVEN`) exigent une
   confirmation explicite à l'installation (audit I1) : elle ne protège pas
   contre une app hostile, mais rend visible ce qu'une app honnête fera.

## Suivi

| Étape | Statut |
|-------|--------|
| Constat vérifié | fait (2026-10-10 ; constat 6 à la revue) |
| 0 — pont sans pickle | fait (2026-10-20) |
| 1 — spawn, pas de mémoire héritée | fait (2026-10-20) |
| 2 — utilisateur dédié | à faire : demande un assistant privilégié lancé par systemd, hors de portée d'un changement de code seul |
| 3 — espace de noms réseau vide, NETWORK_* appliquées | à faire : même raison ; casse aussi les apps qui joignent un service local (assistant IA) |
| 4 — seccomp, rlimits, cgroups | à faire |
| 5 — gVisor / Firecracker | sur décision |
