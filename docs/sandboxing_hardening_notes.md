# AXIS-OS — Renforcement futur du sandboxing (pistes, non implémenté)

> **Constat vérifié et plan ordonné : voir `docs/isolation_apps.md`**
> (audit du 2026-10-10). En résumé : aujourd'hui, une app hostile peut
> commander un actionneur sans passer par la HAL (processus créé par
> `fork`, même utilisateur, réseau non filtré).

**Statut : NON CODÉ.** Ce document liste des pistes concrètes pour
renforcer l'isolation des apps du Play Store au-delà de ce
qu'offre `multiprocessing` aujourd'hui (`axis_os/store/runtime/sandbox.py`).
Aucune n'est implémentée — capturé ici pour ne pas perdre la
recherche, à reprendre quand la portée du projet le justifiera (voir
"Pourquoi pas maintenant").

## L'état actuel et sa vraie limite

`SandboxRunner` isole chaque app dans un processus séparé
(`multiprocessing`) : un crash applicatif ou une boucle infinie
n'affecte pas le reste du système, et les quotas CPU/RAM/temps sont
appliqués. C'est une protection efficace contre le code **buggé**.

Ce que ça ne protège **pas** contre : le code **délibérément
malveillant**. Un processus `multiprocessing` partage le même noyau
Linux que le reste du système — si une app trouve une faille dans
Python ou dans le noyau lui-même, rien n'empêche structurellement une
évasion du bac à sable (contrairement à une vraie isolation au niveau
noyau).

## Pistes identifiées, de la plus légère à la plus lourde

### 1. Linux Security Modules (LSM) — AppArmor ou seccomp

Le noyau Linux expose un framework (LSM) sur lequel s'appuient
AppArmor, SELinux, et d'autres. Pour une app AXIS-OS, un profil
AppArmor pourrait limiter précisément quels fichiers, quels appels
système, quelles interfaces réseau sont accessibles — au-delà de ce
que `ResourceQuota` (CPU/RAM/temps) couvre aujourd'hui.

Seccomp (BPF) est une piste complémentaire, plus fine : filtrer les
appels système eux-mêmes (`open`, `socket`, etc.), pas seulement les
ressources consommées. `Firejail` combine namespaces + seccomp +
capabilities pour offrir ce type de confinement en espace
utilisateur, sans nécessiter un profil AppArmor complet à écrire à la
main — probablement le point d'entrée le plus simple si ce chantier
est repris.

### 2. Isolation au niveau noyau (gVisor / Firecracker)

Déjà identifié lors de l'audit initial de cette session : gVisor
(noyau intercepté en espace utilisateur) ou Firecracker (microVM)
offrent une isolation bien plus forte que `multiprocessing` —
utilisés notamment par les environnements d'exécution de code IA en
sandbox (dont celui de Claude Code lui-même). C'est la protection la
plus robuste contre du code réellement malveillant, au prix d'une
complexité d'infrastructure nettement supérieure (démarrage plus
lent, dépendances système supplémentaires).

## Pourquoi pas maintenant

- **Complexité disproportionnée au stade actuel** : un seul robot en
  simulation, pas de vraie app tierce installée par un utilisateur
  externe — le risque réel d'app malveillante est nul tant qu'AXIS-OS
  n'a pas de vrai Play Store ouvert à des développeurs externes non
  vérifiés.
- **Le vrai prérequis avant celui-ci** : la signature des apps
  (`axis_os/store/sdk/signing.py`, ajoutée cette session) réduit déjà
  une bonne partie du risque en garantissant l'origine et l'intégrité
  du code — un renforcement du sandboxing devient plus utile une fois
  qu'un vrai flux de publication tiers existe (apps de développeurs
  externes, pas seulement les 3 apps de référence internes).
- **Ordre logique recommandé** si ce chantier est repris : commencer
  par un profil AppArmor/Firejail simple (coût d'implémentation
  faible, gain de protection réel contre les cas les plus courants),
  et ne considérer gVisor/Firecracker que si un vrai incident ou un
  audit de sécurité externe (mentionné comme jamais fait dans ce
  projet) le justifie explicitement.

## Sources

- Recherche menée le 2026-09-08, dans le cadre d'un audit large de
  l'état de l'art sécurité robotique/OS pour AXIS-OS. Voir aussi
  `docs/fleet_manager_design.md` pour la piste SROS2 (sécurité
  réseau, sujet distinct de l'isolation locale couverte ici).
