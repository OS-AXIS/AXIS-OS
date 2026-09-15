# AXIS-OS — Arrêt d'urgence matériel : la vraie fonction de sécurité

**Statut : NON IMPLÉMENTÉ (matériel), CONCEPTION DOCUMENTÉE.** Ce
document capture un point de fond soulevé lors d'un audit externe
(2026-09-15) — pas un correctif de code, mais un principe qui doit
rester visible avant tout déploiement sur du matériel physique.

## Le principe, sans détour

> Rien de ce qui a été construit dans ce dépôt — `JointSafetyGuard`,
> `WatchdogSystem`, `Supervisor`, le moniteur de fraîcheur hors
> process (`deploy/freshness_monitor/`), le moniteur de sécurité
> indépendant (`axis_os/hal/independent_monitor/`), le chien de garde
> systemd — **n'est une fonction de sécurité au sens normatif**
> (ISO 13849, IEC 61508). Du Python, ou même du C, tournant sur un
> Linux non temps-réel, ne peut structurellement pas obtenir cette
> qualification : latence non garantie, ordonnanceur non déterministe,
> dépendance à un système d'exploitation généraliste avec sa propre
> surface de panne.

Un **bouton d'arrêt d'urgence câblé, qui coupe l'alimentation de
puissance des moteurs directement**, est une fonction de sécurité
au sens normatif — précisément parce qu'il ne dépend d'aucun
logiciel. C'est cette pièce matérielle, pas AXIS-OS, qui doit être
en place avant qu'un G1 (ou tout autre robot) travaille à proximité
de quelqu'un.

**AXIS-OS reste ce qu'il est de mieux : une couche de supervision
intelligente au-dessus d'une sécurité matérielle — jamais à sa
place.** Tout ce qui a été construit dans ce projet (validation de
commandes, détection de surcharge, redondance diverse, chiens de
garde) réduit la probabilité qu'un arrêt d'urgence matériel soit
nécessaire, et réduit les dégâts en attendant qu'il le soit — mais
ne remplace jamais sa présence.

## Ce que ça implique concrètement pour AXIS-OS

### Portée du bouton, cohérente avec Fleet Manager

Deux niveaux distincts, cohérents avec `docs/fleet_manager_design.md` :

1. **Bouton robot individuel** — coupe l'alimentation de puissance
   d'UN robot précis. C'est la forme la plus simple, déjà standard
   sur la plupart des plateformes robotiques commerciales (le G1
   en a probablement déjà un physique, à vérifier dans sa
   documentation constructeur avant tout déploiement).

2. **Bouton flotte** — arrête TOUS les robots d'une flotte
   simultanément. N'a de sens qu'une fois le Fleet Manager
   implémenté (voir `docs/fleet_manager_design.md`, toujours non
   codé) — et même alors, sa portée normative reste limitée : un
   arrêt "flotte" déclenché par un bouton relié au Fleet Manager
   (donc logiciel, réseau) n'a PAS la même garantie qu'un vrai
   arrêt matériel individuel par robot. Un bouton "flotte" devrait
   être conçu comme un **raccourci pratique** qui déclenche N
   arrêts individuels (chacun avec sa propre garantie matérielle
   locale), jamais comme une fonction de sécurité en soi qui
   remplacerait les boutons individuels.

### Ce qu'AXIS-OS DOIT faire quand un tel bouton existe

- **Détecter et journaliser** l'état "arrêt matériel actif" — même
  si AXIS-OS n'en est pas la cause ni le moyen. Le HAL réel
  (`G1DDSActuator` ou équivalent futur) devrait pouvoir lire cet état
  depuis le firmware du robot (si exposé — à vérifier dans la
  documentation constructeur exacte du G1) et le refléter dans
  `IRobotHAL.estop_active` (déjà existant, voir correctif E10 dans
  `CHANGELOG.md`) — pour que le logiciel sache que le matériel a agi,
  sans jamais prétendre en être la cause.
- **Ne jamais tenter de contourner ou de réarmer** un arrêt matériel
  depuis le logiciel — cohérent avec le choix déjà fait pour
  `WatchdogSystem.estop_fired` (voir M9 dans `CHANGELOG.md` : jamais
  remis à `False` automatiquement). Un arrêt matériel encore moins :
  son réarmement doit rester un geste physique, jamais logiciel.
- **Ne jamais présenter** `JointSafetyGuard`, le moniteur de fraîcheur,
  le moniteur indépendant, ou tout autre composant logiciel de ce
  dépôt comme LA fonction de sécurité d'un déploiement client — les
  présenter, honnêtement, comme une supervision qui réduit le risque
  et facilite le diagnostic, en complément d'un vrai arrêt matériel,
  jamais en remplacement.

## Condition de reprise

Avant tout déploiement sur du matériel physique avec des humains à
proximité :
1. Confirmer l'existence et l'emplacement du bouton d'arrêt matériel
   du G1 (ou du robot cible) dans sa documentation constructeur.
2. Si un Fleet Manager est un jour implémenté et qu'un bouton
   "flotte" est envisagé, le concevoir comme déclencheur de N arrêts
   matériels individuels — jamais comme substitut logiciel unique.
3. Documenter clairement, dans toute présentation commerciale ou
   technique d'AXIS-OS, la limite énoncée ci-dessus — ne jamais
   laisser un client ou un partenaire croire que la sécurité
   logicielle de ce dépôt suffit seule.
