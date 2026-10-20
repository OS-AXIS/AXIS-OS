# Ce qu'un robot doit avoir pour faire tourner AXIS-OS

AXIS-OS s'exécute sur l'ordinateur du robot, ou sur un ordinateur relié à
lui, et parle au matériel par un **driver** qui implémente l'interface
`IRobotHAL`. Tout ce qui suit découle de ce que cette interface exige
réellement — ou d'une erreur rencontrée en simulation, signalée comme
telle.

Trois niveaux, du plus simple au plus complet. Chaque niveau inclut le
précédent.

---

## Niveau 1 — Robot actionné

*Un bras fixe, une plateforme de recherche. AXIS-OS peut y installer des
apps dans un bac à sable (qui protège d'une app buggée, pas encore d'une
app malveillante : voir [SECURITY.md](../SECURITY.md#limites-connues)) et
vérifier chaque commande.*

### Ordinateur

| | Exigence | Pourquoi |
|---|---|---|
| Système | **Linux** | Le sandbox place chaque app dans un processus neuf relié à l'OS par une socket Unix, et s'appuie sur des fonctions propres à Linux (groupe de processus, arrêt avec le parent) |
| Architecture | x86_64 ou ARM64 | |
| Python | 3.12 ou plus récent (la version que teste la CI) | |
| Dépendances | `psutil` (quotas des apps), `cryptography` (signature des apps) | Le cœur du HAL n'utilise que la bibliothèque standard |

### Actionneurs

- **Pilotables articulation par articulation**, au minimum en **position**.
  Les modes vitesse, couple, impédance et courant sont pris en charge si le
  matériel les offre.
- **Retour d'état** de chaque articulation — position, vitesse, effort — à
  **50 Hz au moins**.
- **Arrêt d'urgence logiciel** par actionneur (`emergency_stop()`).

### Latence

Le budget d'un cycle de commande à 50 Hz est de **20 ms**. AXIS-OS en
consomme **5.4 ms** en appel direct et **6.9 ms** depuis une app isolée,
garde-fou compris (mesuré, `docs/latence_chemin_commande.md`). Le bus du
robot et son driver doivent tenir dans le reste.

### Description du robot

- Un **URDF exploitable** : articulations, axes, masses, et **géométries de
  collision pour chaque pièce qui doit toucher quelque chose**. Constaté :
  dans l'URDF officiel du G1 29 DOF, les mains n'avaient aucune collision,
  et traversaient les objets en simulation.
- Pour **chaque articulation**, les six limites dont le garde-fou a besoin :
  position min et max, vitesse max, couple max, courant max, température
  max. Une limite manquante, c'est un contrôle qui ne peut pas avoir lieu.
- Les **groupes d'articulations** — manipulation, locomotion, autres —
  déclarés par le driver, jamais devinés à partir des noms.

### Conventions de signe — vérifiées, pas supposées

Pour chaque articulation, dans quel sens un angle positif fait bouger le
membre. La table doit être :

1. **vérifiée par la suite S2** en simulation (cohérence des signes) ;
2. **contrôlée en sens absolu**, par cinématique directe — S2 ne suffit pas.
   Constaté : sur le G1, la table « validée » inversait le sens du tangage
   d'épaule et du coude ; une app demandant de lever les bras les envoyait
   en arrière.

Tant qu'une convention n'est pas vérifiée, **AXIS-OS refuse de démarrer sur
un robot réel**. Elle n'est tolérée qu'en simulation.

### Arrêt d'urgence matériel

**Non négociable.** Un bouton physique qui coupe l'énergie des moteurs
**indépendamment de tout logiciel**, AXIS-OS compris. Le garde-fou d'AXIS-OS
protège contre les commandes dangereuses ; il ne remplace pas un arrêt
d'urgence câblé.

### Capacités déclarées

Le driver déclare ce que le robot possède — jambes, bras, caméra, micro…
— **ni plus, ni moins**. Une capacité oubliée fait refuser des apps
légitimes ; une capacité déclarée à tort fait installer des apps qui ne
pourront pas fonctionner.

---

## Niveau 2 — Robot mobile

*Le robot se déplace. Les apps pilotent par consignes de vitesse, jamais
pas à pas.*

### Robot à jambes

- Une **centrale inertielle** donnant l'orientation et la vitesse angulaire
  **dans le repère du capteur**, avec la **position du capteur connue** sur
  le robot. Constaté : sur le G1, elle est dans le torse et non dans le
  bassin ; une politique qui attend le bassin doit compenser l'articulation
  de taille.
- Une **politique de locomotion**, fournie par le constructeur ou entraînée,
  branchée par un adaptateur (`ILocomotionAdapter`). Le service de marche
  d'AXIS-OS ne change pas d'un robot à l'autre : seul l'adaptateur change.
  Validé : politique d'Unitree sur le G1 23 DOF, politique GR00T WBC de
  NVIDIA sur le G1 29 DOF, avec le même service et les mêmes apps.

### Robot à roues

- Une interface de **commande en vitesse** et une **odométrie**.

### Dans tous les cas

- Un **arrêt sûr défini par le robot** : pour un bipède, s'immobiliser en
  gardant l'équilibre ; couper les moteurs le ferait tomber. Si une app
  cesse de répondre, AXIS-OS ramène la consigne à zéro en une demi-seconde.

---

## Niveau 3 — Robot autonome

*Le robot perçoit, comprend et agit sans qu'on lui donne de position.*

| Pour… | Il faut | Remarque |
|---|---|---|
| **Voir** | Caméra couleur **calibrée** (paramètres intrinsèques) | Sans calibration : directions fausses, sans erreur visible |
| **Situer en 3D** | Caméra de **profondeur alignée** pixel à pixel sur la couleur | Sans elle : une direction, jamais une distance |
| | **Pose de la caméra** issue de la cinématique | Jamais une valeur supposée : une orientation oubliée projetait tout à l'horizontale |
| **Entendre, parler** | Micro, haut-parleur | La reconnaissance vocale ne doit **jamais** servir d'arrêt d'urgence |
| **Saisir** | Bras + organe de préhension **avec géométrie de collision** | Capteur d'effort recommandé : sans lui, le robot ne sent pas qu'une prise glisse |
| **Faire tourner l'IA** | GPU embarqué (famille Jetson) ou serveur proche | Détection, transcription, politiques de marche et de manipulation |

---

## Ce qu'AXIS-OS ne fournit pas encore

- **Des drivers pour d'autres robots que le Unitree G1.** L'architecture est
  prête ; chaque nouveau robot demande un driver et une table de
  conventions vérifiée.
- **La localisation et la cartographie par capteurs** (SLAM) : pas
  encore. La navigation existe, sur une carte déclarée, avec une position
  lue dans le simulateur.
- **Une politique de locomotion universelle** : elle reste propre à chaque
  robot, comme un pilote de carte graphique.
- **Une validation sur robot physique** : tout ce qui est décrit ici est
  validé **en simulation** (Isaac Sim 6.1).

## Robots de référence

| Robot | Niveau atteint | Statut |
|---|---|---|
| Unitree G1 23 DOF | 3 | Validé en simulation : marche, virage, perception, voix, inventaire |
| Unitree G1 29 DOF | 3 | Validé en simulation : marche sous charge, saisie et port autonomes. **6 conventions non vérifiées** — démarrage refusé sur robot réel |
