# Démonstration pour investisseurs — tri de colis autonome, avec des humains

## Objectif

Un G1 29 DOF trie des colis **pendant 30 minutes sans intervention**, dans un
entrepôt simulé où **des humains circulent**. Aucune position ne lui est
donnée : il trouve les colis, les identifie, les porte au bon poste et les y
dépose. Quand un humain s'approche, **c'est l'OS qui ralentit puis arrête le
robot** — pas l'app.

Ce dernier point est le cœur du pitch : il montre ce qu'aucun concurrent ne
montre, une sécurité garantie **au-dessous** des apps et de l'IA.

## Critères de réussite, mesurés et affichés à l'écran

| Critère | Cible |
|---|---|
| Durée continue | **30 min** simulées |
| Colis triés | ≥ 20, avec le taux de réussite affiché |
| Chutes | **0** |
| Contacts avec un humain | **0** |
| Arrêts de sécurité | tous **journalisés**, avec leur cause |
| Commandes refusées par le garde-fou | comptées et expliquées |

Livrables : une vidéo accélérée de 2 à 3 minutes avec tableau de bord, le
journal complet, et la vidéo intégrale disponible sur demande — **un
investisseur technique voudra voir qu'elle n'est pas montée**.

---

## Étapes — chacune produit une démo montrable

### 1. Navigation
Le robot va d'un poste à l'autre dans l'entrepôt en contournant les
obstacles fixes. Service de navigation d'AXIS-OS, appelé par les apps comme
le service de marche.
*Honnêteté : en simulation, la position du robot est d'abord lue dans la
scène. La localisation par capteurs (lidar, SLAM, comme OM1) vient ensuite
et reste indispensable pour le vrai robot.*
**Démo : « va au poste B ».**

### 2. Dépose sur un poste
Poser le carton sur une étagère ou une table, et non plus le lâcher au sol.
**Démo : un carton déplacé d'une table à une autre.**

### 3. Identifier la destination
Chaque colis porte une marque visible — couleur ou étiquette —, reconnue par
OWL-ViT. Colis rouge vers le poste A, bleu vers le poste B. La règle de tri
est **dans l'app**, pas dans l'IA : c'est de la connaissance métier.
**Démo : trois colis triés vers deux postes.**

### 4. Humains et zone de sécurité
Des personnes circulent dans l'entrepôt. Le robot les détecte ; **le service
de locomotion d'AXIS-OS plafonne sa vitesse** près d'un humain et l'arrête en
dessous d'une distance fixée. L'app ne peut ni désactiver ni contourner ce
plafond.
*À vérifier : disponibilité des personnages animés d'Isaac Sim 6.1 ; à
défaut, des silhouettes déplacées par script.*
**Démo : un humain traverse, le robot ralentit, s'arrête, puis reprend.**

### 5. Endurance
Tenir 30 minutes, c'est rencontrer les pannes rares. L'app doit savoir
**se rattraper** : prise ratée → nouvel essai ; colis perdu → le chercher ;
situation inconnue → s'arrêter proprement et le signaler. Un réapprovisionneur
remet des colis sur la table de départ.
**Démo : la vidéo accélérée de 30 minutes.**

### 6. Présentation
Tableau de bord en incrustation — colis triés, taux de réussite, arrêts de
sécurité —, vidéo accélérée, dossier investisseur.

---

## Budget GPU

L'essentiel du développement se fait **sans rendu**, bien plus vite. Le
rendu vidéo n'est lancé que pour les démos.

Une exécution de 30 minutes simulées **avec rendu** peut prendre plusieurs
heures réelles : on la lance **une fois**, quand tout est validé.

Estimation d'ensemble : **20 à 30 heures GPU**. Sur une RTX PRO 6000 louée
autour de 1.70 $/h, de l'ordre de 35 à 50 $ ; environ moitié moins sur une
RTX 5090 tant qu'on n'entraîne pas de modèle.

## Ce qui reste hors de cette démo, et qu'il faudra dire

- **Simulation seulement** : le passage au vrai robot reste à faire.
- **Gestes de bras programmés** : l'IA voit, marche et équilibre ; elle ne
  décide pas encore des mouvements de bras. C'est l'étape du modèle
  vision-langage-action.
- **Position du robot lue dans la simulation** tant que la localisation par
  capteurs n'est pas intégrée.
