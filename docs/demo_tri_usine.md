# Démonstration pour investisseurs — tri de colis autonome, avec des humains

## Objectif

Un G1 29 DOF trie des colis **pendant 30 minutes sans intervention**, dans un
entrepôt simulé où **des humains circulent**. Aucune position ne lui est
donnée : il trouve les colis, les identifie, les porte au bon poste et les y
dépose. Quand un humain s'approche, **c'est l'OS qui ralentit puis arrête le
robot** — pas l'app.

Ce dernier point est le cœur du pitch : la sécurité est placée
**au-dessous** des apps et de l'IA, au lieu d'être confiée à chacune
d'elles. Elle vaut pour les apps qui passent par l'API : l'isolation
contre une app hostile n'est pas encore terminée (voir
[SECURITY.md](../SECURITY.md#limites-connues), limite connue n° 1).

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
plafond par l'API ; l'isolation contre une app hostile n'est pas encore
terminée ([SECURITY.md](../SECURITY.md#limites-connues), limite connue
n° 1).
*À vérifier : disponibilité des personnages animés d'Isaac Sim 6.1 ; à
défaut, des silhouettes déplacées par script.*
**Démo : un humain traverse, le robot ralentit, s'arrête, puis reprend.**

### 5. Endurance
Tenir 30 minutes, c'est rencontrer les pannes rares. L'app doit savoir
**se rattraper** : prise ratée → nouvel essai ; colis perdu → le chercher ;
situation inconnue → s'arrêter proprement et le signaler. Un réapprovisionneur
remet des colis sur la table de départ.
**Démo : la vidéo accélérée de 30 minutes.**

### 5b. Superviseur en langage naturel
L'opérateur parle au robot : « les bleus vont au poste C », « arrête les
rouges », « fais le point ». Un modèle de langage (ou, hors ligne, un
analyseur à règles) traduit la phrase en ordres, **validés contre une liste
fermée** avant d'être appliqués. Le superviseur ne peut que changer les
règles de tri, suspendre ou reprendre une couleur, et produire un rapport.
Il ne reçoit **jamais** l'accès au robot : « va plus vite » ou « coupe la
sécurité » sont refusés avec une explication. Le rapport est calculé par le
code à partir du bilan, jamais rédigé par le modèle.
Code : `axis_os/apps/tri_colis/superviseur.py` ; tests :
`test/test_superviseur_tri.py`.
**Démo : l'opérateur change une règle en pleine mission, le robot suit.**

### 6. Présentation
Tableau de bord en incrustation — colis triés, taux de réussite, arrêts de
sécurité —, vidéo accélérée, dossier investisseur.

---

## Répétition générale sans GPU (fait)

`test/test_tri_pile_logicielle.py` branche **toute la pile réelle** —
logique de tri, superviseur, façade, navigation, locomotion avec gardien de
proximité, manipulation — sur un entrepôt simulé cinématiquement : robot
qui imite GR00T WBC (bande morte, erreur d'exécution), lidar synthétique,
deux humains qui circulent, faux carton détecté au sol, 15 % de prises
ratées, réapprovisionnement, règle changée de vive voix à mi-parcours.
Trois humains : deux qui traversent, un collègue qui vient régulièrement
près du poste de prise. 30 minutes simulées en ~90 s sur un CPU.

Mesuré sur 30 min (avec dépose sur plateau) : **15 colis triés, tous posés
sur le plateau de leur poste, 100 % de réussite, 0 collision, le
robot ne s'est jamais rapproché d'un humain à moins de 0.75 m, aucun
mouvement de bras pendant un arrêt de sécurité**, ~8 min d'arrêts de
sécurité journalisés, bleus envoyés au poste C après l'ordre vocal, fin
bras vides. Sans le collègue : 19 à 20 colis.

Disposition v2, partagée avec la scène Isaac Sim
(`axis_os/apps/tri_colis/scene_demo.py`) : postes de dépose à 4-6 m.
Sur 30 min : 18 colis, 18/18 posés sur leur plateau, ~8 min d'arrêts de
sécurité dus aux passages humains — le réglage à doser pour la démo.

Couleur des colis : OWL-ViT trouve « une boîte », la couleur est lue sur
les pixels au centre de chaque détection (`couleur.py`). On ne parie pas
sur la capacité, non vérifiée, d'OWL-ViT à distinguer les couleurs.

Constats à reporter dans la scène Isaac Sim :
- **Débit** : un colis toutes les 90 à 120 s selon la présence humaine, à
  0.3 m/s avec des postes à 7 m. Pour viser plus de 20 en 30 min,
  rapprocher les postes de dépose (4-5 m) — c'est nous qui dessinons
  l'entrepôt. Les arrêts de sécurité coûtent du débit : c'est à montrer,
  pas à cacher.
- **Zones de poste** : les tables doivent être déclarées au gardien avec
  leur hauteur de chargement (`Rectangle.zmax`), sinon les cartons voisins
  du carton visé arrêtent le robot à chaque prise.
- **Dépose sur un plateau** : accostage fin au poste jusqu'à ce que le
  carton soit 12 cm à l'intérieur du plateau, descente des bras en boucle
  fermée sur la hauteur mesurée des mains, ouverture quand le fond du
  carton est à 1 cm du plateau. Plateau hors d'atteinte : le carton n'est
  PAS lâché de haut, il reste tenu et la raison est donnée. Pour repartir,
  le robot recule de 35 cm en ligne droite au lieu de pivoter collé à la
  table. Vérifié sur 30 min : 15/15 cartons posés sur leur plateau.
- **Obstacle immobile sur le trajet** : si le gardien voit un obstacle
  inexpliqué qui reste immobile sur le chemin pendant 4 s, la navigation
  le marque comme obstacle temporaire (60 s) et replanifie à 1.1 m de lui.
  Une personne qui passe ne déclenche rien. Le contournement se décide
  pendant le ralentissement : une fois arrêté (personne à moins de ~1 m),
  le robot ne manœuvre pas près d'elle, il attend ; au bout de 60 s l'app
  alerte l'opérateur. Couloir sans autre passage : attente, puis reprise.
  Fait et vérifié (`test/test_navigation_contournement.py`).
- **Bras figés pendant un arrêt** : quand le gardien arrête la marche, le
  ManipulationService fige aussi les bras (sans lâcher l'objet) et reprend
  le geste sans à-coup. Fait et vérifié.
- **Accostage fin** : la navigation amène à une pré-pose dégagée, puis un
  contrôleur d'accostage fait les 45 derniers centimètres (le planificateur
  ne peut pas approcher à 29 cm d'une table, par construction).

Manques identifiés, non traités :
- **Colis ramené au poste de prise** (poste de dépose inaccessible) : il
  est encore relâché à hauteur de port, faute de place connue sur la table
  de prise. Cas rare ; prévoir un emplacement de retour.
- **Marche arrière** : déjà validée sur GPU — W3 a reculé de 68 cm carton
  en main, W4 aussi, sans chute. Le recul de sortie s'appuie dessus.
- **Glissement du carton** : W3 a mesuré 8 cm en 11 s. Les trajets de la
  démo durent ~15-20 s carton en main : c'est le premier risque physique à
  surveiller sur GPU (serrage plus fort, ou poignets raidis).
- **Hauteur des mains en fonction du tangage d'épaule** : modélisée ici à
  ~0.3 m/rad (déduit de W3 : levé de 0.2 rad = +6 cm). La dépose est en
  boucle fermée sur la hauteur mesurée, donc robuste à cette valeur, mais
  le recul des mains vers le torse en descendant n'est pas modélisé : à
  mesurer sur GPU pour régler la marge de 12 cm à l'intérieur du plateau.

## Scène Isaac Sim T1 — prête, PAS ENCORE EXÉCUTÉE

`sim/validation/test_isaac_t1_tri_usine.py` branche sans les modifier les services
répétés sur CPU, sur la même disposition (`scene_demo.py`) : tables à
plateau fin et pied en retrait (leçon W3), stock de cartons colorés hors
site, silhouettes cinématiques de personnes, lidar simulé par rayons
PhysX (modèle Mid-360 inversé, `sim_lidar.py`), caméra de tête grand angle
+ profondeur, OWL-ViT, opérateur simulé (réapprovisionne, débarrasse),
ordre vocal à mi-parcours, tableau de bord incrusté dans la vidéo.

Machine neuve : `deploy/machine_gpu/installer.sh` (Docker, Isaac Sim 6.1,
politiques GR00T, meshes, OWL-ViT, code). Puis :

| Étape | Commande | But |
|---|---|---|
| 1 | `bash deploy/machine_gpu/lancer_t1.sh 240 1` | 4 min, vidéo temps réel : tout s'enchaîne-t-il ? |
| 2 | corrections, puis `lancer_t1.sh 600 4` | 10 min : premières pannes rares |
| 3 | `lancer_t1.sh 1800 10` | la démo : 30 min, vidéo de 3 min |

**Suivi en direct** (`axis_os/hal/drivers/isaac_sim/direct.py`) : pendant
le test, une page web montre la dernière image avec le tableau de bord, le
journal des événements, et un bouton « Arrêter proprement » — plus besoin
d'attendre 30 minutes pour découvrir une erreur de la deuxième minute.
Accès par tunnel SSH (`ssh … -L 8080:localhost:8080`, puis
http://localhost:8080) : la page n'est jamais exposée sur Internet.

Ce qui n'a jamais tourné sur GPU, donc à surveiller au premier essai :
- **OWL-ViT sur des cartons colorés unis** (W4 : carton brun, confiance
  ~0.09, seuil 0.06). S'il ne les voit pas : texture, ou autre requête.
- **Glissement du carton** sur 15-20 s de trajet (W3 : 8 cm en 11 s).
- **Hauteur réelle des mains** pendant la descente de dépose, et leur
  recul vers le torse.
- **Coût du lidar** (1 800 rayons Python à 10 Hz) sur le temps réel.
- **Silhouettes cinématiques** déplacées à chaque pas : si PhysX s'en
  plaint, passer à des cibles cinématiques.

## Prise mesurée (2026-10-28) — prête, PAS ENCORE VALIDÉE SUR GPU

Constat du banc `evaluer_vla.sh 5 1 --prise` (RTX PRO 6000) : réflexes
2/5. Trois « prise non confirmée » : mains 6 cm devant le centre du
carton, qui l'effleuraient sans serrer (serrage vers une posture fixe, sans
mesure) ; mains refermées dans le vide sur un carton hors de portée ;
avant-bras appuyés sur le plateau ; carton tenu loin du buste, penché de
~30° pendant le transport.

Ce qui change (`axis_os/services/manipulation/`, `prise.py`,
`cinematique.py`), quand le robot donne ses mesures (`MesuresBras`) :

| Étape | Ce que fait la prise | Refus ou échec, avec sa raison |
|---|---|---|
| Portée | cinématique de l'URDF : les mains atteignent-elles le milieu des faces ? | « hors de portée des bras » (OUT_OF_REACH), rien ne bouge |
| Distance à la table | la façade s'arrête à la distance où les avant-bras passent à 3 cm du plateau au moins (à partir de 0,29 m) | « avant-bras à X cm du plateau » |
| Placement | mains au milieu des faces (2,5 cm en profondeur, 3 cm en hauteur), 2 corrections au plus | « mains mal placées », « contact inattendu » (effort > 10 N·m) |
| Fermeture | chaque bras jusqu'au contact (hausse du couple ≥ 1,5 N·m, ou main qui ne suit plus), ralentie près du contact attendu | « rien entre les mains » (2 cm au-delà de la largeur), « ce n'est pas lui » (contact 5 cm trop tôt) |
| Serrage | +5 N·m visés, pas adapté à la raideur mesurée, jamais au-delà de 10 N·m (et de la moitié du couple max du garde-fou) | « serrage insuffisant », « serrage refusé » (effort absent, NaN, périmé) |
| Levage | 4 cm, phase à part ; le carton a-t-il suivi ? | sinon reposer, desserrer, recommencer une fois ; « prise NON confirmée (2 essais) » |
| Transport | carton contre le buste (face arrière à 10 cm du bassin), serrage gardé le long du chemin, couple surveillé | penché de plus de 15° : base arrêtée, une correction ; encore penché : arrêt propre, intervention |

Validation GPU à faire, dans cet ordre :

| Étape | Commande | Ce qu'on regarde |
|---|---|---|
| 1 | `bash deploy/machine_gpu/evaluer_vla.sh 5 1 --reflexes` | réflexes seuls, vidéo temps réel ; bandeau : mode, carton, phase, bras |
| 2 | `bash deploy/machine_gpu/recuperer_t1.sh` | vidéo 540p et journal dans un .zip |
| 3 | `bash deploy/machine_gpu/evaluer_vla.sh 20 2 --reflexes` | taux de réussite, échecs par raison, inclinaison max, contact des avant-bras (`~/videos/eval_vla.json`) |

Ce qui n'a pas pu être vérifié sans Isaac Sim (à surveiller au premier
essai) :
- **Le couple mesuré** : `get_applied_joint_efforts()` (celui que lit le
  garde-fou) donne-t-il bien l'effort du serrage au roulis d'épaule ? S'il
  reste nul, le contact n'est vu qu'au retard de la main, et le serrage
  finit en « serrage insuffisant » (sûr, mais raté).
- **Les seuils** : hausse de 1,5 N·m pour le contact (le poids des bras
  qui bougent peut la donner seul), 5 N·m de serrage pour tenir un carton
  de ce poids, 3,5 cm de rayon d'avant-bras.
- **La demi-épaisseur des mains** (4,5 cm, déduite du banc : mains à
  0,331 m sur un carton de 0,24 m) et l'écart entre la cinématique de
  l'URDF et le bras simulé.
- **L'inclinaison** du carton porté contre le buste en marchant.

Correctifs du 2026-10-28 (RTX PRO 5000) : prise refusée « hors de portée »
à tort (lacet du carton non replié, robot tourné d'un quart de tour) ; chute
déclarée à t = 0 en évaluation (physique avancée sans équilibre pendant les
attentes de rendu). Au prochain essai, le journal dit, juste avant la
boucle : « robot avant la boucle : X° d'inclinaison, après N pas de
physique sans équilibre — remis debout » ; avec « [tete_rgb] premières
données après N image(s) », il confirme (ou non) cette cause.

Calibrage des mains (2026-10-28, mains contre la face avant du carton) :
au début de la boucle, le journal donne « calibration des mains : écart
max … » et, pour chaque posture et chaque main, réel − modèle en cm.
Chaque prise ratée donne les mains (mesurées et du modèle), les cibles,
l'objet visé et la position réelle des cartons, dans le repère du bassin.
Lecture : si la calibration passe (modèle juste) mais que « objet visé »
est en avant du carton réel, c'est la position perçue du carton qui est
fausse en profondeur (caméra de profondeur, face avant + demi-profondeur),
pas la cinématique.

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
