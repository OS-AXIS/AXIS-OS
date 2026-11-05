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
  finit en « serrage insuffisant » (sûr, mais raté). *Constaté le
  5 novembre 2026 : nul (efforts imposés, commande en position).* Le pont
  lit désormais `get_measured_joint_efforts()` ; au prochain essai, vérifier
  la ligne « couples lus : effort MESURÉ » au démarrage, puis le verdict de
  la calibration des couples (« fiables », pente près de 1 ; près de -1 :
  signe inversé). Non fiables : la géométrie décide du contact et du
  serrage.
- **L'alerte d'arrêt prolongé** : un carton dérangé par essai, le premier,
  n'est remis en place par l'opérateur simulé qu'après 40 s (les autres :
  20 s) ; le journal doit montrer, avant, l'ALERTE « robot arrêté par la
  sécurité depuis 20 s, toujours pour la même cause ».
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

## Contact décidé en géométrie, et ce que l'OS reçoit en simulation (2026-10-29)

Constat (`evaluer_vla.sh 5 1 --reflexes`) : 0/2, « fermeture au bout de sa
course sans contact », faces intérieures à 0,239 m pour un carton de
0,24 m : les mains étaient CONTRE le carton. Le contact n'était cherché
qu'au roulis d'épaule (couple, retard) ; le roulis suivait sa consigne, et
c'étaient le poignet et le coude qui cédaient. Ce qui change :
- message d'échec : faces intérieures commandées et mesurées, et les trois
  plus grands écarts consigne-mesure du bras ;
- contact = couple, OU retard du roulis, OU face mesurée en retard d'au
  moins 1 cm sur la face commandée et à moins de 1,5 cm de la face attendue
  (2 pas de suite) ; la consigne s'arrête quand la face commandée atteint
  largeur/2 − 2 cm ; une main qui touche situe l'objet pour l'autre ;
- poignets et coudes raidis (gains kp ×2, plafonnés au plus grand gain du
  pilote du G1, 100) de la fermeture au dépôt ; gains normaux à l'échec,
  au dépôt, à la réinitialisation et sur arrêt d'urgence.

**Règle : en simulation, l'OS ne reçoit que ce que le vrai G1 à mains
caoutchouc fournit.** Ce que le banc T1 lui passait, et ce qu'il reçoit
maintenant :

| Entrée de l'OS | Avant (vérité simulation) | Maintenant |
|---|---|---|
| Couples des bras et du garde-fou | couple exact du solveur | couple ESTIMÉ : solveur + gain ±5 %, biais ±0,5 N·m par articulation, bruit 0,3 N·m (`bruit_couples.py`) |
| Position des mains | prims USD des mains | cinématique sur les angles mesurés |
| Prise vérifiée au levage | hauteur PhysX du carton | objet encore entre les mains (cinématique) et hausse du couple des roulis |
| Inclinaison du carton tenu | quaternion PhysX du carton | droite qui joint les mains, centrale inertielle (un carton qui tourne autour de cette droite n'est pas vu) |
| Hauteur du bassin | racine PhysX | cinématique des jambes et centrale inertielle (pied le plus bas au sol) |
| Position et cap du robot (navigation, gardien), pose de la caméra dans le monde | racine PhysX | **inchangé : écart restant**, il faut une localisation (odométrie, LiDAR) ; pour la prise, il s'annule (le carton perçu dans le monde est ramené dans le repère du bassin par la même pose) |

Restent du banc, sans effet sur les décisions : la calibration des mains
(prims USD contre cinématique, au démarrage), l'évaluateur (carton soulevé,
au sol, inclinaison réelle, garde des avant-bras), le journal (position
réelle des cartons), le juge et la vue du robot ci-dessous.

**Juge** (`sim/validation/banc_prise.py`, vérité simulation, invisible
pour l'OS) : capteur de contact PhysX sur chaque main. À chaque prise, le
journal donne par main « contact estimé par l'OS à t (signal), contact réel
à t (force) — vrai contact / CONTACT MANQUÉ / FAUX CONTACT » ; le bilan de
l'évaluation compte vrais, manqués et faux contacts.

**Vue du robot** (`sim/validation/banc_vue_robot.py`) : `t1_vue_robot.mp4`,
caméra de tête (celle de la perception), 640×480, 15 im/s, aussi en
incrustation dans `t1_tri_usine.mp4` et à côté de la vue extérieure sur la
page de suivi. Calques de l'OS (jaune) : détections OWL-ViT (couleur,
score), points de la face avant, contour estimé et centre visé, cibles des
mains (ouverture, contact, borne) et mains mesurées, phase, contact estimé
par main. En rose et pointillé, « vérité simulation » : contour réel du
carton, contact réel du juge. `--vue-sans-calques` les retire (vidéo de
démo).

À vérifier au premier essai GPU : le capteur de contact (le journal dit
« juge des contacts : actif sur … » ou « indisponible ») ; « hauteur du
bassin : estimée X, vraie Y » avant la boucle (écart attendu sous 1 cm) ;
les nouvelles raisons d'échec, avec les écarts consigne-mesure.

## Force estimée à la main, serrage en cartésien (2026-10-30)

Constat (PR 16, 3 essais) : contact vu des deux mains, puis 3 fois « serrage
insuffisant (0,1 / 0,4 / 0,1 N·m au plus serré) ». Le serrage ne bougeait
que le roulis d'épaule et ne mesurait que son couple.

Ce que dit la statique (cinématique de l'URDF, vérifiée contre Pinocchio
4.1 à 1e-16) : aux postures de prise de T1, le roulis a un bras de levier de
0,15 à 0,26 m pour une force latérale à la main, autant que le lacet d'épaule
(0,23 m). Un couple de roulis de 0,1 à 0,4 N·m veut donc dire une force à la
main de 1 à 2 N seulement : le carton s'est dérobé plutôt que la force n'est
passée ailleurs. Cela concorde avec son lacet réel qui change pendant la prise
(-3,5° → +13°) et avec des mains 2,5 à 5 cm trop en avant du milieu de ses
faces (perception, PR B). La force estimée et le juge le diront au prochain
essai.

Ce qui change :
- **Force à la main** : F = (J Jᵀ + λ² I)⁻¹ J Δτ sur les 7 articulations du
  bras (λ = 2 cm), Δτ la hausse des couples depuis la fermeture, poids des
  bras retiré (masses de l'URDF), composante le long de la normale du carton,
  moyenne glissante sur 5 lectures. Bruit : 0,85 N par lecture pour 0,3 N·m
  par couple, ~0,4 N filtré ; mesuré au repos avant chaque fermeture et donné
  au journal (« bruit de la force estimée au repos »).
- **Contact** : couple du roulis (poids retiré), retard du roulis, force à la
  main (3 N, jamais sous 6 fois le bruit filtré mesuré), ou géométrie (retard
  pris depuis le début de la fermeture : bras affaissés, la main mesurée est
  déjà à côté de la commandée).
- **Serrage** : la face commandée de chaque main avance vers le carton
  (cinématique inverse sur tout le bras, pas de 0,1 à 0,5 mm), jusqu'à 8 N à
  la main, refus sous 5 N ; jamais au-delà de largeur/2 - 2 cm (la main là où
  elle est vraiment : consigne plus son décalage mesuré avant le contact).
  Plafond de couple sur CHAQUE articulation du bras ; une main arrêtée par son
  plafond ne serre plus. Pourquoi 8 N : aux gains du banc (raidis), le bras
  fait ~500 N/m à la main, soit ~10 N au plus pour 2 cm ; un carton de 1 kg
  entre deux mains caoutchouc demande 4,9 N à chacune.
- Lacet d'épaule et poignets commandés pendant la fermeture et le serrage,
  rendus à leur consigne d'avant en fin de prise ; lacet d'épaule raidi aussi.
- **Gestes avec l'objet** (levage, transport, dépose) : les mains commandées
  au serrage suivent en translation le milieu des mains mesurées ; leur avance
  dans le carton, le serrage, reste la même en cartésien. Avant, la cinématique
  inverse de référence oubliait le lacet d'épaule et les poignets serrés : la
  main d'un carton décalé de 2 cm lâchait au premier pas du levage. Au buste,
  le bras est plus raide : la force monte (18 N en simulation pour 9 au
  serrage), toujours sous le plafond de couple de chaque articulation.
- Journal d'échec : force estimée par main et couple de chaque articulation.
- **Juge** : rapport de contact PhysX posé sur les corps des mains AVANT
  world.reset() (capteur de contact d'Isaac en repli, extension activée), avec
  les raisons d'un échec au journal ; lacet réel du carton à chaque phase.
- **Vue de diagnostic** (`t1_vue_diagnostic.mp4`, banc seulement) : la caméra
  de tête ne voit ni les mains ni le carton proche au moment de la prise (son
  bord bas passe ~40° sous l'horizontale, les mains sont à ~65°) ; caméra
  plongeante au-dessus et en arrière des mains, incrustée avec la vue du robot
  et sur la page de suivi ; flèche de la force estimée à chaque main.

À vérifier au premier essai GPU : « juge des contacts : actif (…) » ; le
bruit au repos (~0,85 N attendus, plus si le bassin bouge) ; la force estimée
contre le contact réel du juge ; le lacet réel du carton à chaque phase.

## Perception du carton : masque, nuage de points, boîte orientée (2026-11-03)

Constat (PR 16) : mains 2,5 à 5 cm trop en avant du milieu des faces. Le
centre perçu était le point moyen de la face avant, pris dans le cadre
d'OWL-ViT qui déborde sur la table, le fond et les mains : pas le centre du
carton.

L'OS ne lit que les signaux du vrai G1 à mains caoutchouc : image et
profondeur de la caméra de tête, dont la pose vient de son **montage** sur
le bassin (connu, comme dans l'URDF) et de la pose du bassin (localisation,
centrale inertielle), et points du MID-360. Aucun capteur ajouté. Le banc
T1 ne met plus dans l'arbre des repères la pose de la caméra lue dans la
scène : il la compare une fois à celle de l'OS (« montage de la caméra de
tête : pose de l'OS à X cm et Y° de celle de la scène »).

La chaîne (`axis_os/services/perception/boite.py`, `sam2_segmenteur.py`,
la façade de l'app de tri) :
1. **Masque** du carton dans le cadre du détecteur : SAM 2.1 tiny (Meta).
   Sans lui (modèle absent ou en échec), les points du cadre entier.
2. **Nuage** : profondeur des pixels du masque, repère monde.
3. **Table** : plans horizontaux par RANSAC à un point (la verticale est
   connue) ; la table est le plus haut plan au moins une demi-hauteur de
   carton sous son dessus. Seuls les points au-dessus restent.
4. **Boîte orientée** : hauteur par le dessus ; lacet et centre par le
   rectangle d'aire minimale du dessus vu d'en haut ; pour la face cachée,
   les dimensions connues de l'app (centre = bord avant + profondeur/2 si le
   dessus n'est pas vu en entier).
5. **Fusion** des images de l'approche (3 s, 6 images au plus ; remise à
   zéro si le carton a bougé de 15 cm), plus les points du MID-360 près du
   carton.
6. **Résidu** (écart quadratique moyen des points à la surface de la boîte)
   et couverture du dessus au journal (« carton rouge vu : centre (…), lacet
   …, résidu …, couverture … ») ; au-delà de 1,5 cm, ou sous 60 % de
   couverture, « prise refusée — carton mal vu », jamais deviné.
7. **Recalage au toucher** : au contact des deux mains, avant de serrer, le
   milieu de leurs faces mesurées recale le centre le long de la normale
   (« recalage au toucher : carton à +2,0 cm de la perception », au journal) ;
   au-delà de 3 cm, prise refusée. Le centre recalé devient celui du plan de
   prise (dépose, journal, vue du robot) ; le levage et le transport suivent
   déjà les mains mesurées. Les bornes du serrage partent alors des faces
   MESURÉES au toucher, plus du centre estimé : aucune main ne va plus de
   2 cm dans la face qu'elle touche, et une main déjà au-delà recule jusqu'à
   sa borne avant de serrer. Avant de fermer, chaque main s'ouvre à 5 cm de
   sa face perçue (`jeu_ouverture_m`) : l'erreur de position acceptée (3 cm,
   `recalage_max_m`) plus 2 cm de marge (`marge_ouverture_m`, pour l'écart
   entre main commandée et main réelle des bras qui s'affaissent : 2 cm
   constatés). Des réglages qui ne tiennent pas cette relation
   sont refusés dès leur construction. Filet : juste avant de fermer, la
   force ABSOLUE de chaque main vers l'objet (poids du bras retiré par le
   modèle), au-delà de 3 N ou de 4 fois le bruit mesuré au repos, refuse la
   prise (« main déjà en appui sur l'objet ») ; sa valeur est au journal
   (« force absolue à la main avant de fermer »), pour régler ce seuil
   provisoire sur les essais GPU. Le lacet n'est pas observable par deux mains
   plates à la même profondeur : la largeur mesurée en borne l'écart (au
   journal). La profondeur ne l'est pas non plus : elle reste celle de la
   perception.

Mesuré sur des images de synthèse (lancer de rayons depuis la caméra de tête
de T1, carton pris à 0,34 m, approche de 1,20 à 0,85 m ;
`test/test_perception_boite.py`) :

| Cas | Centre | Lacet |
|---|---|---|
| Tourné de 0, 5, 10, 15 et −8° | 0,32 cm au plus | 0,0° |
| 34 % caché par les mains | 0,31 cm | 0,0° |
| Profondeur bruitée (0,5 % de la distance) | 0,59 cm | 0,09° |
| Sans masque, cadre débordant sur les mains | résidu 72 cm : refusé | — |

**Détecteur** : OWLv2, seuil 0,10, par défaut ; OWL-ViT (seuil 0,06) en
repli s'il ne se charge pas, ce que dit le journal (`detecteur_carton`,
`owlvit_detector.py`). Verdict du banc sur les images de l'essai GPU du
5 novembre 2026 : OWLv2 voit 30 cartons sur 30 au seuil 0,10 avec 47 fausses
détections, OWL-ViT 30 sur 30 au seuil 0,06 avec 66. `--detecteur owlvit`
inverse la préférence. Pour comparer de nouveau : lancer T1 avec
`--images-perception DOSSIER`
(une image par seconde de recherche, avec la vérité de la simulation),
puis, dans le conteneur :
`/isaac-sim/python.sh /isaac-sim/axis_os/banc_perception.py DOSSIER`. Par
détecteur et par seuil : cartons vus, fausses détections, IoU moyen ; au
seuil de travail, l'erreur de la boîte ajustée avec le masque et avec le
cadre seul ; puis un verdict.

**Vue du robot** : masque (teinte de la couleur du carton), points retenus,
bord du plan de la table, boîte ajustée (pleine, avec son résidu), boîte
recalée au toucher (vert) ; la vérité en rose et pointillés.

**Licences et versions** (vérifiées sur Hugging Face et PyPI ; rien
d'AGPL, rien de non commercial) :

| Brique | Licence | Version épinglée |
|---|---|---|
| OWL-ViT `google/owlvit-base-patch32` (poids) | Apache-2.0 | commit `85fa9bec488a03a73b7f29206b1550551d0a7556` |
| OWLv2 `google/owlv2-base-patch16-ensemble` (poids) | Apache-2.0 | commit `cfd3195ba4ea9592eec887ded089f4c08eff231d` |
| SAM 2.1 `facebook/sam2.1-hiera-tiny` (poids) | Apache-2.0 | commit `de431c4043854a71d8101e17995dfe596bf101a5` |
| transformers (code d'OWL-ViT, OWLv2, SAM 2.1) | Apache-2.0 | 5.17.0 |
| numpy (RANSAC, rectangle minimal, boîte) | BSD-3-Clause | 2.5.3 (requirements-ci.txt), déjà une dépendance |
| Pinocchio (test de référence de la cinématique, PR A) | BSD-2-Clause | 4.1.0, jamais une dépendance de l'OS |

Poids en safetensors seulement, révision refusée si elle n'est pas un
commit ; `installer.sh` les pré-télécharge aux mêmes révisions
(`test/test_deploiement.py` le vérifie). Aucune dépendance nouvelle. Écartés :
Open3D (MIT), dont la roue aarch64 n'existe qu'en manylinux_2_35 (0.20,
JetPack 6) ou pour Python 3.11 au plus (0.18), d'où RANSAC et rectangle
minimal en numpy ; MobileSAM (MIT), qui demande son propre paquet quand
SAM 2.1 est déjà dans transformers ; et, d'office, Ultralytics (AGPL-3.0),
FoundationPose (licence NVIDIA non commerciale) et cuRobo.

À vérifier au premier essai GPU : la ligne « montage de la caméra de tête »
(attendu sous 0,5 cm et 0,5°) ; « [sam2] … chargé » ; au journal, « carton
… vu » (résidu, couverture) puis « recalage au toucher » à chaque prise ; la
comparaison des deux détecteurs sur les images enregistrées ; « force absolue
à la main avant de fermer » à chaque prise (seuil provisoire de 3 N, à régler),
et que l'ouverture de 5 cm passe la portée et la garde de l'avant-bras
au-dessus du plateau de T1.

## Essai GPU du 5 novembre 2026 : aucune prise tentée, ce qui a changé

Bon : boîte ajustée par masque SAM 2.1, centre à 0,4 cm (90 % sous 1 cm),
lacet 0,0°, aucune refusée sur 30 ; calibration des couples (biais de
tangage d'épaule +3,75 et +3,53 N·m, résidu max 0,81 N·m). Mais aucune
prise : à chaque accostage, « carton pas revu de façon cohérente après
l'arrêt », et des attentes de sécurité sans raison.

- **Pose d'attente** : elle était à 0,28 m fixe derrière la pose de prise ;
  le carton proche y remplissait le bas de l'image et en sortait (son arête
  avant basse 52° sous l'horizontale, le bord de l'image à 38,6°). Elle est
  désormais calculée : la plus proche d'où les 8 coins du carton tombent
  dans l'image, à 5 % du bord, d'après la caméra (paramètres, montage),
  la hauteur de la base et le carton : 0,49 m sur la scène T1 (banc
  synthétique de `test/test_cadrage_attente.py`). Au journal,
  « pose d'attente à … m de la pose de prise : carton entier dans l'image ».
- **Fantômes** : plus de seuil en part d'image (25 %, qui écartait le carton
  proche). Écartée : une détection dont le cadre touche les quatre bords de
  l'image, ou dont la taille métrique (masque et profondeur) ne peut pas
  être celle d'un carton.
- **Raisons** : chaque image d'observation écartée l'est avec sa raison
  exacte (« n°<images> : détection <confiance> (<part> % de l'image, coupée
  par le bord bas) <raison> »), et non plus « carton pas vu ».
- **Bande morte** : GR00T WBC déclare la sienne, 0,05 (son seuil de
  politique d'équilibre) ; près du carton, une consigne au plancher (0,06)
  ralentie à ×0,95 part telle quelle, le robot avance encore.
- **Écarts x de +2,1 et −3,3 cm** (recherches de 41,0 et 119,9 s, avant les
  2ᵉ et 5ᵉ accostages, cartons rouges de gauche puis de droite) : erreurs de
  direction (1,2° et 1,9° à 1 m), pas de position du carton, que la visée,
  robot arrêté, corrige au millimètre (« correction latérale +2,2 » et
  « −3,4 cm »). Ni la fusion (une seule image par recherche), ni le masque
  (le même, robot arrêté, donne 0,4 cm). Hypothèse la plus probable, pas
  démontrée : la recherche part dès l'arrivée de la navigation, tête encore
  en rotation (environ 27°/s dans la vue du robot à 41 s), et sur le banc
  l'image couleur (anticrénelage temporel du rendu) peut retarder d'un rendu
  sur la profondeur et sur la pose. Remède : arrivé au poste par la seule
  navigation, la base s'arrête, et la recherche ne part qu'au bout de
  `duree_visee_s` (0,6 s), le critère de la visée. La ligne PERCEPTION
  donne la vitesse de lacet de la base, à vérifier au prochain essai ; les
  images de `--images-perception` diront si le carton était décalé entre
  couleur et profondeur.

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
