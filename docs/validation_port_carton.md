# Validation — prendre un carton, le porter, l'emporter (2026-09-25)

G1 29 DOF, politique GR00T Whole-Body Control, Isaac Sim 6.1, sur une
instance L4. Reproductible via `sim/validation/test_isaac_w3_gr00t_table.py`.

---

## 1. Résultat

**Le robot prend un carton d'un kilo posé sur une table, le soulève,
recule, fait demi-tour et l'emporte — sans le lâcher, sans tomber, sans
une seule consigne refusée par le garde-fou.**

| Phase | Hauteur du carton | |
|---|---|---|
| Posé sur la table | 0.900 m | |
| Soulevé | **0.95 m** (max) | +5 cm |
| Recul, 68 cm | 0.93 m min | tenu |
| Demi-tour sur place | 0.89 m min | tenu |
| Marche avant, 1.07 m | 0.87 m min | tenu |
| Arrêt | 0.84 m min | tenu |

Distance du carton au bassin : **0.38 m, constante** du soulèvement à
l'arrêt. 901 pas de locomotion, 0 chute, 0 refus.

**Réserve :** le carton glisse lentement — 8 cm en 11 s. La prise est
bonne, pas définitive : sur un long trajet, il finirait par tomber.

---

## 2. Onze essais, sept problèmes, chacun silencieux

| Essai | Symptôme | Cause | Correctif |
|---|---|---|---|
| W1 | tout passe | **mais** la posture « de port » envoyait les bras en arrière | voir §3 |
| W2-1 | plantage | taille du carton changée en cours de simulation | taille fixée à la création |
| W2-2 | mains derrière le bassin | étiquettes anatomiques inversées | posture par cinématique |
| W2-3 | carton tombe d'emblée | placé entre des mains encore ouvertes | scène sur table |
| W3-1 | robot recule de 26 cm | la table poussait sa cuisse | table avancée |
| W3-2 | **mains traversent le carton** | **URDF officiel : mains sans collision** | copie dérivée |
| W3-3 | carton bascule, glisse | poignets trop souples (raideur 4) | poignets raidis |
| W3-4 | carton tombe par le bord | 3 cm d'appui sur la table | plateau fin + pied |
| W3-5 | lâché en marche avant | frottement par défaut | matériau caoutchouc |
| W3-6 | plantage | matériau posé après l'initialisation | posé avant |
| **W3-7** | **tout tient** | | |

---

## 3. Trois découvertes qui dépassent ce test

**Les étiquettes anatomiques de la table de conventions sont fausses**
pour le tangage d'épaule et le coude. Un tangage positif recule le bras ;
un coude positif le déplie. Toute app qui demande une « flexion » obtient
le mouvement opposé. Voir `integration_gr00t_wbc.md` §8. À corriger par un
contrôle systématique en cinématique directe.

**L'URDF officiel du G1 29 DOF n'a aucune collision aux mains.** Les liens
`left_rubber_hand` et `right_rubber_hand` ne sont que des visuels. Tout
utilisateur de ce fichier pour de la manipulation a le même problème. À
signaler à Unitree. Contourné par une copie dérivée, testée pour rester
fidèle à l'original.

**Règle d'Isaac Sim** : tout ce qui touche la physique — ajout ou
suppression d'objet, taille, matériau — se pose **avant**
`world.reset()`. Après, la vue physique est invalidée. Trois plantages
de la même famille (A3, W2, W3).

---

## 4. Ce qui reste pour le Graal complet

- **Serrage plus ferme ou prise contre le torse**, pour supprimer le
  glissement lent.
- **Approche** : le robot part déjà placé devant la table. Il faudra qu'il
  la trouve, s'en approche et se positionne au centimètre.
- **Dépose** sur une autre table.
- **L'app** qui orchestre le tout, pilotée à la voix.
- **Arbitrage des actionneurs** : ici, c'est le script qui commande les
  bras via l'adaptateur de marche. Dans l'OS, l'app devra posséder les
  bras pendant que la marche possède les jambes.

---

## 5. W4 — « Va prendre le carton », sans lui dire où il est

`sim/validation/test_isaac_w4_gr00t_autonome.py`. Le robot ne reçoit **aucune position**.
Le carton est posé sur une table à 90° sur sa gauche, hors de son champ de
vision au départ.

| t | État | |
|---|---|---|
| 0.0 s | CHERCHER | il tourne sur lui-même |
| 3.3 s | APPROCHER | OWL-ViT reconnaît le carton — face avant mesurée à 4 cm près |
| 12.9 s | SAISIR | le carton est entre ses mains : il serre |
| 13.8 s | SOULEVER | |
| 15.0 s | RECULER | prise **vérifiée** avant de partir |
| 17.5 s | TOURNER | demi-tour, carton en main |
| 21.8 s | EMPORTER | |
| 25.0 s | DÉPOSER | au sol, à 2.30 m de la table |
| 27.5 s | FINI | |

0 chute, 0 refus du garde-fou.

**Ce qui relève de l'IA :** la recherche et la localisation (OWL-ViT +
profondeur), l'équilibre et la marche sous charge (GR00T WBC). **Ce qui
reste programmé :** le mouvement des bras — l'app sait *comment* prendre,
la perception lui dit *quand* et *où*. La décision de serrer vient de la
perception, pas d'une seconde fixée d'avance.

### Quatre essais, trois leçons

**La caméra doit regarder là où est l'objet.** Premier essai : rien reconnu
en une minute. La vue enregistrée par la tête du robot a montré le carton
coupé par le haut du cadre : visée reprise des tests où les cartons étaient
au sol, et angle mal calculé (36° vers le bas au lieu de 20°). **Enregistrer
ce que voit le robot** a tranché en un coup d'œil, là où on aurait accusé le
détecteur.

**Sous 0.05 m/s, GR00T WBC ne marche plus : il tient debout.** Des consignes
d'ajustement fin de 2 à 3 cm/s ne font rien bouger. Les petits pas doivent
être francs.

**La règle d'un humain vaut mieux que la précision d'une machine** (règle
de Kenzo). Exiger une immobilité parfaite près d'une table échouait : la
politique de marche ne s'immobilise jamais tout à fait. « Dès que le carton
est entre les mains, on serre » a réussi du premier coup — vérifié avec ce
que le robot sait vraiment : ses mains par ses articulations, le carton par
sa caméra.

### Limites

- **Fin d'approche à l'aveugle** : sous ~0.9 m, l'estimation est figée et
  recalée par la pose du robot, exacte en simulation. Sur le vrai robot, il
  faudra une odométrie.
- **Prise vérifiée** par la position simulée du carton. Sur le vrai robot :
  l'effort mesuré aux épaules.
- **Dépose au sol** par ouverture des bras, pas encore sur une table.
- Tout le déroulé vit dans un script de test, pas encore dans une **app** du
  Play Store : il faut d'abord l'arbitrage des actionneurs entre services et
  la correction des étiquettes anatomiques.

---

## 6. Essai T1 du 5 novembre 2026, 21 h 57 : prise parfaite, glissement au port

Prise parfaite (point de contact au milieu de la face, force estimée 6,46 N
contre 6,5 au juge, ligne des mains à 0,0 cm du centre de gravité), levée ;
puis le carton glisse au port. L'OS le voit (« le carton ne pèse plus sur
les mains (3.5 N, 11.3 au levage) ») et s'arrête proprement, aucun carton au
sol. Rejoué dans `test/test_rejeu_port_t2157.py`
(`test/donnees/t1_prise_20261105_2157.csv`).

### Ce que disent les données

| Constat | Valeur |
|---|---|
| Poids mesuré par l'OS | 10,6 N en haut du levage, 11,1 au plus (vrai : 9,8) |
| Serrage réel (juge) | 6,3 N par main |
| Marge de frottement μ (N_G + N_D) / poids, μ = 0,8 | 0,8 à 0,9 au port |
| Écart des faces mesurées au port | constant à 2 mm près ; chaque face bouge de 5 cm |
| Consigne de la main droite | desserrée de 5 mm au port, jamais resserrée |
| « Contre le torse » | à 3 N, face arrière du carton à 33 cm du bassin, torse attendu à 10 |

- **Les mains ne s'écartent pas** : la marche balance l'ensemble mains et
  carton (± 2,5 cm) ; une face prise seule semble s'écarter, l'écart des
  deux ne bouge pas.
- **La consigne qui change** : le tangage du poignet droit porte 2,0 à
  2,7 N·m, surtout du poids ; à chaque pointe au-delà de son plafond
  (2,5 N·m), `_garder_sous_la_limite` desserre d'un pas : 5 mm, soit ~1 N de
  serrage perdu, pour 0,04 N·m de moins au poignet.
- **La traction** : torse hors d'atteinte, la poussée de 3 N vers le robot
  n'était pas le torse.

### Ce qui change

- **Serrage d'après le poids** (`ReglagesPrise.securite_serrage` = 2,
  `frottement_serrage` = 0,8, `marge_serrage_n` = 1) : en haut du levage, la
  prise confirmée, l'OS serre jusqu'au serrage requis
  `securite × poids / (2 μ)` (le plus grand poids mesuré depuis le levage),
  visé avec 1 N de marge, avant de ramener. Consigne au plus
  `penetration_poids_max_m` = 6 cm dans la face touchée (raideur au contact
  ~300 N/m sur GPU : 15 N demandent ~5 cm). Plafonds de couple inchangés :
  on ne serre plus si une articulation des deux bras est à moins de 0,3 N·m
  du sien. Bloqué sous le requis pendant 3 s : le carton est reposé
  doucement, « trop lourd pour ces mains ». Faces mesurées qui se
  rapprochent de plus de 1 cm : « le carton cède sous le serrage », reposé.
  Au journal : `serrage d'après le poids : G … N, D … N par main pour … N
  de poids (requis …, visé …)`.
- **Ramener** : si le carton ramené ne peut pas toucher le torse, il est
  rapproché en position, sans arrêt sur un effort vers le robot (au journal :
  « (torse hors d'atteinte) »). Torse atteignable : l'arrêt au toucher reste.
- **Port** : la consigne desserrée par un plafond est rendue dès que les
  bras le permettent ; mains écartées de plus de 5 mm de leur serrage (somme
  des deux mains), ou marge de frottement sous 2 : la base s'arrête, on
  resserre ; sans retour en 1 s, à reposer (« serrage perdu au port »).

### Ce que le rejeu prévoit pour le prochain essai

Le serrage, au point de contact 11 cm le long de la main, charge les
poignets (lacet surtout, ~0,15 N·m par newton). Pour ce carton (11,1 N
mesurés), il faut 13,9 N par main : les poignets y seraient à 2,1 N·m en
médiane, 2,6 en pointe, pour un plafond de 2,5. L'OS s'arrêterait vers 13 N
par main (marge 1,9) et **reposerait le carton, « trop lourd pour ces
mains »**, de peu. Les leviers sont des réglages, pas du code :
`frottement_serrage` (le banc : caoutchouc 1,0, carton 0,8 en frottement
dynamique), `securite_serrage`, le poids mesuré par l'OS (13 % au-dessus du
vrai sur cet essai), ou la part du plafond des poignets.

---

## 7. Essai T1 du 6 novembre 2026, 18 h 21 : poids gardé trop haut, carton tordu au retour

Couples fiables, perception, contact et ligne des mains bons (juge : 2 vrais
contacts, 0 manqué, 0 faux). « serrage d'après le poids : G 14.6 N, D 14.4 N
par main pour 11.4 N de poids (requis 14.3, visé 15.3) » ; au port, marge de
frottement 1,58 puis 1,93 malgré le resserrage : à reposer. Le carton n'est
pas tombé. Rejoué dans `test/test_rejeu_port_t1821.py`
(`test/donnees/t1_prise_20261106_1821.csv`).

### Le poids : 11,4 N pour 9,8

- **L'estimation est juste bras immobiles :** les 15 lectures qui suivent la
  confirmation de la prise, en haut du levage, ont une médiane de 9,85 N
  (essai de 21 h 57 : 10,0 N en fin de levage).
- **La référence ne l'était pas :** c'était le plus grand poids lu depuis le
  levage. Elle a gardé 11,44 N, la toute première lecture après la remise à
  zéro de la moyenne glissante à la confirmation, une seule lecture, bruitée.
  Au retour, bras en mouvement et carton tordu, l'estimation a monté
  jusqu'à 11,8 N.
- **Ce qui n'est pas en cause :**
  - le modèle de pesanteur et le poids des avant-bras : résidu de la
    calibration 0,10 N·m au plus, pente 1,00 ; rejoué sur les couples de
    l'essai, le modèle des bras de l'URDF donne 9 à 10 N, torse supposé droit ;
  - le point de contact : le vrai est à 12,4 cm le long de la main (11 dans
    le modèle), 2,5 cm au-dessus de son axe, à la surface de la paume ; le
    prendre change l'estimation de 5 % au plus, et la fait passer sous le
    vrai poids : il reste à 11 cm, réglé sur la force normale (essai de
    16 h 06) ;
  - le frottement vertical, porté à la surface de la paume : sans effet,
    bras immobiles.

**Ce qui change :** le poids est pesé une fois, bras immobiles, en haut du
levage, avant de serrer plus : la médiane d'une fenêtre de
`filtre_force × 3` lectures (`ManipulationService.poids_carton`, au
journal). C'est sur lui que se calculent le serrage requis et la marge de
frottement au port. Le plus grand poids lu ne sert plus qu'au contrôle de
glissement du port (inchangé). Le serrage ne monte plus en levant : il monte
en haut, une fois le carton pesé.

### Le retour : les deux mains ne bougeaient pas comme un seul corps

La cinématique inverse du retour ne tenait que la **position** des points de
contact (tangage et roulis d'épaule, coude). Reconstruites sur les angles
mesurés :
- les paumes ont piqué de ~15° autour de la ligne des mains ;
- la gauche a roulé de ~10° de plus que la droite (l'épaule gauche a roulé de
  −10,5°, la droite de +0,5°) ;
- le carton s'est tordu entre elles : 9,7° de rotation, 4,6 cm de glissement
  à la main gauche, 6 N de serrage perdus du début du retour au port.

**Ce qui change :**
- **Corps rigide :** au retour, chaque main garde son orientation et suit la
  même translation, à `tol_rigide_deg` (1°) et `tol_rigide_m` (1 mm) près,
  par une cinématique inverse de la pose entière sur les articulations que
  la prise commande. Le mode reste en place, objet tenu, pour les gestes
  qui suivent (dépose). Là où il n'arrive pas, le geste est refusé (hors
  d'atteinte), comme avant : l'ancienne cinématique, partie des angles du
  serrage, ferait tourner les mains d'un coup (60° constatés sur le robot
  simulé, dans une descente vers un plateau trop bas).
- **Chemin vérifié d'avance :** étapes de 1 cm, aucune articulation qui
  saute de plus de 0,15 rad d'une étape à l'autre, avant-bras jamais plus
  près du plateau que 3 cm (ni qu'au départ, en haut du levage).
- **Sinon :** on rapproche moins, ou seulement plus haut, ou pas du tout :
  le carton est porté là où le levage l'a laissé (au journal).
- **Dépose et montée au-dessus d'un plateau, objet tenu :** tout le chemin
  est vérifié avant de bouger. La dépose le vérifie par étapes de 2 mm, le
  pas qu'elle commande à chaque tour ; la montée par étapes de 1 cm. Plateau
  hors d'atteinte : l'objet reste tenu là où il est, sans bouger (« dépose
  impossible » ; FAILED/OUT_OF_REACH pour une app).
  - Avant : une descente arrêtée en chemin ne remontait pas forcément. Sur
    le robot simulé, coude en butée, la pose rigide ne se tenait plus là où
    le retour passait.
  - L'interpolation des angles prenait alors le relais : mains tournées de
    30°, carton tombé de 7 cm, retour arrêté par la surveillance (test SDK,
    5 fois sur 6).
  - En corps rigide, les angles ne sont plus jamais interpolés : un geste
    dont la pose ne se tient plus s'arrête au dernier point tenu, et l'OS
    sait où sont les mains.
- **Prix :** les plateaux bas sortent d'atteinte en corps rigide. Sur le
  robot simulé, la limite est vers 70 cm pour un carton tenu à 30 cm du
  bassin, et 75 cm à 41 cm. Les plateaux de la scène T1 sont à 80 cm.
- **Six articulations au moins :** la pose entière d'une main demande six
  articulations commandées, tangage du poignet compris. Un bras à cinq
  (HAL 23 DOF) ne ramène pas le carton.
- **Rejoué :** sur la posture réelle de la fin du levage de l'essai, le
  chemin rigide passe (10,5 cm vers le buste, 5 cm plus haut), mains à
  0,00° de leur orientation de départ. C'est le tangage du poignet qui
  compense : de +0,03 à +0,5 rad.
- **Surveillance :** pendant le retour en corps rigide, le roulis (autour de l'axe avant) et
  le tangage (autour de la ligne des mains) du carton sont estimés sur les
  mains mesurées : rotation moyenne des paumes, et pente de la ligne des
  mains. Au-delà de `rotation_ramener_max_deg` (4°), le geste s'arrête et
  l'on resserre avant de marcher : la base reste arrêtée jusqu'au serrage
  visé, et d'au moins `marge_serrage_n` de plus à chaque main. Les couples
  ne servent pas ici : le couple de basculement passe sous leur bruit.
  Rejouée sur l'essai, cette surveillance aurait arrêté le retour à
  43,66 s, carton tourné de moins de 3°.

### Le port rejoué

Sur le poids pesé (9,85 N), avec le serrage visé gardé par le retour rigide
(13,3 N par main, que les mains ont dépassé pendant l'essai), les baisses de
force du port reprises : marge de frottement 2,13 à 2,16. Essai de 21 h 57,
sur son poids pesé (10,0 N) : les poignets laissent monter le serrage au
requis (12,5 N par main), et la marge reste entre 2,08 et 2,20 pendant tout
le port rejoué.

### À vérifier au prochain essai GPU

- Les lignes du journal :
  - `poids du carton : … N (médiane de 15 lectures …)`, près de 9,8 ;
  - `serrage d'après le poids : …`.
- Au retour, l'une de ces lignes :
  - `carton ramené sans toucher le torse : …`, sans `retour arrêté` ;
  - ou `retour : … porté là où le levage l'a laissé`.
- Sur la vérité du banc, le roulis et le tangage du carton au retour, sous 2°.

## 8. Campagne de cartons variés : la prise hors du seul carton T1

La prise a été mise au point sur un seul carton : celui de la scène T1, 24 ×
20 × 20 cm, ~1 kg, posé toujours sur la même table, à la même place. La
campagne essaie l'OS sur des cartons tirés au hasard, graine fixée, donc
rejouables (`axis_os/testing/cartons.py`, `test/test_campagne_cartons.py`).

### Ce qui est tiré, et comment c'est essayé

- **Cartons** : largeur 15 à 35 cm, profondeur 15 à 30, hauteur 10 à 30,
  masse 0,2 à 2,5 kg, frottement des mains sur leurs faces 0,5 à 1,2, lacet
  de −45 à 45°, retrait de 0 à 25 cm depuis le bord de la table (dans et
  hors de portée), décalage le long du bord.
- **Hors des moyens, exprès** : un carton sur dix trop large (45 à 60 cm),
  un sur dix trop lourd (3 à 5 kg).
- **Rejouable ailleurs** : le tirage ne dépend que de
  `random.Random(graine).random()` ; le banc Isaac tire les mêmes cartons
  (`lancer_t1.sh --cartons-varies N`, graine au journal).
- **Comme sur le banc T1** : approche face à la table à la distance de
  prise de l'OS (erreur de navigation de ±2 cm et ±3° comprise), bras ouverts
  pour ce carton, insertion, prise, port, dépose sur la table. Les couples
  estimés portent le bruit, le biais et le gain du banc (`BruitCouples`).
- **Ce qui ne doit jamais arriver** (vérité du robot simulé), sinon le test
  échoue :
  - carton tombé, ou lâché à plus de 3 cm de haut ;
  - écrasé : consigne de serrage au-delà de ses bornes, ou contact vu trop
    tard (main à plus de `tol_face_contact_m` dans la face) ;
  - couple vrai au-delà du plafond de l'OS (à l'erreur d'estimation du banc
    près) ou de la limite matérielle ;
  - prise confirmée les mains vides ;
  - carton perdu sans que l'OS le voie.
- **Chaque refus a une raison claire**, d'une catégorie connue.

### Ce qu'elle a trouvé, et ce qui est corrigé

- **Bras ouverts qui heurtent le carton à l'insertion** : la pré-saisie fixe
  (`roulis_presaisie`, réglée sur le carton T1) ne laisse que 27 cm entre
  les paumes. Tout carton plus large vu de face était heurté, plafonds de
  couple dépassés de 1 à 2 N·m.
  - `presaisir(objet)` ouvre juste assez : largeur vue de face plus
    `jeu_ouverture_m` de chaque côté, au plus `roulis_presaisie_max` (45 cm
    libres), les mains gardées à la même hauteur.
  - Un carton trop large même ouvert en grand est refusé d'avance
    (`distance_de_prise` : « trop large pour les mains »).
  - L'app de tri ne s'en sert que si on le lui demande
    (`ConfigFacade.presaisie_ajustee`, activé par la campagne GPU) : le tri
    T1 ne change pas.
- **Fond du carton porté** : les repères des mains le donnaient de 2 cm trop
  haut (carton de 13 cm enfoncé dans la table à la dépose) à 3 cm trop bas
  (carton de 30 cm « déjà sous le plateau », pas reposé sur sa table). Il est
  maintenant estimé aux points de contact, moins leur hauteur au-dessus du
  centre du carton à la prise ; les cibles de la descente et de la montée en
  tiennent compte. La dépose est exacte à quelques millimètres.
- **Carton resté sur la table** : un carton de 1,8 kg à peine tenu par le
  serrage de prise y restait, les mains glissant de 3,8 cm le long de ses
  faces ; l'OS confirmait la prise, pesait 0,2 N et partait avec. Pesé sous
  `poids_min_n` (1 N), il est reposé : « ne pèse pas sur les mains ».
- **Levage sans fin** : avec les couples bruités du banc, le délai qui juge
  « trop lourd » repartait à chaque avance aussitôt défaite par le
  garde-plafond. Il ne repart plus que sur un vrai progrès.
- **Robot simulé**, pour que ces cas existent :
  - masse et frottement propres à chaque carton ;
  - glissement quand le frottement ne porte plus le poids ;
  - carton porté à l'équilibre des forces des deux mains (au milieu des mains
    mesurées, un carton tourné de 10° tombait au retour) ;
  - faces bornées à leur rectangle, tables en option ;
  - carton accroché là où les mains l'ont serré.

### Ce que montre le rapport (graine 20261118)

- **Tirés dans toutes leurs plages à la fois, presque aucun carton n'est
  pris** : 0 déposé sur 120, 2 reposés, 118 refusés. Hors de portée : 90 ;
  trop large : 23 ; trop lourd ou trop glissant pour le serrage de prise :
  5. Aucun cas interdit.
- **Un paramètre à la fois autour du carton T1** (balayage du test), la
  prise réussit :
  - largeur de 15 à 30 cm ;
  - profondeur de 15 à 30 cm ;
  - hauteur de 13 à 25 cm ;
  - masse jusqu'à ~1 kg ;
  - frottement dès 0,6 ;
  - lacet jusqu'à 20° ;
  - retrait du bord de 5 cm au plus.
- **Ses limites, raisons comprises** :
  - carton de 10 cm de haut : avant-bras trop près du plateau ;
  - dès 1,2 kg : trop lourd pour les poignets du G1 (plafonds de 5 N·m) ;
  - μ = 0,5 : il ne suit pas le levage ;
  - lacet de 30° et plus : hors de portée, le robot abordant la table de face ;
  - à 20 cm du bord de la table : hors de portée à toute distance.

### Les réglages choisis sur le carton T1

| Réglage | Sur la campagne | Verdict |
|---|---|---|
| `frottement_serrage` 0,8 | Cartons à μ 0,5 et 0,6 portés avec une marge vraie de 1,38 à 1,46, l'OS en croyant 1,74 à 2,30 ; garantie au serrage requis : 2 × μ / 0,8, soit 1,25 à μ 0,5 | **Réglé pour T1** (μ ≥ 0,8). À 0,5, la marge vraie ne descend plus sous 2,04, mais les mains ne portent plus que ~0,6 kg : le carton T1 lui-même devient trop lourd. **Proposition** : mesurer le frottement caoutchouc/carton du site ; le garder à 0,8 pour les cartons T1 (μ = 1 au banc), le descendre à la mesure pour tout autre carton. Inchangé ici. |
| `rotation_ramener_max_deg` 4° | Rotation estimée au retour : 1,1° au plus sur la campagne, 3,3° sur une grille d'essai jusqu'à 1,2 kg. Un carton de 1,39 kg, aux plafonds de couple, l'a dépassée (4,7° estimés) sans tourner : retour arrêté à tort, mais au-delà de ce que les mains portent | **Tient**. Inchangé. |
| `penetration_poids_max_m` 6 cm | Jamais atteinte : avance de 3,4 cm au plus sur la campagne, 5,2 sur un balayage d'essai aux couples bruités ; ce sont les plafonds de couple qui limitent d'abord | **Tient**. Inchangé. |
| Seuils de contact (`force_contact_n`, `sigmas_contact`, `hausse_contact_nm`) | Contact vu la main 0,8 à 1,2 cm dans la face (au plus 1,5) quels que soient taille, masse et lacet, bruit du banc compris ; aucun refus de contact | **Tiennent**. Inchangés. |
| `roulis_presaisie` 0,32 | 27 cm entre les paumes : tout carton plus large était heurté | **Réglé pour T1** : corrigé, pré-saisie à la largeur du carton (voir plus haut). |
| `masse_max_kg` 1,0 | C'est la masse du carton T1 ; les mains portent jusqu'à ~1,2 kg | **Proposition** : 1,2 kg, la charge que les mains portent (tolérance de la ligne des mains au centre de gravité : 1,45 cm au lieu de 1,7). Inchangé ici. |
| `force_serrage_n` 8 N | 8 ou 12 N ne changent rien : le serrage de prise est borné par `penetration_max_m` (2 cm, ~9 N avec les bras du G1) | **Proposition** : au second essai d'un carton qui n'a pas suivi, serrer jusqu'à la borne d'après le poids. Inchangé ici. |

### Ce qui reste

- **Aborder le carton dans l'axe de ses faces** (cap de prise tourné de son
  lacet) : c'est ce qui ferait passer les lacets de 30° et plus.
- **L'app de tri suppose un seul carton** : réglage de taille unique, et
  perception qui ajuste une boîte de cette taille. La campagne GPU lui donne
  la vraie taille de chaque carton posé ; il faudrait qu'elle la perçoive.
- **Plateaux bas** : à atteindre en pliant le corps (taille, genoux), pas en
  inclinant les mains.
- **Campagne GPU, après le premier tri réussi** :
  `bash lancer_t1.sh 3600 10 --cartons-varies 40`. Bilan par plage et
  raisons en fin d'essai, détail dans `videos/t1_campagne_cartons.json`.
