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
