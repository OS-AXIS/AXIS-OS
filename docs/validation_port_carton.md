# Validation — prendre un carton, le porter, l'emporter (2026-09-25)

G1 29 DOF, politique GR00T Whole-Body Control, Isaac Sim 6.1, sur une
instance L4. Reproductible via `test_isaac_w3_gr00t_table.py`.

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
