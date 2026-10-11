# Validation de l'inventaire et de la mémoire — 2026-09-23

Mesures réelles contre Isaac Sim 6.1.0, G1 23 DOF, app sandboxée via le
Play Store. Reproductible via `sim/validation/test_isaac_a3_inventaire.py`.

---

## 1. Résultat

**Le robot recense, mémorise, reconnaît, et remarque une absence.**

Deux colis placés à 90° l'un de l'autre. Le robot balaie en tournant sur
place. Entre les deux missions, **un colis est retiré de sa vue**.

| | mission 1 | mission 2 |
|---|---|---|
| Colis recensés | 2 | 1 |
| Nouveaux | 2 | **0** |
| Déjà connus | 0 | **1** |
| Disparus | 0 | **1** |
| Connus avant | 0 | 2 |

Zéro consigne refusée, aucune chute, sur 1667 pas de locomotion.

La mémoire est écrite sur disque et **cloisonnée par app** : une autre
app, avec la même clé, ne lit rien et n'écrase rien (vérifié par test).

---

## 2. Un inventaire ne peut pas être tenu dans le repère du robot

Premier essai : **un seul colis recensé sur deux**, puis aucun.

Les positions étaient exprimées dans le repère du ROBOT — x devant lui,
y à sa gauche. Or l'app balaie **en tournant sur place**. Face au colis
de devant, celui-ci est à (2.2, 0). Après 90° de rotation, le colis de
gauche est **aussi** à (2.2, 0). Deux objets distincts occupaient les
mêmes coordonnées à des instants différents : l'algorithme de fusion les
a confondus en un seul.

**Distinction désormais explicite dans le SDK :**

| | usage |
|---|---|
| `look_for(query)` | repère du ROBOT — pour se *diriger* vers un objet |
| `look_for(query, frame="world")` | repère FIXE — pour *mémoriser* une position |

Un développeur tiers qui mémorise dans le mauvais repère obtient un
inventaire absurde **sans aucun message d'erreur**. C'est exactement ce
qui s'est produit ici.

---

## 3. Deux décisions métier, au cœur de l'app

Ni la perception ni un modèle ne peuvent les prendre.

**`RAYON_FUSION_M = 0.8`** — à quelle distance deux observations
désignent le même colis. Plus large qu'un colis de manutention (0.4 à
0.6 m), assez pour absorber l'erreur de mesure (12 cm à 1 m), assez
serré pour distinguer deux colis côte à côte. *La bonne valeur dépend de
la taille des objets manipulés.*

**`VUES_MINIMUM = 2`** — combien d'observations avant d'inscrire un
colis. Une détection isolée peut être un reflet ou le sol (mesuré à 0.129
de confiance). Deux observations coûtent une seconde et éliminent
l'essentiel des faux positifs.

La position est **moyennée** à chaque observation, jamais remplacée :
une mesure aberrante déplace un colis vu douze fois de 5 cm seulement.

Et l'inventaire mémorisé est l'**état courant**, pas l'accumulation. Un
inventaire qui ne sait pas oublier décrit l'histoire de l'entrepôt, pas
l'entrepôt.

---

## 4. Les capacités logicielles viennent des services

L'app n'était **jamais installée** : elle exige
`Capability.MEMORY_PERSISTENT`, et la table de correspondance ne
traduisait que des capacités **matérielles** (caméra, micro, jambes).

Cascade silencieuse : installation refusée → mission 1 sans résultat →
rien sur disque → « app non installée » à la mission 2, seul message
visible, qui n'expliquait rien.

**Deuxième fois en deux jours** que cette table est incomplète (après
`SPEECH_IN`). À revoir globalement quand la navigation arrivera.

> Quand le Play Store refuse une app, le message doit dire **quelle
> capacité manque** — et pas seulement à l'installation, mais partout où
> la conséquence se manifeste.

---

## 5. Limites

- **Le robot ne se déplace pas** pendant l'inventaire : il balaie sur
  place. Couvrir une allée entière suppose une navigation, donc une
  odométrie fiable — la dérive d'un bipède est bien plus forte que celle
  d'un robot à roues.
- **Position 2D** : la hauteur n'est pas utilisée pour distinguer deux
  colis empilés.
- **Pas d'identité** : deux colis identiques déplacés l'un vers l'autre
  seraient vus comme « un disparu, un nouveau ». Distinguer un
  déplacement d'une substitution demanderait un code-barres ou une
  étiquette.
