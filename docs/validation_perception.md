# Validation de la chaîne perception — 2026-09-22

Mesures réelles contre Isaac Sim 6.1.0, G1 23 DOF importé depuis l'URDF
officiel AXIS-OS. Reproductible via `test_isaac_p1_perception.py` et
`test_isaac_p2_detection.py`.

---

## 1. Résultat

Chaîne complète validée de bout en bout :

```
caméra Isaac Sim (ICamera)
  -> CameraFrame rgb8 horodatée
  -> OWL-ViT (vocabulaire ouvert)
  -> Detection (boîte en pixels)
  -> CameraIntrinsics + FrameTree
  -> direction dans le repère du robot
```

| | valeur |
|---|---|
| Détection principale | « a robot », confiance 0.329 |
| Boîte | (259, 40, 126, 402) — 402 px de haut sur 480 |
| Centre horizontal | 322 px (centre image : 320) |
| Direction repère robot | (1.00, −0.003, −0.001) — droit devant |
| Inférence OWL-ViT sur L4 | **172 ms** en moyenne |
| Robot posé au sol | point le plus bas à z = +0.000 m |

La latence d'inférence (172 ms) est neuf fois le budget d'un cycle de
commande (20 ms). Une politique de perception ne tourne donc jamais
dans la boucle de contrôle — c'est la justification concrète du
`SetpointFollower` et de son interpolation entre consignes lentes.

---

## 2. Trois problèmes réels, superposés

Chacun masquait le suivant. Tous étaient **silencieux** : aucune
exception, et des tests partiels au vert.

### 2.1 Meshes jamais récupérés

L'URDF référence 47 fichiers `.STL` (27 distincts), volontairement non
versionnés (106 Mo, voir `urdf/meshes/README.md`). Ils n'avaient jamais
été récupérés. Isaac Sim importait le robot **sans erreur** —
articulations, masses, limites — mais sans géométrie visuelle ni meshes
de collision.

Les avertissements existaient depuis le premier jour, masqués par
`USD Diagnostics are currently muted`.

**Correctif :** `find_missing_meshes()` vérifie avant l'import ;
`require_meshes=True` refuse d'importer sans eux.

### 2.2 Caméra tournée de 90°

Axe de visée mesuré : `(0, +1, 0)`. Attendu : vers le robot. La classe
`Camera` d'Isaac Sim convertit les orientations entre trois conventions
d'axes (world, ros, usd) ; `look_at()` en déclarait une mauvaise.

**Aucune caméra n'a jamais regardé le robot.** D'où quatre points de vue
aux statistiques identiques (ciel et sol), et zéro détection.

**Correctif :** `look_at()` auto-vérifiant — il relit l'orientation
réellement composée dans la scène et ne rend `True` que si l'axe de
visée est aligné sur la cible à 0.99 près.

### 2.3 Robot enterré de 70 cm

Boîte englobante de z = −0.73 à +0.59 : pieds sous le sol. Inoffensif
tant que les collisions manquaient ; dès qu'elles ont existé, la
physique a divergé (`PhysX error: non-finite bounds`).

Deux premières tentatives ont échoué — `position=` au Robot après
création du World, puis relèvement d'une ancre de joint monde qui
n'existait pas. **Correctif retenu :** translation écrite dans la scène
USD **avant** le démarrage de la physique, puis `_verify_grounded()`
remesure et le dit si le robot n'est pas au sol.

---

## 3. Deux de mes outils de mesure étaient faux

À documenter autant que les bugs, parce qu'ils ont coûté autant de
temps.

**« 0 mesh dans la scène »** — faux. Le parcours USD ordinaire n'entre
pas dans les instances, or les meshes étaient instanciés. En entrant
dans les instances (`TraverseInstanceProxies`) : 47 meshes. Cette
erreur a produit une fausse conclusion secondaire (« collision quasi
inexistante »).

**« 45 % de pixels non-fond »** — faux. Chaque pixel était comparé à la
médiane de l'image ; sur un simple dégradé de fond, la moitié des pixels
s'en écartent mécaniquement. Des images vides passaient pour des images
contenant un objet.

---

## 4. La méthode qui a tranché

Le **test différentiel** : rendre la même vue avec puis sans le robot,
et compter les pixels qui changent. Il ne dépend d'aucune hypothèse sur
la couleur du robot, l'éclairage ou le fond — un pixel qui change est un
pixel du robot, par construction.

Toutes les tentatives précédentes interprétaient **une** image avec des
seuils. Chacune pouvait se tromper, et deux l'ont fait.

> **Règle retenue :** quand un diagnostic interprète une mesure,
> chercher d'abord une mesure qui n'a pas besoin d'être interprétée.

---

## 4b. P3 — colis dans un entrepôt, avec distance (2026-09-22)

Deux colis placés à des positions **choisies**, donc vérifiables.

| Colis | Centre réel | Face avant | Mesuré | Hauteur |
|---|---|---|---|---|
| proche | x = 1.50 | **1.30** | **1.30** | 0.30 (réel 0.20) |
| lointain | x = 3.00 | **2.85** | **2.85** | 0.19 (réel 0.15) |

**La mesure tombe au centimètre sur la FACE AVANT**, pas sur le centre —
ce qui est physiquement correct : une caméra voit la surface visible. Et
c'est la bonne grandeur pour un robot qui doit s'arrêter *devant* un
colis.

OWL-ViT reconnaît des cartons **sans texture** (« a cardboard box »,
0.31 et 0.23) : les modèles d'entrepôt texturés de NVIDIA ne sont pas
nécessaires pour avancer.

**Erreur corrigée en chemin, et elle vaut d'être retenue.** La pose de la
caméra était déclarée dans l'arbre des repères avec sa seule
*translation*, sans orientation. La caméra étant inclinée de 28° vers le
bas, tout était projeté à l'horizontale : un colis posé à 0.20 m était
situé à 0.99 m — exactement la hauteur de la caméra — alors que x et y
restaient justes à 2 cm près.

P1 et P2 ne pouvaient pas la détecter : leurs caméras étaient quasi
horizontales. Il a fallu une caméra franchement inclinée **et** des
cibles à des positions connues.

> Vérifier qu'« il y a une détection » ne prouve presque rien. Placer les
> cibles à des positions choisies transforme un test de présence en test
> de justesse.

Sur le robot réel, cette pose devra venir de la **cinématique** — point
de montage, orientation de la tête —, jamais d'une valeur supposée.

## 5. Limites connues

- **« a person »** : avec les requêtes par défaut, OWL-ViT a classé le
  G1 comme une personne. Prudent pour l'évitement, mais un G1 qui en
  croise un autre croira voir quelqu'un. **Les étiquettes d'un modèle ne
  fondent jamais une décision de sécurité** — l'évitement relève de la
  géométrie (Nav2), pas de la reconnaissance.
- **Doublons** : deux boîtes pour un seul robot (la seconde à 0.106, haut
  du corps seul). Aucune fusion des détections qui se chevauchent.
- **Rendu en 320×240** : DLSS divise la résolution demandée par deux.
  Sans effet sur la plomberie ; à régler avant de juger la qualité de
  détection.
- **Poids OWL-ViT épinglés** depuis l'audit du 2026-10-10 :
  `OWLVIT_REVISION` (commit 85fa9bec… de google/owlvit-base-patch32,
  poids safetensors) ; le détecteur refuse de charger sans révision, sauf
  autorisation explicite en simulation. À revalider en simulation à
  chaque changement de révision.
- **Simulation ≠ réel** : textures propres, éclairage parfait, aucun
  bruit de capteur. Ce test valide la chaîne, pas la reconnaissance sur
  images réelles.

---

## 6. Non-régression

Le pont a changé en profondeur (robot relevé de 0.80 m, collisions
complètes pour la première fois).

**Latence `SetpointFollower` : aucune régression.** 5.39 ms de moyenne
(contre 5.41 ms avant), fraîcheur OK, dérive après péremption
0.0027 rad — identique.

**S2 (conventions de signe).** *Correction :* avant ces changements, S2
n'était PAS à 23/23 mais à **21/23** (`left_hip_pitch`, `left_hip_yaw`).
Une version antérieure de ce document l'affirmait à tort.

Après les changements : toujours 21/23, mais **d'autres articulations**.
Les deux défaillances de la hanche ont disparu ; les deux chevilles
(`left_ankle_pitch`, `right_ankle_pitch`) échouent en mode position.
Signature de contact avec le sol, pas de bug de signe :
- comportement **identique** à gauche et à droite (−0.063 → −0.059 et
  −0.065 → −0.061), alors que l'une est inversée et l'autre non — un
  bug de convention se comporterait différemment ;
- départ à −0.063 juste après remise à zéro : poussée extérieure ;
- seules échouent les articulations dont la rotation enfonce le pied.

Robot posé, collisions réelles : le plancher bloque la cheville.
Correctif : S2 tourne désormais **robot suspendu** (`ground_clearance`),
même principe que le protocole de mise sous tension réel.

**Résultat : 23/23 cohérents. S2 validée — pour la première fois.**
Robot suspendu à 0.147 m, pieds dans le vide. Les conventions de signe
sont correctes sur les 23 articulations, y compris les 9 inversées, en
mode position ET en mode couple.

Aucun des échecs antérieurs de S2 n'était un bug de convention :
successivement le couple jamais appliqué par le pont, les gains PD
absents, la libération globale des joints, la cible PD non réalignée,
le robot enterré sans collision, puis le plancher bloquant les
chevilles. Tous des problèmes du **banc de test**, jamais de
`conventions.py`.
