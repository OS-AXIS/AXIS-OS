# AXIS-OS — Campagne de validation en simulation

Écrit à partir du code au commit `e0c02cb` et de
`docs/isaac_lab_unitree_sim_status.md`.

**Point de départ acquis :** `JointSafetyGuard` validé contre un G1 physiquement
simulé, 23/23 joints, 23/23 commandes dangereuses rejetées. C'est une vraie base
— la suite l'étend, elle ne la refait pas.

**Ce que cette campagne prouve :** que ta couche logicielle est correcte.
**Ce qu'elle ne prouve pas :** que l'OS peut piloter un robot réel. Voir §9.

---

## 0. Isaac Sim ou Isaac Lab ?

Tu as passé du temps bloqué sur Isaac Lab. Voilà la règle, pour ne pas y
retourner sans raison :

| | Sert à | Tu en as besoin |
|---|---|---|
| **Isaac Sim** | Simuler un robot et sa physique. Valider ton OS contre. | Maintenant, pour tout §1 à §6 |
| **Isaac Lab** | Entraîner et évaluer des politiques RL, en milliers d'environnements parallèles | Plus tard : politique de marche (étape 6), et §7 |

Pour valider AXIS-OS, **Isaac Sim seul suffit** — c'est exactement le chemin qui
a marché le 18/09. Isaac Lab ne devient utile que pour deux choses : évaluer la
politique de marche avant qu'elle touche le vrai robot, et les balayages
randomisés du §7. Ne rouvre pas ce chantier avant.

---

## 1. Prérequis — le pont HAL ↔ Isaac Sim

Aujourd'hui le pont est un intercepteur de fonction écrit pour une session. Tout
le reste en dépend, donc c'est la première chose à faire.

**À écrire :** `axis_os/hal/drivers/isaac_sim/bridge.py` — un composant
réutilisable qui relie `UnitreeG1HAL.send_joint_commands()` à
`robot.apply_action()`, et qui renvoie les états articulaires d'Isaac comme
`ActuatorFeedback`.

Trois exigences :

Il doit fermer la boucle. Un pont qui n'envoie que des commandes ne teste que la
moitié du système — le watchdog, `position_delta_max` et `check_feedback_health`
ont tous besoin du retour physique réel.

Il doit exposer **le couple réellement appliqué** par le solveur physique
(`applied_joint_efforts` de l'articulation Isaac), pas le couple commandé. C'est
la seule façon de valider la suite S2, et c'est impossible en test unitaire.

Il doit gérer le réimport propre du USD. Ton blocage `is_homogeneous` vient d'un
`.usd` déjà présent sur disque : supprime-le systématiquement avant import, dans
le code du pont, pas à la main.

*Validé quand :* les trois apps de référence tournent de bout en bout contre le
G1 simulé, sans intervention manuelle entre deux exécutions.

---

## 2. Suite S1 — Sécurité sous physique réelle

Étend ce que tu as déjà fait.

**S1.1 — Régression C1 : position hors limites dans les 5 modes.** Pour chacun
des 23 joints, envoie une position aberrante avec `mode` valant successivement
`POSITION`, `VELOCITY`, `TORQUE`, `CURRENT`, `IMPEDANCE`. Les 115 commandes
doivent être rejetées. C'est le bug qui a failli passer — teste-le sous physique,
pas seulement en unitaire.
*Critère :* 115/115 rejetées, et le joint ne bouge pas dans Isaac.

**S1.2 — Couple réellement appliqué.** Pour chaque joint, une consigne de
position légale mais éloignée, et tu relèves le couple appliqué par le solveur.
*Critère :* jamais au-dessus de l'`effort` de l'URDF, sur les 23 joints. C'est le
test que `position_delta_max` est censé garantir — il n'a jamais été vérifié
contre une vraie physique.

**S1.3 — Pas de faux positifs.** Tu as testé une commande sûre par joint. Il en
faut plus : balaye toute la plage légale de chaque joint par pas de 0,05 rad,
via `move_to_pose`.
*Critère :* zéro rejet. Un garde-fou qui refuse des commandes valides rendra ton
robot inutilisable, et c'est le genre de défaut qu'on ne découvre qu'en balayant.

**S1.4 — `position_delta_max`.** Saut de consigne au-delà de la limite → rejeté.
Le même mouvement via `move_to_pose(duration_s=2.0)` → accepté et exécuté.
*Critère :* les deux, et la position finale atteinte à moins de 0,02 rad.

---

## 3. Suite S2 — Normalisation contre la physique

**C'est la suite la plus importante du document.** C'est elle qui aurait attrapé
le bug de signe du couple sans lire une ligne de code, et c'est le type de bug
qui casse un réducteur à la première mise sous tension.

**S2.1 — Direction en mode position.** Pour chacun des 23 joints, commande
+0,2 rad en coordonnées normalisées et vérifie dans Isaac que l'articulation
bouge dans la direction anatomiquement attendue. Les 9 joints à `sign=-1.0` sont
les candidats au piège.
*Critère :* 23/23 dans le bon sens.

**S2.2 — Direction en mode couple.** Le même test avec `ControlMode.TORQUE`,
+10 Nm. C'est exactement ici que le bug était.
*Critère :* 23/23, et cohérent avec S2.1 — même signe d'entrée, même direction
physique.

**S2.3 — Aller-retour complet.** Commande normalisée → physique Isaac → feedback
brut → renormalisé. Sur les 23 joints et les 5 modes.
*Critère :* l'écart reste sous la tolérance de la physique. Toute asymétrie
systématique est un bug de conversion, pas du bruit.

**S2.4 — Cohérence posturale.** Commande une posture symétrique (les deux bras
identiques en coordonnées normalisées).
*Critère :* le robot est visuellement symétrique dans Isaac. Un test qu'un humain
lit en une seconde et qui attrape les erreurs de signe restantes.

---

## 4. Suite S3 — Détection de fautes, avec latences mesurées

Le livrable de cette suite n'est pas une liste de ✓ — c'est un **tableau de
latences**. C'est ce chiffre qui te dit si ta couche de sécurité est assez rapide
pour 35 kg en mouvement.

| Faute injectée | Détecteur attendu | Latence à mesurer |
|---|---|---|
| Feedback gelé | `COMM_TIMEOUT` | cycles jusqu'à l'alerte |
| Joint bloqué par un obstacle rigide | `TORQUE_OVERLOAD` | ms jusqu'à l'alerte |
| Température injectée en hausse | échelle d'escalade | ms par palier |
| Thread de contrôle tué | `Supervisor` | ms jusqu'à `emergency_stop_all` |
| Process entier tué (`kill -9`) | moniteur indépendant | ms jusqu'à la commande d'amortissement |
| Feedback `NaN` | `INVALID_VALUE` | cycles |

**S3.1 à S3.6** — une par ligne. Pour chacune, mesure aussi **la distance
parcourue par le joint pendant la latence de détection**. C'est le chiffre qui
compte vraiment : 80 ms de détection sur un bras à 2 rad/s, ça fait 9° de
mouvement non contrôlé.

*Critère :* chaque faute détectée à 100 %, aucun faux positif sur 1000 cycles
normaux, et le tableau de latences documenté dans `docs/`.

---

## 5. Suite S4 — Comportement physique de l'arrêt d'urgence

Suite courte et critique. Elle répond à une question que seule la physique peut
trancher.

**S4.1 — E-stop en plein mouvement rapide.** Bras à vitesse maximale, e-stop
déclenché. Mesure le dépassement après coupure.
*Critère :* le joint ne franchit aucune butée. Si c'est le cas, c'est ta limite
de vitesse qu'il faut baisser, pas l'e-stop qu'il faut accélérer.

**S4.2 — E-stop debout.** Le robot en appui sur ses deux jambes, e-stop.
*Critère : aucun — c'est une expérience, pas un test.* `kp=0, kd=0` sur un bipède
debout, c'est un effondrement. Regarde ce qui se passe, puis décide si tu as
besoin de deux arrêts distincts : un arrêt *mou* (amortissement, posture
maintenue) pour les fautes logicielles, et un arrêt *dur* (couple nul) pour les
fautes dangereuses. Beaucoup de systèmes ont les deux. Cette expérience te dira
si tu en as besoin — et il vaut mieux la faire en simulation.

**S4.3 — E-stop pendant une mission.** Vérifie que l'app reçoit bien
`EstopActiveError`, que le verrou est relâché, et que le robot reste bloqué
ensuite.

---

## 6. Suite S5 — Pile complète et endurance

**S5.1 — Les trois apps de bout en bout.** `package_sorting`, `security_patrol`,
`warehouse_ops`, chacune du Play Store jusqu'à la physique, sandbox inclus.
*Critère :* mission réussie, log chaîné vérifiable, aucune violation de sécurité.

**S5.2 — Latence du pont sous charge.** Le sandbox fait passer chaque commande
par deux `Queue`. Mesure le temps app → moteur à 50 Hz.
*Critère :* pas de dérive cumulative. Si la file s'accumule, ton architecture de
sandbox ne tiendra pas la boucle de contrôle et il faut le savoir maintenant.

**S5.3 — Endurance 8 heures.** Une mission en boucle, toute la nuit.
*Critère :* mémoire stable (pas de fuite), chaîne de hash intacte, zéro
déclenchement intempestif du watchdog, distribution de la gigue du cycle
documentée.

---

## 7. Suite S6 — Robustesse randomisée (Isaac Lab)

Ici seulement Isaac Lab devient utile : des centaines d'environnements en
parallèle avec des paramètres tirés au hasard.

Randomise la masse des segments (±20 %), le frottement, la latence injectée
(0-50 ms), le bruit capteur, et le délai d'un joint.
*Critère :* les limites de sécurité tiennent sur toute la distribution, pas
seulement au point nominal. Le vrai robot n'est jamais au point nominal.

À faire **après** S1-S5, et seulement si tu as le temps. C'est du confort, pas un
bloquant.

---

## 8. Ordre d'exécution

1. Le pont (§1) — sans lui, rien
2. S2 — normalisation, parce qu'un bug de signe invalide tout ce qui suit
3. S1 — sécurité étendue
4. S3 — latences de détection
5. S4 — comportement de l'e-stop
6. S5 — pile complète et endurance
7. S6 — si le temps le permet

---

## 9. Ce qu'aucune simulation ne validera jamais

Lis cette section deux fois. C'est celle qui compte pour ta sécurité et pour ton
matériel.

Isaac Sim valide **ta logique**. Il ne valide rien de ce qui suit, et c'est
exactement là que se trouvent les pannes de première mise sous tension :

**La correspondance des index moteur.** Ton `LowCmd_` écrit dans `motor_cmd[12]`.
Est-ce le genou droit sur le vrai G1 ? Isaac Sim ne le sait pas : il passe tes
tests avec n'importe quelle correspondance, du moment qu'elle est cohérente avec
elle-même. Le vrai robot, lui, se moque de ta cohérence interne. **Une
permutation d'index, c'est une commande de hanche envoyée à une cheville.**

**Le CRC, le `mode_machine`, la sortie du mode sport.** Zéro équivalent simulé.
Si `_ensure_low_level_mode()` échoue silencieusement, le mode sport intégré
commande les moteurs en même temps que toi.

**Le temps réel.** Isaac Sim ne tourne pas à la vitesse du monde. Ta gigue
mesurée en simulation n'a aucun rapport avec celle d'un Jetson sous charge, avec
un Linux non temps-réel et une pile DDS sur Ethernet.

**Le zéro des encodeurs.** En simulation, le robot démarre à la position exacte
que tu lui donnes. Sur le vrai, les encodeurs ont un décalage de calibration. Un
offset d'un demi-tour sur un joint, c'est une commande de 3 rad d'écart au
premier cycle.

**Le thermique, la batterie, le jeu mécanique, le bruit et les pertes de
paquets.** Rien de tout ça n'existe dans ta simulation.

**Et le fait que les commandes partent vraiment.** Le bug `publish()` jamais
appelé aurait passé toute cette campagne si le pont Isaac n'utilise pas le même
chemin de code que `G1DDSActuator`.

### Le protocole de première mise sous tension

À faire quand toute la campagne est verte, et pas avant :

Robot **suspendu**, pieds ne touchant pas le sol, arrêt d'urgence physique dans
la main d'une deuxième personne. Jamais seul.

Puis, dans cet ordre, en vérifiant visuellement à chaque étape : lire les états
articulaires sans rien commander pendant cinq minutes ; vérifier que la
désactivation du mode sport est réellement confirmée ; commander **un seul
joint**, un poignet, de ±0,1 rad avec un `kp` très bas, et vérifier de tes yeux
que c'est bien ce joint-là qui bouge et dans le bon sens ; répéter pour les 23,
un par un, en notant la correspondance réelle ; tester l'arrêt d'urgence en
mouvement ; tuer le process et vérifier que le moniteur indépendant amortit
bien ; seulement ensuite, des mouvements à plusieurs joints.

Les pieds ne touchent le sol qu'après ça, dans un espace dégagé.

### Le critère honnête

Quand toute la campagne est verte, la phrase juste n'est pas « l'OS peut tourner
sur un robot réel ». C'est :

> **La couche logicielle d'AXIS-OS est correcte. Tout problème observé à la
> première mise sous tension vient du matériel, de l'intégration DDS ou de la
> calibration — pas de la logique de sécurité.**

C'est énorme. C'est ce qui te permet de déboguer une première mise sous tension
au lieu de la subir. Mais c'est une autre phrase que celle que tu voulais
écrire, et la différence entre les deux peut coûter un réducteur, un robot, ou
une main.
