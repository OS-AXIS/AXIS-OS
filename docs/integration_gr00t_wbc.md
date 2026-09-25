# Intégration de GR00T Whole-Body Control — analyse préalable

2026-09-24. Analyse **par lecture de code**, sans exécution : aucun GPU
disponible. Tout ce qui est marqué *à vérifier* doit l'être en simulation
avant d'écrire la moindre intégration.

Source : `NVlabs/GR00T-WholeBodyControl` @ b042411, dossier
`decoupled_wbc/`. Référence retenue : le **code de déploiement réel**
(`control/policy/g1_gear_wbc_policy.py`), pas le script de démonstration
MuJoCo — voir §4.

---

## 1. Pourquoi cette politique, et pourquoi le G1 29 DOF

La politique d'Unitree (`motion.pt`) a été entraînée bras immobiles et
sans charge. Porter un carton déplace le centre de gravité vers l'avant :
rien ne garantit qu'elle sache compenser.

La politique GR00T WBC est **découplée** : elle ne commande que les
jambes et la taille (15 articulations), mais elle **observe les 29
articulations du corps, bras compris**. Elle sait donc où sont les bras
et peut compenser une charge tenue devant le corps. Les bras sont
commandés par l'appelant (`arms_target_pose`).

Elle est construite pour le **G1 29 DOF** : trois articulations de taille
(le 23 DOF n'en a qu'une) et sept par bras (cinq sur le 23 DOF). La
brancher sur le 23 DOF reviendrait à lui demander de s'équilibrer avec
des articulations qui n'existent pas. **Décision : le G1 29 DOF devient
le robot de référence en simulation.**

---

## 2. Ce que la politique attend

| | |
|---|---|
| Commande | 50 Hz (physique 200 Hz, décimation 4) |
| Actions | **15** : 12 jambes + 3 taille |
| Cible | `action × 0.25 + pose_par_défaut` |
| Pose par défaut (jambes) | `[-0.1, 0, 0, 0.3, -0.2, 0] × 2`, taille `[0, 0, 0]` |
| Gains kp | jambes `[150,150,150,200,40,40]`, taille `250` |
| Gains kd | jambes `[2,2,2,4,2,2]`, taille `5` |
| Historique | **6 observations** empilées (516 = 86 × 6) |

**Observation unitaire (86 valeurs)** :

| indices | contenu | échelle |
|---|---|---|
| 0–2 | vitesse commandée (vx, vy, wz) | `cmd_scale = [2, 2, 0.5]` |
| 3 | **hauteur** du bassin commandée | brute (défaut 0.74 m) |
| 4–6 | **orientation du torse** commandée (roulis, tangage, lacet) | brute |
| 7–9 | vitesse angulaire du bassin | × 0.5 |
| 10–12 | gravité projetée dans le repère du bassin | — |
| 13–41 | positions des **29** articulations, moins la pose par défaut | × 1.0 |
| 42–70 | vitesses des 29 articulations | × 0.05 |
| 71–85 | action précédente (15) | brute |

Même famille que la politique d'Unitree : même pose accroupie, mêmes
échelles, même 50 Hz. On ne repart pas de zéro.

**Deux politiques** : `Balance` quand la vitesse commandée est inférieure à
0.05, `Walk` sinon. La politique d'équilibre fournit enfin un **vrai mode
debout** — la politique d'Unitree piétinait en permanence, même à
consigne nulle.

**Hauteur et orientation du torse commandables** : le robot peut
s'abaisser et se pencher — indispensable pour saisir un carton.

---

## 3. Ordre des 29 articulations (ordre moteur Unitree)

`0–5` jambe gauche (hanche tangage, roulis, lacet, genou, cheville
tangage, roulis) · `6–11` jambe droite · `12–14` taille (lacet, roulis,
tangage) · `15–21` bras gauche (épaule tangage, roulis, lacet, coude,
poignet roulis, tangage, lacet) · `22–28` bras droit.

Les doigts des mains Dex3 ne font **pas** partie de l'observation.

---

## 4. Piège évité : le script de démonstration est incohérent

`sim2mujoco/scripts/run_mujoco_gear_wbc.py` calcule
`n_joints = qpos.shape[0] - 7` sur un modèle qui contient **43**
articulations (les 14 des doigts Dex3 comprises), puis les range dans une
observation de 86 valeurs prévue pour 29. Pris naïvement, il ferait
observer jambes, taille, bras gauche… puis **les doigts gauches**, sans le
bras droit.

Le code de **déploiement** sélectionne explicitement le groupe `body`
(29 articulations). C'est lui qui fait foi.

Au passage : leur modèle de robot utilise des **groupes d'articulations**
(`body`, `lower_body`) — la même abstraction que `JointGroup` dans
AXIS-OS.

---

## 5. Ce qui reste à vérifier, dans l'ordre

1. **Repère de la vitesse angulaire** : le code lit
   `floating_base_vel[3:6]`. Dans MuJoCo, pour une base libre, c'est le
   repère du **corps** — compatible avec `BaseState.ang_vel_body`. À
   confirmer par la même expérience témoin que pour la politique
   d'Unitree (docs/validation_locomotion.md §5).
2. **Conventions de signe du G1 29 DOF** : refaire la suite S2 sur le
   nouvel URDF. Sur le 23 DOF, 9 articulations sur 23 étaient inversées.
3. **Poids des politiques** : fichiers ONNX (~1.9 Mo) stockés en Git LFS,
   à récupérer sur la machine GPU. **Licence : NVIDIA Open Model
   License**, différente de la BSD d'Unitree — à lire avant tout usage
   commercial.
4. **L'historique de 6 observations** : il doit être rempli correctement au
   démarrage et **réinitialisé** à chaque reprise, sinon la politique
   reçoit un passé qui ne correspond à rien.

---

## 6. Ce qu'il faut construire dans AXIS-OS

- Driver **G1 29 DOF** : URDF (unitree_ros), définitions, groupes
  d'articulations, table de conventions.
- **`GR00TWBCAdapter(ILocomotionAdapter)`** : historique, deux politiques,
  consignes de hauteur et d'orientation. Le service de locomotion ne
  change pas.
- **Arbitrage des actionneurs entre services** : la marche possède jambes
  et taille, l'app possède les bras. Aujourd'hui le service de marche
  commande toutes les articulations. C'est la vraie pièce d'OS de ce
  chantier.
- Puis, seulement : saisie, port, dépose.

**Premier test, avant tout le reste** : faire marcher le G1 29 DOF avec
cette politique, bras en position de port, sans colis. Si le robot tombe,
on le saura en une heure.

---

## 7. Mise en route sur la machine GPU

Tout ce qui ne demande pas de simulateur est prêt et testé : driver
29 DOF (`UnitreeG1HAL(variante="29dof")`), adaptateur
(`GR00TWBCAdapter`), pont Isaac Sim paramétrable
(`IsaacSimBridge(variante="29dof")`). Premier test :
`test_isaac_w1_gr00t_port.py`.

Dans l'ordre, une fois Isaac Sim 6.1 lancé dans Docker :

```bash
# 1. Poids des politiques (Git LFS — absents d'un clone ordinaire)
git clone https://github.com/NVlabs/GR00T-WholeBodyControl.git ~/gr00t_wbc
cd ~/gr00t_wbc && git lfs install && \
  git lfs pull --include "decoupled_wbc/sim2mujoco/resources/robots/g1/policy/*.onnx"
docker exec -u root isim-isaac-sim-1 mkdir -p /isaac-sim/policies
docker cp decoupled_wbc/sim2mujoco/resources/robots/g1/policy/GR00T-WholeBodyControl-Balance.onnx \
  isim-isaac-sim-1:/isaac-sim/policies/gr00t_wbc_balance.onnx
docker cp decoupled_wbc/sim2mujoco/resources/robots/g1/policy/GR00T-WholeBodyControl-Walk.onnx \
  isim-isaac-sim-1:/isaac-sim/policies/gr00t_wbc_walk.onnx

# 2. Moteur d'inférence ONNX dans le Python d'Isaac Sim
docker exec -u root isim-isaac-sim-1 /isaac-sim/python.sh -m pip install onnxruntime-gpu

# 3. URDF 29 DOF et meshes (le pont refuse de démarrer s'il en manque)
docker cp ~/AXIS-OS/axis_os/hal/drivers/unitree_g1/urdf/g1_29dof_rev_1_0.urdf \
  isim-isaac-sim-1:/isaac-sim/
#    meshes : dépôt unitree_ros, robots/g1_description/meshes -> /isaac-sim/meshes

# 4. Code et test
docker exec -u root isim-isaac-sim-1 rm -rf /isaac-sim/axis_os
docker cp ~/AXIS-OS/axis_os isim-isaac-sim-1:/isaac-sim/
docker cp ~/AXIS-OS/test_isaac_w1_gr00t_port.py isim-isaac-sim-1:/isaac-sim/axis_os/
docker exec isim-isaac-sim-1 /isaac-sim/python.sh /isaac-sim/axis_os/test_isaac_w1_gr00t_port.py
```

Au premier chargement, la politique affiche l'**empreinte SHA-256** de
chaque fichier ONNX : la relever et l'épingler dans `GR00TWBCPolicy`,
comme pour la politique d'Unitree.
