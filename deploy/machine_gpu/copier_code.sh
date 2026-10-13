#!/usr/bin/env bash
# Met à jour le code AXIS-OS dans le conteneur Docker (environnement isolé)
# d'Isaac Sim, le simulateur de NVIDIA : récupère la dernière version de la
# branche main du dépôt (git pull), puis la copie dans le conteneur. Appelé
# par installer.sh (étape 9) ; à relancer chaque fois que le code a changé
# sur main.
set -euo pipefail
CONTENEUR="isim-isaac-sim-1"
DEPOT="$HOME/AXIS-OS"
git -C "$DEPOT" pull --ff-only origin main
# L'ancienne copie est effacée d'abord : docker cp ajoute et remplace des
# fichiers mais n'en supprime aucun, si bien qu'un fichier retiré du dépôt
# resterait dans le conteneur.
sudo docker exec -u root "$CONTENEUR" rm -rf /isaac-sim/axis_os
sudo docker cp "$DEPOT/axis_os" "$CONTENEUR:/isaac-sim/"
# URDF (fichier XML standard qui décrit le robot) du G1 où les mains ont une
# forme de collision : dans l'URDF officiel, elles n'en ont pas, et elles
# traversaient le carton en simulation. Le simulateur l'attend à la racine
# de /isaac-sim (voir axis_os/hal/drivers/isaac_sim/bridge.py et
# axis_os/hal/drivers/unitree_g1/urdf/deriver_mains_collision.py).
sudo docker cp "$DEPOT/axis_os/hal/drivers/unitree_g1/urdf/g1_29dof_rev_1_0_mains_collision.urdf" \
  "$CONTENEUR:/isaac-sim/"
# Les scripts d'essai Isaac Sim (sim/validation/test_isaac_*.py) sont copiés
# dans /isaac-sim/axis_os/, d'où les lanceurs les exécutent (par exemple
# lancer_t1.sh).
for f in "$DEPOT"/sim/validation/test_isaac_*.py; do
  sudo docker cp "$f" "$CONTENEUR:/isaac-sim/axis_os/"
done
echo "Code copié ($(git -C "$DEPOT" log --oneline -1))."
