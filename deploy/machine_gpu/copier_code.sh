#!/usr/bin/env bash
# Met à jour le code AXIS-OS dans le conteneur (après un git pull).
set -euo pipefail
CONTENEUR="isim-isaac-sim-1"
DEPOT="$HOME/AXIS-OS"
git -C "$DEPOT" pull --ff-only origin main
sudo docker exec -u root "$CONTENEUR" rm -rf /isaac-sim/axis_os
sudo docker cp "$DEPOT/axis_os" "$CONTENEUR:/isaac-sim/"
sudo docker cp "$DEPOT/axis_os/hal/drivers/unitree_g1/urdf/g1_29dof_rev_1_0_mains_collision.urdf" \
  "$CONTENEUR:/isaac-sim/"
for f in "$DEPOT"/sim/validation/test_isaac_*.py; do
  sudo docker cp "$f" "$CONTENEUR:/isaac-sim/axis_os/"
done
echo "Code copié ($(git -C "$DEPOT" log --oneline -1))."
