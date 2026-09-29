#!/usr/bin/env bash
# Affiche le journal du test T1 en cours, au fil de l'eau, jusqu'à sa fin.
# Sans danger : Ctrl+C ou une déconnexion coupe l'AFFICHAGE, pas le test.
CONTENEUR="isim-isaac-sim-1"
if ! sudo docker exec "$CONTENEUR" test -f /isaac-sim/videos/t1.log; then
  echo "Aucun test T1 lancé (pas de journal)."; exit 0
fi
sudo docker exec "$CONTENEUR" bash -c '
  until [ -f /isaac-sim/videos/t1.fini ]; do sleep 2; done &
  tail -n +1 -f --pid=$! /isaac-sim/videos/t1.log' \
  | grep --line-buffered -v -e "ext:" -e "Warning" -e "cudainterop" || true
if sudo docker exec "$CONTENEUR" test -f /isaac-sim/videos/t1.fini; then
  echo "Test terminé."
else
  echo "Affichage interrompu — le test continue. Reprendre : bash ~/AXIS-OS/deploy/machine_gpu/suivre_t1.sh"
fi
