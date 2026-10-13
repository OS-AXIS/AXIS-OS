#!/usr/bin/env bash
# Affiche au fil de l'eau, jusqu'à sa fin, le journal du test en cours de la
# démonstration de tri de colis (dite « T1 », lancée par lancer_t1.sh).
# Sans danger : Ctrl+C ou une déconnexion coupe l'AFFICHAGE, pas le test, qui
# tourne détaché dans le conteneur Docker d'Isaac Sim (le simulateur de
# robotique de NVIDIA).
CONTENEUR="isim-isaac-sim-1"
if ! sudo docker exec "$CONTENEUR" test -f /isaac-sim/videos/t1.log; then
  echo "Aucun test T1 lancé (pas de journal)."; exit 0
fi
# Dans le conteneur, une boucle attend en arrière-plan le fichier t1.fini
# (créé à la fin du test) ; tail suit le journal tant que cette boucle vit
# (--pid=$!), puis s'arrête de lui-même. Le grep retire le bruit d'Isaac Sim
# (lignes « ext: » des extensions, avertissements, « cudainterop »).
sudo docker exec "$CONTENEUR" bash -c '
  until [ -f /isaac-sim/videos/t1.fini ]; do sleep 2; done &
  tail -n +1 -f --pid=$! /isaac-sim/videos/t1.log' \
  | grep --line-buffered -v -e "ext:" -e "Warning" -e "cudainterop" || true
# t1.fini distingue un test terminé d'un affichage simplement interrompu.
if sudo docker exec "$CONTENEUR" test -f /isaac-sim/videos/t1.fini; then
  echo "Test terminé."
else
  echo "Affichage interrompu — le test continue. Reprendre : bash ~/AXIS-OS/deploy/machine_gpu/suivre_t1.sh"
fi
