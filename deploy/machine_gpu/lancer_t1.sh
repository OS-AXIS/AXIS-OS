#!/usr/bin/env bash
# Démonstration T1 — tri de colis autonome avec des personnes.
#   bash lancer_t1.sh [durée_simulée_s] [accélération_vidéo] [options du test]
#   Premier essai :  bash lancer_t1.sh 240 1       (~4 min simulées, vidéo temps réel)
#   Démo complète :  bash lancer_t1.sh 1800 10     (30 min -> vidéo de 3 min)
#
# SUIVI EN DIRECT : depuis TON ordinateur, ouvre un tunnel avec la commande
# SSH que donne Vast.ai en ajoutant « -L 8080:localhost:8080 », par exemple
#     ssh -p 12345 root@1.2.3.4 -L 8080:localhost:8080
# puis ouvre http://localhost:8080 : image en direct, journal, bouton
# « Arrêter proprement » (la vidéo et le bilan sont quand même écrits).
set -euo pipefail
CONTENEUR="isim-isaac-sim-1"
DUREE="${1:-240}"; ACC="${2:-1}"; shift $(( $# > 2 ? 2 : $# )) || true
# Un Ctrl+C sur « docker exec » coupe l'AFFICHAGE, pas la simulation dans
# le conteneur : d'anciens tests continuaient de tourner, se partageaient
# le GPU et occupaient le port du suivi en direct. On les arrête d'abord.
if sudo docker exec "$CONTENEUR" pgrep -f test_isaac_ >/dev/null 2>&1; then
  echo "Arrêt des anciens tests encore en cours dans le conteneur…"
  sudo docker exec "$CONTENEUR" pkill -f test_isaac_ || true
  sleep 5
fi
bash "$HOME/AXIS-OS/deploy/machine_gpu/copier_code.sh"
mkdir -p "$HOME/videos"
# Le test tourne DÉTACHÉ dans le conteneur : une connexion SSH qui se coupe
# (onglet fermé, ordinateur en veille) n'arrête plus rien. Constaté : essai
# de 30 min perdu sur « connexion terminée » — journal et vidéo jamais
# récupérés. Ici, on ne fait que SUIVRE le journal ; si le suivi se coupe,
#   bash ~/AXIS-OS/deploy/machine_gpu/suivre_t1.sh      reprend l'affichage
#   bash ~/AXIS-OS/deploy/machine_gpu/recuperer_t1.sh   récupère vidéo et journal
sudo docker exec "$CONTENEUR" rm -f /isaac-sim/videos/t1.log /isaac-sim/videos/t1.fini
ARGS="--duree $DUREE --acceleration $ACC $*"
sudo docker exec -d "$CONTENEUR" bash -c \
  "mkdir -p /isaac-sim/videos && /isaac-sim/python.sh -u /isaac-sim/axis_os/test_isaac_t1_tri_usine.py $ARGS \
   > /isaac-sim/videos/t1.log 2>&1; touch /isaac-sim/videos/t1.fini"
echo "Test lancé en arrière-plan (il continue même si tu te déconnectes)."
echo "Suivi en direct : http://localhost:8080 (via le tunnel SSH)"
sleep 3
bash "$HOME/AXIS-OS/deploy/machine_gpu/suivre_t1.sh"
bash "$HOME/AXIS-OS/deploy/machine_gpu/recuperer_t1.sh"
