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
JOURNAL="$HOME/videos/t1_$(date +%Y%m%d_%H%M).log"
echo "Suivi en direct : http://localhost:8080 (après le tunnel SSH, voir en tête de ce script)"
echo "Journal complet : $JOURNAL"
# Affiché AU FIL DE L'EAU (pas seulement à la fin), et gardé dans le journal.
sudo docker exec "$CONTENEUR" /isaac-sim/python.sh -u /isaac-sim/axis_os/test_isaac_t1_tri_usine.py \
  --duree "$DUREE" --acceleration "$ACC" "$@" 2>&1 \
  | grep --line-buffered -v -e "ext:" -e "Warning" | tee "$JOURNAL" || true
sudo docker cp "$CONTENEUR:/isaac-sim/videos/t1_tri_usine.mp4" "$HOME/videos/" 2>/dev/null \
  && echo "Vidéo : $HOME/videos/t1_tri_usine.mp4"
echo "Journal complet : $JOURNAL"
