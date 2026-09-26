#!/usr/bin/env bash
# Démonstration T1 — tri de colis autonome avec des personnes.
#   bash lancer_t1.sh [durée_simulée_s] [accélération_vidéo] [--sans-video]
#   Premier essai :  bash lancer_t1.sh 240 1       (~4 min simulées, vidéo temps réel)
#   Démo complète :  bash lancer_t1.sh 1800 10     (30 min -> vidéo de 3 min)
set -euo pipefail
CONTENEUR="isim-isaac-sim-1"
DUREE="${1:-240}"; ACC="${2:-1}"; shift $(( $# > 2 ? 2 : $# )) || true
bash "$HOME/AXIS-OS/deploy/machine_gpu/copier_code.sh"
mkdir -p "$HOME/videos"
JOURNAL="$HOME/videos/t1_$(date +%Y%m%d_%H%M).log"
sudo docker exec "$CONTENEUR" /isaac-sim/python.sh /isaac-sim/axis_os/test_isaac_t1_tri_usine.py \
  --duree "$DUREE" --acceleration "$ACC" "$@" 2>&1 \
  | grep -v "ext:" | grep -v "Warning" | tee "$JOURNAL" | tail -60 || true
sudo docker cp "$CONTENEUR:/isaac-sim/videos/t1_tri_usine.mp4" "$HOME/videos/" 2>/dev/null \
  && echo "Vidéo : $HOME/videos/t1_tri_usine.mp4"
echo "Journal complet : $JOURNAL"
