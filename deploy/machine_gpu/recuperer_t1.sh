#!/usr/bin/env bash
# Copie la vidéo et le journal du dernier test T1 dans ~/videos, et dit si le
# test est terminé ou encore en cours.
CONTENEUR="isim-isaac-sim-1"
mkdir -p "$HOME/videos"
if sudo docker exec "$CONTENEUR" pgrep -f test_isaac_t1 >/dev/null 2>&1; then
  echo "Le test est ENCORE EN COURS — la vidéo n'est écrite qu'à la fin."
  echo "Suivre : bash ~/AXIS-OS/deploy/machine_gpu/suivre_t1.sh"
fi
H=$(date +%Y%m%d_%H%M)
sudo docker cp "$CONTENEUR:/isaac-sim/videos/t1.log" "$HOME/videos/t1_$H.log" 2>/dev/null \
  && echo "Journal : $HOME/videos/t1_$H.log"
if sudo docker exec "$CONTENEUR" test -f /isaac-sim/videos/t1.fini; then
  sudo docker cp "$CONTENEUR:/isaac-sim/videos/t1_tri_usine.mp4" "$HOME/videos/" 2>/dev/null \
    && echo "Vidéo : $HOME/videos/t1_tri_usine.mp4"
  echo "--- bilan ---"
  sed -n '/RÉSULTATS T1/,/Résultats :/p' "$HOME/videos/t1_$H.log" 2>/dev/null
fi
