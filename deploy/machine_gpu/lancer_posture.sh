#!/usr/bin/env bash
# Banc de la posture GR00T WBC : torse penché de 0 à 0,35 rad par paliers,
# bassin abaissé de 0 à 10 cm, robot immobile, sans puis avec le carton T1
# (1 kg) aux mains, pris par l'OS comme en T1, simulé dans Isaac Sim (le
# simulateur de robotique de NVIDIA) sur la machine GPU louée. Le test est
# sim/validation/test_isaac_w5_gr00t_posture.py (avec banc_posture.py et
# banc_prise.py), copié dans le conteneur par copier_code.sh.
# Décision de revue : l'app de tri ne se penche pas tant que ce banc n'est pas
# passé (docs/validation_port_carton.md, §9).
#
#   bash lancer_posture.sh               lance le banc (~2 min simulées), suit
#                                        son journal, récupère ses résultats
#   bash lancer_posture.sh --recuperer   suit et récupère seulement (le banc
#                                        tourne déjà, ou vient de finir)
#
# Résultats dans ~/videos : posture_AAAAMMJJ_HHMM.log (journal, verdict
# « BANC DE POSTURE : PASSÉ » ou « ÉCHOUÉ » et ses raisons),
# posture_AAAAMMJJ_HHMM.csv (un palier par ligne : accepté ou refusé par l'OS,
# marge d'équilibre prévue, inclinaison et vitesse angulaire du bassin lues
# par la centrale inertielle, glissement des pieds) et posture.mp4.
set -euo pipefail
CONTENEUR="isim-isaac-sim-1"
V=/isaac-sim/videos
if [ "${1:-}" != "--recuperer" ]; then
  # Anciens tests encore en cours : arrêtés d'abord (voir lancer_t1.sh).
  if sudo docker exec "$CONTENEUR" pgrep -f test_isaac_ >/dev/null 2>&1; then
    echo "Arrêt des anciens tests encore en cours dans le conteneur…"
    sudo docker exec "$CONTENEUR" pkill -f test_isaac_ || true
    sleep 5
  fi
  # La carte graphique doit être visible DANS le conteneur (voir lancer_t1.sh).
  if ! sudo docker exec "$CONTENEUR" nvidia-smi -L >/dev/null 2>&1; then
    echo "Le conteneur ne voit pas la carte graphique — redémarrage du conteneur…"
    sudo docker restart "$CONTENEUR" >/dev/null
    sleep 10
    if ! sudo docker exec "$CONTENEUR" nvidia-smi -L >/dev/null 2>&1; then
      echo "ERREUR : toujours pas de carte graphique dans le conteneur. Redémarre l'instance"
      echo "depuis la page Vast.ai (Reboot), puis relance ce script."
      exit 1
    fi
  fi
  bash "$HOME/AXIS-OS/deploy/machine_gpu/copier_code.sh"
  sudo docker exec "$CONTENEUR" rm -f "$V/posture.log" "$V/posture.fini" "$V/posture.csv"
  # Détaché (docker exec -d) : une connexion SSH coupée n'arrête pas le banc.
  sudo docker exec -d "$CONTENEUR" bash -c "mkdir -p $V && /isaac-sim/python.sh -u \
    /isaac-sim/axis_os/test_isaac_w5_gr00t_posture.py > $V/posture.log 2>&1; touch $V/posture.fini"
  echo "Banc lancé en arrière-plan (il continue même si tu te déconnectes)."
  sleep 3
fi
# Suivi du journal jusqu'à la fin du banc (comme suivre_t1.sh) ; Ctrl+C coupe
# l'affichage, pas le banc : reprendre avec --recuperer.
sudo docker exec "$CONTENEUR" bash -c "
  until [ -f $V/posture.fini ]; do sleep 2; done &
  tail -n +1 -f --pid=\$! $V/posture.log" \
  | grep --line-buffered -v -e "ext:" -e "Warning" -e "cudainterop" || true
mkdir -p "$HOME/videos"
H=$(date +%Y%m%d_%H%M)
sudo docker cp "$CONTENEUR:$V/posture.log" "$HOME/videos/posture_$H.log" 2>/dev/null \
  && echo "Journal : $HOME/videos/posture_$H.log"
if sudo docker exec "$CONTENEUR" test -f "$V/posture.fini"; then
  sudo docker cp "$CONTENEUR:$V/posture.csv" "$HOME/videos/posture_$H.csv" 2>/dev/null \
    && echo "Relevé par palier : $HOME/videos/posture_$H.csv"
  sudo docker cp "$CONTENEUR:$V/posture.mp4" "$HOME/videos/" 2>/dev/null \
    && echo "Vidéo : $HOME/videos/posture.mp4"
  echo "--- bilan ---"
  sed -n '/RÉSULTATS POSTURE/,/Résultats :/p' "$HOME/videos/posture_$H.log" 2>/dev/null
else
  echo "Le banc tourne encore. Reprendre : bash ~/AXIS-OS/deploy/machine_gpu/lancer_posture.sh --recuperer"
fi
