#!/usr/bin/env bash
# Copie dans ~/videos la vidéo et le journal du dernier test de la
# démonstration de tri de colis (dite « T1 », lancée par lancer_t1.sh), et
# dit si le test est terminé ou encore en cours. Une fois le test fini, il
# en fait aussi une version réduite de la vidéo (540p) et un .zip avec le
# journal, faciles à télécharger. Le test tourne dans le
# conteneur Docker d'Isaac Sim (le simulateur de robotique de NVIDIA) : ses
# fichiers n'en sortent que par « docker cp ».
CONTENEUR="isim-isaac-sim-1"
mkdir -p "$HOME/videos"
if sudo docker exec "$CONTENEUR" pgrep -f test_isaac_t1 >/dev/null 2>&1; then
  echo "Le test est ENCORE EN COURS — la vidéo n'est écrite qu'à la fin."
  echo "Suivre : bash ~/AXIS-OS/deploy/machine_gpu/suivre_t1.sh"
fi
# Le journal est copié sous un nom horodaté (t1_AAAAMMJJ_HHMM.log) ; la
# vidéo, elle, garde toujours le même nom et remplace la précédente.
H=$(date +%Y%m%d_%H%M)
sudo docker cp "$CONTENEUR:/isaac-sim/videos/t1.log" "$HOME/videos/t1_$H.log" 2>/dev/null \
  && echo "Journal : $HOME/videos/t1_$H.log"
# t1.fini est créé à la fin du test (voir lancer_t1.sh) : la vidéo n'existe
# qu'à partir de là. Le bilan affiché est l'extrait du journal compris entre
# « RÉSULTATS T1 » et « Résultats : ».
if sudo docker exec "$CONTENEUR" test -f /isaac-sim/videos/t1.fini; then
  sudo docker cp "$CONTENEUR:/isaac-sim/videos/t1_tri_usine.mp4" "$HOME/videos/" 2>/dev/null \
    && echo "Vidéo : $HOME/videos/t1_tri_usine.mp4"
  # Vue du robot (caméra de tête, ce que lit la perception, avec ce que
  # l'OS en croit) : t1_vue_robot.mp4, écrite avec la vidéo principale.
  sudo docker cp "$CONTENEUR:/isaac-sim/videos/t1_vue_robot.mp4" "$HOME/videos/" 2>/dev/null \
    && echo "Vue du robot : $HOME/videos/t1_vue_robot.mp4"
  echo "--- bilan ---"
  sed -n '/RÉSULTATS T1/,/Résultats :/p' "$HOME/videos/t1_$H.log" 2>/dev/null
  # Version réduite, prête à télécharger : 540 lignes, quelques Mo (sans
  # son, images ré-encodées en H.264 de qualité moyenne), archivée avec le
  # journal (et le détail de l'évaluation, s'il y en a un) dans un .zip.
  if [ -f "$HOME/videos/t1_tri_usine.mp4" ]; then
    command -v ffmpeg >/dev/null || sudo apt-get install -y -qq ffmpeg
    PETITE="$HOME/videos/t1_${H}_540p.mp4"
    ffmpeg -y -loglevel error -i "$HOME/videos/t1_tri_usine.mp4" -vf "scale=-2:540" \
      -c:v libx264 -preset veryfast -crf 30 -r 20 -an -movflags +faststart "$PETITE" \
      && echo "Vidéo réduite : $PETITE ($(du -h "$PETITE" | cut -f1))"
    VUE="$HOME/videos/t1_${H}_vue_robot_540p.mp4"
    [ -f "$HOME/videos/t1_vue_robot.mp4" ] && ffmpeg -y -loglevel error \
      -i "$HOME/videos/t1_vue_robot.mp4" -vf "scale=-2:540" -c:v libx264 -preset veryfast \
      -crf 30 -an -movflags +faststart "$VUE" \
      && echo "Vue du robot réduite : $VUE ($(du -h "$VUE" | cut -f1))"
    ( cd "$HOME/videos" && python3 -m zipfile -c "t1_$H.zip" "t1_$H.log" \
        $( [ -f "$PETITE" ] && basename "$PETITE" ) \
        $( [ -f "$VUE" ] && basename "$VUE" ) \
        $( [ -f eval_vla.json ] && echo eval_vla.json ) ) \
      && echo "À télécharger : $HOME/videos/t1_$H.zip ($(du -h "$HOME/videos/t1_$H.zip" | cut -f1))"
  fi
fi
