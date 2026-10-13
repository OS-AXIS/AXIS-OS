#!/usr/bin/env bash
# Copie dans ~/videos la vidéo et le journal du dernier test de la
# démonstration de tri de colis (dite « T1 », lancée par lancer_t1.sh), et
# dit si le test est terminé ou encore en cours. Le test tourne dans le
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
  echo "--- bilan ---"
  sed -n '/RÉSULTATS T1/,/Résultats :/p' "$HOME/videos/t1_$H.log" 2>/dev/null
fi
