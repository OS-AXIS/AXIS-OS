#!/usr/bin/env bash
# Copie les épisodes VLA hors du conteneur, dit combien sont réussis, et les
# convertit au format LeRobot v2 (format d'entraînement de GR00T).
CONTENEUR="isim-isaac-sim-1"
rm -rf "$HOME/vla_episodes"
sudo docker cp "$CONTENEUR:/isaac-sim/videos/vla_episodes" "$HOME/vla_episodes" || exit 1
echo "Épisodes : $(wc -l < "$HOME/vla_episodes/episodes.jsonl") au total, \
$(grep -c '"succes": true' "$HOME/vla_episodes/episodes.jsonl") réussis"
command -v pip3 >/dev/null || sudo apt-get install -y -qq python3-pip
# Versions épinglées (audit du 2026-10-10), choisies pour le Python 3.10
# d'Ubuntu 22.04 : les dernières pandas/pyarrow/imageio exigent Python 3.11 —
# épingler la version du jour aurait CASSÉ ce script. Roues cp310 vérifiées
# sur PyPI. Les deux commandes pip restent : le pip d'Ubuntu 22.04 ne connaît
# pas --break-system-packages (la première échoue, la seconde installe).
DEPS=("pandas==2.3.3" "pyarrow==25.0.1" "imageio[ffmpeg]==2.37.4")
python3 -c "import pandas, pyarrow, imageio" 2>/dev/null || \
  pip install -q --break-system-packages "${DEPS[@]}" 2>/dev/null || \
  pip install -q "${DEPS[@]}"
cd "$HOME/AXIS-OS" && python3 -m axis_os.vla.vers_lerobot "$HOME/vla_episodes" "$HOME/vla_lerobot" \
  && echo "Jeu d'entraînement GR00T : $HOME/vla_lerobot ($(du -sh "$HOME/vla_lerobot" | cut -f1))"
