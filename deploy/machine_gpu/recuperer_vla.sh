#!/usr/bin/env bash
# Copie hors du conteneur d'Isaac Sim (le simulateur de robotique de NVIDIA)
# les épisodes collectés pour le VLA, dit combien sont réussis, puis les
# convertit au format LeRobot v2, le format de jeu de données qu'attend
# l'entraînement de GR00T (entrainer_vla.sh).
# Un VLA (modèle « vision-langage-action ») apprend à imiter des
# démonstrations ; chaque épisode est une démonstration de prise enregistrée
# en simulation par collecte_vla.sh. La conversion est faite par
# axis_os/vla/vers_lerobot.py.
CONTENEUR="isim-isaac-sim-1"
rm -rf "$HOME/vla_episodes"
sudo docker cp "$CONTENEUR:/isaac-sim/videos/vla_episodes" "$HOME/vla_episodes" || exit 1
echo "Épisodes : $(wc -l < "$HOME/vla_episodes/episodes.jsonl") au total, \
$(grep -c '"succes": true' "$HOME/vla_episodes/episodes.jsonl") réussis"
command -v pip3 >/dev/null || sudo apt-get install -y -qq python3-pip
# Versions épinglées (version exacte imposée) à la suite d'un audit de
# sécurité du dépôt (2026-10-10), et choisies pour le Python 3.10 d'Ubuntu
# 22.04 : les dernières versions de pandas, pyarrow et imageio exigent
# Python 3.11, si bien qu'épingler la version du jour aurait CASSÉ ce script.
# On a vérifié sur PyPI (l'index public des paquets Python) qu'elles
# existent en paquets précompilés (« roues ») pour Python 3.10.
# Les deux commandes pip restent nécessaires : le pip d'Ubuntu 22.04 ne
# connaît pas l'option --break-system-packages (qui autorise, sur les
# systèmes plus récents, l'installation dans le Python du système). Sur
# Ubuntu 22.04, la première commande échoue donc et la seconde installe.
DEPS=("pandas==2.3.3" "pyarrow==25.0.1" "imageio[ffmpeg]==2.37.4")
python3 -c "import pandas, pyarrow, imageio" 2>/dev/null || \
  pip install -q --break-system-packages "${DEPS[@]}" 2>/dev/null || \
  pip install -q "${DEPS[@]}"
cd "$HOME/AXIS-OS" && python3 -m axis_os.vla.vers_lerobot "$HOME/vla_episodes" "$HOME/vla_lerobot" \
  && echo "Jeu d'entraînement GR00T : $HOME/vla_lerobot ($(du -sh "$HOME/vla_lerobot" | cut -f1))"
