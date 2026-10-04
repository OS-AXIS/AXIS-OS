#!/usr/bin/env bash
# Collecte de démonstrations de prise pour entraîner le VLA (GR00T).
#   bash collecte_vla.sh [nombre_d_episodes]        (200 par défaut)
# Tourne en arrière-plan comme le test T1 (une déconnexion ne l'arrête pas).
# Les épisodes s'accumulent dans le conteneur, /isaac-sim/videos/vla_episodes ;
# relancer la collecte AJOUTE des épisodes (rien n'est écrasé).
# Récupérer ensuite : bash ~/AXIS-OS/deploy/machine_gpu/recuperer_vla.sh
N="${1:-200}"
exec bash "$HOME/AXIS-OS/deploy/machine_gpu/lancer_t1.sh" 60 1 \
  --collecte-vla /isaac-sim/videos/vla_episodes --episodes "$N" --sans-video
