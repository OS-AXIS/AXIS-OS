#!/usr/bin/env bash
# Collecte, en simulation, de démonstrations de prise pour entraîner le VLA
# GR00T. Un VLA (modèle Vision-Language-Action) apprend à produire des
# mouvements à partir d'une image, d'une consigne et de l'état du robot ;
# ici, il apprend en imitant le robot piloté par ses réflexes codés (le
# comportement programmé à la main). Chaque démonstration (un épisode) est
# une prise de carton enregistrée : prises répétées, cartons placés au
# hasard, sans personnes dans la scène (détail dans axis_os/vla/collecte.py).
#
#   bash collecte_vla.sh [nombre_d_episodes]        (200 par défaut)
#
# Le script réutilise lancer_t1.sh, le lanceur de la démonstration de tri de
# colis, en mode collecte et sans vidéo. Comme elle, il tourne en
# arrière-plan : une déconnexion ne l'arrête pas.
# Les épisodes s'accumulent dans le conteneur Docker (environnement isolé)
# d'Isaac Sim, le simulateur de NVIDIA, dans /isaac-sim/videos/vla_episodes ;
# relancer la collecte ajoute des épisodes (rien n'est écrasé). Pour les
# récupérer ensuite : bash ~/AXIS-OS/deploy/machine_gpu/recuperer_vla.sh
N="${1:-200}"
exec bash "$HOME/AXIS-OS/deploy/machine_gpu/lancer_t1.sh" 60 1 \
  --collecte-vla /isaac-sim/videos/vla_episodes --episodes "$N" --sans-video
