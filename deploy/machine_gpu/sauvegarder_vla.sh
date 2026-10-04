#!/usr/bin/env bash
# Met les épisodes VLA à l'abri sur GitHub : une « release » BROUILLON du dépôt
# privé OS-AXIS/AXIS-OS (visible seulement par les membres du dépôt), avec
# l'archive des épisodes en pièce jointe. Une machine Vast détruite — ou à
# court de crédit — efface tout ; GitHub, non.
#
#   bash sauvegarder_vla.sh            sauvegarde (peut tourner PENDANT la collecte)
#   bash sauvegarder_vla.sh --restaurer   sur une nouvelle machine : récupère
#                                         la dernière sauvegarde dans le conteneur
set -euo pipefail
CONTENEUR="isim-isaac-sim-1"
DEPOT="OS-AXIS/AXIS-OS"
ETIQUETTE="vla-episodes"
if [ "${1:-}" = "--restaurer" ]; then
  cd "$HOME"
  rm -f vla_episodes.tar   # (le gh d'Ubuntu 22.04 n'a pas --clobber pour download)
  gh release download "$ETIQUETTE" --repo "$DEPOT" --pattern "vla_episodes.tar"
  tar xf vla_episodes.tar
  sudo docker exec "$CONTENEUR" mkdir -p /isaac-sim/videos
  sudo docker cp "$HOME/vla_episodes" "$CONTENEUR:/isaac-sim/videos/"
  sudo docker exec -u root "$CONTENEUR" chmod -R a+rwX /isaac-sim/videos/vla_episodes
  echo "Restauré : $(wc -l < vla_episodes/episodes.jsonl) épisodes, la collecte continuera à la suite."
  exit 0
fi
rm -rf "$HOME/vla_episodes"
sudo docker cp "$CONTENEUR:/isaac-sim/videos/vla_episodes" "$HOME/vla_episodes"
N=$(wc -l < "$HOME/vla_episodes/episodes.jsonl")
OK=$(grep -c '"succes": true' "$HOME/vla_episodes/episodes.jsonl" || true)
tar cf "$HOME/vla_episodes.tar" -C "$HOME" vla_episodes
if ! gh release view "$ETIQUETTE" --repo "$DEPOT" >/dev/null 2>&1; then
  gh release create "$ETIQUETTE" --repo "$DEPOT" --draft \
    --title "Épisodes VLA (collecte en simulation)" \
    --notes "Démonstrations de prise pour entraîner GR00T. Restaurer : bash deploy/machine_gpu/sauvegarder_vla.sh --restaurer"
fi
gh release upload "$ETIQUETTE" "$HOME/vla_episodes.tar" --repo "$DEPOT" --clobber
echo "Sauvegardé sur GitHub : $N épisodes ($OK réussis), $(du -h "$HOME/vla_episodes.tar" | cut -f1)."
