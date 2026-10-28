#!/usr/bin/env bash
# Met à l'abri sur GitHub les épisodes collectés pour le VLA : les
# démonstrations de prise enregistrées en simulation par collecte_vla.sh, qui
# servent à entraîner le modèle « vision-langage-action » GR00T de NVIDIA.
# Pourquoi : une machine Vast.ai (le service de location de machines GPU)
# détruite, ou à court de crédit, efface tout ; GitHub, non.
# La sauvegarde est une « release » (fichiers publiés à côté du code du
# dépôt) à l'état de BROUILLON, dans le dépôt OS-AXIS/AXIS-OS. Ce dépôt est
# public : seul l'état de brouillon réserve la release aux personnes qui ont
# un accès en écriture. Ne jamais la publier. L'archive des épisodes y est
# en pièce jointe. gh est l'outil en ligne de commande de GitHub ; la release
# est lue et écrite par son API (releases_brouillon.sh), qui retrouve les
# brouillons quelle que soit la version de gh.
#
#   bash sauvegarder_vla.sh            sauvegarde (peut tourner PENDANT la collecte)
#   bash sauvegarder_vla.sh --restaurer   sur une nouvelle machine : récupère
#                                         la dernière sauvegarde dans le conteneur
#   bash sauvegarder_vla.sh --restaurer-ancien   idem, depuis le dépôt
#                                         OS-AXIS/axis-os-ancien (l'historique
#                                         d'avant la réécriture), même
#                                         release « vla-episodes »
set -euo pipefail
source "$(dirname "$0")/releases_brouillon.sh"
CONTENEUR="isim-isaac-sim-1"
DEPOT="OS-AXIS/AXIS-OS"
ETIQUETTE="vla-episodes"
# Restauration : télécharge l'archive, la déballe et la copie dans le
# conteneur d'Isaac Sim (le simulateur), avec des droits ouverts en écriture
# pour que la collecte, relancée ensuite, ajoute ses épisodes à la suite.
if [ "${1:-}" = "--restaurer" ] || [ "${1:-}" = "--restaurer-ancien" ]; then
  [ "$1" = "--restaurer" ] || DEPOT="OS-AXIS/axis-os-ancien"
  cd "$HOME"
  ID=$(release_id "$DEPOT" "$ETIQUETTE")
  [ -n "$ID" ] || { echo "Aucune release « $ETIQUETTE » dans $DEPOT."; exit 1; }
  rm -f vla_episodes.tar
  release_telecharger "$DEPOT" "$ID" '^vla_episodes\\.tar$'
  tar xf vla_episodes.tar
  sudo docker exec "$CONTENEUR" mkdir -p /isaac-sim/videos
  sudo docker cp "$HOME/vla_episodes" "$CONTENEUR:/isaac-sim/videos/"
  sudo docker exec -u root "$CONTENEUR" chmod -R a+rwX /isaac-sim/videos/vla_episodes
  echo "Restauré : $(wc -l < vla_episodes/episodes.jsonl) épisodes, la collecte continuera à la suite."
  exit 0
fi
# Sauvegarde : copie les épisodes hors du conteneur, les compte, en fait une
# archive tar et l'envoie dans la release, créée au premier passage.
# L'envoi remplace l'archive de la sauvegarde précédente.
# grep -c sort en erreur quand il ne trouve rien : « || true » évite que
# set -e arrête le script quand aucun épisode n'est réussi.
rm -rf "$HOME/vla_episodes"
sudo docker cp "$CONTENEUR:/isaac-sim/videos/vla_episodes" "$HOME/vla_episodes"
N=$(wc -l < "$HOME/vla_episodes/episodes.jsonl")
OK=$(grep -c '"succes": true' "$HOME/vla_episodes/episodes.jsonl" || true)
tar cf "$HOME/vla_episodes.tar" -C "$HOME" vla_episodes
ID=$(release_id "$DEPOT" "$ETIQUETTE")
[ -n "$ID" ] || ID=$(release_creer "$DEPOT" "$ETIQUETTE" "Épisodes VLA (collecte en simulation)" \
  "Démonstrations de prise pour entraîner GR00T. Restaurer : bash deploy/machine_gpu/sauvegarder_vla.sh --restaurer")
release_envoyer "$DEPOT" "$ID" "$HOME/vla_episodes.tar"
echo "Sauvegardé sur GitHub : $N épisodes ($OK réussis), $(du -h "$HOME/vla_episodes.tar" | cut -f1)."
