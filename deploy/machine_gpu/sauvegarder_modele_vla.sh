#!/usr/bin/env bash
# Met à l'abri sur GitHub le modèle VLA entraîné, pour ne pas refaire
# l'entraînement sur la prochaine machine louée. Le VLA (modèle
# « vision-langage-action ») est ici GR00T N1.7 de NVIDIA, affiné par
# entrainer_vla.sh pour la prise de cartons par l'humanoïde G1 d'Unitree.
# La sauvegarde est une « release » (fichiers publiés à côté du code du
# dépôt) nommée « vla-modele », à l'état de BROUILLON, dans le dépôt
# OS-AXIS/AXIS-OS. Ce dépôt est public : seul l'état de brouillon réserve la
# release aux personnes qui ont un accès en écriture. Ne jamais la publier :
# les poids sont dérivés de GR00T N1.7, sous licence NVIDIA (THIRD_PARTY.md).
# gh est l'outil en ligne de commande de GitHub ; la release est lue et
# écrite par son API (releases_brouillon.sh), qui retrouve les brouillons
# quelle que soit la version de gh.
# Seuls les POIDS et la configuration sont gardés, pas l'état de
# l'optimiseur : il est plusieurs fois plus gros, et inutile pour se servir
# du modèle.
# GitHub limite une pièce jointe à 2 Go : l'archive est donc découpée en
# morceaux de 1 900 Mo.
# Limite : seul ~/vla_gr00t_g1 est sauvegardé, pas le modèle de prise seule
# (~/vla_gr00t_g1_prise, voir entrainer_vla.sh).
#
#   bash sauvegarder_modele_vla.sh               met à l'abri le dernier
#                                                point de sauvegarde
#                                                (checkpoint)
#   bash sauvegarder_modele_vla.sh --restaurer   le remet dans ~/vla_gr00t_g1
set -euo pipefail
source "$(dirname "$0")/releases_brouillon.sh"
DEPOT="OS-AXIS/AXIS-OS"
ETIQUETTE="vla-modele"
MODELES="$HOME/vla_gr00t_g1"
cd "$HOME"
# Restauration : télécharge les morceaux, les recolle (cat) et déballe
# l'archive dans ~/vla_gr00t_g1, où evaluer_vla.sh ira la chercher.
if [ "${1:-}" = "--restaurer" ]; then
  ID=$(release_id "$DEPOT" "$ETIQUETTE")
  [ -n "$ID" ] || { echo "Aucune release « $ETIQUETTE » dans $DEPOT."; exit 1; }
  rm -rf vla_modele_parts && mkdir vla_modele_parts && cd vla_modele_parts
  release_telecharger "$DEPOT" "$ID" '^vla_modele\\.tar\\.part'
  mkdir -p "$MODELES"
  cat vla_modele.tar.part* | tar xf - -C "$MODELES"
  cd "$HOME" && rm -rf vla_modele_parts
  echo "Modèle restauré : $(ls -d "$MODELES"/checkpoint-* | tail -1)"
  exit 0
fi
# Dernier point de sauvegarde : sort -V trie les numéros comme des nombres
# (checkpoint-900 avant checkpoint-1000).
MODELE=$(ls -d "$MODELES"/checkpoint-* 2>/dev/null | sort -V | tail -1 || true)
[ -n "$MODELE" ] || { echo "Aucun checkpoint dans $MODELES"; exit 1; }
NOM=$(basename "$MODELE")
rm -rf vla_modele_parts && mkdir vla_modele_parts
# Exclus de l'archive : les fichiers d'état de l'entraînement (optimizer*,
# global_step*, rng_state*, scheduler.pt), inutiles pour se servir du modèle.
tar cf - -C "$MODELES" --exclude="$NOM/optimizer*" --exclude="$NOM/global_step*" \
  --exclude="$NOM/rng_state*" --exclude="$NOM/scheduler.pt" "$NOM" \
  | split -b 1900M -d - vla_modele_parts/vla_modele.tar.part
echo "Archive : $(du -sh vla_modele_parts | cut -f1) en $(ls vla_modele_parts | wc -l) morceaux"
ID=$(release_id "$DEPOT" "$ETIQUETTE")
[ -n "$ID" ] || ID=$(release_creer "$DEPOT" "$ETIQUETTE" \
  "Modèle VLA GR00T N1.7 fine-tuné (G1, prise de cartons)" \
  "Poids seuls. Restaurer : bash deploy/machine_gpu/sauvegarder_modele_vla.sh --restaurer")
# Les morceaux d'une sauvegarde plus ancienne et plus longue ne doivent pas
# rester : la restauration les recollerait tous.
release_pieces "$DEPOT" "$ID" '^vla_modele\\.tar\\.part' | while read -r aid nom; do
  [ -e "vla_modele_parts/$nom" ] || gh api --method DELETE "repos/$DEPOT/releases/assets/$aid" >/dev/null
done
release_envoyer "$DEPOT" "$ID" vla_modele_parts/*
rm -rf vla_modele_parts
echo "Sauvegardé sur GitHub : $NOM (release brouillon « $ETIQUETTE »)."
