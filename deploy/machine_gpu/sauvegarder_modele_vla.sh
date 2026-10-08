#!/usr/bin/env bash
# Met le modèle VLA entraîné à l'abri sur GitHub (release BROUILLON
# « vla-modele » du dépôt privé), pour ne pas refaire l'entraînement sur la
# prochaine machine. Seuls les POIDS et la configuration sont gardés (pas
# l'état de l'optimiseur, plusieurs fois plus gros, inutile pour s'en servir).
# GitHub limite une pièce jointe à 2 Go : l'archive est découpée.
#
#   bash sauvegarder_modele_vla.sh               sauvegarde le dernier point
#   bash sauvegarder_modele_vla.sh --restaurer   le remet dans ~/vla_gr00t_g1
set -euo pipefail
DEPOT="OS-AXIS/AXIS-OS"
ETIQUETTE="vla-modele"
MODELES="$HOME/vla_gr00t_g1"
cd "$HOME"
if [ "${1:-}" = "--restaurer" ]; then
  rm -rf vla_modele_parts && mkdir vla_modele_parts && cd vla_modele_parts
  gh release download "$ETIQUETTE" --repo "$DEPOT" --pattern "vla_modele.tar.part*"
  mkdir -p "$MODELES"
  cat vla_modele.tar.part* | tar xf - -C "$MODELES"
  cd "$HOME" && rm -rf vla_modele_parts
  echo "Modèle restauré : $(ls -d "$MODELES"/checkpoint-* | tail -1)"
  exit 0
fi
MODELE=$(ls -d "$MODELES"/checkpoint-* 2>/dev/null | sort -V | tail -1 || true)
[ -n "$MODELE" ] || { echo "Aucun checkpoint dans $MODELES"; exit 1; }
NOM=$(basename "$MODELE")
rm -rf vla_modele_parts && mkdir vla_modele_parts
tar cf - -C "$MODELES" --exclude="$NOM/optimizer*" --exclude="$NOM/global_step*" \
  --exclude="$NOM/rng_state*" --exclude="$NOM/scheduler.pt" "$NOM" \
  | split -b 1900M -d - vla_modele_parts/vla_modele.tar.part
echo "Archive : $(du -sh vla_modele_parts | cut -f1) en $(ls vla_modele_parts | wc -l) morceaux"
if ! gh release view "$ETIQUETTE" --repo "$DEPOT" >/dev/null 2>&1; then
  gh release create "$ETIQUETTE" --repo "$DEPOT" --draft \
    --title "Modèle VLA GR00T N1.7 fine-tuné (G1, prise de cartons)" \
    --notes "Poids seuls. Restaurer : bash deploy/machine_gpu/sauvegarder_modele_vla.sh --restaurer"
fi
gh release upload "$ETIQUETTE" vla_modele_parts/* --repo "$DEPOT" --clobber
rm -rf vla_modele_parts
echo "Sauvegardé sur GitHub : $NOM (release brouillon « $ETIQUETTE »)."
