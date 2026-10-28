#!/usr/bin/env bash
# Fonctions communes à sauvegarder_vla.sh et sauvegarder_modele_vla.sh : lire
# et écrire une release BROUILLON de GitHub par l'API, avec gh.
#
# Pourquoi l'API : une release brouillon n'a pas encore d'étiquette publiée,
# et « gh release download/upload/view <étiquette> » ne la retrouve pas
# toujours. Constaté le 2026-10-28 avec le gh d'Ubuntu 22.04 (2.4) :
# « release not found » sur nos deux sauvegardes. La liste des releases de
# l'API, elle, contient les brouillons pour qui a un accès en écriture au
# dépôt : on y cherche la release par son étiquette, puis on travaille sur
# son numéro (id).
#
# Le jeton GitHub de gh ne paraît jamais sur une ligne de commande (visible
# dans « ps ») : curl le lit sur son entrée standard (-K -).
#
# Usage : source "$(dirname "$0")/releases_brouillon.sh"

# release_id DEPOT ETIQUETTE : numéro de la release (brouillon compris), ou
# rien si elle n'existe pas.
release_id() {
  gh api "repos/$1/releases?per_page=100" --paginate \
    --jq ".[] | select(.tag_name == \"$2\") | .id" | head -1
}

# release_creer DEPOT ETIQUETTE TITRE NOTES : crée la release en BROUILLON
# (jamais publiée : le dépôt est public) et affiche son numéro.
release_creer() {
  gh api --method POST "repos/$1/releases" -f tag_name="$2" -f name="$3" -f body="$4" \
    -F draft=true --jq .id
}

# release_pieces DEPOT ID MOTIF : « numéro nom » de chaque pièce jointe dont
# le nom correspond à l'expression régulière MOTIF.
release_pieces() {
  gh api "repos/$1/releases/$2/assets?per_page=100" --paginate \
    --jq ".[] | select(.name | test(\"$3\")) | \"\(.id) \(.name)\""
}

# release_telecharger DEPOT ID MOTIF : télécharge dans le dossier courant
# les pièces jointes dont le nom correspond à MOTIF ; échoue s'il n'y en a
# aucune.
release_telecharger() {
  local pieces n=0 aid nom
  pieces=$(release_pieces "$1" "$2" "$3")
  [ -n "$pieces" ] || { echo "Aucune pièce jointe « $3 » dans la release $2 de $1." >&2; return 1; }
  while read -r aid nom; do
    gh api -H "Accept: application/octet-stream" "repos/$1/releases/assets/$aid" > "$nom"
    n=$((n + 1))
  done <<< "$pieces"
  echo "Téléchargé : $n pièce(s) jointe(s) de $1."
}

# release_envoyer DEPOT ID FICHIER… : joint les fichiers à la release, en
# remplaçant une pièce jointe de même nom (comme --clobber).
release_envoyer() {
  local depot="$1" id="$2" f nom ancien
  shift 2
  for f in "$@"; do
    nom=$(basename "$f")
    ancien=$(release_pieces "$depot" "$id" "^${nom//./\\\\.}\$" | cut -d' ' -f1)
    [ -z "$ancien" ] || gh api --method DELETE "repos/$depot/releases/assets/$ancien" >/dev/null
    # -T envoie le fichier sans le charger en mémoire (morceaux de 1,9 Go).
    printf 'header = "Authorization: Bearer %s"\n' "$(gh auth token)" \
      | curl -fsS --retry 3 -K - -X POST -T "$f" -o /dev/null \
          -H "Content-Type: application/octet-stream" \
          "https://uploads.github.com/repos/$depot/releases/$id/assets?name=$nom"
  done
}
