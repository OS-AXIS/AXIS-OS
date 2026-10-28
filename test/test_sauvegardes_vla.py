"""
Sauvegardes des épisodes et du modèle VLA dans une release BROUILLON de
GitHub (deploy/machine_gpu/sauvegarder_vla.sh, sauvegarder_modele_vla.sh,
releases_brouillon.sh), exécutées pour de vrai contre de faux gh, curl et
docker : aucun accès à GitHub ni à Isaac Sim.

Le faux gh ne connaît que l'API des releases (comme le gh d'Ubuntu 22.04
devant un brouillon, « gh release download/upload/view » y échouent) ; ses
filtres --jq passent par le vrai jq.
"""
import os
import shutil
import subprocess
import sys
import tarfile
import tempfile
from pathlib import Path

RACINE = Path(__file__).resolve().parents[1]
GPU = RACINE / "deploy/machine_gpu"
FAIL = 0


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


FAUX_GH = r'''#!/usr/bin/env bash
# Faux gh : l'API des releases d'un dépôt (une release par dépôt), dans $ETAT.
echo "gh $*" >> "$ETAT/journal"
if [ "$1 $2" = "auth token" ]; then echo "JETON-FICTIF-123"; exit 0; fi
[ "$1" = "api" ] || { echo "gh $1 : non simulé (release introuvable)" >&2; exit 1; }
shift
methode=GET; jq_=""; champs=()
while [ $# -gt 0 ]; do
  case "$1" in
    --method) methode="$2"; shift 2;;
    --jq) jq_="$2"; shift 2;;
    --paginate) shift;;
    -H) shift 2;;
    -f|-F) champs+=("$2"); shift 2;;
    *) chemin="$1"; shift;;
  esac
done
chemin="${chemin%%\?*}"
depot=$(echo "$chemin" | cut -d/ -f2,3); d="$ETAT/$(echo "$depot" | tr / _)"
sortie() { if [ -n "$jq_" ]; then jq -r "$jq_"; else cat; fi; }
aid() { cksum <<< "$1" | cut -d' ' -f1; }
case "$methode $chemin" in
  "GET repos/$depot/releases")
    if [ -f "$d/tag" ]; then printf '[{"id": 77, "tag_name": "%s", "draft": true}]' "$(cat "$d/tag")"
    else echo '[]'; fi | sortie;;
  "POST repos/$depot/releases")
    mkdir -p "$d/pieces"
    for c in "${champs[@]}"; do
      [ "${c%%=*}" = tag_name ] && echo "${c#*=}" > "$d/tag"
      [ "${c%%=*}" = draft ] && echo "${c#*=}" > "$d/draft"
    done
    echo '{"id": 77}' | sortie;;
  "GET repos/$depot/releases/77/assets")
    ( echo '['; sep=""
      for f in "$d/pieces"/*; do [ -e "$f" ] || continue; n=$(basename "$f")
        printf '%s{"id": %s, "name": "%s"}' "$sep" "$(aid "$n")" "$n"; sep=","; done
      echo ']' ) | sortie;;
  GET\ repos/$depot/releases/assets/*)
    for f in "$d/pieces"/*; do [ "$(aid "$(basename "$f")")" = "${chemin##*/}" ] && cat "$f"; done;;
  DELETE\ repos/$depot/releases/assets/*)
    for f in "$d/pieces"/*; do [ "$(aid "$(basename "$f")")" = "${chemin##*/}" ] && rm -f "$f"; done;;
  *) echo "faux gh : $methode $chemin non simulé" >&2; exit 1;;
esac
'''
FAUX_CURL = r'''#!/usr/bin/env bash
echo "curl $*" >> "$ETAT/journal"
conf=$(cat)
echo "$conf" | grep -q 'Authorization: Bearer JETON-FICTIF-123' || { echo "sans jeton" >&2; exit 1; }
f=""; url=""
while [ $# -gt 0 ]; do
  case "$1" in -T) f="$2"; shift 2;; -X|-H|-o|-K|--retry) shift 2;; -*) shift;; *) url="$1"; shift;; esac
done
d="$ETAT/$(echo "$url" | cut -d/ -f5,6 | tr / _)"; nom=${url##*name=}
[ ! -e "$d/pieces/$nom" ] || { echo "422 already_exists" >&2; exit 22; }
cp "$f" "$d/pieces/$nom"
'''
FAUX_DOCKER = r'''#!/usr/bin/env bash
case "$1" in
  cp) if [[ "$2" == *:* ]]; then cp -r "$ETAT/conteneur/${2#*:}" "$3"
      else mkdir -p "$ETAT/conteneur/$(dirname "${3#*:}")"; cp -r "$2" "$ETAT/conteneur/${3#*:}"; fi;;
esac
'''

manque = [o for o in ("bash", "jq", "tar", "cksum", "split") if shutil.which(o) is None]
if manque:
    print(f"  [—] {', '.join(manque)} absent(s) : scripts non exécutables ici")
    sys.exit(0)

b = Path(tempfile.mkdtemp())
(b / "bin").mkdir()
for nom, code in (("gh", FAUX_GH), ("curl", FAUX_CURL), ("docker", FAUX_DOCKER),
                  ("sudo", '#!/usr/bin/env bash\nexec "$@"\n')):
    (b / "bin" / nom).write_text(code)
    (b / "bin" / nom).chmod(0o755)
etat, home = b / "etat", b / "home"
etat.mkdir()
home.mkdir()
env = dict(os.environ, PATH=f"{b / 'bin'}:{os.environ['PATH']}", ETAT=str(etat), HOME=str(home))
episodes = etat / "conteneur/isaac-sim/videos/vla_episodes"
episodes.mkdir(parents=True)
pieces = etat / "OS-AXIS_AXIS-OS/pieces"


def lancer(script, *args):
    return subprocess.run(["bash", str(GPU / script), *args], env=env, capture_output=True,
                          text=True, timeout=120)


section("1 — Épisodes : brouillon créé, puis retrouvé et remplacé")
(episodes / "episodes.jsonl").write_text('{"succes": true}\n{"succes": false}\n')
r = lancer("sauvegarder_vla.sh")
check("1re sauvegarde : release BROUILLON créée, archive jointe", r.returncode == 0
      and (etat / "OS-AXIS_AXIS-OS/draft").read_text().strip() == "true"
      and (pieces / "vla_episodes.tar").exists(), r.stderr[-300:])
(episodes / "episodes.jsonl").write_text('{"succes": true}\n' * 3)
r = lancer("sauvegarder_vla.sh")
with tarfile.open(pieces / "vla_episodes.tar") as t:
    n = t.extractfile("vla_episodes/episodes.jsonl").read().count(b"\n")
check("2e sauvegarde : le brouillon est retrouvé par l'API, l'archive remplacée",
      r.returncode == 0 and n == 3 and "3 épisodes" in r.stdout, r.stdout + r.stderr[-300:])
journal = (etat / "journal").read_text()
check("Jamais « gh release … » (le gh d'Ubuntu ne retrouve pas un brouillon)",
      "gh release" not in journal)
check("Le jeton de gh n'apparaît sur aucune ligne de commande (curl le lit sur stdin)",
      "JETON-FICTIF" not in journal)

section("2 — Restauration, et restauration depuis OS-AXIS/axis-os-ancien")
r = lancer("sauvegarder_vla.sh", "--restaurer")
check("--restaurer : 3 épisodes recopiés dans le conteneur", r.returncode == 0
      and "Restauré : 3 épisodes" in r.stdout, r.stdout + r.stderr[-300:])
r = lancer("sauvegarder_vla.sh", "--restaurer-ancien")
check("--restaurer-ancien sans release dans l'ancien dépôt : échec clair",
      r.returncode != 0 and "OS-AXIS/axis-os-ancien" in r.stdout)
ancien = etat / "OS-AXIS_axis-os-ancien"
(ancien / "pieces").mkdir(parents=True)
(ancien / "tag").write_text("vla-episodes\n")
src = b / "src/vla_episodes"
src.mkdir(parents=True)
(src / "episodes.jsonl").write_text('{"succes": true}\n' * 7)
with tarfile.open(ancien / "pieces/vla_episodes.tar", "w") as t:
    t.add(src, arcname="vla_episodes")
r = lancer("sauvegarder_vla.sh", "--restaurer-ancien")
check("--restaurer-ancien : les 7 épisodes de l'ancien dépôt", r.returncode == 0
      and "Restauré : 7 épisodes" in r.stdout
      and (etat / "conteneur/isaac-sim/videos/vla_episodes/episodes.jsonl")
      .read_text().count("\n") == 7, r.stdout + r.stderr[-300:])

section("3 — Modèle : poids seuls, vieux morceaux retirés, restauration")
ck = home / "vla_gr00t_g1/checkpoint-1000"
ck.mkdir(parents=True)
(home / "vla_gr00t_g1/checkpoint-900").mkdir()
(ck / "config.json").write_text("{}")
(ck / "model.safetensors").write_text("poids")
(ck / "optimizer.pt").write_text("état de l'optimiseur")
(etat / "OS-AXIS_AXIS-OS/tag").unlink()          # une release par dépôt dans ce faux gh
r = lancer("sauvegarder_modele_vla.sh")
check("Sauvegarde du dernier checkpoint", r.returncode == 0 and "checkpoint-1000" in r.stdout,
      r.stderr[-300:])
(pieces / "vla_modele.tar.part07").write_text("morceau d'une sauvegarde plus longue")
r = lancer("sauvegarder_modele_vla.sh")
check("Nouvelle sauvegarde : le vieux morceau part07 est retiré",
      r.returncode == 0 and not (pieces / "vla_modele.tar.part07").exists()
      and (pieces / "vla_modele.tar.part00").exists(), r.stderr[-300:])
shutil.rmtree(home / "vla_gr00t_g1")
r = lancer("sauvegarder_modele_vla.sh", "--restaurer")
check("--restaurer : poids et configuration, sans l'état de l'optimiseur",
      r.returncode == 0 and (ck / "model.safetensors").read_text() == "poids"
      and (ck / "config.json").exists() and not (ck / "optimizer.pt").exists(),
      r.stdout + r.stderr[-300:])

shutil.rmtree(b)
print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
