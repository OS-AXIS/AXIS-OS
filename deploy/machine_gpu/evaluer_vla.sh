#!/usr/bin/env bash
# Évaluation du VLA entraîné par entrainer_vla.sh, comparé aux réflexes
# codés.
#
# Un VLA (modèle « vision-langage-action ») est un réseau de neurones qui
# regarde par les caméras du robot et propose des mouvements ; ici, le modèle
# GR00T N1.7 de NVIDIA, affiné sur nos démonstrations de prise de cartons.
# Les « réflexes » sont les comportements programmés à la main (approche du
# carton et prise).
# Pour que la comparaison soit juste, chaque mode essaie les MÊMES cartons,
# dans Isaac Sim, le simulateur de robotique de NVIDIA.
#
#   bash evaluer_vla.sh [cartons] [accélération_vidéo] [options du test]
#       cartons      20 par défaut (chacun essayé 2 fois : réflexes puis VLA)
#       accélération 2 par défaut (vidéo) ; « --sans-video » en option l'enlève
#   Exemples :
#       bash evaluer_vla.sh                    20 cartons, avec vidéo
#       bash evaluer_vla.sh 5 1                essai rapide, vidéo en temps réel
#       bash evaluer_vla.sh 20 2 --modes vla   le VLA seul
#
# Déroulement :
# 1. Démarre en arrière-plan le serveur GR00T (le programme de NVIDIA qui
#    charge le modèle et répond aux demandes d'action) sur le dernier point
#    de sauvegarde (checkpoint) de ~/vla_gr00t_g1. Il tourne dans
#    l'environnement Python de NVIDIA, HORS du conteneur Docker d'Isaac Sim.
# 2. Lance dans le conteneur, en mode --evaluer-vla, le test de la
#    démonstration de tri de colis (dite « T1 », d'où le nom lancer_t1.sh) :
#    le test se connecte au serveur par le réseau interne de la machine
#    (127.0.0.1:5555).
# 3. Affiche le bilan dans le terminal ; le détail essai par essai est copié
#    dans ~/videos/eval_vla.json.
#
# Sécurité : le VLA ne fait que PROPOSER des mouvements. La vitesse des bras
# qu'il commande est plafonnée à 1,5 rad/s : 0,03 rad au plus par pas de
# 20 ms, réglé dans sim/validation/test_isaac_t1_tri_usine.py (à titre de
# comparaison, les réflexes referment les bras sur le carton à 1,44 rad/s au
# plus). Ses mouvements passent ensuite par les mêmes protections que ceux
# des réflexes : butées articulaires, JointSafetyGuard (le garde-fou de
# chaque articulation, qui refuse toute commande hors limites) et gardien de
# proximité (qui ralentit puis arrête le robot devant un obstacle). Les
# refus sont comptés pour chaque essai.
#
# Jeton d'API (facultatif) : un secret que chaque requête au serveur doit
# présenter. Usage :   GR00T_API_TOKEN=<jeton> bash evaluer_vla.sh …
# Le serveur n'accepte alors que les requêtes qui portent ce jeton. Il
# n'écoute déjà que sur 127.0.0.1, l'adresse interne de la machine ; le
# jeton le protège en plus des autres processus de la même machine.
# Le jeton ne passe QUE par une variable d'environnement : jamais par une
# ligne de commande (elle est visible dans « ps », la liste des processus,
# et sudo l'écrit dans son journal), ni par un fichier.
#   - Côté serveur : run_gr00t_server.py de NVIDIA n'a pas d'option
#     --api-token ; le lanceur run_gr00t_server_avec_jeton.py (à côté de ce
#     script) lit la variable et passe le jeton à PolicyServer, la classe
#     serveur de NVIDIA.
#   - Côté client : lancer_t1.sh le transmet au conteneur par l'entrée
#     standard (docker exec --env-file /dev/stdin), sans trace ni dans ps,
#     ni dans le journal de sudo.
# Sans la variable, rien ne change : run_gr00t_server.py est lancé tel quel.
set -euo pipefail
# Avec « set -e », le script s'arrête à la première commande en échec, et on
# a constaté qu'il le faisait sans un mot. Ce piège (trap) affiche la ligne
# et la commande fautives, puis arrête le serveur GR00T s'il tourne.
trap 'echo "ÉCHEC (ligne $LINENO) : $BASH_COMMAND"; pkill -f run_gr00t_server || true' ERR
CARTONS="${1:-20}"; ACC="${2:-2}"; shift $(( $# > 2 ? 2 : $# )) || true
# Option « --prise » : évalue le modèle de prise seule, entraîné par
# « entrainer_vla.sh … prise ». Les réflexes amènent le robot jusqu'au
# carton, puis le VLA ne fait que la prise. Ce mode est comparé aux réflexes
# complets (approche comprise).
MODELES="$HOME/vla_gr00t_g1"
EXTRA=()
for a in "$@"; do
  if [ "$a" = "--prise" ]; then MODELES="$HOME/vla_gr00t_g1_prise"; EXTRA+=(--modes reflexes,vla_prise)
  else EXTRA+=("$a"); fi
done
set -- "${EXTRA[@]}"
CONTENEUR="isim-isaac-sim-1"
PORT=5555
export PATH="$HOME/.local/bin:$PATH"

# ---- le modèle : dernier point de sauvegarde ------------------------------
# sort -V trie les numéros comme des nombres (checkpoint-900 avant
# checkpoint-1000). Sans point de sauvegarde, on essaie le dossier lui-même.
# On refuse aussi d'évaluer tant que l'entraînement (launch_finetune) tourne
# encore : il faut attendre sa fin.
MODELE=$(ls -d "$MODELES"/checkpoint-* 2>/dev/null | sort -V | tail -1 || true)
[ -n "$MODELE" ] || MODELE="$MODELES"
if [ ! -f "$MODELE/config.json" ]; then
  echo "Pas de modèle entraîné dans $MODELES — lancer entrainer_vla.sh d'abord,"
  echo "ou restaurer celui de GitHub : sauvegarder_modele_vla.sh --restaurer"
  exit 1
fi
if pgrep -f launch_finetune >/dev/null 2>&1; then
  echo "L'entraînement tourne encore : attendre sa fin (tail -f ~/vla_entrainement.log)."
  exit 1
fi
echo "Modèle évalué : $MODELE"

# ---- 1. serveur GR00T ------------------------------------------------------
if pgrep -f run_gr00t_server >/dev/null 2>&1; then
  echo "Un serveur GR00T tourne déjà — arrêt, pour charger le bon modèle…"
  pkill -f run_gr00t_server || true
  sleep 3
fi
cd "$HOME/Isaac-GR00T"
# Option -u de Python (et PYTHONUNBUFFERED=1), plus bas : sans elle, quand
# la sortie va dans un fichier, Python garde ses messages en mémoire au lieu
# de les écrire, et « Server is ready » n'apparaissait jamais dans le journal
# (constaté sur une carte RTX PRO 5000).
SERVEUR="gr00t/eval/run_gr00t_server.py"
if [ -n "${GR00T_API_TOKEN:-}" ]; then
  # Même serveur de NVIDIA, démarré par le lanceur qui lui passe le jeton lu
  # dans l'environnement (voir l'en-tête). Le nom du lanceur contient
  # « run_gr00t_server » exprès : les pgrep et pkill de ce script, qui
  # cherchent et arrêtent le serveur par son nom, le trouvent aussi.
  SERVEUR="$HOME/AXIS-OS/deploy/machine_gpu/run_gr00t_server_avec_jeton.py"
  export GR00T_API_TOKEN               # lu par le lanceur, par lancer_t1.sh, puis par le client
  echo "Jeton d'API : activé (serveur et client)."
fi
nohup env PYTHONUNBUFFERED=1 uv run python -u "$SERVEUR" \
  --model-path "$MODELE" --embodiment-tag NEW_EMBODIMENT \
  --host 127.0.0.1 --port "$PORT" > "$HOME/vla_serveur.log" 2>&1 &
echo "Serveur GR00T en cours de chargement (journal : ~/vla_serveur.log)…"
# Le serveur est prêt quand son port écoute (ss -ltn liste les ports en
# écoute) : plus sûr que de chercher un message dans le journal, qui peut
# manquer. Le message « Server is ready » reste accepté en second recours.
# Attente de 5 minutes au plus (60 fois 5 s) ; si le serveur s'arrête
# entre-temps, la fin de son journal est affichée.
pret() { ss -ltn 2>/dev/null | grep -q ":$PORT " || grep -q "Server is ready" "$HOME/vla_serveur.log" 2>/dev/null; }
for i in $(seq 1 60); do
  if pret; then break; fi
  if ! pgrep -f run_gr00t_server >/dev/null 2>&1; then
    echo "ÉCHEC : le serveur GR00T s'est arrêté. Fin du journal :"; tail -20 "$HOME/vla_serveur.log"
    exit 1
  fi
  sleep 5
done
pret || { echo "ÉCHEC : serveur pas prêt après 5 min."; tail -20 "$HOME/vla_serveur.log";
           pkill -f run_gr00t_server || true; exit 1; }
echo "Serveur GR00T prêt sur 127.0.0.1:$PORT."

# ---- 2. client dans le conteneur Isaac Sim --------------------------------
if [ "$(sudo docker inspect -f '{{.State.Running}}' "$CONTENEUR" 2>/dev/null)" != "true" ]; then
  echo "Le conteneur Isaac Sim est arrêté — démarrage…"
  sudo docker start "$CONTENEUR" >/dev/null
  sleep 10
fi
# Le client GR00T, dans le conteneur, a besoin de pyzmq, msgpack et
# msgpack-numpy (communication réseau avec le serveur et encodage des
# messages, tableaux numpy compris). Constaté sur RTX PRO 5000 : le pip
# d'Isaac Sim est cassé (« No module named pip._vendor.idna.core »). Dans ce
# cas, on copie les paquets depuis l'environnement de GR00T, qui les a déjà
# et utilise la même version de Python (3.12).
importe() { sudo docker exec "$CONTENEUR" /isaac-sim/python.sh -c \
  "import zmq, msgpack, msgpack_numpy" >/dev/null 2>&1; }
# Versions épinglées (version exacte imposée, pour une installation
# reproductible) à la suite d'un audit de sécurité du dépôt (2026-10-10). On
# a vérifié sur PyPI (l'index public des paquets Python) qu'elles existent
# en paquets précompilés (« roues ») pour Python 3.12. Le repli par copie
# depuis GR00T, ci-dessous, reste tel quel.
if ! importe; then
  sudo docker exec -u root "$CONTENEUR" /isaac-sim/python.sh -m pip install -q \
    "pyzmq==27.1.0" "msgpack==1.2.1" "msgpack-numpy==0.4.8" >/dev/null 2>&1 || true
fi
if ! importe; then
  SP_GROOT=$(ls -d "$HOME"/Isaac-GR00T/.venv/lib/python3.12/site-packages)
  SP_ISAAC=$(sudo docker exec "$CONTENEUR" /isaac-sim/python.sh -c \
    "import site; print(site.getsitepackages()[0])" 2>/dev/null | tail -1 || true)
  [ -n "$SP_ISAAC" ] || SP_ISAAC=/isaac-sim/kit/python/lib/python3.12/site-packages
  echo "Copie de pyzmq/msgpack depuis GR00T vers $SP_ISAAC…"
  for m in zmq pyzmq.libs msgpack msgpack_numpy.py; do
    if [ -e "$SP_GROOT/$m" ]; then sudo docker cp "$SP_GROOT/$m" "$CONTENEUR:$SP_ISAAC/"; fi
  done
fi
importe || { echo "ÉCHEC : pyzmq/msgpack introuvables dans le conteneur."; \
             pkill -f run_gr00t_server || true; exit 1; }
echo "Client GR00T prêt dans le conteneur."

# ---- 3. évaluation ---------------------------------------------------------
# Le premier argument de lancer_t1.sh (la durée simulée, ici « 60 ») est
# ignoré : l'évaluation s'arrête d'elle-même après le dernier carton.
# Le détail est ensuite copié hors du conteneur et le serveur GR00T arrêté.
bash "$HOME/AXIS-OS/deploy/machine_gpu/lancer_t1.sh" 60 "$ACC" \
  --evaluer-vla "tcp://127.0.0.1:$PORT" --episodes "$CARTONS" "$@" || true
sudo docker cp "$CONTENEUR:/isaac-sim/videos/eval_vla.json" "$HOME/videos/eval_vla.json" 2>/dev/null \
  && echo "Détail essai par essai : ~/videos/eval_vla.json"
pkill -f run_gr00t_server || true
echo "Serveur GR00T arrêté. Pense à sauvegarder le modèle (sauvegarder_modele_vla.sh) avant Destroy."
