#!/usr/bin/env bash
# Évaluation du VLA entraîné (entrainer_vla.sh) contre les réflexes codés :
# les MÊMES cartons, essayés par chaque mode, dans Isaac Sim.
#
#   bash evaluer_vla.sh [cartons] [accélération_vidéo] [options du test]
#       cartons      20 par défaut (chacun essayé 2 fois : réflexes puis VLA)
#       accélération 2 par défaut (vidéo) ; « --sans-video » en option l'enlève
#   Exemples :
#       bash evaluer_vla.sh                    20 cartons, avec vidéo
#       bash evaluer_vla.sh 5 1                essai rapide, vidéo en temps réel
#       bash evaluer_vla.sh 20 2 --modes vla   le VLA seul
#
# 1. Démarre le serveur GR00T (environnement de NVIDIA, HORS du conteneur) sur
#    le dernier point de sauvegarde de ~/vla_gr00t_g1, en arrière-plan.
# 2. Lance le test T1 en mode --evaluer-vla dans le conteneur : il se connecte
#    au serveur par le réseau de la machine (127.0.0.1:5555).
# 3. Bilan dans le terminal ; détail essai par essai dans ~/videos/eval_vla.json.
#
# Le VLA ne fait que PROPOSER : bras à 1 rad/s au plus, butées, JointSafetyGuard,
# gardien de proximité — comme les réflexes. Les refus sont comptés par essai.
set -euo pipefail
# Jamais d'arrêt muet : constaté, le script s'arrêtait sans un mot.
trap 'echo "ÉCHEC (ligne $LINENO) : $BASH_COMMAND"; pkill -f run_gr00t_server || true' ERR
CARTONS="${1:-20}"; ACC="${2:-2}"; shift $(( $# > 2 ? 2 : $# )) || true
# « --prise » : modèle de prise seule (entrainer_vla.sh … prise) ; les réflexes
# s'approchent, le VLA ne fait que la prise. Comparé aux réflexes complets.
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
# -u : sans lui, Python garde ses messages en mémoire quand ils vont dans un
# fichier — « Server is ready » n'apparaissait jamais (constaté, RTX PRO 5000).
nohup env PYTHONUNBUFFERED=1 uv run python -u gr00t/eval/run_gr00t_server.py \
  --model-path "$MODELE" --embodiment-tag NEW_EMBODIMENT \
  --host 127.0.0.1 --port "$PORT" > "$HOME/vla_serveur.log" 2>&1 &
echo "Serveur GR00T en cours de chargement (journal : ~/vla_serveur.log)…"
# Prêt = le port écoute (plus sûr que de chercher un message dans le journal).
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
# pyzmq, msgpack, msgpack-numpy. Constaté (RTX PRO 5000) : le pip d'Isaac Sim
# est cassé (« No module named pip._vendor.idna.core »). On copie alors les
# paquets depuis l'environnement de GR00T, qui les a déjà : même Python 3.12.
importe() { sudo docker exec "$CONTENEUR" /isaac-sim/python.sh -c \
  "import zmq, msgpack, msgpack_numpy" >/dev/null 2>&1; }
if ! importe; then
  sudo docker exec -u root "$CONTENEUR" /isaac-sim/python.sh -m pip install -q \
    pyzmq msgpack msgpack-numpy >/dev/null 2>&1 || true
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
# Durée « 60 » ignorée : l'évaluation s'arrête d'elle-même après le dernier carton.
bash "$HOME/AXIS-OS/deploy/machine_gpu/lancer_t1.sh" 60 "$ACC" \
  --evaluer-vla "tcp://127.0.0.1:$PORT" --episodes "$CARTONS" "$@" || true
sudo docker cp "$CONTENEUR:/isaac-sim/videos/eval_vla.json" "$HOME/videos/eval_vla.json" 2>/dev/null \
  && echo "Détail essai par essai : ~/videos/eval_vla.json"
pkill -f run_gr00t_server || true
echo "Serveur GR00T arrêté. Pense à sauvegarder le modèle (sauvegarder_modele_vla.sh) avant Destroy."
