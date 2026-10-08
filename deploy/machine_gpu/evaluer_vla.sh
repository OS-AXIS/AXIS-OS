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
CARTONS="${1:-20}"; ACC="${2:-2}"; shift $(( $# > 2 ? 2 : $# )) || true
CONTENEUR="isim-isaac-sim-1"
PORT=5555
MODELES="$HOME/vla_gr00t_g1"
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
sudo docker exec -u root "$CONTENEUR" /isaac-sim/python.sh -c "import zmq, msgpack, msgpack_numpy" \
  2>/dev/null || sudo docker exec -u root "$CONTENEUR" /isaac-sim/python.sh -m pip install -q \
  pyzmq msgpack msgpack-numpy

# ---- 3. évaluation ---------------------------------------------------------
# Durée « 60 » ignorée : l'évaluation s'arrête d'elle-même après le dernier carton.
bash "$HOME/AXIS-OS/deploy/machine_gpu/lancer_t1.sh" 60 "$ACC" \
  --evaluer-vla "tcp://127.0.0.1:$PORT" --episodes "$CARTONS" "$@" || true
sudo docker cp "$CONTENEUR:/isaac-sim/videos/eval_vla.json" "$HOME/videos/eval_vla.json" 2>/dev/null \
  && echo "Détail essai par essai : ~/videos/eval_vla.json"
pkill -f run_gr00t_server || true
echo "Serveur GR00T arrêté. Pense à sauvegarder le modèle (sauvegarder_modele_vla.sh) avant Destroy."
