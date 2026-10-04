#!/usr/bin/env bash
# Entraînement (fine-tuning) de NVIDIA Isaac GR00T N1.7 sur les démonstrations
# collectées en simulation (collecte_vla.sh puis recuperer_vla.sh).
#
#   bash entrainer_vla.sh [pas_d_entrainement]      (2000 par défaut)
#
# Prérequis :
#   - carte avec au moins 40 Go de mémoire (RTX PRO 6000 : 96 Go) ;
#   - un compte Hugging Face, la licence des modèles NVIDIA acceptée sur
#     huggingface.co (nvidia/GR00T-N1.7-3B et nvidia/Cosmos-Reason2-2B), et un
#     jeton d'accès : le script le demande au premier lancement.
# Tourne HORS du conteneur Isaac Sim (environnement Python 3.12 à part, géré
# par uv), en arrière-plan : une déconnexion ne l'arrête pas.
set -euo pipefail
PAS="${1:-2000}"
JEU="$HOME/vla_lerobot"
SORTIE="$HOME/vla_gr00t_g1"
[ -d "$JEU" ] || { echo "Pas de jeu d'entraînement ($JEU) : lancer recuperer_vla.sh d'abord."; exit 1; }
command -v ffmpeg >/dev/null || sudo apt-get install -y -qq ffmpeg
command -v uv >/dev/null || { curl -LsSf https://astral.sh/uv/install.sh | sh; }
export PATH="$HOME/.local/bin:$PATH"
if [ ! -d "$HOME/Isaac-GR00T" ]; then
  git clone --recurse-submodules https://github.com/NVIDIA/Isaac-GR00T "$HOME/Isaac-GR00T"
fi
cd "$HOME/Isaac-GR00T"
uv sync --python 3.12
uv run python -c "import gr00t; print('GR00T installé')"
# Connexion Hugging Face : le modèle de base de GR00T (Cosmos-Reason2-2B) est
# en accès restreint. « huggingface-cli whoami » répondait OK sans connexion
# (constaté) : on teste le vrai accès au dépôt, et on utilise la commande
# actuelle « hf auth login ».
if ! uv run python -c "from huggingface_hub import HfApi; HfApi().model_info('nvidia/Cosmos-Reason2-2B')" >/dev/null 2>&1; then
  echo "Connexion à Hugging Face — colle ton jeton d'accès (il ne s'affiche pas) :"
  uv run hf auth login || uv run huggingface-cli login
  uv run python -c "from huggingface_hub import HfApi; HfApi().model_info('nvidia/Cosmos-Reason2-2B')" \
    || { echo "Pas d'accès à nvidia/Cosmos-Reason2-2B : accepter la licence sur huggingface.co, puis relancer."; exit 1; }
fi
echo "Entraînement : $PAS pas, journal dans $HOME/vla_entrainement.log"
nohup env CUDA_VISIBLE_DEVICES=0 uv run python gr00t/experiment/launch_finetune.py \
  --base-model-path nvidia/GR00T-N1.7-3B \
  --dataset-path "$JEU" \
  --embodiment-tag NEW_EMBODIMENT \
  --modality-config-path "$HOME/AXIS-OS/axis_os/vla/config_gr00t_g1.py" \
  --num-gpus 1 --output-dir "$SORTIE" \
  --max-steps "$PAS" --global-batch-size 32 --dataloader-num-workers 4 \
  > "$HOME/vla_entrainement.log" 2>&1 &
echo "Lancé (PID $!). Suivre : tail -f ~/vla_entrainement.log"
