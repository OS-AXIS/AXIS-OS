#!/usr/bin/env bash
# Entraînement (fine-tuning) de NVIDIA Isaac GR00T N1.7 sur les démonstrations
# collectées en simulation (collecte_vla.sh puis recuperer_vla.sh).
#
#   bash entrainer_vla.sh [pas_d_entrainement] [taille_de_lot] [prise]   (2000 et 32 par défaut)
#   « prise » : modèle de PRISE SEULE (bras, base immobile), entraîné sur la fin
#   de chaque démonstration — jeu ~/vla_lerobot_prise, modèle ~/vla_gr00t_g1_prise.
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
# Taille de lot : 32 validé sur RTX PRO 6000 (96 Go). Sur une carte de 48 Go,
# si le journal dit « CUDA out of memory », relancer avec 16.
LOT="${2:-32}"
JEU="$HOME/vla_lerobot"
SORTIE="$HOME/vla_gr00t_g1"
if [ "${3:-}" = "prise" ]; then
  JEU="$HOME/vla_lerobot_prise"
  SORTIE="$HOME/vla_gr00t_g1_prise"
  # Toujours reconverti (quelques secondes) : le découpage a évolué. L'ancien
  # modèle de prise est effacé — plusieurs dizaines de Go (disque plein constaté).
  [ -d "$HOME/vla_episodes" ] || { echo "Pas d'épisodes : lancer recuperer_vla.sh d'abord."; exit 1; }
  rm -rf "$JEU" "$SORTIE"
  (cd "$HOME/AXIS-OS" && python3 -m axis_os.vla.vers_lerobot "$HOME/vla_episodes" "$JEU" --prise-seule)
fi
[ -d "$JEU" ] || { echo "Pas de jeu d'entraînement ($JEU) : lancer recuperer_vla.sh d'abord."; exit 1; }
command -v ffmpeg >/dev/null || sudo apt-get install -y -qq ffmpeg
# DeepSpeed (utilisé par l'entraîneur de GR00T) exige le compilateur CUDA
# (nvcc) même sans rien compiler : « CUDA_HOME does not exist » (constaté).
# La VM Vast n'a que le pilote : on installe la boîte à outils CUDA 12.8.
if [ ! -x /usr/local/cuda-12.8/bin/nvcc ]; then
  . /etc/os-release
  DEPOT_CUDA="ubuntu${VERSION_ID/./}"
  wget -q "https://developer.download.nvidia.com/compute/cuda/repos/${DEPOT_CUDA}/x86_64/cuda-keyring_1.1-1_all.deb" -O /tmp/cuda-keyring.deb
  sudo dpkg -i /tmp/cuda-keyring.deb
  sudo apt-get update -qq
  sudo apt-get install -y -qq cuda-toolkit-12-8
fi
export CUDA_HOME=/usr/local/cuda-12.8
export PATH="$CUDA_HOME/bin:$PATH"
# uv : version ÉPINGLÉE et installeur VÉRIFIÉ avant exécution (audit du
# 2026-10-10). « curl … | sh » exécutait la version du jour d'un script
# téléchargé, sans contrôle : le contenu peut changer sans que ce dépôt bouge.
# Ici, l'installeur de la version 0.12.19 (publiée le 2026-09-25), pris sur
# la release GitHub d'astral-sh/uv, n'est lancé que si son empreinte SHA-256
# est celle relevée à l'épinglage. Monter de version : changer les deux
# lignes ensemble (sha256sum de l'installeur de la nouvelle version).
UV_VERSION="0.12.19"
UV_INSTALLEUR_SHA256="61b349611f1b6e1ba33645f30c36da5287df2609dd7af8605d96a031435eb35b"
if ! command -v uv >/dev/null; then
  curl -LsSf "https://github.com/astral-sh/uv/releases/download/$UV_VERSION/uv-installer.sh" \
    -o /tmp/uv-installer.sh
  echo "$UV_INSTALLEUR_SHA256  /tmp/uv-installer.sh" | sha256sum -c --quiet \
    || { echo "ÉCHEC : installeur de uv $UV_VERSION altéré (empreinte différente) — non exécuté."; exit 1; }
  sh /tmp/uv-installer.sh
fi
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
if ! uv run python -c "from huggingface_hub import hf_hub_download; hf_hub_download('nvidia/Cosmos-Reason2-2B', 'config.json')" >/dev/null 2>&1; then
  echo "Connexion à Hugging Face — colle ton jeton d'accès (il ne s'affiche pas) :"
  uv run hf auth login || uv run huggingface-cli login
  uv run python -c "from huggingface_hub import hf_hub_download; hf_hub_download('nvidia/Cosmos-Reason2-2B', 'config.json')" \
    || { echo "Pas d'accès à nvidia/Cosmos-Reason2-2B : accepter la licence sur huggingface.co, puis relancer."; exit 1; }
fi
echo "Entraînement : $PAS pas, journal dans $HOME/vla_entrainement.log"
nohup env CUDA_VISIBLE_DEVICES=0 CUDA_HOME="$CUDA_HOME" uv run python gr00t/experiment/launch_finetune.py \
  --base-model-path nvidia/GR00T-N1.7-3B \
  --dataset-path "$JEU" \
  --embodiment-tag NEW_EMBODIMENT \
  --modality-config-path "$HOME/AXIS-OS/axis_os/vla/config_gr00t_g1.py" \
  --num-gpus 1 --output-dir "$SORTIE" \
  --max-steps "$PAS" --global-batch-size "$LOT" --dataloader-num-workers 4 \
  > "$HOME/vla_entrainement.log" 2>&1 &
echo "Lancé (PID $!). Suivre : tail -f ~/vla_entrainement.log"
