#!/usr/bin/env bash
# Entraînement du VLA d'AXIS-OS sur la machine GPU. Un VLA (modèle
# Vision-Language-Action) reçoit une image, une consigne en langage naturel
# et l'état du robot, et propose des mouvements ; NVIDIA Isaac GR00T N1.7
# est celui de NVIDIA pour les humanoïdes. Le script en fait l'affinage
# (fine-tuning : poursuivre l'entraînement d'un modèle déjà entraîné, sur nos
# propres données) à partir des démonstrations collectées en simulation
# (collecte_vla.sh, puis recuperer_vla.sh).
#
#   bash entrainer_vla.sh [pas_d_entrainement] [taille_de_lot] [prise]
#   (2000 pas et lots de 32 par défaut)
#
#   Un pas d'entraînement traite un lot d'exemples ; la taille de lot est le
#   nombre d'exemples par pas.
#   « prise » : modèle de prise seule (bras, base immobile), entraîné sur la
#   fin de chaque démonstration ; jeu de données ~/vla_lerobot_prise, modèle
#   ~/vla_gr00t_g1_prise.
#
# Prérequis :
#   - une carte graphique d'au moins 40 Go de mémoire (RTX PRO 6000 : 96 Go) ;
#   - un compte Hugging Face (plateforme d'hébergement de modèles d'IA), la
#     licence des modèles NVIDIA acceptée sur huggingface.co
#     (nvidia/GR00T-N1.7-3B et nvidia/Cosmos-Reason2-2B), et un jeton
#     d'accès : le script le demande au premier lancement.
#
# Le script tourne hors du conteneur Isaac Sim (le simulateur de NVIDIA),
# dans un environnement Python 3.12 à part, géré par uv (gestionnaire de
# paquets et d'environnements Python). L'entraînement est lancé en
# arrière-plan : une déconnexion ne l'arrête pas.
set -euo pipefail
PAS="${1:-2000}"
# Taille de lot : 32 validé sur RTX PRO 6000 (96 Go). Sur une carte de
# 48 Go, si le journal affiche « CUDA out of memory » (mémoire du GPU
# saturée), relancer avec 16.
LOT="${2:-32}"
# Jeu de données au format LeRobot v2 (le format de jeux de données de la
# bibliothèque LeRobot, celui qu'attend l'entraînement de GR00T), produit par
# recuperer_vla.sh, puis dossier où sera écrit le modèle entraîné.
JEU="$HOME/vla_lerobot"
SORTIE="$HOME/vla_gr00t_g1"
if [ "${3:-}" = "prise" ]; then
  JEU="$HOME/vla_lerobot_prise"
  SORTIE="$HOME/vla_gr00t_g1_prise"
  # Le jeu de prise est toujours reconverti depuis les épisodes (les
  # démonstrations enregistrées, copiées par recuperer_vla.sh), ce qui prend
  # quelques secondes, car le découpage (la part de chaque démonstration
  # retenue) a évolué : on ne se fie pas à une conversion antérieure.
  # L'ancien modèle de prise est effacé, car il pèse plusieurs dizaines de Go
  # (disque plein constaté).
  [ -d "$HOME/vla_episodes" ] || { echo "Pas d'épisodes : lancer recuperer_vla.sh d'abord."; exit 1; }
  rm -rf "$JEU" "$SORTIE"
  (cd "$HOME/AXIS-OS" && python3 -m axis_os.vla.vers_lerobot "$HOME/vla_episodes" "$JEU" --prise-seule)
fi
[ -d "$JEU" ] || { echo "Pas de jeu d'entraînement ($JEU) : lancer recuperer_vla.sh d'abord."; exit 1; }
# ffmpeg, l'outil de traitement vidéo, est nécessaire à la suite.
command -v ffmpeg >/dev/null || sudo apt-get install -y -qq ffmpeg
# DeepSpeed (bibliothèque d'entraînement de grands modèles, utilisée par
# l'entraîneur de GR00T) exige nvcc, le compilateur de CUDA (la plateforme
# de calcul GPU de NVIDIA), même sans rien compiler : sinon, erreur
# « CUDA_HOME does not exist » (constaté). La VM (machine virtuelle) louée
# sur Vast.ai n'a que le pilote : on installe donc la boîte à outils
# CUDA 12.8, depuis le dépôt apt (apt : le gestionnaire de paquets
# d'Ubuntu) que NVIDIA tient pour chaque version d'Ubuntu.
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
# uv : version épinglée (imposée exactement) et installeur vérifié avant
# exécution, depuis un audit de sécurité (2026-10-10). L'ancienne méthode,
# « curl … | sh », exécutait sans contrôle la version du jour d'un script
# téléchargé, dont le contenu peut changer sans que ce dépôt bouge. Ici,
# l'installeur de la version 0.12.19 (publiée le 2026-09-25), pris sur la
# page de publication GitHub d'astral-sh/uv, n'est lancé que si son
# empreinte SHA-256 (somme de contrôle du contenu exact) est celle relevée
# lors de l'épinglage. Pour monter de version, changer les deux lignes
# ensemble (sha256sum de l'installeur de la nouvelle version).
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
# Code d'entraînement de GR00T (dépôt Isaac-GR00T de NVIDIA) ; uv installe
# ses dépendances dans son propre environnement Python 3.12.
if [ ! -d "$HOME/Isaac-GR00T" ]; then
  git clone --recurse-submodules https://github.com/NVIDIA/Isaac-GR00T "$HOME/Isaac-GR00T"
fi
cd "$HOME/Isaac-GR00T"
uv sync --python 3.12
uv run python -c "import gr00t; print('GR00T installé')"
# Connexion à Hugging Face, parce que le modèle de base de GR00T
# (Cosmos-Reason2-2B) est en accès restreint. « huggingface-cli whoami »
# répondait OK sans connexion (constaté) : on teste donc le vrai accès au
# dépôt du modèle, et l'on utilise la commande actuelle « hf auth login »
# (l'ancienne, « huggingface-cli login », sert de repli).
if ! uv run python -c "from huggingface_hub import hf_hub_download; hf_hub_download('nvidia/Cosmos-Reason2-2B', 'config.json')" >/dev/null 2>&1; then
  echo "Connexion à Hugging Face — colle ton jeton d'accès (il ne s'affiche pas) :"
  uv run hf auth login || uv run huggingface-cli login
  uv run python -c "from huggingface_hub import hf_hub_download; hf_hub_download('nvidia/Cosmos-Reason2-2B', 'config.json')" \
    || { echo "Pas d'accès à nvidia/Cosmos-Reason2-2B : accepter la licence sur huggingface.co, puis relancer."; exit 1; }
fi
# Lancement en arrière-plan avec nohup (la commande survit à la
# déconnexion), sur le premier GPU ; tout le journal va dans
# ~/vla_entrainement.log. NEW_EMBODIMENT déclare le G1 comme un corps que
# GR00T ne connaît pas d'avance ; config_gr00t_g1.py lui indique quelles
# données lire (caméra, état des bras) et comment interpréter les actions.
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
