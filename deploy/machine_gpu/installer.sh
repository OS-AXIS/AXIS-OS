#!/usr/bin/env bash
# =============================================================================
# AXIS-OS — installation d'une machine GPU neuve pour les tests Isaac Sim.
#
# Recrée À L'IDENTIQUE l'environnement validé sur l'instance AWS (W1-W4) :
# même image (Isaac Sim 6.1), même nom de conteneur (isim-isaac-sim-1),
# mêmes chemins dans le conteneur. Idempotent : relançable sans risque.
#
# Prévu pour Ubuntu 22.04 avec pilote NVIDIA déjà installé (modèle VM de
# Vast.ai). Usage, sur la machine :
#
#     bash installer.sh
#
# Le dépôt AXIS-OS est privé : le script demande une connexion GitHub
# (gh auth login, par navigateur) s'il n'est pas déjà cloné.
# =============================================================================
set -euo pipefail

IMAGE="nvcr.io/nvidia/isaac-sim:6.1.0"
CONTENEUR="isim-isaac-sim-1"
DEPOT="$HOME/AXIS-OS"
CACHE="$HOME/docker/isaac-sim"
etape() { printf '\n\033[1;34m== %s ==\033[0m\n' "$*"; }
# sudo partout : l'ajout au groupe docker ne vaut qu'à la session suivante.
# (Gardé à l'audit du 2026-10-10 : sur une VM Vast.ai neuve, docker sans sudo
# échoue tant qu'on ne s'est pas reconnecté.)
dans() { sudo docker exec "$CONTENEUR" "$@"; }
dans_root() { sudo docker exec -u root "$CONTENEUR" "$@"; }

etape "1/9 GPU et pilote"
# Mises à jour automatiques COUPÉES. Constaté sur Vast.ai : pendant la nuit,
# Ubuntu a mis à jour la bibliothèque du pilote NVIDIA mais pas le module du
# noyau (« Driver/library version mismatch ») — plus aucun GPU, ni sur la
# machine ni dans le conteneur, jusqu'au redémarrage.
sudo systemctl disable --now unattended-upgrades apt-daily.timer apt-daily-upgrade.timer \
  >/dev/null 2>&1 || true
# Une mise à jour déjà EN COURS garde le verrou d'apt (constaté sur une
# machine neuve : « Could not get lock … held by unattended-upgr »). On attend
# qu'elle finisse — l'interrompre peut laisser les paquets à moitié installés.
while pgrep -f unattended-upgr >/dev/null 2>&1 \
      || sudo fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1; do
  echo "  mise à jour automatique d'Ubuntu en cours — attente (10 s)…"
  sleep 10
done
# Si elle a mis à jour le pilote NVIDIA, il faut redémarrer la machine.
if ! nvidia-smi -L >/dev/null 2>&1; then
  echo "Le pilote NVIDIA vient d'être mis à jour : tape « reboot », reconnecte-toi"
  echo "dans 2-3 minutes, puis relance ce script (il reprend où il en était)."
  exit 1
fi
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader \
  || { echo "ÉCHEC : pas de GPU NVIDIA visible (nvidia-smi)."; exit 1; }

etape "2/9 Docker + NVIDIA Container Toolkit"
# Noyau FIGÉ. Constaté (RTX PRO 5000) : la mise à jour automatique installait
# un nouveau noyau (5.15.0-1109-kvm) pour lequel le pilote NVIDIA ne se
# compile pas — dpkg en erreur, et au redémarrage, plus de GPU.
for p_ in linux-kvm linux-image-kvm linux-headers-kvm linux-generic \
          linux-image-generic linux-headers-generic; do
  dpkg -s "$p_" >/dev/null 2>&1 && sudo apt-mark hold "$p_" >/dev/null 2>&1 || true
done
# Constaté (RTX PRO 5000) : une mise à jour automatique a pris le verrou d'apt
# APRÈS l'attente de l'étape 1 — on attend à nouveau, juste avant d'installer.
while pgrep -f unattended-upgr >/dev/null 2>&1 \
      || sudo fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1; do
  echo "  mise à jour automatique d'Ubuntu en cours — attente (10 s)…"
  sleep 10
done
# get.docker.com GARDÉ, en connaissance de cause (audit du 2026-10-10) :
# c'est le script d'installation OFFICIEL de Docker, en HTTPS, qui configure
# le dépôt apt signé de Docker ; il ne publie ni version figée ni empreinte,
# donc on ne peut pas le vérifier comme celui de uv. Le docker.io d'Ubuntu
# 22.04 serait l'autre voie, mais ce n'est pas celle validée sur Vast.ai.
if ! command -v docker >/dev/null; then
  curl -fsSL https://get.docker.com | sudo sh
fi
# On TESTE l'accès au GPU depuis un conteneur, au lieu de chercher le mot
# « nvidia » dans `docker info` : sur une VM Vast.ai, le mot y figurait
# alors que Docker ne pouvait pas utiliser le GPU (« could not select
# device driver "" with capabilities: [[gpu]] »).
if ! sudo docker run --rm --gpus all ubuntu:22.04 nvidia-smi -L >/dev/null 2>&1; then
  curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
    | sudo gpg --dearmor --yes -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
  curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
    | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
    | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list >/dev/null
  sudo apt-get update -qq && sudo apt-get install -y -qq nvidia-container-toolkit
  sudo nvidia-ctk runtime configure --runtime=docker
  sudo systemctl restart docker
  sudo docker run --rm --gpus all ubuntu:22.04 nvidia-smi -L \
    || { echo "ÉCHEC : Docker ne voit toujours pas le GPU."; exit 1; }
fi
sudo usermod -aG docker "$USER" 2>/dev/null || true
command -v git-lfs >/dev/null || sudo apt-get install -y -qq git-lfs
command -v gh >/dev/null || sudo apt-get install -y -qq gh || true

etape "3/9 Dépôt AXIS-OS"
if [ ! -d "$DEPOT/.git" ]; then
  # Le gh d'Ubuntu 22.04 est ancien : pas d'option --git-protocol (constaté).
  gh auth status >/dev/null 2>&1 || gh auth login --web --git-protocol https \
    || gh auth login --web
  gh repo clone OS-AXIS/AXIS-OS "$DEPOT"
else
  git -C "$DEPOT" pull --ff-only origin main
fi
# Sans ça, git redemande un nom d'utilisateur à chaque pull (constaté).
gh auth setup-git 2>/dev/null || true

etape "4/9 Image Isaac Sim 6.1 (plusieurs Go, patience)"
sudo docker pull "$IMAGE"

etape "5/9 Conteneur $CONTENEUR"
mkdir -p "$CACHE"/{main,computecache,logs,data,pkg} "$HOME/videos"
sudo chown -R 1234:1234 "$CACHE" 2>/dev/null || true
if ! sudo docker ps --format '{{.Names}}' | grep -qx "$CONTENEUR"; then
  sudo docker rm -f "$CONTENEUR" 2>/dev/null || true
  # --network=host GARDÉ : le test T1 joint le serveur GR00T, lancé HORS du
  # conteneur, sur 127.0.0.1:5555, et le suivi en direct (127.0.0.1:8080) est
  # atteint par le tunnel SSH de la machine. Contrepartie : tout port ouvert
  # dans le conteneur l'est sur la machine — nos serveurs n'écoutent que sur
  # 127.0.0.1 ; ne jamais en lancer un sur 0.0.0.0.
  sudo docker run -d --name "$CONTENEUR" --gpus all --network=host \
    -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y \
    -v "$CACHE/main:/isaac-sim/.cache:rw" \
    -v "$CACHE/computecache:/isaac-sim/.nv/ComputeCache:rw" \
    -v "$CACHE/logs:/isaac-sim/.nvidia-omniverse/logs:rw" \
    -v "$CACHE/data:/isaac-sim/.local/share/ov/data:rw" \
    -v "$CACHE/pkg:/isaac-sim/.local/share/ov/pkg:rw" \
    --entrypoint bash "$IMAGE" -c "sleep infinity"
fi
dans_root mkdir -p /isaac-sim/policies /isaac-sim/meshes /isaac-sim/videos
dans_root chmod 777 /isaac-sim/videos

etape "6/9 Politiques GR00T WBC (Git LFS)"
if [ ! -d "$HOME/gr00t_wbc" ]; then
  git clone --depth 1 https://github.com/NVlabs/GR00T-WholeBodyControl.git "$HOME/gr00t_wbc"
fi
( cd "$HOME/gr00t_wbc" && git lfs install --local \
  && git lfs pull --include "decoupled_wbc/sim2mujoco/resources/robots/g1/policy/*.onnx" )
POL="$HOME/gr00t_wbc/decoupled_wbc/sim2mujoco/resources/robots/g1/policy"
sudo docker cp "$POL/GR00T-WholeBodyControl-Balance.onnx" "$CONTENEUR:/isaac-sim/policies/gr00t_wbc_balance.onnx"
sudo docker cp "$POL/GR00T-WholeBodyControl-Walk.onnx"    "$CONTENEUR:/isaac-sim/policies/gr00t_wbc_walk.onnx"
# Empreintes : doivent correspondre à GR00T_WBC_SHA256 (gr00t_wbc_policy.py)
sha256sum "$POL"/*.onnx

etape "7/9 Meshes du G1 (unitree_ros, BSD-3)"
if [ ! -d "$HOME/unitree_ros" ]; then
  git clone --depth 1 --filter=blob:none --sparse \
    https://github.com/unitreerobotics/unitree_ros.git "$HOME/unitree_ros"
  git -C "$HOME/unitree_ros" sparse-checkout set robots/g1_description/meshes
fi
for f in "$HOME"/unitree_ros/robots/g1_description/meshes/*.STL; do
  sudo docker cp "$f" "$CONTENEUR:/isaac-sim/meshes/"
done

etape "8/9 Python d'Isaac Sim : ONNX Runtime GPU, OWL-ViT"
# PyTorch n'est PAS fourni dans le Python de l'image Isaac Sim 6.1 : sans
# lui, OWL-ViT ne se charge pas (« OwlViTForObjectDetection requires the
# PyTorch library »). Roues CUDA 12.8, compatibles avec les pilotes récents.
# torch NON épinglé, volontairement : la bonne version dépend du pilote et de
# CUDA de la carte louée (index cu128), et aucune version n'a été validée
# nommément. La version installée est affichée juste après : c'est elle
# qu'il faudra épingler une fois validée sur GPU.
dans /isaac-sim/python.sh -c "import torch" 2>/dev/null \
  || dans_root /isaac-sim/python.sh -m pip install -q torch torchvision \
       --index-url https://download.pytorch.org/whl/cu128
dans /isaac-sim/python.sh -c "import torch; print('PyTorch', torch.__version__, '- GPU :', torch.cuda.is_available())"
# imageio-ffmpeg : encodeur des vidéos MP4 (et donc des images du suivi en
# direct). Absent de l'image : « Could not find a backend to open .mp4 ».
# transformers ÉPINGLÉ, même version que requirements.txt (vérifié par
# test/test_deploiement.py) : c'est cette bibliothèque qui a cassé la
# perception en passant en 5.x — modèle chargé, inférence réussie, zéro
# détection. Non épinglée ici, chaque nouvelle machine prenait la version du
# jour, malgré l'épinglage de requirements.txt.
# Versions épinglées quand c'est possible sans risque (audit du 2026-10-10) :
#   - transformers, imageio, imageio-ffmpeg : versions exactes (Python 3.12
#     d'Isaac Sim, paquets purement Python ou validés) ;
#   - pillow : plancher de requirements.txt seulement — Isaac Sim fournit déjà
#     le sien, une version exacte différente le remplacerait sous ses pieds ;
#   - onnxruntime-gpu : NON épinglé, sa version doit suivre CUDA/cuDNN de la
#     carte ; la version installée est affichée ci-dessous pour l'épingler
#     une fois validée sur GPU.
dans_root /isaac-sim/python.sh -m pip install -q onnxruntime-gpu "transformers==5.17.0" \
  "pillow>=10.0.0" "imageio[ffmpeg]==2.37.4" "imageio-ffmpeg==0.6.0"
dans /isaac-sim/python.sh -c "import onnxruntime, transformers, imageio; \
print('Versions : onnxruntime', onnxruntime.__version__, '- transformers', transformers.__version__, \
'- imageio', imageio.__version__)" || true
# Pré-télécharge OWL-ViT (sinon au premier test)
dans /isaac-sim/python.sh -c "from transformers import OwlViTProcessor, OwlViTForObjectDetection as M; \
OwlViTProcessor.from_pretrained('google/owlvit-base-patch32'); M.from_pretrained('google/owlvit-base-patch32'); print('OWL-ViT prêt')"

etape "9/9 Code AXIS-OS dans le conteneur"
bash "$DEPOT/deploy/machine_gpu/copier_code.sh"

echo
echo "Installation terminée. Premier essai :"
echo "    bash $DEPOT/deploy/machine_gpu/lancer_t1.sh 240 1"
