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
dans() { sudo docker exec "$CONTENEUR" "$@"; }
dans_root() { sudo docker exec -u root "$CONTENEUR" "$@"; }

etape "1/9 GPU et pilote"
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader \
  || { echo "ÉCHEC : pas de GPU NVIDIA visible (nvidia-smi)."; exit 1; }

etape "2/9 Docker + NVIDIA Container Toolkit"
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
  gh auth status >/dev/null 2>&1 || gh auth login --web --git-protocol https
  gh repo clone OS-AXIS/AXIS-OS "$DEPOT"
else
  git -C "$DEPOT" pull --ff-only origin main
fi

etape "4/9 Image Isaac Sim 6.1 (plusieurs Go, patience)"
sudo docker pull "$IMAGE"

etape "5/9 Conteneur $CONTENEUR"
mkdir -p "$CACHE"/{main,computecache,logs,data,pkg} "$HOME/videos"
sudo chown -R 1234:1234 "$CACHE" 2>/dev/null || true
if ! sudo docker ps --format '{{.Names}}' | grep -qx "$CONTENEUR"; then
  sudo docker rm -f "$CONTENEUR" 2>/dev/null || true
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
dans_root /isaac-sim/python.sh -m pip install -q onnxruntime-gpu transformers pillow
# Pré-télécharge OWL-ViT (sinon au premier test)
dans /isaac-sim/python.sh -c "from transformers import OwlViTProcessor, OwlViTForObjectDetection as M; \
OwlViTProcessor.from_pretrained('google/owlvit-base-patch32'); M.from_pretrained('google/owlvit-base-patch32'); print('OWL-ViT prêt')"

etape "9/9 Code AXIS-OS dans le conteneur"
bash "$DEPOT/deploy/machine_gpu/copier_code.sh"

echo
echo "Installation terminée. Premier essai :"
echo "    bash $DEPOT/deploy/machine_gpu/lancer_t1.sh 240 1"
