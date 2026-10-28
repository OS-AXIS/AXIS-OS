#!/usr/bin/env bash
# =============================================================================
# AXIS-OS — installation d'une machine GPU neuve pour les tests Isaac Sim.
#
# Isaac Sim est le simulateur robotique de NVIDIA. Il tourne ici dans un
# conteneur Docker (un environnement isolé, livré sous forme d'image prête à
# l'emploi). Ce script recrée à l'identique l'environnement validé sur
# l'instance AWS (machine louée chez Amazon Web Services) lors des quatre
# séries d'essais où le G1 prend et porte un carton
# (sim/validation/test_isaac_w1_gr00t_port.py à test_isaac_w4_gr00t_autonome.py,
# voir docs/validation_port_carton.md) : même image (Isaac Sim 6.1), même nom
# de conteneur (isim-isaac-sim-1), mêmes chemins dans le conteneur. Il est
# idempotent : on peut le relancer sans risque, il reprend où il en était.
#
# Prévu pour Ubuntu 22.04 avec le pilote NVIDIA déjà installé (modèle de VM,
# ou machine virtuelle, proposé par Vast.ai, un service de location de
# machines GPU). Usage, sur la machine :
#
#     bash installer.sh
#
# Le dépôt AXIS-OS était privé quand ce script a été écrit : s'il n'est pas
# déjà cloné, le script demande donc une connexion GitHub (gh auth login, par
# le navigateur), puis le clone avec gh, le client GitHub en ligne de
# commande.
# =============================================================================
set -euo pipefail

IMAGE="nvcr.io/nvidia/isaac-sim:6.1.0"
CONTENEUR="isim-isaac-sim-1"
DEPOT="$HOME/AXIS-OS"
CACHE="$HOME/docker/isaac-sim"
# Affiche le titre d'une étape, en gras et en bleu.
etape() { printf '\n\033[1;34m== %s ==\033[0m\n' "$*"; }
# sudo partout, y compris pour docker : l'ajout de l'utilisateur au groupe
# docker (étape 2) ne prend effet qu'à la session suivante. Constaté sur une
# VM Vast.ai neuve : sans sudo, docker échoue tant qu'on ne s'est pas
# reconnecté. Ce choix a été réexaminé et maintenu lors d'un audit de
# sécurité des scripts (2026-10-10).
# « dans » lance une commande dans le conteneur ; « dans_root », en root.
dans() { sudo docker exec "$CONTENEUR" "$@"; }
dans_root() { sudo docker exec -u root "$CONTENEUR" "$@"; }

etape "1/9 GPU et pilote"
# Mises à jour automatiques d'Ubuntu coupées, parce qu'elles ont déjà privé
# la machine de son GPU. Constaté sur Vast.ai : pendant la nuit, Ubuntu a mis
# à jour la bibliothèque du pilote NVIDIA mais pas son module dans le noyau
# (« Driver/library version mismatch ») ; plus aucun GPU n'était visible, ni
# sur la machine ni dans le conteneur, jusqu'au redémarrage.
sudo systemctl disable --now unattended-upgrades apt-daily.timer apt-daily-upgrade.timer \
  >/dev/null 2>&1 || true
# Une mise à jour déjà en cours garde le verrou d'apt, le gestionnaire de
# paquets d'Ubuntu (constaté sur une machine neuve : « Could not get lock …
# held by unattended-upgr »). On attend qu'elle finisse plutôt que de
# l'interrompre, ce qui pourrait laisser des paquets à moitié installés.
while pgrep -f unattended-upgr >/dev/null 2>&1 \
      || sudo fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1; do
  echo "  mise à jour automatique d'Ubuntu en cours — attente (10 s)…"
  sleep 10
done
# Si cette mise à jour a touché le pilote NVIDIA, nvidia-smi (l'outil de
# diagnostic du pilote) ne voit plus le GPU : il faut redémarrer la machine.
if ! nvidia-smi -L >/dev/null 2>&1; then
  echo "Le pilote NVIDIA vient d'être mis à jour : tape « reboot », reconnecte-toi"
  echo "dans 2-3 minutes, puis relance ce script (il reprend où il en était)."
  exit 1
fi
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader \
  || { echo "ÉCHEC : pas de GPU NVIDIA visible (nvidia-smi)."; exit 1; }

etape "2/9 Docker + NVIDIA Container Toolkit"
# Noyau Linux figé (apt-mark hold exclut ces paquets des mises à jour), car
# un nouveau noyau peut rendre le GPU inutilisable. Constaté sur une machine
# à carte RTX PRO 5000 : la mise à jour automatique installait un nouveau
# noyau (5.15.0-1109-kvm) pour lequel le pilote NVIDIA ne se compile pas ;
# dpkg (l'outil d'installation des paquets) finissait en erreur et, au
# redémarrage, il n'y avait plus de GPU.
for p_ in linux-kvm linux-image-kvm linux-headers-kvm linux-generic \
          linux-image-generic linux-headers-generic; do
  dpkg -s "$p_" >/dev/null 2>&1 && sudo apt-mark hold "$p_" >/dev/null 2>&1 || true
done
# Constaté sur la même machine : une mise à jour automatique a repris le
# verrou d'apt après l'attente de l'étape 1. On attend donc à nouveau, juste
# avant d'installer.
while pgrep -f unattended-upgr >/dev/null 2>&1 \
      || sudo fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1; do
  echo "  mise à jour automatique d'Ubuntu en cours — attente (10 s)…"
  sleep 10
done
# Docker est installé par get.docker.com, choix réexaminé et maintenu en
# connaissance de cause lors d'un audit de sécurité (2026-10-10) :
# c'est le script d'installation OFFICIEL de Docker, servi en HTTPS, qui
# configure le dépôt apt signé de Docker. Il ne publie ni version figée ni
# empreinte (somme de contrôle du contenu), on ne peut donc pas le vérifier
# comme l'installeur de uv, le gestionnaire de paquets Python (voir
# entrainer_vla.sh). L'autre voie serait le paquet docker.io d'Ubuntu 22.04,
# mais ce n'est pas celle qui a été validée sur Vast.ai.
if ! command -v docker >/dev/null; then
  curl -fsSL https://get.docker.com | sudo sh
fi
# NVIDIA Container Toolkit : le composant qui donne aux conteneurs Docker
# l'accès au GPU. Pour savoir s'il fonctionne, on teste vraiment cet accès
# depuis un conteneur ; chercher le mot « nvidia » dans `docker info` ne
# suffit pas : sur une VM Vast.ai, le mot y figurait alors que Docker ne
# pouvait pas utiliser le GPU (« could not select device driver "" with
# capabilities: [[gpu]] »). En cas d'échec, on installe le toolkit depuis le
# dépôt apt signé de NVIDIA, puis on refait le test.
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
# On ajoute l'utilisateur au groupe docker (ce n'est effectif qu'à la session
# suivante, d'où les sudo), puis on installe Git LFS pour les politiques de
# l'étape 6 et gh pour le clonage de l'étape 3.
sudo usermod -aG docker "$USER" 2>/dev/null || true
command -v git-lfs >/dev/null || sudo apt-get install -y -qq git-lfs
# ffmpeg : la version réduite de la vidéo (recuperer_t1.sh).
command -v ffmpeg >/dev/null || sudo apt-get install -y -qq ffmpeg
# gh récent, depuis le dépôt apt officiel de GitHub (cli.github.com). Le gh
# d'Ubuntu 22.04 (2.4) ne retrouve pas les releases brouillons (download,
# upload) et ne connaît pas --git-protocol (constaté le 2026-10-28). La clé
# du dépôt n'est installée que si son empreinte est l'une de celles publiées
# par GitHub (docs/install_linux.md du dépôt cli/cli), vérifiées le
# 2026-10-28 : la clé en service depuis 2026 et l'ancienne, expirée.
GH_EMPREINTES="7F38BBB59D064DBCB3D84D725612B36462313325 2C6106201985B60E6C7AC87323F3D4EA75716059"
gh_recent() {
  command -v gh >/dev/null \
    && dpkg --compare-versions "$(gh --version | head -1 | awk '{print $3}')" ge 2.40
}
if ! gh_recent; then
  CLE=$(mktemp)
  curl -fsSL -o "$CLE" https://cli.github.com/packages/githubcli-archive-keyring.gpg
  # Empreinte de chaque clé principale du trousseau : toutes doivent être
  # connues, sinon on n'installe rien.
  TROUVEES=$(gpg --show-keys --with-colons "$CLE" 2>/dev/null \
    | awk -F: '$1 == "pub" {p = 1; next} $1 == "fpr" && p {print $10; p = 0}')
  [ -n "$TROUVEES" ] || { echo "ÉCHEC : clé du dépôt de gh illisible."; exit 1; }
  for f_ in $TROUVEES; do
    case " $GH_EMPREINTES " in
      *" $f_ "*) ;;
      *) echo "ÉCHEC : clé inattendue pour le dépôt de gh ($f_). Vérifier sur"
         echo "  https://github.com/cli/cli/blob/trunk/docs/install_linux.md"; exit 1 ;;
    esac
  done
  sudo install -d -m 755 /etc/apt/keyrings
  sudo install -m 644 "$CLE" /etc/apt/keyrings/githubcli-archive-keyring.gpg
  rm -f "$CLE"
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" \
    | sudo tee /etc/apt/sources.list.d/github-cli.list >/dev/null
  sudo apt-get update -qq && sudo apt-get install -y -qq gh
  gh_recent || { echo "ÉCHEC : gh récent non installé ($(gh --version 2>/dev/null | head -1))."; exit 1; }
fi

etape "3/9 Dépôt AXIS-OS"
if [ ! -d "$DEPOT/.git" ]; then
  gh auth status >/dev/null 2>&1 || gh auth login --web --git-protocol https
  gh repo clone OS-AXIS/AXIS-OS "$DEPOT"
else
  git -C "$DEPOT" pull --ff-only origin main
fi
# Sans cela, git redemande un nom d'utilisateur à chaque pull (constaté) :
# gh devient le fournisseur d'identifiants de git.
gh auth setup-git 2>/dev/null || true

etape "4/9 Image Isaac Sim 6.1 (plusieurs Go, patience)"
sudo docker pull "$IMAGE"

etape "5/9 Conteneur $CONTENEUR"
# Les caches d'Isaac Sim restent sur la machine, montés dans le conteneur :
# ils survivent ainsi à sa recréation. Ils sont donnés à l'utilisateur 1234,
# celui sous lequel tournent les commandes du conteneur hors root.
mkdir -p "$CACHE"/{main,computecache,logs,data,pkg} "$HOME/videos"
sudo chown -R 1234:1234 "$CACHE" 2>/dev/null || true
# Le conteneur n'est créé que s'il ne tourne pas déjà ; un conteneur arrêté
# du même nom est d'abord supprimé.
if ! sudo docker ps --format '{{.Names}}' | grep -qx "$CONTENEUR"; then
  sudo docker rm -f "$CONTENEUR" 2>/dev/null || true
  # --network=host GARDÉ, pour que le conteneur partage le réseau de la
  # machine : le test de la démonstration de tri de colis (lancer_t1.sh) y
  # joint le serveur GR00T (le modèle d'IA de NVIDIA pour humanoïdes), lancé
  # hors du conteneur, sur 127.0.0.1:5555, et le suivi en direct
  # (127.0.0.1:8080) est atteint par le tunnel SSH de la machine (la
  # connexion à distance chiffrée relaie ce port jusqu'à ton ordinateur, voir
  # lancer_t1.sh). Contrepartie : tout port ouvert dans le conteneur l'est
  # aussi sur la machine. Nos serveurs n'écoutent donc que sur 127.0.0.1
  # (joignable depuis la machine seule) ; ne jamais en lancer un sur 0.0.0.0
  # (toutes les interfaces réseau, y compris l'interface publique).
  sudo docker run -d --name "$CONTENEUR" --gpus all --network=host \
    -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y \
    -v "$CACHE/main:/isaac-sim/.cache:rw" \
    -v "$CACHE/computecache:/isaac-sim/.nv/ComputeCache:rw" \
    -v "$CACHE/logs:/isaac-sim/.nvidia-omniverse/logs:rw" \
    -v "$CACHE/data:/isaac-sim/.local/share/ov/data:rw" \
    -v "$CACHE/pkg:/isaac-sim/.local/share/ov/pkg:rw" \
    --entrypoint bash "$IMAGE" -c "sleep infinity"
fi
# Dossiers de travail dans le conteneur : politiques de contrôle, maillages
# 3D du robot, vidéos. Le dossier des vidéos est ouvert en écriture à tous,
# car les tests, qui ne tournent pas en root, y écrivent.
dans_root mkdir -p /isaac-sim/policies /isaac-sim/meshes /isaac-sim/videos
dans_root chmod 777 /isaac-sim/videos

etape "6/9 Politiques GR00T WBC (Git LFS)"
# Politiques GR00T WBC (Whole-Body Control, contrôle du corps entier) : les
# réseaux de neurones de NVIDIA qui gèrent l'équilibre et la marche du G1,
# l'humanoïde d'Unitree, au format ONNX (format standard d'échange de
# modèles). Le dépôt de NVIDIA les stocke avec Git LFS, l'extension de git
# pour les gros fichiers : on ne télécharge que les .onnx du G1.
if [ ! -d "$HOME/gr00t_wbc" ]; then
  git clone --depth 1 https://github.com/NVlabs/GR00T-WholeBodyControl.git "$HOME/gr00t_wbc"
fi
( cd "$HOME/gr00t_wbc" && git lfs install --local \
  && git lfs pull --include "decoupled_wbc/sim2mujoco/resources/robots/g1/policy/*.onnx" )
POL="$HOME/gr00t_wbc/decoupled_wbc/sim2mujoco/resources/robots/g1/policy"
sudo docker cp "$POL/GR00T-WholeBodyControl-Balance.onnx" "$CONTENEUR:/isaac-sim/policies/gr00t_wbc_balance.onnx"
sudo docker cp "$POL/GR00T-WholeBodyControl-Walk.onnx"    "$CONTENEUR:/isaac-sim/policies/gr00t_wbc_walk.onnx"
# Empreintes SHA-256 affichées pour contrôle : elles doivent correspondre à
# GR00T_WBC_SHA256 (axis_os/hal/drivers/unitree_g1/gr00t_wbc_policy.py), les
# empreintes des politiques validées ; un fichier différent y est refusé au
# chargement.
sha256sum "$POL"/*.onnx

etape "7/9 Meshes du G1 (unitree_ros, BSD-3)"
# Maillages 3D (meshes, fichiers STL) du G1, pris dans le dépôt officiel
# unitree_ros (licence BSD-3). Clone partiel : seul le dossier des maillages
# est téléchargé, pas tout le dépôt.
if [ ! -d "$HOME/unitree_ros" ]; then
  git clone --depth 1 --filter=blob:none --sparse \
    https://github.com/unitreerobotics/unitree_ros.git "$HOME/unitree_ros"
  git -C "$HOME/unitree_ros" sparse-checkout set robots/g1_description/meshes
fi
for f in "$HOME"/unitree_ros/robots/g1_description/meshes/*.STL; do
  sudo docker cp "$f" "$CONTENEUR:/isaac-sim/meshes/"
done

etape "8/9 Python d'Isaac Sim : ONNX Runtime GPU, OWL-ViT"
# PyTorch n'est pas fourni dans le Python de l'image Isaac Sim 6.1 : sans
# lui, OWL-ViT (le modèle de détection d'objets utilisé par la perception
# d'AXIS-OS) ne se charge pas (« OwlViTForObjectDetection requires the
# PyTorch library »). Les roues (paquets Python précompilés) pour CUDA 12.8,
# la plateforme de calcul GPU de NVIDIA, sont compatibles avec les pilotes
# récents.
# torch NON épinglé, volontairement (épingler, c'est imposer une version
# exacte) : la bonne version dépend du pilote et de CUDA de la carte louée
# (index cu128), et aucune version n'a été validée nommément. La version
# installée est affichée juste après : c'est elle qu'il faudra épingler une
# fois validée sur GPU.
dans /isaac-sim/python.sh -c "import torch" 2>/dev/null \
  || dans_root /isaac-sim/python.sh -m pip install -q torch torchvision \
       --index-url https://download.pytorch.org/whl/cu128
dans /isaac-sim/python.sh -c "import torch; print('PyTorch', torch.__version__, '- GPU :', torch.cuda.is_available())"
# imageio-ffmpeg : encodeur des vidéos MP4, et donc des images du suivi en
# direct. Il manque dans l'image : « Could not find a backend to open .mp4 ».
# transformers, la bibliothèque de Hugging Face qui fait tourner OWL-ViT, est
# épinglé à la même version que dans requirements.txt (vérifié par
# test/test_deploiement.py), parce que c'est elle qui a cassé la perception
# en passant en version 5.x : modèle chargé, inférence (calcul du modèle sur
# une image) réussie, mais zéro détection. Tant qu'elle n'était pas épinglée
# ici, chaque nouvelle machine prenait la version du jour, malgré
# l'épinglage de requirements.txt.
# Les versions sont épinglées quand c'est possible sans risque, depuis un
# audit de sécurité (2026-10-10) :
#   - transformers, imageio, imageio-ffmpeg : versions exactes (pour le
#     Python 3.12 d'Isaac Sim ; paquets purement Python ou validés) ;
#   - pillow : plancher (version minimale) de requirements.txt seulement, car
#     Isaac Sim fournit déjà le sien et une version exacte différente le
#     remplacerait sous ses pieds ;
#   - onnxruntime-gpu : NON épinglé, sa version doit suivre CUDA et cuDNN (la
#     bibliothèque de réseaux de neurones de NVIDIA) de la carte ; la version
#     installée est affichée ci-dessous pour l'épingler une fois validée sur
#     GPU. ONNX Runtime est le moteur qui exécute les politiques ONNX.
dans_root /isaac-sim/python.sh -m pip install -q onnxruntime-gpu "transformers==5.17.0" \
  "pillow>=10.0.0" "imageio[ffmpeg]==2.37.4" "imageio-ffmpeg==0.6.0"
dans /isaac-sim/python.sh -c "import onnxruntime, transformers, imageio; \
print('Versions : onnxruntime', onnxruntime.__version__, '- transformers', transformers.__version__, \
'- imageio', imageio.__version__)" || true
# Pré-télécharge OWL-ViT maintenant (sinon, ce serait au premier test), à la
# révision épinglée (une version précise du modèle sur Hugging Face) et au
# format safetensors (un format de poids qui, à la différence de pickle, le
# format de sérialisation de Python, n'exécute aucun code au chargement) :
# exactement ce que charge OwlViTDetector (OWLVIT_REVISION dans
# axis_os/services/perception/owlvit_detector.py ; même valeur, vérifiée par
# test/test_deploiement.py). Sans révision fixée, on téléchargeait la version
# du jour, que le détecteur refuse désormais.
OWLVIT_REVISION="85fa9bec488a03a73b7f29206b1550551d0a7556"
dans /isaac-sim/python.sh -c "from transformers import OwlViTProcessor, OwlViTForObjectDetection as M; \
r = '$OWLVIT_REVISION'; OwlViTProcessor.from_pretrained('google/owlvit-base-patch32', revision=r); \
M.from_pretrained('google/owlvit-base-patch32', revision=r, use_safetensors=True); print('OWL-ViT prêt (révision', r, ')')"

etape "9/9 Code AXIS-OS dans le conteneur"
# On passe par le même script que pour les mises à jour ultérieures du code.
bash "$DEPOT/deploy/machine_gpu/copier_code.sh"

echo
echo "Installation terminée. Premier essai :"
echo "    bash $DEPOT/deploy/machine_gpu/lancer_t1.sh 240 1"
