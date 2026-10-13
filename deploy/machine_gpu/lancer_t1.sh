#!/usr/bin/env bash
# Démonstration « T1 » : tri de colis autonome par le robot humanoïde, en
# présence de personnes, simulé dans Isaac Sim (le simulateur de robotique
# de NVIDIA) sur la machine GPU louée. Le test lui-même est
# sim/validation/test_isaac_t1_tri_usine.py, copié dans le conteneur par
# copier_code.sh ; la démonstration est décrite dans docs/demo_tri_usine.md.
#
#   bash lancer_t1.sh [durée_simulée_s] [accélération_vidéo] [options du test]
#   Premier essai :  bash lancer_t1.sh 240 1       (~4 min simulées, vidéo temps réel)
#   Démo complète :  bash lancer_t1.sh 1800 10     (30 min -> vidéo de 3 min)
#
# Suivi en direct : depuis TON ordinateur, ouvre un tunnel SSH (une connexion
# chiffrée qui amène la page du test sur ton poste sans l'exposer sur
# Internet). Reprends la commande SSH que donne Vast.ai (le service de
# location de machines GPU) en ajoutant « -L 8080:localhost:8080 », par
# exemple (port et adresse fictifs, à remplacer par ceux de ta machine)
#     ssh -p 12345 root@1.2.3.4 -L 8080:localhost:8080
# puis ouvre http://localhost:8080 : image en direct, journal, bouton
# « Arrêter proprement » (la vidéo et le bilan sont quand même écrits).
set -euo pipefail
CONTENEUR="isim-isaac-sim-1"
DUREE="${1:-240}"; ACC="${2:-1}"; shift $(( $# > 2 ? 2 : $# )) || true
# Piège constaté : un Ctrl+C sur « docker exec » coupe l'AFFICHAGE, pas la
# simulation dans le conteneur Docker. D'anciens tests continuaient donc de
# tourner, se partageaient la carte graphique et occupaient le port du suivi
# en direct. On les arrête avant d'en lancer un nouveau.
if sudo docker exec "$CONTENEUR" pgrep -f test_isaac_ >/dev/null 2>&1; then
  echo "Arrêt des anciens tests encore en cours dans le conteneur…"
  sudo docker exec "$CONTENEUR" pkill -f test_isaac_ || true
  sleep 5
fi
# La carte graphique doit être visible DANS le conteneur. Constaté deux
# fois : après une nuit ou un redémarrage de la machine, le conteneur l'avait
# perdue ; le test tournait alors sur le processeur, caméras noires, et le
# journal annonçait « CHUTE à 0 s ». On vérifie donc avant de lancer
# (nvidia-smi -L liste les cartes visibles). Sinon, on tente un redémarrage
# du conteneur ; en dernier recours, on s'arrête en expliquant pourquoi.
if ! sudo docker exec "$CONTENEUR" nvidia-smi -L >/dev/null 2>&1; then
  echo "Le conteneur ne voit pas la carte graphique — redémarrage du conteneur…"
  if ! nvidia-smi -L >/dev/null 2>&1; then
    echo "ERREUR : la MACHINE elle-même ne voit plus la carte graphique (nvidia-smi échoue)."
    echo "Redémarre l'instance depuis la page Vast.ai (Reboot), puis relance ce script."
    exit 1
  fi
  sudo docker restart "$CONTENEUR" >/dev/null
  sleep 10
  if ! sudo docker exec "$CONTENEUR" nvidia-smi -L >/dev/null 2>&1; then
    echo "ERREUR : toujours pas de carte graphique dans le conteneur après redémarrage."
    exit 1
  fi
fi
echo "Carte graphique : $(sudo docker exec "$CONTENEUR" nvidia-smi -L | head -1)"
bash "$HOME/AXIS-OS/deploy/machine_gpu/copier_code.sh"
mkdir -p "$HOME/videos"
# Le test tourne DÉTACHÉ dans le conteneur (docker exec -d). Constaté
# auparavant : un essai de 30 min perdu sur « connexion terminée », journal
# et vidéo jamais récupérés. Désormais, une connexion SSH qui se coupe
# (onglet fermé, ordinateur en veille) n'arrête plus rien : ce script ne
# fait que SUIVRE le journal. Si le suivi se coupe :
#   bash ~/AXIS-OS/deploy/machine_gpu/suivre_t1.sh      reprend l'affichage
#   bash ~/AXIS-OS/deploy/machine_gpu/recuperer_t1.sh   récupère vidéo et journal
# t1.log reçoit toute la sortie du test ; t1.fini est créé à sa fin, ce qui
# permet à suivre_t1.sh et recuperer_t1.sh de savoir qu'il est terminé. On
# efface d'abord ceux de l'essai précédent.
sudo docker exec "$CONTENEUR" rm -f /isaac-sim/videos/t1.log /isaac-sim/videos/t1.fini
ARGS="--duree $DUREE --acceleration $ACC $*"
COMMANDE="mkdir -p /isaac-sim/videos && /isaac-sim/python.sh -u /isaac-sim/axis_os/test_isaac_t1_tri_usine.py $ARGS \
   > /isaac-sim/videos/t1.log 2>&1; touch /isaac-sim/videos/t1.fini"
if [ -n "${GR00T_API_TOKEN:-}" ]; then
  # Jeton d'API GR00T (facultatif, voir evaluer_vla.sh) : le secret que le
  # test doit présenter au serveur GR00T, qui charge le modèle VLA
  # (« vision-langage-action »). Il est transmis au test par l'ENTRÉE
  # STANDARD (docker lit un fichier d'environnement sur /dev/stdin), et par
  # rien d'autre.
  # Pourquoi (relevé lors d'un audit de sécurité du dépôt, le 2026-10-10, et
  # vérifié) : « -e NOM=valeur » met le jeton dans ps (la liste des
  # processus) et dans le journal de sudo, et « sudo --preserve-env=NOM »
  # l'écrit aussi dans ce journal (« ENV=NOM=valeur »). printf, lui, est une
  # commande interne de bash : il n'apparaît pas dans ps. Quant à sudo, il ne
  # journalise que la commande ; le jeton le traverse par le tube (|) sans y
  # être écrit, même avec son option use_pty (qui fait tourner la commande
  # dans un pseudo-terminal).
  printf 'GR00T_API_TOKEN=%s\n' "$GR00T_API_TOKEN" \
    | sudo docker exec -d --env-file /dev/stdin "$CONTENEUR" bash -c "$COMMANDE"
else
  sudo docker exec -d "$CONTENEUR" bash -c "$COMMANDE"
fi
echo "Test lancé en arrière-plan (il continue même si tu te déconnectes)."
echo "Suivi en direct : http://localhost:8080 (via le tunnel SSH)"
sleep 3
bash "$HOME/AXIS-OS/deploy/machine_gpu/suivre_t1.sh"
bash "$HOME/AXIS-OS/deploy/machine_gpu/recuperer_t1.sh"
