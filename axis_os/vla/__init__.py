"""
VLA : collecte de démonstrations, conversion vers le format d'entraînement
de GR00T, et évaluation du modèle entraîné.

Un VLA (Vision-Language-Action) est un modèle appris qui reçoit une image
de caméra, une consigne en langage naturel et l'état du robot, et qui
propose des mouvements. GR00T est le modèle de ce type publié par NVIDIA
pour les robots humanoïdes. On l'entraîne ici par imitation : le robot,
piloté par ses réflexes codés (le comportement programmé à la main), sert
de professeur, et le modèle apprend à refaire ses gestes.

    collecte.py          répète en simulation une prise de carton, pour
                         produire les démonstrations ;
    enregistreur.py      enregistre des épisodes (images de la tête, état
                         et consignes des bras) pendant la simulation ;
    vers_lerobot.py      convertit les épisodes réussis au format LeRobot
                         v2 (format de jeu de données de la bibliothèque
                         LeRobot), attendu par les scripts d'entraînement
                         de GR00T ;
    config_gr00t_g1.py   décrit à GR00T les données du robot (caméra,
                         bras, base) ; lu par l'entraînement, hors
                         d'AXIS-OS ;
    evaluation.py        compare le VLA entraîné aux réflexes codés, sur
                         les mêmes cartons.

Le VLA ne contourne aucune protection : une fois entraîné, il ne fera que
proposer des mouvements. Chaque consigne continue de passer par le
JointSafetyGuard (le garde-fou qui vérifie chaque commande d'articulation
avant les moteurs) et par le gardien de proximité (qui ralentit ou arrête
le robot quand un obstacle, peut-être humain, est trop proche), comme
celles des réflexes actuels.
"""
