"""
VLA (Vision-Language-Action) : collecte de démonstrations et conversion
vers le format d'entraînement de GR00T.

    enregistreur.py   enregistre des épisodes (images de la tête, état et
                      consignes des bras) pendant la simulation ;
    vers_lerobot.py   convertit les épisodes RÉUSSIS au format LeRobot v2
                      attendu par les scripts d'entraînement de GR00T.

Le VLA, une fois entraîné, ne fera que PROPOSER des mouvements : chaque
consigne continue de passer par le JointSafetyGuard et le gardien de
proximité, comme celles des réflexes actuels.
"""
