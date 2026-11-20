"""
Profil du robot : ce qui est propre à UN robot, pas au modèle (G1) — ses
calibrations, chacune avec sa date — dans un fichier JSON par robot.

La première est la calibration de charge (gains des coudes, une fois par
robot, à la première mise en route : docs/PREMIERE_MISE_EN_ROUTE.md) ; sans
elle, l'OS le dit et garde son comportement d'avant
(ManipulationService.appliquer_calibration_charge).

Écrit comme la carte du site (navigation/site_map.py) : un fichier temporaire
dans le même dossier, puis os.replace — jamais un profil à moitié écrit.
Illisible, d'un autre format ou d'un autre robot : ProfilRobotErreur ; on ne
devine pas, on ne repart pas d'un profil vide en écrasant l'ancien.
"""
import json
import os
import tempfile
from typing import Any, Dict

FORMAT = "axis-profil-robot"


class ProfilRobotErreur(Exception):
    pass


def lire_profil(chemin: str, robot_id: str) -> Dict[str, Any]:
    """Le profil du robot `robot_id` ; un profil vide s'il n'existe pas
    encore (robot jamais calibré)."""
    if not os.path.exists(chemin):
        return {"format": FORMAT, "robot": robot_id}
    try:
        with open(chemin, encoding="utf-8") as f:
            doc = json.load(f)
    except (OSError, ValueError) as e:
        raise ProfilRobotErreur(f"profil du robot illisible ({chemin}) : {e}") from e
    if not isinstance(doc, dict) or doc.get("format") != FORMAT:
        raise ProfilRobotErreur(f"{chemin} : pas un profil de robot AXIS-OS")
    if doc.get("robot") != robot_id:
        raise ProfilRobotErreur(f"{chemin} : profil du robot {doc.get('robot')!r}, pas de "
                                f"{robot_id!r}")
    return doc


def enregistrer(chemin: str, robot_id: str, nom: str, donnees: Dict[str, Any]) -> Dict[str, Any]:
    """Ajoute ou remplace la calibration `nom` du profil (`donnees` porte sa
    date), écrit d'un coup ; rend le profil écrit."""
    doc = lire_profil(chemin, robot_id)
    doc[nom] = donnees
    dossier = os.path.dirname(os.path.abspath(chemin))
    os.makedirs(dossier, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=dossier, prefix=".profil_robot.")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(doc, f, ensure_ascii=False, indent=1, sort_keys=True)
        os.replace(tmp, chemin)
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise
    return doc
