"""
Couples ESTIMÉS, comme le vrai G1 les donne : ses moteurs n'ont pas de
capteur de couple, le couple est estimé du courant (tau_est). On ajoute au
couple mesuré par la physique (source_couples) les erreurs d'une telle
estimation, choisies prudentes (plutôt plus que moins) :
  - gain : constante de couple propre à chaque moteur, ±5 % ;
  - biais : frottement du réducteur et décalage du courant, constant par
    articulation, ±0,5 N·m ;
  - bruit blanc : mesure du courant, écart type 0,3 N·m à chaque lecture.
Ordre de grandeur pour la prise : la hausse de couple qui signe un contact
(1,5 N·m sur 2 lectures de suite) est une différence de deux lectures ; gain
et biais s'y annulent, le bruit y vaut 0,42 N·m d'écart type, soit environ
1e-7 de chances d'un faux contact sur deux lectures.
"""
from __future__ import annotations

import numpy as np


class BruitCouples:
    def __init__(self, n: int, graine: int = 0, sigma_nm: float = 0.3,
                 biais_max_nm: float = 0.5, gain_max: float = 0.05):
        self._rng = np.random.default_rng(graine)
        self.sigma_nm = sigma_nm
        self._gain = 1.0 + self._rng.uniform(-gain_max, gain_max, n)
        self._biais = self._rng.uniform(-biais_max_nm, biais_max_nm, n)

    def __call__(self, efforts) -> np.ndarray:
        e = np.asarray(efforts, dtype=float)
        return e * self._gain + self._biais + self._rng.normal(0.0, self.sigma_nm, e.shape)


def source_couples(articulation, n: int):
    """Lecture des couples des articulations dans Isaac Sim : (fonction qui
    rend les n couples à plat, ou None ; texte pour le journal).

    L'effort MESURÉ par la physique (get_measured_joint_efforts : la force
    de l'articulation projetée sur son axe de mouvement), sinon les efforts
    IMPOSÉS (get_applied_joint_efforts : ceux passés à set_joint_efforts,
    nuls quand l'articulation est commandée en position). Essai GPU T1 du
    5 novembre 2026 : on lisait les imposés, l'OS ne recevait que le bruit
    ajouté ici, et les couples vérifiés contre les limites de l'URDF
    valaient zéro.

    À appeler l'articulation initialisée (après world.reset()) : on vérifie
    que la lecture rend n valeurs finies."""
    raison = "méthode absente de cette version d'Isaac Sim"
    mesure = getattr(articulation, "get_measured_joint_efforts", None)
    if mesure is not None:
        try:
            v = _a_plat(mesure)()
        except Exception as e:      # noqa: BLE001 — version ou vue physique : on le dit
            raison = f"lecture impossible ({type(e).__name__} : {e})"
        else:
            if v is not None and v.size == n and np.all(np.isfinite(v)):
                return _a_plat(mesure), ("couples lus : effort MESURÉ par la physique "
                                         "(get_measured_joint_efforts)")
            raison = (f"{0 if v is None else v.size} valeur(s) pour {n} articulations, "
                      f"ou des valeurs non finies")
    impose = getattr(articulation, "get_applied_joint_efforts", None)
    return (_a_plat(impose) if impose is not None else None), (
        f"couples lus : efforts IMPOSÉS (get_applied_joint_efforts), nuls en commande de "
        f"position — effort mesuré indisponible : {raison} ; la calibration jugera les "
        f"couples NON FIABLES")


def _a_plat(lire):
    def couples():
        v = lire()
        return None if v is None else np.asarray(v, dtype=float).reshape(-1)
    return couples
