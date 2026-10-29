"""
Couples ESTIMÉS, comme le vrai G1 les donne : ses moteurs n'ont pas de
capteur de couple, le couple est estimé du courant (tau_est). On ajoute au
couple exact du solveur (get_applied_joint_efforts) les erreurs d'une telle
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
