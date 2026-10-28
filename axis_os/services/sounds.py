"""
AXIS-OS — Sons de l'OS (ctx.play_sound) : un petit jeu fixe de sons
(bip, alerte, confirmation), joués par un ISoundPlayer côté OS. Une app ne
joue aucun fichier à elle : seulement un nom de ce jeu, à un volume que
l'OS plafonne (MAX_VOLUME).
"""
from __future__ import annotations

import threading
from abc import ABC, abstractmethod
from typing import List, Tuple

SOUNDS = ("beep", "alert", "confirmation")
MAX_VOLUME = 0.8


class ISoundPlayer(ABC):
    """Le haut-parleur, côté OS seulement."""

    @abstractmethod
    def play(self, name: str, volume: float) -> bool:
        """Joue le son `name` (un de SOUNDS) ; False s'il n'a pas pu."""


class FakeSoundPlayer(ISoundPlayer):
    """Garde les sons joués (`played` : (nom, volume)), pour les tests."""

    def __init__(self):
        self._verrou = threading.Lock()
        self.played: List[Tuple[str, float]] = []

    def play(self, name: str, volume: float) -> bool:
        with self._verrou:
            self.played.append((name, volume))
        return True
