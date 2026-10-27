"""
AXIS-OS — Synthèse vocale : le contrat (axis_os.services.speech.synthesis)
===========================================================================
ISpeechSynthesizer est ce que l'OS appelle pour faire parler le robot
(ctx.speak). Brancher un vrai moteur de synthèse, c'est implémenter cette
interface ; aucun n'est fourni ni téléchargé par AXIS-OS.

FakeSpeechSynthesizer, pour les tests et la simulation, note ce qui a été
dit au lieu de le prononcer.

Licence : Apache 2.0
"""
from __future__ import annotations

import threading
from abc import ABC, abstractmethod
from typing import List, Tuple


class ISpeechSynthesizer(ABC):
    """Côté OS : prononce un texte déjà borné et neutralisé par le SDK."""

    @property
    @abstractmethod
    def languages(self) -> Tuple[str, ...]:
        """Langues prises en charge (codes « fr », « en », « fr-FR »…)."""

    @abstractmethod
    def say(self, text: str, language: str) -> bool:
        """Prononce `text` dans `language` ; True si c'est dit."""


class FakeSpeechSynthesizer(ISpeechSynthesizer):
    """Synthèse factice : `spoken` garde chaque (texte, langue) dit, dans
    l'ordre. Aucun son, aucun modèle."""

    def __init__(self, languages: Tuple[str, ...] = ("en", "fr")):
        self._langues = tuple(languages)
        self._verrou = threading.Lock()
        self.spoken: List[Tuple[str, str]] = []

    @property
    def languages(self) -> Tuple[str, ...]:
        return self._langues

    def say(self, text: str, language: str) -> bool:
        with self._verrou:
            self.spoken.append((text, language))
        return True
