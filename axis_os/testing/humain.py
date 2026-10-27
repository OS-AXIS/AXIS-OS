"""
Humain simulé pour les tests (axis_os.testing.ScriptedHuman).

Le test fixe à l'avance les réponses de l'humain aux questions des apps
(ctx.ask_confirmation). Il se branche côté OS, comme une vraie interface
humaine : l'app ne le voit jamais et ne peut pas y répondre elle-même.

Licence : Apache 2.0 (open-source — SDK public)
"""
from __future__ import annotations

import threading
import time
from typing import Iterable, List, Optional, Tuple

from axis_os.store.sdk.lifecycle import IHumanInterface


class ScriptedHuman(IHumanInterface):
    """Rend les réponses prévues, dans l'ordre : True (oui), False (non) ou
    None (pas de réponse dans le délai). Une fois la liste épuisée, plus
    personne ne répond (None). Chaque question est notée dans `questions`
    (nom de l'app, question, délai) : un test vérifie ainsi ce que
    l'humain a vu. `delay_s` : temps de réflexion avant de répondre."""

    def __init__(self, answers: Iterable[Optional[bool]] = (), *, delay_s: float = 0.0):
        self._reponses = list(answers)
        self._delai = float(delay_s)
        self._verrou = threading.Lock()
        self.questions: List[Tuple[str, str, float]] = []

    def answer(self, *answers: Optional[bool]) -> None:
        """Ajoute des réponses à la fin de la liste."""
        with self._verrou:
            self._reponses.extend(answers)

    def confirm(self, app_name: str, question: str, timeout_s: float) -> Optional[bool]:
        with self._verrou:
            self.questions.append((app_name, question, timeout_s))
            reponse = self._reponses.pop(0) if self._reponses else None
        if reponse is None:
            # Personne ne répond : on attend tout le délai, comme en vrai.
            time.sleep(timeout_s)
            return None
        time.sleep(min(self._delai, timeout_s))
        return reponse if self._delai <= timeout_s else None
