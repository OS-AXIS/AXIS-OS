"""
AXIS-OS — Notifications vers l'opérateur.

Une app prévient l'opérateur par ctx.notify(titre, message, priority=...) ;
l'OS aussi (batterie critique, moteur trop chaud). Tout passe par le centre
de notifications, côté OS, qui remet chaque notification à un
INotificationSink : aujourd'hui FakeNotificationSink (les tests), demain
l'interface du robot ou un téléphone.

Règles, pour les notifications des apps :
  - titre (TITLE_MAX caractères) et message (MESSAGE_MAX) bornés, caractères
    de contrôle remplacés : une notification ne peut pas réécrire l'écran ;
  - une app ne se fait jamais passer pour l'OS : chaque notification porte
    sa source (« app ») et l'identifiant de l'app, que l'OS met lui-même en
    préfixe ; un texte qui se présente comme l'OS (« AXIS-OS ») est refusé ;
  - débit limité par app : RATE_PER_MIN notifications par minute au plus,
    dont URGENT_PER_MIN urgentes (anti-spam : une urgence qui sonne sans
    cesse n'alerte plus personne).
"""
from __future__ import annotations

import re
import threading
import time
import unicodedata
from abc import ABC, abstractmethod
from collections import deque
from dataclasses import dataclass
from typing import Callable, Deque, Dict, List, Optional, Tuple

PRIORITIES = ("info", "warning", "urgent")
TITLE_MAX = 80
MESSAGE_MAX = 500
RATE_PER_MIN = 6
URGENT_PER_MIN = 2
HISTORY_MAX = 200
_SE_DIT_OS = re.compile(r"axis[\W_]*os", re.IGNORECASE)


def se_dit_os(texte: str) -> bool:
    """Vrai si le texte se présente comme l'OS (« AXIS-OS », « Axis OS »,
    en pleine chasse…). Le vrai rempart reste le préfixe imposé par l'OS."""
    return bool(_SE_DIT_OS.search(unicodedata.normalize("NFKC", texte)))


def neutraliser(texte: str, n: int) -> str:
    """Caractères de contrôle remplacés par une espace, longueur bornée."""
    t = "".join(c if c.isprintable() else " " for c in str(texte))
    return t[:n]


@dataclass(frozen=True)
class Notification:
    """Une notification remise à l'opérateur. `source` : « os » ou « app » ;
    `app_id` : l'app émettrice (None pour l'OS), mis par l'OS."""
    source: str
    app_id: Optional[str]
    app_name: Optional[str]
    title: str
    message: str
    priority: str
    timestamp: float

    @property
    def display_title(self) -> str:
        """Le titre tel que l'opérateur le voit : la source d'abord, imposée
        par l'OS."""
        if self.source == "os":
            return f"AXIS-OS — {self.title}"
        return f"[app {self.app_name} · {self.app_id}] {self.title}"


class INotificationSink(ABC):
    """Là où l'opérateur reçoit les notifications (côté OS seulement)."""

    @abstractmethod
    def deliver(self, notification: Notification) -> bool:
        """Remet la notification ; False si le canal n'a pas pu la prendre."""


class FakeNotificationSink(INotificationSink):
    """Garde les notifications reçues (`delivered`), pour les tests."""

    def __init__(self):
        self._verrou = threading.Lock()
        self.delivered: List[Notification] = []

    def deliver(self, notification: Notification) -> bool:
        with self._verrou:
            self.delivered.append(notification)
        return True


class NotificationCenter:
    """Centre de notifications de l'OS (voir l'en-tête du module)."""

    def __init__(self, sink: INotificationSink,
                 horloge: Callable[[], float] = time.monotonic):
        self._sink = sink
        self._horloge = horloge
        self._verrou = threading.Lock()
        self._envois: Dict[str, Deque[Tuple[float, str]]] = {}
        self.history: Deque[Notification] = deque(maxlen=HISTORY_MAX)
        self.stats = {"sent": 0, "refused_rate": 0, "refused_os_name": 0, "sink_failed": 0}

    def _remettre(self, n: Notification) -> bool:
        try:
            ok = self._sink.deliver(n) is True
        except Exception:                    # noqa: BLE001 — canal en panne
            ok = False
        if not ok:
            self.stats["sink_failed"] += 1
        self.history.append(n)
        return ok

    def from_os(self, title: str, message: str, priority: str = "info") -> bool:
        """Notification de l'OS lui-même (jamais accessible aux apps)."""
        if priority not in PRIORITIES:
            priority = "urgent"
        n = Notification("os", None, None, neutraliser(title, TITLE_MAX),
                         neutraliser(message, MESSAGE_MAX), priority, self._horloge())
        return self._remettre(n)

    def from_app(self, app_id: str, app_name: str, title: str, message: str,
                 priority: str) -> Tuple[bool, Optional[str], str]:
        """Notification d'une app : (envoyée, code de refus, explication).
        app_id et app_name viennent du manifeste installé, jamais de l'app
        au moment de l'appel."""
        titre, msg = neutraliser(title, TITLE_MAX), neutraliser(message, MESSAGE_MAX)
        if se_dit_os(titre) or se_dit_os(msg):
            self.stats["refused_os_name"] += 1
            return False, "INVALID_ARGUMENT", ("une notification d'app ne peut pas se présenter "
                                               "comme l'OS (« AXIS-OS »)")
        # Ni crochet ni séparateur : le nom ne peut pas fermer le préfixe imposé.
        nom = re.sub(r"[\[\]·|]", " ", neutraliser(app_name, 64))
        if se_dit_os(nom):
            nom = app_id                     # un nom d'app qui imite l'OS n'est pas montré
        t = self._horloge()
        with self._verrou:
            envois = self._envois.setdefault(app_id, deque())
            while envois and t - envois[0][0] > 60.0:
                envois.popleft()
            urgentes = sum(1 for _, p in envois if p == "urgent")
            if len(envois) >= RATE_PER_MIN or (priority == "urgent" and urgentes >= URGENT_PER_MIN):
                self.stats["refused_rate"] += 1
                return False, "RATE_LIMITED_APP", (f"trop de notifications : {RATE_PER_MIN} par "
                                                   f"minute au plus, dont {URGENT_PER_MIN} "
                                                   f"urgentes")
            envois.append((t, priority))
        ok = self._remettre(Notification("app", app_id, nom, titre, msg, priority, t))
        if ok:
            self.stats["sent"] += 1
            return True, None, "envoyée"
        return False, "FAILED", "le canal de notification n'a pas pris la notification"
