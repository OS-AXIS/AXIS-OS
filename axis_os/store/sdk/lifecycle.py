"""
AXIS-OS — Cycle de vie d'une mission (axis_os.store.sdk.lifecycle)
===================================================================
Ce qui aide une app à bien finir : annulation coopérative, progression,
événements (arrêt d'urgence, alertes, batterie faible) et confirmation
par un humain.

Annulation coopérative : ctx.should_stop() devient vrai quand un humain
ou l'OS demande l'arrêt de la mission (PlayStoreRuntime.request_stop),
quand il reste moins de STOP_MARGIN_S au hook, ou sous arrêt d'urgence.
L'app range alors ce qu'elle fait (poser l'objet, s'arrêter) et rend la
main. Cela ne remplace PAS l'arrêt forcé : l'arrêt d'urgence coupe tout
de toute façon, et le quota de durée tue le processus d'une app qui ne
rend pas la main.

Les callbacks d'événements tournent dans le processus de l'app, dans un
thread qui interroge l'OS toutes les EventPoller.PERIOD_S secondes (aucune
fonction ne traverse le pont JSON).

Licence : Apache 2.0 (open-source — SDK public)
"""
from __future__ import annotations

import threading
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional

from .results import EXCEPTIONS_PAR_CODE, AxisRefusal, RefusalCode

# Il reste moins que cela au hook : should_stop() devient vrai (source
# « os »), pour que l'app range avant d'être tuée par le quota.
STOP_MARGIN_S = 2.0
# Bornes de report_progress().
PROGRESS_MESSAGE_MAX = 200
PROGRESS_HISTORY_MAX = 50
# Bornes de ask_confirmation().
QUESTION_MAX = 300
CONFIRMATION_MAX_S = 300.0
# Sources d'une demande d'arrêt.
STOP_SOURCES = ("human", "os")


class MissionCancelled(AxisRefusal):
    """Levée par ctx.check_cancelled() quand l'arrêt de la mission est
    demandé (par un humain ou par l'OS). Sous arrêt d'urgence, c'est
    EstopActiveError qui est levée."""
    code = RefusalCode.CANCELLED


EXCEPTIONS_PAR_CODE[RefusalCode.CANCELLED.value] = MissionCancelled


@dataclass
class Progress:
    """Dernière progression rapportée par une app (ctx.report_progress)."""
    fraction: float
    message: str
    timestamp: float          # heure du robot (ctx.now())

    def to_json(self) -> Dict[str, Any]:
        return {"fraction": self.fraction, "message": self.message,
                "timestamp": self.timestamp}


class IHumanInterface(ABC):
    """Côté OS seulement : là où un humain répond aux questions des apps
    (écran du robot, télécommande, application mobile). Une app n'y a
    jamais accès : elle ne peut que poser une question par
    ctx.ask_confirmation(), et ne peut jamais y répondre elle-même."""

    @abstractmethod
    def confirm(self, app_id: str, app_name: str, question: str,
                timeout_s: float) -> Optional[bool]:
        """Pose `question` au nom de l'app `app_id` (« com.auteur.app »,
        vérifié par l'OS à l'installation) ; `app_name` est le nom libre de
        son manifeste. L'interface doit montrer les deux : un nom comme
        « AXIS-OS Sécurité » ne dit pas qui demande. True : oui ; False :
        non ; None : pas de réponse dans `timeout_s` secondes."""


class EventPoller:
    """Thread qui interroge l'OS et appelle les callbacks de l'app.

    `lire(since_index)` rend un dict : estop (bool), alerts (liste de
    dicts), next_index (int), battery_percent (float ou None). Il lève
    quand le pont est fermé : le thread s'arrête alors."""

    PERIOD_S = 0.1

    def __init__(self, lire: Callable[[Optional[int]], Dict[str, Any]],
                 log: Callable[[str], None]):
        self._lire = lire
        self._log = log
        self._verrou = threading.Lock()
        self._estop: List[Callable] = []
        self._alertes: List[Callable] = []
        self._batterie: List[list] = []        # [callback, seuil, déjà sous le seuil]
        self._arret = threading.Event()
        self._fil: Optional[threading.Thread] = None
        self._index: Optional[int] = None
        self._estop_vu = False

    def add(self, kind: str, callback: Callable, threshold: Optional[float] = None) -> None:
        with self._verrou:
            if kind == "estop":
                self._estop.append(callback)
            elif kind == "alert":
                self._alertes.append(callback)
            else:
                self._batterie.append([callback, threshold, False])
            if self._fil is None:
                self._fil = threading.Thread(target=self._boucle, daemon=True,
                                             name="axis-evenements")
                self._fil.start()

    def stop(self) -> None:
        self._arret.set()

    def notify_estop(self) -> None:
        """Arrêt d'urgence appris par un autre chemin (un appel refusé avec
        ESTOP_ACTIVE, should_stop() qui répond « estop ») : les callbacks
        on_estop sont appelés tout de suite, dans le fil appelant, et une
        seule fois en tout. Pourquoi : dans le processus de l'app, la
        mission rend souvent la main dès ce refus ; attendre la prochaine
        interrogation, c'était rater l'événement."""
        with self._verrou:
            if self._estop_vu:
                return
            self._estop_vu = True
            estop = list(self._estop)
        for cb in estop:
            self._appeler(cb)

    def _appeler(self, cb: Callable, *args) -> None:
        try:
            cb(*args)
        except Exception as exc:          # noqa: BLE001 — un callback ne tue pas le fil
            self._log(f"callback d'événement en erreur : {type(exc).__name__}: {exc}")

    def poll_once(self) -> bool:
        """Une interrogation ; False si l'OS ne répond plus."""
        try:
            etat = self._lire(self._index)
        except Exception:                 # noqa: BLE001 — pont fermé ou refus
            return False
        pct = etat.get("battery_percent")
        faible = []
        with self._verrou:
            alertes = list(self._alertes)
            if pct is not None:
                # Une fois par descente sous le seuil, pas à chaque mesure.
                for b in self._batterie:
                    sous = pct <= b[1]
                    if sous and not b[2]:
                        faible.append(b[0])
                    b[2] = sous
        # Les callbacks sont appelés hors du verrou : l'un d'eux peut en
        # enregistrer un autre.
        if etat.get("estop"):
            self.notify_estop()
        if self._index is not None:
            for alerte in etat.get("alerts") or []:
                for cb in alertes:
                    self._appeler(cb, alerte)
        self._index = etat.get("next_index", self._index)
        for cb in faible:
            self._appeler(cb, pct)
        return True

    def _boucle(self) -> None:
        while not self._arret.is_set():
            if not self.poll_once():
                return
            self._arret.wait(self.PERIOD_S)
