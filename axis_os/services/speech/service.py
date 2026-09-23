"""
AXIS-OS — Service de parole : de la voix à une INTENTION.

Quatrième service système. Transforme « va vers le carton » en une
intention nommée, avec ses paramètres — jamais en commandes moteur.

DEUX ÉTAGES, comme pour la perception :

    ISpeechRecognizer  — le MODÈLE : audio -> texte. Remplaçable.
    IntentRegistry     — les INTENTIONS : texte -> action nommée.
                         Déclarées par les apps, pas par l'OS.

INVARIANT DE SÉCURITÉ, le même que pour toute IA dans AXIS-OS : un
modèle de parole ne produit JAMAIS de commande articulaire. Il produit
au mieux le nom d'une intention et des paramètres. C'est ensuite une
app — code écrit, auditable, avec ses permissions — qui décide quoi en
faire. Entre la phrase et le moteur, il y a toujours du code
responsable.

Concrètement, cela veut dire qu'une phrase inattendue ne peut pas faire
n'importe quoi : elle ne correspond à aucune intention déclarée, et il
ne se passe rien. Le pire cas d'une reconnaissance ratée est
l'inaction, jamais un mouvement imprévu.

POURQUOI UN REGISTRE PLUTÔT QU'UN MODÈLE QUI DÉCIDE : une app déclare
ce qu'elle sait faire, l'OS choisit qui appeler. C'est le modèle des
intentions d'Android, et c'est ce qui permet d'installer une app sans
réentraîner quoi que ce soit.
"""
from __future__ import annotations

import re
import time
import unicodedata
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from axis_os.hal.core.interfaces import IMicrophone
from axis_os.hal.core.types import AudioChunk, HALStatus
from axis_os.services.base import IService, ServiceHealth, ServiceResult
from axis_os.store.sdk.manifest import Capability


@dataclass(frozen=True)
class Intent:
    """Intention reconnue dans une phrase."""
    name: str
    params: Dict[str, str] = field(default_factory=dict)
    confidence: float = 0.0
    utterance: str = ""


class ISpeechRecognizer(ABC):
    """Contrat d'un modèle de transcription — le MODÈLE, pas le service."""

    @abstractmethod
    def transcribe(self, audio: AudioChunk) -> Tuple[str, float]:
        """Rend (texte, confiance). Texte vide si rien n'a été compris :
        ne jamais inventer une phrase plausible, le silence est une
        réponse valable."""

    @property
    @abstractmethod
    def language(self) -> str: ...


class NullSpeechRecognizer(ISpeechRecognizer):
    """Ne transcrit rien, honnêtement. Permet de tester toute la chaîne
    — micro, registre, app — sans modèle."""

    def transcribe(self, audio: AudioChunk) -> Tuple[str, float]:
        return "", 0.0

    @property
    def language(self) -> str:
        return "fr"


def normalize(text: str) -> str:
    """Minuscules, sans accents, ponctuation réduite à des espaces.

    Une transcription rend « Va vers le carton ! » ou « va vers le
    carton » selon l'intonation et le modèle ; les deux doivent
    correspondre à la même intention. Faire dépendre une action d'un
    accent ou d'une virgule serait absurde.
    """
    t = unicodedata.normalize("NFD", text.lower())
    t = "".join(c for c in t if unicodedata.category(c) != "Mn")
    return " ".join(re.sub(r"[^a-z0-9]+", " ", t).split())


@dataclass(frozen=True)
class IntentDeclaration:
    """Intention déclarée par une app.

    `keywords` : groupes de synonymes. TOUS les groupes doivent être
    présents pour que l'intention corresponde — « aller » ET « carton »,
    pas l'un ou l'autre. Exiger la conjonction évite qu'une phrase
    vague déclenche une action.
    """
    name: str
    keywords: Sequence[Sequence[str]]
    params: Dict[str, str] = field(default_factory=dict)
    description: str = ""


class IntentRegistry:
    """Correspondance phrase -> intention déclarée.

    Volontairement simple et LISIBLE plutôt que statistique : on doit
    pouvoir dire pourquoi une phrase a déclenché une action, et
    pourquoi une autre n'a rien fait. Un modèle de langage
    remplacerait ce composant un jour, mais il devrait rendre le même
    type de résultat — une intention nommée, jamais une commande.
    """

    def __init__(self):
        self._intents: List[IntentDeclaration] = []

    def register(self, decl: IntentDeclaration) -> None:
        if any(d.name == decl.name for d in self._intents):
            raise ValueError(f"intention '{decl.name}' déjà déclarée")
        self._intents.append(decl)

    @property
    def declarations(self) -> List[IntentDeclaration]:
        return list(self._intents)

    def match(self, utterance: str) -> Optional[Intent]:
        """Meilleure intention correspondante, ou None.

        None est un résultat NORMAL : une phrase hors du vocabulaire
        déclaré ne doit rien déclencher. C'est ce qui garantit qu'une
        transcription erronée ne produit pas d'action imprévue.
        """
        norm = normalize(utterance)
        if not norm:
            return None
        mots = set(norm.split())

        meilleur, score_max = None, 0.0
        for decl in self._intents:
            touches = 0
            for groupe in decl.keywords:
                if any(normalize(k) in norm or normalize(k) in mots
                       for k in groupe):
                    touches += 1
            if touches != len(decl.keywords):
                continue            # tous les groupes sont exigés
            # Score : proportion de mots de la phrase expliqués par
            # l'intention. Départage deux intentions qui correspondent
            # toutes deux, en préférant la plus spécifique.
            attendus = sum(len(g) for g in decl.keywords)
            score = len(decl.keywords) / max(attendus, 1)
            if score > score_max:
                meilleur, score_max = decl, score

        if meilleur is None:
            return None
        return Intent(name=meilleur.name, params=dict(meilleur.params),
                      confidence=score_max, utterance=utterance)


class SpeechService(IService):
    """Écoute, transcrit, et rend une INTENTION."""

    def __init__(self, microphone: IMicrophone,
                 recognizer: Optional[ISpeechRecognizer] = None,
                 registry: Optional[IntentRegistry] = None,
                 min_confidence: float = 0.3):
        self._mic = microphone
        self._rec = recognizer or NullSpeechRecognizer()
        self._registry = registry or IntentRegistry()
        self._min_conf = min_confidence
        self._started = False
        self.stats = {"ecoutes": 0, "transcriptions_vides": 0,
                      "sans_intention": 0, "intentions": 0}

    @property
    def service_id(self) -> str:
        return "speech"

    @property
    def capability(self) -> Capability:
        return Capability.SPEECH_IN

    @property
    def registry(self) -> IntentRegistry:
        return self._registry

    def start(self) -> HALStatus:
        if not self._mic.is_healthy():
            st = self._mic.initialize()
            if st is not HALStatus.OK:
                return st
        self._started = True
        return HALStatus.OK

    def stop(self) -> None:
        self._started = False

    def health(self) -> ServiceHealth:
        if not self._started:
            return ServiceHealth(running=False, healthy=False,
                                 detail="service non démarré")
        if isinstance(self._rec, NullSpeechRecognizer):
            return ServiceHealth(running=True, healthy=True,
                                 detail="aucun modèle de transcription — "
                                        "chaîne testable, mais rien ne sera "
                                        "compris")
        return ServiceHealth(running=True, healthy=True, detail="opérationnel")

    # ---- API ---------------------------------------------------------

    def listen(self, duration_s: float = 3.0) -> ServiceResult:
        """Écoute, transcrit, et cherche une intention.

        value = Intent, ou None si rien ne correspond. `reason` dit
        toujours POURQUOI il ne se passe rien : silence, transcription
        vide, ou phrase hors du vocabulaire déclaré. Un robot muet qui
        n'explique pas son silence est indébogable.
        """
        if not self._started:
            return ServiceResult(ok=False, reason="service non démarré")

        self.stats["ecoutes"] += 1
        audio = self._mic.capture(duration_s)
        if audio is None:
            return ServiceResult(ok=False, reason="aucun son capté")

        texte, conf = self._rec.transcribe(audio)
        if not texte.strip():
            self.stats["transcriptions_vides"] += 1
            return ServiceResult(ok=True, value=None,
                                 reason="rien de compris (silence ou bruit)")
        if conf < self._min_conf:
            return ServiceResult(
                ok=True, value=None,
                reason=f"transcription peu sûre ({conf:.2f} < "
                       f"{self._min_conf}) : « {texte} » — ignorée plutôt "
                       f"qu'exécutée au hasard")

        intent = self._registry.match(texte)
        if intent is None:
            self.stats["sans_intention"] += 1
            return ServiceResult(
                ok=True, value=None,
                reason=f"« {texte} » ne correspond à aucune intention "
                       f"déclarée — aucune action")
        self.stats["intentions"] += 1
        return ServiceResult(ok=True, value=intent, reason=f"« {texte} »")
