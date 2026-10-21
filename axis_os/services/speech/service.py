"""
AXIS-OS — Service de parole : de la voix à une INTENTION.

Quatrième service système. Il transforme une phrase comme « va vers le
carton » en une intention nommée, avec ses paramètres, et jamais en
commandes moteur.

Deux étages, comme pour la perception :

    ISpeechRecognizer  — le modèle, qui passe de l'audio au texte ;
                         il est remplaçable.
    IntentRegistry     — les intentions, qui passent du texte à une
                         action nommée ; elles sont déclarées par les
                         apps, pas par l'OS.

INVARIANT DE SÉCURITÉ, le même que pour toute IA dans AXIS-OS (voir
docs/architecture_services.md §8) : un modèle de parole ne produit
JAMAIS de commande articulaire (une consigne envoyée à un moteur). Il
produit au mieux le nom d'une intention et des paramètres. C'est
ensuite une app, c'est-à-dire du code écrit, auditable et soumis à ses
permissions, qui décide quoi en faire. Entre la phrase et le moteur, il
y a toujours du code responsable.

Concrètement, une phrase inattendue ne peut pas faire n'importe quoi :
si elle ne correspond à aucune intention déclarée, il ne se passe rien.
Le pire cas d'une reconnaissance ratée est l'inaction, jamais un
mouvement imprévu.

Pourquoi un registre plutôt qu'un modèle qui décide : une app déclare
ce qu'elle sait faire, et l'OS choisit qui appeler. C'est le principe
des intentions (« intents ») d'Android, et c'est ce qui permet
d'installer une app sans réentraîner quoi que ce soit.
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
    """Intention reconnue dans une phrase.

    `params` reprend les paramètres déclarés par l'app ; `confidence`
    est le score de correspondance calculé par IntentRegistry.match()
    (entre 0 et 1) ; `utterance` est la phrase d'origine."""
    name: str
    params: Dict[str, str] = field(default_factory=dict)
    confidence: float = 0.0
    utterance: str = ""


class ISpeechRecognizer(ABC):
    """Contrat d'un modèle de transcription (audio vers texte) : le
    modèle lui-même, pas le service."""

    @abstractmethod
    def transcribe(self, audio: AudioChunk) -> Tuple[str, float]:
        """Rend (texte, confiance). Le texte est vide si rien n'a été
        compris : ne jamais inventer une phrase plausible, le silence
        est une réponse valable."""

    @property
    @abstractmethod
    def language(self) -> str: ...


class NullSpeechRecognizer(ISpeechRecognizer):
    """Reconnaisseur qui ne transcrit rien, honnêtement : il rend
    toujours un texte vide. Il permet de tester toute la chaîne (micro,
    registre, app) sans modèle."""

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
    # La décomposition Unicode NFD sépare chaque lettre accentuée en
    # lettre de base suivie d'un accent combinant (catégorie « Mn ») :
    # « é » devient « e » + accent, et la ligne suivante retire l'accent.
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

    `params` : paramètres fixes, renvoyés tels quels dans l'Intent
    reconnu. `description` : texte libre décrivant l'intention.
    """
    name: str
    keywords: Sequence[Sequence[str]]
    params: Dict[str, str] = field(default_factory=dict)
    description: str = ""


class IntentRegistry:
    """Correspondance entre une phrase et une intention déclarée.

    Volontairement simple et LISIBLE plutôt que statistique : on doit
    pouvoir dire pourquoi une phrase a déclenché une action, et
    pourquoi une autre n'a rien fait. Un modèle de langage pourrait
    remplacer ce composant un jour, mais il devrait rendre le même
    type de résultat : une intention nommée, jamais une commande.
    """

    MAX_INTENTIONS = 256

    def __init__(self):
        self._intents: List[IntentDeclaration] = []

    def register(self, decl: IntentDeclaration) -> None:
        """Ajoute une déclaration. Un nom déjà déclaré est refusé
        (ValueError) : un nom d'intention désigne une seule
        déclaration."""
        if any(d.name == decl.name for d in self._intents):
            raise ValueError(f"intention '{decl.name}' déjà déclarée")
        # Le registre vit dans le processus de l'OS et dure plus longtemps
        # que chaque app : sans borne, des déclarations répétées le feraient
        # grossir jusqu'à épuiser sa mémoire.
        if len(self._intents) >= self.MAX_INTENTIONS:
            raise ValueError(f"registre des intentions plein ({self.MAX_INTENTIONS})")
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
                # Un mot-clé compte s'il apparaît dans la phrase
                # normalisée, y compris comme simple sous-chaîne : cela
                # permet des mots-clés de plusieurs mots (« va vers »),
                # mais un mot-clé court peut aussi correspondre à
                # l'intérieur d'un mot plus long.
                if any(normalize(k) in norm or normalize(k) in mots
                       for k in groupe):
                    touches += 1
            if touches != len(decl.keywords):
                continue            # tous les groupes sont exigés
            # Score : nombre de groupes divisé par le nombre total de
            # synonymes déclarés. Il départage deux intentions qui
            # correspondent toutes deux, en préférant la plus spécifique
            # (celle dont le vocabulaire est le plus resserré). À
            # égalité, la première déclarée l'emporte.
            attendus = sum(len(g) for g in decl.keywords)
            score = len(decl.keywords) / max(attendus, 1)
            if score > score_max:
                meilleur, score_max = decl, score

        if meilleur is None:
            return None
        return Intent(name=meilleur.name, params=dict(meilleur.params),
                      confidence=score_max, utterance=utterance)


class SpeechService(IService):
    """Écoute, transcrit, et rend une INTENTION (jamais une commande).

    `microphone` : le micro, vu à travers l'interface IMicrophone du HAL
    (la couche d'abstraction matérielle). `recognizer` : le modèle de
    transcription ; sans modèle, NullSpeechRecognizer. `registry` : les
    intentions déclarées par les apps. `min_confidence` : seuil sous
    lequel une transcription est ignorée plutôt qu'exécutée au hasard.
    `stats` : compteurs d'écoutes, de transcriptions vides, de phrases
    sans intention et d'intentions reconnues, utiles au diagnostic.
    """

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
        # Le micro n'est initialisé que s'il n'est pas déjà en bonne
        # santé ; un échec d'initialisation est rendu tel quel, et le
        # service reste alors arrêté.
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
        # Sans modèle, le service reste déclaré sain : la chaîne
        # fonctionne, elle ne comprend simplement rien, et le détail le
        # dit explicitement.
        if isinstance(self._rec, NullSpeechRecognizer):
            return ServiceHealth(running=True, healthy=True,
                                 detail="aucun modèle de transcription — "
                                        "chaîne testable, mais rien ne sera "
                                        "compris")
        return ServiceHealth(running=True, healthy=True, detail="opérationnel")

    # ---- API ---------------------------------------------------------

    def listen(self, duration_s: float = 3.0) -> ServiceResult:
        """Écoute pendant `duration_s` secondes, transcrit, et cherche
        une intention.

        value = Intent, ou None si rien ne correspond. `reason` dit
        toujours pourquoi il ne se passe rien : silence, transcription
        vide, transcription peu sûre ou phrase hors du vocabulaire
        déclaré. Un robot muet qui n'explique pas son silence est
        indébogable. ok=False est réservé au service non démarré et à
        l'absence totale de son capté.
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
