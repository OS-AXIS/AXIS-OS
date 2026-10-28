"""
AXIS-OS — Détecteurs de personnes : le contrat et ses implémentations
simples.

Même séparation que pour la perception des objets
(axis_os/services/perception/service.py) : le service « personnes »
(service.py) est le CODE (images, repères, suivi, mémoire bornée) ; le
détecteur est le MODÈLE, remplaçable sans toucher à l'OS.

Un détecteur rend des boîtes et des scores, rien d'autre : il ne dit
jamais QUI est la personne. Aucune reconnaissance faciale, aucun gabarit
de visage : AXIS-OS n'en contient pas et n'en accepte pas.

Implémentations :
  - NullPersonDetector  : ne détecte rien, honnêtement ;
  - FakePersonDetector  : rend les détections fixées par un test ;
  - ObjectDetectorPersons : solution de secours, un détecteur d'objets à
    vocabulaire ouvert (OWL-ViT) interrogé avec « a person » ;
  - RTDETRPersonDetector : le détecteur réel retenu, dans
    rtdetr_detector.py.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Optional, Tuple

from axis_os.hal.core.types import CameraFrame


@dataclass(frozen=True)
class PersonDetection:
    """Une personne détectée dans une image.

    bbox           : boîte (x, y, largeur, hauteur), en pixels.
    score          : confiance du détecteur, dans [0, 1].
    position_world : position (x, y, z) dans le repère monde, si le
                     détecteur la connaît déjà (un détecteur simulé) ;
                     sinon None, et le service la calcule à partir de la
                     profondeur ou de la géométrie de la caméra.
    appearance     : descripteur d'apparence (appearance.py), si le
                     détecteur le fournit ; sinon le service le calcule à
                     partir de l'image.
    timestamp_ns   : instant de capture de l'IMAGE, en nanosecondes.
    """
    bbox: Tuple[float, float, float, float]
    score: float
    position_world: Optional[Tuple[float, float, float]] = None
    appearance: Optional[Tuple[float, ...]] = None
    timestamp_ns: int = 0


class IPersonDetector(ABC):
    """Contrat d'un détecteur de personnes."""

    @abstractmethod
    def detect(self, frame: Optional[CameraFrame]) -> List[PersonDetection]:
        """Personnes présentes dans `frame`. Liste vide si aucune, si le
        modèle n'est pas chargé, ou si l'inférence échoue (l'erreur est
        alors gardée dans stats["last_inference_error"], que le service lit
        pour distinguer une panne d'une scène vide)."""

    @property
    def needs_frame(self) -> bool:
        """Faux pour un détecteur qui n'a pas besoin d'image (simulé) : le
        service l'appelle alors sans caméra."""
        return True

    @property
    def stats(self) -> dict:
        return {}


class NullPersonDetector(IPersonDetector):
    """Ne détecte personne : la chaîne tourne sans modèle, et le service le
    dit (health) plutôt que de faire croire qu'il n'y a personne."""

    def detect(self, frame) -> List[PersonDetection]:
        return []


class FakePersonDetector(IPersonDetector):
    """Rend les détections fixées par le test (set()), pour chaque appel,
    sans image."""

    def __init__(self, detections=()):
        self._detections = list(detections)
        self.calls = 0

    def set(self, detections) -> None:
        self._detections = list(detections)

    def detect(self, frame) -> List[PersonDetection]:
        self.calls += 1
        return list(self._detections)

    @property
    def needs_frame(self) -> bool:
        return False


class ObjectDetectorPersons(IPersonDetector):
    """Solution de secours : un IObjectDetector à vocabulaire ouvert
    (OWL-ViT, axis_os/services/perception/owlvit_detector.py, Apache 2.0)
    interrogé avec `query`. Plus lent et moins précis que RT-DETR pour les
    personnes, mais déjà épinglé et validé dans le dépôt."""

    def __init__(self, object_detector, query: str = "a person", min_score: float = 0.15):
        self._det = object_detector
        self._query = query
        self._min = float(min_score)

    def detect(self, frame) -> List[PersonDetection]:
        if frame is None:
            return []
        out = []
        for d in self._det.detect(frame, self._query):
            if d.confidence >= self._min:
                out.append(PersonDetection(bbox=tuple(float(v) for v in d.bbox),
                                           score=float(d.confidence),
                                           timestamp_ns=d.timestamp_ns))
        return out

    @property
    def stats(self) -> dict:
        return dict(getattr(self._det, "stats", {}) or {})
