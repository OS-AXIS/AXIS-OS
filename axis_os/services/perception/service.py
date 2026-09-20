"""
AXIS-OS — Service de perception.

Répond à « qu'est-ce que le robot voit, et où ? ». Deuxième service
système après MemoryService, même contrat IService.

DEUX RESPONSABILITÉS SÉPARÉES, et la séparation est le point important :

    PerceptionService  — le CODE : acquisition, calibration, repères,
                         conversion pixel -> direction dans le repère
                         du robot. Stable, testable, versionné avec
                         l'OS.
    IObjectDetector    — le MODÈLE : ce qui reconnaît les objets.
                         Remplaçable sans toucher à l'OS.

C'est la décision prise en §6.4 du document d'architecture : Android a
mis dix-huit mois à distribuer ses correctifs parce que tout passait
par une mise à jour système. Un détecteur meilleur ne doit pas exiger
de republier AXIS-OS.

CE QUE CE SERVICE NE FAIT PAS : commander un actionneur. Il rend des
observations, jamais des mouvements. Un service ne commande jamais un
actionneur directement — invariant documenté sur IService, et déjà
appliqué au niveau HAL par ICamera qui n'expose aucun moyen d'agir.

HONNÊTETÉ SUR LA PROFONDEUR : sans caméra de profondeur, une détection
donne une DIRECTION, pas une POSITION. Le champ `position_3d` reste
None dans ce cas, et c'est délibéré — inventer une distance plausible
donnerait à la préhension des coordonnées qu'elle prendrait au sérieux.
"""
from __future__ import annotations

import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from axis_os.hal.core.frames import (
    FRAME_BASE, FRAME_CAMERA_OPTICAL, FrameTree,
)
from axis_os.hal.core.interfaces import ICamera
from axis_os.hal.core.types import (
    CameraFrame, CameraIntrinsics, HALStatus, Vector3,
)
from axis_os.services.base import IService, ServiceHealth, ServiceResult
from axis_os.store.sdk.manifest import Capability


@dataclass(frozen=True)
class Detection:
    """Un objet détecté dans une image."""

    label: str
    confidence: float
    bbox: Tuple[int, int, int, int]      # x, y, largeur, hauteur en pixels
    direction: Optional[Vector3] = None
    """Direction vers l'objet DANS LE REPÈRE DU ROBOT. Toujours
    renseignée dès qu'une calibration et un arbre de repères sont
    disponibles — c'est ce qui permet au robot de s'orienter vers
    l'objet, même sans en connaître la distance."""

    position_3d: Optional[Vector3] = None
    """Position dans le repère du robot. None sans caméra de
    profondeur : une image couleur seule ne permet PAS de connaître la
    distance. Un service qui inventerait une valeur ici ferait tendre la
    main au mauvais endroit."""

    timestamp_ns: int = 0
    """Instant de l'IMAGE, pas de la détection. Sur un robot qui bouge,
    associer une observation à la pose du moment où elle est traitée
    plutôt qu'au moment où elle a été capturée décale l'objet de
    plusieurs centimètres."""


class IObjectDetector(ABC):
    """Contrat d'un détecteur d'objets — le MODÈLE, pas le service.

    Implémenter cette interface suffit à brancher un détecteur dans
    AXIS-OS, sans modifier une ligne du service.
    """

    @abstractmethod
    def detect(self, frame: CameraFrame,
               query: Optional[str] = None) -> List[Detection]:
        """Détecte dans `frame`. `query` décrit ce qu'on cherche en
        langage naturel ; un détecteur à vocabulaire fermé peut
        l'ignorer, mais devrait alors le documenter."""

    @property
    @abstractmethod
    def is_open_vocabulary(self) -> bool:
        """True si le détecteur trouve des objets décrits librement,
        sans avoir été entraîné sur une liste fixe.

        Cette distinction est structurante et pas cosmétique : un
        détecteur à vocabulaire fermé oblige à réentraîner pour chaque
        nouvel objet, ce qui ne passe pas à l'échelle d'un catalogue
        d'apps tierces. L'exposer permet à un orchestrateur de savoir
        s'il peut demander n'importe quoi ou seulement une liste connue.
        """


class NullObjectDetector(IObjectDetector):
    """Détecteur qui ne détecte rien, honnêtement.

    Permet de faire tourner et tester toute la chaîne — acquisition,
    calibration, repères — SANS modèle. Même rôle que
    NullBalanceModel : un service dégradé qui le dit, plutôt qu'un
    service absent qui fait tout échouer.
    """

    def detect(self, frame, query=None) -> List[Detection]:
        return []

    @property
    def is_open_vocabulary(self) -> bool:
        return False


class PerceptionService(IService):
    """Acquisition, calibration et mise en repère des observations.

    Usage :
        svc = PerceptionService(camera, frame_tree, detector)
        svc.start()
        r = svc.find("un verre")
        if r.ok:
            for d in r.value:
                print(d.label, d.direction)
    """

    def __init__(
        self,
        camera: ICamera,
        frame_tree: Optional[FrameTree] = None,
        detector: Optional[IObjectDetector] = None,
        max_frame_age_s: float = 1.0,
    ):
        self._camera = camera
        self._tree = frame_tree
        self._detector = detector or NullObjectDetector()
        self._max_frame_age_s = max_frame_age_s
        self._started = False
        self._detections_count = 0
        self._stale_frames = 0

    @property
    def service_id(self) -> str:
        return "perception"

    @property
    def capability(self) -> Capability:
        return Capability.CAMERA_RGB

    def start(self) -> HALStatus:
        if not self._camera.is_healthy():
            st = self._camera.initialize()
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
        if not self._camera.is_healthy():
            return ServiceHealth(running=True, healthy=False,
                                 detail="caméra non saine")
        if self._camera.get_intrinsics() is None:
            # Sain mais DÉGRADÉ : on voit, mais on ne sait pas situer.
            return ServiceHealth(
                running=True, healthy=True,
                detail="caméra non calibrée — directions indisponibles",
            )
        return ServiceHealth(running=True, healthy=True, detail="opérationnel")

    # ---- API ------------------------------------------------------------

    def describe_scene(self) -> ServiceResult:
        """Ce que le robot voit, en clair. Entrée principale d'un futur
        orchestrateur."""
        r = self.find(None)
        if not r.ok:
            return r
        if not r.value:
            if not self._detector.is_open_vocabulary:
                return ServiceResult(
                    ok=True, value="aucun objet reconnu",
                    reason="détecteur à vocabulaire fermé — l'absence de "
                           "détection ne prouve pas l'absence d'objet",
                )
            return ServiceResult(ok=True, value="aucun objet visible")
        labels = ", ".join(sorted({d.label for d in r.value}))
        return ServiceResult(ok=True, value=labels)

    def find(self, query: Optional[str],
             now_ns: Optional[int] = None) -> ServiceResult:
        """Cherche un objet décrit en langage naturel.

        value = List[Detection], triée par confiance décroissante.
        """
        if not self._started:
            return ServiceResult(ok=False, reason="service non démarré")

        if query is not None and not self._detector.is_open_vocabulary:
            # On n'échoue pas — le détecteur peut connaître ce label —
            # mais on ne laisse pas croire que la requête a été comprise
            # telle quelle.
            pass

        frame = self._camera.capture()
        if frame is None:
            return ServiceResult(
                ok=False, reason="aucune image disponible")

        now = now_ns if now_ns is not None else time.time_ns()
        age_s = (now - frame.timestamp_ns) / 1e9 if frame.timestamp_ns else 0.0
        if age_s > self._max_frame_age_s:
            self._stale_frames += 1
            return ServiceResult(
                ok=False, partial=True,
                reason=f"image périmée de {age_s:.2f}s (seuil "
                       f"{self._max_frame_age_s}s) — la scène a pu changer",
            )

        detections = self._detector.detect(frame, query)
        self._detections_count += len(detections)

        intrinsics = self._camera.get_intrinsics()
        if intrinsics is None or self._tree is None:
            # Dégradation HONNÊTE : les détections sont rendues, mais
            # sans direction. Mieux qu'une direction inventée, et mieux
            # qu'un échec total qui priverait l'appelant de l'info utile.
            return ServiceResult(
                ok=True,
                value=sorted(detections, key=lambda d: -d.confidence),
                reason=("caméra non calibrée" if intrinsics is None
                        else "aucun arbre de repères") +
                       " — directions indisponibles",
            )

        located, warn = self._locate(detections, intrinsics, now)
        return ServiceResult(
            ok=True,
            value=sorted(located, key=lambda d: -d.confidence),
            reason=warn,
        )

    def _locate(self, detections, intrinsics: CameraIntrinsics,
                now_ns: int):
        """Ajoute à chaque détection sa direction dans le repère robot."""
        tf, warn = self._tree.lookup(
            FRAME_BASE, FRAME_CAMERA_OPTICAL, now_ns=now_ns)
        if tf is None:
            return detections, warn

        out = []
        for d in detections:
            x, y, w, h = d.bbox
            # Centre de la boîte — approximation assumée : le centre
            # géométrique d'une boîte n'est pas le centre de masse de
            # l'objet, ni son point de préhension. Suffisant pour
            # s'orienter vers lui, pas pour l'attraper.
            ray_optical = intrinsics.pixel_to_ray(x + w / 2.0, y + h / 2.0)
            direction = tf.apply_direction(ray_optical)
            out.append(Detection(
                label=d.label, confidence=d.confidence, bbox=d.bbox,
                direction=direction,
                position_3d=d.position_3d,
                timestamp_ns=d.timestamp_ns,
            ))
        return out, warn

    @property
    def stats(self) -> dict:
        return {
            "detections": self._detections_count,
            "stale_frames": self._stale_frames,
            "open_vocabulary": self._detector.is_open_vocabulary,
        }
