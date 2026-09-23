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
    FRAME_BASE, FRAME_CAMERA_OPTICAL, FRAME_WORLD, FrameTree,
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
        depth_camera: Optional[ICamera] = None,
    ):
        """
        depth_camera : caméra de profondeur, si disponible. Sans elle,
            les détections portent une DIRECTION mais jamais de
            POSITION — un robot sait alors vers où s'orienter, pas où
            s'arrêter. C'est la différence entre « le colis est par là »
            et « le colis est à 2.3 m ».

            Doit regarder la MÊME scène que `camera` : le service
            suppose les deux images alignées pixel à pixel. Sur une
            RealSense, cela exige l'alignement profondeur/couleur ; sur
            un montage où elles ne le sont pas, les positions seraient
            fausses sans qu'aucune vérification ne s'en aperçoive.
        """
        self._camera = camera
        self._depth_camera = depth_camera
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
             now_ns: Optional[int] = None,
             frame: str = FRAME_BASE) -> ServiceResult:
        """Cherche un objet décrit en langage naturel.

        value = List[Detection], triée par confiance décroissante.

        `frame` : repère dans lequel exprimer directions et positions.

        FRAME_BASE (défaut) — repère du ROBOT. C'est ce qu'il faut pour
        se diriger vers un objet : « 15° à gauche, à 2 m ».

        FRAME_WORLD — repère FIXE. Indispensable dès qu'on mémorise une
        position. Constaté : une app d'inventaire qui balayait en
        TOURNANT SUR PLACE enregistrait ses observations dans le repère
        du robot ; deux colis situés à 90° l'un de l'autre apparaissaient
        tous deux à (2.2, 0) — face à chacun — et ont été fusionnés en un
        seul. Un inventaire tenu dans un repère mobile n'a pas de sens.
        """
        # Le paramètre `frame` désigne un REPÈRE ; la variable `frame`
        # plus bas désigne l'IMAGE capturée. On fige le repère tout de
        # suite pour éviter que l'une n'écrase l'autre — collision qui
        # donnait un « unhashable type: CameraFrame » très opaque.
        repere_sortie = frame

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

        # Une inférence en PANNE rend zéro détection — strictement
        # indiscernable d'une scène vide pour l'appelant. Constaté en
        # exécution réelle : le post-traitement d'OWL-ViT plantait à
        # chaque image et find() rendait ok=True avec 0 détection, sans
        # le moindre signal. Un détecteur qui expose son erreur doit la
        # voir remonter.
        det_err = getattr(self._detector, "stats", {}).get(
            "last_inference_error") if hasattr(self._detector, "stats") else None
        if det_err and not detections:
            return ServiceResult(
                ok=False, value=[],
                reason=f"le détecteur est en panne ({det_err}) — zéro "
                       f"détection ne signifie PAS que la scène est vide",
            )

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

        depth = None
        if self._depth_camera is not None:
            depth = getattr(self._depth_camera, "capture_depth", lambda: None)()
        located, warn = self._locate(detections, intrinsics, now, depth,
                                     repere_sortie)
        return ServiceResult(
            ok=True,
            value=sorted(located, key=lambda d: -d.confidence),
            reason=warn,
        )

    def _locate(self, detections, intrinsics: CameraIntrinsics,
                now_ns: int, depth=None, frame: str = FRAME_BASE):
        """Ajoute à chaque détection sa direction dans le repère robot."""
        tf, warn = self._tree.lookup(
            frame, FRAME_CAMERA_OPTICAL, now_ns=now_ns)
        if tf is None:
            return detections, warn

        out = []
        for d in detections:
            x, y, w, h = d.bbox
            # Centre de la boîte — approximation assumée : le centre
            # géométrique d'une boîte n'est pas le centre de masse de
            # l'objet, ni son point de préhension. Suffisant pour
            # s'orienter vers lui, pas pour l'attraper.
            cx, cy = x + w / 2.0, y + h / 2.0
            ray_optical = intrinsics.pixel_to_ray(cx, cy)
            direction = tf.apply_direction(ray_optical)

            # POSITION : direction x distance. pixel_to_ray rend un rayon
            # de z=1 dans le repère optique, et la profondeur est mesurée
            # perpendiculairement au plan image : le produit des deux
            # donne directement le point, sans trigonométrie.
            position = d.position_3d
            if position is None and depth is not None:
                dist = self._depth_of_object(depth, d.bbox)
                if dist is not None:
                    p_opt = Vector3(ray_optical.x * dist,
                                    ray_optical.y * dist,
                                    ray_optical.z * dist)
                    position = tf.apply(p_opt)     # apply, PAS
                    # apply_direction : un POINT subit la translation,
                    # contrairement à une direction.

            out.append(Detection(
                label=d.label, confidence=d.confidence, bbox=d.bbox,
                direction=direction,
                position_3d=position,
                timestamp_ns=d.timestamp_ns,
            ))
        return out, warn

    @staticmethod
    def _depth_of_object(depth, bbox, quantile: float = 0.25):
        """Distance de la SURFACE VISIBLE de l'objet détecté.

        Prend la médiane du quart le plus PROCHE des profondeurs valides
        dans la boîte, plutôt qu'un point au centre.

        Pourquoi : une boîte de détection contient toujours de
        l'arrière-plan — d'autant plus qu'elle est large ou imprécise. Un
        échantillon au centre tombe alors sur ce qu'il y a DERRIÈRE
        l'objet. Constaté : un robot arrivé à 5 cm d'un colis le croyait
        encore à 1.87 m, parce que la mesure portait sur le sol visible
        au-delà (position z = -0.74 dans son repère, soit le plancher).

        Le quart le plus proche, lui, appartient à l'objet : c'est la
        partie de la boîte qui est devant. Et prendre la médiane de ce
        quart plutôt que le minimum absolu résiste au bruit et aux
        pixels aberrants d'une caméra de profondeur.

        Rend None si rien d'exploitable — jamais une distance par défaut.
        """
        import numpy as np
        h, w = depth.shape[0], depth.shape[1]
        x, y, bw, bh = (int(v) for v in bbox)
        x0, x1 = max(0, x), min(w, x + max(bw, 1))
        y0, y1 = max(0, y), min(h, y + max(bh, 1))
        if x0 >= x1 or y0 >= y1:
            return None
        patch = np.asarray(depth[y0:y1, x0:x1], dtype=float).ravel()
        valid = patch[np.isfinite(patch) & (patch > 1e-3)]
        if valid.size == 0:
            return None
        seuil = float(np.quantile(valid, quantile))
        proches = valid[valid <= seuil]
        return float(np.median(proches)) if proches.size else float(seuil)

    @staticmethod
    def _depth_at(depth, cx: float, cy: float, half: int = 3):
        """Distance au centre d'une détection, par MÉDIANE d'un petit
        voisinage.

        Un pixel isolé suffit à tout fausser : sur un bord d'objet il
        peut appartenir à l'arrière-plan, et une caméra de profondeur
        rend régulièrement des valeurs invalides (0, inf, NaN). La
        médiane d'un voisinage est robuste à ces deux cas. Rend None si
        aucune valeur exploitable — jamais une distance par défaut.
        """
        import numpy as np
        h, w = depth.shape[0], depth.shape[1]
        x0, x1 = max(0, int(cx) - half), min(w, int(cx) + half + 1)
        y0, y1 = max(0, int(cy) - half), min(h, int(cy) + half + 1)
        if x0 >= x1 or y0 >= y1:
            return None
        patch = np.asarray(depth[y0:y1, x0:x1], dtype=float).ravel()
        valid = patch[np.isfinite(patch) & (patch > 1e-3)]
        return float(np.median(valid)) if valid.size else None

    @property
    def stats(self) -> dict:
        return {
            "detections": self._detections_count,
            "stale_frames": self._stale_frames,
            "open_vocabulary": self._detector.is_open_vocabulary,
        }
