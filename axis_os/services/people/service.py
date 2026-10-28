"""
AXIS-OS — Service « personnes » : qui est autour du robot, où, et depuis
quand. JAMAIS « qui est-ce ».

Ce service détecte les personnes dans l'image de la caméra (un
IPersonDetector, le MODÈLE, voir detector.py), les situe au sol dans le
repère monde, puis les suit d'une image à l'autre (tracker.py). Il rend des
numéros temporaires, des distances et des directions. Il ne commande
aucun actionneur (invariant de tout IService) : c'est le service de suivi
(follow.py) qui fait marcher le robot, par le service de locomotion, sous
le gardien de proximité.

Vie privée, règles du service :
  - aucune reconnaissance faciale, aucun gabarit de visage, aucune donnée
    biométrique : le détecteur rend des boîtes, l'apparence ne lit que les
    vêtements (appearance.py) ;
  - aucune image ni découpe de personne n'est écrite sur disque ou dans un
    journal : l'image capturée n'est gardée que le temps de la détection ;
  - les numéros sont temporaires et les apparences ne vivent qu'en mémoire,
    effacées avec leur piste (REID_MAX_S sans voir la personne au plus) ;
  - le service ne regarde que lorsqu'une app le lui demande : sans demande
    depuis DEMAND_S, il ne capture plus rien et oublie toutes les pistes.

Position d'une personne : par la caméra de profondeur si le robot en a une
(la surface visible du torse), sinon par la géométrie de la caméra (le
rayon qui passe par les pieds, prolongé jusqu'au sol), seulement si les
pieds sont dans l'image. Sinon, la personne est comptée (stats["unlocated"])
mais pas suivie : jamais une distance inventée.

À cadencer par la boucle de l'OS (tick(), vers 10 Hz), dans un fil à part
sur un vrai robot : une inférence prend des dizaines de millisecondes et ne
doit pas retarder la boucle de commande à 50 Hz.
"""
from __future__ import annotations

import math
import threading
import time
from typing import Callable, List, Optional, Tuple

import numpy as np

from axis_os.hal.core.frames import FRAME_CAMERA_OPTICAL, FRAME_WORLD
from axis_os.hal.core.types import HALStatus, Vector3
from axis_os.services.base import IService, ServiceHealth, ServiceResult
from axis_os.store.sdk.manifest import Capability

from .appearance import appearance_descriptor
from .detector import IPersonDetector, NullPersonDetector, PersonDetection
from .tracker import Observation, PersonTracker, TrackSnapshot


class PeopleService(IService):
    """Détection et suivi des personnes (voir l'en-tête du module).

    detector     : IPersonDetector.
    camera       : caméra couleur (ICamera) ; facultative pour un détecteur
                   qui n'a pas besoin d'image (needs_frame faux).
    frame_tree   : arbre de repères qui relie la caméra au monde.
    depth_camera : caméra de profondeur alignée sur la caméra couleur,
                   facultative (voir PerceptionService).
    source_pose  : fonction sans argument -> (x, y, cap) du robot, repère
                   monde : sert à exprimer une piste par rapport au robot.
    horloge      : horloge du robot (secondes).
    """

    # Le service regarde tant qu'une app l'a sollicité depuis moins de ça.
    DEMAND_S = 10.0
    # Image plus vieille : pas analysée (la scène a pu changer).
    MAX_FRAME_AGE_S = 0.5
    # Au-delà, une position calculée n'est pas crue (m).
    MAX_RANGE_M = 8.0
    # Une piste est « visible » si elle a été mesurée depuis moins de ça.
    VISIBLE_S = 0.3

    def __init__(self, detector: Optional[IPersonDetector] = None, *, camera=None,
                 frame_tree=None, depth_camera=None,
                 source_pose: Optional[Callable[[], Tuple[float, float, float]]] = None,
                 horloge: Callable[[], float] = time.monotonic):
        self._detector = detector or NullPersonDetector()
        self._camera = camera
        self._tree = frame_tree
        self._depth = depth_camera
        self._pose = source_pose
        self._horloge = horloge
        self._verrou = threading.RLock()
        self._tracker = PersonTracker()
        self._demarre = False
        self._demande_jusqua: Optional[float] = None
        self._dernier_cycle: Optional[float] = None
        self._cycles = 0
        self._hz: Optional[float] = None
        self._lat_ms_total = 0.0
        self._lat_ms_dernier: Optional[float] = None
        self._derniere_erreur: Optional[str] = None
        self.stats = {"cycles": 0, "detections": 0, "unlocated": 0, "stale_frames": 0,
                      "inference_errors": 0, "forgotten": 0}

    # ---- IService ------------------------------------------------------------

    @property
    def service_id(self) -> str:
        return "people"

    @property
    def capability(self) -> Capability:
        return Capability.CAMERA_RGB

    def start(self) -> HALStatus:
        self._demarre = True
        return HALStatus.OK

    def stop(self) -> None:
        self._demarre = False
        self.forget()

    def health(self) -> ServiceHealth:
        if not self._demarre:
            return ServiceHealth(running=False, healthy=False, detail="service non démarré")
        if isinstance(self._detector, NullPersonDetector):
            return ServiceHealth(running=True, healthy=True,
                                 detail="aucun détecteur : personne ne sera vu")
        if self._derniere_erreur:
            return ServiceHealth(running=True, healthy=False,
                                 detail=f"détecteur en panne : {self._derniere_erreur[:120]}")
        return ServiceHealth(running=True, healthy=True, detail="opérationnel")

    # ---- demande, oubli -------------------------------------------------------

    def demander(self) -> bool:
        """Une app a besoin du service : il regarde pendant DEMAND_S. Rend
        True s'il ne regardait pas (ses premières pistes ne sont donc pas
        encore confirmées)."""
        with self._verrou:
            maintenant = self._horloge()
            endormi = self._demande_jusqua is None or maintenant > self._demande_jusqua
            self._demande_jusqua = maintenant + self.DEMAND_S
            return endormi

    @property
    def watching(self) -> bool:
        """Vrai si une demande est en cours."""
        return (self._demarre and self._demande_jusqua is not None
                and self._horloge() <= self._demande_jusqua)

    def release(self) -> None:
        """Ne plus regarder (jusqu'à la prochaine demande) et tout oublier :
        appelée à la fin de chaque suivi."""
        with self._verrou:
            self._demande_jusqua = None
            self.forget()

    def forget(self) -> None:
        """Oublie toutes les pistes et leurs apparences."""
        with self._verrou:
            if len(self._tracker):
                self.stats["forgotten"] += 1
            self._tracker.forget_all()

    # ---- cycle de détection ---------------------------------------------------

    def tick(self) -> ServiceResult:
        """Un cycle : capture, détection, position au sol, suivi. Sans
        demande en cours, ne capture rien et oublie les pistes."""
        if not self._demarre:
            return ServiceResult(ok=False, reason="service non démarré")
        if not self.watching:
            if len(self._tracker):
                self.forget()
            return ServiceResult(ok=True, value=0, reason="aucune app ne regarde")
        t = self._horloge()
        frame = None
        if self._detector.needs_frame:
            if self._camera is None:
                return ServiceResult(ok=False, reason="aucune caméra")
            frame = self._camera.capture()
            if frame is None:
                return ServiceResult(ok=False, reason="aucune image disponible")
            if frame.timestamp_ns:
                age = (time.time_ns() - frame.timestamp_ns) / 1e9
                if age > self.MAX_FRAME_AGE_S:
                    self.stats["stale_frames"] += 1
                    return ServiceResult(ok=False, reason=f"image périmée de {age:.2f} s")
        t0 = time.perf_counter()
        detections = self._detector.detect(frame)
        erreur = (self._detector.stats or {}).get("last_inference_error")
        if erreur and not detections:
            self._derniere_erreur = str(erreur)
            self.stats["inference_errors"] += 1
            return ServiceResult(ok=False, reason=f"détecteur en panne ({str(erreur)[:120]}) — "
                                                  f"zéro détection ne veut PAS dire personne")
        self._derniere_erreur = None
        observations = self._situer(detections, frame)
        # L'image n'est plus utile : elle n'est gardée nulle part.
        frame = None
        latence = (time.perf_counter() - t0) * 1000.0
        with self._verrou:
            self._tracker.update(observations, t)
            if self._dernier_cycle is not None and t > self._dernier_cycle:
                hz = 1.0 / (t - self._dernier_cycle)
                self._hz = hz if self._hz is None else 0.8 * self._hz + 0.2 * hz
            self._dernier_cycle = t
            self._cycles += 1
            self._lat_ms_total += latence
            self._lat_ms_dernier = latence
            self.stats["cycles"] += 1
            self.stats["detections"] += len(detections)
        return ServiceResult(ok=True, value=len(observations))

    def _situer(self, detections: List[PersonDetection], frame) -> List[Observation]:
        """Position au sol (repère monde) et apparence de chaque détection ;
        les détections qu'on ne sait pas situer sont comptées et écartées."""
        image = None
        if frame is not None and getattr(frame, "encoding", "") == "rgb8":
            try:
                image = np.frombuffer(frame.data, dtype=np.uint8).reshape(
                    frame.height, frame.width, 3)
            except ValueError:
                image = None
        geometrie = self._geometrie(frame) if frame is not None else None
        depth = None
        if frame is not None and self._depth is not None:
            depth = getattr(self._depth, "capture_depth", lambda: None)()
        out = []
        for d in detections:
            pos, portee = self._position(d, frame, geometrie, depth)
            if pos is None:
                self.stats["unlocated"] += 1
                continue
            apparence = d.appearance
            if apparence is None and image is not None:
                apparence = appearance_descriptor(image, d.bbox)
            out.append(Observation(
                xy=(pos[0], pos[1]), score=float(d.score),
                appearance=None if apparence is None else np.asarray(apparence, dtype=float),
                range_m=portee, bbox=tuple(float(v) for v in d.bbox)))
        return out

    def _geometrie(self, frame):
        """(intrinsèques, transformation monde <- caméra optique), ou None."""
        if self._camera is None or self._tree is None:
            return None
        intr = self._camera.get_intrinsics()
        if intr is None:
            return None
        tf, _ = self._tree.lookup(FRAME_WORLD, FRAME_CAMERA_OPTICAL,
                                  now_ns=frame.timestamp_ns or None)
        return None if tf is None else (intr, tf)

    def _position(self, d: PersonDetection, frame, geometrie, depth):
        """((x, y, z) monde, portée en m) ou (None, None)."""
        if d.position_world is not None:
            p = tuple(float(v) for v in d.position_world)
            if not all(math.isfinite(v) for v in p):
                return None, None
            portee = None
            if self._pose is not None:
                rx, ry, _ = self._pose()
                portee = math.hypot(p[0] - rx, p[1] - ry)
            return p, (portee if portee is not None else 2.0)
        if geometrie is None:
            return None, None
        intr, tf = geometrie
        x, y, w, h = (float(v) for v in d.bbox)
        origine = tf.apply(Vector3(0.0, 0.0, 0.0))
        # 1. Profondeur : la surface visible du torse, le long du rayon qui
        # passe par son centre.
        if depth is not None:
            from axis_os.services.perception.service import PerceptionService
            torse = (x + 0.3 * w, y + 0.25 * h, 0.4 * w, 0.3 * h)
            dist = PerceptionService._depth_of_object(depth, torse)
            if dist is not None and 0.2 < dist < self.MAX_RANGE_M:
                r = intr.pixel_to_ray(x + w / 2.0, y + 0.4 * h)
                p = tf.apply(Vector3(r.x * dist, r.y * dist, r.z * dist))
                return (p.x, p.y, 0.0), dist
        # 2. Géométrie : le rayon des pieds jusqu'au sol (z = 0, repère
        # monde), seulement si les pieds sont dans l'image.
        if y + h > frame.height - 2:
            return None, None
        r = tf.apply_direction(intr.pixel_to_ray(x + w / 2.0, y + h))
        if r.z >= -1e-3:
            return None, None
        s = -origine.z / r.z
        p = Vector3(origine.x + s * r.x, origine.y + s * r.y, 0.0)
        portee = math.hypot(p.x - origine.x, p.y - origine.y)
        if not (math.isfinite(portee) and portee < self.MAX_RANGE_M):
            return None, None
        return (p.x, p.y, 0.0), portee

    # ---- lecture ---------------------------------------------------------------

    def snapshots(self) -> List[TrackSnapshot]:
        """Pistes confirmées (copies)."""
        with self._verrou:
            return self._tracker.snapshots()

    def track(self, track_id: int) -> Optional[TrackSnapshot]:
        for s in self.snapshots():
            if s.track_id == track_id:
                return s
        return None

    @property
    def last_cycle(self) -> Optional[float]:
        """Instant (horloge du robot) du dernier cycle de détection."""
        return self._dernier_cycle

    def is_visible(self, snap: TrackSnapshot, maintenant: Optional[float] = None) -> bool:
        t = self._horloge() if maintenant is None else maintenant
        return t - snap.last_seen <= self.VISIBLE_S

    def relative(self, snap: TrackSnapshot) -> Optional[Tuple[float, float]]:
        """(distance en m, direction en rad, positive vers la gauche) de la
        piste par rapport au robot, maintenant ; None sans pose du robot."""
        if self._pose is None:
            return None
        rx, ry, cap = (float(v) for v in self._pose())
        dx, dy = snap.x - rx, snap.y - ry
        b = math.atan2(dy, dx) - cap
        return math.hypot(dx, dy), (b + math.pi) % (2 * math.pi) - math.pi

    @property
    def metrics(self) -> dict:
        """Fréquence de détection (Hz), latence (ms), pertes et reprises."""
        with self._verrou:
            return {
                "detection_hz": self._hz,
                "latency_ms_last": self._lat_ms_dernier,
                "latency_ms_mean": (self._lat_ms_total / self._cycles) if self._cycles else None,
                "cycles": self.stats["cycles"],
                "detections": self.stats["detections"],
                "unlocated": self.stats["unlocated"],
                "stale_frames": self.stats["stale_frames"],
                "inference_errors": self.stats["inference_errors"],
                "tracks": len(self._tracker),
                "tracks_created": self._tracker.stats["tracks_created"],
                "tracks_lost": self._tracker.stats["tracks_lost"],
                "reidentified": self._tracker.stats["reid"],
                "reid_ambiguous": self._tracker.stats["reid_ambiguous"],
                "watching": self.watching,
            }
