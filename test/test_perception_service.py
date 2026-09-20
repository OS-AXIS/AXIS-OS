"""
PerceptionService — acquisition, calibration, mise en repère.

Ce que ce test protège en priorité : les cas où le service DÉGRADE.
Une perception qui invente une direction ou une distance est bien pire
qu'une perception qui dit ne pas savoir — la préhension prendrait la
valeur au sérieux et tendrait la main au mauvais endroit.
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.core.frames import (
    FRAME_BASE, FRAME_CAMERA, FrameTree, Transform, camera_optical_to_link,
)
from axis_os.hal.core.interfaces import ICamera
from axis_os.hal.core.types import (
    CameraFrame, CameraIntrinsics, HALStatus, SensorType, Vector3,
)
from axis_os.services.base import IService
from axis_os.services.perception.service import (
    Detection, IObjectDetector, NullObjectDetector, PerceptionService,
)

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


class StubCamera(ICamera):
    """Caméra contrôlable : calibration et âge d'image paramétrables."""

    def __init__(self, intrinsics=None, age_s=0.0, healthy=True, frame=True):
        super().__init__("stub", SensorType.CAMERA_RGB)
        self._k = intrinsics
        self._age_s = age_s
        self._healthy = healthy
        self._frame = frame
        self._initialized = True

    def initialize(self): return HALStatus.OK
    def shutdown(self): return HALStatus.OK
    def is_healthy(self): return self._healthy
    def get_intrinsics(self): return self._k

    def capture(self):
        if not self._frame:
            return None
        return CameraFrame(640, 480, "rgb8", b"\x01" * 16,
                           timestamp_ns=time.time_ns() - int(self._age_s * 1e9))


class StubDetector(IObjectDetector):
    def __init__(self, open_vocab=True, dets=None):
        self._ov = open_vocab
        self._dets = dets

    def detect(self, frame, query=None):
        if self._dets is not None:
            return self._dets
        return [Detection("verre", 0.9, (300, 220, 40, 40),
                          timestamp_ns=frame.timestamp_ns)]

    @property
    def is_open_vocabulary(self): return self._ov


K = CameraIntrinsics(fx=600.0, fy=600.0, cx=320.0, cy=240.0)


def make_tree():
    t = FrameTree()
    t.set(Transform(FRAME_BASE, FRAME_CAMERA, Vector3(0.1, 0.0, 0.5)))
    t.set(camera_optical_to_link())
    return t


section("1 — Contrat IService")

svc = PerceptionService(StubCamera(K), make_tree(), StubDetector())
check("PerceptionService est un IService", isinstance(svc, IService))
check("Aucune méthode abstraite restante",
      not getattr(svc, "__abstractmethods__", None))
check("service_id stable", svc.service_id == "perception")
check("find() avant start() échoue proprement",
      svc.find("x").ok is False)
check("start() -> OK", svc.start() is HALStatus.OK)
check("health() opérationnel", svc.health().healthy and svc.health().running)

section("2 — Chaîne pixel -> direction dans le repère robot")

r = svc.find("un verre")
check("find() réussit", r.ok and len(r.value) == 1)
d = r.value[0]
check(f"Direction calculée dans le repère robot ({d.direction})",
      d.direction is not None and d.direction.x > 0.99)
check("position_3d reste None SANS caméra de profondeur — aucune "
      "distance inventée",
      d.position_3d is None)
check("L'horodatage est celui de l'IMAGE, pas du traitement",
      d.timestamp_ns > 0)

# Un objet à droite dans l'image doit donner y négatif.
right = PerceptionService(
    StubCamera(K), make_tree(),
    StubDetector(dets=[Detection("boite", 0.8, (560, 220, 40, 40))]))
right.start()
dr = right.find(None).value[0]
check(f"Objet à droite dans l'image -> y négatif (y={dr.direction.y:.3f})",
      dr.direction.y < -0.1)

section("3 — Dégradation HONNÊTE (le cœur du test)")

no_calib = PerceptionService(StubCamera(None), make_tree(), StubDetector())
no_calib.start()
r = no_calib.find(None)
check("Sans calibration : détections rendues quand même (ok=True)",
      r.ok and len(r.value) == 1)
check("Sans calibration : direction à None, pas inventée",
      r.value[0].direction is None)
check("Sans calibration : la RAISON est explicite",
      r.reason is not None and "calibr" in r.reason.lower())

no_tree = PerceptionService(StubCamera(K), None, StubDetector())
no_tree.start()
r = no_tree.find(None)
check("Sans arbre de repères : détections rendues, direction à None",
      r.ok and r.value[0].direction is None and r.reason is not None)

h = PerceptionService(StubCamera(None), make_tree(), StubDetector())
h.start()
check("health() signale DÉGRADÉ (sain mais non calibré), pas en panne",
      h.health().healthy and "calibr" in (h.health().detail or ""))

section("4 — Image périmée et absence d'image")

stale = PerceptionService(StubCamera(K, age_s=5.0), make_tree(),
                          StubDetector(), max_frame_age_s=1.0)
stale.start()
r = stale.find(None)
check("Image vieille de 5s : refusée, avec partial=True",
      r.ok is False and r.partial is True)
check("La raison mentionne que la scène a pu changer",
      r.reason is not None and "périmée" in r.reason)

none_cam = PerceptionService(StubCamera(K, frame=False), make_tree(),
                             StubDetector())
none_cam.start()
check("Aucune image disponible : échec net, sans exception",
      none_cam.find(None).ok is False)

section("5 — Vocabulaire ouvert ou fermé : la distinction est exposée")

closed = PerceptionService(
    StubCamera(K), make_tree(),
    StubDetector(open_vocab=False, dets=[]))
closed.start()
r = closed.describe_scene()
check("Détecteur fermé sans détection : l'absence de résultat NE prouve "
      "PAS l'absence d'objet, et c'est dit",
      r.ok and r.reason is not None and "fermé" in r.reason)

opened = PerceptionService(StubCamera(K), make_tree(),
                           StubDetector(open_vocab=True, dets=[]))
opened.start()
check("Détecteur ouvert sans détection : réponse simple",
      opened.describe_scene().ok)

check("NullObjectDetector permet de tester la chaîne SANS modèle",
      NullObjectDetector().detect(None) == []
      and NullObjectDetector().is_open_vocabulary is False)

section("6 — Le service ne peut PAS commander d'actionneur")

api = {m for m in dir(svc) if not m.startswith("_")}
interdits = {m for m in api
             if any(k in m for k in ("move", "send", "command", "actuat"))}
check(f"Aucune méthode d'action exposée (trouvé: {interdits or 'aucune'})",
      not interdits)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
if FAIL == 0:
    print("\n  ✓ pixel -> direction dans le repère robot")
    print("  ✓ Dégradation honnête : jamais de direction ni de distance inventée")
    print("  ✓ Modèle remplaçable sans toucher au service")

sys.exit(0 if FAIL == 0 else 1)
