"""
AXIS-OS — Tests ICamera/IMicrophone/ISpeaker et faux drivers
==================================================================
Valide les 3 nouvelles interfaces HAL (axis_os/hal/core/interfaces.py)
et FakeCamera/FakeMicrophone/FakeSpeaker (axis_os/hal/drivers/
sim_common/sim_perception.py) — dernière étape de l'ordre demandé
pour cette session sur l'architecture services système.

Point testé explicitement en section 1 : FakeCamera rejoue de VRAIES
images PPM écrites sur disque, pas des données synthétiques générées
en mémoire — conforme à la consigne "pas de driver caméra réel pour
le G1, seulement le faux qui rejoue des images depuis le disque".
"""

import sys
import tempfile
import shutil
sys.path.insert(0, ".")

from pathlib import Path
from axis_os.hal.core.interfaces import ICamera, IMicrophone, ISpeaker
from axis_os.hal.core.types import CameraFrame, AudioChunk, HALStatus
from axis_os.hal.drivers.sim_common.sim_perception import (
    FakeCamera, FakeMicrophone, FakeSpeaker,
)

FAIL = 0
TMP_DIR = tempfile.mkdtemp(prefix="axis_os_perception_test_")


def check(description: str, condition: bool) -> None:
    global FAIL
    status = "✓" if condition else "✗"
    print(f"  [{status}] {description}")
    if not condition:
        FAIL += 1


def section(title: str) -> None:
    print(f"\n{'=' * 66}\n  {title}\n{'=' * 66}")


def write_ppm(path: Path, width: int, height: int, color: tuple) -> None:
    with open(path, "wb") as f:
        f.write(b"P6\n")
        f.write(f"{width} {height}\n".encode())
        f.write(b"255\n")
        f.write(bytes(color) * (width * height))


# ══════════════════════════════════════════════════════════════════

section("0 — Les 3 interfaces sont abstraites, ICamera/IMicrophone restent des ISensor")

from axis_os.hal.core.interfaces import ISensor
check("ICamera hérite de ISensor (capteur en lecture, pas d'actionneur)",
      issubclass(ICamera, ISensor))
check("IMicrophone hérite de ISensor", issubclass(IMicrophone, ISensor))
check("ISpeaker n'hérite PAS de ISensor (émet le monde, ne le perçoit pas)",
      not issubclass(ISpeaker, ISensor))

for cls in (ICamera, IMicrophone, ISpeaker):
    try:
        cls("test")
        check(f"{cls.__name__} correctement abstraite", False)
    except TypeError:
        check(f"{cls.__name__} correctement abstraite", True)

# ══════════════════════════════════════════════════════════════════

section("1 — FakeCamera rejoue de VRAIES images depuis le disque (pas synthétique)")

img_dir = Path(TMP_DIR) / "images"
img_dir.mkdir()
write_ppm(img_dir / "001_red.ppm", 4, 4, (255, 0, 0))
write_ppm(img_dir / "002_blue.ppm", 4, 4, (0, 0, 255))
write_ppm(img_dir / "003_green.ppm", 4, 4, (0, 255, 0))

cam = FakeCamera("test_cam", img_dir)
check("initialize() réussit", cam.initialize() == HALStatus.OK)
check("is_healthy() vrai avec des images présentes", cam.is_healthy() is True)

frame1 = cam.capture()
check("Frame 1 : dimensions correctes (4x4)", frame1.width == 4 and frame1.height == 4)
check("Frame 1 : encodage rgb8", frame1.encoding == "rgb8")
check("Frame 1 : premier pixel EXACTEMENT celui écrit sur disque (rouge)",
      frame1.data[:3] == bytes([255, 0, 0]))

frame2 = cam.capture()
check("Frame 2 : deuxième vraie image (bleue), pas la même que la 1ère",
      frame2.data[:3] == bytes([0, 0, 255]))

frame3 = cam.capture()
check("Frame 3 : troisième vraie image (verte)",
      frame3.data[:3] == bytes([0, 255, 0]))

frame4 = cam.capture()
check("Frame 4 : boucle bien, revient à la 1ère image (rouge)",
      frame4.data[:3] == bytes([255, 0, 0]))

# ══════════════════════════════════════════════════════════════════

section("2 — FakeCamera : robustesse (dossier absent, fichier corrompu)")

cam_missing = FakeCamera("missing_cam", Path(TMP_DIR) / "dossier_absent")
cam_missing.initialize()
check("Dossier absent : is_healthy() == False", cam_missing.is_healthy() is False)
check("Dossier absent : capture() retourne None (pas d'exception)",
      cam_missing.capture() is None)

corrupt_dir = Path(TMP_DIR) / "corrupt"
corrupt_dir.mkdir()
with open(corrupt_dir / "bad.ppm", "wb") as f:
    f.write(b"CECI N'EST PAS UN FICHIER PPM VALIDE")
cam_corrupt = FakeCamera("corrupt_cam", corrupt_dir)
cam_corrupt.initialize()
check("Fichier PPM corrompu : capture() échoue proprement, pas d'exception",
      cam_corrupt.capture() is None)

# ══════════════════════════════════════════════════════════════════

section("3 — FakeMicrophone / FakeSpeaker")

mic = FakeMicrophone("test_mic", sample_rate=16000)
mic.initialize()
chunk = mic.capture(0.5)
check("AudioChunk : sample_rate correct", chunk.sample_rate == 16000)
check("AudioChunk : durée correcte (0.5s à 16kHz, 16-bit)",
      len(chunk.data) == int(16000 * 0.5) * 2)

spk = FakeSpeaker("test_speaker")
spk.initialize()
status = spk.play(chunk)
check("play() réussit", status == HALStatus.OK)
check("play() enregistre bien le chunk pour inspection", spk.last_played is chunk)
check("play_count incrémenté", spk.play_count == 1)

status_uninit = FakeSpeaker("uninit_speaker").play(chunk)
check("play() sur un haut-parleur non initialisé échoue proprement",
      status_uninit == HALStatus.ERROR_NOT_INIT)

# ══════════════════════════════════════════════════════════════════

shutil.rmtree(TMP_DIR, ignore_errors=True)

# ══════════════════════════════════════════════════════════════════

section("CameraIntrinsics — pixel vers rayon")

from axis_os.hal.core.types import CameraIntrinsics

_k = CameraIntrinsics(fx=600.0, fy=600.0, cx=320.0, cy=240.0)

_centre = _k.pixel_to_ray(320.0, 240.0)
check("Le centre optique donne un rayon droit devant (x=0, y=0, z=1)",
      abs(_centre.x) < 1e-9 and abs(_centre.y) < 1e-9 and _centre.z == 1.0)

_coin = _k.pixel_to_ray(0.0, 0.0)
check("Le coin haut-gauche part vers la gauche ET vers le haut "
      "(convention optique : y vers le bas, donc y negatif en haut)",
      _coin.x < 0 and _coin.y < 0)

_droite = _k.pixel_to_ray(640.0, 240.0)
check("Un pixel a droite du centre donne un rayon vers la droite",
      _droite.x > 0 and abs(_droite.y) < 1e-9)

# Une focale plus longue = champ plus etroit = meme pixel plus proche
# de l'axe optique. Verifie que fx entre bien au denominateur.
_tele = CameraIntrinsics(fx=1200.0, fy=1200.0, cx=320.0, cy=240.0)
check("Doubler la focale halve l'angle du meme pixel (fx au denominateur)",
      abs(_tele.pixel_to_ray(640.0, 240.0).x - _droite.x / 2) < 1e-9)

section("get_intrinsics() — l'absence de calibration doit se DIRE")

_cam_nue = FakeCamera("cam_sans_calib", Path("/tmp/axisos_inexistant"))
check("FakeCamera sans calibration retourne None, pas des valeurs inventees",
      _cam_nue.get_intrinsics() is None)

_cam_cal = FakeCamera("cam_calib", Path("/tmp/axisos_inexistant"), intrinsics=_k)
check("FakeCamera avec calibration la restitue telle quelle",
      _cam_cal.get_intrinsics() is not None
      and _cam_cal.get_intrinsics().fx == 600.0)

check("ICamera impose get_intrinsics() (methode abstraite)",
      "get_intrinsics" in ICamera.__abstractmethods__)

# ══════════════════════════════════════════════════════════════════

section("Caméra de profondeur : le mode doit être ACCESSIBLE")

# Ce test existe parce que capture_depth() et l'activation de
# l'annotateur avaient été écrits AVANT le paramètre qui les active :
# self._depth était lu mais aucun appelant ne pouvait le mettre à True.
# Une caméra de profondeur était donc impossible à construire, et
# l'erreur n'est apparue qu'en simulation.
import inspect as _inspect
try:
    from axis_os.hal.drivers.isaac_sim.sim_camera import IsaacSimCamera
    _params = _inspect.signature(IsaacSimCamera.__init__).parameters
    check("IsaacSimCamera accepte depth= (sinon capture_depth() est "
          "inatteignable)", "depth" in _params)
    check("IsaacSimCamera expose capture_depth()",
          hasattr(IsaacSimCamera, "capture_depth"))
except ImportError:
    # Module dépendant d'Isaac Sim : absent hors conteneur, ce n'est pas
    # un échec.
    print("  [—] IsaacSimCamera non importable hors du conteneur Isaac Sim")

check("FakeCamera aussi peut être interrogée sans profondeur",
      not hasattr(FakeCamera("c", Path("/tmp/x")), "capture_depth")
      or FakeCamera("c", Path("/tmp/x")).capture_depth() is None)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)

if FAIL == 0:
    print("\n  ✓ ICamera/IMicrophone/ISpeaker correctement définies et abstraites")
    print("  ✓ FakeCamera rejoue de VRAIES images PPM depuis le disque")
    print("  ✓ Robuste à un dossier absent ou un fichier corrompu")
    print("  ✓ CameraIntrinsics convertit pixel -> rayon, absence de calibration explicite")

sys.exit(0 if FAIL == 0 else 1)
