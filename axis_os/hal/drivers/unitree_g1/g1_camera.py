"""
AXIS-OS — Driver caméra Intel RealSense pour le Unitree G1.

CHEMIN MATÉRIEL, à ne pas confondre avec celui des moteurs : la caméra
est branchée en USB sur le Jetson embarqué et se lit via librealsense
(`pyrealsense2`). Elle ne passe PAS par le bus DDS/Ethernet de
g1_dds_actuator.py. Les deux chemins sont indépendants : la caméra peut
fonctionner alors que les moteurs sont injoignables, et réciproquement.

C'est la configuration de référence de NVIDIA pour G1 + GR00T (RealSense
D435 sur le robot). Ce driver ne suppose cependant aucun modèle précis :
il interroge le capteur présent.

CAPTURE EN ARRIÈRE-PLAN — exigence de conception, pas un détail :
`capture()` ne bloque JAMAIS. Un thread dédié lit le flux et conserve la
dernière image ; `capture()` rend celle-ci immédiatement, ou None si
aucune n'est encore arrivée. Une boucle de contrôle qui attendrait
l'image suivante à 30 Hz s'imposerait 33 ms de latence par appel — plus
de quatre fois le budget complet d'un cycle mesuré (6.87 ms, voir
docs/latence_chemin_commande.md).

CALIBRATION : get_intrinsics() interroge le CAPTEUR. Les valeurs sont
propres à chaque exemplaire, jamais au modèle — deux G1 sortis de la
même chaîne n'ont pas la même calibration. Rien n'est codé en dur ici.
"""
from __future__ import annotations

import threading
import time
from typing import Optional

from axis_os.hal.core.interfaces import ICamera
from axis_os.hal.core.types import (
    CameraFrame, CameraIntrinsics, HALStatus, SensorType,
)


class G1RealSenseCamera(ICamera):
    """Caméra RealSense du G1, via librealsense.

    Usage :
        cam = G1RealSenseCamera("head_rgb", width=640, height=480, fps=30)
        if cam.initialize() == HALStatus.OK:
            frame = cam.capture()          # non bloquant, peut rendre None
            k = cam.get_intrinsics()       # calibration réelle du capteur
    """

    def __init__(
        self,
        sensor_id: str = "g1_head_rgb",
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        serial_number: Optional[str] = None,
        depth: bool = False,
    ):
        """
        serial_number : cible un exemplaire précis si plusieurs caméras
            sont branchées. None = la première trouvée, ce qui est
            ambigu dès qu'il y en a deux — à renseigner explicitement
            sur un robot multi-caméras.
        depth : lit le flux de profondeur plutôt que la couleur. Les
            deux flux d'un même capteur ont des calibrations
            DIFFÉRENTES et ne sont pas alignés par défaut ; instancier
            deux objets distincts si les deux sont nécessaires.
        """
        super().__init__(
            sensor_id,
            SensorType.CAMERA_DEPTH if depth else SensorType.CAMERA_RGB,
        )
        self._width = width
        self._height = height
        self._fps = fps
        self._serial = serial_number
        self._depth = depth

        self._pipeline = None
        self._intrinsics: Optional[CameraIntrinsics] = None
        self._latest: Optional[CameraFrame] = None
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._consecutive_errors = 0
        self._frames_received = 0

    # ---- cycle de vie ----------------------------------------------------

    def initialize(self) -> HALStatus:
        # Import local, même raison que dans g1_dds_actuator : ce module
        # reste importable et lisible sur une machine sans le SDK, et
        # le coût d'import n'est payé qu'à l'usage réel.
        try:
            import pyrealsense2 as rs
        except ImportError:
            print(f"  [{self.sensor_id}] pyrealsense2 absent — "
                  f"`pip install pyrealsense2`. Caméra non initialisée.")
            return HALStatus.ERROR_COMM

        try:
            config = rs.config()
            if self._serial:
                config.enable_device(self._serial)
            stream = rs.stream.depth if self._depth else rs.stream.color
            fmt = rs.format.z16 if self._depth else rs.format.rgb8
            config.enable_stream(stream, self._width, self._height,
                                 fmt, self._fps)

            self._pipeline = rs.pipeline()
            profile = self._pipeline.start(config)

            # Calibration lue SUR LE CAPTEUR. C'est tout l'intérêt :
            # ces valeurs sont propres à cet exemplaire.
            vs = profile.get_stream(stream).as_video_stream_profile()
            intr = vs.get_intrinsics()
            self._intrinsics = CameraIntrinsics(
                fx=float(intr.fx), fy=float(intr.fy),
                cx=float(intr.ppx), cy=float(intr.ppy),
                distortion=tuple(float(c) for c in intr.coeffs),
            )
        except Exception as exc:        # noqa: BLE001 — matériel absent/occupé
            print(f"  [{self.sensor_id}] échec d'initialisation : {exc}")
            self._pipeline = None
            return HALStatus.ERROR_COMM

        self._stop.clear()
        self._thread = threading.Thread(
            target=self._capture_loop,
            name=f"camera-{self.sensor_id}",
            daemon=True,
        )
        self._thread.start()
        self._initialized = True
        print(f"  [{self.sensor_id}] RealSense initialisée — "
              f"{self._width}x{self._height}@{self._fps}Hz, "
              f"fx={self._intrinsics.fx:.1f} fy={self._intrinsics.fy:.1f}")
        return HALStatus.OK

    def shutdown(self) -> HALStatus:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:           # noqa: BLE001 — déjà arrêté
                pass
            self._pipeline = None
        self._initialized = False
        return HALStatus.OK

    def is_healthy(self) -> bool:
        """Saine = initialisée, thread vivant, et pas d'échecs répétés.

        Le seuil sur _consecutive_errors évite de déclarer saine une
        caméra dont le thread tourne mais dont chaque lecture échoue —
        un capteur débranché à chaud se comporte exactement comme ça.
        """
        return (
            self._initialized
            and self._thread is not None
            and self._thread.is_alive()
            and self._consecutive_errors < 10
        )

    # ---- lecture ---------------------------------------------------------

    def capture(self) -> Optional[CameraFrame]:
        """Dernière image reçue, ou None. NE BLOQUE JAMAIS.

        Rend potentiellement la MÊME image sur deux appels rapprochés si
        aucune nouvelle n'est arrivée : c'est voulu et cohérent avec
        `timestamp_ns`, qui permet à l'appelant de détecter une image
        périmée — même principe que l'échéance de fraîcheur de
        SetpointFollower.
        """
        with self._lock:
            return self._latest

    def get_intrinsics(self) -> Optional[CameraIntrinsics]:
        """Calibration réelle du capteur, ou None si non initialisé.

        Ne fabrique JAMAIS de valeurs plausibles à partir de la
        résolution : un service de perception qui reçoit None doit
        dégrader honnêtement, pas travailler sur une calibration
        inventée qu'une préhension prendrait au sérieux.
        """
        return self._intrinsics

    @property
    def frames_received(self) -> int:
        return self._frames_received

    # ---- thread de capture -----------------------------------------------

    def _capture_loop(self) -> None:
        import pyrealsense2 as rs      # noqa: F401 — déjà validé

        while not self._stop.is_set():
            if self._pipeline is None:
                break
            try:
                frames = self._pipeline.wait_for_frames(timeout_ms=1000)
                f = (frames.get_depth_frame() if self._depth
                     else frames.get_color_frame())
                if not f:
                    self._consecutive_errors += 1
                    continue

                frame = CameraFrame(
                    width=f.get_width(),
                    height=f.get_height(),
                    encoding="depth16" if self._depth else "rgb8",
                    data=bytes(f.get_data()),
                    # Horodatage DU CAPTEUR quand il est disponible, pas
                    # l'instant de réception : sur un robot qui bouge,
                    # quelques dizaines de ms d'écart déplacent un objet
                    # de plusieurs centimètres dans le repère monde.
                    timestamp_ns=int(f.get_timestamp() * 1e6),
                )
                with self._lock:
                    self._latest = frame
                self._frames_received += 1
                self._consecutive_errors = 0

            except Exception:           # noqa: BLE001 — flux interrompu
                self._consecutive_errors += 1
                # Ne pas boucler à plein régime sur un capteur mort :
                # même erreur que la pompe Isaac Sim qui monopolisait le
                # GIL (voir docs/latence_chemin_commande.md §4).
                time.sleep(0.05)
