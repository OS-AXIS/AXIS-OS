"""
AXIS-OS — Driver de la caméra Intel RealSense du Unitree G1.

Ce driver lit la caméra du G1 (l'humanoïde Unitree G1, robot de
référence d'AXIS-OS). Intel RealSense est une gamme de caméras couleur
et de profondeur ; librealsense est la bibliothèque d'Intel qui les
pilote, utilisée ici depuis Python par le module `pyrealsense2`.

La caméra suit un chemin matériel distinct de celui des moteurs, à ne
pas confondre : elle est branchée en USB sur le Jetson embarqué
(l'ordinateur de bord NVIDIA du robot) et se lit via librealsense. Elle
ne passe pas par le réseau DDS sur Ethernet (Data Distribution Service,
le système de messages qui commande les moteurs, voir
g1_dds_actuator.py). Les deux chemins sont indépendants : la caméra peut
fonctionner alors que les moteurs sont injoignables, et inversement.

C'est la configuration de référence de NVIDIA pour faire tourner GR00T
(son modèle d'IA pour robots humanoïdes, voir docs/groot_architecture.md)
sur un G1 : une RealSense D435 montée sur le robot. Ce driver ne suppose
cependant aucun modèle précis : il interroge le capteur présent.

Capture en arrière-plan, une exigence de conception et non un détail :
`capture()` ne bloque JAMAIS. Un thread dédié lit le flux et conserve la
dernière image ; `capture()` rend celle-ci immédiatement, ou None si
aucune n'est encore arrivée. Pourquoi : une boucle de contrôle qui
attendrait l'image suivante à 30 Hz s'imposerait 33 ms de latence par
appel, soit plus de quatre fois le budget complet d'un cycle de commande
mesuré (6.87 ms, voir docs/latence_chemin_commande.md).

Calibration : get_intrinsics() interroge le capteur. Les paramètres
intrinsèques (focales, centre optique, distorsion de l'objectif) sont
propres à chaque exemplaire, jamais au modèle : deux G1 sortis de la
même chaîne n'ont pas la même calibration. Rien n'est donc codé en dur
ici.
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
    """Caméra RealSense du G1, lue via librealsense.

    Implémente ICamera, l'interface commune à toutes les caméras dans la
    HAL (couche d'abstraction matérielle : la partie d'AXIS-OS qui cache
    aux applications les détails propres à chaque robot).

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
        serial_number : numéro de série d'un exemplaire précis, à donner
            si plusieurs caméras sont branchées. Avec None, c'est la
            première caméra trouvée qui est lue, ce qui devient ambigu
            dès qu'il y en a deux : à renseigner explicitement sur un
            robot à plusieurs caméras.
        depth : lit le flux de profondeur au lieu du flux couleur. Les
            deux flux d'un même capteur ont des calibrations différentes
            et ne sont pas alignés par défaut : créer deux objets
            distincts si les deux sont nécessaires.
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
        # Import local, pour la même raison que dans g1_dds_actuator.py :
        # ce module reste importable et lisible sur une machine où le SDK
        # (kit de développement, ici pyrealsense2) n'est pas installé, et
        # le coût de l'import n'est payé qu'à l'usage réel.
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

            # Calibration lue sur le capteur lui-même : c'est tout
            # l'intérêt, ces valeurs sont propres à cet exemplaire.
            # librealsense fournit les focales (fx, fy, en pixels), le
            # point principal, c'est-à-dire le centre optique (ppx, ppy,
            # rangés dans cx, cy) et les coefficients de distorsion.
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
        """La caméra est saine si elle est initialisée, si son thread de
        capture est vivant et si les lectures n'échouent pas à répétition.

        Pourquoi le seuil sur _consecutive_errors (10 échecs de suite) :
        sans lui, une caméra dont le thread tourne mais dont chaque
        lecture échoue serait déclarée saine. C'est exactement ainsi que
        se comporte un capteur débranché à chaud.
        """
        return (
            self._initialized
            and self._thread is not None
            and self._thread.is_alive()
            and self._consecutive_errors < 10
        )

    # ---- lecture ---------------------------------------------------------

    def capture(self) -> Optional[CameraFrame]:
        """Dernière image reçue, ou None. Ne bloque JAMAIS.

        Peut rendre la même image à deux appels rapprochés si aucune
        nouvelle n'est arrivée entre-temps. C'est voulu, et cohérent avec
        `timestamp_ns` (l'horodatage de l'image), qui permet à l'appelant
        de repérer une image périmée : même principe que l'échéance de
        fraîcheur de SetpointFollower, le suiveur de consignes qui sait
        reconnaître une consigne trop ancienne
        (axis_os/services/policy/setpoint_follower.py).
        """
        with self._lock:
            return self._latest

    def get_intrinsics(self) -> Optional[CameraIntrinsics]:
        """Calibration réelle du capteur, ou None si la caméra n'est pas
        initialisée.

        Ne fabrique JAMAIS de valeurs plausibles à partir de la
        résolution. Pourquoi : un service de perception qui reçoit None
        doit passer honnêtement en mode dégradé, plutôt que travailler
        sur une calibration inventée qu'une préhension (la saisie d'un
        objet par le robot) prendrait au sérieux.
        """
        return self._intrinsics

    @property
    def frames_received(self) -> int:
        return self._frames_received

    # ---- thread de capture -----------------------------------------------

    def _capture_loop(self) -> None:
        """Boucle du thread de capture : lit les images en continu et
        garde la dernière, jusqu'à l'appel de shutdown()."""
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
                    # Horodatage donné par le capteur quand il est
                    # disponible, et non l'instant de réception.
                    # Pourquoi : sur un robot qui bouge, quelques dizaines
                    # de ms d'écart déplacent un objet de plusieurs
                    # centimètres dans le repère du monde.
                    # get_timestamp() est en millisecondes, d'où la
                    # conversion en nanosecondes.
                    timestamp_ns=int(f.get_timestamp() * 1e6),
                )
                with self._lock:
                    self._latest = frame
                self._frames_received += 1
                self._consecutive_errors = 0

            except Exception:           # noqa: BLE001 — flux interrompu
                self._consecutive_errors += 1
                # Ne pas boucler à plein régime sur un capteur mort. Ce
                # serait la même erreur que celle de la boucle qui fait
                # avancer le simulateur Isaac Sim de NVIDIA
                # (MainThreadSimPump) : elle tournait à vide sans jamais
                # céder le GIL, le verrou global de l'interpréteur Python
                # qui empêche les autres threads d'avancer (voir
                # docs/latence_chemin_commande.md, section 4).
                time.sleep(0.05)
