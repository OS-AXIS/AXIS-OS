"""
AXIS-OS — Caméra virtuelle Isaac Sim, via l'interface ICamera.

MÊME CONTRAT que G1RealSenseCamera : une app ou un service de
perception ne voit AUCUNE différence entre une caméra simulée et la
caméra réelle du robot. C'est tout l'intérêt d'avoir défini ICamera
avant d'écrire le moindre driver — la chaîne perception peut être
développée et validée entièrement en simulation, puis basculée sur le
matériel en changeant une seule ligne d'instanciation.

DIFFÉRENCE ASSUMÉE avec la RealSense : ici la capture est SYNCHRONE.
Isaac Sim rend l'image sur demande, depuis le thread principal, et
n'a pas de flux continu à consommer en arrière-plan. Le contrat
ICamera est respecté (capture() ne bloque pas sur un flux réseau),
mais le coût d'un rendu est réel — voir capture() pour le détail.

CALIBRATION : Isaac Sim expose les paramètres de la caméra virtuelle
(focale, ouverture, résolution) ; get_intrinsics() les convertit en
CameraIntrinsics. Une caméra simulée a une optique PARFAITE, donc
aucune distorsion — c'est une différence réelle avec le matériel, et
l'une des raisons pour lesquelles un modèle de perception validé en
simulation peut échouer sur une vraie image.
"""
from __future__ import annotations

import math
from typing import Optional

from axis_os.hal.core.interfaces import ICamera
from axis_os.hal.core.types import (
    CameraFrame, CameraIntrinsics, HALStatus, SensorType,
)


class IsaacSimCamera(ICamera):
    """Caméra virtuelle attachée à un prim de la scène Isaac Sim.

    Usage (THREAD PRINCIPAL uniquement, comme tout appel Isaac Sim) :

        cam = IsaacSimCamera("sim_head_rgb",
                             prim_path="/World/g1/head_camera",
                             position=(0.1, 0.0, 0.5))
        cam.initialize()
        bridge.step(1)          # un rendu doit avoir eu lieu
        frame = cam.capture()
    """

    def __init__(
        self,
        sensor_id: str = "sim_head_rgb",
        prim_path: str = "/World/axis_camera",
        width: int = 640,
        height: int = 480,
        position: tuple = (0.0, 0.0, 1.0),
        orientation: Optional[tuple] = None,
        focal_length_mm: float = 24.0,
        horizontal_aperture_mm: float = 20.955,
        depth: bool = False,
    ):
        """
        focal_length_mm / horizontal_aperture_mm : valeurs par défaut
            d'Isaac Sim (capteur 35 mm). Elles déterminent le champ de
            vision, donc la calibration rendue par get_intrinsics().
        orientation : quaternion (w, x, y, z) du prim caméra. None =
            orientation par défaut du prim.
        """
        # depth=True : caméra de PROFONDEUR. capture() rend toujours
        # l'image couleur ; capture_depth() rend la carte de distances,
        # et n'est active que dans ce mode. Deux instances distinctes
        # pour couleur et profondeur, à la MÊME position, comme sur une
        # RealSense dont les deux flux sont alignés.
        self._depth = depth
        super().__init__(
            sensor_id,
            SensorType.CAMERA_DEPTH if depth else SensorType.CAMERA_RGB,
        )
        self._prim_path = prim_path
        self._width = width
        self._height = height
        self._position = position
        self._orientation = orientation
        self._focal_mm = focal_length_mm
        self._aperture_mm = horizontal_aperture_mm

        self._camera = None
        self._frames_captured = 0
        self._last_error: Optional[str] = None

    # ---- cycle de vie ----------------------------------------------------

    def initialize(self) -> HALStatus:
        try:
            from isaacsim.sensors.camera import Camera
            import numpy as np
        except ImportError as exc:
            self._last_error = f"Isaac Sim indisponible : {exc}"
            print(f"  [{self.sensor_id}] {self._last_error}")
            return HALStatus.ERROR_COMM

        try:
            kwargs = dict(
                prim_path=self._prim_path,
                resolution=(self._width, self._height),
                position=np.array(self._position),
            )
            if self._orientation is not None:
                kwargs["orientation"] = np.array(self._orientation)
            self._camera = Camera(**kwargs)
            self._camera.initialize()
            if self._depth:
                # La profondeur n'est PAS rendue par défaut : il faut
                # activer l'annotateur correspondant. Sans lui,
                # get_depth() rend None et la perception ne peut donner
                # que des directions, jamais de distances — donc un
                # robot qui ne sait pas quand s'arrêter devant un objet.
                self._camera.add_distance_to_image_plane_to_frame()
            self._camera.set_focal_length(self._focal_mm / 10.0)
            self._camera.set_horizontal_aperture(self._aperture_mm / 10.0)
        except Exception as exc:        # noqa: BLE001
            self._last_error = f"création de la caméra : {exc}"
            print(f"  [{self.sensor_id}] échec — {self._last_error}")
            self._camera = None
            return HALStatus.ERROR_COMM

        self._initialized = True
        print(f"  [{self.sensor_id}] caméra Isaac Sim créée sur "
              f"{self._prim_path} — {self._width}x{self._height}")
        return HALStatus.OK

    def wait_until_ready(self, world, max_frames: int = 120) -> bool:
        """Rend des images jusqu'à ce que la caméra produise VRAIMENT des
        données, ou renonce après max_frames.

        NÉCESSAIRE, et constaté en exécution réelle : Isaac Sim avertit
        lui-même que « Annotator 'rgb' returned None or unexpected shape
        — a few render frames may be required before data is available ».
        Un nombre fixe de step() est un pari : l'annotateur peut mettre
        plus ou moins de temps selon la scène, la charge GPU et le moment
        où la caméra a été créée par rapport au reste.

        Le piège que cette méthode ferme : capture() rendait un tableau
        de la BONNE TAILLE mais entièrement noir. Rien dans la forme des
        données ne signalait l'anomalie — seul le contenu le trahissait.
        On attend donc une image réellement non nulle, pas un compteur.
        """
        for i in range(max_frames):
            world.step(render=True)
            frame = self.capture()
            if frame is not None and any(frame.data[:4096]):
                print(f"  [{self.sensor_id}] premières données après "
                      f"{i + 1} image(s) rendue(s).")
                return True
        print(f"  [{self.sensor_id}] AUCUNE donnée après {max_frames} "
              f"images — la caméra rend du noir. Vérifier qu'une source "
              f"de lumière existe dans la scène.")
        return False

    def shutdown(self) -> HALStatus:
        self._camera = None
        self._initialized = False
        return HALStatus.OK

    def is_healthy(self) -> bool:
        return self._initialized and self._camera is not None

    # ---- lecture ---------------------------------------------------------

    def capture_depth(self) -> Optional["np.ndarray"]:
        """Carte de distances, en mètres, une valeur par pixel.

        « distance_to_image_plane » : la distance mesurée PERPENDICULAIREMENT
        au plan de l'image, pas la distance en ligne droite jusqu'au point.
        Pour un pixel au centre les deux coïncident ; sur les bords, la
        distance vraie est plus grande. C'est exactement la convention
        attendue par pixel_to_ray(), dont le rayon a z=1 : multiplier le
        rayon par cette valeur donne directement la position 3D.

        Rend None si la caméra n'a pas été créée avec depth=True, ou si
        aucune image n'est encore disponible — jamais une distance
        inventée.
        """
        if self._camera is None or not self._depth:
            return None
        try:
            import numpy as np
            d = self._camera.get_depth()
        except Exception:               # noqa: BLE001
            return None
        if d is None or getattr(d, "size", 0) == 0:
            return None
        return np.asarray(d, dtype=np.float32)

    def capture(self) -> Optional[CameraFrame]:
        """Dernière image rendue par Isaac Sim, ou None.

        Rend None tant qu'aucun `world.step(render=True)` n'a eu lieu :
        la caméra ne produit une image qu'au rendu, et un rendu ne se
        déclenche pas tout seul. Ce n'est pas une erreur — un capteur
        qui n'a pas encore de donnée rend None, comme la RealSense
        avant sa première image.

        ATTENTION : contrairement à G1RealSenseCamera, il n'y a pas de
        thread de capture ici. Le coût d'un rendu Isaac Sim est porté
        par le `step()` du thread principal, pas par cet appel — c'est
        déjà comptabilisé dans les 4.26 ms de « pas de physique »
        mesurés dans docs/latence_chemin_commande.md, qui deviennent
        nettement plus élevés dès que render=True.
        """
        if self._camera is None:
            return None
        try:
            rgba = self._camera.get_rgba()
        except Exception as exc:        # noqa: BLE001
            self._last_error = str(exc)
            return None

        if rgba is None or getattr(rgba, "size", 0) == 0:
            return None

        # Isaac Sim rend du RGBA ; ICamera/CameraFrame transportent du
        # rgb8 pour rester alignés sur la RealSense et sur
        # sensor_msgs/Image. On retire le canal alpha plutôt que
        # d'inventer un encodage propre à la simulation, sinon tout
        # consommateur devrait traiter deux formats selon la source.
        import numpy as np
        rgb = np.asarray(rgba)[:, :, :3].astype(np.uint8)
        h, w = rgb.shape[0], rgb.shape[1]
        self._frames_captured += 1

        return CameraFrame(
            width=int(w), height=int(h),
            encoding="rgb8", data=rgb.tobytes(),
        )

    def get_intrinsics(self) -> Optional[CameraIntrinsics]:
        """Calibration déduite des paramètres de la caméra virtuelle.

        fx = largeur_px * focale / ouverture_horizontale. Le centre
        optique est exactement au centre de l'image et la distorsion est
        vide : une optique simulée est PARFAITE. C'est une différence
        réelle avec le matériel — une vraie caméra a un centre décalé de
        quelques pixels et une distorsion non nulle — et l'une des
        raisons pour lesquelles une perception validée en simulation
        peut échouer sur une image réelle.
        """
        if not self._initialized:
            return None
        fx = self._width * self._focal_mm / self._aperture_mm
        # fy = fx tant que les pixels sont carrés, ce qui est le cas en
        # simulation. Sur un vrai capteur, fy se lit séparément.
        return CameraIntrinsics(
            fx=fx, fy=fx,
            cx=self._width / 2.0, cy=self._height / 2.0,
            distortion=(),
        )

    def look_at(self, target=(0.0, 0.0, 0.0), tolerance: float = 0.99) -> bool:
        """Oriente la caméra vers un point du monde, et VÉRIFIE le résultat.

        HISTORIQUE, constaté en exécution réelle : la première version
        rendait True alors que la caméra regardait à 90° de la cible,
        selon (0, +1, 0) au lieu de (-0.99, 0, -0.26). Cause : la classe
        Camera d'Isaac Sim convertit les orientations entre trois
        conventions d'axes (world, ros, usd), et on lui passait une
        cible en déclarant la mauvaise. Aucune caméra n'a donc jamais
        regardé le robot — d'où des images de ciel et de sol, et zéro
        détection pour OWL-ViT, qui n'avait simplement rien à voir.

        Désormais : on calcule nous-mêmes l'orientation en convention
        USD (la caméra regarde selon -Z, +Y vers le haut de l'image), on
        la déclare explicitement comme telle, puis on RELIT la
        transformation réellement composée dans la scène. True n'est
        rendu que si l'axe de visée mesuré pointe vers la cible à
        `tolerance` près (cosinus). Une réussite qui n'est pas vérifiée
        n'est plus possible.
        """
        if self._camera is None:
            return False
        import numpy as np

        eye = np.array(self._position, dtype=float)
        tgt = np.array(target, dtype=float)
        fwd = tgt - eye
        n = float(np.linalg.norm(fwd))
        if n < 1e-9:
            return False
        fwd = fwd / n
        world_up = np.array([0.0, 0.0, 1.0])
        if abs(float(np.dot(fwd, world_up))) > 0.999:
            world_up = np.array([0.0, 1.0, 0.0])
        right = np.cross(fwd, world_up)
        right = right / np.linalg.norm(right)
        up = np.cross(right, fwd)
        # Colonnes = axes de la caméra exprimés dans le monde :
        #   +X caméra = droite, +Y caméra = haut, +Z caméra = ARRIÈRE
        # (une caméra USD regarde selon -Z).
        q = _matrix_to_quat_wxyz(np.column_stack((right, up, -fwd)))

        tentatives = [
            ("orientation USD déclarée USD",
             lambda: self._camera.set_world_pose(
                 position=eye, orientation=np.array(q), camera_axes="usd")),
            ("cible, convention monde",
             lambda: self._camera.set_world_pose(
                 position=eye, target=tgt, camera_axes="world")),
        ]
        for nom, appliquer in tentatives:
            try:
                appliquer()
            except Exception:           # noqa: BLE001 — signature absente
                continue
            cos = self._measured_alignment(fwd)
            if cos is not None and cos >= tolerance:
                self._last_look_at = (nom, cos)
                return True
        cos = self._measured_alignment(fwd)
        print(f"  [{self.sensor_id}] look_at ÉCHOUÉ : axe de visée mesuré "
              f"aligné à {cos} avec la cible (attendu >= {tolerance}). La "
              f"caméra ne regarde PAS la cible — refus plutôt qu'un succès "
              f"silencieux.")
        return False

    def optical_transform(self, parent: str = "world",
                          child: Optional[str] = None):
        """Transformation `parent` <- repère OPTIQUE de la caméra, lue
        dans la scène.

        NÉCESSAIRE, et découvert par la mesure : déclarer la caméra dans
        l'arbre des repères avec sa seule POSITION, sans son
        orientation, projette toutes les détections à l'horizontale. Une
        caméra inclinée de 28° vers le bas plaçait ainsi un colis posé à
        0.20 m de haut à 0.99 m — soit exactement sa propre hauteur. Les
        coordonnées x et y restaient justes à 2 cm près, ce qui rend
        l'erreur d'autant plus discrète.

        Sur le robot réel, cette pose doit venir de la cinématique (où
        la caméra est montée, comment la tête est orientée), jamais
        d'une valeur supposée.

        Le repère optique (x droite, y BAS, z avant) n'est pas celui du
        prim caméra USD (x droite, y haut, z ARRIÈRE) : les deux
        diffèrent d'une rotation d'un demi-tour autour de x, appliquée
        ici. C'est la même distinction que camera_optical_to_link(), mais
        mesurée sur la scène plutôt que supposée.
        """
        try:
            import numpy as np
            import omni.usd
            from pxr import Usd, UsdGeom
            from axis_os.hal.core.frames import Transform
            from axis_os.hal.core.types import Quaternion, Vector3

            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(self._prim_path)
            m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default())
            t = m.ExtractTranslation()
            # Axes du prim dans le monde (USD : vecteurs-lignes).
            ax = np.array([[m.TransformDir((1, 0, 0))[i] for i in range(3)],
                           [m.TransformDir((0, 1, 0))[i] for i in range(3)],
                           [m.TransformDir((0, 0, 1))[i] for i in range(3)]]).T
            for c in range(3):
                n = np.linalg.norm(ax[:, c])
                if n > 1e-9:
                    ax[:, c] /= n
            # USD caméra -> optique : demi-tour autour de x (y et z inversés).
            r_opt = ax @ np.diag([1.0, -1.0, -1.0])
            q = _matrix_to_quat_wxyz(r_opt)
            return Transform(
                parent=parent, child=child or f"{self.sensor_id}_optical",
                translation=Vector3(float(t[0]), float(t[1]), float(t[2])),
                rotation=Quaternion(w=q[0], x=q[1], y=q[2], z=q[3]),
            )
        except Exception as exc:        # noqa: BLE001
            print(f"  [{self.sensor_id}] pose optique indisponible : {exc}")
            return None

    def _measured_alignment(self, expected_fwd):
        """Cosinus entre l'axe de visée RÉEL (relu dans la scène USD
        composée) et la direction attendue. C'est la vérité terrain : on
        ne se fie pas à ce qu'on a demandé, on mesure ce qui est."""
        try:
            import omni.usd
            import numpy as np
            from pxr import Gf, Usd, UsdGeom
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(self._prim_path)
            m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default())
            f = m.TransformDir(Gf.Vec3d(0, 0, -1)).GetNormalized()
            return float(np.dot([f[0], f[1], f[2]], expected_fwd))
        except Exception:               # noqa: BLE001
            return None

    @property
    def horizontal_fov_deg(self) -> float:
        """Champ de vision horizontal, utile pour vérifier qu'une scène
        tient dans l'image avant de lancer une détection."""
        return math.degrees(
            2.0 * math.atan(self._aperture_mm / (2.0 * self._focal_mm))
        )

    @property
    def frames_captured(self) -> int:
        return self._frames_captured


def _matrix_to_quat_wxyz(m):
    """Matrice de rotation 3x3 (colonnes = axes) -> quaternion (w, x, y, z)."""
    tr = float(m[0, 0] + m[1, 1] + m[2, 2])
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2.0
        return [0.25 * s,
                (m[2, 1] - m[1, 2]) / s,
                (m[0, 2] - m[2, 0]) / s,
                (m[1, 0] - m[0, 1]) / s]
    i = int(max(range(3), key=lambda k: m[k, k]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k]) * 2.0
    q = [0.0, 0.0, 0.0, 0.0]
    q[0] = (m[k, j] - m[j, k]) / s
    q[i + 1] = 0.25 * s
    q[j + 1] = (m[j, i] + m[i, j]) / s
    q[k + 1] = (m[k, i] + m[i, k]) / s
    return q
