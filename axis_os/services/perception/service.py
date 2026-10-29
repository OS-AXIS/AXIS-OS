"""
AXIS-OS — Service de perception.

Ce service répond à la question « qu'est-ce que le robot voit, et où ? ».
C'est le deuxième service système écrit après MemoryService (la mémoire du
robot), et il respecte le même contrat : IService, l'interface commune à
tous les services système (démarrage, arrêt, état de santé), définie dans
axis_os/services/base.py.

Le point important est la séparation en deux responsabilités :

    PerceptionService  — le CODE : acquisition des images, calibration de
                         la caméra, repères (systèmes de coordonnées) et
                         conversion d'un pixel en direction dans le repère
                         du robot. Stable, testable, versionné avec l'OS.
    IObjectDetector    — le MODÈLE, c'est-à-dire ce qui reconnaît les
                         objets. Seule implémentation réelle à ce jour :
                         OWL-ViT, un modèle de Google qui repère des
                         objets décrits en texte libre (voir
                         owlvit_detector.py). Remplaçable sans toucher à
                         l'OS.

Pourquoi cette séparation : Android a mis dix-huit mois à distribuer ses
correctifs parce que tout passait par une mise à jour du système. Un
meilleur détecteur ne doit pas exiger de republier AXIS-OS. C'est la
décision prise au §6.4 du document d'architecture
(docs/architecture_services.md).

Ce que ce service ne fait pas : commander un actionneur (moteur, pince).
Il rend des observations, jamais des mouvements. C'est un invariant de
sécurité documenté sur IService : un service ne commande jamais un
actionneur directement. Il est déjà appliqué au niveau du HAL (la couche
d'abstraction matérielle, qui présente chaque capteur et chaque moteur
sous une interface commune) : ICamera, l'interface d'une caméra, n'expose
aucun moyen d'agir.

Honnêteté sur la profondeur : sans caméra de profondeur, une détection
donne une DIRECTION, pas une POSITION. Le champ `position_3d` reste alors
à None, et c'est délibéré : inventer une distance plausible donnerait à la
préhension (la saisie des objets) des coordonnées qu'elle prendrait au
sérieux.
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
    # Boîte englobante (bbox) : le rectangle qui entoure l'objet dans
    # l'image.
    bbox: Tuple[int, int, int, int]      # x, y, largeur, hauteur en pixels
    direction: Optional[Vector3] = None
    """Direction vers l'objet, exprimée dans le repère du ROBOT (ou dans
    le repère demandé à find()). Renseignée dès que la caméra est calibrée
    (on connaît ses paramètres optiques, qui relient un pixel à une
    direction) et qu'un arbre de repères (la position de la caméra, du
    robot et du monde les uns par rapport aux autres) donne la
    transformation nécessaire. C'est ce qui permet au robot de s'orienter
    vers l'objet, même sans en connaître la distance."""

    position_3d: Optional[Vector3] = None
    """Position de l'objet, dans le même repère que `direction`. None sans
    caméra de profondeur : une image couleur seule ne permet PAS de
    connaître la distance. Un service qui inventerait une valeur ici
    ferait tendre la main du robot au mauvais endroit."""

    timestamp_ns: int = 0
    """Instant de capture de l'IMAGE, en nanosecondes, et non instant de
    la détection. Pourquoi : sur un robot qui bouge, associer une
    observation à la pose (position et orientation) du moment où elle est
    traitée, plutôt qu'à celle du moment où elle a été capturée, décale
    l'objet de plusieurs centimètres."""

    # Dernier champ : un appel positionnel écrit avant lui reste valide.
    frame: str = "base"
    """Repère de `direction` et de `position_3d` : « base » (le robot,
    qui bouge avec lui) ou « world » (fixe, celui de la carte). Le SDK
    exige « world » pour saisir un objet (RobotContext.pick)."""


class IObjectDetector(ABC):
    """Contrat d'un détecteur d'objets — le MODÈLE, pas le service.

    Implémenter cette interface suffit à brancher un détecteur dans
    AXIS-OS, sans modifier une ligne du service.
    """

    @abstractmethod
    def detect(self, frame: CameraFrame,
               query: Optional[str] = None) -> List[Detection]:
        """Détecte les objets présents dans `frame`, l'image capturée.

        `query` décrit ce qu'on cherche en langage naturel (None : pas de
        recherche précise, comme pour describe_scene()). Un détecteur à
        vocabulaire fermé, qui ne connaît qu'une liste fixe d'objets, peut
        l'ignorer, mais devrait alors le documenter."""

    @property
    @abstractmethod
    def is_open_vocabulary(self) -> bool:
        """True si le détecteur trouve des objets décrits librement, sans
        avoir été entraîné sur une liste fixe de classes (« vocabulaire
        ouvert »).

        Cette distinction est structurante, pas cosmétique : un détecteur
        à vocabulaire fermé oblige à réentraîner le modèle pour chaque
        nouvel objet, ce qui ne passe pas à l'échelle d'un catalogue
        d'apps tierces. L'exposer permet à un orchestrateur (le composant
        qui enchaîne les tâches du robot) de savoir s'il peut demander
        n'importe quoi ou seulement une liste connue.
        """


class NullObjectDetector(IObjectDetector):
    """Détecteur qui ne détecte rien, honnêtement : il se déclare à
    vocabulaire fermé, si bien que describe_scene() précise qu'une absence
    de détection ne prouve pas l'absence d'objet.

    Pourquoi : il permet de faire tourner et de tester toute la chaîne
    (acquisition, calibration, repères) SANS modèle. Même rôle que
    NullBalanceModel, le modèle d'équilibre factice qui ne corrige rien
    (axis_os/hal/core/balance_model.py) : mieux vaut un service dégradé
    qui le dit qu'un service absent qui fait tout échouer.
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
        camera : caméra couleur principale (interface ICamera du HAL).
        frame_tree : arbre de repères. Sans lui, les détections sont
            rendues sans direction.
        detector : modèle de détection. Par défaut NullObjectDetector, qui
            ne détecte rien.
        max_frame_age_s : âge maximal d'une image, en secondes. Au-delà,
            find() refuse de l'analyser, car la scène a pu changer.
        depth_camera : caméra de profondeur, si disponible ; elle est lue
            par sa méthode capture_depth(). Sans elle, les détections
            portent une DIRECTION mais jamais de POSITION : le robot sait
            alors vers où s'orienter, pas où s'arrêter. C'est la
            différence entre « le colis est par là » et « le colis est à
            2.3 m ».

            Elle doit regarder la MÊME scène que `camera` : le service
            suppose les deux images alignées pixel à pixel. Sur une
            RealSense (caméra de profondeur d'Intel), cela exige
            l'alignement profondeur/couleur ; sur un montage où les images
            ne sont pas alignées, les positions seraient fausses sans
            qu'aucune vérification ne s'en aperçoive.
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
            # Sain mais dégradé : le service voit, mais sans calibration
            # il ne sait pas situer ce qu'il voit.
            return ServiceHealth(
                running=True, healthy=True,
                detail="caméra non calibrée — directions indisponibles",
            )
        return ServiceHealth(running=True, healthy=True, detail="opérationnel")

    # ---- API ------------------------------------------------------------

    def describe_scene(self) -> ServiceResult:
        """Ce que le robot voit, en clair : une chaîne qui énumère les
        labels (noms des objets) détectés, sans doublon, par ordre
        alphabétique et séparés par des virgules. Entrée principale d'un
        futur orchestrateur."""
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

        Échecs (ok=False) : service non démarré, aucune image disponible,
        image trop ancienne (avec partial=True), détecteur en panne.

        `now_ns` : instant présent, en nanosecondes ; par défaut,
        l'horloge système.

        `frame` : repère dans lequel exprimer directions et positions.

        FRAME_BASE (défaut) — repère du ROBOT, qui bouge avec lui. C'est
        ce qu'il faut pour se diriger vers un objet : « 15° à gauche, à
        2 m ».

        FRAME_WORLD — repère FIXE, celui du monde. Indispensable dès qu'on
        mémorise une position. Constaté : une app d'inventaire qui
        balayait la pièce en TOURNANT SUR PLACE enregistrait ses
        observations dans le repère du robot ; deux colis situés à 90°
        l'un de l'autre apparaissaient tous deux à (2.2, 0), puisque le
        robot leur faisait face tour à tour, et ont été fusionnés en un
        seul. Un inventaire tenu dans un repère mobile n'a pas de sens
        (détail dans docs/validation_inventaire.md).
        """
        # Piège de nommage : le paramètre `frame` désigne un REPÈRE, mais
        # la variable `frame` plus bas désigne l'IMAGE capturée. On copie
        # le repère tout de suite pour que l'image ne l'écrase pas. Cette
        # collision provoquait une erreur très opaque : « unhashable type:
        # CameraFrame ».
        repere_sortie = frame

        if not self._started:
            return ServiceResult(ok=False, reason="service non démarré")

        if query is not None and not self._detector.is_open_vocabulary:
            # Un détecteur à vocabulaire fermé ne comprend pas une
            # description libre. On n'échoue pas pour autant, car il peut
            # connaître ce label. L'intention est de ne pas laisser croire
            # que la requête a été comprise telle quelle, mais ce bloc ne
            # signale encore rien à l'appelant : celui-ci peut consulter
            # stats["open_vocabulary"].
            pass

        frame = self._camera.capture()
        if frame is None:
            return ServiceResult(
                ok=False, reason="aucune image disponible")

        # Une image trop ancienne décrit une scène qui a pu changer : on
        # refuse de l'analyser plutôt que de rendre des positions
        # périmées. Une image sans horodatage (timestamp_ns à 0) est
        # traitée comme fraîche, faute de pouvoir mesurer son âge.
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

        # Une inférence (le passage de l'image dans le modèle) en PANNE
        # rend zéro détection, ce qui est strictement indiscernable d'une
        # scène vide pour l'appelant. Constaté en exécution réelle : le
        # post-traitement d'OWL-ViT (la conversion de la sortie brute du
        # modèle en boîtes et en scores) plantait à chaque image et find()
        # rendait ok=True avec 0 détection, sans le moindre signal. Un
        # détecteur qui expose son erreur, dans stats["last_inference_error"],
        # doit donc la voir remonter.
        # Attention : OwlViTDetector garde cette erreur en mémoire même
        # après une inférence réussie ; une scène réellement vide observée
        # après une panne est donc encore signalée comme une panne.
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
            # Dégradation honnête : les détections sont rendues, mais sans
            # direction. C'est mieux qu'une direction inventée, et mieux
            # qu'un échec total qui priverait l'appelant de l'information
            # utile : ce qui a été vu.
            return ServiceResult(
                ok=True,
                value=sorted(detections, key=lambda d: -d.confidence),
                reason=("caméra non calibrée" if intrinsics is None
                        else "aucun arbre de repères") +
                       " — directions indisponibles",
            )

        # Image de profondeur (une distance par pixel), seulement si la
        # caméra de profondeur sait en fournir une.
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
        """Ajoute à chaque détection sa direction et, si une image de
        profondeur est fournie, sa position, dans le repère `frame`.

        Si l'arbre de repères ne connaît pas la transformation entre la
        caméra et ce repère, les détections sont rendues telles quelles.
        Rend (détections, avertissement de l'arbre de repères ou None).
        """
        tf, warn = self._tree.lookup(
            frame, FRAME_CAMERA_OPTICAL, now_ns=now_ns)
        if tf is None:
            return detections, warn

        out = []
        for d in detections:
            x, y, w, h = d.bbox
            # Centre de la boîte : approximation assumée. Le centre
            # géométrique d'une boîte n'est ni le centre de masse de
            # l'objet ni son point de préhension. Suffisant pour
            # s'orienter vers lui, pas pour l'attraper.
            cx, cy = x + w / 2.0, y + h / 2.0
            # Rayon dans le repère optique de la caméra (x vers la droite,
            # y vers le bas, z vers l'avant), puis exprimé dans le repère
            # demandé.
            ray_optical = intrinsics.pixel_to_ray(cx, cy)
            direction = tf.apply_direction(ray_optical)

            # POSITION = direction × distance. pixel_to_ray rend un rayon
            # de composante z = 1 dans le repère optique, et la profondeur
            # est mesurée perpendiculairement au plan de l'image : le
            # produit des deux donne directement le point, sans
            # trigonométrie.
            position = d.position_3d
            if position is None and depth is not None:
                position = self._point_de_face(depth, d.bbox, intrinsics, tf)
            if position is None and depth is not None:
                # Repli (trop peu de points pour trouver la face) : la
                # profondeur proche le long du rayon du centre de la boîte.
                dist = self._depth_of_object(depth, d.bbox)
                if dist is not None:
                    p_opt = Vector3(ray_optical.x * dist,
                                    ray_optical.y * dist,
                                    ray_optical.z * dist)
                    # apply, et non apply_direction : un POINT subit la
                    # translation entre repères, contrairement à une
                    # direction.
                    position = tf.apply(p_opt)

            out.append(Detection(
                label=d.label, confidence=d.confidence, bbox=d.bbox,
                direction=direction,
                position_3d=position,
                frame="world" if frame == FRAME_WORLD else "base",
                timestamp_ns=d.timestamp_ns,
            ))
        return out, warn

    @staticmethod
    def _point_de_face(depth, bbox, intrinsics: CameraIntrinsics, tf, pas: int = 2,
                       bande_m: float = 0.06, support_m: float = 0.015,
                       marge_verticale: float = 0.15) -> Optional[Vector3]:
        """Point de la FACE de l'objet tournée vers la caméra (son milieu,
        repère de `tf`), d'après les points 3D de la boîte de détection.

        Pourquoi (constaté sur GPU, 2026-10-29 : carton vu 2 à 5 cm trop
        près) : _depth_of_object rend la profondeur du quart le plus proche
        de la boîte, c'est-à-dire le HAUT de la face avant pour une caméra
        qui regarde vers le bas, et on la portait le long du rayon du
        CENTRE de la boîte. Le point tombait donc sur le plan perpendiculaire
        à l'axe de la caméra passant par le haut de la face, qui revient
        vers le robot à mesure qu'il descend : trop près (de Δz × tan de la
        plongée), et trop haut. Une boîte qui déborde vers le bas (bord de
        la table) aggravait l'écart.

        Ici, chaque pixel est reporté le long de SON rayon, dans le repère
        de sortie (z vertical) :
          - la distance horizontale de la face : celle de la partie la plus
            proche (médiane du quart le plus proche), et tous les points à
            moins de `bande_m` derrière (une face tournée de 12° y tient) ;
          - le plan qui porte l'objet (le dessus de la table) en est
            écarté : les points à moins de `support_m` au-dessus du plus
            bas de la face ;
          - le point rendu : dans la direction du centre de la boîte, à la
            distance horizontale médiane de la face, à mi-hauteur de la face.
        Les bords haut et bas de la boîte (`marge_verticale`) sont laissés
        de côté : ils débordent d'ordinaire sur le fond et sur la table.
        Rend None si trop peu de points : jamais une position par défaut."""
        import numpy as np
        h, w = depth.shape[0], depth.shape[1]
        x, y, bw, bh = (float(v) for v in bbox)
        x0, x1 = int(max(0, x)), int(min(w, x + max(bw, 1)))
        y0 = int(max(0, y + marge_verticale * bh))
        y1 = int(min(h, y + (1 - marge_verticale) * bh))
        if x0 >= x1 or y0 >= y1:
            return None
        vs, us = np.mgrid[y0:y1:pas, x0:x1:pas]
        d = np.asarray(depth, dtype=float)[vs, us]
        ok = np.isfinite(d) & (d > 1e-3)
        if ok.sum() < 10:
            return None
        u, v, d = us[ok], vs[ok], d[ok]
        k = intrinsics
        opt = np.stack([(u - k.cx) / k.fx * d, (v - k.cy) / k.fy * d, d], axis=1)
        o = tf.apply(Vector3(0.0, 0.0, 0.0))
        t = np.array([o.x, o.y, o.z])
        rot = np.array([[c.x, c.y, c.z] for c in (tf.apply_direction(Vector3(*e))
                                                   for e in np.eye(3))]).T
        P = opt @ rot.T + t
        rho = np.hypot(P[:, 0] - t[0], P[:, 1] - t[1])
        proche = float(np.median(rho[rho <= np.quantile(rho, 0.25)]))
        face = rho <= proche + bande_m
        face &= P[:, 2] > np.quantile(P[face, 2], 0.02) + support_m
        if face.sum() < 5:
            return None
        Q = P[face]
        centre = rot @ np.array([(x + bw / 2 - k.cx) / k.fx, (y + bh / 2 - k.cy) / k.fy, 1.0])
        n = float(np.hypot(centre[0], centre[1]))
        if n < 1e-9:
            return None                    # caméra à la verticale : pas de direction horizontale
        r = float(np.median(np.hypot(Q[:, 0] - t[0], Q[:, 1] - t[1])))
        z = 0.5 * (float(np.quantile(Q[:, 2], 0.02)) + float(np.quantile(Q[:, 2], 0.98))) \
            - support_m / 2
        return Vector3(float(t[0] + r * centre[0] / n), float(t[1] + r * centre[1] / n), z)

    @staticmethod
    def _depth_of_object(depth, bbox, quantile: float = 0.25):
        """Distance de la SURFACE VISIBLE de l'objet détecté.

        Prend la médiane du quart le plus PROCHE des profondeurs valides
        dans la boîte (réglé par `quantile`), plutôt qu'un point au
        centre.

        Pourquoi : une boîte de détection contient toujours de
        l'arrière-plan, d'autant plus qu'elle est large ou imprécise. Un
        échantillon pris au centre tombe alors sur ce qu'il y a DERRIÈRE
        l'objet. Constaté : un robot arrivé à 5 cm d'un colis le croyait
        encore à 1.87 m, parce que la mesure portait sur le sol visible
        au-delà (position z = -0.74 dans son repère, soit le plancher).

        Le quart le plus proche, lui, appartient à l'objet : c'est la
        partie de la boîte qui se trouve devant. Prendre la médiane de ce
        quart plutôt que le minimum absolu résiste au bruit et aux pixels
        aberrants d'une caméra de profondeur. Les valeurs invalides sont
        écartées : NaN (« not a number », valeur non définie), infinies,
        négatives, nulles ou quasi nulles (0.001 ou moins).

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
        voisinage (un carré de 2 × half + 1 pixels de côté).

        Pourquoi la médiane : un pixel isolé suffit à tout fausser. Sur un
        bord d'objet, il peut appartenir à l'arrière-plan, et une caméra
        de profondeur rend régulièrement des valeurs invalides (0, inf,
        NaN). La médiane d'un voisinage résiste à ces deux cas. Rend None
        si aucune valeur n'est exploitable — jamais une distance par
        défaut.

        find() n'utilise pas cette méthode : il passe par
        _depth_of_object, qui évite de mesurer l'arrière-plan derrière
        l'objet. Elle reste couverte par test/test_perception_service.py.
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
        """Compteurs de fonctionnement : nombre total de détections,
        nombre d'images refusées parce que trop anciennes, et vocabulaire
        ouvert ou non du détecteur."""
        return {
            "detections": self._detections_count,
            "stale_frames": self._stale_frames,
            "open_vocabulary": self._detector.is_open_vocabulary,
        }
