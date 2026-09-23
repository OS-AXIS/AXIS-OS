"""
AXIS-OS — Repères et transformations entre eux.

PROBLÈME RÉSOLU : `CameraIntrinsics.pixel_to_ray()` donne une direction
dans le repère DE LA CAMÉRA. Pour que la préhension s'en serve, il faut
la convertir dans le repère DU ROBOT — donc savoir où est la caméra par
rapport au corps, et comment cette relation change quand le torse pivote.
Sans ça, perception et manipulation ne peuvent pas se parler.

Inspiré de TF dans ROS, volontairement réduit à ce qui est utile ici :
un arbre de repères, des transformations rigides, et la composition de
l'un à l'autre. Pas de bus distribué, pas de découverte automatique —
cohérent avec le choix d'architecture à frontière stricte d'AXIS-OS.

CONVENTION : `Transform(parent, child, translation, rotation)` exprime
la pose de `child` DANS `parent`. Appliquée à un point exprimé dans
`child`, elle rend ce point exprimé dans `parent`. C'est la convention
de ROS ; s'en écarter serait une source d'erreurs permanente pour
quiconque vient de cet écosystème.

HORODATAGE : une transformation porte l'instant où elle était valable.
Sur un robot qui bouge, associer une image prise à T à une pose lue à
T+50 ms déplace un objet de plusieurs centimètres. `FrameTree.lookup()`
signale une transformation périmée plutôt que de la rendre en silence —
même principe que l'échéance de fraîcheur de SetpointFollower.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from axis_os.hal.core.types import Quaternion, Vector3


# Repères conventionnels d'AXIS-OS. Des chaînes libres restent
# acceptées — un robot exotique peut nommer les siens — mais ces
# constantes évitent que deux modules écrivent "base" et "base_link"
# pour désigner la même chose.
FRAME_WORLD = "world"          # repère fixe de l'environnement
FRAME_BASE = "base_link"       # corps du robot, origine de sa cinématique
FRAME_CAMERA = "camera_link"   # corps physique de la caméra
FRAME_CAMERA_OPTICAL = "camera_optical"
"""Repère OPTIQUE, distinct de camera_link et c'est une source d'erreurs
classique : l'optique utilise x à droite, y vers le BAS, z vers l'avant
(c'est ce que rend pixel_to_ray), alors que la convention robotique
usuelle est x vers l'avant, y à gauche, z vers le haut. La rotation
entre les deux n'est PAS l'identité."""


def _quat_mul(a: Quaternion, b: Quaternion) -> Quaternion:
    return Quaternion(
        w=a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        x=a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y=a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z=a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    )


def _quat_conj(q: Quaternion) -> Quaternion:
    return Quaternion(w=q.w, x=-q.x, y=-q.y, z=-q.z)


def _quat_rotate(q: Quaternion, v: Vector3) -> Vector3:
    """Applique une rotation à un vecteur : q * v * q⁻¹."""
    vq = Quaternion(w=0.0, x=v.x, y=v.y, z=v.z)
    r = _quat_mul(_quat_mul(q, vq), _quat_conj(q))
    return Vector3(r.x, r.y, r.z)


@dataclass(frozen=True)
class Transform:
    """Pose rigide de `child` exprimée dans `parent`."""

    parent: str
    child: str
    translation: Vector3 = field(default_factory=Vector3)
    rotation: Quaternion = field(default_factory=Quaternion)
    timestamp_ns: int = 0
    """0 = transformation STATIQUE (montage mécanique fixe), jamais
    périmée. Une valeur non nulle signifie qu'elle a été mesurée à cet
    instant et vieillit."""

    def apply(self, point: Vector3) -> Vector3:
        """Point exprimé dans `child` -> même point exprimé dans `parent`."""
        rotated = _quat_rotate(self.rotation, point)
        return Vector3(
            rotated.x + self.translation.x,
            rotated.y + self.translation.y,
            rotated.z + self.translation.z,
        )

    def apply_direction(self, direction: Vector3) -> Vector3:
        """Comme apply(), mais SANS la translation.

        Une DIRECTION n'a pas de position : la déplacer n'a aucun sens.
        Confondre les deux est l'erreur la plus fréquente sur ce genre
        de code — c'est exactement la même distinction que celle entre
        position et vitesse dans kinematic_normalizer, où appliquer un
        offset à une vitesse produisait un biais constant.
        """
        return _quat_rotate(self.rotation, direction)

    def inverse(self) -> "Transform":
        inv_rot = _quat_conj(self.rotation)
        t = _quat_rotate(inv_rot, self.translation)
        return Transform(
            parent=self.child, child=self.parent,
            translation=Vector3(-t.x, -t.y, -t.z),
            rotation=inv_rot, timestamp_ns=self.timestamp_ns,
        )

    def compose(self, other: "Transform") -> "Transform":
        """self(parent<-child) ∘ other(child<-grandchild)."""
        if self.child != other.parent:
            raise ValueError(
                f"Composition impossible : {self.parent}<-{self.child} puis "
                f"{other.parent}<-{other.child}. Le repère intermédiaire ne "
                f"correspond pas."
            )
        rot = _quat_mul(self.rotation, other.rotation)
        trans = self.apply(other.translation)
        # L'âge d'une chaîne est celui de son MAILLON LE PLUS ANCIEN :
        # une pose composée n'est pas plus fraîche que la plus périmée
        # des transformations qui la constituent. Les maillons statiques
        # (0) n'entrent pas dans le calcul.
        stamps = [t for t in (self.timestamp_ns, other.timestamp_ns) if t]
        return Transform(
            parent=self.parent, child=other.child,
            translation=trans, rotation=rot,
            timestamp_ns=min(stamps) if stamps else 0,
        )


class FrameTree:
    """Arbre des repères du robot.

    Chaque repère a au plus un parent — un arbre, pas un graphe
    quelconque. C'est ce qui garantit qu'il existe un seul chemin entre
    deux repères, donc une seule réponse possible.
    """

    def __init__(self, max_age_s: float = 0.5):
        self._transforms: Dict[str, Transform] = {}   # child -> Transform
        self._max_age_s = max_age_s

    def set(self, transform: Transform) -> None:
        """Publie ou remplace la pose d'un repère dans son parent."""
        if transform.parent == transform.child:
            raise ValueError(f"Repère '{transform.child}' parent de lui-même.")
        existing = self._transforms.get(transform.child)
        if existing is not None and existing.parent != transform.parent:
            raise ValueError(
                f"'{transform.child}' a déjà pour parent '{existing.parent}' ; "
                f"le rattacher à '{transform.parent}' ferait un graphe, pas "
                f"un arbre. Un repère n'a qu'un seul parent."
            )
        self._transforms[transform.child] = transform

    def _chain_to_root(self, frame: str) -> List[Transform]:
        chain, seen, cur = [], set(), frame
        while cur in self._transforms:
            if cur in seen:
                raise ValueError(f"Cycle de repères détecté en '{cur}'.")
            seen.add(cur)
            tf = self._transforms[cur]
            chain.append(tf)
            cur = tf.parent
        return chain

    def _to_root(self, frame: str) -> Tuple[Optional["Transform"], str]:
        """Transformation racine <- frame, et le nom de cette racine.

        Rend None pour la transformation si `frame` EST la racine : il n'y
        a alors rien à composer.
        """
        chain = self._chain_to_root(frame)
        if not chain:
            return None, frame
        # chain = [T(p1<-frame), T(p2<-p1), ...]. On compose depuis le
        # HAUT : T(p2<-p1) ∘ T(p1<-frame) = T(p2<-frame), etc.
        acc = chain[0]
        for tf in chain[1:]:
            acc = tf.compose(acc)
        return acc, acc.parent

    def lookup(self, target: str, source: str,
               now_ns: Optional[int] = None) -> Tuple[Optional[Transform], Optional[str]]:
        """Transformation qui exprime dans `target` un point donné dans
        `source`.

        Méthode : chacun est ramené à la racine de son arbre, puis
        combiné — T(target<-source) = T(target<-racine) ∘ T(racine<-source).
        C'est valable pour TOUS les cas, y compris deux repères frères
        sous un ancêtre commun (robot et caméra tous deux sous le monde),
        cas où la version précédente composait dans le mauvais ordre et
        levait « le repère intermédiaire ne correspond pas ».

        Rend (transform, avertissement). L'avertissement est non-None si
        la chaîne contient une transformation périmée : on ne masque pas
        le problème, mais on ne refuse pas non plus de répondre —
        l'appelant décide s'il peut s'en contenter.
        """
        if target == source:
            return Transform(parent=target, child=source), None

        up, root_s = self._to_root(source)
        down, root_t = self._to_root(target)
        if root_s != root_t:
            return None, (f"'{source}' et '{target}' ne sont pas reliés : "
                          f"racines '{root_s}' et '{root_t}'.")

        if down is None:                 # target EST la racine
            result = up
        elif up is None:                 # source EST la racine
            result = down.inverse()
        else:
            result = down.inverse().compose(up)

        if result is None:
            return Transform(parent=target, child=source), None

        warn = None
        if result.timestamp_ns:
            now = now_ns if now_ns is not None else time.time_ns()
            age_s = (now - result.timestamp_ns) / 1e9
            if age_s > self._max_age_s:
                warn = (f"Transformation {target}<-{source} périmée de "
                        f"{age_s:.3f}s (seuil {self._max_age_s}s).")
        return result, warn

    def frames(self) -> List[str]:
        out = set(self._transforms.keys())
        out |= {tf.parent for tf in self._transforms.values()}
        return sorted(out)


def camera_optical_to_link() -> Transform:
    """Rotation fixe entre le repère optique et le repère mécanique.

    Optique : x droite, y bas, z avant. Robotique : x avant, y gauche,
    z haut. Cette rotation n'est PAS l'identité, et l'oublier fait
    pointer la préhension dans une direction absurde sans qu'aucune
    vérification ne s'en aperçoive — le résultat reste un vecteur
    parfaitement valide.
    """
    h = math.sqrt(2.0) / 2.0
    return Transform(
        parent=FRAME_CAMERA, child=FRAME_CAMERA_OPTICAL,
        rotation=_quat_mul(Quaternion(w=h, x=0.0, y=0.0, z=-h),
                           Quaternion(w=h, x=-h, y=0.0, z=0.0)),
    )
