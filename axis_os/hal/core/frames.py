"""
AXIS-OS — Repères et transformations entre repères.

Un repère est un système de coordonnées attaché à quelque chose (le
monde, le corps du robot, la caméra). Une transformation dit où se
trouve un repère par rapport à un autre : une translation et une
rotation.

Pourquoi ce module : `CameraIntrinsics.pixel_to_ray()` donne une
direction dans le repère de la caméra. Pour que la préhension (la saisie
d'objets) s'en serve, il faut la convertir dans le repère du robot, donc
savoir où est la caméra par rapport au corps et comment cette relation
change quand le torse pivote. Sans cela, perception et manipulation ne
peuvent pas se parler.

Le modèle est TF, la bibliothèque de repères de ROS (Robot Operating
System, l'intergiciel de référence en robotique), volontairement réduite
à ce qui sert ici : un arbre de repères, des transformations rigides et
leur composition d'un repère à l'autre. Il n'y a ni bus distribué ni
découverte automatique, ce qui reste cohérent avec le choix
d'architecture d'AXIS-OS : des frontières strictes entre composants.

Convention : `Transform(parent, child, translation, rotation)` exprime
la pose (position et orientation) de `child` dans `parent`. Appliquée à
un point exprimé dans `child`, elle rend ce point exprimé dans `parent`.
C'est la convention de ROS ; s'en écarter serait une source d'erreurs
permanente pour quiconque vient de cet écosystème.

Horodatage : une transformation porte l'instant où elle était valable.
Sur un robot qui bouge, associer une image prise à l'instant T à une
pose lue à T + 50 ms déplace un objet de plusieurs centimètres.
`FrameTree.lookup()` signale donc une transformation périmée plutôt que
de la rendre en silence, selon le même principe que l'échéance de
fraîcheur du suiveur de consignes (SetpointFollower,
axis_os/services/policy/setpoint_follower.py).
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from axis_os.hal.core.types import Quaternion, Vector3


# Noms de repères conventionnels d'AXIS-OS. Des noms libres restent
# acceptés, car un robot exotique peut nommer les siens, mais ces
# constantes évitent que deux modules écrivent "base" et "base_link"
# pour désigner la même chose.
FRAME_WORLD = "world"          # repère fixe de l'environnement
FRAME_BASE = "base_link"       # corps du robot, origine de sa cinématique
FRAME_CAMERA = "camera_link"   # corps physique (boîtier) de la caméra
FRAME_CAMERA_OPTICAL = "camera_optical"
"""Repère optique de la caméra, distinct de camera_link. Les confondre
est une erreur classique : l'optique utilise x à droite, y vers le bas,
z vers l'avant (c'est ce que rend pixel_to_ray), alors que la convention
robotique usuelle est x vers l'avant, y à gauche, z vers le haut. La
rotation entre les deux n'est PAS l'identité : voir
camera_optical_to_link() en fin de fichier."""


# Opérations sur les quaternions. Un quaternion unitaire (w, x, y, z)
# représente une rotation dans l'espace ; c'est la forme sous laquelle
# les transformations ci-dessous portent leur orientation.
def _quat_mul(a: Quaternion, b: Quaternion) -> Quaternion:
    """Produit a × b : la rotation b, suivie de la rotation a."""
    return Quaternion(
        w=a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        x=a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y=a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z=a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    )


def _quat_conj(q: Quaternion) -> Quaternion:
    """Conjugué de q : la rotation inverse quand q est unitaire."""
    return Quaternion(w=q.w, x=-q.x, y=-q.y, z=-q.z)


def _quat_rotate(q: Quaternion, v: Vector3) -> Vector3:
    """Applique la rotation q au vecteur v : q × v × q⁻¹."""
    vq = Quaternion(w=0.0, x=v.x, y=v.y, z=v.z)
    r = _quat_mul(_quat_mul(q, vq), _quat_conj(q))
    return Vector3(r.x, r.y, r.z)


@dataclass(frozen=True)
class Transform:
    """Pose rigide (sans déformation) de `child` exprimée dans `parent`."""

    parent: str
    child: str
    translation: Vector3 = field(default_factory=Vector3)
    rotation: Quaternion = field(default_factory=Quaternion)
    timestamp_ns: int = 0
    """Instant de validité, en nanosecondes (même horloge que
    time.time_ns(), utilisée par FrameTree.lookup()). 0 signifie
    transformation statique (montage mécanique fixe), jamais périmée.
    Une valeur non nulle signifie qu'elle a été mesurée à cet instant et
    qu'elle vieillit."""

    def apply(self, point: Vector3) -> Vector3:
        """Point exprimé dans `child` -> même point exprimé dans `parent`."""
        rotated = _quat_rotate(self.rotation, point)
        return Vector3(
            rotated.x + self.translation.x,
            rotated.y + self.translation.y,
            rotated.z + self.translation.z,
        )

    def apply_direction(self, direction: Vector3) -> Vector3:
        """Comme apply(), mais sans la translation.

        Une direction n'a pas de position : la déplacer n'a aucun sens.
        Confondre les deux est l'erreur la plus fréquente dans ce genre
        de code. C'est exactement la même distinction qu'entre position
        et vitesse dans kinematic_normalizer.py, où appliquer à une
        vitesse le décalage (offset) prévu pour une position produisait
        un biais constant.
        """
        return _quat_rotate(self.rotation, direction)

    def inverse(self) -> "Transform":
        """Transformation réciproque : parent et enfant échangés.

        L'horodatage est conservé : l'inverse d'une mesure n'est ni plus
        frais ni plus ancien qu'elle.
        """
        inv_rot = _quat_conj(self.rotation)
        t = _quat_rotate(inv_rot, self.translation)
        return Transform(
            parent=self.child, child=self.parent,
            translation=Vector3(-t.x, -t.y, -t.z),
            rotation=inv_rot, timestamp_ns=self.timestamp_ns,
        )

    def compose(self, other: "Transform") -> "Transform":
        """self(parent<-child) ∘ other(child<-grandchild).

        Le résultat va de grandchild à parent. Lève ValueError si le
        repère intermédiaire ne correspond pas (self.child doit être
        other.parent).
        """
        if self.child != other.parent:
            raise ValueError(
                f"Composition impossible : {self.parent}<-{self.child} puis "
                f"{other.parent}<-{other.child}. Le repère intermédiaire ne "
                f"correspond pas."
            )
        rot = _quat_mul(self.rotation, other.rotation)
        trans = self.apply(other.translation)
        # L'âge d'une chaîne est celui de son maillon le plus ancien : une
        # pose composée n'est pas plus fraîche que la plus périmée des
        # transformations qui la constituent. Les maillons statiques
        # (horodatage 0) n'entrent pas dans le calcul.
        stamps = [t for t in (self.timestamp_ns, other.timestamp_ns) if t]
        return Transform(
            parent=self.parent, child=other.child,
            translation=trans, rotation=rot,
            timestamp_ns=min(stamps) if stamps else 0,
        )


class FrameTree:
    """Arbre des repères du robot.

    Chaque repère a au plus un parent : c'est un arbre, pas un graphe
    quelconque. C'est ce qui garantit qu'il existe un seul chemin entre
    deux repères, donc une seule réponse possible.

    max_age_s : âge, en secondes, au-delà duquel lookup() signale une
    transformation mesurée comme périmée (0,5 s par défaut).
    """

    def __init__(self, max_age_s: float = 0.5):
        self._transforms: Dict[str, Transform] = {}   # enfant -> sa pose
        self._max_age_s = max_age_s

    def set(self, transform: Transform) -> None:
        """Publie ou remplace la pose d'un repère dans son parent.

        Refuse (ValueError) un repère parent de lui-même, ou un repère
        déjà rattaché à un autre parent : l'accepter ferait un graphe,
        pas un arbre.
        """
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
        """Transformations de `frame` jusqu'à sa racine, une par niveau.

        Lève ValueError en cas de cycle : set() refuse qu'un repère ait
        deux parents, mais pas qu'une suite de rattachements forme une
        boucle.
        """
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

        Rend None comme transformation si `frame` est lui-même la
        racine : il n'y a alors rien à composer.
        """
        chain = self._chain_to_root(frame)
        if not chain:
            return None, frame
        # chain = [T(p1<-frame), T(p2<-p1), ...]. On compose vers le
        # haut : T(p2<-p1) ∘ T(p1<-frame) = T(p2<-frame), et ainsi de
        # suite jusqu'à la racine.
        acc = chain[0]
        for tf in chain[1:]:
            acc = tf.compose(acc)
        return acc, acc.parent

    def lookup(self, target: str, source: str,
               now_ns: Optional[int] = None) -> Tuple[Optional[Transform], Optional[str]]:
        """Transformation qui exprime dans `target` un point donné dans
        `source`.

        Méthode : chacun des deux repères est ramené à la racine de son
        arbre, puis les deux sont combinés :
        T(target<-source) = T(target<-racine) ∘ T(racine<-source).
        Cette formule vaut dans tous les cas, y compris pour deux repères
        frères sous un ancêtre commun (robot et caméra tous deux sous le
        monde). Dans ce cas, une version précédente composait dans le
        mauvais ordre et levait « le repère intermédiaire ne correspond
        pas ».

        now_ns : instant de référence pour juger la fraîcheur, en
        nanosecondes ; par défaut l'horloge système (time.time_ns()).

        Rend (transformation, avertissement). Si les deux repères ne sont
        pas reliés, la transformation vaut None et l'avertissement dit
        pourquoi. Sinon, l'avertissement est non None quand la chaîne
        contient une transformation périmée : on ne masque pas le
        problème, mais on ne refuse pas non plus de répondre ; l'appelant
        décide s'il peut s'en contenter.
        """
        if target == source:
            return Transform(parent=target, child=source), None

        up, root_s = self._to_root(source)
        down, root_t = self._to_root(target)
        if root_s != root_t:
            return None, (f"'{source}' et '{target}' ne sont pas reliés : "
                          f"racines '{root_s}' et '{root_t}'.")

        if down is None:                 # target est la racine
            result = up
        elif up is None:                 # source est la racine
            result = down.inverse()
        else:
            result = down.inverse().compose(up)

        # Garde-fou : les deux à None voudrait dire source == target,
        # cas déjà traité plus haut.
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
        """Noms de tous les repères connus (enfants et parents), triés."""
        out = set(self._transforms.keys())
        out |= {tf.parent for tf in self._transforms.values()}
        return sorted(out)


def camera_optical_to_link() -> Transform:
    """Rotation fixe entre le repère optique et le repère mécanique de la
    caméra.

    Optique : x à droite, y vers le bas, z vers l'avant. Robotique : x
    vers l'avant, y à gauche, z vers le haut. Cette rotation n'est PAS
    l'identité. L'oublier fait pointer la préhension dans une direction
    absurde sans qu'aucune vérification ne s'en aperçoive, car le
    résultat reste un vecteur parfaitement valide.

    Transformation statique (horodatage 0), donc jamais périmée.
    """
    # h = cos 45° = sin 45° : les deux quaternions sont des quarts de
    # tour, -90° autour de z appliqué après -90° autour de x.
    h = math.sqrt(2.0) / 2.0
    return Transform(
        parent=FRAME_CAMERA, child=FRAME_CAMERA_OPTICAL,
        rotation=_quat_mul(Quaternion(w=h, x=0.0, y=0.0, z=-h),
                           Quaternion(w=h, x=-h, y=0.0, z=0.0)),
    )
