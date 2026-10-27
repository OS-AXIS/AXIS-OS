"""
AXIS-OS — Animations (axis_os.store.sdk.animation)
===================================================
Une animation est un geste décrit par des images clés : à chaque instant
`t` (secondes depuis le début), la position de quelques articulations
(radians, convention normalisée, celle des apps). Entre deux images, la
position est interpolée en ligne droite.

Format JSON :

    {
      "format": "axis-animation",
      "format_version": 1,
      "name": "wave",
      "version": "1.0.0",
      "robots": ["G1"],
      "description": "Salue de la main droite.",
      "keyframes": [
        {"t": 0.0, "positions": {"right_shoulder_pitch_joint": 0.0}},
        {"t": 1.0, "positions": {"right_shoulder_pitch_joint": -1.2}}
      ]
    }

Champ facultatif : "stationary_legs_still": true, la mention « robot à
l'arrêt, jambes immobiles ». Sans elle, une animation qui nomme une
articulation de jambe est refusée. Avec elle, les jambes doivent rester
immobiles d'un bout à l'autre, et l'OS ne les commande jamais : elles
décrivent seulement la posture attendue, que l'OS compare à la posture
mesurée avant de jouer. Un robot à jambes debout ne se pilote pas
articulation par articulation depuis une animation.

Tout est vérifié AVANT de jouer (validate_animation) : format, robots
compatibles, articulations connues et animables, positions à l'intérieur
des butées (avec une marge), vitesses et accélérations entre les images
(bornées par ANIM_MAX_VEL, ANIM_MAX_ACC et la vitesse maximale de chaque
articulation), durée et nombre d'images bornés. En ligne de commande :
`axis animation validate <fichier>`.

Licence : Apache 2.0 (open-source — SDK public)
"""
from __future__ import annotations

import json
import math
import re
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

from .results import InvalidArgumentError

FORMAT = "axis-animation"
FORMAT_VERSION = 1
MAX_KEYFRAMES = 500
MAX_DURATION_S = 60.0
MAX_FILE_BYTES = 256 * 1024
MIN_SEGMENT_S = 0.02
# Vitesse et accélération maximales d'une articulation dans une animation
# (rad/s, rad/s²), en plus de la vitesse maximale déclarée par le driver.
ANIM_MAX_VEL = 2.0
ANIM_MAX_ACC = 10.0
# Marge aux butées (rad) : une animation ne va jamais jusqu'à la butée.
LIMIT_MARGIN = 0.02
# Jambes « immobiles » : variation au plus de LEG_HOLD_TOL (rad) entre deux
# images ; écart toléré entre la posture décrite et la posture mesurée.
LEG_HOLD_TOL = 0.01
LEG_MATCH_TOL = 0.05
_NOM = re.compile(r"^[a-z0-9][a-z0-9_-]{0,63}$")
_SEMVER = re.compile(r"^\d+\.\d+\.\d+$")


@dataclass(frozen=True)
class Keyframe:
    t: float
    positions: Dict[str, float]


@dataclass
class Animation:
    name: str
    version: str
    robots: Tuple[str, ...]
    keyframes: Tuple[Keyframe, ...]
    description: str = ""
    stationary_legs_still: bool = False

    @property
    def duration_s(self) -> float:
        return self.keyframes[-1].t

    @property
    def joints(self) -> Tuple[str, ...]:
        vus: Dict[str, None] = {}
        for k in self.keyframes:
            for j in k.positions:
                vus.setdefault(j)
        return tuple(vus)

    def to_json(self) -> Dict[str, Any]:
        d = {"format": FORMAT, "format_version": FORMAT_VERSION, "name": self.name,
             "version": self.version, "robots": list(self.robots),
             "description": self.description,
             "keyframes": [{"t": k.t, "positions": dict(k.positions)} for k in self.keyframes]}
        if self.stationary_legs_still:
            d["stationary_legs_still"] = True
        return d

    def position_at(self, joint: str, t: float) -> Optional[float]:
        """Position interpolée de `joint` à l'instant `t` (bornée au début
        et à la fin), entre les images qui le nomment ; None s'il n'y
        figure pas."""
        points = [(k.t, k.positions[joint]) for k in self.keyframes if joint in k.positions]
        if not points:
            return None
        if t <= points[0][0]:
            return points[0][1]
        for (t0, p0), (t1, p1) in zip(points, points[1:]):
            if t <= t1:
                return p0 + (p1 - p0) * (t - t0) / (t1 - t0)
        return points[-1][1]


def _nombre_fini(v: Any) -> bool:
    return not isinstance(v, bool) and isinstance(v, (int, float)) and math.isfinite(v)


def load_animation(source: Any) -> Animation:
    """Animation depuis un dict, un texte JSON, ou le chemin d'un fichier
    .json. Vérifie le FORMAT (types, champs, ordre des images) ; les
    limites du robot sont vérifiées par validate_animation(). Lève
    InvalidArgumentError en listant les problèmes trouvés."""
    if isinstance(source, Animation):
        return source
    if isinstance(source, Path) or (isinstance(source, str) and source.endswith(".json")
                                    and not source.lstrip().startswith("{")):
        chemin = Path(source)
        if not chemin.is_file():
            raise InvalidArgumentError(f"fichier d'animation introuvable : {chemin}")
        if chemin.stat().st_size > MAX_FILE_BYTES:
            raise InvalidArgumentError(f"fichier d'animation de plus de {MAX_FILE_BYTES} octets")
        source = chemin.read_text(encoding="utf-8")
    if isinstance(source, str):
        if len(source.encode("utf-8")) > MAX_FILE_BYTES:
            raise InvalidArgumentError(f"animation de plus de {MAX_FILE_BYTES} octets")
        try:
            source = json.loads(source)
        except json.JSONDecodeError as exc:
            raise InvalidArgumentError(f"JSON invalide : {exc}") from None
    if not isinstance(source, dict):
        raise InvalidArgumentError("une animation est un objet JSON")
    try:
        taille = len(json.dumps(source, allow_nan=False).encode("utf-8"))
    except (TypeError, ValueError):
        raise InvalidArgumentError("animation : seules des données JSON finies sont "
                                   "acceptées") from None
    if taille > MAX_FILE_BYTES:
        raise InvalidArgumentError(f"animation de plus de {MAX_FILE_BYTES} octets")
    d = source
    erreurs: List[str] = []
    connus = {"format", "format_version", "name", "version", "robots", "description",
              "keyframes", "stationary_legs_still"}
    inconnus = sorted(set(d) - connus)
    if inconnus:
        erreurs.append(f"champs inconnus : {inconnus[:5]}")
    if d.get("format") != FORMAT:
        erreurs.append(f"« format » doit valoir « {FORMAT} »")
    if d.get("format_version") != FORMAT_VERSION or isinstance(d.get("format_version"), bool):
        erreurs.append(f"« format_version » doit valoir {FORMAT_VERSION}")
    nom = d.get("name")
    if not isinstance(nom, str) or not _NOM.match(nom):
        erreurs.append("« name » : minuscules, chiffres, « _ » ou « - », 64 caractères au plus")
    version = d.get("version")
    if not isinstance(version, str) or not _SEMVER.match(version):
        erreurs.append("« version » : MAJEUR.MINEUR.CORRECTIF (ex. 1.0.0)")
    robots = d.get("robots")
    if not isinstance(robots, list) or not robots or len(robots) > 16 \
            or not all(isinstance(r, str) and 0 < len(r) <= 64 for r in robots):
        erreurs.append("« robots » : liste non vide des modèles compatibles (ex. [\"G1\"])")
        robots = []
    description = d.get("description", "")
    if not isinstance(description, str) or len(description) > 500:
        erreurs.append("« description » : texte de 500 caractères au plus")
        description = ""
    immobiles = d.get("stationary_legs_still", False)
    if not isinstance(immobiles, bool):
        erreurs.append("« stationary_legs_still » : true ou false")
        immobiles = False
    brutes = d.get("keyframes")
    images: List[Keyframe] = []
    if not isinstance(brutes, list) or not 2 <= len(brutes) <= MAX_KEYFRAMES:
        erreurs.append(f"« keyframes » : liste de 2 à {MAX_KEYFRAMES} images")
        brutes = []
    for i, k in enumerate(brutes):
        if not isinstance(k, dict) or set(k) != {"t", "positions"}:
            erreurs.append(f"image {i} : exactement « t » et « positions »")
            continue
        t, pos = k["t"], k["positions"]
        if not _nombre_fini(t) or t < 0:
            erreurs.append(f"image {i} : « t » doit être un nombre fini positif")
            continue
        if not isinstance(pos, dict) or not pos or len(pos) > 64:
            erreurs.append(f"image {i} : « positions » : de 1 à 64 articulations")
            continue
        mauvais = [j for j, v in pos.items()
                   if not isinstance(j, str) or len(j) > 128 or not _nombre_fini(v)]
        if mauvais:
            erreurs.append(f"image {i} : positions non numériques ou non finies : {mauvais[:3]}")
            continue
        images.append(Keyframe(float(t), {j: float(v) for j, v in pos.items()}))
    if images and len(images) == len(brutes):
        if images[0].t != 0.0:
            erreurs.append("la première image doit être à t = 0")
        for a, b in zip(images, images[1:]):
            if b.t - a.t < MIN_SEGMENT_S:
                erreurs.append(f"images à t = {a.t:g} et {b.t:g} : instants croissants, "
                               f"espacés d'au moins {MIN_SEGMENT_S} s")
                break
        if images[-1].t > MAX_DURATION_S:
            erreurs.append(f"durée de plus de {MAX_DURATION_S:g} s")
        for j in {j for k in images for j in k.positions}:
            if sum(j in k.positions for k in images) < 2:
                erreurs.append(f"« {j} » doit figurer dans au moins deux images")
    if erreurs:
        raise InvalidArgumentError("animation refusée : " + " ; ".join(erreurs))
    return Animation(name=nom, version=version, robots=tuple(robots), keyframes=tuple(images),
                     description=description, stationary_legs_still=immobiles)


def validate_animation(anim: Animation, joint_defs: Iterable, *,
                       robot_model: str) -> List[str]:
    """Vérifie l'animation pour un robot donné, avant de la jouer : rend
    la liste des problèmes (vide si elle est jouable).

    joint_defs  : JointDefinition du robot (limites normalisées, groupe).
    robot_model : modèle du robot (robot_info()["model"]).
    Les articulations de locomotion (jambes) ne bougent jamais par une
    animation (voir l'en-tête du module)."""
    from axis_os.hal.core.types import JointGroup, infer_joint_group

    erreurs: List[str] = []
    if robot_model.lower() not in {r.lower() for r in anim.robots}:
        erreurs.append(f"animation prévue pour {list(anim.robots)}, pas pour {robot_model}")
    defs = {d.joint_id: d for d in joint_defs}
    for j in anim.joints:
        d = defs.get(j)
        if d is None:
            erreurs.append(f"« {j} » : articulation inconnue de ce robot")
            continue
        groupe = d.group if d.group is not None else infer_joint_group(j)
        points = [(k.t, k.positions[j]) for k in anim.keyframes if j in k.positions]
        if groupe is JointGroup.LOCOMOTION:
            if not anim.stationary_legs_still:
                erreurs.append(f"« {j} » : articulation de jambe — refusée sans la mention "
                               f"« stationary_legs_still » (robot à l'arrêt, jambes immobiles)")
            elif any(abs(b[1] - a[1]) > LEG_HOLD_TOL for a, b in zip(points, points[1:])):
                erreurs.append(f"« {j} » : la jambe bouge, alors que l'animation la déclare "
                               f"immobile")
            continue
        if groupe is not JointGroup.MANIPULATION:
            erreurs.append(f"« {j} » : articulation non animable (ni bras ni jambe)")
            continue
        lim = d.limits
        bas, haut = lim.pos_min + LIMIT_MARGIN, lim.pos_max - LIMIT_MARGIN
        hors = [p for _, p in points if not bas <= p <= haut]
        if hors:
            erreurs.append(f"« {j} » : position {hors[0]:+.3f} rad hors de "
                           f"[{bas:+.3f}, {haut:+.3f}] (butées moins {LIMIT_MARGIN} rad)")
        vmax = min(ANIM_MAX_VEL, lim.vel_max) if lim.vel_max and lim.vel_max > 0 else ANIM_MAX_VEL
        vitesses = []
        for (t0, p0), (t1, p1) in zip(points, points[1:]):
            v = (p1 - p0) / (t1 - t0)
            if abs(v) > vmax + 1e-9:
                erreurs.append(f"« {j} » : {abs(v):.2f} rad/s entre t = {t0:g} et {t1:g} s "
                               f"(au plus {vmax:g})")
            vitesses.append((v, t1 - t0))
        # Accélération : du repos à la première vitesse, d'un segment au
        # suivant, puis jusqu'au repos.
        segments = [(0.0, 0.0)] + vitesses + [(0.0, 0.0)]
        for (va, da), (vb, db) in zip(segments, segments[1:]):
            duree = (da + db) / 2.0
            if duree > 0 and abs(vb - va) / duree > ANIM_MAX_ACC + 1e-9:
                erreurs.append(f"« {j} » : accélération de {abs(vb - va) / duree:.1f} rad/s² "
                               f"(au plus {ANIM_MAX_ACC:g})")
                break
    return erreurs


def check_animation(anim: Animation, joint_defs: Iterable, *, robot_model: str) -> None:
    """validate_animation(), qui lève InvalidArgumentError s'il y a un
    problème."""
    erreurs = validate_animation(anim, joint_defs, robot_model=robot_model)
    if erreurs:
        raise InvalidArgumentError("animation refusée : " + " ; ".join(erreurs[:10]))


def library_path(robot_model: str) -> Optional[Path]:
    """Dossier des animations fournies avec le driver d'un robot."""
    if robot_model.lower() == "g1":
        return Path(__file__).resolve().parents[2] / "hal" / "drivers" / "unitree_g1" / "animations"
    return None


def library_names(robot_model: str) -> List[str]:
    dossier = library_path(robot_model)
    if dossier is None or not dossier.is_dir():
        return []
    return sorted(p.stem for p in dossier.glob("*.json"))


def load_library_animation(name: str, robot_model: str) -> Animation:
    """Animation fournie avec le driver (ex. « wave » pour le G1)."""
    if not isinstance(name, str) or not _NOM.match(name):
        raise InvalidArgumentError("nom d'animation : minuscules, chiffres, « _ » ou « - »")
    if name not in library_names(robot_model):
        raise InvalidArgumentError(f"animation « {name} » inconnue pour {robot_model} — "
                                   f"fournies : {library_names(robot_model)}")
    return load_animation(library_path(robot_model) / f"{name}.json")


class AnimationPlayer:
    """Joue une animation validée, côté OS, dans son propre fil : un pas
    toutes les PERIOD_S secondes, par ctx.move_arms() (permissions, arrêt
    d'urgence, JointSafetyGuard à chaque pas). D'abord une approche douce
    (LEAD_IN_SPEED) de la posture mesurée vers la première image, puis
    l'interpolation des images. Les jambes ne sont jamais commandées.

    Pause si le gardien de proximité arrête le robot (une personne trop
    près) : un bras en mouvement la heurterait ; le geste reprend quand
    elle s'éloigne. Fin : PLAYED ; arrêt par stop_animation() : STOPPED
    (CANCELLED) ; arrêt d'urgence, fin du hook, arrêt demandé : REFUSED ;
    un pas refusé par le garde-fou : FAILED (SAFETY_LIMIT)."""

    PERIOD_S = 0.02
    LEAD_IN_SPEED = 0.5      # rad/s
    LEAD_IN_MIN_S = 0.2

    def __init__(self, ctx, anim: Animation, joints: List[str]):
        import threading
        self._ctx = ctx
        self.anim = anim
        self._joints = list(joints)
        etat = ctx.get_full_state()
        self._depart = {j: etat[j].position for j in self._joints}
        premiere = {j: anim.position_at(j, 0.0) for j in self._joints}
        ecart = max((abs(premiere[j] - self._depart[j]) for j in self._joints), default=0.0)
        self._approche_s = max(self.LEAD_IN_MIN_S, ecart / self.LEAD_IN_SPEED)
        self._premiere = premiere
        self.paused = False
        self.result = None
        self._arret = threading.Event()
        self._fini = threading.Event()
        self._fil = threading.Thread(target=self._boucle, daemon=True, name="axis-animation")
        self._fil.start()

    @property
    def total_s(self) -> float:
        return self._approche_s + self.anim.duration_s

    @property
    def done(self) -> bool:
        return self._fini.is_set()

    def stop(self) -> None:
        self._arret.set()

    def wait(self, timeout_s: Optional[float] = None):
        self._fini.wait(timeout_s)
        return self.result

    def _cibles(self, t: float) -> Dict[str, float]:
        if t < self._approche_s:
            u = t / self._approche_s
            return {j: self._depart[j] + (self._premiere[j] - self._depart[j]) * u
                    for j in self._joints}
        return {j: self.anim.position_at(j, t - self._approche_s) for j in self._joints}

    def _finir(self, res) -> None:
        self.result = res
        self._fini.set()

    def _boucle(self) -> None:
        from axis_os.hal.core.types import ActuatorCommand, ControlMode, HALStatus
        from .app_base import EstopActiveError
        from .results import ActionResult, RefusalCode, refused
        ctx = self._ctx
        t, prec = 0.0, ctx.now()
        try:
            while True:
                motif = ctx._motif_interruption()
                if motif is not None:
                    return self._finir(refused("REFUSED", motif[0],
                                               f"animation interrompue : {motif[1]}"))
                if self._arret.is_set():
                    return self._finir(refused("STOPPED", RefusalCode.CANCELLED,
                                               "animation arrêtée par l'app"))
                maintenant = ctx.now()
                dt, prec = max(0.0, maintenant - prec), maintenant
                v = getattr(ctx._locomotion, "dernier_verdict", None)
                self.paused = v is not None and v.facteur <= 0.0
                if not self.paused:
                    t = min(self.total_s, t + min(dt, 0.1))
                    cmds = {j: ActuatorCommand(mode=ControlMode.POSITION, position=p)
                            for j, p in self._cibles(t).items()}
                    refus = [j for j, s in ctx.move_arms(cmds).items() if s != HALStatus.OK]
                    if refus:
                        return self._finir(refused(
                            "FAILED", RefusalCode.SAFETY_LIMIT,
                            f"pas refusé par le garde-fou pour {refus[:3]}"))
                    if t >= self.total_s:
                        return self._finir(ActionResult(
                            ok=True, status="PLAYED", message=f"« {self.anim.name} » jouée",
                            data={"duration_s": round(self.total_s, 3)}))
                time.sleep(self.PERIOD_S)
        except EstopActiveError:
            return self._finir(refused("REFUSED", RefusalCode.ESTOP_ACTIVE,
                                       "animation interrompue : arrêt d'urgence"))
        except Exception as exc:              # noqa: BLE001 — l'animation s'arrête, jamais à l'aveugle
            return self._finir(refused("FAILED", RefusalCode.FAILED,
                                       f"animation arrêtée : {type(exc).__name__}"))
