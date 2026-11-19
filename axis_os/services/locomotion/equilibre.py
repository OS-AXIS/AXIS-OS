"""
AXIS-OS — Équilibre debout : le centre de masse au-dessus des pieds.

Avant de changer la posture du robot (torse penché, bassin abaissé : la
consigne de posture de la marche GR00T WBC), le service de marche vérifie
que le centre de masse du robot entier tombe dans le polygone d'appui de
ses pieds, avec une marge : jambes debout (pliées pour un bassin plus bas),
torse penché, bras à leur posture, charge portée aux mains.

C'est un équilibre STATIQUE, robot immobile : une condition nécessaire, pas
suffisante. La politique de marche (GR00T WBC) tient l'équilibre en
dynamique ; sa tenue sous une consigne de torse penché se valide sur GPU.

Modèle : l'URDF du G1 29 DOF (masse et centre de masse de chaque lien),
bassin de niveau, pieds à plat. Polygone d'appui : l'enveloppe convexe des
sphères de contact des deux pieds (talons et pointes, URDF). Le tangage
demandé au torse est porté par l'articulation de tangage de la taille
(waist_pitch_joint), le bassin restant de niveau : la politique pourrait
aussi reculer les hanches pour compenser, ce qui ne ferait que reculer le
centre de masse ; le modèle reste donc du côté prudent.

Licence : Apache 2.0 (open-source — SDK public)
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from axis_os.services.manipulation.cinematique import URDF_G1_29DOF, _rotation, _rpy

# Jambes debout : la pose par défaut de la politique GR00T WBC (hanche en
# tangage, roulis, lacet ; genou ; cheville en tangage, roulis), la même
# pour les deux jambes (GR00TWBCConfig.default_angles).
JAMBE_DEBOUT = (-0.1, 0.0, 0.0, 0.3, -0.2, 0.0)
ARTICULATIONS_JAMBE = ("hip_pitch_joint", "hip_roll_joint", "hip_yaw_joint", "knee_joint",
                       "ankle_pitch_joint", "ankle_roll_joint")
TAILLE = "waist_pitch_joint"


@dataclass(frozen=True)
class _Articulation:
    nom: str
    parent: str
    enfant: str
    origine: np.ndarray            # 4 × 4
    axe: Optional[np.ndarray]      # None : fixe


@dataclass(frozen=True)
class Equilibre:
    """Bilan d'une posture : centre de masse (repère du bassin, m), masse
    totale (kg), sommets du polygone d'appui (x, y), marge (m) : distance
    du centre de masse projeté au bord du polygone, positive dedans."""
    centre: np.ndarray
    masse: float
    polygone: np.ndarray
    marge: float

    def texte(self) -> str:
        return (f"centre de masse à {100 * self.centre[0]:+.1f} cm devant le bassin, "
                f"{100 * self.marge:.1f} cm {'dans' if self.marge >= 0 else 'HORS de'} "
                f"l'appui des pieds")


class ModeleCorps:
    """Le robot entier, de l'URDF : liens (masse, centre de masse) et
    articulations, à partir du bassin."""

    def __init__(self, chemin: Path = URDF_G1_29DOF):
        racine = ET.parse(str(chemin)).getroot()
        self.inerties: Dict[str, Tuple[float, np.ndarray]] = {}
        self.contacts: Dict[str, List[Tuple[np.ndarray, float]]] = {}
        for l_ in racine.findall("link"):
            nom = l_.get("name")
            i_ = l_.find("inertial")
            if i_ is not None and i_.find("mass") is not None:
                o_ = i_.find("origin")
                c_ = [float(v) for v in ((o_.get("xyz") if o_ is not None else None)
                                         or "0 0 0").split()]
                self.inerties[nom] = (float(i_.find("mass").get("value")), np.array(c_))
            if nom.endswith("ankle_roll_link"):
                spheres = []
                for c in l_.findall("collision"):
                    g = c.find("geometry")[0]
                    if g.tag == "sphere":
                        o = c.find("origin")
                        xyz = [float(v) for v in (o.get("xyz") or "0 0 0").split()]
                        spheres.append((np.array(xyz), float(g.get("radius"))))
                self.contacts[nom] = spheres
        self.articulations: List[_Articulation] = []
        for j in racine.findall("joint"):
            o = j.find("origin")
            xyz = [float(v) for v in (o.get("xyz") or "0 0 0").split()] if o is not None \
                else [0.0, 0.0, 0.0]
            rpy = [float(v) for v in (o.get("rpy") or "0 0 0").split()] if o is not None \
                else [0.0, 0.0, 0.0]
            m = np.eye(4)
            m[:3, :3], m[:3, 3] = _rpy(*rpy), xyz
            axe = None
            if j.get("type") in ("revolute", "continuous"):
                a = np.array([float(v) for v in j.find("axis").get("xyz").split()])
                axe = a / np.linalg.norm(a)
            self.articulations.append(_Articulation(j.get("name"), j.find("parent").get("link"),
                                                    j.find("child").get("link"), m, axe))
        self._enfants: Dict[str, List[_Articulation]] = {}
        for a in self.articulations:
            self._enfants.setdefault(a.parent, []).append(a)

    def reperes(self, angles: Dict[str, float]) -> Dict[str, np.ndarray]:
        """Repère (4 × 4) de chaque lien, dans celui du bassin ; une
        articulation absente de `angles` est à zéro."""
        sortie = {"pelvis": np.eye(4)}
        pile = ["pelvis"]
        while pile:
            lien = pile.pop()
            t = sortie[lien]
            for a in self._enfants.get(lien, ()):
                m = t @ a.origine
                if a.axe is not None:
                    r = np.eye(4)
                    r[:3, :3] = _rotation(a.axe, float(angles.get(a.nom, 0.0)))
                    m = m @ r
                sortie[a.enfant] = m
                pile.append(a.enfant)
        return sortie

    def equilibre(self, angles: Dict[str, float],
                  charges: Iterable[Tuple[float, Sequence[float]]] = ()) -> Equilibre:
        """Centre de masse et marge pour ces angles ; `charges` : (masse en
        kg, position dans le repère du bassin), le carton porté par exemple."""
        reperes = self.reperes(angles)
        somme, masse = np.zeros(3), 0.0
        for lien, (m, c) in self.inerties.items():
            t = reperes.get(lien)
            if t is None or m <= 0.0:
                continue
            somme += m * (t[:3, :3] @ c + t[:3, 3])
            masse += m
        for m, p in charges:
            somme += float(m) * np.asarray(p, dtype=float)
            masse += float(m)
        centre = somme / masse
        points = []
        for lien, spheres in self.contacts.items():
            t = reperes[lien]
            for c, rayon in spheres:
                p = t[:3, :3] @ c + t[:3, 3]
                points.append((p[0], p[1]))
        poly = _enveloppe(np.array(points))
        return Equilibre(centre, masse, poly, _marge(poly, centre[:2]))


def jambes(hauteur_relative: float = 0.0) -> Dict[str, float]:
    """Angles des deux jambes, debout (pose par défaut de la marche),
    bassin abaissé de `hauteur_relative` (m, ≤ 0) : genoux pliés, bassin de
    niveau, pieds à plat et à la même place sous lui (cinématique de
    l'URDF ; la politique de marche choisit les siens, voisins)."""
    sortie = {f"{c}_{n}": v for c in ("left", "right")
              for n, v in zip(ARTICULATIONS_JAMBE, JAMBE_DEBOUT)}
    if abs(hauteur_relative) < 1e-6:
        return sortie
    chaine = _jambe_gauche()
    q0 = dict(zip([f"left_{n}" for n in ARTICULATIONS_JAMBE], JAMBE_DEBOUT))
    cible = chaine.pose(q0)[0] - np.array([0.0, 0.0, hauteur_relative])
    h, g = q0["left_hip_pitch_joint"], q0["left_knee_joint"]
    for _ in range(30):
        def cheville(h_, g_):
            q = dict(q0, left_hip_pitch_joint=h_, left_knee_joint=g_,
                     left_ankle_pitch_joint=-(h_ + g_) + (q0["left_hip_pitch_joint"]
                                                           + q0["left_knee_joint"]
                                                           + q0["left_ankle_pitch_joint"]))
            return chaine.pose(q)[0]
        p = cheville(h, g)
        e = (cible - p)[[0, 2]]
        if float(np.linalg.norm(e)) < 1e-5:
            break
        eps = 1e-5
        jac = np.column_stack([((cheville(h + eps, g) - p) / eps)[[0, 2]],
                               ((cheville(h, g + eps) - p) / eps)[[0, 2]]])
        dh, dg = np.linalg.solve(jac, e)
        h, g = h + float(dh), g + float(dg)
    cheville_ = -(h + g) + sum(JAMBE_DEBOUT[k] for k in (0, 3, 4))
    for c in ("left", "right"):
        sortie[f"{c}_hip_pitch_joint"] = h
        sortie[f"{c}_knee_joint"] = g
        sortie[f"{c}_ankle_pitch_joint"] = cheville_
    return sortie


@lru_cache(maxsize=1)
def _jambe_gauche():
    from axis_os.services.manipulation.cinematique import ChaineBras
    return ChaineBras.depuis_urdf("left", fin="left_ankle_roll_link")


@lru_cache(maxsize=1)
def modele_g1() -> ModeleCorps:
    return ModeleCorps()


@lru_cache(maxsize=1)
def _pieds_debout() -> Tuple[Tuple[float, float, float], ...]:
    """Sphères de contact des pieds du G1 debout (pose par défaut de la
    marche) : (x, y, rayon), repère du bassin."""
    m = modele_g1()
    rep = m.reperes(jambes())
    return tuple((float(p[0]), float(p[1]), float(r)) for lien, spheres in m.contacts.items()
                 for c, r in spheres for p in (rep[lien][:3, :3] @ c + rep[lien][:3, 3],))


def avancee_pieds(cap: float = 0.0) -> float:
    """Jusqu'où (m) les pieds du G1 debout avancent vers un bord dont la
    normale fait l'angle `cap` (rad) avec l'avant du robot : le plus avancé
    des points de contact (talons et pointes de l'URDF, rayon compris).
    0 : la pointe des pieds, 9,9 cm devant le bassin."""
    n = (math.cos(cap), math.sin(cap))
    return max(x * n[0] + y * n[1] + r for x, y, r in _pieds_debout())


def _enveloppe(points: np.ndarray) -> np.ndarray:
    """Enveloppe convexe (sens trigonométrique) de points (x, y)."""
    pts = sorted(set(map(tuple, np.round(points, 9))))
    if len(pts) <= 2:
        return np.array(pts)

    def croix(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    bas, haut = [], []
    for p in pts:
        while len(bas) >= 2 and croix(bas[-2], bas[-1], p) <= 0:
            bas.pop()
        bas.append(p)
    for p in reversed(pts):
        while len(haut) >= 2 and croix(haut[-2], haut[-1], p) <= 0:
            haut.pop()
        haut.append(p)
    return np.array(bas[:-1] + haut[:-1])


def _marge(poly: np.ndarray, p: np.ndarray) -> float:
    """Distance signée de `p` au bord du polygone convexe `poly` (sens
    trigonométrique) : positive dedans, négative dehors."""
    dedans, plus_pres = True, math.inf
    n = len(poly)
    for k in range(n):
        a, b = poly[k], poly[(k + 1) % n]
        ab = b - a
        longueur = float(np.linalg.norm(ab))
        cote = (ab[0] * (p[1] - a[1]) - ab[1] * (p[0] - a[0])) / max(longueur, 1e-12)
        if cote < 0:
            dedans = False
        t = min(1.0, max(0.0, float((p - a) @ ab) / max(longueur * longueur, 1e-12)))
        plus_pres = min(plus_pres, float(np.linalg.norm(p - (a + t * ab))))
    return plus_pres if dedans else -plus_pres
