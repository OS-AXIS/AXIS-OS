"""
AXIS-OS — Boîte orientée d'un carton, d'après un nuage de points.

Méthode standard (celle d'Open3D : segment_plane, puis boîte orientée), ici
en numpy. Pourquoi pas Open3D : sa roue aarch64 n'existe que pour glibc 2.35
(JetPack 6) en 0.20 ; sur JetPack 5 (Ubuntu 20.04), seulement la 0.18, qui
s'arrête à Python 3.11. Les deux briques tiennent en quelques dizaines de
lignes, testées (test/test_perception_boite.py).

  1. plan de la table par RANSAC, normale proche de la verticale (celle de
     la centrale inertielle) ; points au-dessus seulement ;
  2. dessus du carton : le plan horizontal le plus haut ; sa hauteur
     au-dessus de la table, celle du carton ;
  3. centre et lacet : rectangle minimal (enveloppe convexe, compas
     tournant) des points du dessus vus d'en haut ; les dimensions connues
     de l'app complètent ce qui n'est pas vu, à partir du bord avant ;
  4. résidu (écart quadratique moyen des points à la surface de la boîte
     ajustée) et couverture du dessus : au-delà des seuils de l'appelant,
     « carton mal vu », jamais deviné.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

HAUT = np.array([0.0, 0.0, 1.0])


@dataclass
class BoiteEstimee:
    """Boîte ajustée : centre (repère des points), lacet (rad, direction de
    sa profondeur dans le plan horizontal, celle de Objet.lacet), dimensions
    vues (profondeur, largeur, hauteur ; m), résidu (m), couverture du
    dessus (0 à 1), points utilisés, hauteur de la table (None : pas vue)."""
    centre: np.ndarray
    lacet: float
    vues: Tuple[float, float, float]
    residu_m: float
    couverture: float
    n_points: int
    hauteur_table: Optional[float]
    points: Optional[np.ndarray] = None


def _etale(q: np.ndarray, haut: np.ndarray, etendue_min_m: float) -> bool:
    """Des points qui s'étalent dans les deux directions horizontales (un
    plan), pas une ligne : la bande d'une face verticale à hauteur fixe."""
    if len(q) < 3:
        return False
    c = q - q.mean(axis=0)
    c = c - np.outer(c @ haut, haut)
    return float(np.linalg.svd(c, compute_uv=False)[1]) / math.sqrt(len(q)) >= etendue_min_m


def plan_ransac(points: np.ndarray, haut: np.ndarray = HAUT, seuil_m: float = 0.008,
                iterations: int = 200, angle_max_deg: float = 5.0, graine: int = 0,
                etendue_min_m: float = 0.015) -> Optional[Tuple[np.ndarray, float, np.ndarray]]:
    """Plan horizontal qui porte le plus de points à `seuil_m` près, étalés
    dans les deux directions (écart type d'au moins `etendue_min_m` dans
    chacune) : (normale vers le haut, d tel que normale·p + d = 0, masque des
    points du plan), ou None.

    RANSAC à un point : la verticale est connue (centrale inertielle), un
    plan horizontal n'a qu'une inconnue, sa hauteur. Il suffit de quelques
    dizaines de tirages là où trois points en demandent des milliers (le
    dessus d'un carton n'est parfois que 10 % du nuage). La normale est
    ensuite affinée par moindres carrés, gardée à moins de `angle_max_deg`
    de la verticale."""
    n = len(points)
    if n < 3:
        return None
    rng = np.random.default_rng(graine)
    h = points @ haut
    meilleur = None
    for i in rng.choice(n, min(n, iterations), replace=False):
        dedans = np.abs(h - h[i]) < seuil_m
        if (meilleur is None or dedans.sum() > meilleur.sum()) \
                and _etale(points[dedans], haut, etendue_min_m):
            meilleur = dedans
    if meilleur is None:
        return None
    q = points[meilleur]
    centre = q.mean(axis=0)
    _, _, vt = np.linalg.svd(q - centre)
    normale = vt[2] if vt[2] @ haut > 0 else -vt[2]
    if normale @ haut < math.cos(math.radians(angle_max_deg)):
        normale = np.asarray(haut, dtype=float)
    dedans = np.abs((points - centre) @ normale) < seuil_m
    return normale, -float(normale @ centre), dedans


def _enveloppe(xy: np.ndarray) -> np.ndarray:
    """Enveloppe convexe (chaîne monotone d'Andrew), sens trigonométrique."""
    p = np.unique(xy, axis=0)
    if len(p) < 3:
        return p
    p = p[np.lexsort((p[:, 1], p[:, 0]))]

    def demi(pts):
        h: List[np.ndarray] = []
        for q in pts:
            while len(h) >= 2 and ((h[-1][0] - h[-2][0]) * (q[1] - h[-2][1])
                                   - (h[-1][1] - h[-2][1]) * (q[0] - h[-2][0])) <= 0:
                h.pop()
            h.append(q)
        return h
    bas, haut = demi(p), demi(p[::-1])
    return np.array(bas[:-1] + haut[:-1])


def rectangle_minimal(xy: np.ndarray) -> Tuple[np.ndarray, float, Tuple[float, float]]:
    """Rectangle d'aire minimale qui contient les points `xy` (N × 2) :
    (centre, angle de son premier côté en rad dans [0, π/2), (longueur le
    long de cet angle, longueur perpendiculaire)). Compas tournant : un côté
    du rectangle minimal porte un côté de l'enveloppe convexe."""
    h = _enveloppe(np.asarray(xy, dtype=float))
    if len(h) < 3:
        c = h.mean(axis=0) if len(h) else np.zeros(2)
        return c, 0.0, (0.0, 0.0)
    meilleur = None
    for i in range(len(h)):
        e = h[(i + 1) % len(h)] - h[i]
        a = math.atan2(e[1], e[0]) % (math.pi / 2)
        u = np.array([math.cos(a), math.sin(a)])
        v = np.array([-u[1], u[0]])
        pu, pv = h @ u, h @ v
        aire = (pu.max() - pu.min()) * (pv.max() - pv.min())
        if meilleur is None or aire < meilleur[0]:
            c = u * (pu.max() + pu.min()) / 2 + v * (pv.max() + pv.min()) / 2
            meilleur = (aire, c, a, (pu.max() - pu.min(), pv.max() - pv.min()))
    return meilleur[1], meilleur[2], meilleur[3]


def _distance_boite(q: np.ndarray, demi: np.ndarray) -> np.ndarray:
    """Distance (m) de points q (repère de la boîte) à sa surface."""
    d = np.abs(q) - demi
    dehors = np.linalg.norm(np.maximum(d, 0.0), axis=1)
    dedans = np.minimum(d.max(axis=1), 0.0)
    return np.abs(dehors + dedans)


def ajuster_boite(points: np.ndarray, dims: Tuple[float, float, float],
                  vers_camera: np.ndarray, haut: np.ndarray = HAUT,
                  cellule_m: float = 0.02) -> Optional[BoiteEstimee]:
    """Boîte de dimensions connues `dims` (largeur entre les mains,
    profondeur, hauteur ; celles de l'app) ajustée aux `points` (N × 3, dans
    un repère où `haut` est la verticale ; la caméra du côté de
    `vers_camera`, horizontal, depuis le carton). None si trop peu de points
    au-dessus de la table."""
    largeur, profondeur, hauteur = dims
    p = np.asarray(points, dtype=float)
    p = p[np.all(np.isfinite(p), axis=1)]
    if len(p) < 30:
        return None
    h = p @ haut
    # 1. Plans horizontaux : la table est le plus haut de ceux qui sont sous
    #    le dessus d'au moins une demi-hauteur (pas le sol, vu autour d'elle) ;
    #    une boîte vue sans la table n'en a qu'un, son dessus.
    plans, reste = [], p
    for k in range(4):
        pl = plan_ransac(reste, haut, graine=k)
        if pl is None or pl[2].sum() < max(20, 0.05 * len(p)):
            break
        plans.append(float(np.median(reste[pl[2]] @ haut)))   # sa hauteur, le long de `haut`
        reste = reste[~pl[2]]
    if not plans:
        return None
    h_haut = max(plans)
    dessous = [pl for pl in plans if h_haut - pl > 0.5 * hauteur]
    table = max(dessous) if dessous else None
    plancher = table if table is not None else h_haut - hauteur
    boite = p[h > plancher + 0.01]
    if len(boite) < 20:
        return None
    hb = boite @ haut
    dessus_h = h_haut if h_haut > plancher + 0.5 * hauteur else float(np.quantile(hb, 0.98))
    dessus = boite[hb > dessus_h - 0.015]
    if len(dessus) < 10:
        return None
    # Points du carton : autour de son dessus (pas le fond, ni un voisin).
    e1 = np.cross(haut, [1.0, 0.0, 0.0] if abs(haut[0]) < 0.9 else [0.0, 1.0, 0.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(haut, e1)
    plat = np.stack([boite @ e1, boite @ e2], axis=1)
    plat_dessus = np.stack([dessus @ e1, dessus @ e2], axis=1)
    c0 = np.median(plat_dessus, axis=0)
    rayon = math.hypot(largeur, profondeur) / 2 + 0.03
    garde = np.linalg.norm(plat - c0, axis=1) < rayon
    boite, plat, hb = boite[garde], plat[garde], hb[garde]
    plat_dessus = plat_dessus[np.linalg.norm(plat_dessus - c0, axis=1) < rayon]
    # 2. Lacet : rectangle minimal du dessus vu d'en haut ; l'axe le plus
    #    proche de la direction de la caméra est la profondeur.
    _, a, _ = rectangle_minimal(plat_dessus)
    cam = np.array([vers_camera @ e1, vers_camera @ e2])
    cam /= max(1e-9, float(np.linalg.norm(cam)))
    axes = [np.array([math.cos(a), math.sin(a)]), np.array([-math.sin(a), math.cos(a)])]
    u = max(axes, key=lambda ax: abs(ax @ cam))
    u = -u if u @ cam > 0 else u                # u : de la caméra vers le fond du carton
    v = np.array([-u[1], u[0]])
    # 3. Centre, d'après le dessus (une main devant le carton n'y est pas) :
    #    le milieu s'il est vu en entier, sinon le bord avant (le plus sûr)
    #    plus une demi-profondeur ; en largeur, le milieu de ce qui est vu
    #    (la couverture, contrôlée par l'appelant, dit s'il manque un bord).
    pu, pv = plat @ u, plat @ v
    du_, dv_ = plat_dessus @ u, plat_dessus @ v
    avant, fond = float(np.quantile(du_, 0.01)), float(np.quantile(du_, 0.99))
    cu = (avant + fond) / 2 if fond - avant >= 0.85 * profondeur else avant + profondeur / 2
    gv, dv = float(np.quantile(dv_, 0.01)), float(np.quantile(dv_, 0.99))
    cv = (gv + dv) / 2
    centre_plat = cu * u + cv * v
    centre = centre_plat[0] * e1 + centre_plat[1] * e2 + (plancher + hauteur / 2) * haut
    dir_prof = u[0] * e1 + u[1] * e2             # profondeur, dans le repère des points
    lacet = math.atan2(float(dir_prof[1]), float(dir_prof[0]))
    # 4. Résidu et couverture du dessus.
    q = np.stack([pu - cu, pv - cv, hb - (plancher + hauteur / 2)], axis=1)
    demi = np.array([profondeur, largeur, hauteur]) / 2
    residu = float(np.sqrt(np.mean(_distance_boite(q, demi) ** 2)))
    qd = np.stack([plat_dessus @ u - cu, plat_dessus @ v - cv], axis=1)
    nu, nv = max(1, int(profondeur / cellule_m)), max(1, int(largeur / cellule_m))
    iu = np.floor((qd[:, 0] + profondeur / 2) / profondeur * nu).astype(int)
    iv = np.floor((qd[:, 1] + largeur / 2) / largeur * nv).astype(int)
    ok = (iu >= 0) & (iu < nu) & (iv >= 0) & (iv < nv)
    couverture = len(set(zip(iu[ok], iv[ok]))) / float(nu * nv)
    return BoiteEstimee(centre=centre, lacet=lacet,
                        vues=(fond - avant, dv - gv, dessus_h - plancher), residu_m=residu,
                        couverture=couverture, n_points=len(boite),
                        hauteur_table=table, points=boite)


class FusionCarton:
    """Nuages d'un même carton, image après image pendant l'approche (repère
    fixe : le monde), gardés `fenetre_s` ; la boîte est ajustée sur leur
    union."""

    def __init__(self, fenetre_s: float = 3.0, images_max: int = 6):
        self.fenetre_s, self.images_max = fenetre_s, images_max
        self._nuages: List[Tuple[float, np.ndarray]] = []

    def ajouter(self, t: float, points: np.ndarray) -> None:
        self._nuages = [(ti, p) for ti, p in self._nuages if t - ti <= self.fenetre_s]
        self._nuages = (self._nuages + [(t, np.asarray(points, dtype=float))])[-self.images_max:]

    def points(self) -> np.ndarray:
        return np.concatenate([p for _, p in self._nuages]) if self._nuages else np.zeros((0, 3))

    @property
    def images(self) -> int:
        return len(self._nuages)


def points_du_masque(depth: np.ndarray, intrinsics, rotation: np.ndarray, translation: np.ndarray,
                     masque: Optional[np.ndarray] = None,
                     bbox: Optional[Sequence[float]] = None, pas: int = 2) -> np.ndarray:
    """Points 3D (N × 3, repère de la pose caméra donnée par `rotation`,
    `translation`) des pixels du masque (ou du cadre `bbox`), d'après la
    profondeur (distance au plan image) et les paramètres de la caméra."""
    h, w = depth.shape[:2]
    if masque is None:
        x, y, bw, bh = (float(v) for v in bbox)
        masque = np.zeros((h, w), dtype=bool)
        masque[int(max(0, y)):int(min(h, y + bh)), int(max(0, x)):int(min(w, x + bw))] = True
    vs, us = np.nonzero(masque[::pas, ::pas])
    vs, us = vs * pas, us * pas
    d = np.asarray(depth, dtype=float)[vs, us]
    ok = np.isfinite(d) & (d > 1e-3)
    u, v, d = us[ok], vs[ok], d[ok]
    k = intrinsics
    opt = np.stack([(u - k.cx) / k.fx * d, (v - k.cy) / k.fy * d, d], axis=1)
    return opt @ np.asarray(rotation).T + np.asarray(translation)
