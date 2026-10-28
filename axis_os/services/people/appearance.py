"""
AXIS-OS — Apparence globale d'une personne, pour la retrouver pendant
quelques secondes (ré-identification courte).

Ce que c'est : les couleurs des VÊTEMENTS dans deux bandes de la boîte de
détection, le haut du corps et les jambes, plus la forme générale de la
silhouette (le rapport hauteur sur largeur de la boîte). Le résultat est un
petit vecteur de DESCRIPTOR_SIZE nombres, calculé en mémoire à partir de
l'image ; l'image n'est pas gardée.

Ce que ce n'est PAS : une donnée biométrique. Le haut de la boîte, où se
trouve la tête, n'est JAMAIS lu (HEAD_FRACTION) : aucun pixel du visage
n'entre dans le calcul. Le vecteur ne permet ni de reconstruire une image ni
de reconnaître quelqu'un d'un jour à l'autre : deux personnes habillées
pareil ont le même, et la même personne qui retire sa veste en a un autre.
C'est voulu : il sert à ne pas perdre quelqu'un derrière un poteau, pas à
savoir qui il est.

Où il vit : dans la mémoire du service « personnes », jamais sur disque ni
dans un journal. Il disparaît avec sa piste (voir tracker.py), au plus tard
REID_MAX_S secondes après la dernière fois que la personne a été vue.
"""
from __future__ import annotations

from typing import Optional, Sequence, Tuple

import numpy as np

# Part du haut de la boîte jamais lue : la tête, donc le visage.
HEAD_FRACTION = 0.22
# Bandes lues, en fraction de la hauteur de la boîte : le haut du corps,
# puis les jambes. Seule la partie centrale en largeur est lue : les bords
# d'une boîte contiennent surtout le décor.
TORSO_BAND = (HEAD_FRACTION, 0.55)
LEGS_BAND = (0.55, 0.95)
CENTER_WIDTH = 0.6
# Histogramme de chaque bande : 8 teintes pour les pixels colorés, 3
# niveaux (sombre, moyen, clair) pour les pixels gris, noirs ou blancs.
HUE_BINS = 8
GREY_BINS = 3
BAND_SIZE = HUE_BINS + GREY_BINS
DESCRIPTOR_SIZE = 2 * BAND_SIZE + 1
# Un pixel est « coloré » au-delà de cette saturation et de cette valeur.
MIN_SATURATION = 0.25
MIN_VALUE = 0.20


def _hsv(rgb: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Teinte (dans [0, 1[), saturation et valeur de pixels RGB 8 bits."""
    c = rgb.astype(float) / 255.0
    r, g, b = c[..., 0], c[..., 1], c[..., 2]
    vmax, vmin = c.max(axis=-1), c.min(axis=-1)
    delta = vmax - vmin
    s = np.where(vmax > 0, delta / np.where(vmax > 0, vmax, 1.0), 0.0)
    d = np.where(delta > 0, delta, 1.0)
    h = np.where(vmax == r, ((g - b) / d) % 6.0,
                 np.where(vmax == g, (b - r) / d + 2.0, (r - g) / d + 4.0)) / 6.0
    h = np.where(delta > 0, h, 0.0)
    return h, s, vmax


def _band_histogram(pixels: np.ndarray) -> np.ndarray:
    """Histogramme normalisé (somme 1) d'une bande ; uniforme si vide."""
    if pixels.size == 0:
        return np.full(BAND_SIZE, 1.0 / BAND_SIZE)
    h, s, v = _hsv(pixels.reshape(-1, 3))
    colore = (s >= MIN_SATURATION) & (v >= MIN_VALUE)
    hist = np.zeros(BAND_SIZE)
    if colore.any():
        # Chaque teinte est partagée entre les deux cases les plus proches,
        # en cercle : sans cela, le rouge (teinte 0, ou presque 1) sautait
        # d'une case à l'autre au moindre bruit.
        pos = h[colore] * HUE_BINS - 0.5
        k0 = np.floor(pos)
        w1 = pos - k0
        k0 = k0.astype(int) % HUE_BINS
        hist[:HUE_BINS] = (np.bincount(k0, weights=1.0 - w1, minlength=HUE_BINS)
                           + np.bincount((k0 + 1) % HUE_BINS, weights=w1, minlength=HUE_BINS))
    if (~colore).any():
        idx = np.minimum((v[~colore] * GREY_BINS).astype(int), GREY_BINS - 1)
        hist[HUE_BINS:] = np.bincount(idx, minlength=GREY_BINS)
    return hist / hist.sum()


def appearance_descriptor(image: np.ndarray,
                          bbox: Sequence[float]) -> Optional[np.ndarray]:
    """Descripteur d'apparence de la personne dans la boîte `bbox` (x, y,
    largeur, hauteur, en pixels) de `image` (tableau H x L x 3, RGB 8
    bits). None si la boîte est trop petite pour être lue (moins de 8 x 16
    pixels dans l'image).

    La tête (HEAD_FRACTION du haut de la boîte) n'est jamais lue : seules
    les deux bandes TORSO_BAND et LEGS_BAND le sont, dans la partie centrale
    de la boîte."""
    img = np.asarray(image)
    if img.ndim != 3 or img.shape[2] < 3:
        return None
    H, W = img.shape[0], img.shape[1]
    x, y, w, h = (float(v) for v in bbox)
    if not all(np.isfinite((x, y, w, h))):
        return None
    x0, x1 = max(0, int(round(x))), min(W, int(round(x + w)))
    y0, y1 = max(0, int(round(y))), min(H, int(round(y + h)))
    if x1 - x0 < 8 or y1 - y0 < 16:
        return None
    marge = (x1 - x0) * (1.0 - CENTER_WIDTH) / 2.0
    cx0, cx1 = int(x0 + marge), int(x1 - marge)
    bandes = []
    for haut, bas in (TORSO_BAND, LEGS_BAND):
        by0 = max(y0, int(y + haut * h))
        by1 = min(y1, int(y + bas * h))
        bandes.append(_band_histogram(img[by0:by1, cx0:cx1, :3]))
    silhouette = min(max(h / max(w, 1e-6), 1.0), 5.0) / 5.0
    return np.concatenate(bandes + [np.array([silhouette])])


def appearance_distance(a: Optional[np.ndarray], b: Optional[np.ndarray]) -> Optional[float]:
    """Écart entre deux descripteurs, dans [0, 1] : 0 pour deux apparences
    identiques. None si l'un manque : l'apparence ne dit alors rien.

    Distance de Hellinger (une mesure classique de l'écart entre deux
    histogrammes) sur chaque bande, en moyenne, plus un faible poids pour
    la forme de la silhouette."""
    if a is None or b is None:
        return None
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    if a.shape != (DESCRIPTOR_SIZE,) or b.shape != (DESCRIPTOR_SIZE,):
        return None
    ecarts = []
    for k in range(2):
        p = a[k * BAND_SIZE:(k + 1) * BAND_SIZE]
        q = b[k * BAND_SIZE:(k + 1) * BAND_SIZE]
        bc = float(np.sum(np.sqrt(np.clip(p, 0, None) * np.clip(q, 0, None))))
        ecarts.append(float(np.sqrt(max(0.0, 1.0 - min(1.0, bc)))))
    forme = min(1.0, 2.0 * abs(float(a[-1]) - float(b[-1])))
    return 0.85 * (sum(ecarts) / 2.0) + 0.15 * forme


def synthetic_person_image(torso_rgb: Sequence[int], legs_rgb: Sequence[int], *,
                           width: int = 40, height: int = 100,
                           head_rgb: Sequence[int] = (200, 160, 130),
                           background_rgb: Sequence[int] = (120, 120, 120),
                           noise: float = 0.0,
                           rng: Optional[np.random.Generator] = None) -> np.ndarray:
    """Image synthétique d'une silhouette (tête, haut, jambes, sur un fond
    gris), pour les détecteurs factices : le descripteur se calcule alors
    par le même code que sur une vraie image. `noise` : bruit gaussien des
    couleurs (écart type, sur 255), pour imiter l'éclairage qui change."""
    img = np.empty((height, width, 3), dtype=float)
    img[:] = background_rgb
    corps = slice(int(width * 0.15), int(width * 0.85))
    img[:int(height * HEAD_FRACTION), int(width * 0.35):int(width * 0.65)] = head_rgb
    img[int(height * HEAD_FRACTION):int(height * 0.55), corps] = torso_rgb
    img[int(height * 0.55):int(height * 0.97), corps] = legs_rgb
    if noise > 0:
        rng = rng or np.random.default_rng()
        img += rng.normal(0.0, noise, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)
