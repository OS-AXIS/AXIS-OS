"""
Banc Isaac Sim T1 : la vue du robot, filmée par la caméra de tête (celle que
lit la perception), avec par-dessus ce que l'OS croit. Lecture seule : rien
ici n'agit sur ses décisions.

Dessiné depuis l'état de l'OS : cadres des détections (OWL-ViT, couleur et
score), points de la face avant retenus par la perception, contour 3D du
carton estimé et centre visé, cibles des mains (ouverture, contact, borne de
fermeture) reliées à la main mesurée, phase et contact estimé par main.
En pointillé et d'une autre couleur, légendés « vérité simulation » : le
contour RÉEL du carton et le contact réel du juge (banc_prise.JugeContacts).

Projection : les paramètres de la caméra que lit la perception
(get_intrinsics) et sa pose optique (optical_transform), x à droite, y en
bas, z devant.
"""
from __future__ import annotations

import math
from typing import Dict, Iterable, Optional, Sequence

import numpy as np

COULEURS = {"rouge": (230, 60, 50), "bleu": (60, 130, 240), "vert": (60, 190, 90),
            "jaune": (240, 200, 40)}
OS = (255, 220, 60)              # ce que l'OS estime
VERITE = (255, 80, 220)          # vérité de la simulation (pointillés)
MAIN = (90, 230, 255)
ARETES = [(0, 1), (1, 3), (3, 2), (2, 0), (4, 5), (5, 7), (7, 6), (6, 4),
          (0, 4), (1, 5), (2, 6), (3, 7)]


def projeter(points, k, tf_optique):
    """Points du repère de la pose `tf_optique` (repère parent -> optique de
    la caméra) en pixels : (N×2, masque des points devant la caméra)."""
    p = np.atleast_2d(np.asarray(points, dtype=float))
    o = tf_optique.inverse()
    t = o.apply(_v(0.0, 0.0, 0.0))
    rot = np.array([[c.x, c.y, c.z] for c in (o.apply_direction(_v(*e)) for e in np.eye(3))]).T
    q = p @ rot.T + np.array([t.x, t.y, t.z])
    devant = q[:, 2] > 1e-3
    z = np.where(devant, q[:, 2], 1.0)
    return np.stack([k.fx * q[:, 0] / z + k.cx, k.fy * q[:, 1] / z + k.cy], axis=1), devant


def _v(x, y, z):
    from axis_os.hal.core.frames import Vector3
    return Vector3(float(x), float(y), float(z))


def coins(centre, largeur, profondeur, hauteur, lacet) -> np.ndarray:
    """Les 8 coins d'une boîte : `largeur` le long de la normale
    (-sin lacet, cos lacet), `profondeur` le long de (cos lacet, sin lacet)."""
    a = np.array([math.cos(lacet), math.sin(lacet), 0.0])
    n = np.array([-math.sin(lacet), math.cos(lacet), 0.0])
    c = np.asarray(centre, dtype=float)
    return np.array([c + sx * profondeur / 2 * a + sy * largeur / 2 * n
                     + sz * hauteur / 2 * np.array([0.0, 0.0, 1.0])
                     for sz in (-1, 1) for sx in (-1, 1) for sy in (-1, 1)])


def _trait(d, a, b, couleur, largeur=2, pointille=False):
    if not pointille:
        d.line([tuple(a), tuple(b)], fill=couleur, width=largeur)
        return
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = max(1, int(np.linalg.norm(b - a) / 6))
    for i in range(0, n, 2):
        d.line([tuple(a + (b - a) * i / n), tuple(a + (b - a) * min(n, i + 1) / n)],
               fill=couleur, width=largeur)


def _boite(d, k, tf, pts, couleur, pointille=False):
    uv, ok = projeter(pts, k, tf)
    for i, j in ARETES:
        if ok[i] and ok[j]:
            _trait(d, uv[i], uv[j], couleur, 2, pointille)


def _point(d, k, tf, p, couleur, r=4, plein=True):
    uv, ok = projeter(p, k, tf)
    if ok[0]:
        x, y = uv[0]
        d.ellipse([x - r, y - r, x + r, y + r], fill=couleur if plein else None,
                  outline=couleur, width=2)
        return uv[0]
    return None


def dessiner(img, k, tf, couches: Dict, verite: bool = True):
    """Dessine `couches` (repère monde, voir banc T1) sur l'image PIL `img`
    (modifiée et rendue).

    couches : detections [(bbox, couleur, score)], faces [N×3],
    boite (8×3), centre, cibles {nom: (g, d)}, mains (g, d), phase,
    contacts {"left"/"right": bool} ; verite : {"boite": 8×3,
    "contacts": {"left"/"right": force en N ou None}}."""
    from PIL import ImageDraw

    from axis_os.hal.drivers.isaac_sim.video import _police
    f = _police(13)                      # une police qui dessine les accents
    d = ImageDraw.Draw(img)
    for bbox, couleur, score in couches.get("detections", ()):
        x, y, w, h = bbox
        c = COULEURS.get(couleur, (200, 200, 200))
        d.rectangle([x, y, x + w, y + h], outline=c, width=2)
        d.text((x + 3, max(0, y - 15)), f"{couleur or '?'} {score:.2f}", fill=c, font=f)
    for pts in couches.get("faces", ()):
        uv, ok = projeter(pts, k, tf)
        for x, y in uv[ok][::3]:
            d.point((float(x), float(y)), fill=OS)
    if couches.get("boite") is not None:
        _boite(d, k, tf, couches["boite"], OS)
    if couches.get("centre") is not None:
        _point(d, k, tf, couches["centre"], OS, 5, plein=False)
    mains = couches.get("mains")
    for nom, paire in couches.get("cibles", {}).items():
        for i, p in enumerate(paire or ()):
            uv = _point(d, k, tf, p, OS, 3)
            if uv is not None:
                d.text((uv[0] + 5, uv[1] - 5), nom[:3], fill=OS, font=f)
                if mains is not None and nom == "borne":
                    m = _point(d, k, tf, mains[i], MAIN, 5)
                    if m is not None:
                        _trait(d, uv, m, MAIN, 1)
    if mains is not None:
        for p in mains:
            _point(d, k, tf, p, MAIN, 5)
    contacts = couches.get("contacts", {})
    lignes = [f"phase {couches.get('phase', '?')}",
              "contact estimé : G " + ("oui" if contacts.get("left") else "non")
              + ", D " + ("oui" if contacts.get("right") else "non")]
    for i, l in enumerate(lignes):
        d.text((8, 8 + 16 * i), l, fill=OS, font=f)
    v = couches.get("verite") if verite else None
    if v:
        if v.get("boite") is not None:
            _boite(d, k, tf, v["boite"], VERITE, pointille=True)
        cr = v.get("contacts", {})
        reel = ", ".join(f"{n} {'%.0f N' % cr[c] if cr.get(c) else 'non'}"
                         for c, n in (("left", "G"), ("right", "D")))
        d.text((8, 8 + 16 * len(lignes)), f"vérité simulation (pointillés) — contact réel : "
               f"{reel}", fill=VERITE, font=f)
    return img


class VueRobot:
    """Vidéo de la vue du robot (640×480, 15 im/s par défaut) ; image()
    rend aussi l'image dessinée (incrustation, suivi en direct)."""

    def __init__(self, chemin: str, ips: int = 15, verite: bool = True):
        self.chemin, self.ips, self.verite = chemin, ips, verite
        self._writer = None
        self.images = 0

    def image(self, rgb: np.ndarray, k, tf, couches: Dict):
        from PIL import Image
        img = dessiner(Image.fromarray(np.asarray(rgb, dtype=np.uint8)).convert("RGB"),
                       k, tf, couches, self.verite)
        if self._writer is None:
            try:
                import imageio.v2 as imageio
                self._writer = imageio.get_writer(self.chemin, fps=self.ips, codec="libx264",
                                                  quality=6, macro_block_size=8)
            except Exception as exc:        # noqa: BLE001
                print(f"  [vue du robot] écriture MP4 impossible ({exc})", flush=True)
                self._writer = False
        if self._writer:
            self._writer.append_data(np.asarray(img))
            self.images += 1
        return img

    def terminer(self) -> None:
        if self._writer:
            self._writer.close()
