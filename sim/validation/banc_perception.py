"""
Banc Isaac Sim T1 : images de la perception, pour comparer les détecteurs
hors ligne (OWL-ViT contre OWLv2, boîte ajustée avec ou sans masque SAM 2.1).

Enregistrement, pendant le T1 (--images-perception DOSSIER) : pendant une
recherche, au plus une image par seconde simulée, un fichier .npz : image
couleur, profondeur, paramètres de la caméra, pose optique que l'OS tire du
montage de la caméra (repère monde) et, VÉRITÉ SIMULATION, les cartons de la
table de prise (centre, lacet, demi-tailles). L'OS n'en lit rien.

Comparaison, dans le conteneur (le GPU) :
  /isaac-sim/python.sh /isaac-sim/axis_os/banc_perception.py DOSSIER
Pour chaque détecteur et chaque seuil : cartons vus (cadre de recouvrement
IoU ≥ 0,5 avec le cadre vrai), fausses détections, IoU moyen ; puis, au
seuil de travail, l'erreur de la boîte ajustée (centre, lacet) avec le
masque SAM 2.1 et avec le cadre seul, et les prises qu'elle refuserait
(« carton mal vu »). Verdict : celui qui voit le plus de cartons sans plus
de fausses détections ; à égalité, la plus petite erreur de centre.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Callable, Dict, Iterable, List, Optional, Sequence

import numpy as np

# Les modules d'AXIS-OS : à côté de ce fichier dans le conteneur
# (/isaac-sim/axis_os/), à deux dossiers au-dessus dans le dépôt.
for _p in (Path(__file__).resolve().parent.parent, Path(__file__).resolve().parents[2]):
    if (_p / "axis_os").is_dir() and str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from axis_os.hal.core.types import CameraFrame, CameraIntrinsics  # noqa: E402
from axis_os.services.perception.boite import ajuster_boite, points_du_masque  # noqa: E402

REQUETES = ("a box", "a cardboard box")          # celles de PerceptionCouleur
RESIDU_MAX_M, COUVERTURE_MIN = 0.015, 0.6        # ConfigFacade : « carton mal vu » au-delà


class EnregistreurImages:
    """Images de la perception (voir le module) ; `verite()` rend les
    cartons de la table de prise : [(couleur, centre xyz, lacet, demi-tailles)]."""

    def __init__(self, dossier: str, cam, dep, pose_optique: Callable, verite: Callable,
                 periode_s: float = 1.0, max_images: int = 400):
        self.dossier = Path(dossier)
        self.dossier.mkdir(parents=True, exist_ok=True)
        self._cam, self._dep, self._pose, self._verite = cam, dep, pose_optique, verite
        self.periode_s, self.max_images = periode_s, max_images
        self.images, self._t = 0, None

    def prendre(self, t: float) -> bool:
        if self.images >= self.max_images or (self._t is not None
                                              and t - self._t < self.periode_s):
            return False
        f, d, k = self._cam.capture(), self._dep.capture_depth(), self._cam.get_intrinsics()
        pose = self._pose()
        if f is None or d is None or k is None or pose is None:
            return False
        cartons = list(self._verite())
        rgb = np.frombuffer(f.data, dtype=np.uint8).reshape(f.height, f.width, 3)
        np.savez_compressed(
            self.dossier / f"image_{self.images:04d}.npz", rgb=rgb,
            profondeur=np.asarray(d, dtype=np.float32), k=np.array([k.fx, k.fy, k.cx, k.cy]),
            rot=pose[0], trans=pose[1], t=t,
            couleurs=np.array([c[0] for c in cartons], dtype=str),
            centres=np.array([c[1] for c in cartons], dtype=float).reshape(-1, 3),
            lacets=np.array([c[2] for c in cartons], dtype=float),
            demis=np.array([c[3] for c in cartons], dtype=float).reshape(-1, 3))
        self.images, self._t = self.images + 1, t
        return True


def charger(dossier: str) -> List[dict]:
    images = []
    for f in sorted(Path(dossier).glob("image_*.npz")):
        with np.load(f) as z:
            images.append({n: z[n] for n in z.files})
    return images


def _coins(centre, lacet, demi) -> np.ndarray:
    c, s = math.cos(lacet), math.sin(lacet)
    rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return np.array([np.asarray(centre, float) + rz @ (np.asarray(demi, float) * [sx, sy, sz])
                     for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])


def _pixels(points, im) -> np.ndarray:
    """Points du monde -> (u, v, z optique) dans la caméra de l'image `im`."""
    fx, fy, cx, cy = im["k"]
    q = (np.atleast_2d(points) - im["trans"]) @ im["rot"]
    z = q[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        return np.stack([fx * q[:, 0] / z + cx, fy * q[:, 1] / z + cy, z], axis=1)


def cartons_a_voir(im) -> List[dict]:
    """Cartons vrais de l'image, avec leur cadre (x, y, l, h) en pixels :
    seulement ceux devant la caméra, dans l'image, et dont le milieu du
    dessus n'est pas caché (profondeur à 4 cm près)."""
    h, w = im["profondeur"].shape
    sortie = []
    for i in range(len(im["centres"])):
        c, lacet, demi = im["centres"][i], float(im["lacets"][i]), im["demis"][i]
        uvz = _pixels(_coins(c, lacet, demi), im)
        if (uvz[:, 2] <= 0.05).any():
            continue
        x0, y0 = max(0.0, uvz[:, 0].min()), max(0.0, uvz[:, 1].min())
        x1, y1 = min(w - 1.0, uvz[:, 0].max()), min(h - 1.0, uvz[:, 1].max())
        if (x1 - x0) * (y1 - y0) < 400:
            continue
        u, v, z = _pixels(np.asarray(c, float) + [0.0, 0.0, demi[2]], im)[0]
        if not (0 <= u < w and 0 <= v < h) or abs(float(im["profondeur"][int(v), int(u)]) - z) > 0.04:
            continue
        sortie.append({"couleur": str(im["couleurs"][i]), "centre": np.asarray(c, float),
                       "lacet": lacet, "cadre": (x0, y0, x1 - x0, y1 - y0)})
    return sortie


def iou(a: Sequence[float], b: Sequence[float]) -> float:
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    ix = max(0.0, min(ax + aw, bx + bw) - max(ax, bx))
    iy = max(0.0, min(ay + ah, by + bh) - max(ay, by))
    inter = ix * iy
    return inter / max(1e-9, aw * ah + bw * bh - inter)


def detections(detecteur, im, requetes: Iterable[str] = REQUETES) -> List[tuple]:
    """(cadre, confiance) des requêtes, doublons retirés (IoU > 0,7 : la
    plus confiante), comme PerceptionCouleur."""
    rgb = np.ascontiguousarray(im["rgb"], dtype=np.uint8)
    frame = CameraFrame(width=rgb.shape[1], height=rgb.shape[0], encoding="rgb8",
                        data=rgb.tobytes())
    toutes = sorted(((tuple(float(v) for v in d.bbox), float(d.confidence))
                     for q in requetes for d in detecteur.detect(frame, q)),
                    key=lambda d: -d[1])
    gardees: List[tuple] = []
    for d in toutes:
        if all(iou(d[0], g[0]) <= 0.7 for g in gardees):
            gardees.append(d)
    return gardees


def apparier(dets: List[tuple], vrais: List[dict], iou_min: float = 0.5):
    """Appariement glouton, plus confiantes d'abord : [(détection, vrai, IoU)]
    et le nombre de fausses détections."""
    libres, paires, fausses = list(range(len(vrais))), [], 0
    for d in dets:
        meilleur = max(libres, key=lambda i: iou(d[0], vrais[i]["cadre"]), default=None)
        if meilleur is not None and iou(d[0], vrais[meilleur]["cadre"]) >= iou_min:
            paires.append((d, vrais[meilleur], iou(d[0], vrais[meilleur]["cadre"])))
            libres.remove(meilleur)
        else:
            fausses += 1
    return paires, fausses


def erreur_boite(im, cadre, vrai, dims, segmenteur=None) -> Optional[dict]:
    """Boîte ajustée sur les points du masque (ou du cadre) d'une image :
    erreurs de centre (m) et de lacet (degrés, rectangle replié à ±45°),
    et refus « carton mal vu » de la façade."""
    masque = None
    if segmenteur is not None:
        rgb = np.ascontiguousarray(im["rgb"], dtype=np.uint8)
        masque = segmenteur.masque(CameraFrame(width=rgb.shape[1], height=rgb.shape[0],
                                               encoding="rgb8", data=rgb.tobytes()), cadre)
    fx, fy, cx, cy = (float(v) for v in im["k"])
    pts = points_du_masque(im["profondeur"], CameraIntrinsics(fx=fx, fy=fy, cx=cx, cy=cy),
                           im["rot"], im["trans"], masque=masque, bbox=cadre)
    vers = np.asarray(im["trans"], float) - vrai["centre"]
    b = ajuster_boite(pts, dims, np.array([vers[0], vers[1], 0.0]))
    if b is None:
        return None
    return {"centre_m": float(np.linalg.norm(b.centre - vrai["centre"])),
            "lacet_deg": abs((math.degrees(b.lacet - vrai["lacet"]) + 45.0) % 90.0 - 45.0),
            "refus": b.residu_m > RESIDU_MAX_M or b.couverture < COUVERTURE_MIN,
            "masque": masque is not None}


def evaluer(images: List[dict], detecteur, seuils: Sequence[float], dims, seuil_travail: float,
            segmenteur=None, requetes: Iterable[str] = REQUETES) -> dict:
    """Bilan d'un détecteur sur les images (voir le module)."""
    par_seuil = {s: {"vus": 0, "fausses": 0, "iou": []} for s in seuils}
    a_voir = 0
    boites: Dict[str, List[dict]] = {"masque": [], "cadre": []}
    for im in images:
        vrais = cartons_a_voir(im)
        a_voir += len(vrais)
        dets = detections(detecteur, im, requetes)
        for s in seuils:
            paires, fausses = apparier([d for d in dets if d[1] >= s], vrais)
            par_seuil[s]["vus"] += len(paires)
            par_seuil[s]["fausses"] += fausses
            par_seuil[s]["iou"] += [p[2] for p in paires]
        for d, vrai, _ in apparier([d for d in dets if d[1] >= seuil_travail], vrais)[0]:
            for nom, seg in (("masque", segmenteur), ("cadre", None)):
                if nom == "masque" and seg is None:
                    continue
                e = erreur_boite(im, d[0], vrai, dims, seg)
                if e is not None and (nom == "cadre" or e["masque"]):
                    boites[nom].append(e)
    return {"a_voir": a_voir, "seuils": par_seuil, "boites": boites}


def _resume(e: List[dict]) -> str:
    if not e:
        return "aucune boîte"
    c = np.array([x["centre_m"] for x in e]) * 100
    a = np.array([x["lacet_deg"] for x in e])
    return (f"centre {np.median(c):.1f} cm (90 % sous {np.percentile(c, 90):.1f}), lacet "
            f"{np.median(a):.1f}° (90 % sous {np.percentile(a, 90):.1f}), refusées "
            f"{sum(x['refus'] for x in e)}/{len(e)}")


def rapport(bilans: Dict[str, dict], seuils_travail: Dict[str, float]) -> str:
    """Tableau lisible et verdict (voir le module)."""
    lignes = []
    for nom, b in bilans.items():
        lignes.append(f"\n  {nom} — {b['a_voir']} cartons à voir")
        for s, r in b["seuils"].items():
            moy = np.mean(r["iou"]) if r["iou"] else 0.0
            marque = "  ← seuil de travail" if abs(s - seuils_travail[nom]) < 1e-9 else ""
            lignes.append(f"    seuil {s:.2f} : vus {r['vus']}/{b['a_voir']}, fausses "
                          f"{r['fausses']}, IoU moyen {moy:.2f}{marque}")
        for m in ("masque", "cadre"):
            if b["boites"][m] or m == "cadre":
                lignes.append(f"    boîte ajustée ({'masque SAM 2.1' if m == 'masque' else 'cadre seul'}) : "
                              f"{_resume(b['boites'][m])}")

    def cle(nom):
        r = bilans[nom]["seuils"][seuils_travail[nom]]
        e = bilans[nom]["boites"]["masque"] or bilans[nom]["boites"]["cadre"]
        return (-r["vus"], r["fausses"], np.median([x["centre_m"] for x in e]) if e else 1e9)
    meilleur = min(bilans, key=cle)
    lignes.append(f"\n  Verdict : garder {meilleur} (le plus de cartons vus au seuil de travail, "
                  f"sans plus de fausses détections ; à égalité, la plus petite erreur de centre)")
    return "\n".join(lignes)


def main(argv=None) -> int:
    from axis_os.services.perception.owlvit_detector import Owlv2Detector, OwlViTDetector
    from axis_os.services.perception.sam2_segmenteur import Sam2Segmenteur
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("dossier")
    ap.add_argument("--seuils", default="0.04,0.06,0.08,0.10,0.15,0.20")
    ap.add_argument("--sans-sam2", action="store_true")
    ap.add_argument("--dims", default="0.24,0.20,0.20",
                    help="largeur, profondeur, hauteur du carton (m), celles de l'app")
    args = ap.parse_args(argv)
    images = charger(args.dossier)
    if not images:
        print(f"Aucune image dans {args.dossier} (lancer le T1 avec --images-perception).")
        return 1
    seuils = [float(s) for s in args.seuils.split(",")]
    travail = {"OWL-ViT": 0.06, "OWLv2": 0.10}       # ceux du T1 par défaut
    seuils = sorted(set(seuils) | set(travail.values()))
    seg = None
    if not args.sans_sam2:
        seg = Sam2Segmenteur()
        if not seg.load():
            print(f"  [sam2] {seg.erreur} : cadre seul")
            seg = None
    dims = tuple(float(v) for v in args.dims.split(","))
    bilans = {}
    for nom, cls in (("OWL-ViT", OwlViTDetector), ("OWLv2", Owlv2Detector)):
        det = cls(threshold=min(seuils))
        if not det.load():
            print(f"  {nom} : chargement impossible ({det.stats.get('load_error')})")
            continue
        bilans[nom] = evaluer(images, det, seuils, dims, travail[nom], seg)
    if not bilans:
        return 1
    print(f"\n  {len(images)} images de {args.dossier}" + rapport(bilans, travail))
    return 0


if __name__ == "__main__":
    sys.exit(main())
