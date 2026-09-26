"""
Couleur des colis : l'IA trouve les boîtes, les pixels disent leur couleur.

OWL-ViT reconnaît « une boîte » ; qu'il distingue de façon fiable « une
boîte rouge » d'« une boîte bleue » n'a jamais été vérifié. On ne parie
pas dessus : chaque détection est gardée ou écartée selon la teinte
MESURÉE au centre de sa boîte englobante. C'est simple, déterministe,
testable sans GPU — et honnête sur qui fait quoi.

`PerceptionCouleur` a la même méthode find() que le PerceptionService : la
façade du tri l'utilise sans rien savoir de ce découpage. La requête reçue
est le NOM de la couleur (ConfigFacade(requete="{couleur}")).
"""
from __future__ import annotations

import colorsys
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from axis_os.hal.core.frames import FRAME_WORLD
from axis_os.services.base import ServiceResult

# Plages de teinte (degrés), saturation et luminosité minimales.
TEINTES: Dict[str, Tuple[Tuple[float, float], ...]] = {
    "rouge": ((0.0, 18.0), (340.0, 360.0)),
    "vert": ((85.0, 160.0),),
    "bleu": ((195.0, 255.0),),
    "jaune": ((45.0, 70.0),),
}
SATURATION_MIN = 0.45        # en dessous : gris, beige, carton brun pâle
LUMINOSITE_MIN = 0.12        # en dessous : trop sombre pour juger


def couleur_de(rgb: Tuple[float, float, float],
               couleurs: Iterable[str] = TEINTES) -> Optional[str]:
    """rgb dans [0, 1]. None si la couleur n'est pas franche."""
    h, s, v = colorsys.rgb_to_hsv(*rgb)
    if s < SATURATION_MIN or v < LUMINOSITE_MIN:
        return None
    deg = h * 360.0
    for nom in couleurs:
        if any(a <= deg < b for a, b in TEINTES.get(nom, ())):
            return nom
    return None


def couleur_boite(data: bytes, largeur: int, hauteur: int,
                  bbox: Sequence[int], couleurs: Iterable[str] = TEINTES,
                  part_centrale: float = 0.5, majorite: float = 0.6) -> Optional[str]:
    """Couleur majoritaire au CENTRE d'une boîte englobante, image rgb8.

    Le centre seulement : les bords d'une détection débordent sur la table
    ou le fond. Une couleur n'est rendue que si elle couvre `majorite` des
    pixels jugés — sinon None, plutôt qu'un tri au mauvais poste.
    """
    x, y, w, h = (int(v) for v in bbox)
    mx, my = int(w * (1 - part_centrale) / 2), int(h * (1 - part_centrale) / 2)
    x0, x1 = max(0, x + mx), min(largeur, x + w - mx)
    y0, y1 = max(0, y + my), min(hauteur, y + h - my)
    if x1 <= x0 or y1 <= y0:
        return None
    votes: Dict[Optional[str], int] = {}
    pas = max(1, min(x1 - x0, y1 - y0) // 12)      # ~150 pixels suffisent
    n = 0
    for yy in range(y0, y1, pas):
        for xx in range(x0, x1, pas):
            i = 3 * (yy * largeur + xx)
            c = couleur_de((data[i] / 255, data[i + 1] / 255, data[i + 2] / 255), couleurs)
            votes[c] = votes.get(c, 0) + 1
            n += 1
    if not n:
        return None
    meilleure = max((c for c in votes if c is not None), key=lambda c: votes[c],
                    default=None)
    if meilleure is None or votes[meilleure] < majorite * n:
        return None
    return meilleure


def _recouvrement(a, b) -> float:
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    ix = max(0, min(ax + aw, bx + bw) - max(ax, bx))
    iy = max(0, min(ay + ah, by + bh) - max(ay, by))
    inter = ix * iy
    union = aw * ah + bw * bh - inter
    return inter / union if union > 0 else 0.0


def sans_doublons(dets: List, seuil: float = 0.5) -> List:
    """Plusieurs formulations voient la même boîte : on garde la plus sûre."""
    gardees: List = []
    for d in sorted(dets, key=lambda d: -d.confidence):
        if all(_recouvrement(d.bbox, g.bbox) < seuil for g in gardees):
            gardees.append(d)
    return gardees


class PerceptionCouleur:
    """find(couleur) : boîtes détectées par l'IA, filtrées par leur couleur."""

    def __init__(self, perception, camera, couleurs: Iterable[str],
                 requetes: Sequence[str] = ("a box", "a cardboard box")):
        self.perception, self.camera = perception, camera
        self.couleurs = list(couleurs)
        self.requetes = tuple(requetes)
        self._cache_ns: Optional[int] = None
        self._cache: List = []
        self.stats = {"inferences": 0, "sans_couleur": 0}

    def _detections(self, now_ns, frame) -> Tuple[List, Optional[str]]:
        if now_ns is not None and now_ns == self._cache_ns:
            return self._cache, None
        dets, raison = [], None
        for rq in self.requetes:
            r = self.perception.find(rq, now_ns=now_ns, frame=frame)
            self.stats["inferences"] += 1
            if r.ok:
                dets += list(r.value or [])
            else:
                raison = r.reason
        dets = sans_doublons(dets)
        img = self.camera.capture()
        classees = []
        if img is not None:
            for d in dets:
                c = couleur_boite(img.data, img.width, img.height, d.bbox, self.couleurs)
                if c is None:
                    self.stats["sans_couleur"] += 1
                classees.append((c, d))
        self._cache_ns, self._cache = now_ns, classees
        return classees, (raison if not dets else None)

    def find(self, query: str, now_ns: Optional[int] = None,
             frame: str = FRAME_WORLD) -> ServiceResult:
        classees, raison = self._detections(now_ns, frame)
        if raison and not classees:
            return ServiceResult(ok=False, value=[], reason=raison)
        gardees = [d for c, d in classees if c == query]
        return ServiceResult(ok=True, value=sorted(gardees, key=lambda d: -d.confidence))
