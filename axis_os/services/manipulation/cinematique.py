"""
AXIS-OS — Cinématique des bras, lue dans la description URDF du robot.

Cinématique directe : la position des points d'un bras (épaule, coude,
poignet, main) à partir des angles de ses articulations, dans le repère du
bassin (x devant, y à gauche, z en haut). Cinématique inverse : les angles
qui mettent la main en un point donné, avec les trois articulations qui
font la prise à deux bras (tangage et roulis d'épaule, coude) ; les autres
articulations de la chaîne (taille, lacet d'épaule, poignets) gardent la
valeur donnée au départ, mesurée sur le robot (zéro si on ne la donne pas).
La chaîne va du bassin au repère de la main (rubber_hand), main comprise.

Les angles sont BRUTS, dans la convention de l'URDF, comme les postures du
service de manipulation (voir Postures). Les origines, rotations et axes
sont ceux du fichier : rien n'est recopié à la main.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

URDF_G1_29DOF = (Path(__file__).resolve().parents[2]
                 / "hal/drivers/unitree_g1/urdf/g1_29dof_rev_1_0.urdf")

# Points d'un bras, du bassin à la main.
EPAULE, COUDE, POIGNET, MAIN = "epaule", "coude", "poignet", "main"


def _rpy(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _rotation(axe: np.ndarray, q: float) -> np.ndarray:
    k = np.array([[0.0, -axe[2], axe[1]], [axe[2], 0.0, -axe[0]], [-axe[1], axe[0], 0.0]])
    return np.eye(3) + math.sin(q) * k + (1.0 - math.cos(q)) * k @ k


@dataclass(frozen=True)
class _Lien:
    nom: str                       # articulation
    enfant: str                    # lien enfant
    origine: np.ndarray            # 4x4
    axe: Optional[np.ndarray]      # None : articulation fixe
    bornes: Tuple[float, float]


class ChaineBras:
    """Un bras, du bassin à la main (le lien `fin` de l'URDF)."""

    def __init__(self, liens: Sequence[_Lien], cote: str):
        self._liens = list(liens)
        self.cote = cote
        p = f"{cote}_"
        self.tangage, self.roulis = p + "shoulder_pitch_joint", p + "shoulder_roll_joint"
        self.coude = p + "elbow_joint"
        self._points = {p + "shoulder_pitch_link": EPAULE, p + "elbow_link": COUDE,
                        p + "wrist_roll_link": POIGNET}
        self.bornes = {l.nom: l.bornes for l in self._liens if l.axe is not None}

    @classmethod
    def depuis_urdf(cls, cote: str, chemin: Path = URDF_G1_29DOF,
                    base: str = "pelvis") -> "ChaineBras":
        racine = ET.parse(str(chemin)).getroot()
        par_enfant = {j.find("child").get("link"): j for j in racine.findall("joint")}
        fin = f"{cote}_rubber_hand"
        liens, lien = [], fin
        while lien != base:
            j = par_enfant[lien]
            o = j.find("origin")
            xyz = [float(v) for v in (o.get("xyz") or "0 0 0").split()] if o is not None \
                else [0.0, 0.0, 0.0]
            rpy = [float(v) for v in (o.get("rpy") or "0 0 0").split()] if o is not None \
                else [0.0, 0.0, 0.0]
            m = np.eye(4)
            m[:3, :3], m[:3, 3] = _rpy(*rpy), xyz
            axe, bornes = None, (0.0, 0.0)
            if j.get("type") in ("revolute", "continuous"):
                a = np.array([float(v) for v in j.find("axis").get("xyz").split()])
                axe = a / np.linalg.norm(a)
                lim = j.find("limit")
                bornes = (float(lim.get("lower")), float(lim.get("upper")))
            liens.append(_Lien(j.get("name"), lien, m, axe, bornes))
            lien = j.find("parent").get("link")
        return cls(liens[::-1], cote)

    def points(self, q: Dict[str, float]) -> Dict[str, np.ndarray]:
        """Épaule, coude, poignet et main, repère du bassin (m)."""
        t = np.eye(4)
        sortie = {}
        for l in self._liens:
            t = t @ l.origine
            if l.axe is not None:
                r = np.eye(4)
                r[:3, :3] = _rotation(l.axe, float(q.get(l.nom, 0.0)))
                t = t @ r
            nom = self._points.get(l.enfant)
            if nom is not None:
                sortie[nom] = t[:3, 3].copy()
        sortie[MAIN] = t[:3, 3].copy()
        return sortie

    def main(self, q: Dict[str, float]) -> np.ndarray:
        return self.points(q)[MAIN]

    def dans_les_bornes(self, q: Dict[str, float], marge: float = 0.02) -> bool:
        return all(self.bornes[j][0] + marge <= v <= self.bornes[j][1] - marge
                   for j, v in q.items() if j in self.bornes)

    def inverse(self, cible: np.ndarray, depart: Dict[str, float], *,
                tol: float = 0.003, iterations: int = 60
                ) -> Tuple[Optional[Dict[str, float]], float]:
        """Angles (tangage, roulis, coude) qui mettent la main en `cible`
        (repère du bassin), en partant de `depart` ; moindres carrés amortis,
        dans les bornes de l'URDF. Les autres articulations de la chaîne
        restent à leur valeur dans `depart` (0 si absente ou non finie).
        Rend (angles des trois articulations, erreur en m) ; angles None si
        la main n'y arrive pas à `tol` près."""
        noms = (self.tangage, self.roulis, self.coude)
        q = {}
        for nom in self.bornes:
            v = depart.get(nom, 0.0)
            q[nom] = float(v) if v is not None and math.isfinite(float(v)) else 0.0
        cible = np.asarray(cible, dtype=float)
        erreur = math.inf
        for _ in range(iterations):
            e = cible - self.main(q)
            erreur = float(np.linalg.norm(e))
            if erreur < tol:
                break
            jac = np.zeros((3, 3))
            for i, n in enumerate(noms):
                q2 = dict(q)
                q2[n] += 1e-5
                jac[:, i] = (self.main(q2) - self.main(q)) / 1e-5
            dq = jac.T @ np.linalg.solve(jac @ jac.T + 1e-4 * np.eye(3), e)
            for i, n in enumerate(noms):
                lo, hi = self.bornes[n]
                q[n] = min(hi, max(lo, q[n] + float(np.clip(dq[i], -0.3, 0.3))))
        return ({n: q[n] for n in noms} if erreur < tol else None), erreur

    def avant_bras(self, q: Dict[str, float], n: int = 8) -> List[np.ndarray]:
        """Points de l'avant-bras, du coude à la main (repère du bassin)."""
        p = self.points(q)
        pts = []
        for a, b in ((p[COUDE], p[POIGNET]), (p[POIGNET], p[MAIN])):
            pts += [a + (b - a) * k / n for k in range(n)]
        return pts + [p[MAIN]]


class ModeleBras:
    """Les deux bras d'un robot à deux bras symétriques (le G1)."""

    def __init__(self, chemin: Path = URDF_G1_29DOF):
        self.gauche = ChaineBras.depuis_urdf("left", chemin)
        self.droite = ChaineBras.depuis_urdf("right", chemin)

    def mains(self, q: Dict[str, float]) -> Tuple[np.ndarray, np.ndarray]:
        return self.gauche.main(q), self.droite.main(q)
