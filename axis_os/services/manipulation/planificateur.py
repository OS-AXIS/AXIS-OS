"""
AXIS-OS — Planificateur de prise : où se placer, et face à quelle paire de
faces, pour prendre à deux mains une boîte posée sur une table.

À partir de la boîte estimée (ses 8 coins, repère de la pièce) et du bord de
la table qui la porte :
  1. candidates : chaque paire de faces verticales opposées (deux paires),
     le robot face à elle (les mains sur ces deux faces) ; pour chacune, des
     positions de base de PAS_M en PAS_M, en profondeur et en travers, dans
     la zone permise par la règle du bord : pieds derrière le bord, erreur
     d'approche de la marche comprise, bord vu en biais ;
  2. chaque candidate est évaluée SANS bouger, par les contrôles de la prise
     (ManipulationService.evaluer_prise : pieds, largeur, portée, garde des
     avant-bras, ligne des mains par le centre de gravité, couples des bras
     pour le serrage d'après le poids estimé, équilibre ; puis, pour les
     meilleures, chemin des bras au retour et dépose) : aucun seuil nouveau ;
  3. score : la plus faible des marges, chacune rapportée à son propre seuil
     (0 : juste au seuil ; négative : refusée) ; la meilleure candidate qui
     passe est retenue. Aucune : refus, avec la raison de la MEILLEURE.

Rien ici ne commande le robot : la pose choisie est donnée à l'appelant
(campagne de cartons, pick() du SDK), qui l'y conduit.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

PAS_M = 0.02                        # entre deux positions de base candidates
DISTANCES_M = (0.30, 0.62)          # du bassin au centre de la boîte, le long du cap
TRAVERS_M = 0.08                    # de part et d'autre de l'axe de la boîte
COMPLETES_MAX = 4                   # candidates dont on vérifie aussi les chemins


@dataclass(frozen=True)
class BordTable:
    """Le bord de la table qui porte la boîte : un point du bord (x, y,
    repère de la pièce), la direction (rad) de sa normale VERS la table, la
    hauteur du plateau (m)."""
    point: Tuple[float, float]
    normale: float
    plateau_z: float

    def au_dela(self, x: float, y: float) -> float:
        """De combien (m) le point (x, y) est au-delà du bord, au-dessus de
        la table (négatif : devant elle)."""
        return ((x - self.point[0]) * math.cos(self.normale)
                + (y - self.point[1]) * math.sin(self.normale))


@dataclass(frozen=True)
class Boite:
    """Boîte posée : centre (repère de la pièce), lacet (rad) de son côté
    `a`, côtés horizontaux `a` et `b` (le second à lacet + π/2), hauteur."""
    centre: Tuple[float, float, float]
    lacet: float
    a: float
    b: float
    hauteur: float


def boite_depuis_coins(coins: Sequence[Sequence[float]]) -> Boite:
    """La boîte d'après ses 8 coins (repère de la pièce) : rectangle d'aire
    minimale de leur empreinte au sol, hauteur de leur étendue verticale."""
    from axis_os.services.perception.boite import rectangle_minimal
    p = np.asarray(coins, dtype=float)
    if p.shape != (8, 3) or not np.all(np.isfinite(p)):
        raise ValueError("8 coins finis (x, y, z) attendus")
    c, angle, (a, b) = rectangle_minimal(p[:, :2])
    z0, z1 = float(p[:, 2].min()), float(p[:, 2].max())
    return Boite((float(c[0]), float(c[1]), (z0 + z1) / 2.0), float(angle), float(a), float(b),
                 z1 - z0)


def coins_boite(centre: Sequence[float], lacet: float, a: float, b: float,
                hauteur: float) -> np.ndarray:
    """Les 8 coins (8 × 3) d'une boîte posée (voir Boite)."""
    u = np.array([math.cos(lacet), math.sin(lacet), 0.0])
    v = np.array([-u[1], u[0], 0.0])
    c = np.asarray(centre, dtype=float)
    return np.array([c + su * a / 2.0 * u + sv * b / 2.0 * v + np.array([0.0, 0.0, sz * hauteur / 2.0])
                     for su in (-1, 1) for sv in (-1, 1) for sz in (-1, 1)])


def _angle(a: float) -> float:
    return math.remainder(a, 2.0 * math.pi)


@dataclass(frozen=True)
class Paire:
    """Une paire de faces opposées, vue du robot qui lui fait face : son cap
    (rad, repère de la pièce), la largeur prise entre les mains, la
    profondeur (le long du cap)."""
    numero: int
    cap: float
    largeur: float
    profondeur: float


def paires(boite: Boite, bord: BordTable) -> List[Paire]:
    """Les deux paires de faces verticales opposées. Paire 0 : les mains sur
    les faces le long du côté `a` (largeur `b`) ; paire 1 : largeur `a`. Le
    cap fait face à la table : des deux sens possibles, le plus proche de la
    normale du bord."""
    sortie = []
    for k, (cap0, largeur, profondeur) in enumerate(
            ((boite.lacet, boite.b, boite.a), (boite.lacet + math.pi / 2.0, boite.a, boite.b))):
        cap = min((cap0, cap0 + math.pi), key=lambda c: abs(_angle(c - bord.normale)))
        sortie.append(Paire(k, _angle(cap), largeur, profondeur))
    return sortie


def positions(boite: Boite, paire: Paire, pas_m: float = PAS_M
              ) -> List[Tuple[float, float, float, float, float]]:
    """Positions de base face à la paire : (x, y, cap, d, s), le bassin à `d`
    du centre de la boîte le long du cap, décalé de `s` en travers (positif :
    la boîte à droite du robot), de pas_m en pas_m."""
    u = (math.cos(paire.cap), math.sin(paire.cap))
    v = (-u[1], u[0])
    cx, cy = boite.centre[0], boite.centre[1]
    sortie = []
    n_d = int(round((DISTANCES_M[1] - DISTANCES_M[0]) / pas_m))
    n_s = int(round(TRAVERS_M / pas_m))
    for i in range(n_d + 1):
        d = DISTANCES_M[0] + i * pas_m
        for j in range(-n_s, n_s + 1):
            s = j * pas_m
            sortie.append((cx - d * u[0] + s * v[0], cy - d * u[1] + s * v[1], paire.cap, d, s))
    return sortie


@dataclass
class Marge:
    """Un contrôle de la prise : sa marge rapportée à son seuil (0 : juste au
    seuil ; négative : refusé ; 1 : au double du seuil, pour une marge de
    distance), et ce qu'elle dit, en clair."""
    nom: str
    relative: float
    texte: str


@dataclass
class Candidate:
    """Une prise candidate : la paire, la pose du bassin (repère de la
    pièce), les marges de ses contrôles (dans l'ordre où ils ont été faits ;
    le premier refus arrête), si les chemins ont été vérifiés."""
    paire: Paire
    x: float
    y: float
    cap: float
    d: float
    s: float
    marges: List[Marge] = field(default_factory=list)
    complete: bool = False
    # Posture de la marche qu'il y faut : (tangage du torse en rad,
    # abaissement du bassin en m) ; (0, 0) debout.
    posture: Tuple[float, float] = (0.0, 0.0)

    @property
    def refus(self) -> Optional[Marge]:
        return next((m for m in self.marges if m.relative < 0.0), None)

    @property
    def ok(self) -> bool:
        return bool(self.marges) and self.refus is None

    @property
    def score(self) -> float:
        return min((m.relative for m in self.marges), default=-math.inf)

    def explication(self, n: int = 2) -> str:
        """La pose, puis les `n` marges les plus faibles (ou le refus)."""
        tete = (f"paire {self.paire.numero} ({100 * self.paire.largeur:.0f} cm entre les mains), "
                f"bassin à ({self.x:.2f}, {self.y:.2f}), cap {math.degrees(self.cap):+.0f}°")
        if self.refus is not None:
            return f"{tete} : {self.refus.texte}"
        faibles = sorted(self.marges, key=lambda m: m.relative)[:n]
        return tete + " ; marges les plus faibles : " + " ; ".join(
            f"{m.texte} ({100 * m.relative:.0f} %)" for m in faibles)


@dataclass
class PlanPrise:
    """Ce que le planificateur a vu et choisi : toutes les candidates, la
    retenue (None : aucune ne passe), et la raison d'un refus."""
    boite: Boite
    bord: BordTable
    candidates: List[Candidate]
    choisie: Optional[Candidate]
    raison: str = ""

    @property
    def meilleure(self) -> Optional[Candidate]:
        """La choisie, ou à défaut celle qui va le plus loin dans les
        contrôles (le premier refus le plus tardif), puis au meilleur score."""
        if self.choisie is not None:
            return self.choisie
        return max(self.candidates, key=lambda c: (len(c.marges), c.score), default=None)

    def refus_par_controle(self) -> Dict[str, int]:
        """Combien de candidates chaque contrôle a refusées (le premier refus)."""
        compte: Dict[str, int] = {}
        for c in self.candidates:
            if c.refus is not None:
                compte[c.refus.nom] = compte.get(c.refus.nom, 0) + 1
        return compte

    def resume(self) -> str:
        n_ok = sum(c.ok for c in self.candidates)
        refus = ", ".join(f"{nom} {n}" for nom, n in sorted(self.refus_par_controle().items(),
                                                           key=lambda kv: -kv[1]))
        tete = (f"{len(self.candidates)} candidates, {n_ok} passent"
                + (f" (refusées : {refus})" if refus else ""))
        if self.choisie is not None:
            return f"prise planifiée : {tete} ; retenue : {self.choisie.explication()}"
        return f"prise refusée — {self.raison} ({tete})"


def choisir(candidates: List[Candidate]) -> Optional[Candidate]:
    """La candidate qui passe tous ses contrôles, chemins compris, au
    meilleur score ; None s'il n'y en a pas."""
    return max((c for c in candidates if c.ok and c.complete), key=lambda c: c.score,
               default=None)


def dessiner(plan: PlanPrise, chemin, largeur_px: int = 900, hauteur_px: int = 700,
             echelle: Optional[float] = None) -> None:
    """Vue plongeante du plan, en PNG : la table (au-delà du bord), la boîte,
    chaque candidate (bassin) en vert si elle passe, en rouge sinon ; la
    retenue cerclée, avec son cap ; en bas, la retenue expliquée, ou la
    raison du refus."""
    from PIL import Image, ImageDraw
    from axis_os.hal.drivers.isaac_sim.video import _police   # une police avec accents
    b, bord = plan.boite, plan.bord
    xs = [c.x for c in plan.candidates] + [b.centre[0]]
    ys = [c.y for c in plan.candidates] + [b.centre[1]]
    cx0, cy0 = (min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0
    ech = echelle or 0.8 * min(largeur_px, hauteur_px - 80) / max(
        0.5, max(xs) - min(xs) + 0.4, max(ys) - min(ys) + 0.4)

    def px(x, y):          # x de la pièce vers le haut de l'image, y vers la gauche
        return (largeur_px / 2.0 - (y - cy0) * ech, (hauteur_px - 80) / 2.0 - (x - cx0) * ech)

    img = Image.new("RGB", (largeur_px, hauteur_px), (250, 250, 248))
    d = ImageDraw.Draw(img)
    n = (math.cos(bord.normale), math.sin(bord.normale))
    t = (-n[1], n[0])
    loin = 10.0
    p0 = (bord.point[0] + t[0] * loin, bord.point[1] + t[1] * loin)
    p1 = (bord.point[0] - t[0] * loin, bord.point[1] - t[1] * loin)
    table = [p0, p1, (p1[0] + n[0] * loin, p1[1] + n[1] * loin),
             (p0[0] + n[0] * loin, p0[1] + n[1] * loin)]
    d.polygon([px(*p) for p in table], fill=(226, 222, 212))
    d.line([px(*p0), px(*p1)], fill=(120, 110, 95), width=3)
    coins = coins_boite(b.centre, b.lacet, b.a, b.b, b.hauteur)
    bas = sorted({(round(float(c[0]), 6), round(float(c[1]), 6)) for c in coins},
                 key=lambda q: math.atan2(q[1] - b.centre[1], q[0] - b.centre[0]))
    d.polygon([px(*q) for q in bas], fill=(196, 150, 96), outline=(120, 85, 45))
    for c in sorted(plan.candidates, key=lambda c: c.ok):
        x, y = px(c.x, c.y)
        couleur = (40, 150, 70) if c.ok else (205, 60, 50)
        d.ellipse([x - 3, y - 3, x + 3, y + 3], fill=couleur)
    if plan.choisie is not None:
        c = plan.choisie
        x, y = px(c.x, c.y)
        d.ellipse([x - 11, y - 11, x + 11, y + 11], outline=(20, 70, 160), width=4)
        x2, y2 = px(c.x + 0.12 * math.cos(c.cap), c.y + 0.12 * math.sin(c.cap))
        d.line([(x, y), (x2, y2)], fill=(20, 70, 160), width=4)
    texte = plan.resume()
    lignes, ligne = [], ""
    for mot in texte.split():
        if len(ligne) + len(mot) > largeur_px // 7:
            lignes.append(ligne)
            ligne = ""
        ligne = (ligne + " " + mot).strip()
    lignes.append(ligne)
    police = _police(13)
    for i, l_ in enumerate(lignes[:5]):
        d.text((10, hauteur_px - 78 + 15 * i), l_, fill=(30, 30, 30), font=police)
    img.save(str(chemin))
