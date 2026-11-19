"""
Banc Isaac Sim de la posture (lancer_posture.sh, test_isaac_w5_gr00t_posture.py) :
paliers, premiers signes de perte d'équilibre, relevé et verdict. Rien ici
n'est lu par l'OS (axis_os/) ; test/test_banc_posture.py le vérifie, sans GPU.

Ce que le banc valide, avant que l'app de tri ne se penche (décision de
revue) : la consigne de posture de la marche GR00T WBC, torse penché et
bassin abaissé (LocomotionService.posture, ses bornes et son modèle
d'équilibre), robot immobile, sans puis avec un carton de masse_max_kg aux
mains. Par palier, le relevé met côte à côte :
  - ce que l'OS prévoit : accepté ou refusé, marge d'équilibre prévue
    (equilibre.py : modèle statique, sans recul des hanches) ;
  - ce que fait le robot : inclinaison et vitesse angulaire du bassin
    (centrale inertielle, signaux du vrai G1), tangage mesuré de la taille
    (angle articulaire), glissement des pieds (vérité de la simulation,
    pour le banc seul : le vrai G1 ne le mesure pas).
Au premier signe de perte d'équilibre, le banc arrête la série et ramène
le robot droit (retour toujours permis par l'OS).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

# Paliers, du plus petit au plus grand, dans les bornes de la consigne
# (BornesPosture : torse jusqu'à 0,35 rad, bassin jusqu'à 10 cm).
TANGAGES = (0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35)
ABAISSEMENTS = (0.025, 0.05, 0.075, 0.10)
MAINTIEN_S = 2.0                # chaque palier, une fois atteint, est tenu 2 s

# Premiers signes de perte d'équilibre : un seul suffit. Valeurs de départ,
# à revoir sur le relevé du premier essai GPU (il dit ce qui est normal).
INCLINAISON_MAX_DEG = 10.0      # bassin incliné de plus, depuis le début de la série
VITESSE_MAX_RAD_S = 0.6         # vitesse angulaire du bassin
GLISSEMENT_MAX_M = 0.01         # un pied a glissé de plus, depuis le début de la série

COLONNES = ("serie", "charge_kg", "palier", "tangage_rad", "abaissement_m", "accepte",
            "marge_prevue_cm", "tangage_mesure_rad", "inclinaison_max_deg",
            "vitesse_max_rad_s", "glissement_max_cm", "carton_tenu", "signe", "raison")


def paliers() -> List[Tuple[str, List[Tuple[float, float]]]]:
    """Trois séries de (tangage du torse en rad, abaissement du bassin en
    m) : le torse seul, le bassin seul, puis les deux ensemble (bassin
    abaissé au prorata du tangage)."""
    n = len(TANGAGES)
    return [("torse", [(t, 0.0) for t in TANGAGES]),
            ("bassin", [(0.0, a) for a in ABAISSEMENTS]),
            ("les deux", [(t, round(ABAISSEMENTS[-1] * (k + 1) / n, 4))
                          for k, t in enumerate(TANGAGES)])]


def inclinaison_deg(quat_wxyz: Sequence[float]) -> float:
    """Angle (°) entre l'axe vertical du bassin et la verticale, d'après
    l'orientation que donne la centrale inertielle (quaternion w, x, y, z)."""
    w, x, y, z = (float(v) for v in quat_wxyz)
    n = w * w + x * x + y * y + z * z
    c = 1.0 - 2.0 * (x * x + y * y) / n          # cos de l'inclinaison
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def signe_de_perte(inclinaison: float, vitesse: float,
                   glissement: Optional[float]) -> Optional[str]:
    """Le premier signe de perte d'équilibre, en clair, ou None.
    `inclinaison` : du bassin, depuis le début de la série (°) ; `vitesse` :
    norme de sa vitesse angulaire (rad/s) ; `glissement` : du pied qui a le
    plus glissé depuis le début de la série (m), None si inconnu."""
    if not all(math.isfinite(v) for v in (inclinaison, vitesse)):
        return "centrale inertielle illisible (valeur non finie)"
    if abs(inclinaison) > INCLINAISON_MAX_DEG:
        return f"bassin incliné de {inclinaison:+.1f}° (au plus {INCLINAISON_MAX_DEG:g})"
    if vitesse > VITESSE_MAX_RAD_S:
        return f"bassin qui tourne à {vitesse:.2f} rad/s (au plus {VITESSE_MAX_RAD_S:g})"
    if glissement is not None and glissement > GLISSEMENT_MAX_M:
        return f"pied glissé de {100 * glissement:.1f} cm (au plus {100 * GLISSEMENT_MAX_M:g})"
    return None


@dataclass
class Palier:
    """Un palier : ce que l'OS en a dit, ce que le robot a fait."""
    serie: str
    charge_kg: float
    k: int
    tangage: float
    abaissement: float
    accepte: bool = False
    marge_prevue_m: Optional[float] = None
    raison: str = ""
    tangage_mesure: Optional[float] = None
    inclinaison_max_deg: float = 0.0
    vitesse_max: float = 0.0
    glissement_max_m: Optional[float] = None
    carton_tenu: Optional[bool] = None
    signe: Optional[str] = None

    def suivre(self, inclinaison: float, vitesse: float, glissement: Optional[float]
               ) -> Optional[str]:
        """Un pas de mesure : retient les plus grandes valeurs ; rend le
        signe de perte d'équilibre s'il y en a un (et le retient)."""
        if math.isfinite(inclinaison) and abs(inclinaison) > abs(self.inclinaison_max_deg):
            self.inclinaison_max_deg = inclinaison
        if math.isfinite(vitesse):
            self.vitesse_max = max(self.vitesse_max, vitesse)
        if glissement is not None:
            self.glissement_max_m = max(self.glissement_max_m or 0.0, glissement)
        s = signe_de_perte(inclinaison, vitesse, glissement)
        if s and not self.signe:
            self.signe = s
        return s

    def ligne(self) -> Dict[str, str]:
        """La ligne du relevé (COLONNES)."""
        def f(v, fmt):
            return "" if v is None else format(v, fmt)
        return {"serie": self.serie, "charge_kg": f"{self.charge_kg:g}", "palier": str(self.k),
                "tangage_rad": f"{self.tangage:.3f}", "abaissement_m": f"{self.abaissement:.3f}",
                "accepte": "oui" if self.accepte else "non",
                "marge_prevue_cm": f(None if self.marge_prevue_m is None
                                     else 100 * self.marge_prevue_m, ".2f"),
                "tangage_mesure_rad": f(self.tangage_mesure, ".3f"),
                "inclinaison_max_deg": f"{self.inclinaison_max_deg:.2f}",
                "vitesse_max_rad_s": f"{self.vitesse_max:.3f}",
                "glissement_max_cm": f(None if self.glissement_max_m is None
                                       else 100 * self.glissement_max_m, ".2f"),
                "carton_tenu": "" if self.carton_tenu is None else
                               ("oui" if self.carton_tenu else "non"),
                "signe": self.signe or "", "raison": self.raison}


def verdict(releve: Sequence[Palier], chute: Optional[str], refus_garde_fou: int,
            droit_a_la_fin: bool) -> Tuple[bool, List[str]]:
    """Le banc est-il passé ? Aucun signe de perte d'équilibre sur un palier
    que l'OS a accepté, ni chute, ni consigne refusée par le JointSafetyGuard,
    le carton toujours tenu, le robot droit à la fin, et chaque série
    essayée (au moins un palier accepté, sans et avec carton). Les paliers
    que l'OS refuse (bornes, équilibre prévu) ne sont pas des échecs : le
    modèle est prudent ; le relevé les montre. Rend (passé, raisons)."""
    raisons = []
    for p in releve:
        if p.accepte and p.signe:
            raisons.append(f"{p.serie}, {p.charge_kg:g} kg, palier {p.k} ({p.tangage:.2f} rad, "
                           f"{100 * p.abaissement:.1f} cm) : {p.signe}")
        if p.carton_tenu is False:
            raisons.append(f"{p.serie}, palier {p.k} : carton lâché")
    if chute:
        raisons.append(f"chute : {chute}")
    if refus_garde_fou:
        raisons.append(f"{refus_garde_fou} consigne(s) refusée(s) par le garde-fou")
    if not droit_a_la_fin:
        raisons.append("le robot n'est pas revenu droit à la fin")
    for serie, _ in paliers():
        for charge in sorted({p.charge_kg for p in releve}) or [0.0]:
            if not any(p.accepte for p in releve if p.serie == serie and p.charge_kg == charge):
                raisons.append(f"série « {serie} », {charge:g} kg : aucun palier accepté "
                               f"par l'OS")
    if len({p.charge_kg for p in releve}) < 2:
        raisons.append("il manque la série sans carton ou celle avec carton")
    return not raisons, raisons
