"""
AXIS-OS — Prise mesurée : géométrie et mesures de la prise à deux bras.

Ce que le service de manipulation (service.py) en tire, quand le robot lui
donne ses mesures (MesuresBras) :

  1. Portée : les mains peuvent-elles atteindre le milieu des deux faces
     opposées de l'objet (cinématique inverse, bornes de l'URDF) ? Sinon,
     refus OUT_OF_REACH, avec la distance à corriger. Jamais de serrage
     dans le vide.
  2. Distance à la table : les avant-bras restent au-dessus du plateau
     (`marge_plateau_m` plus le rayon de l'avant-bras), au-delà du bord de
     la table ; distance_ideale() la choisit pour l'approche.
  3. Mains au milieu des faces, en profondeur et en hauteur, d'après la
     position, la taille et le lacet de l'objet ; tolérances
     `tol_profondeur_m` et `tol_hauteur_m`, corrigées avant de serrer.
  4. Serrage mesuré : les bras se ferment jusqu'au CONTACT, détecté par
     l'effort (hausse du couple du roulis d'épaule depuis le début de la
     fermeture) ou par l'arrêt du rapprochement (la main ne suit plus sa
     consigne) ; puis un serrage supplémentaire, monté jusqu'à
     `effort_serrage_nm`, jamais au-delà de `effort_max_nm` ni d'une part
     `part_couple_garde` du couple maximal du garde-fou. Effort absent,
     non fini ou plus vieux que `age_max_effort_s` : on ne serre pas.

Repère : celui du bassin du robot (x devant, y à gauche, z en haut).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Dict, Iterable, Optional, Tuple

import numpy as np

from .cinematique import ModeleBras


@dataclass(frozen=True)
class ReglagesPrise:
    """Réglages de la prise mesurée (m, rad, N·m, s)."""
    # Demi-épaisseur d'une main : du repère de la main (rubber_hand de
    # l'URDF) à sa surface de contact. Mesurée sur le banc GPU du
    # 2026-10-28 : mains à 0,331 m l'une de l'autre sur un carton de
    # 0,24 m qu'elles touchaient.
    demi_main_m: float = 0.045
    jeu_ouverture_m: float = 0.04      # chaque main, à l'écart de sa face avant de fermer
    tol_profondeur_m: float = 0.025    # mains au milieu des faces, en profondeur
    tol_hauteur_m: float = 0.03        # et en hauteur
    corrections_max: int = 2
    rayon_avant_bras_m: float = 0.035
    marge_plateau_m: float = 0.03      # sous l'avant-bras, au-dessus du plateau
    bord_min_m: float = 0.15           # le bord d'une table n'est jamais plus près du bassin
    duree_placement_s: float = 0.8
    vitesse_fermeture: float = 0.5     # rad/s, roulis d'épaule
    vitesse_contact: float = 0.2       # rad/s, à moins de `zone_contact_rad` du contact
    zone_contact_rad: float = 0.05     # attendu (~1,5 cm) : petit pas, petit dépassement
    hausse_contact_nm: float = 1.5     # hausse du couple qui signe le contact
    retard_contact_rad: float = 0.06   # la main ne suit plus sa consigne
    penetration_max_m: float = 0.02    # au-delà, plus rien entre les mains : vide
    ecart_largeur_max_m: float = 0.05  # contact bien avant la largeur attendue : autre chose
    effort_serrage_nm: float = 5.0     # hausse de couple visée, pour tenir
    effort_min_nm: float = 2.0         # en dessous, la prise ne tient pas
    effort_max_nm: float = 10.0        # jamais au-delà
    part_couple_garde: float = 0.5     # ni au-delà de cette part du couple max du garde-fou
    vitesse_serrage: float = 0.3       # rad/s
    serrage_max_rad: float = 0.20      # au-delà du contact, au plus
    duree_maintien_s: float = 0.3
    age_max_effort_s: float = 0.10
    hauteur_test_m: float = 0.04       # levage de vérification
    duree_levage_test_s: float = 0.8
    essais_max: int = 2
    distance_torse_m: float = 0.10     # face arrière de l'objet porté, devant le bassin
    hausse_transport_m: float = 0.05
    duree_ramener_s: float = 1.2
    inclinaison_max_deg: float = 15.0


@dataclass
class MesuresBras:
    """Ce que le robot mesure, et que la prise mesurée exige.

    modele      : la cinématique des bras (URDF).
    positions   : () -> angles bruts mesurés des articulations des bras.
    efforts     : () -> {articulation: (couple en N·m, âge de la mesure en s)}.
    bassin_z    : () -> hauteur du bassin, repère monde (m).
    couple_max  : couple maximal du garde-fou par articulation (N·m).
    inclinaison : () -> inclinaison de l'objet tenu (degrés), ou None si
                  elle n'est pas mesurée.
    """
    modele: ModeleBras
    positions: Callable[[], Dict[str, float]]
    efforts: Callable[[], Dict[str, Tuple[float, float]]]
    bassin_z: Callable[[], float]
    couple_max: Dict[str, float] = field(default_factory=dict)
    inclinaison: Optional[Callable[[], Optional[float]]] = None


def efforts_valides(mesures: MesuresBras, joints: Iterable[str], age_max: float
                    ) -> Tuple[Optional[Dict[str, float]], str]:
    """Couples des articulations `joints`, ou (None, raison) si une mesure
    manque, n'est pas finie ou est périmée : jamais « tout va bien »."""
    try:
        brut = mesures.efforts()
    except Exception as exc:              # noqa: BLE001 — capteur en panne
        return None, f"lecture de l'effort impossible ({type(exc).__name__})"
    sortie = {}
    for j in joints:
        v = brut.get(j) if isinstance(brut, dict) else None
        if v is None:
            return None, f"effort de {j} absent"
        try:
            couple, age = float(v[0]), float(v[1])
        except (TypeError, ValueError, IndexError):
            return None, f"effort de {j} illisible"
        if not (math.isfinite(couple) and math.isfinite(age)):
            return None, f"effort de {j} non fini"
        if age < 0.0 or age > age_max:
            return None, f"effort de {j} périmé ({age:.2f} s)"
        sortie[j] = couple
    return sortie, ""


def positions_valides(mesures: MesuresBras, joints: Iterable[str]
                      ) -> Tuple[Optional[Dict[str, float]], str]:
    try:
        brut = mesures.positions()
    except Exception as exc:              # noqa: BLE001
        return None, f"lecture des angles impossible ({type(exc).__name__})"
    sortie = {}
    for j in joints:
        v = brut.get(j) if isinstance(brut, dict) else None
        if v is None or not math.isfinite(float(v)):
            return None, f"angle de {j} absent ou non fini"
        sortie[j] = float(v)
    return sortie, ""


def faces(centre: np.ndarray, largeur: float, lacet: float
          ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Milieux des faces gauche et droite de l'objet (repère du bassin), et
    la normale de la face gauche ; `lacet` : rotation de l'objet par
    rapport au robot (rad)."""
    n = np.array([-math.sin(lacet), math.cos(lacet), 0.0])
    return centre + n * largeur / 2.0, centre - n * largeur / 2.0, n


def cibles_mains(centre: np.ndarray, largeur: float, lacet: float, ecart: float
                 ) -> Tuple[np.ndarray, np.ndarray]:
    """Repères des mains à `ecart` de chaque face, vers l'extérieur."""
    fg, fd, n = faces(centre, largeur, lacet)
    return fg + n * ecart, fd - n * ecart


def resoudre(modele: ModeleBras, cg: np.ndarray, cd: np.ndarray, depart: Dict[str, float],
             tol: float = 0.003) -> Tuple[Optional[Dict[str, float]], float]:
    """Angles des deux bras pour ces repères de mains (à `tol` m près), ou
    (None, erreur)."""
    qg, eg = modele.gauche.inverse(cg, depart, tol=tol)
    qd, ed = modele.droite.inverse(cd, depart, tol=tol)
    if qg is None or qd is None:
        return None, max(eg, ed)
    q = dict(qg)
    q.update(qd)
    if not (modele.gauche.dans_les_bornes(q) and modele.droite.dans_les_bornes(q)):
        return None, max(eg, ed)
    return q, max(eg, ed)


def garde_plateau(modele: ModeleBras, q: Dict[str, float], plateau_z: float, bord_x: float,
                  rayon: float) -> float:
    """Distance (m) du dessous de l'avant-bras au plateau, sur la partie
    des avant-bras au-dessus de la table (x ≥ bord_x) ; +inf si aucune."""
    garde = math.inf
    for chaine in (modele.gauche, modele.droite):
        for p in chaine.avant_bras(q):
            if p[0] >= bord_x:
                garde = min(garde, float(p[2]) - rayon - plateau_z)
    return garde


def distance_ideale(modele: ModeleBras, largeur: float, profondeur: float, hauteur: float,
                    centre_z: float, depart: Dict[str, float], *, bord_m: float = 0.0,
                    reglages: ReglagesPrise = ReglagesPrise(),
                    distances=np.arange(0.20, 0.45, 0.005)) -> Tuple[Optional[float], str]:
    """Distance (m) entre le bassin et le centre de l'objet, le long de
    l'axe du robot, pour que les mains atteignent le milieu de ses faces et
    que les avant-bras ne touchent pas le plateau. `centre_z` : hauteur du
    centre, repère du bassin ; `bord_m` : distance de la face avant de
    l'objet au bord de la table. La plus courte qui convient (le carton
    reste près du corps) ; (None, raison) s'il n'y en a aucune."""
    r = reglages
    plateau = centre_z - hauteur / 2.0
    meilleure_garde = -math.inf
    for d in distances:
        bord = d - profondeur / 2.0 - bord_m
        if bord < r.bord_min_m:
            continue
        c = np.array([float(d), 0.0, centre_z])
        q, _ = resoudre(modele, *cibles_mains(c, largeur, 0.0, r.demi_main_m), depart)
        if q is None:
            continue
        g = garde_plateau(modele, q, plateau, bord, r.rayon_avant_bras_m)
        meilleure_garde = max(meilleure_garde, g)
        if g >= r.marge_plateau_m:
            return float(d), ""
    if meilleure_garde == -math.inf:
        return None, "objet hors de portée des bras à toute distance"
    return None, (f"avant-bras trop près du plateau à toute distance (au mieux "
                  f"{100 * meilleure_garde:.1f} cm, il faut {100 * r.marge_plateau_m:.0f})")
