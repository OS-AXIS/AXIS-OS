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
    """Réglages de la prise mesurée (m, rad, N, N·m, s)."""
    # Demi-épaisseur d'une main : du repère de la main (rubber_hand de
    # l'URDF) à sa surface de contact. Mesurée sur le banc GPU du
    # 2026-10-28 : mains à 0,331 m l'une de l'autre sur un carton de
    # 0,24 m qu'elles touchaient.
    demi_main_m: float = 0.045
    # Chaque main, à l'écart de sa face perçue avant de fermer. L'ouverture
    # dépasse l'erreur de position acceptée (`recalage_max_m`) d'au moins
    # `marge_ouverture_m` : sinon la main pourrait reposer déjà sur un carton
    # que la perception place à `recalage_max_m` de là, le zéro de la force
    # serait pris en appui et le contact vu trop tard (constaté en simulation
    # à 3 cm avec 4 cm d'ouverture : la main 2,5 cm dans la face). Vérifié à
    # la construction des réglages (__post_init__).
    jeu_ouverture_m: float = 0.05
    marge_ouverture_m: float = 0.02
    tol_profondeur_m: float = 0.025    # mains au milieu des faces, en profondeur
    # Point de prise : les mains au moins `recul_face_min_m` derrière la
    # face avant de l'objet (celle tournée vers le robot), idéalement au
    # milieu. Constaté sur GPU (essai T1, novembre 2026) : mains près du
    # bord avant d'un carton. Avec un carton de 20 cm, la tolérance
    # ci-dessus les garde déjà à 7.5 cm ; la règle borne la tolérance vers
    # l'avant d'un objet moins profond (voir avance_max).
    recul_face_min_m: float = 0.07
    # et en hauteur ; constaté sur GPU (2026-10-29) : mains 3,3 cm plus bas
    # que leurs cibles (bras qui s'affaissent), d'où une tolérance serrée et
    # la compensation ci-dessous.
    tol_hauteur_m: float = 0.015
    compensation_max_m: float = 0.10   # visée décalée d'au plus 10 cm par main pour compenser
    corrections_max: int = 2
    rayon_avant_bras_m: float = 0.035
    marge_plateau_m: float = 0.03      # sous l'avant-bras, au-dessus du plateau
    bord_min_m: float = 0.15           # le bord d'une table n'est jamais plus près du bassin
    duree_placement_s: float = 0.8
    vitesse_fermeture: float = 0.5     # rad/s, roulis d'épaule
    vitesse_contact: float = 0.2       # rad/s, face commandée à moins de `zone_contact_m`
    zone_contact_m: float = 0.015      # de la face attendue : petit pas, petit dépassement
    pas_contact_max_rad: float = 0.004  # et jamais plus par pas (un pas lent ne l'agrandit pas)
    hausse_contact_nm: float = 1.5     # hausse du couple qui signe le contact
    retard_contact_rad: float = 0.06   # ou le roulis ne suit plus sa consigne
    # ou, en géométrie (constaté sur GPU le 2026-10-29 : le roulis suit sa
    # consigne, ce sont le poignet et le coude qui cèdent), la face
    # intérieure MESURÉE de la main en retard d'au moins `retard_face_m` sur
    # sa face COMMANDÉE, le long de la normale de l'objet, et à moins de
    # `tol_face_contact_m` de la face attendue de l'objet.
    retard_face_m: float = 0.01
    tol_face_contact_m: float = 0.015
    # Borne de la consigne de fermeture : la face COMMANDÉE de chaque main
    # ne va jamais au-delà de largeur/2 - penetration_max_m du centre.
    penetration_max_m: float = 0.02
    # Vide : les faces intérieures des mains (cinématique sur TOUS les
    # angles mesurés) plus proches que la largeur de l'objet moins vide_m.
    # Jamais d'après le seul angle du roulis : bras affaissés, ce même
    # angle ne ferme pas autant (constaté sur GPU, 2026-10-29).
    vide_m: float = 0.02
    ecart_largeur_max_m: float = 0.05  # contact bien avant la largeur attendue : autre chose
    # Force estimée à la main (N) : pseudo-inverse amortie de la transposée
    # de la jacobienne (`amorti_force_m`), appliquée à la hausse des couples
    # de TOUTES les articulations du bras depuis la fermeture, poids des bras
    # retiré (URDF), composante le long de la normale de l'objet, moyenne
    # glissante sur `filtre_force` lectures. Bruit : les couples estimés du
    # G1 (bruit_couples.py, 0,3 N·m par lecture) donnent ~0,85 N par lecture
    # à la main, ~0,4 N filtrés ; le bruit réel est mesuré au repos avant
    # chaque fermeture (`duree_bruit_s`), et le seuil de contact n'est jamais
    # sous `sigmas_contact` fois ce bruit filtré.
    amorti_force_m: float = 0.02
    filtre_force: int = 5
    duree_bruit_s: float = 0.2
    # Le zéro des couples (τ0) n'est pris qu'à `duree_raidissement_s` au
    # moins du dernier changement de gains. Pourquoi (analyse de l'essai GPU
    # T1 de novembre 2026, force estimée de mauvais signe pendant toute la
    # fermeture, contacts réels manqués) : les gains étaient raidis au début
    # même de la fenêtre du zéro ; le bras, moins affaissé sous son poids,
    # bougeait pendant cette fenêtre, et le couple de ce mouvement, pris dans
    # τ0, décalait la force relative jusqu'au bout. Les gains sont désormais
    # raidis dès le placement des mains.
    duree_raidissement_s: float = 0.5
    force_contact_n: float = 3.0
    sigmas_contact: float = 6.0
    # Serrage visé et minimal, à chaque main. Bras du G1 aux gains du banc
    # T1 (raidis) : ~500 N/m à la main, soit ~10 N au plus sur un objet
    # raide pour les 2 cm de `penetration_max_m` ; un carton de 1 kg entre
    # deux mains caoutchouc (frottement ~1) demande 4,9 N à chacune.
    force_serrage_n: float = 8.0
    force_min_n: float = 5.0
    force_deja_n: float = 16.0         # à la prise, avant de bouger : bras déjà en effort
    pas_serrage_max_m: float = 0.0005  # avance de la face commandée par pas, au plus
    pas_serrage_min_m: float = 0.0001
    duree_serrage_max_s: float = 6.0   # au-delà, on juge la force obtenue
    # Recalage au toucher : les faces mesurées des deux mains au contact
    # disent où est vraiment le carton, le long de sa normale ; au-delà de
    # `recalage_max_m` de la perception (l'erreur de position acceptée), la
    # prise est refusée. Jamais plus que jeu_ouverture_m - marge_ouverture_m.
    recalage_max_m: float = 0.03
    # Filet avant de fermer : la force ABSOLUE à chaque main (poids du bras
    # retiré par le modèle, pas le zéro de la fermeture), au-delà de
    # max(force_appui_n, sigmas_appui × bruit mesuré au repos), veut dire une
    # main déjà en appui sur l'objet : prise refusée. Seuil provisoire, à
    # régler sur les essais GPU (valeur mesurée au journal).
    force_appui_n: float = 3.0
    sigmas_appui: float = 4.0
    # … et en géométrie : la main MESURÉE (cinématique sur les angles
    # mesurés) retenue en dehors de sa main COMMANDÉE (cinématique sur la
    # consigne), le long de la normale de l'objet, de `retard_appui_m` de
    # plus qu'au début du placement (mains grandes ouvertes, libres) : la
    # main est retenue par quelque chose à la pose d'ouverture.
    # L'affaissement des bras sous leur poids, et sa compensation, sont les
    # mêmes au départ et à l'arrivée : ils s'annulent.
    retard_appui_m: float = 0.01
    # Plafond de couple de CHAQUE articulation du bras (valeur absolue).
    effort_max_nm: float = 10.0        # jamais au-delà
    part_couple_garde: float = 0.5     # ni au-delà de cette part du couple max du garde-fou
    duree_maintien_s: float = 0.3
    age_max_effort_s: float = 0.10
    hauteur_test_m: float = 0.04       # levage de vérification
    duree_levage_test_s: float = 0.8
    essais_max: int = 2
    distance_torse_m: float = 0.10     # face arrière de l'objet porté, devant le bassin
    hausse_transport_m: float = 0.05
    duree_ramener_s: float = 1.2
    inclinaison_max_deg: float = 15.0
    # Écart admis entre une main mesurée par le robot (le lien de la main,
    # rubber_hand) et la même main calculée par la cinématique sur les
    # angles mesurés. Au-delà, le modèle des bras ne décrit pas le robot :
    # pas de prise mesurée (constaté sur GPU le 2026-10-28 : mains contre
    # la face avant du carton, ~10 cm du milieu des faces).
    ecart_modele_max_m: float = 0.015

    def __post_init__(self):
        # Refus au démarrage : une ouverture qui ne couvre pas l'erreur de
        # position acceptée, marge comprise, ferait fermer une main déjà en
        # appui sur l'objet.
        if self.recalage_max_m > self.jeu_ouverture_m - self.marge_ouverture_m + 1e-12:
            raise ValueError(
                f"réglages de prise refusés : erreur de position acceptée (recalage_max_m = "
                f"{self.recalage_max_m:.3f} m) au-delà de l'ouverture moins sa marge "
                f"(jeu_ouverture_m - marge_ouverture_m = "
                f"{self.jeu_ouverture_m - self.marge_ouverture_m:.3f} m)")

    def avance_max(self, profondeur: float) -> float:
        """Écart maximal (m) des mains vers la face avant, depuis le milieu
        de l'objet : `recul_face_min_m` derrière la face avant au moins, et
        jamais plus que `tol_profondeur_m`. Un objet de moins de deux fois
        `recul_face_min_m` de profondeur est pris au milieu, à
        `tol_profondeur_m` près (la règle ne peut pas y tenir)."""
        reste = profondeur / 2.0 - self.recul_face_min_m
        return self.tol_profondeur_m if reste <= 0.0 else min(self.tol_profondeur_m, reste)


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
    vers_bassin : point du repère monde -> repère du bassin, rotation
                  complète (tangage et roulis du bassin compris), celui de
                  la cinématique. None : le cap seul, et la hauteur au-dessus
                  du bassin (bassin supposé droit).
    raideur     : (bool) -> None ; True raidit les poignets (et le coude)
                  par leurs gains kp/kd, dans les limites du pilote, False
                  rend les gains normaux. Appelé à la fermeture, puis au
                  dépôt, à l'échec et sur arrêt d'urgence. None : pas de
                  réglage de gains.
    arret_urgence : () -> arrêt d'urgence en cours ? (gains rendus aussitôt)
    commandables : articulations des bras que la prise peut commander (en
                  plus du tangage, du roulis et du coude : lacet d'épaule,
                  poignets) pour la fermeture et le serrage en cartésien.
                  None : tangage, roulis et coude seulement.
    """
    modele: ModeleBras
    positions: Callable[[], Dict[str, float]]
    efforts: Callable[[], Dict[str, Tuple[float, float]]]
    bassin_z: Callable[[], float]
    couple_max: Dict[str, float] = field(default_factory=dict)
    inclinaison: Optional[Callable[[], Optional[float]]] = None
    vers_bassin: Optional[Callable[[np.ndarray], np.ndarray]] = None
    raideur: Optional[Callable[[bool], None]] = None
    arret_urgence: Optional[Callable[[], bool]] = None
    commandables: Optional[Tuple[str, ...]] = None


def efforts_valides(mesures: MesuresBras, joints: Iterable[str], age_max: float,
                    optionnels: Iterable[str] = ()) -> Tuple[Optional[Dict[str, float]], str]:
    """Couples des articulations `joints`, ou (None, raison) si une mesure
    manque, n'est pas finie ou est périmée : jamais « tout va bien ». Les
    `optionnels` sont ajoutés quand le robot les mesure (absents : ignorés,
    illisibles : refus)."""
    try:
        brut = mesures.efforts()
    except Exception as exc:              # noqa: BLE001 — capteur en panne
        return None, f"lecture de l'effort impossible ({type(exc).__name__})"
    requis = list(joints)
    sortie = {}
    for j in requis + [o for o in optionnels if o not in requis]:
        v = brut.get(j) if isinstance(brut, dict) else None
        if v is None:
            if j not in requis:
                continue
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
        g = garde_plateau(modele, {**depart, **q}, plateau, bord, r.rayon_avant_bras_m)
        meilleure_garde = max(meilleure_garde, g)
        if g >= r.marge_plateau_m:
            return float(d), ""
    if meilleure_garde == -math.inf:
        return None, "objet hors de portée des bras à toute distance"
    return None, (f"avant-bras trop près du plateau à toute distance (au mieux "
                  f"{100 * meilleure_garde:.1f} cm, il faut {100 * r.marge_plateau_m:.0f})")


def ecarts_modele(modele: ModeleBras, angles: Dict[str, float],
                  mains_reelles: Tuple[np.ndarray, np.ndarray]
                  ) -> Tuple[np.ndarray, np.ndarray]:
    """Écarts (m, repère du bassin) entre les mains mesurées et les mains
    calculées par la cinématique sur les angles mesurés : (gauche, droite),
    réel moins modèle."""
    pg, pd = modele.mains(angles)
    return (np.asarray(mains_reelles[0], dtype=float) - pg,
            np.asarray(mains_reelles[1], dtype=float) - pd)


class CalibrationCouples:
    """Biais des couples estimés, articulation par articulation. Bras en
    l'air et immobiles, dans quelques postures (celles de la calibration des
    mains), le couple estimé devrait être celui de la pesanteur sur le bras
    (modèle de l'URDF, angles mesurés, centrale inertielle) ; l'écart moyen
    est le biais (moindres carrés d'un décalage constant sur tous les
    relevés), et ce qui reste, posture par posture, le résidu (le modèle
    n'explique pas tout). Constaté sur GPU (essai T1, novembre 2026) : 4.6 N
    puis 3.9 N à la main droite au repos, sans contact, pour un bruit de 0.6
    à 1.05 N : un biais, pas du bruit. La force à la main se calcule ensuite
    sur les couples corrigés (ManipulationService.biais_couples) ; les
    plafonds de couple, eux, restent jugés sur le couple mesuré."""

    def __init__(self):
        self.releves: list = []          # (posture, {articulation: (couple, gravité)})

    def ajouter(self, posture: str, couples: Dict[str, float],
                gravite: Dict[str, float]) -> None:
        self.releves.append((posture, {j: (float(couples[j]), float(gravite[j]))
                                       for j in gravite if j in couples}))

    def resoudre(self) -> Dict[str, Tuple[float, float, int]]:
        """{articulation: (biais N·m, résidu N·m, relevés)} ; le résidu est
        l'écart quadratique moyen, posture par posture (moyennes des relevés
        de chaque posture), de ce que le biais n'explique pas."""
        ecarts: Dict[str, Dict[str, list]] = {}
        for posture, rel in self.releves:
            for j, (t, g) in rel.items():
                if math.isfinite(t) and math.isfinite(g):
                    ecarts.setdefault(j, {}).setdefault(posture, []).append(t - g)
        sortie = {}
        for j, par_posture in ecarts.items():
            tous = [e for es in par_posture.values() for e in es]
            b = float(np.mean(tous))
            moyennes = [float(np.mean(es)) - b for es in par_posture.values()]
            sortie[j] = (b, float(np.sqrt(np.mean(np.square(moyennes)))), len(tous))
        return sortie

    @staticmethod
    def bilan(biais: Dict[str, Tuple[float, float, int]], postures: int) -> str:
        """Texte lisible : une ligne d'en-tête, puis une par articulation."""
        if not biais:
            return "calibration des couples : aucun relevé — force à la main sans correction"
        pire = max(biais.items(), key=lambda kv: kv[1][1])
        lignes = [f"calibration des couples ({postures} postures) : biais par articulation "
                  f"(couple estimé - pesanteur du modèle), résidu max {pire[1][1]:.2f} N·m "
                  f"({pire[0]})"]
        for j, (b, res, n) in sorted(biais.items()):
            lignes.append(f"    {j:28s} biais {b:+.2f} N·m, résidu {res:.2f} N·m ({n} relevés)")
        return "\n".join(lignes)


class CalibrationMains:
    """Compare, sur quelques postures, les mains calculées par la
    cinématique (angles mesurés) aux mains mesurées par le robot. Le banc
    T1 la fait au démarrage ; au-delà de `ecart_max_m`, la prise mesurée
    est refusée (ManipulationService.calibration_refus)."""

    def __init__(self, modele: ModeleBras, ecart_max_m: float = 0.015):
        self.modele, self.ecart_max_m = modele, ecart_max_m
        self.releves: list = []          # (posture, écart gauche, écart droit)

    def ajouter(self, posture: str, angles: Dict[str, float],
                mains_reelles: Tuple[np.ndarray, np.ndarray]) -> None:
        eg, ed = ecarts_modele(self.modele, angles, mains_reelles)
        self.releves.append((posture, eg, ed))

    @property
    def ecart_max(self) -> float:
        """Plus grand écart relevé (m) ; +inf sans relevé, ou s'il n'est pas fini."""
        if not self.releves:
            return math.inf
        v = max(float(np.linalg.norm(e)) for _, eg, ed in self.releves for e in (eg, ed))
        return v if math.isfinite(v) else math.inf

    def bilan(self) -> Tuple[bool, str]:
        """(acceptée, texte lisible, une ligne par posture et par main)."""
        lignes = []
        for nom, eg, ed in self.releves:
            for cote, e in (("gauche", eg), ("droite", ed)):
                lignes.append(f"    {nom:10s} main {cote:6s} : réel - modèle = "
                              f"({100 * e[0]:+.1f}, {100 * e[1]:+.1f}, {100 * e[2]:+.1f}) cm, "
                              f"{100 * float(np.linalg.norm(e)):.1f} cm")
        ok = self.ecart_max <= self.ecart_max_m
        tete = (f"calibration des mains : écart max {100 * self.ecart_max:.1f} cm "
                f"(limite {100 * self.ecart_max_m:.1f}) — "
                + ("prise mesurée autorisée" if ok else
                   "MODÈLE DES BRAS FAUX, prise mesurée REFUSÉE"))
        return ok, "\n".join([tete] + lignes)
