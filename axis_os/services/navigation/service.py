"""
AXIS-OS — Service de navigation.

Cinquième service système. Une app demande « va au poste B » ; le service
planifie un chemin qui contourne les obstacles, puis le suit en envoyant
des consignes de VITESSE au service de locomotion. L'app ne connaît ni la
carte en détail, ni la marche du robot.

TROIS PIÈCES, chacune remplaçable :

  Carte          — obstacles fixes et postes nommés, déclarés par le site.
  Planificateur  — A* sur une grille, obstacles ÉLARGIS du rayon du robot
                   et d'une marge : le chemin garde ses distances.
  Suiveur        — poursuite pure (« pure pursuit ») : vise un point un peu
                   plus loin sur le chemin, en déduit vitesse et rotation.

LA POSITION DU ROBOT vient d'une SOURCE injectée. En simulation, c'est la
pose lue dans la scène ; sur un vrai robot, ce sera un SLAM au lidar. Le
service ne sait pas laquelle — et c'est voulu : le jour où la localisation
par capteurs arrive, rien d'autre ne change.

TROIS RÈGLES DE SÉCURITÉ, testées :
  - pose PÉRIMÉE -> arrêt : naviguer sur une position ancienne, c'est
    naviguer à l'aveugle ;
  - pas de chemin -> ÉCHEC explicite avec sa raison, jamais un robot qui
    fonce vers un mur ;
  - hors du chemin de plus d'une limite -> on replanifie depuis la position
    réelle plutôt que de revenir en arrière en ligne droite.

LEÇON APPLIQUÉE (docs/validation_port_carton.md, W4) : sous 0.05 m/s, la
politique GR00T WBC passe en mode équilibre et n'avance plus. Le suiveur
n'émet donc jamais de vitesse dans la bande morte : soit une vitesse
franche, soit zéro.
"""
from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from axis_os.hal.core.types import HALStatus
from axis_os.services.base import IService, ServiceHealth, ServiceResult
from axis_os.store.sdk.manifest import Capability


# ─── Géométrie ────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    cap: float = 0.0                 # radians, 0 = axe x du monde
    horodatage: float = 0.0          # secondes, horloge du robot


@dataclass(frozen=True)
class Rectangle:
    """Obstacle fixe, aligné sur les axes : xmin, ymin, xmax, ymax."""
    xmin: float
    ymin: float
    xmax: float
    ymax: float
    # Hauteur de l'obstacle. None = toute hauteur (un mur, une étagère
    # jusqu'au plafond). Pour une TABLE, la hauteur du plateau : ce qui est
    # au-dessus n'est pas expliqué par la table — un humain penché dessus,
    # ou le carton posé, doivent rester visibles au gardien de proximité.
    zmax: Optional[float] = None

    def contient(self, x: float, y: float, marge: float = 0.0) -> bool:
        return (self.xmin - marge <= x <= self.xmax + marge
                and self.ymin - marge <= y <= self.ymax + marge)


def _angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ─── Carte ────────────────────────────────────────────────────────────────

@dataclass
class Carte:
    """Ce que le SITE sait de lui-même : ses murs, ses étagères, ses postes.

    Les postes sont des poses d'arrivée : position ET orientation, parce
    qu'arriver à une table de côté ne permet pas d'y prendre un carton.
    """
    xmin: float
    ymin: float
    xmax: float
    ymax: float
    obstacles: List[Rectangle] = field(default_factory=list)
    postes: Dict[str, Pose2D] = field(default_factory=dict)
    resolution: float = 0.05         # mètres par case


# ─── Planificateur ────────────────────────────────────────────────────────

class Planificateur:
    """A* sur grille, obstacles élargis.

    ÉLARGIR les obstacles du rayon du robot plus une marge revient à
    planifier pour un POINT : si le chemin du point évite les obstacles
    élargis, le corps du robot évite les vrais. Sans ça, un chemin peut
    raser un coin et la hanche du robot le heurter.
    """

    def __init__(self, carte: Carte, rayon_robot: float = 0.30,
                 marge: float = 0.10):
        self.carte = carte
        self.gonflement = rayon_robot + marge
        r = carte.resolution
        self.nx = int(math.ceil((carte.xmax - carte.xmin) / r))
        self.ny = int(math.ceil((carte.ymax - carte.ymin) / r))
        self.temporaires: List[Rectangle] = []
        self._libre = [[self._case_libre(i, j) for j in range(self.ny)]
                       for i in range(self.nx)]

    def _recalculer(self, o: "Rectangle") -> None:
        g = self.gonflement
        i0, j0 = self._case(o.xmin - g, o.ymin - g)
        i1, j1 = self._case(o.xmax + g, o.ymax + g)
        for i in range(max(0, i0), min(self.nx, i1 + 1)):
            for j in range(max(0, j0), min(self.ny, j1 + 1)):
                self._libre[i][j] = self._case_libre(i, j)

    def bloquer(self, o: "Rectangle") -> None:
        """Obstacle TEMPORAIRE, absent de la carte du site (une personne
        immobile, une palette laissée là)."""
        self.temporaires.append(o)
        self._recalculer(o)

    def debloquer(self, o: "Rectangle") -> None:
        if o in self.temporaires:
            self.temporaires.remove(o)
            self._recalculer(o)

    def _centre(self, i: int, j: int) -> Tuple[float, float]:
        r = self.carte.resolution
        return self.carte.xmin + (i + 0.5) * r, self.carte.ymin + (j + 0.5) * r

    def _case(self, x: float, y: float) -> Tuple[int, int]:
        r = self.carte.resolution
        return (int((x - self.carte.xmin) / r), int((y - self.carte.ymin) / r))

    def _case_libre(self, i: int, j: int) -> bool:
        x, y = self._centre(i, j)
        c = self.carte
        if not (c.xmin + self.gonflement <= x <= c.xmax - self.gonflement
                and c.ymin + self.gonflement <= y <= c.ymax - self.gonflement):
            return False                     # les bords de la carte sont des murs
        return not any(o.contient(x, y, self.gonflement)
                       for o in list(c.obstacles) + getattr(self, "temporaires", []))

    def libre(self, x: float, y: float) -> bool:
        i, j = self._case(x, y)
        return 0 <= i < self.nx and 0 <= j < self.ny and self._libre[i][j]

    def _segment_libre(self, a: Tuple[float, float], b: Tuple[float, float]) -> bool:
        d = math.dist(a, b)
        n = max(2, int(d / (self.carte.resolution / 2)))
        return all(self.libre(a[0] + (b[0] - a[0]) * k / n,
                              a[1] + (b[1] - a[1]) * k / n) for k in range(n + 1))

    def planifier(self, depart: Tuple[float, float],
                  arrivee: Tuple[float, float]) -> Tuple[Optional[List[Tuple[float, float]]], str]:
        """Chemin de points, ou (None, raison)."""
        if not self.libre(*arrivee):
            return None, (f"arrivée ({arrivee[0]:.2f}, {arrivee[1]:.2f}) dans "
                          f"un obstacle ou trop près : inatteignable")
        s = self._case(*depart)
        g = self._case(*arrivee)
        if not (0 <= s[0] < self.nx and 0 <= s[1] < self.ny):
            return None, "départ hors de la carte"
        if not self._libre[s[0]][s[1]]:
            # Le robot peut se trouver légèrement dans la marge (arrêté près
            # d'une table). On part de la case libre la plus proche plutôt
            # que de refuser de bouger.
            s = self._plus_proche_libre(s)
            if s is None:
                return None, "robot entouré d'obstacles : aucun départ libre"

        voisins = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                   (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
                   (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))]
        ouvert = [(0.0, s)]
        cout = {s: 0.0}
        parent: Dict[Tuple[int, int], Tuple[int, int]] = {}
        while ouvert:
            _, c = heapq.heappop(ouvert)
            if c == g:
                break
            for di, dj, w in voisins:
                n = (c[0] + di, c[1] + dj)
                if not (0 <= n[0] < self.nx and 0 <= n[1] < self.ny):
                    continue
                if not self._libre[n[0]][n[1]]:
                    continue
                if di and dj and not (self._libre[c[0] + di][c[1]]
                                      and self._libre[c[0]][c[1] + dj]):
                    continue                 # pas de diagonale qui coupe un coin
                nc = cout[c] + w
                if nc < cout.get(n, float("inf")):
                    cout[n] = nc
                    parent[n] = c
                    heapq.heappush(ouvert, (nc + math.dist(n, g), n))
        if g not in cout:
            return None, "aucun chemin : destination isolée par des obstacles"

        cases = [g]
        while cases[-1] != s:
            cases.append(parent[cases[-1]])
        cases.reverse()
        points = [self._centre(*c) for c in cases]
        points[-1] = arrivee
        return self._lisser([depart] + points[1:]), ""

    def _plus_proche_libre(self, s):
        for rayon in range(1, 20):
            for di in range(-rayon, rayon + 1):
                for dj in range(-rayon, rayon + 1):
                    n = (s[0] + di, s[1] + dj)
                    if (0 <= n[0] < self.nx and 0 <= n[1] < self.ny
                            and self._libre[n[0]][n[1]]):
                        return n
        return None

    def _lisser(self, pts):
        """Supprime les points intermédiaires quand une ligne droite libre
        les relie : un chemin en escalier de grille devient une suite de
        segments francs, plus naturelle à suivre pour un bipède."""
        if len(pts) <= 2:
            return pts
        out = [pts[0]]
        i = 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1 and not self._segment_libre(pts[i], pts[j]):
                j -= 1
            out.append(pts[j])
            i = j
        return out


# ─── Suiveur ──────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class Consigne:
    vx: float
    vy: float
    wz: float


class Suiveur:
    """Poursuite pure, adaptée à un bipède.

    Tant que l'écart de cap est grand, on TOURNE SUR PLACE : un bipède qui
    avance en virant serré dérape. Puis on avance en corrigeant le cap.
    À l'arrivée, on s'aligne sur l'orientation du poste.
    """

    def __init__(self, v_max: float = 0.30, w_max: float = 0.6,
                 v_min: float = 0.06, anticipation: float = 0.45,
                 tol_position: float = 0.08, tol_cap: float = 0.10,
                 cap_avant_avance: float = 0.5):
        self.v_max, self.w_max, self.v_min = v_max, w_max, v_min
        self.anticipation = anticipation
        self.tol_position, self.tol_cap = tol_position, tol_cap
        self.cap_avant_avance = cap_avant_avance
        # Alignement final : HYSTÉRÉSIS et arrivée par stagnation. Constaté
        # sur GPU (essai 30 min) : « navigation vers prise en échec » 4 fois,
        # robot tout près du poste pendant 90 s. En pivotant sur place, la
        # marche le déplace de quelques cm : il sortait de la tolérance de
        # position, repartait « en avance », revenait… sans fin. Désormais, une
        # fois arrivé, il reste en alignement tant qu'il est à moins de
        # 2 × tol_position ; et si le cap ne s'améliore plus pendant
        # `pas_stagnation` appels alors qu'il est à moins de 2 × tol_cap, il
        # est déclaré arrivé (un poste de navigation n'est pas une prise : la
        # précision fine, c'est l'accostage qui la donne).
        self._alignement = False
        self._but = None
        self._meilleur_e, self._pas_sans_progres = math.inf, 0
        self.pas_stagnation = 150            # ~3 s à 50 Hz
        self.arrivees_par_stagnation = 0

    def _franche(self, v: float, vmax: float) -> float:
        """Jamais dans la bande morte de la politique de marche."""
        if abs(v) < 1e-9:
            return 0.0
        return math.copysign(min(vmax, max(self.v_min, abs(v))), v)

    def _cible(self, pose: Pose2D, chemin) -> Tuple[float, float, int]:
        """Point du chemin situé à `anticipation` mètres devant le robot."""
        # Segment le plus proche.
        meilleur, k_min = float("inf"), 0
        for k in range(len(chemin) - 1):
            ax, ay = chemin[k]
            bx, by = chemin[k + 1]
            dx, dy = bx - ax, by - ay
            l2 = dx * dx + dy * dy or 1e-9
            t = max(0.0, min(1.0, ((pose.x - ax) * dx + (pose.y - ay) * dy) / l2))
            d = math.hypot(ax + t * dx - pose.x, ay + t * dy - pose.y)
            if d < meilleur:
                meilleur, k_min = d, k
        # Puis on avance le long du chemin.
        reste = self.anticipation
        x, y = pose.x, pose.y
        for k in range(k_min, len(chemin) - 1):
            bx, by = chemin[k + 1]
            d = math.hypot(bx - x, by - y)
            if d >= reste:
                return x + (bx - x) * reste / d, y + (by - y) * reste / d, k
            reste -= d
            x, y = bx, by
        return chemin[-1][0], chemin[-1][1], len(chemin) - 2

    def ecart_au_chemin(self, pose: Pose2D, chemin) -> float:
        meilleur = float("inf")
        for k in range(len(chemin) - 1):
            ax, ay = chemin[k]
            bx, by = chemin[k + 1]
            dx, dy = bx - ax, by - ay
            l2 = dx * dx + dy * dy or 1e-9
            t = max(0.0, min(1.0, ((pose.x - ax) * dx + (pose.y - ay) * dy) / l2))
            meilleur = min(meilleur, math.hypot(ax + t * dx - pose.x,
                                                ay + t * dy - pose.y))
        return meilleur

    def commande(self, pose: Pose2D, chemin, cap_final: Optional[float]
                 ) -> Tuple[Consigne, bool]:
        """(consigne, arrivé)."""
        fx, fy = chemin[-1]
        if self._but != (fx, fy, cap_final):
            self._but, self._alignement = (fx, fy, cap_final), False
            self._meilleur_e, self._pas_sans_progres = math.inf, 0
        d_fin = math.hypot(fx - pose.x, fy - pose.y)
        if d_fin < self.tol_position or (self._alignement and d_fin < 2 * self.tol_position):
            self._alignement = True
            if cap_final is None:
                return Consigne(0.0, 0.0, 0.0), True
            e = _angle(cap_final - pose.cap)
            if abs(e) < self.tol_cap:
                return Consigne(0.0, 0.0, 0.0), True
            if abs(e) < self._meilleur_e - 0.02:
                self._meilleur_e, self._pas_sans_progres = abs(e), 0
            else:
                self._pas_sans_progres += 1
            if self._pas_sans_progres >= self.pas_stagnation and abs(e) < 2 * self.tol_cap:
                self.arrivees_par_stagnation += 1
                return Consigne(0.0, 0.0, 0.0), True
            return Consigne(0.0, 0.0, self._franche(1.2 * e, self.w_max)), False
        self._alignement = False

        tx, ty, _ = self._cible(pose, chemin)
        e = _angle(math.atan2(ty - pose.y, tx - pose.x) - pose.cap)
        wz = self._franche(1.5 * e, self.w_max)
        if abs(e) > self.cap_avant_avance:
            return Consigne(0.0, 0.0, wz), False     # s'orienter d'abord
        # Ralentir à l'approche, et d'autant plus que le cap est faux.
        v = min(self.v_max, 0.8 * d_fin) * max(0.0, math.cos(e))
        return Consigne(self._franche(v, self.v_max), 0.0, wz), False


# ─── Service ──────────────────────────────────────────────────────────────

class EtatNavigation(Enum):
    INACTIF = auto()
    EN_ROUTE = auto()
    ARRIVE = auto()
    ECHEC = auto()


class NavigationService(IService):
    """Planifie, suit, et parle au service de locomotion."""

    def __init__(self, carte: Carte, source_pose: Callable[[], Pose2D],
                 locomotion, horloge: Callable[[], float] = time.monotonic,
                 pose_max_age_s: float = 0.5, ecart_replanif_m: float = 0.5,
                 rayon_robot: float = 0.30, marge: float = 0.10,
                 suiveur: Optional[Suiveur] = None,
                 attente_contournement_s: float = 4.0,
                 passage_min_m: float = 1.1,
                 duree_obstacle_temp_s: float = 60.0,
                 obstacles_temp_max: int = 4):
        """
        Contournement : si le gardien de proximité signale un obstacle
        INEXPLIQUÉ qui reste IMMOBILE (à 0.3 m près) sur le chemin restant
        pendant `attente_contournement_s`, on l'ajoute comme obstacle
        temporaire et on replanifie. Une personne qui passe bouge : elle ne
        déclenche rien. Le contournement se décide pendant le RALENTISSEMENT :
        une fois le robot arrêté par le gardien (personne à moins d'environ
        1 m), il ne manœuvre pas près d'elle, il attend — et c'est l'app
        qui alerte si l'attente dure.

        passage_min_m : distance minimale entre le centre du robot et
        l'obstacle sur le nouveau chemin. Au-dessus de la distance de
        reprise du gardien, sinon le robot s'arrêterait en le longeant.
        """
        self.carte = carte
        self._pose = source_pose
        self._loco = locomotion
        self._horloge = horloge
        self._age_max = pose_max_age_s
        self._ecart_max = ecart_replanif_m
        self.planif = Planificateur(carte, rayon_robot, marge)
        self.suiveur = suiveur or Suiveur()
        self.etat = EtatNavigation.INACTIF
        self.chemin: Optional[List[Tuple[float, float]]] = None
        self._but: Optional[Pose2D] = None
        self.raison = ""
        self._demarre = False
        self._attente_ct = attente_contournement_s
        self._passage = passage_min_m
        self._duree_temp = duree_obstacle_temp_s
        self._temp_max = obstacles_temp_max
        self._temp: List[Tuple[Rectangle, float]] = []      # (obstacle, fin)
        self._suspect: Optional[Tuple[Tuple[float, float], float]] = None
        self.stats = {"planifications": 0, "replanifications": 0,
                      "arrets_pose_perimee": 0, "contournements": 0,
                      "contournements_impossibles": 0}

    @property
    def service_id(self) -> str:
        return "navigation"

    @property
    def capability(self) -> Capability:
        return Capability.NAVIGATION_MAP

    def start(self) -> HALStatus:
        self._demarre = True
        return HALStatus.OK

    def stop(self) -> None:
        self.annuler()
        self._demarre = False

    def health(self) -> ServiceHealth:
        return ServiceHealth(running=self._demarre,
                             healthy=self.etat is not EtatNavigation.ECHEC,
                             detail=self.raison or self.etat.name)

    # ---- API ------------------------------------------------------------

    def aller_vers(self, destination) -> ServiceResult:
        """Destination : nom d'un poste, ou Pose2D. Non bloquant."""
        if not self._demarre:
            return ServiceResult(ok=False, reason="service non démarré")
        if isinstance(destination, str):
            if destination not in self.carte.postes:
                return ServiceResult(ok=False, reason=(
                    f"poste inconnu : « {destination} » — postes : "
                    f"{sorted(self.carte.postes)}"))
            destination = self.carte.postes[destination]
        self._but = destination
        self._suspect = None
        self._expirer_temporaires(self._horloge())
        return self._planifier(premiere=True)

    def annuler(self) -> None:
        self._but, self.chemin = None, None
        self.etat = EtatNavigation.INACTIF
        self._loco.halt()

    def _planifier(self, premiere: bool) -> ServiceResult:
        p = self._pose()
        chemin, raison = self.planif.planifier((p.x, p.y), (self._but.x, self._but.y))
        self.stats["planifications" if premiere else "replanifications"] += 1
        if chemin is None:
            self.etat, self.raison, self.chemin = EtatNavigation.ECHEC, raison, None
            self._loco.halt()
            return ServiceResult(ok=False, reason=raison)
        self.chemin, self.etat, self.raison = chemin, EtatNavigation.EN_ROUTE, ""
        return ServiceResult(ok=True, value=chemin)

    # ---- contournement ----------------------------------------------------

    def _expirer_temporaires(self, t: float) -> None:
        for o, fin in list(self._temp):
            if t >= fin:
                self._temp.remove((o, fin))
                self.planif.debloquer(o)

    def _sur_le_chemin(self, pose: Pose2D, pt: Tuple[float, float]) -> bool:
        """Le point gênera-t-il la suite du chemin ?"""
        ch = self.chemin
        _, _, k = self.suiveur._cible(pose, ch)
        reste = [(pose.x, pose.y)] + list(ch[k + 1:])
        for a, b in zip(reste, reste[1:]):
            dx, dy = b[0] - a[0], b[1] - a[1]
            l2 = dx * dx + dy * dy or 1e-9
            u = max(0.0, min(1.0, ((pt[0] - a[0]) * dx + (pt[1] - a[1]) * dy) / l2))
            if math.hypot(a[0] + u * dx - pt[0], a[1] + u * dy - pt[1]) < self._passage:
                return True
        return False

    def _surveiller_obstacle(self, pose: Pose2D) -> None:
        t = self._horloge()
        self._expirer_temporaires(t)
        v = getattr(self._loco, "dernier_verdict", None)
        pt = getattr(v, "point", None)
        deja = any(o.contient(pt[0], pt[1], 0.5) for o, _ in self._temp) \
            if pt is not None else False          # le même, vu sous un autre angle
        if v is None or pt is None or v.facteur >= 1.0 or deja \
                or not self._sur_le_chemin(pose, pt):
            self._suspect = None
            return
        if self._suspect is None or math.dist(self._suspect[0], pt) > 0.3:
            self._suspect = (pt, t)               # nouveau, ou il bouge
            return
        if t - self._suspect[1] < self._attente_ct:
            return
        self._suspect = None
        if len(self._temp) >= self._temp_max:
            return
        # Le lidar voit la face de l'objet tournée vers le robot : son centre
        # est un peu plus loin. On centre le carré sur ce centre estimé, et
        # on l'élargit de la demi-taille supposée de l'objet (une personne,
        # une palette) : le robot passe alors à `passage_min_m` de TOUTE sa
        # surface, pas seulement du point vu.
        demi_objet = 0.25
        ux, uy = pt[0] - pose.x, pt[1] - pose.y
        n = math.hypot(ux, uy) or 1.0
        cx, cy = pt[0] + 0.2 * ux / n, pt[1] + 0.2 * uy / n
        h = max(0.05, self._passage - self.planif.gonflement + demi_objet)
        o = Rectangle(cx - h, cy - h, cx + h, cy + h)
        self.planif.bloquer(o)
        ancien = self.chemin
        r = self._planifier(premiere=False)
        if r.ok:
            self._temp.append((o, t + self._duree_temp))
            self.stats["contournements"] += 1
            self.raison = (f"obstacle immobile en ({pt[0]:.1f}, {pt[1]:.1f}) — "
                           f"contourné")
        else:
            # Pas d'autre passage : on garde le chemin et on attend.
            self.planif.debloquer(o)
            self.chemin, self.etat = ancien, EtatNavigation.EN_ROUTE
            self.raison = "obstacle immobile, aucun autre passage — attente"
            self.stats["contournements_impossibles"] += 1

    def tick(self) -> ServiceResult:
        """Un pas, au rythme du service de locomotion."""
        if self.etat is not EtatNavigation.EN_ROUTE:
            return ServiceResult(ok=True, value=self.etat)
        p = self._pose()
        self._surveiller_obstacle(p)
        if self._horloge() - p.horodatage > self._age_max:
            self.stats["arrets_pose_perimee"] += 1
            self._loco.halt()
            return ServiceResult(ok=False, reason=(
                f"position du robot périmée de "
                f"{self._horloge() - p.horodatage:.2f} s — arrêt : naviguer sur "
                f"une position ancienne, c'est naviguer à l'aveugle"))
        if self.suiveur.ecart_au_chemin(p, self.chemin) > self._ecart_max:
            r = self._planifier(premiere=False)
            if not r.ok:
                return r
        c, arrive = self.suiveur.commande(p, self.chemin, self._but.cap)
        if arrive:
            self._loco.halt()
            self.etat = EtatNavigation.ARRIVE
            return ServiceResult(ok=True, value=self.etat)
        self._loco.set_velocity(c.vx, c.vy, c.wz)
        return ServiceResult(ok=True, value=self.etat)
