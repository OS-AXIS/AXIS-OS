"""
AXIS-OS — Gardien de proximité : des champs de protection autour du robot.

Pourquoi : un humanoïde qui marche dans un entrepôt partage l'espace avec
des personnes. Ce module reprend le principe des scanners laser de sécurité
qui équipent les chariots autonomes industriels, et l'applique à un
humanoïde. Le lidar (le capteur laser qui mesure les distances tout autour
du robot) voit dans toutes les directions. Tout ce qu'il voit et qui n'est
pas expliqué est traité comme un obstacle, potentiellement humain. La
vitesse autorisée dépend de la distance horizontale au plus proche de ces
obstacles :

    au-delà de d_ralenti   pleine vitesse
    entre les deux         vitesse plafonnée proportionnellement : d'autant
                           plus basse que l'obstacle est proche
    en deçà de d_arret     ARRÊT, en gardant l'équilibre (ce n'est pas une
                           coupure des moteurs)

La sécurité ne repose pas sur la reconnaissance. S'arrêter seulement devant
ce qu'un modèle identifie comme « une personne », c'est ignorer l'humain vu
de dos, accroupi, à contre-jour, ou simplement mal reconnu. Ici, un
obstacle inexpliqué suffit. La reconnaissance peut enrichir l'affichage
(« humain détecté ») ; elle ne décide jamais de la sécurité.

Ce qui est expliqué, et donc ignoré :
  - le sol et le plafond : seuls comptent les points situés dans une bande
    de hauteur utile, entre h_min et h_max ;
  - le robot lui-même : son empreinte au sol (rayon_robot) ;
  - le volume de port, devant le torse, à hauteur des bras : le carton que
    le robot porte ne doit pas l'arrêter. Un humain qui se tiendrait dans
    ce volume n'est pas ignoré pour autant : ses jambes restent visibles en
    dessous ;
  - les obstacles fixes de la carte (étagères, tables) : le robot doit
    pouvoir s'en approcher pour y prendre un carton.

Un carton oublié par terre, lui, n'est expliqué par rien : le robot
s'arrête. C'est voulu : mieux vaut s'arrêter pour rien que heurter
quelqu'un.

Sûr par défaut (« fail-safe ») : pas de balayage, ou un balayage périmé, et
le robot s'arrête. Un capteur de sécurité muet n'est pas un capteur qui ne
voit rien.

Entrée invalide : arrêt. Pourquoi : avec NaN (valeur numérique invalide),
toute comparaison est fausse. Un point NaN n'était donc ni « proche » ni
« expliqué » : il disparaissait du calcul. Un audit de sécurité (octobre
2026) l'a constaté : une pose du robot (sa position et son orientation) NaN
rendait « zone dégagée » alors qu'un obstacle se tenait à 0.58 m ; un
balayage entièrement NaN, ou un horodatage NaN ou situé dans le futur,
passaient aussi. Désormais, le robot s'arrête, comme devant un capteur
muet, dans chacun de ces cas : pose ou horodatage non finis, horodatage
dans le futur, balayage mal formé ou sans aucun point fini, source de
données en erreur. Ces cas sont testés dans
test/test_proximite_entrees_invalides.py.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Optional, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class ChampsProtection:
    """Réglages du gardien : distances en mètres, durées en secondes."""

    # Distances horizontales, mesurées depuis le centre du robot.
    d_arret: float = 0.80          # obstacle plus proche : arrêt
    d_ralenti: float = 2.00        # obstacle plus proche : vitesse plafonnée
    # Hystérésis (un seuil de reprise plus large que le seuil d'arrêt) : un
    # robot arrêté ne repart pas dès que l'obstacle s'éloigne à 0.81 m, mais
    # seulement au-delà de d_reprise...
    d_reprise: float = 1.00
    # ... et seulement quand la zone est restée dégagée pendant ce délai.
    delai_reprise_s: float = 1.0
    rayon_robot: float = 0.35      # empreinte du robot, ses points ignorés
    h_min: float = 0.05            # plus bas : le sol, ignoré
    h_max: float = 1.90            # plus haut : le plafond, ignoré
    # Volume de port : la boîte, devant le torse, où se trouve le carton
    # porté. x et y sont dans le repère du robot (x vers l'avant, y vers la
    # gauche, origine au centre du robot) ; z est la hauteur dans le repère
    # monde (le repère fixe de l'entrepôt).
    port_x: tuple = (0.15, 0.60)   # de 15 à 60 cm devant le centre du robot
    port_y: float = 0.30           # jusqu'à 30 cm de chaque côté de son axe
    port_z: tuple = (0.60, 1.30)   # entre 0.60 m et 1.30 m de hauteur
    # Tolérance autour des obstacles de la carte : un point situé jusqu'à
    # 5 cm hors d'un obstacle connu lui est encore attribué.
    marge_carte: float = 0.05
    age_max_s: float = 0.3         # balayage plus vieux : périmé -> arrêt
    # Avance maximale tolérée pour un horodatage situé dans le futur. Au-delà,
    # les horloges du lidar et du robot sont désynchronisées et l'âge mesuré
    # ne veut plus rien dire : un vieux balayage paraîtrait frais
    # indéfiniment. Le robot s'arrête.
    avance_max_s: float = 0.05
    # Exemption pendant une manipulation (voir GardienProximite.exempter) :
    # longueur maximale de chaque côté du volume exempté, et durée maximale.
    exemption_cote_max: float = 0.70
    exemption_duree_max_s: float = 30.0


@dataclass(frozen=True)
class Verdict:
    """Décision du gardien pour un balayage."""

    facteur: float                 # multiplie la vitesse : 0 arrêt, 1 pleine
    # Distance horizontale (m) à l'obstacle inexpliqué le plus proche ; inf
    # s'il n'y en a aucun, None faute de mesure fiable.
    distance: Optional[float]
    raison: str                    # explication lisible de la décision
    # Position (x, y) de ce point dans le repère monde. La navigation s'en
    # sert pour contourner un obstacle qui reste planté sur son chemin.
    point: Optional[Tuple[float, float]] = None


class GardienProximite:
    """Évalue le dernier balayage du lidar et rend un facteur de vitesse.

    Le service de locomotion l'appelle à chaque pas de sa boucle de commande
    et multiplie la vitesse demandée par le facteur rendu.

    source_scan  : fonction sans argument qui rend (points, horodatage), ou
                   None si aucun balayage n'est disponible. points est un
                   tableau N x 3 de points (x, y, z) dans le repère monde ;
                   l'horodatage est l'instant de la mesure, sur la même
                   horloge que l'instant passé à evaluer().
    source_pose  : fonction sans argument qui rend (x, y, cap) : la position
                   du robot dans le repère monde et son cap (son orientation
                   autour de l'axe vertical, en radians).
    obstacles    : obstacles fixes de la carte, des rectangles alignés sur
                   les axes, avec des attributs xmin, ymin, xmax, ymax et un
                   attribut zmax facultatif (comme Rectangle, dans
                   axis_os/services/navigation/service.py).
    champs       : distances et réglages (ChampsProtection).
    horloge      : facultative, conservée mais pas lue par le gardien :
                   l'instant présent lui est passé à chaque evaluer().
    v_min, v_max : vitesse minimale de marche et vitesse maximale, en m/s.
                   Elles servent à décider quand un ralentissement doit
                   devenir un arrêt (voir evaluer).
    """

    def __init__(self, source_scan: Callable, source_pose: Callable,
                 obstacles: Sequence = (), champs: ChampsProtection = ChampsProtection(),
                 horloge: Callable[[], float] = None, v_min: float = 0.06,
                 v_max: float = 0.30):
        self._scan = source_scan
        self._pose = source_pose
        self._obstacles = list(obstacles)
        self.c = champs
        self._horloge = horloge
        self._v_min, self._v_max = v_min, v_max
        self._arrete = False
        self._degage_depuis: Optional[float] = None
        # Exemption en cours, ou None : (xmin, xmax, ymin, ymax, zmin, zmax,
        # instant de fin), repère monde.
        self._exemption = None
        self.journal_exemptions = []
        self.stats = {"arrets": 0, "ralentissements": 0, "capteur_muet": 0,
                      "exemptions": 0, "exemptions_refusees": 0,
                      "entrees_invalides": 0}

    def exempter(self, volume, duree_s: float, maintenant: float,
                 motif: str = "") -> tuple:
        """Exemption contrôlée pendant une manipulation.

        Pourquoi : le carton à prendre est posé sur une table, au-dessus de
        son plateau. Rien ne l'explique (la table n'explique que ce qui est
        sous son plateau), et le robot s'arrêterait devant l'objet même
        qu'il doit saisir. Comme les chariots industriels qui neutralisent
        un champ de protection pour s'amarrer à un poste, on retire de la
        surveillance un petit volume autour de l'objet ciblé.

        Garde-fous : chaque côté du volume mesure au plus
        exemption_cote_max et la durée vaut au plus exemption_duree_max_s ;
        au-delà, l'exemption est refusée. Chaque exemption accordée est
        journalisée (journal_exemptions). Une seule à la fois : une nouvelle
        exemption remplace la précédente. Tout ce qui est hors du volume
        reste surveillé, par exemple les jambes d'un humain qui s'approche
        de la table, sous le plateau. L'exemption expire d'elle-même ;
        lever_exemption() la retire plus tôt.

        volume     : (xmin, xmax, ymin, ymax, zmin, zmax), en mètres, dans le
                     repère monde.
        duree_s    : durée, en secondes, dans ]0, exemption_duree_max_s].
        maintenant : instant présent, sur l'horloge passée à evaluer().
        motif      : texte libre, conservé dans le journal.

        Rend (True, "") si l'exemption est accordée, (False, raison) si elle
        est refusée : volume vide ou inversé, volume trop grand, durée hors
        des bornes.
        """
        xmin, xmax, ymin, ymax, zmin, zmax = volume
        cotes = (xmax - xmin, ymax - ymin, zmax - zmin)
        if min(cotes) <= 0:
            self.stats["exemptions_refusees"] += 1
            return False, "volume vide ou inversé"
        if max(cotes) > self.c.exemption_cote_max:
            self.stats["exemptions_refusees"] += 1
            return False, (f"volume trop grand ({max(cotes):.2f} m > "
                           f"{self.c.exemption_cote_max} m) — une exemption ne "
                           f"peut couvrir que l'objet manipulé")
        if duree_s <= 0 or duree_s > self.c.exemption_duree_max_s:
            self.stats["exemptions_refusees"] += 1
            return False, (f"durée {duree_s} s hors de ]0, "
                           f"{self.c.exemption_duree_max_s}] s")
        self._exemption = (xmin, xmax, ymin, ymax, zmin, zmax, maintenant + duree_s)
        self.stats["exemptions"] += 1
        self.journal_exemptions.append((maintenant, volume, duree_s, motif))
        return True, ""

    def lever_exemption(self) -> None:
        """Retire l'exemption en cours : la surveillance reprend partout."""
        self._exemption = None

    def distance_arret(self) -> float:
        """Distance (m) en deçà de laquelle le gardien tient le robot à
        l'arrêt : d_arret, ou plus loin quand le plafond de vitesse tombe sous
        la vitesse minimale de marche (voir evaluer), et au moins d_reprise.
        Lecture seule : un service qui garde ses distances (le suivi d'une
        personne, axis_os/services/people/follow.py) s'adapte au gardien,
        jamais l'inverse."""
        c = self.c
        bande = (c.d_arret + (self._v_min / self._v_max) * (c.d_ralenti - c.d_arret)
                 if self._v_max > 0 else c.d_ralenti)
        return max(c.d_arret, c.d_reprise, bande)

    @property
    def arrete(self) -> bool:
        """Vrai si le gardien tient le robot à l'arrêt.

        Les autres services le lisent aussi, comme celui des bras : un arrêt
        de sécurité fige tout le robot, pas seulement les jambes.
        """
        return self._arrete

    def _dans_carte(self, x: np.ndarray, y: np.ndarray, z: np.ndarray) -> np.ndarray:
        """Masque des points expliqués par un obstacle connu, en trois
        dimensions.

        Pourquoi la hauteur compte : une première version travaillait en
        deux dimensions, et tout ce qui se trouvait au-dessus de l'emprise
        d'une table était ignoré, à n'importe quelle hauteur. Un humain
        penché au-dessus d'une table aurait été invisible. Désormais, un
        obstacle qui déclare sa hauteur (zmax, par exemple le plateau d'une
        table) n'explique que les points situés en dessous ; sans zmax, il
        explique toute la hauteur (un mur, une étagère qui monte jusqu'au
        plafond). La marge marge_carte s'ajoute de chaque côté.
        """
        m = np.zeros(x.shape, dtype=bool)
        mc = self.c.marge_carte
        for o in self._obstacles:
            dedans = ((x >= o.xmin - mc) & (x <= o.xmax + mc)
                      & (y >= o.ymin - mc) & (y <= o.ymax + mc))
            zmax = getattr(o, "zmax", None)
            if zmax is not None:
                dedans &= z <= zmax + mc
            m |= dedans
        return m

    def _dans_exemption(self, x, y, z, maintenant):
        """Masque des points situés dans l'exemption en cours, s'il y en a
        une. Une exemption expirée est retirée ici."""
        e = self._exemption
        if e is None:
            return np.zeros(x.shape, dtype=bool)
        if maintenant > e[6]:
            self._exemption = None       # expirée : la surveillance reprend
            return np.zeros(x.shape, dtype=bool)
        return ((x >= e[0]) & (x <= e[1]) & (y >= e[2]) & (y <= e[3])
                & (z >= e[4]) & (z <= e[5]))

    def distance_inexpliquee(self, points: np.ndarray, pose,
                             maintenant: float = 0.0) -> Optional[float]:
        """Distance horizontale, en mètres, au point inexpliqué le plus
        proche : inf s'il n'y en a aucun, None si le balayage est vide.

        Contrairement à evaluer(), ne vérifie ni l'âge du balayage ni la
        validité de la pose.
        """
        return self._plus_proche(points, pose, maintenant)[0]

    def _plus_proche(self, points: np.ndarray, pose, maintenant: float = 0.0):
        """Rend (distance, (x, y)) du point inexpliqué le plus proche :
        distance horizontale en mètres, position dans le repère monde.

        (inf, None) si aucun point n'est inexpliqué ; (None, None) si le
        balayage est vide.
        """
        if points is None or len(points) == 0:
            return None, None
        px, py, cap = pose
        P = np.asarray(points, dtype=float)
        # Les points non finis (NaN ou infinis) sont écartés explicitement.
        # Ils l'étaient déjà, mais par accident : toute comparaison avec NaN
        # est fausse. evaluer() arrête le robot quand il ne reste aucun point
        # fini.
        P = P[np.isfinite(P[:, :3]).all(axis=1)]
        x, y, z = P[:, 0], P[:, 1], P[:, 2]
        dx, dy = x - px, y - py
        d = np.hypot(dx, dy)
        # Coordonnées dans le repère du robot (x vers l'avant, y vers la
        # gauche) : le volume de port suit le robot quand il tourne.
        c, s = math.cos(cap), math.sin(cap)
        xr, yr = c * dx + s * dy, -s * dx + c * dy
        # `garde` marque les points à surveiller : au départ, ceux de la
        # bande de hauteur utile ; on retire ensuite, un à un, ceux qu'une
        # raison connue explique.
        garde = (z >= self.c.h_min) & (z <= self.c.h_max)          # ni sol ni plafond
        garde &= d > self.c.rayon_robot                              # pas le robot
        port = ((xr >= self.c.port_x[0]) & (xr <= self.c.port_x[1])
                & (np.abs(yr) <= self.c.port_y)
                & (z >= self.c.port_z[0]) & (z <= self.c.port_z[1]))
        garde &= ~port                                               # carton porté
        garde &= ~self._dans_carte(x, y, z)                          # étagères, tables
        garde &= ~self._dans_exemption(x, y, z, maintenant)          # objet manipulé
        if not garde.any():
            return float("inf"), None
        k = np.flatnonzero(garde)[np.argmin(d[garde])]
        return float(d[k]), (float(x[k]), float(y[k]))

    def _arret_capteur(self, raison: str, cle: str = "capteur_muet") -> Verdict:
        """Arrêt faute de mesure fiable.

        Le délai de reprise repart de zéro. Pourquoi : pendant que le
        capteur ne donnait rien de fiable, rien ne prouve que la zone soit
        restée dégagée. Sans cette remise à zéro, un robot arrêté pouvait
        repartir dès le retour du capteur.

        `cle` désigne le compteur de self.stats à incrémenter : capteur_muet
        par défaut, entrees_invalides pour une donnée invalide.
        """
        self.stats[cle] += 1
        self._arrete, self._degage_depuis = True, None
        return Verdict(0.0, None, raison)

    def evaluer(self, maintenant: float) -> Verdict:
        """Décide de la vitesse autorisée à l'instant `maintenant`.

        `maintenant` doit être sur la même horloge que les horodatages des
        balayages. Lit un balayage et la pose du robot, puis rend un
        Verdict. Tout doute sur les données (horloge, balayage ou pose
        invalides ; balayage absent, vide ou périmé) donne un arrêt, jamais
        la pleine vitesse.

        Un arrêt est verrouillé : le robot ne repart que lorsque la zone est
        restée dégagée au-delà de d_reprise pendant delai_reprise_s.
        """
        if not math.isfinite(maintenant):
            return self._arret_capteur("horloge du robot non finie — ARRÊT",
                                       "entrees_invalides")
        try:
            s = self._scan()
        except Exception as exc:          # noqa: BLE001 — capteur en panne
            # Une source en erreur arrête le robot au lieu de faire tomber la
            # boucle de commande : une boucle qui plante ne laisse pas le
            # robot s'arrêter en équilibre.
            return self._arret_capteur(f"lidar en erreur ({exc!r:.80}) — ARRÊT",
                                       "entrees_invalides")
        if s is None:
            return self._arret_capteur("lidar muet — ARRÊT par sécurité")
        try:
            points, horodatage = s
            horodatage = float(horodatage)
        except (TypeError, ValueError):
            return self._arret_capteur("balayage mal formé — ARRÊT",
                                       "entrees_invalides")
        if not math.isfinite(horodatage):
            # Avec un horodatage NaN, « maintenant - NaN > age_max_s » est
            # faux : un balayage d'âge inconnu passait pour frais.
            return self._arret_capteur("horodatage du balayage non fini — ARRÊT",
                                       "entrees_invalides")
        if horodatage - maintenant > self.c.avance_max_s:
            return self._arret_capteur(
                f"horodatage du balayage {horodatage - maintenant:.2f} s dans le "
                f"FUTUR — horloges désynchronisées — ARRÊT", "entrees_invalides")
        if maintenant - horodatage > self.c.age_max_s:
            return self._arret_capteur(f"balayage périmé de "
                                       f"{maintenant - horodatage:.2f} s — ARRÊT")
        if points is None or len(points) == 0:
            # Un lidar voit toujours quelque chose, au moins le sol : un
            # balayage vide signale un capteur aveugle ou masqué. Constaté au
            # premier essai sur carte graphique (GPU) de la scène Isaac Sim
            # (le simulateur de robots de NVIDIA) de la démo de tri, voir
            # sim/validation/test_isaac_t1_tri_usine.py : le lidar, placé à
            # l'intérieur du volume de collision de la tête du robot, voyait
            # chacun de ses rayons s'arrêter sur le robot lui-même, qui
            # restait arrêté pour de bon. On s'arrête en donnant la vraie
            # raison : la version précédente affichait à tort « zone pas
            # encore dégagée ».
            return self._arret_capteur("balayage VIDE — lidar aveugle ou masqué — ARRÊT")
        try:
            P = np.asarray(points, dtype=float)
        except (TypeError, ValueError):
            return self._arret_capteur("points du balayage illisibles — ARRÊT",
                                       "entrees_invalides")
        if P.ndim != 2 or P.shape[1] < 3:
            return self._arret_capteur(f"balayage de forme {P.shape}, attendu "
                                       f"(N, 3) — ARRÊT", "entrees_invalides")
        if not np.isfinite(P[:, :3]).all(axis=1).any():
            # Aucun point fini : c'est un capteur aveugle, pas une zone vide.
            return self._arret_capteur("balayage sans AUCUN point fini — lidar "
                                       "aveugle — ARRÊT", "entrees_invalides")
        try:
            px, py, cap = (float(v) for v in self._pose())
        except Exception as exc:          # noqa: BLE001 — localisation en panne
            return self._arret_capteur(f"pose du robot indisponible ({exc!r:.80}) "
                                       f"— ARRÊT", "entrees_invalides")
        if not (math.isfinite(px) and math.isfinite(py) and math.isfinite(cap)):
            # Constaté par l'audit de sécurité : avec une pose NaN, toutes
            # les distances sont NaN, aucun point n'est retenu, et le gardien
            # rendait « zone dégagée » alors qu'un obstacle était à 0.58 m.
            return self._arret_capteur("pose du robot non finie — ARRÊT",
                                       "entrees_invalides")
        d, pt = self._plus_proche(P, (px, py, cap), maintenant)

        if d is not None and d < self.c.d_arret:
            if not self._arrete:
                self.stats["arrets"] += 1
            self._arrete, self._degage_depuis = True, None
            return Verdict(0.0, d, f"obstacle inexpliqué à {d:.2f} m — ARRÊT", pt)

        if self._arrete:
            # Robot déjà arrêté : il ne repart que si la zone est dégagée
            # au-delà de d_reprise, sans interruption pendant delai_reprise_s.
            if d is None or d < self.c.d_reprise:
                self._degage_depuis = None
                return Verdict(0.0, d, "en attente : zone pas encore dégagée", pt)
            if self._degage_depuis is None:
                self._degage_depuis = maintenant
            if maintenant - self._degage_depuis < self.c.delai_reprise_s:
                return Verdict(0.0, d, "zone dégagée — reprise imminente", pt)
            self._arrete, self._degage_depuis = False, None

        if d is not None and d < self.c.d_ralenti:
            f = (d - self.c.d_arret) / (self.c.d_ralenti - self.c.d_arret)
            # Un plafond inférieur à la vitesse minimale de marche ne ferait
            # pas ralentir le robot : il le ferait tenir debout en piétinant.
            # C'est la bande morte de GR00T WBC (GR00T Whole-Body Control,
            # la politique de marche de NVIDIA, un réseau de neurones
            # entraîné qui commande les jambes et la taille du robot) : sous
            # une certaine vitesse demandée, elle n'avance plus. Dans ce
            # cas, on arrête franchement.
            if f * self._v_max < self._v_min:
                # Arrêt verrouillé, comme sous d_arret : sans ce verrou, le
                # robot oscillerait entre arrêt et marche à la frontière.
                if not self._arrete:
                    self.stats["arrets"] += 1
                self._arrete, self._degage_depuis = True, None
                return Verdict(0.0, d, f"obstacle à {d:.2f} m — arrêt "
                                       f"(vitesse plafonnée sous le minimum)", pt)
            self.stats["ralentissements"] += 1
            return Verdict(f, d, f"obstacle à {d:.2f} m — vitesse ×{f:.2f}", pt)
        return Verdict(1.0, d, "zone dégagée", pt)
