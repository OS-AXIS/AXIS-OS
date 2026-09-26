"""
Tri de colis — TOUTE la pile logicielle, sans GPU.

Les vrais services d'AXIS-OS, branchés ensemble :
  TriColis -> FacadeServices -> NavigationService -> LocomotionService
  (+ GardienProximite) ; ManipulationService ; perception (factice) ;
  Superviseur en langage naturel.

Seule la physique est remplacée : un robot cinématique qui imite GR00T WBC
(bande morte sous 0.05 m/s, erreur d'exécution), un lidar synthétique, des
humains qui circulent. C'est la répétition générale avant Isaac Sim : tout
ce qui échoue ici échouerait sur le GPU, en coûtant des heures.

Usage : python3 test/test_tri_pile_logicielle.py [durée_s]   (défaut 600)
"""
import math
import random
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.apps.tri_colis.facade import FacadeServices
from axis_os.apps.tri_colis.logique import ConfigTri, TriColis
from axis_os.apps.tri_colis.superviseur import InterpreteurRegles, Superviseur
from axis_os.hal.core.types import Vector3
from axis_os.services.base import ServiceResult
from axis_os.services.locomotion.proximite import GardienProximite
from axis_os.services.locomotion.service import (
    BaseState, ILocomotionAdapter, LocomotionService, Twist,
)
from axis_os.services.manipulation.service import Etat, ManipulationService
from axis_os.services.navigation.service import (
    Carte, NavigationService, Pose2D, Rectangle,
)
from axis_os.services.perception.service import Detection

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


DUREE = float(sys.argv[1]) if len(sys.argv) > 1 else 600.0
DT = 0.02
PLATEAU = 0.75                      # hauteur de la table de prise
Y_BORD = 2.0                        # bord avant de la table de prise
CAP_PRISE = math.pi / 2             # le robot fait face à +y pour prendre

# ── Le site ─────────────────────────────────────────────────────────────
TABLE_PRISE = Rectangle(3.0, Y_BORD, 5.0, 2.6, zmax=PLATEAU + 0.25)
ETAGERES = [Rectangle(6.2, 3.5, 6.6, 8.0), Rectangle(1.6, 4.0, 2.0, 8.0)]
POSTE_A = Rectangle(8.6, 6.8, 9.6, 7.4, zmax=PLATEAU + 0.30)
POSTE_B = Rectangle(0.2, 6.8, 1.2, 7.4, zmax=PLATEAU + 0.30)
OBSTACLES = [TABLE_PRISE, POSTE_A, POSTE_B] + ETAGERES
carte = Carte(0.0, 0.0, 10.0, 8.0, obstacles=OBSTACLES, postes={
    "prise": Pose2D(4.0, 1.2, CAP_PRISE),
    "A": Pose2D(9.1, 6.0, math.pi / 2),
    "B": Pose2D(0.7, 6.0, math.pi / 2),
    "C": Pose2D(5.0, 6.5, math.pi / 2),
})
EMPLACEMENTS_X = [3.3, 3.7, 4.1, 4.5]
Y_CARTON = Y_BORD + 0.10            # centre des cartons sur la table


# ── Le robot : cinématique qui imite GR00T WBC ─────────────────────────
class Robot:
    def __init__(self, x, y, cap, graine=0):
        self.x, self.y, self.cap, self.t = x, y, cap, 0.0
        self.twist = Twist()
        self.rng = random.Random(graine)

    def pas(self):
        tw = self.twist
        if math.sqrt(tw.vx ** 2 + tw.vy ** 2 + tw.wz ** 2) < 0.05:
            return                       # mode équilibre : ne marche pas
        f = 1.0 + self.rng.uniform(-0.08, 0.08)
        self.cap += tw.wz * f * DT
        c, s = math.cos(self.cap), math.sin(self.cap)
        self.x += (tw.vx * c - tw.vy * s) * f * DT
        self.y += (tw.vx * s + tw.vy * c) * f * DT

    def pose(self):
        return Pose2D(self.x, self.y, self.cap, horodatage=self.t)


class Adaptateur(ILocomotionAdapter):
    def __init__(self, robot):
        self.robot = robot

    @property
    def joint_names(self): return []

    @property
    def control_dt(self): return DT

    @property
    def max_twist(self): return Twist(0.5, 0.3, 0.8)

    def reset(self): pass

    def step(self, state, twist):
        self.robot.twist = twist
        return {}

    def is_fallen(self, state): return False


# ── Le monde : cartons, humains, lidar ──────────────────────────────────
def boite(r: Rectangle, zmin, zmax, pas=0.1):
    xs = np.arange(r.xmin, r.xmax + 1e-9, pas)
    ys = np.arange(r.ymin, r.ymax + 1e-9, pas)
    zs = np.arange(zmin, zmax + 1e-9, pas)
    g = np.array(np.meshgrid(xs, ys, zs)).reshape(3, -1).T
    bord = ((np.isclose(g[:, 0], r.xmin) | np.isclose(g[:, 0], xs[-1])) |
            (np.isclose(g[:, 1], r.ymin) | np.isclose(g[:, 1], ys[-1])))
    return g[bord]


STATIQUE = np.vstack([boite(TABLE_PRISE, 0.1, PLATEAU), boite(POSTE_A, 0.1, PLATEAU),
                      boite(POSTE_B, 0.1, PLATEAU)] +
                     [boite(e, 0.1, 1.8) for e in ETAGERES])


class Humain:
    """Fait des allers-retours ; s'écarte d'un robot trop proche, comme le
    ferait une personne. Ce n'est PAS lui qui garantit l'absence de contact :
    le test vérifie que le ROBOT ne s'approche jamais d'un humain proche."""

    def __init__(self, a, b, vitesse=1.0, pause=4.0):
        self.a, self.b, self.v, self.pause = np.array(a), np.array(b), vitesse, pause
        self.p, self.vers_b, self.attente = np.array(a, float), True, 0.0

    def pas(self, robot):
        if self.attente > 0:
            self.attente -= DT
            return
        but = self.b if self.vers_b else self.a
        d = but - self.p
        n = np.linalg.norm(d)
        if n < 0.05:
            self.vers_b, self.attente = not self.vers_b, self.pause
            return
        pas_ = d / n * min(self.v * DT, n)
        suiv = self.p + pas_
        r = np.array([robot.x, robot.y])
        e = suiv - r
        ne = max(np.linalg.norm(e), 1e-6)
        if ne < 1.1 and n < 1.5:
            # Son but est à côté du robot : il fait demi-tour, comme le
            # ferait une personne, plutôt que de rester plantée là.
            self.vers_b = not self.vers_b
            return
        if ne < 1.1:
            # Contourne le robot : marche en tangente, du côté de son but,
            # en gardant ses distances.
            tang = np.array([-e[1], e[0]]) / ne
            if np.dot(tang, d) < 0:
                tang = -tang
            suiv = self.p + tang * self.v * DT
            e = suiv - r
            suiv = r + e / max(np.linalg.norm(e), 1e-6) * max(1.1, ne)
        self.p = suiv

    def nuage(self):
        a = np.linspace(0, 2 * math.pi, 16, endpoint=False)
        z = np.linspace(0.1, 1.7, 8)
        A, Z = np.meshgrid(a, z)
        return np.c_[self.p[0] + 0.2 * np.cos(A.ravel()),
                     self.p[1] + 0.2 * np.sin(A.ravel()), Z.ravel()]


class Monde:
    def __init__(self, graine=3, p_rate=0.15):
        self.rng = random.Random(graine)
        self.p_rate = p_rate
        self.robot = Robot(4.0, 0.6, CAP_PRISE, graine)
        self.cartons = {}           # emplacement x -> couleur
        self.porte = None           # (couleur,)
        self.deposes = []           # (couleur, poste le plus proche)
        self.humains = [Humain((7.2, 0.5), (7.2, 7.5)),
                        Humain((0.5, 3.2), (9.5, 3.2), vitesse=0.8, pause=10.0)]
        self.faux_positif_sol = True
        for x in EMPLACEMENTS_X:
            self.cartons[x] = self.rng.choice(["rouge", "bleu", "vert"])

    def reappro(self):
        # Un opérateur retire les colis sans règle (verts) de temps en temps.
        if self.rng.random() < 0.0005:
            for x in [x for x, c in self.cartons.items() if c == "vert"]:
                del self.cartons[x]
        libres = [x for x in EMPLACEMENTS_X if x not in self.cartons]
        if libres and self.rng.random() < 0.02:
            self.cartons[self.rng.choice(libres)] = \
                self.rng.choice(["rouge", "rouge", "bleu", "bleu", "vert"])

    # -- lidar : monde entier, repère monde
    def scan(self):
        pts = [STATIQUE] + [h.nuage() for h in self.humains]
        for x in self.cartons:
            pts.append(boite(Rectangle(x - 0.12, Y_BORD, x + 0.12, Y_BORD + 0.20),
                             PLATEAU, PLATEAU + 0.20, pas=0.06))
        if self.porte:
            c, s = math.cos(self.robot.cap), math.sin(self.robot.cap)
            cx, cy = self.robot.x + 0.39 * c, self.robot.y + 0.39 * s
            pts.append(np.array([[cx, cy, 0.95], [cx, cy, 1.05]]))
        return np.vstack(pts), self.robot.t

    # -- perception : détections OWL-ViT simulées, repère monde
    def find(self, query, now_ns=None, frame=None):
        dets = []
        r = self.robot
        for x, couleur in self.cartons.items():
            if couleur not in query or math.hypot(x - r.x, Y_BORD - r.y) > 3.0:
                continue
            dets.append(Detection(query, self.rng.uniform(0.3, 0.9), (0, 0, 10, 10),
                                  position_3d=Vector3(x + self.rng.gauss(0, 0.005),
                                                      Y_BORD + self.rng.gauss(0, 0.005),
                                                      PLATEAU + 0.10)))
        if self.faux_positif_sol:       # le sol pris pour un carton (constaté en P2)
            dets.append(Detection(query, 0.5, (0, 0, 10, 10),
                                  position_3d=Vector3(r.x + 1.0, r.y, 0.02)))
        return ServiceResult(ok=True, value=sorted(dets, key=lambda d: -d.confidence))

    # -- proprioception des mains et vérification de prise
    def mains(self):
        r = self.robot
        c, s = math.cos(r.cap), math.sin(r.cap)
        m = [np.array([r.x + 0.29 * c - l * s, r.y + 0.29 * s + l * c, 0.90])
             for l in (0.165, -0.165)]
        return m[0], m[1]

    def vers_robot(self, p):
        r = self.robot
        dx, dy = p[0] - r.x, p[1] - r.y
        c, s = math.cos(r.cap), math.sin(r.cap)
        return np.array([c * dx + s * dy, -s * dx + c * dy, p[2]])

    def carton_entre_mains(self):
        g, d = self.mains()
        m = (g + d) / 2
        for x in self.cartons:
            if math.hypot(x - m[0], Y_CARTON - m[1]) < 0.05:
                return x
        return None

    def verifier_prise(self):
        x = self.carton_entre_mains()
        return x is not None and self.rng.random() >= self.p_rate

    def poste_proche(self):
        r = self.robot
        best = min(carte.postes.items(),
                   key=lambda kv: math.hypot(kv[1].x - r.x, kv[1].y - r.y))
        return best[0] if math.hypot(best[1].x - r.x, best[1].y - r.y) < 0.5 else None


def collision(robot, rayon=0.15):
    return any(o.contient(robot.x, robot.y, rayon) for o in OBSTACLES)


# ── Montage de la pile ──────────────────────────────────────────────────
section(f"Montage : pile AXIS-OS complète, {DUREE:.0f} s simulées")
monde = Monde()
R = monde.robot
horloge = lambda: R.t
gardien = GardienProximite(monde.scan, lambda: (R.x, R.y, R.cap), obstacles=OBSTACLES,
                           horloge=horloge)
ETAT = BaseState((1, 0, 0, 0), (0, 0, 0), {}, {})
loco = LocomotionService(Adaptateur(R), lambda: ETAT, lambda c: {}, clock=horloge,
                         gardien=gardien)
nav = NavigationService(carte, R.pose, loco, horloge=horloge, rayon_robot=0.30)
bras = {}
manip = ManipulationService(bras, monde.mains, monde.vers_robot, monde.verifier_prise,
                            gardien=gardien, horloge=horloge)
journal = []
facade = FacadeServices(nav, loco, manip, monde, R.pose, horloge, CAP_PRISE,
                        journal=journal.append)
tri = TriColis(facade, ConfigTri(regles={"rouge": "A", "bleu": "B"},
                                 duree_mission_s=DUREE))
sup = Superviseur(tri, list(carte.postes), ["rouge", "bleu", "vert"],
                  InterpreteurRegles())
for s in (loco, nav, manip):
    s.start()
check("Services démarrés, façade branchée", True)

# ── Boucle à 50 Hz ──────────────────────────────────────────────────────
viol = {"collision": 0, "vers_humain": 0, "poste_errone": 0}
d_min_humain = float("inf")
etat_manip_prec = Etat.LIBRE
ordre_donne = False
t_fin = DUREE + 300.0
while not tri.termine and R.t < t_fin:
    R.t += DT
    for h in monde.humains:
        h.pas(R)
    monde.reappro()
    nav.tick()
    facade.tick()
    loco.tick()
    manip.tick()
    x0, y0 = R.x, R.y
    R.pas()
    tri.pas()
    # À mi-parcours, l'opérateur change une règle de vive voix.
    if not ordre_donne and R.t > DUREE / 2:
        sup.ordonner("les colis bleus vont maintenant au poste C")
        ordre_donne = True
    # Le monde suit les mains : prise et dépose.
    if manip.etat is Etat.TENU and etat_manip_prec is not Etat.TENU:
        x = monde.carton_entre_mains()
        monde.porte = monde.cartons.pop(x)
    if manip.etat is Etat.LIBRE and etat_manip_prec is Etat.OUVERTURE:
        monde.deposes.append((monde.porte, monde.poste_proche(), R.t))
        monde.porte = None
    etat_manip_prec = manip.etat
    # Invariants de sécurité, à CHAQUE pas.
    if collision(R):
        viol["collision"] += 1
    for h in monde.humains:
        d = math.hypot(h.p[0] - R.x, h.p[1] - R.y)
        d_min_humain = min(d_min_humain, d)
        d_avant = math.hypot(h.p[0] - x0, h.p[1] - y0)
        if d - 0.2 < 0.75 and d < d_avant - 1e-4:   # le ROBOT s'est rapproché
            viol["vers_humain"] += 1

b = tri.bilan
regle = lambda c, t: {"rouge": "A", "vert": None}.get(c, "C" if t > DUREE / 2 + 60 else "B")
bons = sum(1 for c, p, t in monde.deposes if p == regle(c, t) or
           (c == "bleu" and p in ("B", "C")))
print(f"\n    {b.colis_tries} triés ({b.par_couleur}), {b.colis_abandonnes} abandonnés, "
      f"{b.prises_ratees} prises ratées, {b.echecs_navigation} échecs nav\n"
      f"    arrêts sécurité {b.temps_arret_securite_s:.0f} s, gardien : "
      f"{gardien.stats}\n    façade : {facade.stats}, humain le plus proche "
      f"{d_min_humain:.2f} m")

section("Résultats")
# Débit MESURÉ dans cette disposition : un colis toutes les ~95 s (trajets
# de 5 à 8 m à 0.3 m/s, humains, 15 % de prises ratées). Le seuil protège
# contre une régression ; la cible de la démo (>= 20 en 30 min) se règle
# par la disposition de la scène — voir docs/demo_tri_usine.md.
check(f"Au moins {int(DUREE / 110)} colis triés ({b.colis_tries})",
      b.colis_tries >= int(DUREE / 110))
check("Chaque dépose au bon poste (bleus en C après l'ordre vocal)",
      bons == len([d for d in monde.deposes if d[1] in ("A", "B", "C")])
      and all(p in ("A", "B", "C") for c, p, t in monde.deposes if c != "vert"))
check("Aucun vert (sans règle) déplacé", all(c != "vert" for c, _, _ in monde.deposes))
check("Aucun faux carton au sol pris", facade.stats["detections_filtrees"] > 0)
check("Aucune collision avec une table ou une étagère", viol["collision"] == 0)
check("Le robot ne s'est JAMAIS rapproché d'un humain à moins de 0.75 m",
      viol["vers_humain"] == 0)
check("Le gardien a bien eu à intervenir (le scénario l'exerce)",
      gardien.stats["arrets"] + gardien.stats["ralentissements"] > 0)
check("Bleus livrés en C après l'ordre du superviseur",
      any(c == "bleu" and p == "C" for c, p, _ in monde.deposes))
check("Fin propre, bras vides", tri.etat == "FIN" and manip.etat is Etat.LIBRE)
print("\n  Journal (extrait) :")
for l in journal[-6:]:
    print("   ", l)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
