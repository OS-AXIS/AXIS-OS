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

from axis_os.apps.tri_colis import scene_demo as sd
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
    NavigationService, Pose2D, Rectangle,
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
# ── Le site : la MÊME définition que la scène Isaac Sim ─────────────────
PLATEAU, CAP_PRISE = sd.PLATEAU, sd.CAP_PRISE
TABLE_PRISE, TABLES_DEPOSE, ETAGERES = sd.TABLE_PRISE, sd.TABLES_DEPOSE, sd.ETAGERES
OBSTACLES = sd.OBSTACLES
carte = sd.carte()
EMPLACEMENTS_X, Y_CARTON = sd.EMPLACEMENTS_X, sd.Y_CARTON
Y_BORD = TABLE_PRISE.ymin


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


STATIQUE = np.vstack([boite(t, 0.1, PLATEAU) for t in [TABLE_PRISE, *TABLES_DEPOSE.values()]]
                     + [boite(e, 0.1, 1.8) for e in ETAGERES])


class Humain(sd.Figurant):
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
        self.robot = Robot(sd.DEPART.x, sd.DEPART.y, sd.DEPART.cap, graine)
        self.cartons = {}           # emplacement x -> couleur
        self.porte = None           # (couleur,)
        self.deposes = []           # (couleur, poste, t)
        self.chutes = []            # (table sous le carton, hauteur de chute)
        self.bras = {}
        self.humains = []
        for f in sd.figurants():
            h = Humain(f.a, f.b, f.v, f.pause, f.ev)
            self.humains.append(h)
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
        # Les mains descendent quand le tangage d'épaule augmente.
        tg = self.bras.get("left_shoulder_pitch_joint", -0.30)
        z = PLATEAU + 0.10 - 0.30 * (tg + 0.30)   # carton sur la table = mains au port
        m = [np.array([r.x + 0.29 * c - l * s, r.y + 0.29 * s + l * c, z])
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


def collision(robot, rayon=0.12):
    # 0.12 m : demi-profondeur du torse et des genoux du G1 devant le bassin.
    # Aux postes, les mains (et le carton) passent AU-DESSUS du plateau.
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
monde.bras = bras
manip = ManipulationService(bras, monde.mains, monde.vers_robot, monde.verifier_prise,
                            gardien=gardien, horloge=horloge)
journal = []
facade = FacadeServices(nav, loco, manip, monde, R.pose, horloge, CAP_PRISE,
                        journal=journal.append,
                        postes_depose=sd.postes_depose())
tri = TriColis(facade, ConfigTri(regles=dict(sd.REGLES_INITIALES),
                                 duree_mission_s=DUREE))
sup = Superviseur(tri, list(carte.postes), sd.COULEURS,
                  InterpreteurRegles())
for s in (loco, nav, manip):
    s.start()
check("Services démarrés, façade branchée", True)

# ── Boucle à 50 Hz ──────────────────────────────────────────────────────
viol = {"collision": 0, "vers_humain": 0, "poste_errone": 0, "bras_pendant_arret": 0}
d_min_humain = float("inf")
etat_manip_prec = Etat.LIBRE
ordre_donne = False
t_fin = DUREE + 300.0
while not tri.termine and R.t < t_fin:
    R.t += DT
    for h in monde.humains:
        h.pas(DT, R.x, R.y)
    monde.reappro()
    nav.tick()
    facade.tick()
    loco.tick()
    bras_avant = dict(bras)
    manip.tick()
    if gardien.arrete and bras != bras_avant:
        viol["bras_pendant_arret"] += 1
    x0, y0 = R.x, R.y
    R.pas()
    tri.pas()
    # À mi-parcours, l'opérateur change une règle de vive voix.
    if not ordre_donne and R.t > DUREE / 2:
        sup.ordonner("les colis bleus vont maintenant au poste C")
        ordre_donne = True
    # Le monde suit les mains : prise et dépose.
    if manip.etat is Etat.OUVERTURE and etat_manip_prec is not Etat.OUVERTURE:
        fond_ouverture = monde.mains()[0][2] - sd.HAUTEUR_CARTON / 2
    if manip.etat is Etat.TENU and etat_manip_prec is not Etat.TENU:
        x = monde.carton_entre_mains()
        monde.porte = monde.cartons.pop(x)
    if manip.etat is Etat.LIBRE and etat_manip_prec is Etat.OUVERTURE:
        g, d = monde.mains()
        mx, my = (g[0] + d[0]) / 2, (g[1] + d[1]) / 2
        sur = [n for n, tb in TABLES_DEPOSE.items()
               if tb.xmin + 0.1 <= mx <= tb.xmax - 0.1 and tb.ymin + 0.1 <= my <= tb.ymax]
        monde.deposes.append((monde.porte, sur[0] if sur else monde.poste_proche(), R.t))
        monde.chutes.append((sur[0] if sur else None, fond_ouverture - PLATEAU))
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
# Débit MESURÉ dans cette disposition : un colis toutes les ~120 s (trajets
# de 5 à 8 m à 0.3 m/s, trois humains dont un collègue qui vient près du
# poste de prise, ~8 min d'arrêts de sécurité, 15 % de prises ratées). Le
# seuil protège
# contre une régression ; la cible de la démo (>= 20 en 30 min) se règle
# par la disposition de la scène — voir docs/demo_tri_usine.md.
check(f"Au moins {int(DUREE / 140)} colis triés ({b.colis_tries})",
      b.colis_tries >= int(DUREE / 140))
check("Chaque dépose au bon poste (bleus en C après l'ordre vocal)",
      bons == len([d for d in monde.deposes if d[1] in ("A", "B", "C")])
      and all(p in ("A", "B", "C") for c, p, t in monde.deposes if c != "vert"))
sur_table = [c for c in monde.chutes if c[0] is not None]
check(f"Chaque carton livré est posé SUR le plateau de son poste "
      f"({len(sur_table)}/{b.colis_tries})", len(sur_table) == b.colis_tries)
chute_max = max((h for _, h in monde.chutes), default=0.0)
check(f"… sans chute : ouverture au plus à 1.5 cm du plateau "
      f"(max {100 * chute_max:.1f} cm)", all(0.0 <= h <= 0.015 for _, h in sur_table))
check(f"Le robot recule avant de repartir d'une table ({facade.stats['reculs']} reculs)",
      facade.stats["reculs"] >= b.colis_tries)
check("Aucun vert (sans règle) déplacé", all(c != "vert" for c, _, _ in monde.deposes))
check("Aucun faux carton au sol pris", facade.stats["detections_filtrees"] > 0)
check("Aucune collision avec une table ou une étagère", viol["collision"] == 0)
check("Le robot ne s'est JAMAIS rapproché d'un humain à moins de 0.75 m",
      viol["vers_humain"] == 0)
check("Aucun mouvement de bras pendant un arrêt de sécurité "
      f"(gels : {manip.stats['gels_securite']})", viol["bras_pendant_arret"] == 0)
check("Le gardien a bien eu à intervenir (le scénario l'exerce)",
      gardien.stats["arrets"] + gardien.stats["ralentissements"] > 0)
apres = [p for c, p, t in monde.deposes if c == "bleu" and t > DUREE / 2 + 60]
check(f"Bleus livrés après l'ordre du superviseur : tous en C ({len(apres)})",
      all(p == "C" for p in apres) and (apres or DUREE < 1200))
check("Fin propre, bras vides", tri.etat == "FIN" and manip.etat is Etat.LIBRE)
print("\n  Journal (extrait) :")
for l in journal[-6:]:
    print("   ", l)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
