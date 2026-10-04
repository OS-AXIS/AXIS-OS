"""
Service de navigation — planifier, suivre, et ne jamais toucher un mur.

Le robot simulé imite le G1 sous GR00T WBC : sous 0.05 m/s il n'avance
PAS (mode équilibre), et il exécute ses consignes avec une erreur de
vitesse et une légère dérive. Un test qui réussirait avec un robot
parfait ne prouverait rien.
"""
import math
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.services.base import IService
from axis_os.services.navigation.service import (
    Carte, EtatNavigation, NavigationService, Planificateur, Pose2D,
    Rectangle, Suiveur,
)

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


RAYON = 0.30


def entrepot():
    """8 m × 6 m, deux rangées d'étagères, un mur en L, deux postes."""
    return Carte(
        xmin=0.0, ymin=0.0, xmax=8.0, ymax=6.0,
        obstacles=[
            Rectangle(2.0, 0.0, 2.4, 4.0),            # étagère depuis le bas
            Rectangle(4.6, 2.0, 5.0, 6.0),            # étagère depuis le haut
            Rectangle(6.0, 1.0, 7.0, 1.4),            # table
        ],
        postes={
            "depart": Pose2D(1.0, 1.0, 0.0),
            "poste_A": Pose2D(6.5, 2.2, -math.pi / 2),  # face à la table
            "poste_B": Pose2D(3.5, 5.0, math.pi / 2),
        })


class RobotSimule:
    """Unicycle à 50 Hz, qui imite GR00T WBC."""

    def __init__(self, pose, erreur=0.10, derive=0.02, graine=0):
        self.x, self.y, self.cap = pose.x, pose.y, pose.cap
        self.t = 0.0
        self.cmd = (0.0, 0.0, 0.0)
        self.rng = random.Random(graine)
        self.erreur, self.derive = erreur, derive
        self.trace = []
        self.bande_morte = 0

    # --- l'interface du service de locomotion ---
    def set_velocity(self, vx=0.0, vy=0.0, wz=0.0):
        if 0 < abs(vx) < 0.06 or 0 < abs(wz) < 0.06:
            self.bande_morte += 1
        self.cmd = (vx, vy, wz)

    def halt(self):
        self.cmd = (0.0, 0.0, 0.0)

    # --- la physique ---
    def pas(self, dt=0.02):
        vx, vy, wz = self.cmd
        if math.sqrt(vx * vx + vy * vy + wz * wz) < 0.05:
            vx = vy = wz = 0.0           # la politique tient debout
        f = 1.0 + self.rng.uniform(-self.erreur, self.erreur)
        self.cap += wz * f * dt + self.rng.gauss(0, self.derive) * dt
        self.x += vx * f * math.cos(self.cap) * dt
        self.y += vx * f * math.sin(self.cap) * dt
        self.t += dt
        self.trace.append((self.x, self.y))

    def pose(self):
        return Pose2D(self.x, self.y, self.cap, horodatage=self.t)


def naviguer(carte, depart, but, graine=0, t_max=120.0, perturbation=None):
    r = RobotSimule(depart, graine=graine)
    nav = NavigationService(carte, r.pose, r, horloge=lambda: r.t)
    nav.start()
    res = nav.aller_vers(but)
    pousse = False
    while nav.etat is EtatNavigation.EN_ROUTE and r.t < t_max:
        nav.tick()
        r.pas()
        # Déclenchement par seuil, pas par égalité : un temps qui avance
        # par pas de 0.02 ne tombe jamais pile sur 6.0 en flottant.
        if perturbation and not pousse and r.t >= perturbation[0]:
            r.x += perturbation[1]
            r.y += perturbation[2]
            pousse = True
    return nav, r, res


def collision(carte, trace, rayon=RAYON):
    """Le CORPS du robot (disque de rayon `rayon`) a-t-il touché un obstacle
    ou un mur ? On vérifie les vrais obstacles, pas les obstacles élargis."""
    for x, y in trace:
        if any(o.contient(x, y, rayon) for o in carte.obstacles):
            return (x, y)
        if not (carte.xmin + rayon <= x <= carte.xmax - rayon
                and carte.ymin + rayon <= y <= carte.ymax - rayon):
            return (x, y)
    return None


section("0 — Contrat")
c = entrepot()
r0 = RobotSimule(c.postes["depart"])
nav0 = NavigationService(c, r0.pose, r0)
check("NavigationService est un IService", isinstance(nav0, IService))
check("Refuse d'agir avant start()", not nav0.aller_vers("poste_A").ok)
nav0.start()
check("Poste inconnu : refusé, avec la liste des postes existants",
      not nav0.aller_vers("poste_Z").ok and "poste_A" in nav0.aller_vers("poste_Z").reason)

section("1 — Planification : contourner, et garder ses distances")
p = Planificateur(c, RAYON, 0.10)
chemin, _ = p.planifier((1.0, 1.0), (6.5, 2.2))
check(f"Un chemin existe autour des étagères ({len(chemin)} points après lissage)",
      chemin is not None)
libre = all(p._segment_libre(chemin[k], chemin[k + 1]) for k in range(len(chemin) - 1))
check("Chaque segment du chemin reste à au moins rayon + marge (0.40 m) de "
      "tout obstacle", libre)
check("Le lissage supprime l'escalier de grille (moins de 12 points)",
      len(chemin) < 12)

ferme = Carte(0, 0, 6, 6, obstacles=[Rectangle(3.0, 3.0, 3.1, 5.0),
                                      Rectangle(4.9, 3.0, 5.0, 5.0),
                                      Rectangle(3.0, 3.0, 5.0, 3.1),
                                      Rectangle(3.0, 4.9, 5.0, 5.0)],
              postes={"enclos": Pose2D(4.0, 4.0)})
_, raison = Planificateur(ferme).planifier((1.0, 1.0), (4.0, 4.0))
check(f"Destination enfermée : AUCUN chemin, et la raison est dite "
      f"(« {raison[:45]}… »)", "chemin" in raison or "inatteignable" in raison)
_, raison2 = Planificateur(c).planifier((1.0, 1.0), (2.2, 2.0))
check("Destination DANS une étagère : refusée comme inatteignable",
      "inatteignable" in raison2)

section("2 — Suivi : arriver, sans jamais toucher un obstacle")
for graine in range(5):
    for but in ("poste_A", "poste_B"):
        nav, r, _ = naviguer(c, c.postes["depart"], but, graine=graine)
        cible = c.postes[but]
        d = math.hypot(r.x - cible.x, r.y - cible.y)
        e = abs(math.atan2(math.sin(r.cap - cible.cap), math.cos(r.cap - cible.cap)))
        choc = collision(c, r.trace)
        check(f"graine {graine}, {but} : arrivé à {d * 100:.1f} cm, cap à "
              f"{math.degrees(e):.1f}°, en {r.t:.1f} s, sans contact",
              nav.etat is EtatNavigation.ARRIVE and d < 0.12
              and e < 0.15 and choc is None)

section("3 — Jamais dans la bande morte de GR00T WBC")
nav, r, _ = naviguer(c, c.postes["depart"], "poste_A")
check(f"Aucune consigne entre 0 et 0.06 m/s ou rad/s ({r.bande_morte} "
      f"trouvée) — elle ferait tenir debout le robot au lieu d'avancer",
      r.bande_morte == 0)

section("4 — Hors du chemin : replanifier depuis la position réelle")
# Poussée PERPENDICULAIRE au couloir. Première version : poussé de 90 cm
# DANS le sens de sa marche, le robot restait sur son chemin — rien à
# replanifier, et le test échouait sans rien prouver.
nav, r, _ = naviguer(c, c.postes["depart"], "poste_A",
                     perturbation=(6.0, -0.8, 0.0))
check(f"Poussé de 80 cm sur le côté : {nav.stats['replanifications']} "
      f"replanification(s), et il arrive quand même",
      nav.stats["replanifications"] >= 1 and nav.etat is EtatNavigation.ARRIVE)
check("… sans contact malgré la poussée", collision(c, r.trace[400:]) is None)

section("5 — Position périmée : arrêt")
r5 = RobotSimule(c.postes["depart"])
fige = Pose2D(1.0, 1.0, 0.0, horodatage=0.0)
nav5 = NavigationService(c, lambda: fige, r5, horloge=lambda: r5.t)
nav5.start()
nav5.aller_vers("poste_A")
r5.t = 2.0
res = nav5.tick()
check("Pose vieille de 2 s : le service ARRÊTE le robot et dit pourquoi",
      not res.ok and r5.cmd == (0.0, 0.0, 0.0) and "périmée" in res.reason)

section("6 — Échec franc plutôt que foncer vers un mur")
r6 = RobotSimule(Pose2D(1.0, 1.0))
nav6 = NavigationService(ferme, r6.pose, r6, horloge=lambda: r6.t)
nav6.start()
res6 = nav6.aller_vers("enclos")
check("Destination inaccessible : ÉCHEC immédiat, robot à l'arrêt",
      not res6.ok and nav6.etat is EtatNavigation.ECHEC and r6.cmd == (0.0, 0.0, 0.0))

section("Alignement final : pas de va-et-vient quand pivoter fait dériver")
# Constaté sur GPU : en pivotant sur place, la marche déplace le robot de
# quelques cm ; il sortait de la tolérance, repartait, revenait — 90 s.
def aligner(sv, pas_max=6000):
    x, y, cap = 0.05, 0.0, 0.0
    for k in range(pas_max):
        c, ok = sv.commande(Pose2D(x, y, cap), [(0.0, 0.0)], math.pi)
        if ok:
            return k
        dt = 0.02
        cap += c.wz * dt
        x += (c.vx * math.cos(cap) - c.vy * math.sin(cap)) * dt
        y += (c.vx * math.sin(cap) + c.vy * math.cos(cap)) * dt
        if c.vx == 0.0 and c.wz != 0.0:       # pivot : la marche fait un pas de côté
            x += 0.0025 * math.cos(cap + math.pi / 2)
            y += 0.0025 * math.sin(cap + math.pi / 2)
    return None
k_arr = aligner(Suiveur())
check(f"Arrivée malgré la dérive en pivotant (pas {k_arr}, soit "
      f"{'jamais' if k_arr is None else f'{k_arr * 0.02:.0f} s'})",
      k_arr is not None and k_arr * 0.02 < 30)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
