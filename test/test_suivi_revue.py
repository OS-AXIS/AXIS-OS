"""
Suivi d'une personne : constats des revues adversariales, chacun avec son
essai. Horloge et locomotion factices : chaque essai est déterministe et
tourne en temps simulé, sans attendre.

A1 — Une personne habillée pareil, apparue à 1 m de la personne suivie
     pendant qu'elle était cachée, prenait son numéro : le robot la suivait
     sans rien dire (porte d'association en simple rayon, qui grandissait
     de 2 m par seconde sans mesure). Désormais la porte est statistique
     (Mahalanobis).
A2 — Pendant l'absence de la personne suivie, une personne qui lui
     ressemble passait là où elle pouvait être : à la reprise, rien ne
     permettait de savoir laquelle était revenue. Désormais : LOST.
A3 — Une reprise par l'apparence repartait aussitôt. Désormais le robot
     reste immobile une seconde (« verifying ») et rend LOST si une autre
     personne d'apparence proche est visible ; l'app voit la reprise.
A4 — Pose du robot non finie : le suivi commandait une petite rotation au
     lieu de s'arrêter.
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.services.locomotion.service import Twist  # noqa: E402
from axis_os.services.people.appearance import (  # noqa: E402
    appearance_descriptor, synthetic_person_image,
)
from axis_os.services.people.detector import FakePersonDetector, PersonDetection  # noqa: E402
from axis_os.services.people.follow import FollowService  # noqa: E402
from axis_os.services.people.service import PeopleService  # noqa: E402

FAIL = 0


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


rng = np.random.default_rng(3)
ROUGE, BLEU = (200, 30, 30), (30, 50, 200)


def apparence(torse):
    img = synthetic_person_image(torse, (30, 30, 30), noise=5.0, rng=rng)
    return tuple(float(v) for v in appearance_descriptor(img, (0, 0, 40, 100)))


class Loco:
    """Locomotion factice : garde chaque consigne."""
    is_guarded = True
    dernier_verdict = None
    state = None
    max_twist = Twist(0.4, 0.2, 0.8)

    def __init__(self):
        self.consignes = []

    def set_velocity(self, vx=0.0, vy=0.0, wz=0.0):
        self.consignes.append((vx, vy, wz))

    def halt(self):
        self.set_velocity(0.0, 0.0, 0.0)


class Gardien:
    def distance_arret(self):
        return 1.04


class Proprio:
    """Le contexte d'app qui suit (un RobotContext dans l'OS)."""


class Banc:
    """Service « personnes » et suivi, sur une horloge que l'essai avance."""

    def __init__(self):
        self.t, self.pose = 0.0, [0.0, 0.0, 0.0]
        self.det = FakePersonDetector()
        horloge = lambda: self.t                               # noqa: E731
        pose = lambda: tuple(self.pose)                        # noqa: E731
        self.people = PeopleService(self.det, source_pose=pose, horloge=horloge)
        self.loco = Loco()
        self.follow = FollowService(self.people, self.loco, Gardien(), source_pose=pose,
                                    horloge=horloge)
        self.people.start()
        self.follow.start()
        self.people.demander()
        self.owner = Proprio()
        self.phases = []

    def vues(self, *personnes):
        """Détections de ce cycle : (x, y, apparence)."""
        self.det.set([PersonDetection((0, 0, 40, 100), 0.9, (x, y, 0.0), a)
                      for x, y, a in personnes])

    def avancer(self, secondes, *personnes):
        """Fait tourner détection (10 Hz) et suivi (50 Hz) pendant
        `secondes`, avec ces personnes visibles."""
        self.vues(*personnes)
        for _ in range(int(round(secondes / 0.1))):
            self.people.tick()
            for _ in range(5):
                self.t += 0.02
                self.follow.battement(self.owner)
                self.follow.tick()
                self.phases.append(self.follow.status(self.owner).get("phase")
                                   or self.follow.status(self.owner)["status"])

    def suivre(self):
        cible = self.people.snapshots()[0]
        issue = self.follow.suivre(cible.track_id, distance_m=1.5, timeout_s=120.0,
                                   deadman_s=1.0, owner=self.owner, interruption=lambda: None)
        assert issue is None, issue
        return cible.track_id

    def statut(self):
        return self.follow.status(self.owner)


A_ROUGE, A_ROUGE2, A_BLEU = apparence(ROUGE), apparence(ROUGE), apparence(BLEU)

section("A1. Une personne habillée pareil surgit à 1 m de la personne cachée")
b = Banc()
b.avancer(1.0, (3.0, 0.0, A_ROUGE))
pid = b.suivre()
b.avancer(1.0, (3.0, 0.0, A_ROUGE))
b.avancer(0.3)                                   # cachée
debut = len(b.phases)
b.avancer(1.0, (3.0, 1.0, A_ROUGE2))             # le sosie, la vraie reste cachée
piste = b.people.track(pid)
check("Le numéro suivi n'a pas été donné au sosie (position restée à (3, 0))",
      piste is not None and math.hypot(piste.x - 3.0, piste.y) < 0.3, str(piste))
b.avancer(5.0, (3.0, 1.0, A_ROUGE2))
st = b.statut()
check("Le robot ne suit pas le sosie : il rend LOST", st["status"] == "LOST", str(st))
check("… sans jamais repasser en « following » entre-temps",
      "following" not in b.phases[debut:], str(sorted(set(b.phases[debut:]))))

section("A2. Un sosie passe là où la personne cachée pouvait être")
b = Banc()
b.avancer(1.0, (3.0, 0.0, A_ROUGE), (3.0, 3.0, A_ROUGE2))
cibles = sorted(b.people.snapshots(), key=lambda s: abs(s.y))
issue = b.follow.suivre(cibles[0].track_id, distance_m=1.5, timeout_s=120.0, deadman_s=1.0,
                        owner=b.owner, interruption=lambda: None)
b.avancer(0.4, (3.0, 0.0, A_ROUGE), (3.0, 3.0, A_ROUGE2))
for k in range(10):                              # cachée ; le sosie descend vers elle
    b.avancer(0.1, (3.0, 3.0 - 0.18 * (k + 1), A_ROUGE2))
b.avancer(1.0, (3.2, 0.0, A_ROUGE), (3.0, 1.2, A_ROUGE2))    # la vraie revient
st = b.statut()
check("À la reprise : LOST (identité incertaine), jamais un choix au hasard",
      st["status"] == "LOST" and "identité" in st["message"], str(st))
check("… doute compté", b.follow.stats["identity_doubts"] >= 1, str(b.follow.stats))

section("A3. Reprise par l'apparence : immobile une seconde, et l'app le sait")
b = Banc()
b.avancer(1.0, (3.5, 0.0, A_ROUGE))
pid = b.suivre()
b.avancer(0.6, (3.5, 0.0, A_ROUGE))
b.avancer(1.2)                                   # cachée (poteau)
n = len(b.loco.consignes)
b.avancer(0.6, (3.8, 0.2, A_ROUGE))              # revient
st = b.statut()
check("Phase « verifying » après la reprise", "verifying" in b.phases, str(set(b.phases)))
check("… robot immobile pendant la vérification",
      all(c == (0.0, 0.0, 0.0) for c in b.loco.consignes[n:]))
check("… la reprise est signalée à l'app (reacquisitions = 1)",
      st["status"] == "FOLLOWING" and st["reacquisitions"] == 1, str(st))
b.avancer(1.0, (3.8, 0.2, A_ROUGE))
check("Puis le suivi reprend la même personne", b.statut()["status"] == "FOLLOWING"
      and b.statut()["phase"] == "following" and b.statut()["person_id"] == pid, str(b.statut()))

b = Banc()
b.avancer(1.0, (3.5, 0.0, A_ROUGE))
b.suivre()
b.avancer(0.6, (3.5, 0.0, A_ROUGE))
b.avancer(1.2)
b.avancer(0.3, (3.8, 0.2, A_ROUGE))
b.avancer(0.6, (3.8, 0.2, A_ROUGE), (2.0, -2.5, A_ROUGE2))   # un sosie apparaît, loin
check("Un sosie visible pendant la vérification : LOST", b.statut()["status"] == "LOST",
      str(b.statut()))

b = Banc()
b.avancer(1.0, (3.5, 0.0, A_ROUGE))
b.suivre()
b.avancer(0.6, (3.5, 0.0, A_ROUGE))
b.avancer(1.2)
b.avancer(1.5, (3.8, 0.2, A_ROUGE), (2.0, -2.5, A_BLEU))     # une personne en bleu, loin
check("Une personne d'apparence différente ne gêne pas la reprise",
      b.statut()["status"] == "FOLLOWING", str(b.statut()))

section("A4. Pose du robot non finie : arrêt, jamais une rotation")
b = Banc()
b.avancer(1.0, (3.0, 0.5, A_ROUGE))
b.suivre()
b.pose[2] = float("nan")
n = len(b.loco.consignes)
b.avancer(0.5, (3.0, 0.5, A_ROUGE))
check("Aucune consigne non nulle avec une pose NaN",
      all(c == (0.0, 0.0, 0.0) for c in b.loco.consignes[n:]), str(b.loco.consignes[n:n + 3]))

section("Données : rien ne reste après le suivi")
b = Banc()
b.avancer(1.0, (3.0, 0.0, A_ROUGE), (2.0, 1.5, A_BLEU))
b.suivre()
b.avancer(0.5, (3.0, 0.0, A_ROUGE), (2.0, 1.5, A_BLEU))
issue = b.follow.arreter(b.owner)
check("Issue : numéro et compteurs seulement, aucune apparence",
      set(vars(issue)) == {"status", "code", "message", "person_id", "reacquisitions", "moved"})
check("Plus aucune piste (ni la personne suivie, ni les passants), plus de détection",
      len(b.people._tracker) == 0 and not b.people.watching and b.follow._reference is None)
calls = b.det.calls
b.avancer(1.0, (3.0, 0.0, A_ROUGE))
check("… et le service ne regarde plus tant qu'aucune app ne le demande",
      b.det.calls == calls and len(b.people._tracker) == 0)

section("B1. L'OS ne garde pas en vie le contexte d'une app terminée")
import gc  # noqa: E402
import weakref  # noqa: E402
b = Banc()
b.avancer(1.0, (3.0, 0.0, A_ROUGE))
b.suivre()
b.avancer(0.3, (3.0, 0.0, A_ROUGE))
b.follow.arreter(b.owner)
ref = weakref.ref(b.owner)
b.owner = None
gc.collect()
check("Après l'issue, le propriétaire du suivi peut être libéré", ref() is None)

print(f"\n{'=' * 66}\n  {'TOUT PASSE' if FAIL == 0 else f'{FAIL} ÉCHEC(S)'}\n{'=' * 66}")
sys.exit(1 if FAIL else 0)
