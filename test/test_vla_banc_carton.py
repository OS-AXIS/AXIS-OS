"""
Banc de collecte et d'évaluation VLA — l'opérateur simulé reprend le carton
AVANT le recul.

Constaté sur GPU (audit du 2026-10-10, I4/I5) :
  - après une prise réussie, le carton reposé dépassait parfois du bord de la
    table ; au recul, le gardien le voyait à moins de d_arret et arrêtait le
    robot, qui ne pouvait plus s'en éloigner : bloqué jusqu'au délai du
    retour (60 s) — probablement les ~1100 arrêts « sans personne » ;
  - une prise ratée a fait tomber le carton au sol : même blocage.

Le monde factice ci-dessous reproduit ce mécanisme sans GPU : tant qu'un
carton gênant (« débordant » ou « au sol ») est près du robot, le gardien
l'arrête — la navigation n'avance plus, et les bras sont figés.
Le gardien lui-même n'est pas modifié par cette correction.
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.apps.tri_colis.logique import Colis  # noqa: E402
from axis_os.services.policy.vla_service import VLAService  # noqa: E402
from axis_os.store.sdk.ai.gr00t_connector import GR00TAction  # noqa: E402
from axis_os.vla.collecte import CollecteurPrises  # noqa: E402
from axis_os.vla.evaluation import EvaluateurPrises, tirer_placements  # noqa: E402

FAIL = 0
DT = 0.02


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


class Monde:
    """Le carton : None (au stock), "table", "mains", "deborde", "sol".
    Le gardien arrête le robot tant qu'il est "deborde" ou "sol"."""

    def __init__(self, issue_pose="deborde"):
        self.t, self.carton, self.issue_pose = 0.0, None, issue_pose
        self.evenements = []
        self.arrets = 0

    def horloge(self):
        return self.t

    def gardien_arrete(self):
        return self.carton in ("deborde", "sol")

    def placer(self, *_):
        self.carton = "table"
        self.evenements.append("placer")

    def retirer(self):
        self.evenements.append(f"retirer({self.carton})")
        self.carton = None


class Facade:
    """Accostage 3 s, prise 2 s, pose 1 s, retour 3 s — mais la navigation
    N'AVANCE PAS tant que le gardien arrête le robot."""

    def __init__(self, m, prise_ratee_au_sol=False):
        self.m, self.ratee = m, prise_ratee_au_sol
        self.reste_nav, self.fin_manip, self.manip = None, None, "LIBRE"
        self.duree_retours, self._t_retour = [], None

    def chercher(self, couleur):
        return [Colis(couleur, (4.0, 2.1, 0.9), 0.5)]

    def accoster(self, colis):
        self.reste_nav = 3.0
        return True, ""

    def aller_au_poste(self, poste):
        self.m.evenements.append(f"recul(carton={self.m.carton})")
        self.reste_nav, self._t_retour = 3.0, self.m.t
        return True, ""

    def navigation(self):
        if self.reste_nav is None:
            return "EN_ROUTE"
        if not self.m.gardien_arrete():
            self.reste_nav -= DT
        else:
            self.m.arrets += 1
        if self.reste_nav <= 0:
            if self._t_retour is not None:
                self.duree_retours.append(self.m.t - self._t_retour)
                self._t_retour = None
            self.reste_nav = None
            return "ARRIVE"
        return "EN_ROUTE"

    def prendre(self, colis):
        self.fin_manip, self.manip = self.m.t + 2.0, "EN_COURS"
        return True, ""

    def manipulation(self):
        if self.manip == "EN_COURS" and self.m.t >= self.fin_manip:
            if self.ratee:
                self.manip, self.m.carton = "ECHEC", "sol"
            else:
                self.manip, self.m.carton = "TENU", "mains"
        if self.manip == "POSE" and self.m.t >= self.fin_manip:
            self.manip, self.m.carton = "LIBRE", self.m.issue_pose
        return self.manip

    def poser(self):
        self.manip, self.fin_manip = "POSE", self.m.t + 1.0
        return True, ""

    def reinitialiser_manipulation(self):
        self.manip = "LIBRE"

    def suspendre(self, au_contact=False):
        pass

    def reprendre_bras(self):
        pass


class Enregistreur:
    actif = False

    def veut_image(self, t):
        return False

    def debut(self, *a, **k):
        self.actif = True

    def fin(self, *a, **k):
        self.actif = False


def collecter(m, f, n=4):
    col = CollecteurPrises(f, Enregistreur(), placer=lambda rng: (m.placer() or "rouge", {}),
                           retirer=m.retirer, horloge=m.horloge,
                           etat_action=lambda: (np.zeros(14), np.zeros(17)),
                           image=lambda: None, n_episodes=n, journal=lambda s: None)
    while not col.termine and m.t < 2000:
        col.pas()
        m.t += DT
    return col


section("1 — Collecte : carton reposé en DÉPASSANT de la table")

m = Monde(issue_pose="deborde")
f = Facade(m)
col = collecter(m, f)
check(f"4 épisodes réussis ({col.stats})", col.termine and col.stats["reussis"] == 4)
check("Le carton est repris AVANT chaque recul (aucun recul avec un carton "
      "débordant derrière soi)",
      all("carton=None" in e for e in m.evenements if e.startswith("recul"))
      and "retirer(deborde)" in m.evenements)
check(f"Retours en ~3 s, pas au délai de 60 s ({[round(d, 1) for d in f.duree_retours]})",
      f.duree_retours and max(f.duree_retours) < 4.0)
check(f"Aucun arrêt de sécurité « sans personne » ({m.arrets})", m.arrets == 0)

section("2 — Collecte : prise ratée, carton tombé au sol")

m = Monde()
f = Facade(m, prise_ratee_au_sol=True)
col = collecter(m, f, n=3)
check(f"3 épisodes ratés, notés comme tels ({col.stats})",
      col.termine and col.stats["rates"] == 3)
check("Le carton au sol est ramassé avant le recul", "retirer(sol)" in m.evenements
      and all("carton=None" in e for e in m.evenements if e.startswith("recul")))
check(f"Pas de blocage d'une minute ({[round(d, 1) for d in f.duree_retours]})",
      max(f.duree_retours) < 4.0 and m.arrets == 0)

section("3 — Évaluation : mêmes garanties, réflexes et VLA")

ARMS = [f"left_j{i}" for i in range(7)] + [f"right_j{i}" for i in range(7)]


class Connecteur:
    arm_joints = ARMS

    def is_connected(self):
        return True

    def reset(self):
        pass

    def predict_action(self, image, instruction, state):
        return GR00TAction(target_angles={j: 0.1 for j in ARMS})


class Loco:
    def set_velocity(self, *a):
        pass

    def halt(self):
        pass


def evaluer(m, f, modes, au_sol_apres=None, t_max=400.0, chute_en_ouverture=False):
    bras = {j: 0.0 for j in ARMS}
    vla = VLAService(Connecteur(), bras, Loco(), lire_etat_bras=lambda: dict(bras),
                     lire_image=lambda: "image", delta_max_par_joint={j: 0.04 for j in ARMS},
                     horloge=m.horloge, arret_securite=m.gardien_arrete, journal=lambda s: None)
    vla.start()
    ev = EvaluateurPrises(
        tirer_placements(2, 1000), modes, placer=lambda c, meta: m.placer(),
        retirer=m.retirer, horloge=m.horloge, facade=f, vla=vla, bras=bras,
        carton_souleve=lambda: m.carton == "mains", carton_au_sol=lambda: m.carton == "sol",
        arret_securite=m.gardien_arrete, halt=lambda: None,
        compteurs=lambda: {"refus": 0, "arrets": m.arrets}, journal=lambda s: None,
        duree_max_s=20.0, roulis=("left_j1", "right_j1"))
    while not ev.termine and m.t < t_max:
        if au_sol_apres is not None and ev.etat == "ESSAI" and m.t > au_sol_apres(ev) \
                and m.carton == "table":
            m.carton = "sol"                     # le VLA a fait tomber le carton
        if chute_en_ouverture and ev.etat == "LIBERER" and ev.sous_etat == "OUVRIR" \
                and m.carton == "table":
            m.carton = "sol"                     # lâché en ouvrant les mains
        ev.pas()
        m.t += DT
    return ev


m = Monde(issue_pose="deborde")
f = Facade(m)
ev = evaluer(m, f, ("reflexes",))
check(f"Réflexes, carton reposé débordant : 2 essais terminés "
      f"({[r['succes'] for r in ev.resultats]})", ev.termine and len(ev.resultats) == 2)
check(f"Repris avant le recul, retours en ~3 s ({[round(d, 1) for d in f.duree_retours]})",
      all("carton=None" in e for e in m.evenements if e.startswith("recul"))
      and max(f.duree_retours) < 4.0 and m.arrets == 0)

m = Monde()
f = Facade(m)
debut = {}


def apres(ev_):
    return debut.setdefault(ev_.k, m.t) + 0.5


ev = evaluer(m, f, ("vla",), au_sol_apres=apres, t_max=200.0)
check(f"VLA, carton tombé au sol : l'évaluation se TERMINE ({ev.termine}, "
      f"t = {m.t:.0f} s) — avant : bras figés par l'arrêt de sécurité, sans fin",
      ev.termine and m.t < 60.0)
check("Essais notés « carton tombé au sol »",
      [r["raison"] for r in ev.resultats] == ["carton tombé au sol"] * 2)
check("Carton ramassé dès sa chute, avant la libération des bras",
      m.evenements.count("retirer(sol)") == 2)

# Trouvé à la revue : chute PENDANT la libération des bras (ouverture), après
# un essai raté. Le gardien arrêtait le robot, les bras restaient figés et le
# délai de libération était suspendu : évaluation bloquée sans fin.
m = Monde()
f = Facade(m)
ev = evaluer(m, f, ("vla",), chute_en_ouverture=True, t_max=300.0)
check(f"VLA, carton lâché au sol en ouvrant les mains : l'évaluation se TERMINE "
      f"({ev.termine}, t = {m.t:.0f} s)", ev.termine and m.t < 120.0)
check("Carton ramassé dès sa chute pendant la libération",
      m.evenements.count("retirer(sol)") == 2)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
