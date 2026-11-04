"""
Accostage ralenti par le gardien sous la vitesse minimale de marche :
attente de sécurité, pas blocage.

Constat (pile logicielle, test_tri_pile_logicielle.py sur 12 graines, main) :
13 reculs « BLOQUÉ » et 2 graines en échec. Le gardien de proximité
multiplie la consigne par un facteur (×0.52 à ×0.88 pour quelqu'un à 1.4–
1.9 m) ; une consigne déjà à la vitesse minimale de marche (0.06 m/s)
tombait sous la bande morte de la marche, le robot ne bougeait plus, et
l'accostage se déclarait bloqué au bout de 2 s.

Désormais (axis_os/apps/tri_colis/facade.py, ConfigFacade) : attente de
sécurité, robot à l'arrêt, délai suspendu, jamais de consigne remontée pour
passer ; au-delà de attente_securite_max_s, l'accostage échoue avec la
raison « attente de sécurité trop longue : personne à X m » et l'app le
signale. GardienProximite n'est pas touché. Le seuil est la bande morte
que déclare l'adaptateur de locomotion (ILocomotionAdapter.bande_morte), ou
0.06 m/s s'il ne la déclare pas (section 5).

Monde factice déterministe : une base cinématique qui applique, comme le
LocomotionService, le facteur du gardien à la consigne, avec la bande morte
de la marche (0.05 m/s) du robot de test_tri_pile_logicielle.py.
"""
import math
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.apps.tri_colis.facade import ConfigFacade, FacadeServices  # noqa: E402
from axis_os.apps.tri_colis.logique import ConfigTri, TriColis  # noqa: E402
from axis_os.services.locomotion.proximite import Verdict  # noqa: E402
from axis_os.services.manipulation.service import Etat  # noqa: E402
from axis_os.services.navigation.service import Carte, EtatNavigation, Pose2D  # noqa: E402

FAIL = 0
DT = 0.02
BANDE_MORTE = 0.05


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


class Monde:
    """Base cinématique, gardien factice (facteur imposé par le test),
    navigation et bras réduits à ce que la façade en lit."""

    def __init__(self, bloquee=False, bande_morte=None):
        self.t, self.x, self.y, self.cap = 0.0, 0.0, 0.0, 0.0
        self.cmd = (0.0, 0.0, 0.0)
        self.consignes = []            # (t, vx, vy, wz) demandées par la façade
        self.facteur, self.distance = 1.0, 1.8
        self.bloquee = bloquee         # quelque chose retient vraiment le robot
        self.bande_morte = bande_morte # déclarée par l'« adaptateur », ou None
        self.dernier_verdict = None
        self.journal, self.instants = [], []     # lignes du journal, et leur instant
        self.etat = EtatNavigation.ARRIVE
        self.carte = Carte(-5.0, -5.0, 5.0, 5.0, postes={"prise": Pose2D(0.0, 0.0, 0.0)})
        self.manip = SimpleNamespace(etat=Etat.LIBRE, reinitialiser=lambda *a, **k: None)
        self.percep = SimpleNamespace(find=lambda *a, **k: SimpleNamespace(ok=False, value=None))

    def noter(self, ligne):
        self.journal.append(ligne)
        self.instants.append(self.t)

    def debut_attente(self):
        """Instant où la façade a signalé l'attente de sécurité."""
        return next((t for t, l in zip(self.instants, self.journal)
                     if "attente de sécurité — le gardien" in l), None)

    # Locomotion (ce que la façade appelle)
    def set_velocity(self, vx, vy, wz):
        self.cmd = (vx, vy, wz)
        self.consignes.append((self.t, vx, vy, wz))

    def halt(self):
        self.cmd = (0.0, 0.0, 0.0)

    # Navigation
    def aller_vers(self, dest):
        return SimpleNamespace(ok=True, reason=None)

    def annuler(self):
        pass

    def pose(self):
        return Pose2D(self.x, self.y, self.cap)

    def pas(self):
        """Un pas de commande : verdict du gardien, consigne multipliée par
        son facteur (comme LocomotionService), bande morte de la marche."""
        self.t += DT
        f = self.facteur
        self.dernier_verdict = Verdict(f, self.distance if f < 1.0 else math.inf,
                                       f"obstacle à {self.distance:.2f} m — vitesse ×{f:.2f}"
                                       if f < 1.0 else "zone dégagée")
        vx, vy, wz = (c * f for c in self.cmd) if f > 0.0 else (0.0, 0.0, 0.0)
        if self.bloquee or math.sqrt(vx * vx + vy * vy + wz * wz) < BANDE_MORTE:
            return
        c, s = math.cos(self.cap), math.sin(self.cap)
        self.x += (vx * c - vy * s) * DT
        self.y += (vx * s + vy * c) * DT
        self.cap += wz * DT


def facade(m, cfg=ConfigFacade()):
    f = FacadeServices(m, m, m.manip, m.percep, m.pose, lambda: m.t, 0.0,
                       journal=m.noter, cfg=cfg)
    f._accoste = True           # collé à une table : la sortie commence par un recul
    return f


def reculer(m, f, facteur, de=0.5, a=math.inf, duree=60.0):
    """Recul de sortie (0,35 m) ; le gardien impose `facteur` entre `de` et
    `a` secondes. Rend l'état de navigation final et son instant."""
    ok, _ = f.aller_au_poste("prise")
    assert ok
    while m.t < duree:
        m.facteur = facteur if de <= m.t < a else 1.0
        m.pas()
        f.tick()
        e = f.navigation()
        if e in ("ARRIVE", "ECHEC"):
            return e, m.t
    return "EN_ROUTE", m.t


section("1. Ralenti ×0.8 pendant 10 s, au début du recul (personne à 1.8 m)")
m = Monde()
f = facade(m)
etat, t_fin = reculer(m, f, 0.8, de=0.5, a=10.5)
check("Pas de « BLOQUÉ » : le recul attend que le gardien relâche, puis s'achève",
      etat == "ARRIVE" and not any("BLOQUÉ" in l for l in m.journal), f"{etat} {m.journal}")
check("… attente signalée au journal, une fois, avec le facteur et la distance",
      sum("attente de sécurité — le gardien ralentit le robot (×0.80, personne à 1.8 m)" in l
          for l in m.journal) == 1, str(m.journal))
V_MIN = ConfigFacade().v_min
sous = [c for c in m.consignes if 0.5 + DT < c[0] < 10.5
        and max(abs(c[1]), abs(c[2]), abs(c[3])) * 0.8 < V_MIN]
debut = m.debut_attente()
check("… le robot recule tant que la consigne ralentie reste au-dessus de la vitesse minimale ; "
      "en dessous, il s'arrête : aucune consigne qui, ralentie, tomberait sous elle",
      not sous and debut is not None and m.x < -0.25, f"{sous[:3]} {debut} {m.x:.3f}")
check("… et aucune consigne du tout pendant l'attente (robot à l'arrêt, en équilibre)",
      debut is not None and not [c for c in m.consignes if debut < c[0] < 10.5],
      str([c for c in m.consignes if debut and debut < c[0] < 10.5][:3]))
check("… jamais une consigne remontée pour passer : chacune au plus v_max",
      all(max(abs(c[1]), abs(c[2])) <= ConfigFacade().v_max + 1e-9 and abs(c[3])
          <= ConfigFacade().w_max + 1e-9 for c in m.consignes), str(m.consignes[:3]))
check("… recul fini après la fin du ralentissement (vers 10.5 s + 6 s de marche)",
      10.5 < t_fin < 20.0, f"{t_fin:.1f} s")

section("2. Ralenti sans fin : attente bornée, raison donnée")
m = Monde()
f = facade(m)
etat, t_fin = reculer(m, f, 0.8)
cfg = ConfigFacade()
attendu = None if m.debut_attente() is None else t_fin - m.debut_attente()
check(f"Échec après {cfg.attente_securite_max_s:.0f} s d'attente (et non 2 s, ni le délai "
      f"d'accostage de {cfg.delai_accostage_s:.0f} s)", etat == "ECHEC" and attendu is not None
      and cfg.attente_securite_max_s - DT < attendu < cfg.attente_securite_max_s + 0.1,
      f"{etat} {attendu}")
check("… raison : « attente de sécurité trop longue : personne à 1.8 m »",
      f.raison_navigation() == "attente de sécurité trop longue : personne à 1.8 m",
      f.raison_navigation())
check("… journal : abandon, durée et limite",
      any("attente de sécurité trop longue : personne à 1.8 m (30 s, au plus 30) — abandon"
          in l for l in m.journal), str(m.journal[-2:]))

section("3. L'app le signale à l'opérateur, puis réessaie")
m = Monde()
f = facade(m)
tri = TriColis(f, ConfigTri(regles={"bleu": "A"}, duree_mission_s=1e6))
m.facteur, m.distance = 0.7, 1.6
while m.t < 80.0 and not tri.bilan.alertes:
    m.pas()
    f.tick()
    tri.pas()
check("Alerte : « attente de sécurité trop longue : personne à 1.6 m — trajet vers prise "
      "abandonné, nouvel essai »", tri.bilan.alertes == [
          "attente de sécurité trop longue : personne à 1.6 m — trajet vers prise abandonné, "
          "nouvel essai"], str(tri.bilan.alertes))
check("… compté comme un échec de navigation, la mission continue (nouvel essai)",
      tri.bilan.echecs_navigation == 1 and tri.etat == "REVENIR" and not tri.termine,
      f"{tri.bilan.echecs_navigation} {tri.etat}")

section("4. Ce qui ne change pas")
m = Monde()
f = facade(m)
etat, t_fin = reculer(m, f, 0.0, de=0.5, a=45.0, duree=40.0)
check("Arrêt du gardien (facteur 0) pendant 40 s : on attend toujours, sans limite (l'app "
      "alerte à 60 s, inchangé)", etat == "EN_ROUTE" and not f.raison_navigation()
      and not any("BLOQUÉ" in l for l in m.journal), f"{etat} {m.journal}")
m = Monde(bloquee=True)
f = facade(m)
etat, t_fin = reculer(m, f, 1.0, duree=10.0)
check("Vraiment retenu, gardien à pleine vitesse : « BLOQUÉ » après 2 s, comme avant",
      etat == "ECHEC" and any("accostage BLOQUÉ (RECUL)" in l for l in m.journal)
      and t_fin < 3.0, f"{etat} {t_fin:.1f} {m.journal}")
m = Monde()
f = facade(m)
x_avant = []


def noter():
    if 5.8 < m.t < 5.9 and not x_avant:
        x_avant.append(m.x)


ok, _ = f.aller_au_poste("prise")
while m.t < 30.0 and f.navigation() not in ("ARRIVE", "ECHEC"):
    m.facteur = 0.9 if 0.5 <= m.t < 6.0 else 1.0
    m.pas()
    f.tick()
    noter()
check("Sans bande morte déclarée (seuil 0.06 m/s), ralenti ×0.9 : le robot recule tant que la "
      "consigne ralentie reste au-dessus de 0.06 (29 cm sur 35 au moins), attend pour les "
      "derniers centimètres, puis finit", f.navigation() == "ARRIVE" and x_avant and x_avant[0] < -0.29
      and not any("BLOQUÉ" in l for l in m.journal), f"{f.navigation()} {x_avant} {m.journal}")

section("5. Bande morte déclarée par l'adaptateur (0.05 m/s, celle de cette base)")
m = Monde(bande_morte=BANDE_MORTE)
f = facade(m)
etat, t_fin = reculer(m, f, 0.9, de=0.0)
mini = min(max(abs(c[1]), abs(c[2]), abs(c[3])) for c in m.consignes)
check("Ralenti ×0.9 tout le recul : près de la cible, la consigne ralentie (0.054 m/s) "
      "reste au-dessus de la bande morte, le robot avance encore et arrive, sans attente",
      etat == "ARRIVE" and not any("attente" in l for l in m.journal) and t_fin < 10.0,
      f"{etat} {t_fin:.1f} {m.journal}")
check("… consigne ralentie envoyée telle quelle, jamais remontée : la plus petite vaut le "
      "plancher de la façade (0.06), aucune au-dessus de v_max",
      abs(mini - ConfigFacade().v_min) < 1e-9 and all(
          max(abs(c[1]), abs(c[2])) <= ConfigFacade().v_max + 1e-9 for c in m.consignes),
      f"{mini}")
m = Monde(bande_morte=BANDE_MORTE)
f = facade(m)
etat, t_fin = reculer(m, f, 0.8, de=0.0, a=10.0)
check("Ralenti ×0.8 : près de la cible, 0.048 m/s, sous la bande morte : attente, puis "
      "le recul finit quand le gardien relâche", etat == "ARRIVE" and t_fin > 10.0
      and sum("attente de sécurité — le gardien" in l and "bande morte de la marche "
              "(0.05 m/s)" in l for l in m.journal) == 1, f"{etat} {t_fin:.1f} {m.journal}")

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
