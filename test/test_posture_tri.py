"""
L'app de tri se penche et plie les jambes, dans les limites validées au
banc de posture (lancer_posture.sh, essai GPU du 20 novembre 2026) : torse
0,35 rad au plus demandé, bassin 10 cm au plus, marge d'équilibre 2 cm au
moins ; les jambes d'abord pour descendre, le torse pour aller plus loin ;
la prise calculée sur la taille MESURÉE. Avant, le penché était interdit à
T1 (test_banc_posture §6, remplacé par celui-ci).

  1. Les limites : celles de la marche (BornesPosture, inchangées), que
     toute consigne de l'app traverse (LocomotionService.posture : bornes,
     gardien de proximité, équilibre) ; aucune autre voie vers la posture.
  2. Robot simulé léger, G1 29 DOF, mains du G1 :
     - carton sur une table basse (66 cm) : les jambes pliées pour le
       prendre (mains à sa hauteur), puis dépliées le carton tenu ;
     - trop bas (56 cm) : refusé avec la raison ;
     - dépose sur un plateau de 64 cm (sous les 68 cm qu'atteignent les bras
       debout) : les jambes pliées d'abord, posé à 1 cm près, puis droit ;
     - 56 cm : refusé avant de bouger, la raison dite, le carton tenu ;
     - le planificateur essaie les jambes puis le torse quand rien ne passe
       debout.
  3. La façade : la posture appliquée une fois la base arrêtée à la pose de
     prise, ARRIVE seulement une fois atteinte ; on ne repart que droit
     (redressement, objet tenu en place) ; plateau bas : jambes pliées, puis
     dépose.
"""
import contextlib
import io
import math
import re
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.apps.tri_colis.facade import ConfigFacade, FacadeServices  # noqa: E402
from axis_os.services.base import ServiceResult  # noqa: E402
from axis_os.services.locomotion.service import BornesPosture  # noqa: E402
from axis_os.services.manipulation.service import (  # noqa: E402
    Etat, ManipulationService, Objet,
)
from axis_os.services.navigation.service import EtatNavigation, Pose2D  # noqa: E402
from axis_os.testing import SimulatedObject, SimulatedRobot  # noqa: E402

FAIL = 0
RACINE = Path(__file__).resolve().parents[1]
B = BornesPosture()


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


section("1. Les limites du banc, et une seule voie vers la posture")
check("BornesPosture : torse 0,35 rad au plus, bassin 10 cm au plus, marge d'équilibre 2 cm au "
      "moins (validées au banc de posture)",
      (B.tangage_max_rad, B.abaissement_max_m, B.marge_min_m) == (0.35, 0.10, 0.02))
essayes = list(ManipulationService.TANGAGES_ESSAYES) + [
    t for t, _ in ManipulationService.POSTURES_PLAN]
abaisses = list(ManipulationService.ABAISSEMENTS_ESSAYES) + [
    a for _, a in ManipulationService.POSTURES_PLAN]
check("Ce que l'OS essaie reste dans ces bornes : torse ≤ 0,35 rad, bassin ≤ 10 cm",
      max(essayes) <= B.tangage_max_rad and max(abaisses) <= B.abaissement_max_m
      and min(essayes + abaisses) >= 0.0)
SOURCES = sorted((RACINE / "axis_os/apps/tri_colis").glob("*.py")) + [
    RACINE / "sim/validation/test_isaac_t1_tri_usine.py"]
appels = {f.name: re.findall(r"(\w+(?:\.\w+)*)\.posture\(", f.read_text(encoding="utf-8"))
          for f in SOURCES}
check("App de tri et scène T1 : la posture seulement par self.loco.posture (LocomotionService : "
      "bornes, gardien, équilibre) ou les gestes de la manipulation (changer_tangage, "
      "changer_abaissement), jamais par l'adaptateur",
      all(a == "self.loco" for v in appels.values() for a in v)
      and "self.loco.posture(" in (RACINE / "axis_os/apps/tri_colis/facade.py").read_text(
          encoding="utf-8"), str(appels))


class Banc:
    def __init__(self, objets=(), plateaux=(), contact=0.11):
        self.t = [0.0]
        with contextlib.redirect_stdout(io.StringIO()):
            self.r = SimulatedRobot(objects=list(objets), horloge=lambda: self.t[0],
                                    poignets_29dof=True, contact_main_m=contact,
                                    plateaux=list(plateaux))
            self.r.start(fil=False)
        self.m = self.r.manipulation
        self.journal = []
        self.m._journal = self.journal.append
        self.pas(20)

    def pas(self, n=1):
        with contextlib.redirect_stdout(io.StringIO()):
            for _ in range(n):
                self.t[0] += self.r._dt
                self.r.pas()

    def jusqua(self, fini, duree):
        t0 = self.t[0]
        while not fini() and self.t[0] - t0 < duree:
            self.pas()


section("2. Robot simulé : les jambes d'abord pour descendre, le torse pour aller plus loin")
for table, attendu in ((0.66, True), (0.56, False)):
    zc = table + 0.10
    b = Banc([SimulatedObject("carton", (0.40, 0.0, zc))], [(0.30, 0.90, -0.6, 0.6, table)])
    objet = Objet((0.40, 0.0, zc), 0.24, 0.20, 0.20, bord_m=0.0)
    d, t, a, raison = b.m.plan_posture(objet)
    if not attendu:
        check(f"Carton sur une table de {100 * table:.0f} cm : refusé, « trop bas », la raison "
              f"dite ; rien ne bouge", d is None and "trop bas" in raison
              and not b.r.locomotion.penche, raison)
        continue
    check(f"Carton sur une table de {100 * table:.0f} cm (mains du port plus haut que lui) : "
          f"jambes pliées de {100 * a:.1f} cm, torse droit, marge d'équilibre écrite",
          d is not None and t == 0.0 and 0.0 < a <= B.abaissement_max_m
          and "cm dans l'appui des pieds" in raison, raison)
    r = b.r.locomotion.posture(t, a)
    b.jusqua(lambda: b.r.locomotion.posture_atteinte, 10.0)
    b.r.x = 0.40 - d
    b.pas(5)
    res = b.m.prendre(objet)
    b.jusqua(lambda: b.m.etat in (Etat.TENU, Etat.ECHEC), 40.0)
    check("… pris et tenu, jambes pliées", res.ok and b.m.etat is Etat.TENU
          and b.r.held is not None, f"{res} {b.m.raison}")
    # La taille lue à 0,05° (bruit de la mesure) : la consigne du torse reste
    # droite, sinon la marche le croirait penché et ne partirait jamais.
    lue = b.m._articulations_fixes
    b.m._articulations_fixes = lambda: {**lue(), b.m.TAILLE: 0.0008}
    red = b.m.changer_abaissement(0.0)
    b.m._articulations_fixes = lue
    b.jusqua(lambda: b.r.locomotion.posture_atteinte, 10.0)
    check("… jambes dépliées le carton tenu (il monte avec le bassin, les bras ne bougent pas), "
          "taille lue à 0,05° : droit, prêt à marcher", red.ok and not b.r.locomotion.penche
          and b.r.held is not None and b.m.etat is Etat.TENU, f"{red} {b.m.raison}")

for plateau, attendu in ((0.64, True), (0.56, False)):
    b = Banc([SimulatedObject("carton", (0.30, 0.0, 0.90))], [(0.15, 0.70, -0.6, 0.6, plateau)],
             contact=0.0)
    b.m.prendre(Objet((0.30, 0.0, 0.90), 0.24, 0.20, 0.20))
    b.jusqua(lambda: b.m.etat in (Etat.TENU, Etat.ECHEC), 40.0)
    a, raison = b.m.abaissement_pour_deposer(plateau)
    if not attendu:
        check(f"Dépose sur un plateau de {100 * plateau:.0f} cm : refusée avant de bouger, la "
              f"raison dite, le carton tenu, robot droit",
              a is None and "dépose impossible" in raison and b.m.etat is Etat.TENU
              and b.r.held is not None and not b.r.locomotion.penche, raison)
        continue
    debout = b.m._depose_possible(plateau)[0] is None
    check(f"Dépose sur un plateau de {100 * plateau:.0f} cm, hors d'atteinte des bras debout : "
          f"jambes pliées de {100 * a:.1f} cm d'abord, marge d'équilibre écrite",
          debout and a is not None and 0.0 < a <= B.abaissement_max_m
          and "cm dans l'appui des pieds" in raison, f"{a} {raison}")
    b.m.changer_abaissement(a)
    b.jusqua(lambda: b.r.locomotion.posture_atteinte, 10.0)
    p = b.m.poser(plateau)
    b.jusqua(lambda: b.m.etat is Etat.LIBRE, 30.0)
    fond = b.r.objects[0].position[2] - 0.10
    check(f"… posé, lâché à {100 * (fond - plateau):.1f} cm au-dessus du plateau (au plus 1), "
          f"aucune chute", p.ok and b.m.etat is Etat.LIBRE and b.r.held is None
          and -0.005 <= fond - plateau <= 0.01 and not b.r.chutes, f"{p} {fond} {b.r.chutes}")
    b.r.locomotion.posture(0.0, 0.0)
    b.jusqua(lambda: b.r.locomotion.posture_atteinte, 10.0)
    check("… puis droit (mains vides)", not b.r.locomotion.penche)

section("3. La façade : posture à la prise, droit pour marcher, jambes pour un plateau bas")


class Loco:
    """Ce que la façade appelle de la marche ; la posture, enregistrée, est
    atteinte quand le test le dit."""

    def __init__(self):
        self.consignes, self.cible, self.atteinte = [], (0.0, 0.0), True

    def posture(self, tangage=0.0, abaissement=0.0, **_):
        self.consignes.append((tangage, abaissement))
        self.cible, self.atteinte = (tangage, abaissement), False
        return ServiceResult(ok=True, reason="posture : essai")

    @property
    def penche(self):
        # Comme LocomotionService : hors de la posture droite, consigne
        # retenue ou en cours.
        return max(self.cible) > 1e-4 or not self.atteinte

    @property
    def posture_atteinte(self):
        return self.atteinte

    def halt(self):
        pass

    def set_velocity(self, *a):
        pass

    dernier_verdict, bande_morte = None, 0.05


class Manip:
    """Le service de manipulation, réduit à ce que la façade appelle."""

    def __init__(self, loco):
        self.loco, self.etat, self.mesures, self._rigide = loco, Etat.LIBRE, object(), None
        self.appels, self.tangage_taille, self.tangage_en_cours, self.penche = [], 0.0, False, False
        self.a_reposer, self.raison = False, ""

    def distance_de_prise(self, objet, d_min=0.25, **_):
        return None, "objet hors de portée des bras à toute distance"

    def plan_posture(self, objet, d_min=0.25, **_):
        return 0.40, 0.10, 0.0, "penché de 6° : posture : …"

    def changer_tangage(self, t):
        self.appels.append(("changer_tangage", t))
        self.loco.cible, self.loco.atteinte = (t, 0.0), False
        return ServiceResult(ok=True)

    def changer_abaissement(self, a):
        self.appels.append(("changer_abaissement", a))
        self.loco.cible, self.loco.atteinte = (self.loco.cible[0], a), False
        return ServiceResult(ok=True)

    def abaissement_pour_deposer(self, h):
        return 0.05, "jambes pliées, bassin abaissé de 5.0 cm : …"

    def poser(self, hauteur_surface=None):
        self.appels.append(("poser", hauteur_surface))
        return ServiceResult(ok=True)

    def reinitialiser(self):
        pass


def facade():
    loco = Loco()
    manip = Manip(loco)
    nav = SimpleNamespace(etat=EtatNavigation.ARRIVE, carte=SimpleNamespace(postes={}),
                          annuler=lambda: None, aller_vers=lambda dest: ServiceResult(ok=True))
    temps = [0.0]
    journal = []
    f = FacadeServices(nav, loco, manip, SimpleNamespace(), lambda: Pose2D(4.0, 1.0, 0.0),
                       lambda: temps[0], 0.0, journal=journal.append, cfg=ConfigFacade())
    return f, loco, manip, temps, journal


f, loco, manip, temps, journal = facade()
cible, raison = f._cible_prise((4.40, 1.0, 0.90))
check("Hors de portée debout : la façade prend la posture du plan (plan_posture : torse 0,10 rad), "
      "dite au journal", cible is not None and f._posture_prise == (0.10, 0.0)
      and any(l.startswith("prise : penché") for l in journal), f"{cible} {raison} {journal}")
f._phase_avant_stab, f._apres_stab, f._phase = "INSERTION", "ARRIVE", "STABILISATION"
f._etat_accostage = "EN_ROUTE"
f._t_stab0, f._ref_stab = -100.0, (-100.0, 4.0, 1.0)
temps[0] = 1.0
f.tick()
check("Base arrêtée à la pose de prise : la posture demandée à la marche (bornes, gardien, "
      "équilibre), pas encore ARRIVE", loco.consignes == [(0.10, 0.0)]
      and f.navigation() == "EN_ROUTE" and f._phase == "POSTURE_PRISE", f"{loco.consignes}")
loco.atteinte = True
manip.tangage_taille = 0.08                     # la marche ne suit qu'en partie
temps[0] = 2.0
f.tick()
check("Posture atteinte : ARRIVE ; le journal dit le torse demandé et le MESURÉ (la prise se "
      "calcule sur la taille mesurée)", f.navigation() == "ARRIVE"
      and any("mesuré 4.6°" in l for l in journal), str(journal[-2:]))
manip.etat, manip._rigide = Etat.TENU, object()
ok, _ = f.aller_au_poste("A")
check("Penché, le carton tenu, au départ : d'abord redressé (changer_tangage(0), l'objet tenu en "
      "place), la marche attend", ok and manip.appels[-1] == ("changer_tangage", 0.0)
      and f._phase == "REDRESSEMENT" and f.navigation() == "EN_ROUTE", f"{manip.appels}")
temps[0] = 3.0
f.tick()
check("… toujours penché : on ne part pas", f._phase == "REDRESSEMENT")
loco.cible, loco.atteinte = (0.0, 0.0), True
temps[0] = 4.0
f.tick()
check("… droit : le départ mis de côté repart", f._phase != "REDRESSEMENT"
      and f._redressement is None)

f, loco, manip, temps, journal = facade()
manip.etat, manip._rigide, manip.tangage_taille = Etat.TENU, object(), 0.08
loco.cible = (0.10, 0.0)
f.aller_au_poste("A")
temps[0] = ConfigFacade().delai_accostage_s + 1.0
f.tick()
check("… jamais droit (posture jamais atteinte) : échec après le délai, la raison dite, aucun "
      "départ", f.navigation() == "ECHEC" and "redressement" in f.raison_echec
      and f._redressement is None, f"{f._phase} {f.raison_echec}")

f, loco, manip, temps, journal = facade()
pd = SimpleNamespace(hauteur=0.64)
f.postes_depose = {"C": pd}
f._poste_depose, f._accoste = "C", True
manip.etat, manip._rigide = Etat.TENU, object()
ok, _ = f.poser()
check("Plateau bas : les jambes pliées d'abord (changer_abaissement), la dépose attend",
      ok and manip.appels == [("changer_abaissement", 0.05)]
      and any("dépose : plateau à 0.64 m" in l for l in journal), f"{manip.appels} {journal}")
temps[0] = 1.0
f.tick()
check("… bassin pas encore descendu : rien de posé", ("poser", 0.64) not in manip.appels)
loco.atteinte = True
temps[0] = 2.0
f.tick()
check("… descendu : posé sur le plateau", manip.appels[-1] == ("poser", 0.64), str(manip.appels))

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
