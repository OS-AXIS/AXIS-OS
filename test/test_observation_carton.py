"""
Carton revu après l'arrêt, avant la prise.

Constat GPU (essai T1 de main, novembre 2026, opérateur actif) : trois fois,
juste après « OPÉRATEUR : carton DÉPLACÉ de 5 / 10 / 3 cm », la ligne
« carton … vu » donnait la position d'AVANT le déplacement, sur « 1
image(s) » : l'estimation de la visée (depuis la pré-pose) n'était jamais
renouvelée, et la prise visait l'ancienne position.

Désormais (axis_os/apps/tri_colis/facade.py, ConfigFacade, observation) :
toute estimation est oubliée à chaque nouvel accostage et à chaque nouvel
essai ; arrivé à la pose d'attente, robot immobile et bras ouverts, la
façade exige au moins 3 images prises APRÈS cet arrêt, cohérentes entre
elles (< 1 cm, < 3°), sinon elle regarde encore ; si le carton a bougé
depuis la dernière estimation, le journal le dit, et la cible suit.

Robot simulé léger (axis_os.testing) : vrais services de navigation, de
locomotion et de manipulation (prise mesurée), caméra idéale (une image
neuve à chaque appel, horodatée à l'heure du robot).
"""
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.apps.tri_colis.facade import ConfigFacade, FacadeServices  # noqa: E402
from axis_os.apps.tri_colis.logique import Colis  # noqa: E402
from axis_os.services.manipulation.service import Etat  # noqa: E402
from axis_os.testing import SimulatedObject, SimulatedRobot  # noqa: E402

FAIL = 0


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


X_CARTON = 1.40


def monter(x=X_CARTON, y=0.0):
    r = SimulatedRobot(objects=[SimulatedObject("carton bleu", (x, y, 0.90))], time_scale=2.0)
    r.__enter__()
    time.sleep(0.2)
    journal = []
    # La caméra idéale donne le centre du carton (pas la face avant).
    f = FacadeServices(r.navigation, r.locomotion, r.manipulation, r.perception, r._pose2d,
                       r.clock, cap_prise=0.0, journal=journal.append,
                       cfg=ConfigFacade(requete="{couleur}", position_face_avant=False))
    return r, f, journal


def approcher(r, f, colis, pendant=None, duree=60.0):
    """Accostage complet ; `pendant(f)` est appelé à chaque pas (pour
    déplacer le carton au bon moment). Rend l'état de navigation final et
    les phases traversées."""
    ok, raison = f.accoster(colis)
    assert ok, raison
    phases, t0 = [], time.monotonic()
    while time.monotonic() - t0 < duree:
        f.tick()
        if not phases or phases[-1] != f._phase:
            phases.append(f._phase)
        if pendant is not None:
            pendant(f)
        e = f.navigation()
        if e in ("ARRIVE", "ECHEC"):
            return e, phases
        time.sleep(0.005)
    return "DÉLAI", phases


def prendre(r, f, colis, duree=20.0):
    ok, raison = f.prendre(colis)
    if not ok:
        return ok, raison
    m, t0 = r.manipulation, time.monotonic()
    while m.etat not in (Etat.TENU, Etat.ECHEC) and time.monotonic() - t0 < duree:
        time.sleep(0.005)
    return m.etat is Etat.TENU, m.raison


def ecart_mains(r):
    """Écart latéral (m, repère du robot) entre le milieu des mains et le
    VRAI centre du carton (vérité de la simulation)."""
    g, d = r._mains()
    milieu = (g + d) / 2
    c = r._vers_robot(r.objects[0].position)
    return abs(float(r._vers_robot(milieu)[1] - c[1]))


section("1. Carton déplacé entre la perception et la prise")
r, f, journal = monter()
colis = f.chercher("bleu")[0]
deplace = []


def pousser(f_):
    # Pendant l'ouverture des bras (la visée est déjà faite) : le carton est
    # poussé de 5 cm sur le côté et de 3 cm vers l'arrière.
    if f_._phase == "PRESAISIE" and not deplace:
        o = r.objects[0]
        o.position = (o.position[0] + 0.03, o.position[1] + 0.05, o.position[2])
        deplace.append(o.position)


etat, phases = approcher(r, f, colis, pousser)
check("Carton poussé pendant l'ouverture des bras, après la visée", bool(deplace), str(phases))
check("Accostage achevé (arrivé à la pose de prise)", etat == "ARRIVE", f"{etat} {phases}")
bouge = [l for l in journal if "a BOUGÉ" in l]
check("Le journal dit que le carton a bougé depuis la dernière estimation, de combien",
      bool(bouge) and "a BOUGÉ de 5.8 cm" in bouge[0], str(journal))
check("… le robot s'est réaligné devant lui, puis l'a regardé de nouveau",
      any("réalignement (1/2)" in l for l in journal)
      and phases.count("OBSERVATION") >= 2, f"{phases} {journal}")
check("… estimation sur 3 images prises APRÈS l'arrêt, cohérentes",
      f._observe is not None and "3 images prises après l'arrêt" in f._observe.vu,
      str(f._observe))
check("… position revue = vraie position, à 1 cm près",
      f._observe is not None and math.dist(f._observe.position[:2], deplace[0][:2]) < 0.01,
      str(f._observe))
# La logique de tri passe encore le colis de sa recherche (position périmée) :
# la façade prend le carton revu.
tenu, raison = prendre(r, f, colis)
check("Prise sur la NOUVELLE position : le carton est tenu", tenu, raison)
check("… mains centrées sur le vrai carton (écart latéral < 1.5 cm)", ecart_mains(r) < 0.015,
      f"{100 * ecart_mains(r):.1f} cm")
check("… la ligne « carton vu » donne la position revue, pas l'ancienne",
      any(l.startswith("carton bleu vu : centre (1.430, 0.050") for l in journal),
      str([l for l in journal if "vu :" in l]))
check("… sans consigne refusée par le garde-fou", r.locomotion.stats["refused"] == 0)
r.__exit__(None, None, None)

section("2. Sans images prises après l'arrêt : pas de prise")


class ImagesFigees:
    """La caméra rend toujours la même image, prise avant l'arrêt."""

    def __init__(self, p, t_ns):
        self.p, self.derniere_image_ns = p, t_ns

    def find(self, *a, **k):
        return self.p.find(*a, **k)


r, f, journal = monter()
colis = f.chercher("bleu")[0]
f.percep = ImagesFigees(r.perception, int(r.clock() * 1e9))
etat, phases = approcher(r, f, colis)
check("Images toutes d'avant l'arrêt : accostage en ÉCHEC après l'observation",
      etat == "ECHEC" and "OBSERVATION" in phases, f"{etat} {phases}")
check("… le journal dit pourquoi : aucune image neuve",
      any("pas revu de façon cohérente après l'arrêt" in l and "0 image(s) neuve(s)" in l
          for l in journal), str(journal))
ok, raison = f.prendre(colis)
check("… et la prise est refusée, bras immobiles", not ok and "pas revu après l'arrêt" in raison
      and r.manipulation.etat in (Etat.PRET, Etat.LIBRE), raison)
r.__exit__(None, None, None)

section("3. Images incohérentes entre elles : on regarde encore, puis on renonce")
r, f, journal = monter()
colis = f.chercher("bleu")[0]
vrai = f.chercher
n_images = [0]


def instable(couleur, fusion=True):
    vus = vrai(couleur, fusion)
    if f._phase != "OBSERVATION":
        return vus
    n_images[0] += 1
    s = 1 if n_images[0] % 2 else -1
    # Lacet qui saute de ±2.5° d'une image à l'autre (5° d'écart > 3°).
    return [Colis(c.couleur, c.position, c.confiance, math.radians(2.5 * s)) for c in vus]


f.chercher = instable
etat, phases = approcher(r, f, colis)
check("Lacets à 5° l'un de l'autre : jamais 3 images cohérentes, accostage en ÉCHEC",
      etat == "ECHEC", f"{etat} {phases}")
check("… après avoir regardé bien plus de 3 images", n_images[0] > 6, str(n_images[0]))
check("… journal : écart max des images retenues",
      any("pas revu de façon cohérente" in l and "5.0°" in l for l in journal), str(journal))
r.__exit__(None, None, None)

section("4. Estimation oubliée à chaque nouvel essai et à chaque nouvel accostage")
r, f, journal = monter()
colis = f.chercher("bleu")[0]
etat, _ = approcher(r, f, colis)
revu = f._observe is not None
f.reinitialiser_manipulation()          # une prise ratée
check("Après une prise ratée : plus aucune estimation (visée, observation, boîtes)",
      revu and f._colis_vise is None and f._observe is None and not f._suivis
      and not f.derniere_boite, str(f._observe))
ok, raison = f.prendre(colis)
check("… prise refusée tant que le carton n'est pas revu", not ok
      and "pas revu après l'arrêt" in raison, raison)
f._observe = Colis("bleu", (X_CARTON, 0.0, 0.90), 1.0, vu="ancienne")
f.accoster(colis)
check("Nouvel accostage : l'observation précédente est oubliée", f._observe is None)
r.__exit__(None, None, None)

section("5. Prise hors de portée : nouvel accostage plutôt qu'une prise au bord")
from axis_os.apps.tri_colis.logique import ConfigTri, TriColis  # noqa: E402
from axis_os.services.manipulation.prise import ReglagesPrise  # noqa: E402
from axis_os.services.navigation.service import Carte, Pose2D  # noqa: E402

rg = ReglagesPrise()
check("Mains au moins 7 cm derrière la face avant : avance vers elle bornée à 2.5 cm pour un "
      "carton de 20 cm, 0.5 cm pour 15 cm, au milieu (2.5 cm) sous 14 cm",
      abs(rg.avance_max(0.20) - 0.025) < 1e-12 and abs(rg.avance_max(0.15) - 0.005) < 1e-12
      and abs(rg.avance_max(0.12) - 0.025) < 1e-12)


def mission(trop_loin_a_chaque_fois, duree=150.0):
    """TriColis sur le robot simulé ; l'insertion s'arrête 12 cm avant la
    pose de prise (le premier accostage seulement, ou tous) : le carton est
    alors à 0.41 m du bassin, hors de portée des bras."""
    carte = Carte(-2.0, -2.0, 3.0, 2.0, postes={"prise": Pose2D(0.0, 0.0, 0.0)})
    r = SimulatedRobot(carte=carte, objects=[SimulatedObject("carton bleu", (X_CARTON, 0.0, 0.90))],
                       time_scale=2.0)
    r.__enter__()
    time.sleep(0.2)
    journal = []
    f = FacadeServices(r.navigation, r.locomotion, r.manipulation, r.perception, r._pose2d,
                       r.clock, cap_prise=0.0, journal=journal.append,
                       cfg=ConfigFacade(requete="{couleur}", position_face_avant=False))
    insertions = [0]
    lancer = f._lancer_insertion

    def court():
        insertions[0] += 1
        if trop_loin_a_chaque_fois or insertions[0] == 1:
            q = f._insertion
            f._insertion = Pose2D(q.x - 0.12, q.y, q.cap)
        lancer()

    f._lancer_insertion = court
    tri = TriColis(f, ConfigTri(regles={"bleu": "A"}, duree_mission_s=1e6))
    t0 = time.monotonic()
    while time.monotonic() - t0 < duree:
        tri.pas()
        f.tick()
        if tri.colis is None and tri.bilan.colis_abandonnes or any("en main" in l for l in journal):
            break
        time.sleep(0.005)
    return r, f, tri, journal, insertions[0]


r, f, tri, journal, n = mission(False)
loin = [l for l in journal if "nouvel accostage, plus près (1/2)" in l]
check("Robot arrêté trop loin : prise refusée « hors de portée », sans fermer les mains, puis "
      "nouvel accostage", bool(loin) and "hors de portée" in loin[0]
      and tri.bilan.prises_ratees == 0, str(journal))
check("… le second accostage aboutit : carton tenu, mains au milieu", any(
    "colis bleu en main" in l for l in journal) and r.manipulation.etat is Etat.TENU
      and ecart_mains(r) < 0.015, f"{r.manipulation.etat} {journal[-3:]}")
r.__exit__(None, None, None)
r, f, tri, journal, n = mission(True)
check("Toujours trop loin : 2 nouveaux accostages au plus, puis colis SIGNALÉ et laissé",
      tri.bilan.colis_abandonnes == 1 and n == 3 and any(
          "hors de portée après 2 nouveaux accostages" in l for l in journal)
      and tri.bilan.prises_ratees == 0, f"{n} {journal[-4:]}")
check("… bras jamais refermés sur le carton (aucun serrage)", r.manipulation.stats.get(
    "prises", 0) == 0 and r.objects[0].position == (X_CARTON, 0.0, 0.90),
      f"{r.manipulation.stats} {r.objects[0].position}")
r.__exit__(None, None, None)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
