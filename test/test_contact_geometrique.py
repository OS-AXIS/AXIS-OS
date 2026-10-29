"""
Contact décidé en géométrie, borne de fermeture en géométrie, raideur des
poignets pendant la prise.

Constat GPU (2026-10-29, evaluer_vla.sh 5 1 --reflexes) : 0/2, « fermeture
au bout de sa course sans contact : faces intérieures des mains à 0.239 m,
l'objet attendu en ferait 0.24 ». Les mains étaient CONTRE le carton, mais
le contact n'était cherché que sur le roulis d'épaule (couple, retard) : le
roulis suivait sa consigne, et c'étaient le poignet et le coude qui
cédaient.

Robot simulé léger (axis_os.testing) : `poignets_cedent` reproduit ce cas
(le carton bloque la main, le lacet du poignet et le coude plient, le
roulis suit sa consigne sans effort) ; la raideur demandée par l'OS
(MesuresBras.raideur) réduit au quart ce qui peut céder.
"""
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.services.manipulation.prise import ReglagesPrise  # noqa: E402
from axis_os.services.manipulation.service import Etat, Objet  # noqa: E402
from axis_os.testing import SimulatedObject, SimulatedRobot  # noqa: E402

FAIL = 0


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


def robot(objets=True, y=0.0, lacet_deg=0.0, cedent=True):
    r = SimulatedRobot(objects=[SimulatedObject("carton", (0.30, y, 0.90),
                                                lacet=math.radians(lacet_deg))] if objets else [],
                       time_scale=2.0)
    r.__enter__()
    r.poignets_cedent = cedent
    time.sleep(0.2)
    return r


def prendre(r, objet, reglages=None, jusqua=(Etat.TENU, Etat.ECHEC), duree=15.0):
    m = r.manipulation
    if reglages is not None:
        m.r = reglages
    m.prendre(objet)
    etats, t0 = [], time.monotonic()
    while m.etat not in jusqua and time.monotonic() - t0 < duree:
        if not etats or etats[-1] is not m.etat:
            etats.append(m.etat)
        time.sleep(0.005)
    etats.append(m.etat)
    return etats


CARTON = Objet((0.30, 0.0, 0.90), 0.24, 0.20, 0.20)

section("1. Le constat GPU : main contre le carton, poignet et coude qui cèdent")
# Sans le contact géométrique (seuil hors d'atteinte), c'est le code d'avant :
# le message d'échec dit maintenant pourquoi.
r = robot()
etats = prendre(r, CARTON, ReglagesPrise(retard_face_m=1.0))
m = r.manipulation
check("Contact cherché sur le seul roulis : « fermeture au bout de sa course sans contact »",
      m.etat is Etat.ECHEC and "au bout de sa course sans contact" in m.raison, m.raison)
check("… le message donne les faces COMMANDÉES et MESURÉES",
      "faces intérieures commandées à" in m.raison and "mesurées à" in m.raison, m.raison)
check("… et les plus grands écarts consigne-mesure : le coude, le poignet, pas le roulis",
      "écarts consigne-mesure" in m.raison and "elbow" in m.raison.split("écarts")[1]
      and "shoulder_roll" not in m.raison.split("écarts")[1], m.raison)
r.__exit__(None, None, None)
r = robot()
etats = prendre(r, CARTON)
m = r.manipulation
check("Contact en géométrie : la prise tient", m.etat is Etat.TENU, f"{m.raison} {etats}")
check("… contact estimé des deux mains par la géométrie",
      all(m.contacts_estimes[c] and m.contacts_estimes[c]["signal"] == "géométrie"
          for c in ("left", "right")), str(m.contacts_estimes))
check("… sans consigne refusée par le garde-fou", r.locomotion.stats["refused"] == 0)
r.__exit__(None, None, None)

section("2. Serrage dans le vide : toujours refusé, aucune fausse prise")
r = robot(objets=False)
etats = prendre(r, CARTON)
m = r.manipulation
check("Rien entre les mains (poignets mous) : ÉCHEC « rien entre les mains »",
      m.etat is Etat.ECHEC and "rien entre les mains" in m.raison, m.raison)
check("… aucun contact estimé, aucun serrage", m.contacts_estimes == {"left": None, "right": None}
      and Etat.SERRAGE not in etats, f"{m.contacts_estimes} {etats}")
r.__exit__(None, None, None)

section("3. Carton tourné de 11°")
r = robot(lacet_deg=11.0)
etats = prendre(r, Objet((0.30, 0.0, 0.90), 0.24, 0.20, 0.20, lacet=math.radians(11.0)))
m = r.manipulation
check("Carton et lacet connus tournés de 11° : la prise tient", m.etat is Etat.TENU,
      f"{m.raison} {etats}")
r.__exit__(None, None, None)

section("4. Une main touche avant l'autre")
r = robot(y=0.012)          # carton 1,2 cm plus à gauche que perçu
etats = prendre(r, CARTON)
m = r.manipulation
ce = m.contacts_estimes
check("La main gauche touche d'abord, la droite continue puis touche",
      ce["left"] and ce["right"] and ce["left"]["t"] < ce["right"]["t"], str(ce))
check("… et la prise tient", m.etat is Etat.TENU, m.raison)
r.__exit__(None, None, None)

section("5. Gains des poignets : raidis pour la prise, rendus au dépôt, à l'échec, sur arrêt "
        "d'urgence")
r = robot(objets=False)
prendre(r, CARTON)
check("Échec (rien entre les mains) : raidis à la fermeture, rendus à l'échec",
      r.changements_raideur == [True, False] and not r.raideur_active, str(r.changements_raideur))
r.__exit__(None, None, None)
r = robot()
prendre(r, CARTON)
m = r.manipulation
check("Prise tenue : gains raidis pendant le port", r.raideur_active
      and r.changements_raideur == [True])
m.poser(0.70)
t0 = time.monotonic()
while m.etat is not Etat.LIBRE and time.monotonic() - t0 < 10:
    time.sleep(0.01)
check("Dépôt fini : gains normaux", not r.raideur_active
      and r.changements_raideur == [True, False], str(r.changements_raideur))
r.__exit__(None, None, None)
r = robot()
prendre(r, CARTON)
r.hal.emergency_stop_all()
time.sleep(0.3)
check("Arrêt d'urgence, carton tenu : gains normaux aussitôt", r.hal.estop_active
      and not r.raideur_active and r.manipulation.stats.get("raideur_rendue_urgence") == 1,
      str(r.changements_raideur))
r.__exit__(None, None, None)
r = robot()
prendre(r, CARTON, jusqua=(Etat.SERRAGE, Etat.ECHEC))
r.manipulation.reinitialiser(large=True)
time.sleep(0.1)
check("Prise interrompue (reinitialiser) : gains normaux", not r.raideur_active)
r.__exit__(None, None, None)

print(f"\n{'=' * 66}\n  {'TOUT PASSE' if FAIL == 0 else f'{FAIL} ÉCHEC(S)'}\n{'=' * 66}")
sys.exit(1 if FAIL else 0)
