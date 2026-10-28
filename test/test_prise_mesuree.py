"""
Prise mesurée d'un carton (ManipulationService avec MesuresBras) : portée
par la cinématique de l'URDF, distance à la table pour que les avant-bras ne
touchent pas le plateau, mains au milieu des faces, fermeture jusqu'au
contact, serrage borné sous les limites du garde-fou, levage vérifié et
reprise, transport contre le buste, inclinaison surveillée ; refus quand
l'effort est absent, non fini ou périmé.

Sur le robot simulé léger (axis_os.testing) : cinématique de l'URDF du G1,
contact simulé (une main ne traverse pas le carton, le couple du roulis
d'épaule monte avec le serrage).
"""
import contextlib
import io
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.services.manipulation.cinematique import COUDE, MAIN, POIGNET, ModeleBras  # noqa: E402
from axis_os.services.manipulation.prise import ReglagesPrise, distance_ideale  # noqa: E402
from axis_os.services.manipulation.service import Etat, Objet, posture_bras  # noqa: E402
from axis_os.testing import SimulatedObject, SimulatedRobot  # noqa: E402

FAIL = 0
ECHELLE = 2.0
R = ReglagesPrise()
ROULIS = ("left_shoulder_roll_joint", "right_shoulder_roll_joint")


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


def immobiles(robot, avant):
    apres = robot._angles_bras()
    return max(abs(apres[j] - avant[j]) for j in avant) < 1e-3


def carton(x, largeur=0.24, z=0.90):
    return Objet((x, 0.0, z), largeur, 0.20, 0.20)


class Journal:
    """Relève, à chaque pas, l'état, les consignes et les mesures."""

    def __init__(self, robot):
        self.robot, self.pas = robot, []

    def suivre(self, duree):
        m, r = self.robot.manipulation, self.robot
        fin = time.monotonic() + duree
        while time.monotonic() < fin:
            etat = r.raw_hal.get_full_state()
            g, d = r._mains()
            self.pas.append({"etat": m.etat, "q": dict(m._q), "couples": {j: etat[j].torque
                                                                         for j in ROULIS},
                             "mains": (g, d), "objet": r.objects[0].position if r.objects
                             else None, "exemption": r.guard._exemption is not None})
            if m.etat in (Etat.TENU, Etat.ECHEC, Etat.LIBRE) and len(self.pas) > 3:
                break
            time.sleep(0.01)
        return m.etat

    def premier(self, etat):
        return next((p for p in self.pas if p["etat"] is etat), None)


def prise(robot, objet, duree=15.0):
    j = Journal(robot)
    res = robot.manipulation.prendre(objet)
    if not res.ok:
        return res, j
    j.suivre(duree)
    return res, j


section("1. Cinématique de l'URDF : la posture de port, la portée")
modele = ModeleBras()
q_port = posture_bras(0.20, -0.30, 0.0)
mg, md = modele.mains(q_port)
check("Posture de port : mains à 0,29 m devant le bassin, 33 cm l'une de l'autre "
      "(les valeurs validées sur GPU)", abs(mg[0] - 0.29) < 0.01
      and abs(float(np.linalg.norm(mg - md)) - 0.335) < 0.005, f"{mg} {md}")
q, e = modele.gauche.inverse(np.array([0.30, 0.165, 0.18]), q_port)
check("Cinématique inverse : la main va où on la veut (3 mm près)", q is not None
      and float(np.linalg.norm(modele.gauche.main(q) - np.array([0.30, 0.165, 0.18]))) < 0.003)
check("Hors de portée à 0,55 m : aucune solution", modele.gauche.inverse(
    np.array([0.55, 0.165, 0.18]), q_port)[0] is None)
d, raison = distance_ideale(modele, 0.24, 0.20, 0.20, 0.18, q_port)
d_bas, _ = distance_ideale(modele, 0.24, 0.20, 0.20, 0.22, q_port)
check("Distance à la table calculée (avant-bras au-dessus du plateau)", d is not None, raison)
check("Bassin plus bas : la distance s'adapte (au-delà de 0,30 m)", d_bas is not None
      and d_bas > d, f"{d} {d_bas}")
check("Plateau trop haut pour les bras : aucune distance, raison claire",
      distance_ideale(modele, 0.24, 0.20, 0.20, 0.55, q_port)[0] is None)


section("2. Prise normale : placement, contact, serrage borné, levage, transport")
with SimulatedRobot(objects=[SimulatedObject("carton", (0.30, 0.0, 0.90))],
                    time_scale=ECHELLE) as robot:
    res, j = prise(robot, carton(0.30))
    m = robot.manipulation
    check("Prise réussie : TENU", res.ok and m.etat is Etat.TENU, m.raison)
    f = j.premier(Etat.FERMETURE)
    g, d = f["mains"]
    check("Avant de fermer : mains au milieu des faces (2,5 cm en profondeur, 3 cm en hauteur)",
          abs((g[0] + d[0]) / 2 - 0.30) <= R.tol_profondeur_m
          and abs((g[2] + d[2]) / 2 - 0.90) <= R.tol_hauteur_m, f"{g} {d}")
    serres = [p for p in j.pas if p["etat"] in (Etat.FERMETURE, Etat.SERRAGE)]
    tangages = {round(p["q"]["left_shoulder_pitch_joint"], 6) for p in serres}
    check("Fermeture et serrage : seuls les roulis bougent (le levage est une phase à part)",
          len(tangages) == 1, str(tangages))
    leve = j.premier(Etat.LEVAGE)
    check("Le levage ne commence qu'une fois l'effort de serrage atteint",
          leve is not None and min(abs(leve["couples"][x]) for x in ROULIS)
          - robot.COUPLE_REPOS_NM >= R.effort_serrage_nm - 0.5, str(leve and leve["couples"]))
    limite = min(R.effort_max_nm, R.part_couple_garde * 25.0)
    check(f"Effort mesuré toujours sous {limite:.1f} N·m (et la moitié du couple max du "
          f"garde-fou)", m.effort_max_mesure <= limite, f"{m.effort_max_mesure:.2f}")
    o = robot.objects[0].position
    check("Carton ramené contre le buste (face arrière à ~10 cm du bassin)",
          o[0] < 0.25 and robot.held is not None, str(o))
    check("Carton à plat pendant le transport (inclinaison sous 15°)",
          robot._inclinaison() is not None and robot._inclinaison() < R.inclinaison_max_deg)
    check("Exemption du gardien levée une fois le carton soulevé",
          robot.guard._exemption is None)
    check("Aucune consigne refusée par le garde-fou", robot.locomotion.stats["refused"] == 0)
    r = robot.manipulation.poser(0.70)
    j2 = Journal(robot)
    j2.suivre(10.0)
    check("Dépose sur un plateau à 70 cm : bras vides, carton posé dessus",
          m.etat is Etat.LIBRE and abs(robot.objects[0].position[2] - 0.80) < 0.03,
          str(robot.objects[0].position))

section("3. Mains 6 cm devant le centre du carton (constat GPU) : corrigées avant de serrer")
with SimulatedRobot(objects=[SimulatedObject("carton", (0.355, 0.0, 0.90))],
                    time_scale=ECHELLE) as robot:
    g0, d0 = robot._mains()
    res, j = prise(robot, carton(0.355))
    f = j.premier(Etat.FERMETURE)
    g, d = f["mains"]
    check("Au départ, mains à ~6 cm devant le centre", abs((g0[0] + d0[0]) / 2 - 0.355) > 0.05)
    check("À la fermeture, mains au milieu (2,5 cm près)",
          abs((g[0] + d[0]) / 2 - 0.355) <= R.tol_profondeur_m, f"{(g[0] + d[0]) / 2:.3f}")
    check("… et la prise tient", robot.manipulation.etat is Etat.TENU, robot.manipulation.raison)

section("4. Carton hors de portée (26 cm plus loin) : refus, jamais de serrage dans le vide")
with SimulatedRobot(objects=[SimulatedObject("carton", (0.56, 0.0, 0.90))],
                    time_scale=ECHELLE) as robot:
    avant = robot._angles_bras()
    res = robot.manipulation.prendre(carton(0.56))
    time.sleep(0.5)
    check("Refus OUT_OF_REACH, avec la raison", not res.ok and res.reason.startswith(
        "prise refusée") and "portée" in res.reason, res.reason)
    check("… bras immobiles, aucune exemption demandée", immobiles(robot, avant)
          and robot.guard._exemption is None)

section("5. Objet mal détecté : jamais écrasé, jamais serré dans le vide")
for titre, reel, attendu in (("rien à cet endroit", None, "rien entre les mains"),
                             ("plus étroit que détecté (16 cm pour 24)", 0.16,
                              "rien entre les mains"),
                             ("plus large que détecté (31 cm pour 24)", 0.31, "ce n'est pas lui")):
    with SimulatedRobot(objects=[], time_scale=ECHELLE) as robot:
        m = robot.manipulation
        j = Journal(robot)
        m.prendre(carton(0.30))
        # L'objet réel n'apparaît qu'une fois les mains ouvertes à ses
        # faces (en posture de port, un carton de 31 cm serait déjà dans
        # les mains du robot simulé).
        while m.etat is Etat.PLACEMENT:
            time.sleep(0.005)
        if reel is not None:
            robot.objects.append(SimulatedObject("carton", (0.30, 0.0, 0.90),
                                                 size=(reel, 0.20, 0.20)))
        j.suivre(10.0)
        roulis_min = min(p["q"]["left_shoulder_roll_joint"] for p in j.pas) if j.pas else None
        q_vide = m._plan["q_vide"]["left_shoulder_roll_joint"] if m._plan else None
        check(f"{titre} : ÉCHEC « {attendu} »", m.etat is Etat.ECHEC and attendu in m.raison,
              m.raison)
        check("… la main ne dépasse jamais la borne de fermeture (2 cm dans l'objet attendu)",
              roulis_min is not None and q_vide is not None and roulis_min >= q_vide - 1e-6,
              f"{roulis_min} {q_vide}")
        check("… exemption levée, aucun serrage", robot.guard._exemption is None
              and m.effort_max_mesure < R.effort_serrage_nm, f"{m.effort_max_mesure:.1f} N·m")
with SimulatedRobot(objects=[SimulatedObject("carton", (0.30, 0.0, 0.90), size=(0.36, 0.2, 0.2))],
                    time_scale=ECHELLE) as robot:
    time.sleep(0.2)                    # le premier pas simule le contact
    avant = robot._angles_bras()
    res = robot.manipulation.prendre(carton(0.30))
    time.sleep(0.3)
    check("Bras déjà en effort au-delà de la limite (carton de 36 cm dans les mains) : refus, "
          "rien ne bouge", not res.ok and "déjà en effort" in res.reason
          and immobiles(robot, avant), res.reason)

section("6. Effort absent, non fini ou périmé : refus de serrer")
for panne in ("absent", "nan", "stale"):
    with SimulatedRobot(objects=[SimulatedObject("carton", (0.30, 0.0, 0.90))],
                        time_scale=ECHELLE) as robot:
        robot.effort_fault = panne
        avant = robot._angles_bras()
        res = robot.manipulation.prendre(carton(0.30))
        check(f"Effort « {panne} » avant la prise : refus « serrage refusé », bras immobiles",
              not res.ok and res.reason.startswith("serrage refusé")
              and immobiles(robot, avant) and robot.guard._exemption is None, res.reason)
with SimulatedRobot(objects=[SimulatedObject("carton", (0.30, 0.0, 0.90))],
                    time_scale=ECHELLE) as robot:
    m = robot.manipulation
    m.prendre(carton(0.30))
    while m.etat is not Etat.FERMETURE:
        time.sleep(0.005)
    robot.effort_fault = "nan"
    time.sleep(0.3)
    gele = dict(m._q)
    time.sleep(0.3)
    check("Effort NaN pendant la fermeture : ÉCHEC « serrage refusé », plus aucune consigne",
          m.etat is Etat.ECHEC and m.raison.startswith("serrage refusé") and m._q == gele,
          m.raison)
    check("… exemption levée", robot.guard._exemption is None)

with SimulatedRobot(objects=[SimulatedObject("carton", (0.30, 0.0, 0.90))],
                    time_scale=ECHELLE) as robot:
    m = robot.manipulation
    prise(robot, carton(0.30))
    q0, delta = dict(m._q), dict(m._delta)
    robot.effort_fault = "nan"
    time.sleep(0.3)
    check("Effort NaN pendant le port : signalé (raison, stats), serrage figé, carton tenu",
          m.etat is Etat.TENU and m.raison.startswith("effort illisible")
          and m._q == q0 and m._delta == delta and robot.held is not None
          and m.stats.get("efforts_illisibles") == 1, m.raison)
    robot.effort_fault = None
    time.sleep(0.2)
    check("… effort revenu : le signalement s'efface", m.raison == "", m.raison)

section("6b. Exemption du gardien : jamais gardée sans tenir l'objet")
with SimulatedRobot(objects=[SimulatedObject("carton", (0.30, 0.0, 0.90))],
                    time_scale=ECHELLE) as robot:
    m = robot.manipulation
    m.prendre(carton(0.30))
    check("Accordée au début de la prise (volume de l'objet)", robot.guard._exemption is not None)
    while m.etat is not Etat.FERMETURE:
        time.sleep(0.005)
    m.reinitialiser(large=True)
    check("Prise interrompue (reinitialiser) pendant la fermeture : exemption levée",
          robot.guard._exemption is None)
    time.sleep(1.2)
    check("… bras rouverts, prêts", m.etat is Etat.LIBRE)
with SimulatedRobot(objects=[SimulatedObject("carton", (0.30, 0.0, 0.90))],
                    time_scale=ECHELLE) as robot:
    m = robot.manipulation
    j = Journal(robot)
    m.prendre(carton(0.30))
    j.suivre(10.0)
    tenu = [p for p in j.pas if p["etat"] in (Etat.RAMENER, Etat.TENU)]
    check("Prise réussie : levée dès la confirmation, jamais pendant le transport",
          tenu and not any(p["exemption"] for p in tenu))

section("7. Serrage borné : raideur extrême, objet mou")
with SimulatedRobot(objects=[SimulatedObject("carton", (0.30, 0.0, 0.90))],
                    time_scale=ECHELLE) as robot:
    robot.KP_ROULIS = 200.0
    res, j = prise(robot, carton(0.30))
    m = robot.manipulation
    couples = [abs(p["couples"][x]) for p in j.pas for x in ROULIS]
    check(f"Carton très raide : effort mesuré au plus {max(couples):.1f} N·m, sous la moitié "
          f"du couple max du garde-fou (12,5)", max(couples) <= 12.5, str(max(couples)))
    check("… et la prise tient", m.etat is Etat.TENU, m.raison)
with SimulatedRobot(objects=[SimulatedObject("carton", (0.30, 0.0, 0.90))],
                    time_scale=ECHELLE) as robot:
    robot.KP_ROULIS = 5.0
    res, j = prise(robot, carton(0.30))
    m = robot.manipulation
    check("Objet mou : « serrage insuffisant », on n'insiste pas", m.etat is Etat.ECHEC
          and "serrage insuffisant" in m.raison, m.raison)
    roulis = min(p["q"]["left_shoulder_roll_joint"] for p in j.pas)
    contact = m._contact["left"]
    check("… au plus 0,20 rad au-delà du contact", contact is not None
          and contact - roulis <= R.serrage_max_rad + 1e-6, f"{contact} {roulis}")

for kp in (200.0, 400.0, 800.0):
    with SimulatedRobot(objects=[SimulatedObject("carton", (0.30, 0.0, 0.90))],
                        time_scale=ECHELLE) as robot:
        robot.KP_ROULIS = kp
        res, j = prise(robot, carton(0.30))
        serre = [abs(p["couples"][x]) for p in j.pas if p["etat"] in (Etat.SERRAGE, Etat.LEVAGE,
                                                                       Etat.RAMENER, Etat.TENU)
                 for x in ROULIS]
        check(f"Raideur {kp:.0f} N·m/rad : serrage arrivé à l'effort visé sans dépasser la "
              f"limite ({max(serre):.1f} N·m au plus, limite 10)",
              robot.manipulation.etat is Etat.TENU and max(serre) <= 10.0, robot.manipulation.raison)

section("7b. Objet tenu : jamais au-delà de la limite, même pour tenir ou corriger")
with SimulatedRobot(objects=[SimulatedObject("carton", (0.30, 0.0, 0.90))],
                    time_scale=ECHELLE) as robot:
    m = robot.manipulation
    prise(robot, carton(0.30))
    # L'objet se raidit (mains glissées vers une arête) : le couple tenu
    # passe à ~12 N·m, au-dessus de la limite de la prise (10), sous celle
    # du garde-fou (25 N·m, arrêt d'urgence au-delà de 27,5).
    robot.KP_ROULIS = 100.0
    robot._contact()
    pic = max(abs(robot.raw_hal.get_full_state()[x].torque) for x in ROULIS)
    time.sleep(1.0)
    apres = max(abs(robot.raw_hal.get_full_state()[x].torque) for x in ROULIS)
    check(f"Couple tenu à {pic:.1f} N·m : desserré jusque sous la limite ({apres:.1f} N·m), "
          f"carton toujours tenu, pas d'arrêt d'urgence", pic > 10.0 and apres <= 10.0
          and m.etat is Etat.TENU and robot.held is not None and not robot.hal.estop_active
          and m.stats.get("serrages_limites", 0) > 0, f"{pic} {apres} {m.etat}")
with SimulatedRobot(objects=[SimulatedObject("carton", (0.30, 0.0, 0.90))],
                    time_scale=ECHELLE) as robot:
    m = robot.manipulation
    prise(robot, carton(0.30))
    robot.KP_ROULIS = 68.0         # couple tenu ~9 N·m : moins de 2 N·m de marge
    time.sleep(0.3)
    delta = dict(m._delta)
    robot.object_tilt_deg = 25.0
    j = Journal(robot)
    time.sleep(0.2)
    j.suivre(4.0)
    couples = [abs(p["couples"][x]) for p in j.pas for x in ROULIS]
    fin = max(abs(robot.raw_hal.get_full_state()[x].torque) for x in ROULIS)
    check(f"Correction d'inclinaison près de la limite : ramené plus près sans serrer plus ; "
          f"au plus {max(couples):.1f} N·m en chemin, {fin:.1f} à la fin (limite 10, "
          f"garde-fou 25)", m.stats.get("corrections_inclinaison") == 1
          and all(m._delta[c] <= delta[c] + 1e-9 for c in delta) and fin <= 10.0
          and max(couples) < 25.0 and not robot.hal.estop_active, f"{delta} {m._delta}")
with SimulatedRobot(objects=[SimulatedObject("carton", (0.30, 0.0, 0.90))],
                    time_scale=ECHELLE) as robot:
    m = robot.manipulation
    prise(robot, carton(0.30))
    delta = dict(m._delta)
    robot.effort_fault = "stale"
    robot.object_tilt_deg = 25.0
    time.sleep(0.2)
    Journal(robot).suivre(4.0)
    check("Correction d'inclinaison, effort périmé : aucun serrage de plus",
          m.stats.get("corrections_inclinaison") == 1
          and all(m._delta[c] <= delta[c] + 1e-9 for c in delta), f"{delta} {m._delta}")

section("8. Levage vérifié : reposer, desserrer, recommencer une fois")
with SimulatedRobot(objects=[SimulatedObject("carton", (0.30, 0.0, 0.90))],
                    grasp_succeeds=False, time_scale=ECHELLE) as robot:
    res, j = prise(robot, carton(0.30), duree=25.0)
    m = robot.manipulation
    check("Prise qui ne tient pas : une reprise, puis ÉCHEC clair",
          m.etat is Etat.ECHEC and m.stats.get("reprises") == 1 and "2 essais" in m.raison,
          f"{m.raison} {m.stats}")
    check("… entre les deux essais, les bras ont reposé et desserré (état REPOSE)",
          any(p["etat"] is Etat.REPOSE for p in j.pas))
    check("… exemption levée", robot.guard._exemption is None)
    m.reinitialiser(large=True)
    time.sleep(1.0)
    check("reinitialiser() : bras rouverts, prêts", m.etat is Etat.LIBRE)

section("9. Transport : inclinaison surveillée, corrigée, signalée")
with SimulatedRobot(objects=[SimulatedObject("carton", (0.30, 0.0, 0.90))],
                    time_scale=ECHELLE) as robot:
    m = robot.manipulation
    prise(robot, carton(0.30))
    robot.object_tilt_deg = 25.0
    time.sleep(0.2)
    check("Carton penché de 25° : correction lancée (serrer, ramener plus près)",
          m.stats.get("corrections_inclinaison") == 1 and m.penche)
    Journal(robot).suivre(5.0)
    time.sleep(0.2)
    check("Toujours penché après la correction : signalé « à reposer »",
          m.penche and "à reposer" in m.raison and m.inclinaison_max >= 25.0, m.raison)
    robot.object_tilt_deg = 3.0
    time.sleep(0.2)
    check("Carton redevenu à plat : la façade le sait (inclinaison mesurée 3°)",
          m.inclinaison == 3.0)

section("10. Avant-bras au-dessus du plateau : la distance s'en déduit, sinon refus")
# Carton de 20 cm sur un plateau à 0,80 m, à 5 cm du bord de la table :
# robot à 0,30 m, le bord est à 15 cm du bassin et les avant-bras passent à
# 2,5 cm du plateau (il en faut 3).
with SimulatedRobot(objects=[SimulatedObject("carton", (0.30, 0.0, 0.90))],
                    time_scale=ECHELLE) as robot:
    avant = robot._angles_bras()
    res = robot.manipulation.prendre(Objet((0.30, 0.0, 0.90), 0.24, 0.20, 0.20, bord_m=0.05))
    check("Trop près de la table : refus « avant-bras … du plateau », bras immobiles",
          not res.ok and "avant-bras" in res.reason and immobiles(robot, avant),
          res.reason)
d, raison = distance_ideale(modele, 0.24, 0.20, 0.20, 0.18, q_port, bord_m=0.05)
check("distance_ideale() choisit une distance plus grande", d is not None and d > 0.30,
      f"{d} {raison}")
with SimulatedRobot(objects=[SimulatedObject("carton", (d, 0.0, 0.90))],
                    time_scale=ECHELLE) as robot:
    res, j = prise(robot, Objet((d, 0.0, 0.90), 0.24, 0.20, 0.20, bord_m=0.05))
    m = robot.manipulation
    check(f"… à {d:.3f} m, la prise tient, avant-bras à {100 * (m._plan or {}).get('garde', 0):.1f}"
          f" cm du plateau", res.ok and m.etat is Etat.TENU
          and m._plan["garde"] >= R.marge_plateau_m, f"{res.reason} {m.raison}")

print(f"\n{'=' * 66}\n  {'TOUT PASSE' if FAIL == 0 else f'{FAIL} ÉCHEC(S)'}\n{'=' * 66}")
sys.exit(1 if FAIL else 0)
