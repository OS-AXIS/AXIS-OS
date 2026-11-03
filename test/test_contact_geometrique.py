"""
Contact décidé en géométrie, borne de fermeture en géométrie, raideur des
poignets pendant la prise.

Constat GPU (2026-10-29, evaluer_vla.sh 5 1 --reflexes) : 0/2, « fermeture
au bout de sa course sans contact : faces intérieures des mains à 0.239 m,
l'objet attendu en ferait 0.24 ». Les mains étaient CONTRE le carton, mais
le contact n'était cherché que sur le roulis d'épaule (couple, retard) : le
roulis suivait sa consigne, et c'étaient le poignet et le coude qui
cédaient.

Robot simulé léger (axis_os.testing) : chaque articulation des bras est un
ressort autour de sa consigne ; `poignets_cedent` reproduit ce cas (lacet
d'épaule, coude et poignets assouplis de `souplesse_poignets` : ils plient,
et le couple du roulis monte peu) ; la raideur demandée par l'OS
(MesuresBras.raideur) les double, plafonnée au plus grand gain du pilote.
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


def robot(objets=True, y=0.0, lacet_deg=0.0, cedent=True, souplesse=0.25, x=0.30):
    r = SimulatedRobot(objects=[SimulatedObject("carton", (x, y, 0.90),
                                                lacet=math.radians(lacet_deg))] if objets else [],
                       time_scale=2.0)
    r.__enter__()
    r.poignets_cedent = cedent
    r.souplesse_poignets = souplesse
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
# Sans le contact géométrique ni la force à la main (seuils hors d'atteinte),
# c'est le code d'avant : le message d'échec dit maintenant pourquoi. Bras
# très souples (dixième de leur raideur) : dans les 2 cm de la borne, le
# roulis ne prend pas 1,5 N·m.
r = robot(souplesse=0.1)
etats = prendre(r, CARTON, ReglagesPrise(retard_face_m=1.0, force_contact_n=1e3))
m = r.manipulation
check("Contact cherché sur le seul roulis : « fermeture au bout de sa course sans contact »",
      m.etat is Etat.ECHEC and "au bout de sa course sans contact" in m.raison, m.raison)
check("… le message donne les faces COMMANDÉES et MESURÉES",
      "faces intérieures commandées à" in m.raison and "mesurées à" in m.raison, m.raison)
check("… et les plus grands écarts consigne-mesure : lacet d'épaule, coude, poignet, pas le "
      "roulis", "écarts consigne-mesure" in m.raison
      and "shoulder_yaw" in m.raison.split("écarts")[1]
      and "shoulder_roll" not in m.raison.split("écarts")[1], m.raison)
r.__exit__(None, None, None)
r = robot()
etats = prendre(r, CARTON)
m = r.manipulation
check("Contact en géométrie : la prise tient", m.etat is Etat.TENU, f"{m.raison} {etats}")
check("… contact estimé des deux mains sans le roulis : par la géométrie ou la force",
      all(m.contacts_estimes[c] and m.contacts_estimes[c]["signal"] in ("géométrie", "force")
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



# Tolérance des relevés d'enfoncement : la face mesurée au toucher, d'où part
# la borne, est déjà comprimée par le contact, et le carton (20 000 N/m) l'est
# encore sous les 15 à 25 N du port, au buste (~1,3 mm).
COMPRESSION_M = 0.002


def penetrations(r):
    """Relevé, dans le fil de la simulation, à chaque consigne des bras : de
    combien la face intérieure de chaque main, là où elle est vraiment
    (consigne plus son décalage mesuré avant le contact), entre dans la VRAIE
    face du carton (vérité de la simulation, repère du robot, lacet nul).
    Rend {"fermeture": pire avant le toucher, "toucher": pire du toucher au
    port (m), toutes mains}."""
    m, pire = r.manipulation, {"fermeture": -1.0, "toucher": -1.0}
    ecrire = m._ecrire_q
    phases = {Etat.FERMETURE: "fermeture", Etat.SERRAGE: "toucher", Etat.LEVAGE: "toucher",
              Etat.RAMENER: "toucher", Etat.TENU: "toucher"}

    def releve(q):
        ecrire(q)
        phase = phases.get(m.etat)
        if phase is not None:
            g, d = r.modele_bras.mains({**m._fixes, **m._q})
            c, demi = r.objects[0].position, r.objects[0].size[0] / 2.0
            yc = float(c[1]) - r.y
            fg = float(g[1]) - m.r.demi_main_m + m._decalage("left")
            fd = float(d[1]) + m.r.demi_main_m + m._decalage("right")
            pire[phase] = max(pire[phase], (yc + demi) - fg, fd - (yc - demi))
    m._ecrire_q = releve
    return pire


section("4. Une main touche avant l'autre")
# Bras aux gains du banc : les bornes du serrage partent des faces mesurées
# au toucher, la main gauche ne s'enfonce plus de 3,2 cm dans la vraie face
# (2 cm depuis la face perçue, 1,2 cm plus loin que la vraie).
r = robot(y=0.012, cedent=False)          # carton 1,2 cm plus à gauche que perçu
etats = prendre(r, CARTON)
m = r.manipulation
ce = m.contacts_estimes
check("La main gauche touche d'abord, la droite continue puis touche",
      ce["left"] and ce["right"] and ce["left"]["t"] < ce["right"]["t"], str(ce))
check("… et la prise tient", m.etat is Etat.TENU, m.raison)
r.__exit__(None, None, None)
# Poignets mous (constat GPU, section 1) : la main gauche y faisait ~170 N/m ;
# elle tenait en s'enfonçant de 3,2 cm dans la vraie face. Dans 2 cm, elle ne
# peut donner que ~3,5 N : refus, sans jamais passer la borne.
r = robot(y=0.012)
pire = penetrations(r)
etats = prendre(r, CARTON)
m = r.manipulation
ce = m.contacts_estimes
check("Poignets mous : la main gauche touche d'abord, la droite continue puis touche",
      ce["left"] and ce["right"] and ce["left"]["t"] < ce["right"]["t"], str(ce))
check(f"… aucune main à plus de 2 cm dans la vraie face : refus « serrage insuffisant » "
      f"(au plus {100 * max(pire.values()):.2f} cm, fermeture comprise)",
      m.etat is Etat.ECHEC and "serrage insuffisant" in m.raison
      and max(pire.values()) <= m.r.penetration_max_m + COMPRESSION_M, f"{m.raison} {pire}")
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

section("6. Recalage au toucher : carton 2 cm à côté de sa perception")
journal = []
r = robot(y=0.02, cedent=False)
m = r.manipulation
m._journal = journal.append
pire = penetrations(r)
etats = prendre(r, CARTON)
rec = m.recalage or {}
check("Au contact des deux mains, avant de serrer : « recalage au toucher : carton à +2.0 cm "
      "de la perception » au journal", any("recalage au toucher : carton à +2.0 cm" in l
                                            for l in journal), str(journal))
check("… le centre de la prise suit, 2 mm près (milieu des faces mesurées), et la prise tient",
      rec and abs(rec["decalage_m"] - 0.02) <= 0.002
      and abs(float(m._plan["centre"][1]) - float(rec["avant"][1]) - 0.02) <= 0.002
      and m.etat is Etat.TENU, f"{rec} {m.raison}")
check(f"… bornes du serrage depuis les faces mesurées : au plus {100 * pire['toucher']:.2f} cm "
      f"dans la vraie face, du toucher au port", pire["toucher"] <= m.r.penetration_max_m + COMPRESSION_M,
      str(pire))
r.__exit__(None, None, None)
r = robot(y=0.02, cedent=False)
etats = prendre(r, CARTON, ReglagesPrise(recalage_max_m=0.01))
m = r.manipulation
check("Au-delà de recalage_max_m (1 cm ici, 3 par défaut) : « prise refusée — recalage au "
      "toucher … la perception le voyait ailleurs », sans serrer", m.etat is Etat.ECHEC
      and "prise refusée — recalage au toucher" in m.raison and "voyait ailleurs" in m.raison
      and Etat.SERRAGE not in etats, f"{m.raison} {etats}")
check("… gains des poignets rendus", not r.raideur_active, str(r.changements_raideur))
r.__exit__(None, None, None)

section("7. Carton 3 cm à côté de son estimation : aucune main à plus de 2 cm dans sa vraie face "
        "à partir du toucher")
# Les bornes du serrage partent des faces mesurées au toucher : du toucher au
# port, aucune main n'entre de plus de 2 cm dans la vraie face (à la
# compression du carton près, COMPRESSION_M). AVANT le toucher, la fermeture
# vise la face perçue :
# à 3 cm, la main est déjà contre le carton quand elle commence (géométrie de
# T1 : les mains l'encadrent à 2,5 mm en posture de port), le contact n'est vu
# qu'à la force, et la première main va ~2,5 cm dans sa face. Relevé, non
# borné ici (voir la PR).
for dy in (0.03, -0.03):
    r = robot(y=dy, cedent=False)
    pire = penetrations(r)
    etats = prendre(r, CARTON)
    m = r.manipulation
    issue = ("tenu" if m.etat is Etat.TENU else
             "refusé au recalage" if "recalage au toucher" in m.raison else m.raison[:60])
    check(f"Décalé de {100 * dy:+.0f} cm ({issue}) : au plus {100 * pire['toucher']:.2f} cm dans "
          f"la vraie face du toucher au port (fermeture : {100 * pire['fermeture']:.2f} cm)",
          Etat.FERMETURE in etats and m.etat in (Etat.TENU, Etat.ECHEC)
          and pire["toucher"] <= m.r.penetration_max_m + COMPRESSION_M, f"{pire} {m.raison} {etats}")
    r.__exit__(None, None, None)

print(f"\n{'=' * 66}\n  {'TOUT PASSE' if FAIL == 0 else f'{FAIL} ÉCHEC(S)'}\n{'=' * 66}")
sys.exit(1 if FAIL else 0)
