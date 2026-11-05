"""
Fermeture de la prise mesurée sur des bras qui s'affaissent.

Constat GPU (2026-10-29, evaluer_vla.sh 5 1 --reflexes) : 0/5, « rien
entre les mains » ×5. Mains ~3,3 cm plus bas que leurs cibles (bras qui
s'affaissent sous leur poids) ; au refus, leurs faces intérieures étaient
encore à ~0,5 cm des faces du carton : l'angle mesuré du roulis avait
atteint la borne q_vide, calculée pour des bras à la hauteur visée.

Robot simulé léger (axis_os.testing) : `affaissement_rad` ajoute au
tangage d'épaule mesuré un écart à sa consigne (main plus basse).

Horloge simulée : le test fait lui-même chaque pas du robot (r.pas()) et lit
les mains entre deux pas. Constat (CI de la PR 26, puis 1 fois sur 3 sur main
sous charge) : « mains à 3,2 cm de la mi-hauteur à la fermeture » alors que
la prise, elle, avait décidé de fermer mains à 0,1 cm. Le test lisait les
mains depuis son propre fil pendant que celui du robot simulé écrivait un
pas : entre la consigne du bras et sa position physique (affaissement
compris), le tangage mesuré valait la consigne, mains 3,2 cm trop haut. La
prise lit ses mesures dans le fil du robot, jamais au milieu d'un pas.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.services.manipulation.prise import ReglagesPrise  # noqa: E402
from axis_os.services.manipulation.service import Etat, Objet  # noqa: E402
from axis_os.testing import SimulatedObject, SimulatedRobot  # noqa: E402

FAIL = 0
R = ReglagesPrise()
CARTON = Objet((0.30, 0.0, 0.90), 0.24, 0.20, 0.20)
Z_CIBLE = 0.90 - SimulatedRobot.PELVIS_HEIGHT_M       # milieu des faces, repère du bassin


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


def prise(affaissement, objets=True, reglages=None, duree=30.0, pas_s=None):
    """Lance une prise sur une horloge simulée (le test fait chaque pas du
    robot, `duree` secondes du robot au plus ; `pas_s()` : durée de chaque
    pas, par défaut celle de la boucle du robot) ; rend (robot, états vus,
    mains à l'entrée en FERMETURE, roulis gauche le plus fermé commandé)."""
    horloge = [0.0]
    r = SimulatedRobot(objects=[SimulatedObject("carton", (0.30, 0.0, 0.90))] if objets else [],
                       horloge=lambda: horloge[0])
    r.start(fil=False)
    r.affaissement_rad = affaissement
    m = r.manipulation
    if reglages is not None:
        m.r = reglages

    def pas():
        horloge[0] += r._dt if pas_s is None else pas_s()
        r.pas()

    for _ in range(20):                    # 0,4 s du robot avant la prise
        pas()
    m.prendre(CARTON)
    etats, mains, roulis = [], None, []
    while m.etat not in (Etat.TENU, Etat.ECHEC) and horloge[0] < duree:
        if not etats or etats[-1] is not m.etat:
            etats.append(m.etat)
        if m.etat is Etat.FERMETURE and mains is None:
            mains = m._mains_bassin()
        roulis.append(m._q["left_shoulder_roll_joint"])
        pas()
    etats.append(m.etat)
    return r, etats, mains, min(roulis) if roulis else None


section("1. Le constat GPU : bras affaissés, borne q_vide atteinte avant les faces")
# Sans compensation (tolérances larges, aucune correction), comme sur le
# GPU : le roulis atteint q_vide alors que les mains n'ont pas touché.
# L'ancien critère (angle du roulis contre q_vide) concluait « rien entre
# les mains » ; le critère géométrique continue jusqu'au contact. Cette
# section imite le placement d'avant (tolérances larges) pour atteindre la
# fermeture : sans la règle des 7 cm derrière la face avant, venue après
# (recul_face_min_m), que l'affaissement non compensé enfreint (5,6 cm).
large = ReglagesPrise(tol_hauteur_m=0.12, tol_profondeur_m=0.06, corrections_max=0,
                      recul_face_min_m=0.0)
r, etats, mains, _ = prise(0.25, reglages=large)
m = r.manipulation
check("Mains 6 à 7 cm sous leurs cibles à la fermeture (affaissement non compensé)",
      mains is not None and Z_CIBLE - mains[0][2] > 0.05, str(mains))
check("Pas de « rien entre les mains » : la fermeture va jusqu'au contact, puis serre",
      "rien entre les mains" not in m.raison and Etat.SERRAGE in etats, f"{m.raison} {etats}")
r.__exit__(None, None, None)

section("2. Rien entre les mains : décidé sur la position des mains")
r, etats, mains, roulis_min = prise(0.11, objets=False)
m = r.manipulation
check("Bras affaissés, aucun carton : ÉCHEC « rien entre les mains », faces intérieures dites",
      m.etat is Etat.ECHEC and "rien entre les mains" in m.raison
      and "faces intérieures des mains" in m.raison, m.raison)
borne = m._cible_ferm["left"]["left_shoulder_roll_joint"]
check("… la consigne ne passe jamais la borne de fermeture (face commandée à largeur/2 - 2 cm)",
      roulis_min is not None and roulis_min >= borne - 1e-6, f"{roulis_min} {borne}")
check("… exemption levée, aucun serrage", r.guard._exemption is None
      and m.force_max_mesuree < R.force_serrage_n)
r.__exit__(None, None, None)

section("3. Affaissement compensé : mains au milieu des faces avant de fermer")
for aff in (0.11, 0.25):
    r, etats, mains, _ = prise(aff)
    m = r.manipulation
    ecart = None if mains is None else max(abs(mains[0][2] - Z_CIBLE), abs(mains[1][2] - Z_CIBLE))
    check(f"Affaissement de {aff} rad : mains à {100 * ecart:.1f} cm de la mi-hauteur à la "
          f"fermeture (1,5 au plus), après {m.stats.get('corrections_placement', 0)} "
          f"correction(s)", ecart is not None and ecart <= R.tol_hauteur_m
          and m.stats.get("corrections_placement", 0) >= 1, str(mains))
    check("… et la prise tient", m.etat is Etat.TENU, m.raison)
    check("… sans consigne refusée par le garde-fou", r.locomotion.stats["refused"] == 0)
    r.__exit__(None, None, None)

section("4. Pas irréguliers, comme sur une machine chargée")
# La prise suit son horloge : chaque pas dure ici de 20 à 100 ms (un robot
# dont la boucle de commande prend du retard), au hasard mais à graine fixe.
import random  # noqa: E402

for aff in (0.11, 0.25):
    alea = random.Random(7)
    r, etats, mains, _ = prise(aff, pas_s=lambda: alea.uniform(0.02, 0.10))
    m = r.manipulation
    ecart = None if mains is None else max(abs(mains[0][2] - Z_CIBLE), abs(mains[1][2] - Z_CIBLE))
    check(f"Affaissement de {aff} rad, pas de 20 à 100 ms : mains à "
          f"{100 * (ecart if ecart is not None else 0):.1f} cm de la mi-hauteur à la fermeture "
          f"(1,5 au plus), et la prise tient", ecart is not None and ecart <= R.tol_hauteur_m
          and m.etat is Etat.TENU, f"{mains} {m.raison}")
    check("… sans consigne refusée par le garde-fou", r.locomotion.stats["refused"] == 0)
    r.__exit__(None, None, None)

section("5. Compensation bornée")
r, etats, mains, _ = prise(0.30)
m = r.manipulation
check("Affaissement de plus de 10 cm : refus clair, avant toute fermeture",
      m.etat is Etat.ECHEC and "s'affaissent trop" in m.raison and Etat.FERMETURE not in etats,
      m.raison)
check("… exemption levée", r.guard._exemption is None)
r.__exit__(None, None, None)

print(f"\n{'=' * 66}\n  {'TOUT PASSE' if FAIL == 0 else f'{FAIL} ÉCHEC(S)'}\n{'=' * 66}")
sys.exit(1 if FAIL else 0)
