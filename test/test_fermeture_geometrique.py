"""
Fermeture de la prise mesurée sur des bras qui s'affaissent.

Constat GPU (2026-10-29, evaluer_vla.sh 5 1 --reflexes) : 0/5, « rien
entre les mains » ×5. Mains ~3,3 cm plus bas que leurs cibles (bras qui
s'affaissent sous leur poids) ; au refus, leurs faces intérieures étaient
encore à ~0,5 cm des faces du carton : l'angle mesuré du roulis avait
atteint la borne q_vide, calculée pour des bras à la hauteur visée.

Robot simulé léger (axis_os.testing) : `affaissement_rad` ajoute au
tangage d'épaule mesuré un écart à sa consigne (main plus basse).
"""
import sys
import time
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


def prise(affaissement, objets=True, reglages=None, duree=15.0):
    """Lance une prise ; rend (robot, états vus, mains à l'entrée en
    FERMETURE, roulis gauche le plus fermé commandé)."""
    r = SimulatedRobot(objects=[SimulatedObject("carton", (0.30, 0.0, 0.90))] if objets else [],
                       time_scale=2.0)
    r.__enter__()
    r.affaissement_rad = affaissement
    m = r.manipulation
    if reglages is not None:
        m.r = reglages
    time.sleep(0.2)
    m.prendre(CARTON)
    etats, mains, roulis = [], None, []
    t0 = time.monotonic()
    while m.etat not in (Etat.TENU, Etat.ECHEC) and time.monotonic() - t0 < duree:
        if not etats or etats[-1] is not m.etat:
            etats.append(m.etat)
        if m.etat is Etat.FERMETURE and mains is None:
            mains = m._mains_bassin()
        roulis.append(m._q["left_shoulder_roll_joint"])
        time.sleep(0.005)
    etats.append(m.etat)
    return r, etats, mains, min(roulis) if roulis else None


section("1. Le constat GPU : bras affaissés, borne q_vide atteinte avant les faces")
# Sans compensation (tolérances larges, aucune correction), comme sur le
# GPU : le roulis atteint q_vide alors que les mains n'ont pas touché.
# L'ancien critère (angle du roulis contre q_vide) concluait « rien entre
# les mains » ; le critère géométrique continue jusqu'au contact.
large = ReglagesPrise(tol_hauteur_m=0.12, tol_profondeur_m=0.06, corrections_max=0)
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
borne = m._plan["q_vide"]["left_shoulder_roll_joint"] - R.course_sup_rad
check("… la consigne ne passe jamais la borne de fermeture (q_vide + course_sup_rad)",
      roulis_min is not None and roulis_min >= borne - 1e-6, f"{roulis_min} {borne}")
check("… exemption levée, aucun serrage", r.guard._exemption is None
      and m.effort_max_mesure < R.effort_serrage_nm)
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

section("4. Compensation bornée")
r, etats, mains, _ = prise(0.30)
m = r.manipulation
check("Affaissement de plus de 10 cm : refus clair, avant toute fermeture",
      m.etat is Etat.ECHEC and "s'affaissent trop" in m.raison and Etat.FERMETURE not in etats,
      m.raison)
check("… exemption levée", r.guard._exemption is None)
r.__exit__(None, None, None)

print(f"\n{'=' * 66}\n  {'TOUT PASSE' if FAIL == 0 else f'{FAIL} ÉCHEC(S)'}\n{'=' * 66}")
sys.exit(1 if FAIL else 0)
