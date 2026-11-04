"""
Tri de colis, toute la pile logicielle, sur plusieurs graines.

Pourquoi : test_tri_pile_logicielle.py ne tourne que sur la graine 3. Lancé
sur les graines 1 à 12, main échouait sur deux d'entre elles (4 et 12) : le
gardien de proximité ralentissait le robot sous sa vitesse minimale de
marche, et les reculs se déclaraient « BLOQUÉ » (voir
test_attente_securite.py). Avec une seule graine, ce défaut passait
inaperçu en CI.

Ici, le même script, inchangé, pour chaque graine de 1 à 12 sauf la 3 (déjà
lancée par test_tri_pile_logicielle.py), plusieurs à la fois (un processus
par cœur, quatre au plus). Chaque graine change les cartons, le bruit de la
perception, l'erreur d'exécution de la marche et les prises ratées ; les
personnes suivent toujours le même trajet. Environ 40 s par graine sur un
cœur : 3 min sur quatre cœurs.

Usage : python3 test/test_tri_pile_graines.py [graine ...]
"""
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

RACINE = Path(__file__).resolve().parents[1]
SCRIPT = RACINE / "test" / "test_tri_pile_logicielle.py"
GRAINES = [int(g) for g in sys.argv[1:]] or [g for g in range(1, 13) if g != 3]
DUREE_S = 600            # celle du test par défaut
DELAI_S = 600            # par graine : au-delà, quelque chose est bloqué


def lancer(graine):
    t0 = time.monotonic()
    try:
        r = subprocess.run([sys.executable, str(SCRIPT), str(DUREE_S), str(graine)],
                           cwd=str(RACINE), capture_output=True, text=True, timeout=DELAI_S)
        sortie, code = r.stdout, r.returncode
    except subprocess.TimeoutExpired:
        sortie, code = "", None
    lignes = sortie.splitlines()
    bilan = next((l.strip() for l in lignes if " triés (" in l), "bilan absent")
    echecs = [l.strip() for l in lignes if "[✗]" in l]
    return graine, code, bilan, echecs, time.monotonic() - t0


print(f"\n{'=' * 66}\n  Pile logicielle complète, graines {GRAINES}, {DUREE_S} s simulées "
      f"chacune\n{'=' * 66}")
travailleurs = max(1, min(4, os.cpu_count() or 1))
with ThreadPoolExecutor(travailleurs) as pool:
    resultats = sorted(pool.map(lancer, GRAINES))
ratees = 0
for graine, code, bilan, echecs, duree in resultats:
    ok = code == 0
    ratees += not ok
    etat = "délai dépassé" if code is None else ("" if ok else f"code {code}")
    print(f"  [{'✓' if ok else '✗'}] graine {graine:2d} : {bilan} ({duree:.0f} s)"
          + (f" — {etat}" if etat else ""))
    for e in echecs:
        print(f"        {e}")
print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if not ratees else str(ratees) + ' graine(s) en échec'}")
print("=" * 66)
sys.exit(0 if not ratees else 1)
