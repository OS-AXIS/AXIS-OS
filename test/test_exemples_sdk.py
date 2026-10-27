"""
Les trois exemples du SDK (examples/) : chacun lance son propre test, dans
un processus à part (chaque exemple a son app.py), sur le robot simulé
léger, et passe `axis validate`. Un exemple qui ne marche plus est un
exemple qui trompe : il échoue ici.
"""
import subprocess
import sys
from pathlib import Path

RACINE = Path(__file__).resolve().parents[1]
FAIL = 0


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"\n{detail}" if detail and not cond else ""))
    if not cond:
        FAIL += 1


for nom in ("hello_robot", "fetch", "teleop_local"):
    test = RACINE / "examples" / nom / "test_app.py"
    r = subprocess.run([sys.executable, str(test)], cwd=RACINE, capture_output=True,
                       text=True, timeout=300)
    check(f"examples/{nom} : son test passe", r.returncode == 0 and f"✓ {nom}" in r.stdout,
          (r.stdout + r.stderr)[-3000:])
    app = (RACINE / "examples" / nom / "app.py").read_text()
    check(f"examples/{nom} : déclare min_sdk_version", "min_sdk_version=" in app)
    r = subprocess.run([sys.executable, "-m", "axis_os.cli.main", "validate",
                        str(RACINE / "examples" / nom)], cwd=RACINE, capture_output=True,
                       text=True, timeout=300)
    check(f"examples/{nom} : axis validate le dit prêt (code 0)", r.returncode == 0,
          r.stdout + r.stderr)

print(f"\n  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
sys.exit(0 if FAIL == 0 else 1)
