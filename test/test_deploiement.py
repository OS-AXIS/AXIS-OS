"""
Scripts de déploiement (deploy/) — vérifications statiques, sans GPU.

Ces scripts tournent sur des VM Vast.ai (Ubuntu 22.04) : on ne peut pas les
exécuter ici. On vérifie ce qui peut l'être sans machine :
  - chaque script est syntaxiquement valide pour bash ;
  - les versions épinglées concordent avec requirements.txt — une version
    épinglée à un endroit et libre à un autre, c'est la version du jour sur
    chaque nouvelle machine (cause de la panne de perception de transformers
    5.x, audit du 2026-10-10).
"""
import re
import shutil
import subprocess
import sys
from pathlib import Path

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE))

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


SCRIPTS = sorted((RACINE / "deploy").rglob("*.sh"))
INSTALLER = (RACINE / "deploy/machine_gpu/installer.sh").read_text(encoding="utf-8")
REQUIREMENTS = (RACINE / "requirements.txt").read_text(encoding="utf-8")


def version_requirements(paquet):
    m = re.search(rf"^{re.escape(paquet)}==([\w.]+)\s*$", REQUIREMENTS, re.M)
    return m.group(1) if m else None


section("1 — Syntaxe bash de chaque script")

if shutil.which("bash"):
    check(f"{len(SCRIPTS)} scripts trouvés dans deploy/", len(SCRIPTS) >= 10)
    for s in SCRIPTS:
        r = subprocess.run(["bash", "-n", str(s)], capture_output=True, text=True)
        check(f"bash -n {s.relative_to(RACINE)}", r.returncode == 0)
else:
    print("  [—] bash absent : syntaxe non vérifiable ici")

section("2 — transformers : même version épinglée partout")

v = version_requirements("transformers")
check(f"requirements.txt épingle transformers exactement ({v})", v is not None)
installs = re.findall(r"pip install[^\n]*(?:\\\n[^\n]*)*", INSTALLER)
mentions = [m for i in installs for m in re.findall(r"transformers[^\s\"']*", i)]
check(f"installer.sh installe transformers=={v} ({mentions})",
      mentions == [f"transformers=={v}"])

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
