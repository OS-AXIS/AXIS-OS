"""
Hygiène du dépôt (audit du 2026-10-10, M7).

  - La suite de tests ne modifie plus de fichier SUIVI par git : les
    journaux d'exécution (./logs/*.jsonl, écrits par structured_log) ne sont
    plus suivis, et sont ignorés. Le dossier logs/ reste (logs/.gitkeep) :
    systemd l'exige (ReadWritePaths=/opt/axis-os/logs).
  - requirements.txt déclare les dépendances sans lesquelles des tests
    échouent sur une installation neuve (msgpack, msgpack-numpy).
"""
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE))

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def git(*args):
    return subprocess.run(["git", "-C", str(RACINE), *args], capture_output=True, text=True)


print("\n  Journaux d'exécution hors du dépôt\n")
def dans_son_depot() -> bool:
    """Vrai seulement si RACINE est la racine de SON dépôt git — et non un
    dossier non suivi d'un autre dépôt (copie par archive dans un monorepo),
    où les vérifications ci-dessous échoueraient à tort (revue)."""
    if not shutil.which("git"):
        return False
    top = git("rev-parse", "--show-toplevel")
    return top.returncode == 0 and Path(top.stdout.strip()).resolve() == RACINE


if dans_son_depot():
    suivis = git("ls-files", "logs").stdout.split()
    check(f"Seul logs/.gitkeep est suivi dans logs/ ({suivis})", suivis == ["logs/.gitkeep"])
    for nom in ("estop.jsonl", "safety_violation.jsonl", "watchdog_alert.jsonl"):
        check(f"logs/{nom} ignoré par git", git("check-ignore", "-q", f"logs/{nom}").returncode == 0)

    # Écriture réelle d'un journal, comme le font les tests, depuis la racine.
    code = ("from axis_os.hal.core.structured_log import StructuredLogger, LogCategory; "
            "StructuredLogger().log(LogCategory.ESTOP, {'test': 'hygiene'})")
    r = subprocess.run([sys.executable, "-c", code], cwd=str(RACINE), capture_output=True,
                       text=True)
    # Fichiers SUIVIS modifiés dans l'arbre de travail (indépendant de l'index).
    modifies = git("ls-files", "-m", "--", "logs").stdout.strip()
    nouveaux = git("ls-files", "--others", "--exclude-standard", "--", "logs").stdout.strip()
    check(f"Un journal écrit depuis la racine ne modifie aucun fichier suivi, n'en crée "
          f"aucun à suivre (modifiés : « {modifies} », nouveaux : « {nouveaux} »)",
          r.returncode == 0 and modifies == "" and nouveaux == "")
else:
    print("  [—] pas le dépôt git d'AXIS-OS ici : vérification sautée")

print("\n  requirements.txt\n")
req = (RACINE / "requirements.txt").read_text(encoding="utf-8")
for paquet in ("msgpack", "msgpack-numpy", "pyzmq"):
    check(f"{paquet} déclaré (ligne non commentée)",
          re.search(rf"^{re.escape(paquet)}\s*(==|>=)", req, re.M) is not None)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
