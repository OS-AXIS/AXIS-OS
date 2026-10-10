"""
Table de conventions — une entrée est NON VÉRIFIÉE sauf déclaration
explicite verifiee=True (audit du 2026-10-10, M5).

Avant : getattr(c, "verifiee", True) — un objet sans ce champ était tenu
pour vérifié, donc accepté sur un vrai robot ; et JointConvention avait
lui-même verifiee=True par défaut. Désormais, « vérifiée » s'écrit. Le refus sur robot réel des
conventions non vérifiées (UnitreeG1HAL.initialize_all) reste testé par
test_g1_29dof.py ; ici, on vérifie ce qui l'alimente.
"""
import sys
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.drivers.unitree_g1.convention_validator import validate_convention_table  # noqa: E402
from axis_os.hal.drivers.unitree_g1.conventions import (  # noqa: E402
    G1_JOINT_CONVENTIONS, JointConvention, conventions_pour,
)

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


@dataclass
class SansChamp:
    """Convention construite à la main, sans le champ « verifiee »."""
    joint_id: str
    sign: float = 1.0
    offset_rad: float = 0.0
    positive_label: str = "p"
    negative_label: str = "n"


@dataclass
class ChampFlou(SansChamp):
    verifiee: object = "oui"


print("\n  Conventions : vérifiée seulement si déclarée vérifiée\n")
r = validate_convention_table({"a": SansChamp("a")}, ["a"])
check("Entrée SANS champ verifiee : NON vérifiée (avant : tenue pour vérifiée)",
      r.valid and r.non_verifiees == ["a"])
r = validate_convention_table({"a": ChampFlou("a"), "b": ChampFlou("b", verifiee=1)}, ["a", "b"])
check("verifiee = « oui » ou 1 : NON vérifiée (seul True compte)",
      sorted(r.non_verifiees) == ["a", "b"])
r = validate_convention_table({"a": JointConvention("a", 1.0, 0.0, "p", "n", "")}, ["a"])
check("JointConvention construite à la main SANS verifiee : NON vérifiée (défaut False, "
      "trouvé à la revue : le défaut était True)", r.non_verifiees == ["a"])
r = validate_convention_table({"a": JointConvention("a", 1.0, 0.0, "p", "n", "", verifiee=True)},
                              ["a"])
check("verifiee=True écrit explicitement : vérifiée", r.non_verifiees == [])
r = validate_convention_table(G1_JOINT_CONVENTIONS, list(G1_JOINT_CONVENTIONS))
check("Table du 23 DOF : toujours 23/23 vérifiées", r.valid and r.non_verifiees == [])
r = validate_convention_table(conventions_pour("29dof"), list(conventions_pour("29dof")))
check(f"Table du 29 DOF : toujours exactement 6 non vérifiées ({len(r.non_verifiees)})",
      len(r.non_verifiees) == 6)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
