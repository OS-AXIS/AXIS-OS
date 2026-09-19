"""
AXIS-OS — Test de non-divergence RobotContext / HALBridgeClient
=====================================================================
Écrit en premier, délibérément AVANT toute correction — voir la
recommandation qui motive ce fichier : "le livrable de l'étape 0
n'est pas la classe, c'est le test". Une IRobotContext abstraite ne
résout rien à elle seule : deux classes peuvent hériter de la même
base et diverger quand même. Ce qui empêche vraiment le problème de
revenir, c'est ce test — il compare les surfaces publiques réelles
des deux implémentations et échoue si l'une a une méthode que l'autre
n'a pas, peu importe si elles héritent ou non d'une base commune.

RobotContext (axis_os/store/sdk/app_base.py) est utilisée par une app
qui tourne HORS sandbox. HALBridgeClient (axis_os/store/runtime/
hal_bridge.py) est utilisée par une app SANDBOXÉE — systématique
depuis le correctif C4 pour on_start()/on_stop()/run_mission(). Une
app ne doit JAMAIS avoir un comportement différent selon qu'elle
tourne sandboxée ou non — si une méthode existe d'un côté et pas de
l'autre, une app qui l'utilise fonctionne dans un contexte et casse
dans l'autre, de façon souvent invisible tant que personne ne teste
les deux chemins avec la même app.

Documenté comme trou trouvé la première fois par un audit externe :
RobotContext exposait 11 méthodes, HALBridgeClient seulement 8 (avant
ce commit) — get_joint_state, move_all et subscribe_alerts
manquantes côté sandbox.
"""

import sys
sys.path.insert(0, ".")

from axis_os.store.sdk.app_base import RobotContext
from axis_os.store.runtime.hal_bridge import HALBridgeClient

FAIL = 0


def check(description: str, condition: bool) -> None:
    global FAIL
    status = "✓" if condition else "✗"
    print(f"  [{status}] {description}")
    if not condition:
        FAIL += 1


def section(title: str) -> None:
    print(f"\n{'=' * 66}\n  {title}\n{'=' * 66}")


def public_methods(cls) -> set:
    """Méthodes publiques (pas préfixées par _) et appelables —
    exclut délibérément les attributs de données, propriétés non
    callable, et tout ce qui commence par un underscore (détails
    d'implémentation internes, jamais partie du contrat public que
    les apps consomment).

    Exclut aussi EXPLICITEMENT for_testing (classmethod ajoutée sur
    RobotContext seul) — pas une méthode d'API que le CODE d'une app
    appelle au runtime, contrairement à move_arms/get_imu/etc. C'est
    un point d'entrée de CONSTRUCTION réservé aux tests unitaires
    (voir sa docstring), qui n'existe délibérément QUE côté
    RobotContext puisque HALBridgeClient n'a jamais besoin d'un
    équivalent (les tests qui exercent le pont HAL construisent déjà
    HALBridgeClient directement, sans barrière de sécurité
    équivalente à celle de RobotContext — voir UnauthorizedContextError).
    """
    EXCLUDED = {"for_testing"}
    return {
        name for name in dir(cls)
        if not name.startswith("_") and name not in EXCLUDED
        and callable(getattr(cls, name, None))
    }


# ══════════════════════════════════════════════════════════════════

section("Non-divergence des surfaces publiques RobotContext / HALBridgeClient")

rc_methods = public_methods(RobotContext)
hb_methods = public_methods(HALBridgeClient)

only_in_rc = rc_methods - hb_methods
only_in_hb = hb_methods - rc_methods

check(f"Aucune méthode présente UNIQUEMENT dans RobotContext "
      f"(trouvé: {sorted(only_in_rc) or 'aucune'})",
      only_in_rc == set())
check(f"Aucune méthode présente UNIQUEMENT dans HALBridgeClient "
      f"(trouvé: {sorted(only_in_hb) or 'aucune'})",
      only_in_hb == set())

# ══════════════════════════════════════════════════════════════════

section("Non-divergence des TYPES DE RETOUR (méthodes de lecture)")

# La comparaison de surfaces ci-dessus compare des NOMS de méthodes.
# Elle a rattrapé move_all/get_joint_state/subscribe_alerts, mais elle
# est aveugle aux types : get_full_state() existait des deux côtés et
# rendait des ActuatorFeedback d'un côté, des dicts sérialisés de
# l'autre. Une app écrite contre la doc de RobotContext plantait alors
# UNIQUEMENT en sandbox — donc uniquement sur le vrai robot.
# Constaté réellement : AttributeError: 'dict' object has no attribute
# 'position'. D'où ce second contrôle.

from axis_os.hal.core.types import ActuatorFeedback, ActuatorState
from axis_os.store.runtime.hal_bridge import (
    _serialize_feedback, _deserialize_feedback,
)

_fb = ActuatorFeedback("right_knee_joint", ActuatorState.ACTIVE,
                       1.23, 0.5, 42.0, 3.1, 37.5, 48.2, 7)
_round = _deserialize_feedback("right_knee_joint", _serialize_feedback(_fb))

check("Le pont reconstruit un ActuatorFeedback, pas un dict",
      isinstance(_round, ActuatorFeedback))

_fields = ["joint_id", "state", "position", "velocity", "torque",
           "current", "temperature", "voltage", "error_code"]
_lost = [f for f in _fields if getattr(_fb, f) != getattr(_round, f)]
check(f"Aller-retour sérialisation sans perte sur les {len(_fields)} champs "
      f"(perdus: {_lost or 'aucun'})", not _lost)

check("get_full_state() annote Dict[str, ActuatorFeedback] des deux côtés",
      "ActuatorFeedback" in str(
          HALBridgeClient.get_full_state.__annotations__.get("return", "")))

# ══════════════════════════════════════════════════════════════════

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)

if FAIL == 0:
    print("\n  ✓ RobotContext et HALBridgeClient exposent exactement la même")
    print("    surface publique — une app se comporte identiquement,")
    print("    sandboxée ou non.")
else:
    print("\n  ✗ Une app utilisant les méthodes manquantes d'un côté aurait")
    print("    un comportement différent selon qu'elle est sandboxée ou non.")

sys.exit(0 if FAIL == 0 else 1)
