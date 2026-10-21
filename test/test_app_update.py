"""
AXIS-OS — Tests de mise à jour d'app (update_app)
======================================================
Valide PlayStoreRuntime.update_app() — trouvé nécessaire : rien
n'empêchait install() d'écraser silencieusement une app déjà
installée, sans l'arrêter proprement, sans vérifier le sens de la
mise à jour, et en perdant l'état de facturation.

Inspiré du cycle de vie réel d'Android (PackageManager) :
  - INSTALL_FAILED_VERSION_DOWNGRADE : un downgrade est un vrai
    risque de sécurité (revenir à une version avec une faille déjà
    corrigée) — refusé par défaut, testé explicitement en section 2.
  - ACTION_MY_PACKAGE_REPLACED : Android arrête l'ancien process avant
    de remplacer le paquet — même principe en section 3 (stop_app()
    réutilisé avant remplacement).
"""

import sys
import threading
import time
sys.path.insert(0, ".")

from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer
from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL
from axis_os.hal.watchdog.system import WatchdogSystem
from axis_os.hal.core.types import ActuatorCommand, ControlMode
from axis_os.store.runtime.play_store import PlayStoreRuntime, accepter_tout_SIMULATION
from axis_os.store.runtime.profil import Profil
from axis_os.store.sdk.manifest import AppManifest, AppCategory, PricingModel, Permission
from axis_os.store.sdk.app_base import RobotApp, MissionResult

FAIL = 0


def check(description: str, condition: bool) -> None:
    global FAIL
    status = "✓" if condition else "✗"
    print(f"  [{status}] {description}")
    if not condition:
        FAIL += 1


def section(title: str) -> None:
    print(f"\n{'=' * 66}\n  {title}\n{'=' * 66}")


def make_runtime():
    robot = NormalizedRobotHAL(
        UnitreeG1HAL(f"g1_update_{id(object())}"), build_g1_normalizer(),
    )
    robot.initialize_all()
    wd = WatchdogSystem(robot)
    return PlayStoreRuntime(robot, wd, confirmer_permissions=accepter_tout_SIMULATION,
                            profil=Profil.SIMULATION)


def make_app(app_id, version, *, pricing_model=PricingModel.FREE,
             base_price_eur=0.0, moves_robot=False):
    """
    Classes définies au NIVEAU MODULE (_ParametrizedTestApp), pas à
    l'intérieur de cette fonction — trouvé nécessaire en testant ce
    fichier : une classe locale à une fonction n'est PAS picklable
    (`Can't pickle local object`), donc échoue au sandboxing
    (SandboxRunner pickle l'app pour son sous-process, voir
    packaging.py pour le même problème déjà rencontré et corrigé
    pour les apps chargées depuis un fichier chiffré). Ce n'est pas
    un bug de update_app() lui-même, mais de la construction du
    test — toute vraie app définie normalement dans un fichier .py
    (comme les 3 apps de référence du projet) est déjà picklable
    sans y penser.
    """
    manifest_kwargs = dict(
        app_id=app_id, name="T", version=version, author="D",
        author_email="d@d.com", description="d",
        category=AppCategory.UTILITIES, pricing_model=pricing_model,
    )
    if pricing_model != PricingModel.FREE:
        manifest_kwargs["base_price_eur"] = base_price_eur
        manifest_kwargs["permissions"] = [Permission.ACTUATORS_WRITE_ARMS]
    return _ParametrizedTestApp(manifest_kwargs, moves_robot=moves_robot)


class _ParametrizedTestApp(RobotApp):
    """Voir make_app() — définie au niveau module pour rester
    picklable, ses paramètres réels sont transmis via __init__."""

    def __init__(self, manifest_kwargs: dict, *, moves_robot: bool = False):
        self._manifest_kwargs = manifest_kwargs
        self._moves_robot = moves_robot

    def manifest(self):
        return AppManifest(**self._manifest_kwargs)

    def on_start(self, ctx):
        return True

    def on_stop(self, ctx):
        pass

    def run_mission(self, ctx, params):
        if self._moves_robot:
            ctx.move_arms({
                "left_elbow_joint": ActuatorCommand(
                    mode=ControlMode.POSITION, position=0.1)
            })
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0)


# ══════════════════════════════════════════════════════════════════

section("1 — Cycle normal : mise à jour préserve l'état de facturation")

runtime1 = make_runtime()
app_v1 = make_app("com.dev.myapp_w1", "1.0.0", pricing_model=PricingModel.ONE_TIME,
                  base_price_eur=1.40, moves_robot=True)
runtime1.install(app_v1)
runtime1.start_app("com.dev.myapp_w1")
runtime1.run_mission("com.dev.myapp_w1", params={})
check("Facturée une fois après la première mission",
      len(runtime1._billing) == 1)
check("one_time_billed vrai avant mise à jour",
      runtime1._apps["com.dev.myapp_w1"].one_time_billed is True)

app_v2 = make_app("com.dev.myapp_w1", "2.0.0", pricing_model=PricingModel.ONE_TIME,
                  base_price_eur=1.40)
result = runtime1.update_app(app_v2)
check("update_app() vers une version supérieure réussit", result is True)

entry1 = runtime1._apps["com.dev.myapp_w1"]
check("Version bien mise à jour", entry1.manifest.version == "2.0.0")
check("is_started == False après mise à jour (pas de redémarrage automatique)",
      entry1.is_started is False)
check("CRITIQUE : one_time_billed PRÉSERVÉ après mise à jour "
      "(déjà payée, ne doit pas être refacturée)",
      entry1.one_time_billed is True)

# ══════════════════════════════════════════════════════════════════

section("2 — CRITIQUE : downgrade refusé par défaut (sécurité)")

runtime2 = make_runtime()
runtime2.install(make_app("com.dev.app_w2", "2.0.0"))
result_downgrade = runtime2.update_app(make_app("com.dev.app_w2", "1.0.0"))
check("Downgrade (2.0.0 -> 1.0.0) refusé par défaut", result_downgrade is False)
check("La version installée reste inchangée après le refus",
      runtime2._apps["com.dev.app_w2"].manifest.version == "2.0.0")

result_downgrade_allowed = runtime2.update_app(
    make_app("com.dev.app_w2", "1.0.0"), allow_downgrade=True,
)
check("Downgrade explicitement autorisé (allow_downgrade=True) réussit",
      result_downgrade_allowed is True)
check("La version est bien redescendue à 1.0.0",
      runtime2._apps["com.dev.app_w2"].manifest.version == "1.0.0")

# ══════════════════════════════════════════════════════════════════

section("3 — Arrêt propre de l'ancienne version si elle est démarrée")

runtime3 = make_runtime()
app3_v1 = make_app("com.dev.app_w3", "1.0.0")
runtime3.install(app3_v1)
runtime3.start_app("com.dev.app_w3")
check("App démarrée avant mise à jour (précondition du test)",
      runtime3._apps["com.dev.app_w3"].is_started is True)

runtime3.update_app(make_app("com.dev.app_w3", "2.0.0"))
check("is_started == False après mise à jour (arrêtée proprement, "
      "pas laissée dans un état incohérent)",
      runtime3._apps["com.dev.app_w3"].is_started is False)

# ══════════════════════════════════════════════════════════════════

section("4 — CRITIQUE : mise à jour refusée pendant une mission en cours")

runtime4 = make_runtime()

class _SlowApp(RobotApp):
    def manifest(self):
        return AppManifest(
            app_id="com.dev.slow_w4", name="T", version="1.0.0", author="D",
            author_email="d@d.com", description="d", category=AppCategory.UTILITIES,
        )
    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass
    def run_mission(self, ctx, params):
        time.sleep(0.4)
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.4)


runtime4.install(_SlowApp())
runtime4.start_app("com.dev.slow_w4")

mission_thread = threading.Thread(
    target=lambda: runtime4.run_mission("com.dev.slow_w4", params={})
)
mission_thread.start()
time.sleep(0.1)  # laisse la mission vraiment prendre le verrou

result_during_mission = runtime4.update_app(make_app("com.dev.slow_w4", "2.0.0"))
mission_thread.join()

check("CRITIQUE : update_app() refusé pendant qu'une mission tourne "
      "réellement (verrou pris par un vrai thread concurrent)",
      result_during_mission is False)
check("La version n'a pas changé pendant la mission",
      runtime4._apps["com.dev.slow_w4"].manifest.version == "1.0.0")

# ══════════════════════════════════════════════════════════════════

section("5 — Cas d'erreur : app jamais installée, version invalide")

runtime5 = make_runtime()
result_missing = runtime5.update_app(make_app("com.dev.never_installed_w5", "1.0.0"))
check("update_app() sur un app_id jamais installé échoue proprement",
      result_missing is False)

runtime5.install(make_app("com.dev.badversion_w5", "1.0.0"))


class _BadVersionApp(RobotApp):
    def manifest(self):
        return AppManifest(
            app_id="com.dev.badversion_w5", name="T", version="pas-une-version",
            author="D", author_email="d@d.com", description="d",
            category=AppCategory.UTILITIES,
        )
    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass
    def run_mission(self, ctx, params): pass


result_bad_version = runtime5.update_app(_BadVersionApp())
check("update_app() avec une chaîne de version invalide échoue proprement",
      result_bad_version is False)

# ══════════════════════════════════════════════════════════════════

section("6 — SUBSCRIPTION : la fenêtre de facturation survit à une mise à jour")

runtime6 = make_runtime()
app6_v1 = make_app("com.dev.sub_w6", "1.0.0", pricing_model=PricingModel.SUBSCRIPTION,
                   base_price_eur=1.40, moves_robot=True)
runtime6.install(app6_v1)
runtime6.start_app("com.dev.sub_w6")
runtime6.run_mission("com.dev.sub_w6", params={})
check("Facturée une fois (SUBSCRIPTION, première mission)",
      len(runtime6._billing) == 1)

last_bill_time_before = runtime6._apps["com.dev.sub_w6"].last_subscription_bill_time_s
check("last_subscription_bill_time_s bien renseigné avant mise à jour",
      last_bill_time_before is not None)

runtime6.update_app(make_app("com.dev.sub_w6", "2.0.0",
                             pricing_model=PricingModel.SUBSCRIPTION,
                             base_price_eur=1.40))
last_bill_time_after = runtime6._apps["com.dev.sub_w6"].last_subscription_bill_time_s
check("CRITIQUE : la fenêtre de facturation SUBSCRIPTION est préservée "
      "à l'identique après mise à jour (pas relancée à zéro)",
      last_bill_time_after == last_bill_time_before)

# ══════════════════════════════════════════════════════════════════

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)

if FAIL == 0:
    print("\n  ✓ Mise à jour préserve l'état de facturation (ONE_TIME et")
    print("    SUBSCRIPTION)")
    print("  ✓ Downgrade refusé par défaut, autorisable explicitement")
    print("  ✓ L'ancienne version est arrêtée proprement avant remplacement")
    print("  ✓ CRITIQUE : refusée pendant une mission réellement en cours")
    print("    (vérifié avec un vrai thread concurrent)")

sys.exit(0 if FAIL == 0 else 1)
