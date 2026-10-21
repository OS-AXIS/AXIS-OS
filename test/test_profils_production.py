"""
Profils du Play Store (simulation, développement, production) et
ACTUATORS_AI_DRIVEN vérifiée à l'exécution.

Vérifié ici :
  1. le profil est obligatoire et doit être un Profil (jamais deviné) ;
  2. SIMULATION est refusé sur un robot réel (G1 en DDS) ;
  3. PRODUCTION refuse de démarrer sans signature obligatoire, sans fichier
     de clés approuvées (ou avec un fichier mal formé ou modifiable par
     d'autres), ou avec accepter_tout_SIMULATION ;
  4. PRODUCTION refuse d'installer un objet app déjà chargé dans l'OS ;
  5. PRODUCTION, install_source : app non signée, mal signée ou signée par
     une clé non approuvée -> refusée ; signée par une clé approuvée ->
     installée, sans que son code s'exécute dans le processus de l'OS, et
     ses permissions DANGEROUS passent par la confirmation ;
  6. une app qui demande au pont de changer de profil est refusée, et le
     profil ne change pas ;
  7. SIMULATION et DÉVELOPPEMENT signalent leurs facilités dans le journal ;
  8. une consigne produite par une action d'IA du SDK exige
     ACTUATORS_AI_DRIVEN, en direct comme depuis le processus de l'app.
"""
import contextlib
import io
import os
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer  # noqa: E402
from axis_os.hal.core.normalized_robot import NormalizedRobotHAL  # noqa: E402
from axis_os.hal.core.types import ActuatorCommand, ControlMode, HALStatus  # noqa: E402
from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL  # noqa: E402
from axis_os.hal.watchdog.system import WatchdogSystem  # noqa: E402
from axis_os.store.runtime.play_store import (  # noqa: E402
    PlayStoreRuntime, accepter_tout_SIMULATION,
)
from axis_os.store.runtime.profil import Profil  # noqa: E402
from axis_os.store.sdk.ai.connector import VLAAction  # noqa: E402
from axis_os.store.sdk.app_base import (  # noqa: E402
    MissionResult, PermissionError as SdkPermissionError, RobotApp, RobotContext,
)
from axis_os.store.sdk.manifest import AppCategory, AppManifest, Permission  # noqa: E402
from axis_os.store.sdk.signing import generate_developer_keypair, sign_app_code  # noqa: E402

FAIL = 0


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


N = [0]


def robot(**kw):
    N[0] += 1
    with contextlib.redirect_stdout(io.StringIO()):
        r = NormalizedRobotHAL(UnitreeG1HAL(f"g1_profil_{N[0]}", **kw), build_g1_normalizer())
        r.initialize_all()
    return r


def runtime(r=None, **kw):
    r = r or robot()
    sortie = io.StringIO()
    with contextlib.redirect_stdout(sortie):
        rt = PlayStoreRuntime(r, WatchdogSystem(r), **kw)
    return rt, sortie.getvalue()


def refuse(label, exc_attendue, **kw):
    try:
        runtime(**kw)
        check(label, False, "aucune exception")
    except exc_attendue as exc:
        check(f"{label} ({str(exc)[:60]}…)", True)


def silencieux(f, *a, **kw):
    sortie = io.StringIO()
    with contextlib.redirect_stdout(sortie):
        r = f(*a, **kw)
    return r, sortie.getvalue()


# Clés de test, créées pour ce script et jamais écrites ailleurs que dans
# un dossier temporaire.
APPROUVEE = generate_developer_keypair()
INCONNUE = generate_developer_keypair()
DOSSIER = Path(tempfile.mkdtemp(prefix="axis_profils_"))
CLES = DOSSIER / "cles_approuvees.txt"
CLES.write_text(f"# clés de test\n{APPROUVEE.public_key_b64}  Développeur de test\n",
                encoding="utf-8")
os.chmod(CLES, 0o600)

SOURCE_APP = '''
from axis_os.hal.core.types import ActuatorCommand, ControlMode
from axis_os.store.sdk.app_base import MissionResult, RobotApp
from axis_os.store.sdk.manifest import AppCategory, AppManifest, Permission

class AppSignee(RobotApp):
    def manifest(self):
        return AppManifest(app_id="com.test.signee", name="Signée", version="1.0.0",
                           author="T", author_email="t@t.com", description="d",
                           category=AppCategory.UTILITIES,
                           permissions=[Permission.ACTUATORS_WRITE_ARMS])

    def on_start(self, ctx):
        return True

    def on_stop(self, ctx):
        pass

    def run_mission(self, ctx, params):
        if params.get("changer_profil"):
            try:
                ctx._call("changer_profil", {"profil": "DEVELOPPEMENT"})
                refus = None
            except Exception as exc:
                refus = str(exc)
            return MissionResult(success=refus is not None, mission_id="", app_id="",
                                 duration_s=0.0, error_message=refus)
        statuts = ctx.move_arms({"left_elbow_joint": ActuatorCommand(
            mode=ControlMode.POSITION, position=0.05)})
        return MissionResult(success=all(s.name == "OK" for s in statuts.values()),
                             mission_id="", app_id="", duration_s=0.0, items_processed=1)
'''

CONFIRMATIONS = []


def confirmer_test(manifest, permissions):
    """Tient le rôle de l'opérateur humain : note la demande et accepte."""
    CONFIRMATIONS.append([p.name for p in permissions])
    return True


section("1. Le profil est obligatoire et explicite")
try:
    runtime()
    check("Sans profil : refusé", False)
except TypeError:
    check("Sans profil : refusé (TypeError)", True)
refuse("Profil donné en chaîne (« production ») : refusé", TypeError, profil="production")

section("2. SIMULATION refusé sur un robot réel")
# Pas d'initialize_all() : il ouvrirait le bus DDS (unitree_sdk2py).
reel = NormalizedRobotHAL(UnitreeG1HAL("g1_profil_reel", backend="dds", network_interface="lo"),
                          build_g1_normalizer())
try:
    runtime(reel, profil=Profil.SIMULATION)
    check("Robot réel (G1 en DDS) en SIMULATION : refusé", False)
except ValueError as exc:
    check(f"Robot réel (G1 en DDS) en SIMULATION : refusé ({str(exc)[:40]}…)", True)

section("3. PRODUCTION : configuration non sûre refusée au démarrage")
refuse("require_signed_apps=False", ValueError, profil=Profil.PRODUCTION,
       require_signed_apps=False, cles_approuvees=CLES)
refuse("Sans fichier de clés approuvées", ValueError, profil=Profil.PRODUCTION)
refuse("accepter_tout_SIMULATION", ValueError, profil=Profil.PRODUCTION,
       cles_approuvees=CLES, confirmer_permissions=accepter_tout_SIMULATION)
ouvert = DOSSIER / "cles_ouvertes.txt"
ouvert.write_text(CLES.read_text(encoding="utf-8"), encoding="utf-8")
os.chmod(ouvert, 0o666)
refuse("Fichier de clés modifiable par les autres utilisateurs", ValueError,
       profil=Profil.PRODUCTION, cles_approuvees=ouvert)
mauvais = DOSSIER / "cles_mal_formees.txt"
mauvais.write_text("pas-une-cle  Quelqu'un\n", encoding="utf-8")
os.chmod(mauvais, 0o600)
refuse("Fichier de clés mal formé", ValueError, profil=Profil.PRODUCTION, cles_approuvees=mauvais)
prod, journal = runtime(profil=Profil.PRODUCTION, cles_approuvees=CLES,
                        confirmer_permissions=confirmer_test)
check("Configuration complète : PRODUCTION démarre", prod is not None)
check("… sans avertissement de profil permissif", "[PROFIL" not in journal)


class AppObjet(RobotApp):
    def manifest(self):
        return AppManifest(app_id="com.test.objet", name="Objet", version="1.0.0",
                           author="T", author_email="t@t.com", description="d",
                           category=AppCategory.UTILITIES)

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0)


section("4. PRODUCTION : pas d'objet app chargé dans l'OS")
ok, sortie = silencieux(prod.install, AppObjet())
check("install(objet) refusé en production", ok is False and "install_source" in sortie)

section("5. PRODUCTION : installation depuis la source signée")
signature = sign_app_code(SOURCE_APP, APPROUVEE.private_key_b64)
ok, sortie = silencieux(prod.install_source, SOURCE_APP + "\n# modifié\n",
                        code_signature=signature,
                        developer_public_key=APPROUVEE.public_key_b64, origine="modifiee")
check("Source modifiée après signature : refusée", ok is False and "Signature invalide" in sortie)
ok, sortie = silencieux(prod.install_source, SOURCE_APP,
                        code_signature=sign_app_code(SOURCE_APP, INCONNUE.private_key_b64),
                        developer_public_key=INCONNUE.public_key_b64, origine="inconnue")
check("Signée par une clé non approuvée : refusée", ok is False and "non approuvée" in sortie)
modules_avant = set(sys.modules)
ok, sortie = silencieux(prod.install_source, SOURCE_APP, code_signature=signature,
                        developer_public_key=APPROUVEE.public_key_b64, origine="signee")
check("Signée par une clé approuvée : installée", ok is True, sortie[-300:])
check("… le code de l'app n'a pas été chargé dans le processus de l'OS",
      not [m for m in set(sys.modules) - modules_avant if "signee" in m or m.startswith("axis_app_")])
check("… la permission DANGEROUS a été soumise à confirmation",
      CONFIRMATIONS == [["ACTUATORS_WRITE_ARMS"]])
entree = prod._apps.get("com.test.signee")
check("… manifeste reconstruit, avec la signature vérifiée par l'OS",
      entree is not None and entree.manifest.developer_public_key == APPROUVEE.public_key_b64
      and entree.manifest.code_signature == signature and entree.app is None)
ok_start, _ = silencieux(prod.start_app, "com.test.signee")
res, _ = silencieux(prod.run_mission, "com.test.signee", {})
check("… démarre et bouge le bras par le pont", ok_start is True and res is not None
      and res.success, getattr(res, "error_message", None))
prod_sans_humain, _ = runtime(profil=Profil.PRODUCTION, cles_approuvees=CLES)
ok, sortie = silencieux(prod_sans_humain.install_source, SOURCE_APP, code_signature=signature,
                        developer_public_key=APPROUVEE.public_key_b64, origine="signee")
check("Sans confirmation humaine configurée : permission DANGEROUS refusée",
      ok is False and "refus par défaut" in sortie)

section("6. Une app ne peut pas changer de profil")
res, _ = silencieux(prod.run_mission, "com.test.signee", {"changer_profil": True})
check("Demande de changement de profil par le pont : refusée",
      res is not None and res.success and "inconnue" in (res.error_message or ""),
      getattr(res, "error_message", None))
check("… le profil reste PRODUCTION", prod._profil is Profil.PRODUCTION)

section("7. SIMULATION et DÉVELOPPEMENT signalent leurs facilités")
_, journal_sim = runtime(profil=Profil.SIMULATION, confirmer_permissions=accepter_tout_SIMULATION)
check("SIMULATION : facilités annoncées au démarrage",
      "[PROFIL SIMULATION]" in journal_sim and "sans humain" in journal_sim)
dev, journal_dev = runtime(profil=Profil.DEVELOPPEMENT)
check("DÉVELOPPEMENT : facilités annoncées au démarrage", "[PROFIL DEVELOPPEMENT]" in journal_dev)
ok, sortie = silencieux(dev.install, AppObjet())
check("DÉVELOPPEMENT : app non signée installée, et signalée",
      ok is True and "[PROFIL DEVELOPPEMENT] app non signée installée" in sortie)

section("8. ACTUATORS_AI_DRIVEN vérifiée à l'exécution")
r = robot()
wd = WatchdogSystem(r)
ia = VLAAction(target_angles={"left_elbow_joint": 0.05}).to_commands(velocity=0.5)


def manifeste(perms, app_id="com.test.ia"):
    return AppManifest(app_id=app_id, name="IA", version="1.0.0", author="T",
                       author_email="t@t.com", description="d",
                       category=AppCategory.UTILITIES, permissions=perms)


sans = RobotContext.for_testing(r, wd, manifeste([Permission.ACTUATORS_WRITE_ARMS]))
try:
    silencieux(sans.move_arms, ia)
    check("Consigne d'IA sans ACTUATORS_AI_DRIVEN : refusée", False)
except SdkPermissionError:
    check("Consigne d'IA sans ACTUATORS_AI_DRIVEN : refusée (PermissionError)", True)
fixe = {"left_elbow_joint": ActuatorCommand(mode=ControlMode.POSITION, position=0.05)}
statuts, _ = silencieux(sans.move_arms, fixe)
check("Consigne ordinaire, même app : acceptée", statuts.get("left_elbow_joint") == HALStatus.OK)
avec = RobotContext.for_testing(r, wd, manifeste([Permission.ACTUATORS_WRITE_ARMS,
                                                  Permission.ACTUATORS_AI_DRIVEN]))
statuts, _ = silencieux(avec.move_arms, ia)
check("Consigne d'IA avec ACTUATORS_AI_DRIVEN : acceptée",
      statuts.get("left_elbow_joint") == HALStatus.OK)


class AppPiloteeParIA(RobotApp):
    """Fait tourner un « modèle » du SDK sans déclarer ACTUATORS_AI_DRIVEN."""

    def manifest(self):
        return manifeste([Permission.ACTUATORS_WRITE_ARMS], "com.test.ia_non_declaree")

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        action = VLAAction(target_angles={"left_elbow_joint": 0.05})
        ctx.move_arms(action.to_commands(velocity=0.5))
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0)


sim, _ = runtime(r, profil=Profil.SIMULATION, confirmer_permissions=accepter_tout_SIMULATION)
silencieux(sim.install, AppPiloteeParIA())
silencieux(sim.start_app, "com.test.ia_non_declaree")
res, _ = silencieux(sim.run_mission, "com.test.ia_non_declaree", {})
check("Depuis le processus de l'app : mission refusée faute de permission",
      res is not None and not res.success and "Permission" in (res.error_message or ""),
      getattr(res, "error_message", None))

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
