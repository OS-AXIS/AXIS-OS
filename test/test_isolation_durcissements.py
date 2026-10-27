"""
Isolation des apps — correctifs de la revue du bac à sable (2026-10-20).

Chaque section vérifie qu'un comportement ordinaire d'une app ne peut plus
coûter à l'OS plus que ce qui est prévu, ou qu'une règle tient :
  1. une sortie standard très longue, sans saut de ligne, ne fait pas
     grossir la mémoire du processus de l'OS ;
  2. un message d'erreur de l'app arrive au terminal de l'OS sans
     caractère de contrôle ;
  3. une écoute demandée par une app ne dure jamais plus que ce qui reste
     à son hook, et une durée démesurée est refusée ;
  4. get_joint_groups exige ACTUATORS_READ, comme get_full_state ;
  5. le journal de mission tenu par l'OS est plafonné ;
  6. un app_id ne peut pas contenir « : » (espaces internes de l'OS) ;
  7. le fichier des clés approuvées ne peut pas être un lien symbolique ;
  8. en PRODUCTION, update_app() est refusé d'emblée ;
  9. en PRODUCTION, une app chiffrée s'installe par
     decrypt_verified_app_source() puis install_source(), sans que son code
     s'exécute dans le processus de l'OS ;
 10. SetpointFollower transmet la marque « ia » : sans ACTUATORS_AI_DRIVEN,
     ses consignes sont refusées ;
 11. seconde revue : journal borné en volume et neutralisé, relais borné
     ligne par ligne, tube nommé et fichier énorme refusés comme fichier de
     clés, date de facturation future gardée, registre des intentions
     plafonné, erreurs de validate() neutralisées à l'affichage.
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
from axis_os.hal.core.types import ORIGINE_IA  # noqa: E402
from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL  # noqa: E402
from axis_os.hal.watchdog.system import WatchdogSystem  # noqa: E402
from axis_os.services.base import ServiceResult  # noqa: E402
from axis_os.services.policy.setpoint_follower import SetpointFollower  # noqa: E402
from axis_os.store.runtime.hal_bridge import HALBridgeServer  # noqa: E402
from axis_os.store.runtime.packaging import decrypt_verified_app_source  # noqa: E402
from axis_os.store.runtime.play_store import PlayStoreRuntime  # noqa: E402
from axis_os.store.runtime.manifeste_json import manifeste_texte  # noqa: E402
from axis_os.store.runtime.profil import (  # noqa: E402
    ClesApprouveesInvalides, Profil, creer_registre_apps, lire_cles_approuvees,
)
from axis_os.store.runtime.sandbox import SandboxRunner  # noqa: E402
from axis_os.store.sdk.app_base import (  # noqa: E402
    MissionResult, PermissionError as SdkPermissionError, RobotApp, RobotContext,
)
from axis_os.store.sdk.encryption import encrypt_app_code, generate_platform_keypair  # noqa: E402
from axis_os.store.sdk.manifest import (  # noqa: E402
    AppCategory, AppManifest, Permission, ResourceQuota,
)
from axis_os.store.sdk.signing import generate_developer_keypair, sign_app_package  # noqa: E402

FAIL = 0


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


def silencieux(f, *a, **kw):
    sortie = io.StringIO()
    with contextlib.redirect_stdout(sortie):
        r = f(*a, **kw)
    return r, sortie.getvalue()


def robot():
    with contextlib.redirect_stdout(io.StringIO()):
        r = NormalizedRobotHAL(UnitreeG1HAL("g1_durcissement"), build_g1_normalizer())
        r.initialize_all()
    return r


def manifeste(perms, app_id="com.test.durcissement"):
    return AppManifest(app_id=app_id, name="Test", version="1.0.0", author="T",
                       author_email="t@t.com", description="d",
                       category=AppCategory.UTILITIES, permissions=perms)


# Fonctions exécutées dans le processus neuf par SandboxRunner.run().
def sortie_sans_fin_de_ligne():
    for _ in range(40):
        os.write(1, b"x" * (512 * 1024))      # 20 Mio, aucun saut de ligne
    return "fini"


def erreur_avec_echappements():
    raise ValueError("\x1b[2J\x1b[31mfaux message de l'OS\x07")


def beaucoup_de_lignes_courtes():
    os.write(1, b"l\n" * 20000)
    return "fini"


section("1. Sortie très longue : la mémoire de l'OS ne grossit pas")
import psutil  # noqa: E402
avant = psutil.Process().memory_info().rss
res, journal = silencieux(SandboxRunner("com.test.bavarde", ResourceQuota(max_runtime_s=20.0)).run,
                          sortie_sans_fin_de_ligne)
apres = psutil.Process().memory_info().rss
check("La fonction se termine normalement", res.success and res.result == "fini", str(res))
check(f"Mémoire de l'OS : +{(apres - avant) / 2**20:.1f} Mio pour 20 Mio écrits (moins de 8)",
      apres - avant < 8 * 2**20)
check("Ce qui est relayé au terminal reste court", len(journal) < 10_000, f"{len(journal)} car.")

section("2. Erreur de l'app : aucun caractère de contrôle vers le terminal")
res, _ = silencieux(SandboxRunner("com.test.erreur", ResourceQuota(max_runtime_s=10.0)).run,
                    erreur_avec_echappements)
check("Échec rapporté", not res.success and "faux message" in (res.error or ""), str(res.error))
check("… sans séquence d'échappement", res.error is not None and "\x1b" not in res.error
      and "\x07" not in res.error)

section("3. Écoute bornée par le temps qui reste au hook")


class ParoleNotee:
    def __init__(self):
        self.durees = []

    def listen(self, duree, app_id=None):
        self.durees.append(duree)
        return ServiceResult(ok=False, value=None, reason=None)


r = robot()
wd = WatchdogSystem(r)
ctx = RobotContext.for_testing(r, wd, manifeste([Permission.SENSORS_MICROPHONE]))
parole = ParoleNotee()
ctx._attach_services(speech=parole)
serveur = HALBridgeServer(ctx)
rep, _ = silencieux(serveur.traiter, {"action": "listen", "payload": {"duration_s": 100.0},
                                      "request_id": 1}, reste_s=2.0)
check("listen(100 s) avec 2 s restantes : écoute ramenée à moins de la moitié",
      rep["success"] and parole.durees and parole.durees[-1] <= 0.75, str(parole.durees))
rep, _ = silencieux(serveur.traiter, {"action": "listen", "payload": {"duration_s": 3600.0},
                                      "request_id": 2}, reste_s=2.0)
check("listen(3600 s) : refusé par le schéma", rep["success"] is False
      and rep["error"].startswith("MALFORMED_COMMAND"), str(rep))

section("4. get_joint_groups exige ACTUATORS_READ")
sans_lecture = HALBridgeServer(RobotContext.for_testing(r, wd, manifeste([])))
rep, _ = silencieux(sans_lecture.traiter, {"action": "get_joint_groups", "payload": {},
                                           "request_id": 3})
check("Sans ACTUATORS_READ : refusé", rep["success"] is False
      and rep["error"].startswith("PERMISSION_DENIED"), str(rep))
avec_lecture = HALBridgeServer(RobotContext.for_testing(r, wd, manifeste([Permission.ACTUATORS_READ])))
rep, _ = silencieux(avec_lecture.traiter, {"action": "get_joint_groups", "payload": {},
                                           "request_id": 4})
check("Avec ACTUATORS_READ : groupes rendus", rep["success"] and rep["data"])

section("5. Journal de mission plafonné")
bavard = RobotContext.for_testing(r, wd, manifeste([]))
silencieux(lambda: [bavard.log(f"ligne {i}") for i in range(RobotContext.MAX_LIGNES_JOURNAL + 50)])
journal = bavard.get_mission_log()
check(f"{RobotContext.MAX_LIGNES_JOURNAL + 50} lignes écrites : "
      f"{RobotContext.MAX_LIGNES_JOURNAL} gardées, plus une ligne de troncature",
      len(journal) == RobotContext.MAX_LIGNES_JOURNAL + 1 and "tronqué" in journal[-1])

section("6. app_id : « : » réservé à l'OS")
check("app_id « billing::com.x.y » : manifeste invalide",
      any("« : »" in e for e in manifeste([], "billing::com.x.y").validate()))
check("app_id ordinaire : valide", manifeste([], "com.x.y").validate() == [])

section("7. Fichier des clés approuvées : pas de lien symbolique")
dossier = Path(tempfile.mkdtemp(prefix="axis_durcissement_"))
dev = generate_developer_keypair()
vrai = dossier / "cles.txt"
vrai.write_text(f"{dev.public_key_b64}  Test\n", encoding="utf-8")
os.chmod(vrai, 0o600)
lien = dossier / "lien.txt"
lien.symlink_to(vrai)
try:
    lire_cles_approuvees(lien)
    check("Lien symbolique : refusé", False)
except ClesApprouveesInvalides:
    check("Lien symbolique : refusé", True)
check("Fichier ordinaire : lu", dev.public_key_b64 in lire_cles_approuvees(vrai))

section("8. PRODUCTION : update_app() refusé d'emblée")


class AppObjet(RobotApp):
    def manifest(self): return manifeste([], "com.test.objet")
    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0)


registre = dossier / "registre_apps.json"
creer_registre_apps(registre)
prod, _ = silencieux(PlayStoreRuntime, r, wd, profil=Profil.PRODUCTION, cles_approuvees=vrai,
                     registre_apps=registre)
ok, sortie = silencieux(prod.update_app, AppObjet())
check("update_app(objet) en production : refusé", ok is False and "PRODUCTION" in sortie)

section("9. PRODUCTION : app chiffrée installée sans exécution dans l'OS")
SOURCE = '''
from axis_os.store.sdk.app_base import MissionResult, RobotApp
from axis_os.store.sdk.manifest import AppCategory, AppManifest

class AppChiffree(RobotApp):
    def manifest(self):
        return AppManifest(app_id="com.test.chiffree", name="Chiffrée", version="1.0.0",
                           author="T", author_email="t@t.com", description="d",
                           category=AppCategory.UTILITIES)
    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass
    def run_mission(self, ctx, params):
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0)
'''
# Le manifeste déclaré voyage en clair à côté du paquet chiffré, signé avec
# le code (le même que celui que rend AppChiffree.manifest()).
MANIFESTE = manifeste_texte(AppManifest(
    app_id="com.test.chiffree", name="Chiffrée", version="1.0.0", author="T",
    author_email="t@t.com", description="d", category=AppCategory.UTILITIES))
plateforme = generate_platform_keypair()
paquet = encrypt_app_code(SOURCE, plateforme.public_key_b64).to_bytes()
signature = sign_app_package(MANIFESTE, SOURCE, dev.private_key_b64)
modules_avant = set(sys.modules)
texte = decrypt_verified_app_source(paquet, plateforme.private_key_b64,
                                    code_signature=signature,
                                    developer_public_key=dev.public_key_b64,
                                    manifeste=MANIFESTE)
check("Déchiffré et vérifié : texte rendu, rien d'exécuté",
      texte == SOURCE and not [m for m in set(sys.modules) - modules_avant
                               if "chiffree" in m or "encrypted_app" in m])
ok, sortie = silencieux(prod.install_source, texte, manifeste=MANIFESTE, code_signature=signature,
                        developer_public_key=dev.public_key_b64, origine="chiffree")
check("install_source : installée", ok is True, sortie[-300:])
check("… toujours rien de chargé dans le processus de l'OS",
      not [m for m in set(sys.modules) - modules_avant if "chiffree" in m])

section("10. SetpointFollower transmet la marque « ia »")
sans_ia = RobotContext.for_testing(r, wd, manifeste([Permission.ACTUATORS_WRITE_ARMS,
                                                     Permission.ACTUATORS_READ]))
suiveur = SetpointFollower(sans_ia, {"left_elbow_joint": 0.05})
suiveur.set_target({"left_elbow_joint": 0.1}, origine=ORIGINE_IA)
try:
    silencieux(suiveur.tick)
    check("Consigne d'IA via le suiveur, sans ACTUATORS_AI_DRIVEN : refusée", False)
except SdkPermissionError:
    check("Consigne d'IA via le suiveur, sans ACTUATORS_AI_DRIVEN : refusée", True)
avec_ia = RobotContext.for_testing(r, wd, manifeste([Permission.ACTUATORS_WRITE_ARMS,
                                                     Permission.ACTUATORS_READ,
                                                     Permission.ACTUATORS_AI_DRIVEN]))
suiveur = SetpointFollower(avec_ia, {"left_elbow_joint": 0.05})
suiveur.set_target({"left_elbow_joint": 0.1}, origine=ORIGINE_IA)
etat, _ = silencieux(suiveur.tick)
check("… avec ACTUATORS_AI_DRIVEN : acceptée", suiveur.stats.commands_sent == 1,
      str(suiveur.stats))

section("11. Seconde revue : bornes et neutralisation")
journal_os = RobotContext.for_testing(r, wd, manifeste([]))
silencieux(lambda: [journal_os.log("\x1b[31m" + "é" * 5000) for _ in range(3000)])
lignes = journal_os.get_mission_log()
taille = sum(len(x) for x in lignes)
check(f"Journal de l'OS borné en volume ({taille} caractères pour 15 M écrits)",
      taille <= RobotContext.MAX_CARACTERES_JOURNAL + 200 and "tronqué" in lignes[-1])
check("… chaque ligne coupée et sans caractère de contrôle",
      all("\x1b" not in x and len(x) < RobotContext.MAX_CARACTERES_LIGNE + 100 for x in lignes))
res, journal = silencieux(SandboxRunner("com.test.lignes", ResourceQuota(max_runtime_s=20.0)).run,
                          beaucoup_de_lignes_courtes)
relayees = journal.count("[app com.test.lignes]")
check(f"20 000 lignes écrites : {relayees} relayées (au plus 5 001)", res.success and relayees <= 5001)
tube = dossier / "tube"
os.mkfifo(tube)
try:
    lire_cles_approuvees(tube)
    check("Tube nommé comme fichier de clés : refusé sans bloquer", False)
except ClesApprouveesInvalides:
    check("Tube nommé comme fichier de clés : refusé sans bloquer", True)
gros = dossier / "gros.txt"
gros.write_text("# commentaire\n" * 100000, encoding="utf-8")
os.chmod(gros, 0o600)
try:
    lire_cles_approuvees(gros)
    check("Fichier de clés de plus de 1 Mio : refusé", False)
except ClesApprouveesInvalides:
    check("Fichier de clés de plus de 1 Mio : refusé", True)


class MemoireFactice:
    def __init__(self, valeur):
        self.valeur = valeur

    def app_load(self, espace, cle):
        return ServiceResult(ok=True, value=self.valeur)


import time  # noqa: E402
futur = time.time() + 30 * 86400
prod._memory_service = MemoireFactice(futur)
lu, _ = silencieux(prod._read_billing_flag, "com.x.y", "last_subscription_bill_time_s", None)
check("Date de facturation dans le futur : gardée (pas de refacturation)", lu == futur)
prod._memory_service = MemoireFactice("pas une date")
lu, _ = silencieux(prod._read_billing_flag, "com.x.y", "last_subscription_bill_time_s", None)
check("Date de facturation illisible : ignorée", lu is None)
prod._memory_service = None

from axis_os.services.speech.service import IntentDeclaration, IntentRegistry  # noqa: E402
registre = IntentRegistry()
for i in range(IntentRegistry.MAX_INTENTIONS):
    registre.register(IntentDeclaration(name=f"i{i}", keywords=[["mot"]]))
try:
    registre.register(IntentDeclaration(name="une_de_trop", keywords=[["mot"]]))
    check("Registre des intentions plafonné", False)
except ValueError:
    check(f"Registre des intentions plafonné à {IntentRegistry.MAX_INTENTIONS}", True)

SOURCE_VERSION = SOURCE.replace('category=AppCategory.UTILITIES)',
                                'category=AppCategory.UTILITIES, sdk_version_min="\\x1b[31m9")')
SOURCE_VERSION = SOURCE_VERSION.replace("com.test.chiffree", "com.test.version")
MANIFESTE_VERSION = manifeste_texte(AppManifest(
    app_id="com.test.version", name="Chiffrée", version="1.0.0", author="T",
    author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
    sdk_version_min="\x1b[31m9"))
ok, sortie = silencieux(prod.install_source, SOURCE_VERSION, manifeste=MANIFESTE_VERSION,
                        code_signature=sign_app_package(MANIFESTE_VERSION, SOURCE_VERSION,
                                                        dev.private_key_b64),
                        developer_public_key=dev.public_key_b64, origine="version")
check("Erreur de validate() reprenant un champ de l'app : affichée neutralisée",
      ok is False and "sdk_version_min" in sortie and "\x1b" not in sortie, sortie[-200:])

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
