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
     ses consignes sont refusées.
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
from axis_os.store.runtime.profil import (  # noqa: E402
    ClesApprouveesInvalides, Profil, lire_cles_approuvees,
)
from axis_os.store.runtime.sandbox import SandboxRunner  # noqa: E402
from axis_os.store.sdk.app_base import (  # noqa: E402
    MissionResult, PermissionError as SdkPermissionError, RobotApp, RobotContext,
)
from axis_os.store.sdk.encryption import encrypt_app_code, generate_platform_keypair  # noqa: E402
from axis_os.store.sdk.manifest import (  # noqa: E402
    AppCategory, AppManifest, Permission, ResourceQuota,
)
from axis_os.store.sdk.signing import generate_developer_keypair, sign_app_code  # noqa: E402

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

    def listen(self, duree):
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
check("listen(100 s) avec 2 s restantes : écoute ramenée sous 2 s",
      rep["success"] and parole.durees and parole.durees[-1] <= 2.0, str(parole.durees))
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


prod, _ = silencieux(PlayStoreRuntime, r, wd, profil=Profil.PRODUCTION, cles_approuvees=vrai)
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
plateforme = generate_platform_keypair()
paquet = encrypt_app_code(SOURCE, plateforme.public_key_b64).to_bytes()
signature = sign_app_code(SOURCE, dev.private_key_b64)
modules_avant = set(sys.modules)
texte = decrypt_verified_app_source(paquet, plateforme.private_key_b64,
                                    code_signature=signature,
                                    developer_public_key=dev.public_key_b64)
check("Déchiffré et vérifié : texte rendu, rien d'exécuté",
      texte == SOURCE and not [m for m in set(sys.modules) - modules_avant
                               if "chiffree" in m or "encrypted_app" in m])
ok, sortie = silencieux(prod.install_source, texte, code_signature=signature,
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

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
