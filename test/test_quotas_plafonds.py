"""
Ressources d'une app bornées par l'OS (SECURITY.md, ancienne limite n° 6).

Vérifié ici :
  1. un quota mal formé (NaN, infini, nul, négatif, booléen, autre type)
     rend le manifeste invalide : il désactivait en silence le contrôle ;
  2. un quota au-delà du plafond de l'OS est refusé à l'installation, avec
     un message qui dit lequel, sa valeur, le plafond et le profil ; il
     n'est jamais ramené ; le refus a lieu avant de solliciter l'humain ;
  3. les plafonds de PRODUCTION sont plus stricts : la même app, refusée en
     production, s'installe en développement ;
  4. l'humain qui confirme les permissions voit les quotas et la clé qui a
     signé l'app (empreinte, nom inscrit dans le fichier des clés
     approuvées), pas seulement l'auteur déclaré ; en production, un
     confirmateur qui ne recevrait pas ces détails est refusé ;
  5. la mémoire persistante (remember) a un quota par app, en clés et en
     octets, avec un refus clair, en direct comme par le pont.
"""
import builtins
import contextlib
import io
import os
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer  # noqa: E402
from axis_os.hal.core.normalized_robot import NormalizedRobotHAL  # noqa: E402
from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL  # noqa: E402
from axis_os.hal.watchdog.system import WatchdogSystem  # noqa: E402
from axis_os.services.memory.service import MemoryService  # noqa: E402
from axis_os.store.runtime.hal_bridge import HALBridgeServer  # noqa: E402
from axis_os.store.runtime.play_store import (  # noqa: E402
    PlayStoreRuntime, accepter_tout_SIMULATION, confirmer_au_terminal,
)
from axis_os.store.runtime.profil import PLAFONDS_QUOTAS, Profil  # noqa: E402
from axis_os.store.sdk.app_base import MissionResult, RobotApp, RobotContext  # noqa: E402
from axis_os.store.sdk.manifest import (  # noqa: E402
    AppCategory, AppManifest, Permission, ResourceQuota,
)
from axis_os.store.sdk.signing import (  # noqa: E402
    generate_developer_keypair, public_key_fingerprint, sign_app_code,
)

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


with contextlib.redirect_stdout(io.StringIO()):
    ROBOT = NormalizedRobotHAL(UnitreeG1HAL("g1_quotas"), build_g1_normalizer())
    ROBOT.initialize_all()
WD = WatchdogSystem(ROBOT)


def manifeste(quota=None, perms=(), app_id="com.test.quota"):
    return AppManifest(app_id=app_id, name="Quota", version="1.0.0", author="T",
                       author_email="t@t.com", description="d",
                       category=AppCategory.UTILITIES, permissions=list(perms),
                       quota=ResourceQuota() if quota is None else quota)


class Espion:
    """Tient le rôle de l'opérateur : note chaque demande et accepte."""
    recoit_details = True

    def __init__(self):
        self.appels = []

    def __call__(self, manifest, permissions, details=None):
        self.appels.append((manifest.app_id, [p.name for p in permissions], details))
        return True


class AppQuota(RobotApp):
    """App dont le quota est réglé par l'attribut de classe QUOTA (le test le
    change d'une installation à l'autre ; l'installation lit manifest())."""
    QUOTA = ResourceQuota()

    def manifest(self):
        return AppManifest(app_id="com.test.quota", name="Quota", version="1.0.0", author="T",
                           author_email="t@t.com", description="d",
                           category=AppCategory.UTILITIES,
                           permissions=[Permission.ACTUATORS_WRITE_ARMS], quota=self.QUOTA)

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0)


section("1. Un quota mal formé rend le manifeste invalide")
for nom, quota in [("max_runtime_s infini", ResourceQuota(max_runtime_s=float("inf"))),
                   ("max_memory_mb NaN", ResourceQuota(max_memory_mb=float("nan"))),
                   ("max_cpu_percent nul", ResourceQuota(max_cpu_percent=0)),
                   ("max_runtime_s négatif", ResourceQuota(max_runtime_s=-1.0)),
                   ("max_memory_mb booléen", ResourceQuota(max_memory_mb=True)),
                   ("quota d'un autre type", "pas un quota")]:
    erreurs = manifeste(quota).validate()
    check(f"{nom} : refusé", any("quota" in e for e in erreurs), str(erreurs))
check("Quota par défaut : valide", manifeste().validate() == [])
check("Quota court (0,5 s) : valide", manifeste(ResourceQuota(max_runtime_s=0.5)).validate() == [])

section("2. Au-delà du plafond de l'OS : refusé, jamais ramené, avant l'humain")
espion = Espion()
sim, _ = silencieux(PlayStoreRuntime, ROBOT, WD, profil=Profil.SIMULATION,
                    confirmer_permissions=espion)
plafond = PLAFONDS_QUOTAS[Profil.SIMULATION]
for champ, valeur in [("max_memory_mb", plafond.max_memory_mb + 1),
                      ("max_runtime_s", plafond.max_runtime_s + 1),
                      ("max_cpu_percent", plafond.max_cpu_percent + 1)]:
    AppQuota.QUOTA = ResourceQuota(**{champ: valeur})
    ok, sortie = silencieux(sim.install, AppQuota())
    check(f"{champ} = {valeur:g} (plafond {getattr(plafond, champ):g}) : refusé",
          ok is False and "com.test.quota" not in sim._apps)
    check("… le message dit lequel, sa valeur, le plafond et le profil",
          champ in sortie and f"{valeur:g}" in sortie
          and f"{getattr(plafond, champ):g}" in sortie and "SIMULATION" in sortie, sortie[-300:])
check("… et l'humain n'a pas été sollicité pour une app refusée", espion.appels == [])
AppQuota.QUOTA = ResourceQuota(max_cpu_percent=plafond.max_cpu_percent,
                               max_memory_mb=plafond.max_memory_mb,
                               max_runtime_s=plafond.max_runtime_s)
ok, _ = silencieux(sim.install, AppQuota())
check("Quotas égaux aux plafonds : installée", ok is True)
check("… quota gardé tel que déclaré (jamais ramené)",
      ok and sim._apps["com.test.quota"].manifest.quota == AppQuota.QUOTA)

section("3. PRODUCTION : plafonds plus stricts")
CLE = generate_developer_keypair()
DOSSIER = Path(tempfile.mkdtemp(prefix="axis_quotas_"))
CLES = DOSSIER / "cles_approuvees.txt"
CLES.write_text(f"{CLE.public_key_b64}  Atelier Dupont\n", encoding="utf-8")
os.chmod(CLES, 0o600)
for champ in ("max_cpu_percent", "max_memory_mb", "max_runtime_s"):
    check(f"{champ} : plafond PRODUCTION < plafond SIMULATION",
          getattr(PLAFONDS_QUOTAS[Profil.PRODUCTION], champ)
          < getattr(PLAFONDS_QUOTAS[Profil.SIMULATION], champ))

SOURCE = '''
from axis_os.store.sdk.app_base import MissionResult, RobotApp
from axis_os.store.sdk.manifest import AppCategory, AppManifest, Permission, ResourceQuota

class AppSource(RobotApp):
    def manifest(self):
        return AppManifest(app_id="com.test.quota_source", name="Source", version="1.0.0",
                           author="Quelqu'un d'autre", author_email="t@t.com",
                           description="d", category=AppCategory.UTILITIES,
                           permissions=[Permission.ACTUATORS_WRITE_ARMS],
                           quota=ResourceQuota(max_memory_mb=MEMOIRE))
    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass
    def run_mission(self, ctx, params):
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0)
'''


def installer_source(rt, memoire):
    texte = SOURCE.replace("MEMOIRE", str(memoire))
    return silencieux(rt.install_source, texte, code_signature=sign_app_code(texte, CLE.private_key_b64),
                      developer_public_key=CLE.public_key_b64, origine="quota")


espion_prod = Espion()
prod, _ = silencieux(PlayStoreRuntime, ROBOT, WD, profil=Profil.PRODUCTION, cles_approuvees=CLES,
                     confirmer_permissions=espion_prod)
ok, sortie = installer_source(prod, 2048)
check("PRODUCTION, max_memory_mb = 2048 : refusé", ok is False and "PRODUCTION" in sortie
      and "max_memory_mb" in sortie, sortie[-300:])
check("… sans solliciter l'humain", espion_prod.appels == [])
dev, _ = silencieux(PlayStoreRuntime, ROBOT, WD, profil=Profil.DEVELOPPEMENT, cles_approuvees=CLES,
                    confirmer_permissions=Espion())
ok, _ = installer_source(dev, 2048)
check("La même app en DÉVELOPPEMENT : installée", ok is True)
ok, sortie = installer_source(prod, PLAFONDS_QUOTAS[Profil.PRODUCTION].max_memory_mb)
check("PRODUCTION, au plafond : installée", ok is True, sortie[-300:])

section("4. L'humain voit les quotas et la clé signataire")
check("Le confirmateur reçoit la clé vérifiée, pas l'auteur déclaré",
      len(espion_prod.appels) == 1 and espion_prod.appels[0][2] is not None
      and espion_prod.appels[0][2].signataire == public_key_fingerprint(CLE.public_key_b64)
      and espion_prod.appels[0][2].nom_signataire == "Atelier Dupont"
      and espion_prod.appels[0][2].fichier_cles == str(CLES), str(espion_prod.appels))
check("Le journal d'installation montre la clé et les quotas",
      public_key_fingerprint(CLE.public_key_b64) in sortie and "Atelier Dupont" in sortie
      and "1024 Mo" in sortie, sortie[-400:])
details = espion_prod.appels[0][2]
m = manifeste(ResourceQuota(max_cpu_percent=60.0, max_memory_mb=384, max_runtime_s=600.0),
              [Permission.ACTUATORS_WRITE_ARMS], "com.test.affichage")
reponses = iter(["o"])
vrai_input = builtins.input
builtins.input = lambda invite="": (print(invite), next(reponses))[1]
try:
    accord, ecran = silencieux(confirmer_au_terminal, m, [Permission.ACTUATORS_WRITE_ARMS], details)
finally:
    builtins.input = vrai_input
check("Au terminal : quotas affichés (CPU, mémoire, durée, mémoire persistante)",
      "60 %" in ecran and "384 Mo" in ecran and "600 s" in ecran
      and str(MemoryService.MAX_CLES_PAR_APP) in ecran, ecran)
check("Au terminal : empreinte de la clé, nom et fichier des clés approuvées",
      details.signataire in ecran and "Atelier Dupont" in ecran and str(CLES) in ecran, ecran)
check("… et la question reste la même", accord is True
      and "Accorder ACTUATORS_WRITE_ARMS à com.test.affichage ?" in ecran)
try:
    silencieux(PlayStoreRuntime, ROBOT, WD, profil=Profil.PRODUCTION, cles_approuvees=CLES,
               confirmer_permissions=lambda manifest, permissions: True)
    check("PRODUCTION : confirmateur sans les détails : refusé", False)
except ValueError:
    check("PRODUCTION : confirmateur sans les détails : refusé", True)

section("5. Mémoire persistante : quota par app")
svc = MemoryService(storage_dir=Path(tempfile.mkdtemp(prefix="axis_quota_memoire_")))
svc.start()
svc.MAX_CLES_PAR_APP = 3
for i in range(3):
    svc.app_store("com.a.app", f"k{i}", i)
r = svc.app_store("com.a.app", "k3", 3)
check("Une clé de plus que le quota : refusée, avec la raison",
      r.ok is False and "quota" in (r.reason or "") and not svc.app_load("com.a.app", "k3").ok,
      str(r))
check("Réécrire une clé existante : permis", svc.app_store("com.a.app", "k0", 10).ok)
check("Le quota est par app : une autre app écrit encore", svc.app_store("com.b.app", "k0", 1).ok)
autre = MemoryService(storage_dir=svc._storage_dir)
autre.start()
autre.MAX_CLES_PAR_APP = 3
check("Le quota tient après redémarrage (rien n'est compté en mémoire)",
      autre.app_store("com.a.app", "k9", 1).ok is False)
svc.MAX_CLES_PAR_APP, svc.MAX_OCTETS_PAR_APP = 1000, 200
r = svc.app_store("com.a.app", "gros", "x" * 500)
check("Une valeur qui dépasse le quota d'octets : refusée",
      r.ok is False and "octets" in (r.reason or "") and not svc.app_load("com.a.app", "gros").ok,
      str(r))
check("Une petite valeur : acceptée", svc.app_store("com.a.app", "petit", "x" * 10).ok)
refus = [svc.app_store("com.a.app", f"s{i}", "y" * 20).ok for i in range(10)]
check("Une suite de petites valeurs : refusée une fois le total atteint",
      True in refus and refus[-1] is False, str(refus))
svc.MAX_OCTETS_PAR_APP = 10
check("Au-dessus du quota, une app peut encore réduire ses données",
      svc.app_store("com.a.app", "petit", "").ok)
check("… mais pas les faire grossir", svc.app_store("com.a.app", "petit", "xxxx").ok is False)
r = svc.app_store("com.a.app", "ensemble", {1, 2})
check("Valeur non JSON : refusée proprement, sans fichier temporaire laissé",
      r.ok is False and not list(svc._storage_dir.glob("*.tmp")), str(r))

svc2 = MemoryService(storage_dir=Path(tempfile.mkdtemp(prefix="axis_quota_pont_")))
svc2.start()
svc2.MAX_CLES_PAR_APP = 1
ctx = RobotContext.for_testing(ROBOT, WD, manifeste(perms=[Permission.STORAGE_PERSISTENT]))
ctx._attach_services(memory=svc2)
serveur = HALBridgeServer(ctx)
rep, _ = silencieux(serveur.traiter, {"action": "remember", "payload": {"key": "a", "value": 1},
                                      "request_id": 1})
check("Par le pont : première clé enregistrée", rep["success"] and rep["data"] is True, str(rep))
rep, _ = silencieux(serveur.traiter, {"action": "remember", "payload": {"key": "b", "value": 1},
                                      "request_id": 2})
check("Par le pont : clé de trop refusée (remember rend False)",
      rep["success"] and rep["data"] is False, str(rep))
check("… et la raison est au journal de mission",
      any("quota" in ligne for ligne in ctx.get_mission_log()), str(ctx.get_mission_log()))

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
