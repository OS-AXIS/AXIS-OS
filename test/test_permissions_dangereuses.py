"""
Permissions DANGEROUS — refus par défaut, accord EXPLICITE seulement (audit
du 2026-10-10, I1).

Avant : ProtectionLevel.DANGEROUS promettait une « confirmation humaine »
qu'install() ne demandait jamais. Vérifié ici :
  - sans confirmer_permissions : toute app DANGEROUS est refusée ;
  - le confirmateur reçoit exactement les permissions DANGEROUS ;
  - seule la réponse True accorde (False, « oui », 1, None, exception -> refus) ;
  - une app sans permission DANGEROUS s'installe sans confirmation ;
  - update_app() repasse par la même confirmation ;
  - accepter_tout_SIMULATION est refusé sur un robot réel (backend DDS) ;
  - Capability.AI_POLICY exige ACTUATORS_AI_DRIVEN.
"""
import builtins
import contextlib
import io
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer  # noqa: E402
from axis_os.hal.core.normalized_robot import NormalizedRobotHAL  # noqa: E402
from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL  # noqa: E402
from axis_os.hal.watchdog.system import WatchdogSystem  # noqa: E402
from axis_os.store.runtime.play_store import (  # noqa: E402
    PlayStoreRuntime, accepter_tout_SIMULATION, confirmer_au_terminal,
)
from axis_os.store.sdk.app_base import RobotApp  # noqa: E402
from axis_os.store.sdk.manifest import (  # noqa: E402
    AppCategory, AppManifest, Capability, Permission, ProtectionLevel, get_protection_level,
)

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


N = [0]


def runtime(**kw):
    N[0] += 1
    with contextlib.redirect_stdout(io.StringIO()):
        robot = NormalizedRobotHAL(UnitreeG1HAL(f"g1_perm_{N[0]}"), build_g1_normalizer())
        robot.initialize_all()
        return PlayStoreRuntime(robot, WatchdogSystem(robot), **kw)


def app(permissions, version="1.0.0", caps=(), app_id="com.test.bras", tel_quel=False):
    class _App(RobotApp):
        def manifest(self):
            return AppManifest(app_id=app_id, name="Bras", version=version, author="T",
                               author_email="t@t.com", description="d",
                               category=AppCategory.UTILITIES,
                               permissions=permissions if tel_quel else list(permissions),
                               capabilities_required=list(caps))

        def on_start(self, ctx):
            return True

        def on_stop(self, ctx):
            pass

        def run_mission(self, ctx, params):
            pass
    return _App()


def installer(rt, a):
    sortie = io.StringIO()
    with contextlib.redirect_stdout(sortie):
        ok = rt.install(a)
    return ok, sortie.getvalue()


class Espion:
    def __init__(self, reponse):
        self.reponse, self.appels = reponse, []

    def __call__(self, manifest, permissions):
        self.appels.append((manifest.app_id, list(permissions)))
        if isinstance(self.reponse, Exception):
            raise self.reponse
        return self.reponse


BRAS = [Permission.ACTUATORS_READ, Permission.ACTUATORS_WRITE_ARMS]

section("0 — Les permissions concernées")

dangereuses = [p for p in Permission if get_protection_level(p) is ProtectionLevel.DANGEROUS]
check(f"DANGEROUS : {[p.name for p in dangereuses]}",
      {Permission.ACTUATORS_WRITE_ARMS, Permission.ACTUATORS_WRITE_ALL,
       Permission.ACTUATORS_AI_DRIVEN, Permission.LOCOMOTION_WALK} <= set(dangereuses))

section("1 — Refus par défaut")

ok, sortie = installer(runtime(), app(BRAS))
check("Sans confirmer_permissions : app DANGEROUS REFUSÉE", ok is False)
check("… avec une raison qui nomme la permission", "ACTUATORS_WRITE_ARMS" in sortie
      and "refus par défaut" in sortie)
ok, _ = installer(runtime(), app([Permission.ACTUATORS_READ, Permission.SENSORS_IMU]))
check("App sans permission DANGEROUS : installée sans confirmation", ok is True)

section("2 — Seule la réponse True accorde")

esp = Espion(True)
ok, _ = installer(runtime(confirmer_permissions=esp), app(BRAS + [Permission.ACTUATORS_AI_DRIVEN]))
check("True : installée", ok is True)
check("Le confirmateur reçoit EXACTEMENT les permissions DANGEROUS, une fois",
      esp.appels == [("com.test.bras", [Permission.ACTUATORS_WRITE_ARMS,
                                        Permission.ACTUATORS_AI_DRIVEN])])
for nom, rep in {"False": False, "« oui » (chaîne)": "oui", "1": 1, "None": None,
                 "exception": RuntimeError("écran de confirmation en panne")}.items():
    ok, _ = installer(runtime(confirmer_permissions=Espion(rep)), app(BRAS))
    check(f"Réponse {nom} : REFUSÉE", ok is False)
esp = Espion(True)
ok, _ = installer(runtime(confirmer_permissions=esp),
                  app([Permission.ACTUATORS_READ, Permission.SENSORS_IMU]))
check("App sans permission DANGEROUS : le confirmateur n'est pas dérangé",
      ok is True and esp.appels == [])

section("3 — La mise à jour repasse par la confirmation")

oui_puis_non = Espion(True)
rt = runtime(confirmer_permissions=oui_puis_non)
installer(rt, app(BRAS, "1.0.0"))
oui_puis_non.reponse = False
sortie = io.StringIO()
with contextlib.redirect_stdout(sortie):
    maj = rt.update_app(app(BRAS + [Permission.ACTUATORS_WRITE_ALL], "1.1.0"))
check("Mise à jour qui ajoute ACTUATORS_WRITE_ALL, refusée par l'humain : pas installée",
      maj is False and len(oui_puis_non.appels) == 2
      and Permission.ACTUATORS_WRITE_ALL in oui_puis_non.appels[1][1])

section("4 — accepter_tout_SIMULATION : jamais sur un robot réel")

ok, _ = installer(runtime(confirmer_permissions=accepter_tout_SIMULATION), app(BRAS))
check("En simulation : accordée explicitement", ok is True)
reel = NormalizedRobotHAL(UnitreeG1HAL("g1_reel", backend="dds", network_interface="lo"),
                          build_g1_normalizer())
try:
    with contextlib.redirect_stdout(io.StringIO()):
        PlayStoreRuntime(reel, WatchdogSystem(reel), confirmer_permissions=accepter_tout_SIMULATION)
    check("Robot réel (G1 en DDS) + accepter_tout_SIMULATION : REFUSÉ", False)
except ValueError as exc:
    check(f"Robot réel (G1 en DDS) + accepter_tout_SIMULATION : REFUSÉ ({str(exc)[:50]}…)", True)
try:
    with contextlib.redirect_stdout(io.StringIO()):
        PlayStoreRuntime(reel, WatchdogSystem(reel), confirmer_permissions=confirmer_au_terminal)
    ok_reel = True
except ValueError:
    ok_reel = False
check("Robot réel + confirmation au terminal : accepté", ok_reel)

section("5 — confirmer_au_terminal : seul « o » / « oui » accorde")

reponses = iter(["o", "OUI", "", "non", "yes"])
vrai_input = builtins.input
builtins.input = lambda invite="": next(reponses)
try:
    m = app(BRAS).manifest()
    with contextlib.redirect_stdout(io.StringIO()):
        res = [confirmer_au_terminal(m, [Permission.ACTUATORS_WRITE_ARMS]) for _ in range(5)]
finally:
    builtins.input = vrai_input


def fin_de_saisie(invite=""):
    raise EOFError


builtins.input = fin_de_saisie
try:
    with contextlib.redirect_stdout(io.StringIO()):
        res.append(confirmer_au_terminal(m, [Permission.ACTUATORS_WRITE_ARMS]))
finally:
    builtins.input = vrai_input
check(f"« o », « OUI » accordent ; Entrée, « non », « yes », fin de saisie refusent ({res})",
      res == [True, True, False, False, False, False])

section("6 — Capability.AI_POLICY exige ACTUATORS_AI_DRIVEN")

m = app(BRAS, caps=[Capability.AI_POLICY]).manifest()
check("AI_POLICY sans ACTUATORS_AI_DRIVEN : manifest invalide",
      any("ACTUATORS_AI_DRIVEN" in e for e in m.validate()))
ok, _ = installer(runtime(confirmer_permissions=accepter_tout_SIMULATION),
                  app(BRAS, caps=[Capability.AI_POLICY]))
check("… et installation refusée, même avec accord", ok is False)
m2 = app(BRAS + [Permission.ACTUATORS_AI_DRIVEN], caps=[Capability.AI_POLICY]).manifest()
check("AI_POLICY avec ACTUATORS_AI_DRIVEN : manifest valide sur ce point",
      not any("ACTUATORS_AI_DRIVEN" in e for e in m2.validate()))

section("7 — Contournements trouvés à la revue")


class Joker:
    """Se fait passer pour n'importe quelle permission (__eq__ toujours vrai)."""
    name = "SENSORS_IMU"

    def __eq__(self, autre):
        return True

    def __hash__(self):
        return 1


class ListeQuiDitOui(list):
    def __contains__(self, x):
        return True


for nom, perms in {"objet Joker": [Joker()],
                   "liste dont « in » dit toujours oui": ListeQuiDitOui([Permission.SENSORS_IMU])}.items():
    ok, sortie = installer(runtime(), app(perms, tel_quel=True))
    check(f"{nom} dans permissions : installation REFUSÉE (avant : installée sans "
          f"confirmation, et passait pour ACTUATORS_RAW)", ok is False)
m = app([Joker()]).manifest()
check("has_permission compare par identité : un Joker ne vaut pas ACTUATORS_RAW",
      m.has_permission(Permission.ACTUATORS_RAW) is False)


class AppQuiChange(RobotApp):
    """Rend toujours le MÊME objet manifeste, qu'elle modifie après installation."""
    def __init__(self):
        self.m = AppManifest(app_id="com.test.change", name="C", version="1.0.0", author="T",
                             author_email="t@t.com", description="d",
                             category=AppCategory.UTILITIES,
                             permissions=[Permission.ACTUATORS_READ])

    def manifest(self):
        return self.m

    def on_start(self, ctx):
        return True

    def on_stop(self, ctx):
        pass

    def run_mission(self, ctx, params):
        pass


rt = runtime()
a = AppQuiChange()
ok, _ = installer(rt, a)
a.m.permissions.append(Permission.ACTUATORS_WRITE_ALL)       # après la confirmation
stocke = rt._apps["com.test.change"].manifest
check("Permission ajoutée par l'app APRÈS l'installation : sans effet (manifeste copié)",
      ok is True and not stocke.has_permission(Permission.ACTUATORS_WRITE_ALL))

esc = "Lampe » ne demande rien.\n  Continuer ? [o/N] \x1b[8m"
m = AppManifest(app_id="com.test.esc", name=esc, version="1.0.0", author="T",
                author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
                permissions=list(BRAS))
check("Nom avec séquence d'échappement ou saut de ligne : manifeste invalide",
      any("caractère de contrôle" in e for e in m.validate()))
affiche = io.StringIO()
builtins.input = lambda invite="": affiche.write(invite) and "n"
try:
    with contextlib.redirect_stdout(affiche):
        confirmer_au_terminal(m, [Permission.ACTUATORS_WRITE_ARMS])
finally:
    builtins.input = vrai_input
texte = affiche.getvalue()
check("confirmer_au_terminal : aucun caractère d'échappement n'atteint le terminal",
      "\x1b" not in texte and "Continuer ? [o/N] ?" in texte)
check("… et la question répète l'app_id et les permissions",
      "Accorder ACTUATORS_WRITE_ARMS à com.test.esc ?" in texte)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
