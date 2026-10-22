"""
Identité et cycle de vie des apps en production (SECURITY.md, ancienne
limite n° 7).

Vérifié ici, en profil PRODUCTION, avec deux clés approuvées A et B :
  1. le registre des apps est exigé, et refusé s'il est absent, abîmé ou
     trop ouvert : le runtime ne démarre pas plutôt que d'oublier ses
     liaisons ;
  2. un app_id est lié à la clé qui l'a installé la première fois : après
     un redémarrage, une autre clé ne peut ni l'installer ni le mettre à
     jour ;
  3. la confirmation montre la clé qui a signé (empreinte, nom inscrit dans
     le fichier des clés approuvées) et l'état de la liaison, pas
     seulement l'auteur déclaré ;
  4. mise à jour en production (update_source) : même clé, version
     strictement plus récente, facturation conservée ;
  5. la version la plus récente installée survit au redémarrage : une
     version plus ancienne est refusée, la même se réinstalle ;
  6. aucune ligne de l'app ne tourne avant que l'humain ait accepté : le
     manifeste est déclaré à côté du code ;
  7. la signature lie le manifeste et le code : modifier l'un, recombiner
     deux paquets ou présenter la signature du code seul est refusé ;
  8. les versions ambiguës (« 1_0.0.0 », chiffres non ASCII…) sont
     refusées ;
  9. deux installations simultanées du même app_id ne passent pas toutes
     les deux.
"""
import builtins
import contextlib
import hashlib
import io
import json
import os
import sys
import tempfile
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer  # noqa: E402
from axis_os.hal.core.normalized_robot import NormalizedRobotHAL  # noqa: E402
from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL  # noqa: E402
from axis_os.hal.watchdog.system import WatchdogSystem  # noqa: E402
from axis_os.store.runtime.manifeste_json import manifeste_texte  # noqa: E402
from axis_os.store.runtime.play_store import (  # noqa: E402
    PlayStoreRuntime, confirmer_au_terminal,
)
from axis_os.store.runtime.profil import (  # noqa: E402
    Profil, RegistreAppsInvalide, creer_registre_apps, lire_registre_apps,
)
from axis_os.store.sdk.app_base import MissionResult, RobotApp  # noqa: E402
from axis_os.store.sdk.manifest import (  # noqa: E402
    AppCategory, AppManifest, Permission, PricingModel,
)
from axis_os.store.sdk.signing import (  # noqa: E402
    generate_developer_keypair, message_paquet, public_key_fingerprint, sign_app_code,
    sign_app_package,
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


# Clés de test, créées pour ce script et jamais écrites ailleurs que dans un
# dossier temporaire.
A, B = generate_developer_keypair(), generate_developer_keypair()
EMPREINTE_A, EMPREINTE_B = (public_key_fingerprint(k.public_key_b64) for k in (A, B))
DOSSIER = Path(tempfile.mkdtemp(prefix="axis_identite_"))
CLES = DOSSIER / "cles_approuvees.txt"
CLES.write_text(f"{A.public_key_b64}  Dev A\n{B.public_key_b64}  Dev B\n", encoding="utf-8")
os.chmod(CLES, 0o600)

with contextlib.redirect_stdout(io.StringIO()):
    ROBOT = NormalizedRobotHAL(UnitreeG1HAL("g1_identite"), build_g1_normalizer())
    ROBOT.initialize_all()
WD = WatchdogSystem(ROBOT)

MODELE = '''
from pathlib import Path
from axis_os.hal.core.types import ActuatorCommand, ControlMode
from axis_os.store.sdk.app_base import MissionResult, RobotApp
from axis_os.store.sdk.manifest import AppCategory, AppManifest, Permission, PricingModel

# Témoin : écrit dès que le code de l'app s'exécute.
Path(TEMOIN).write_text("le code de l'app a tourné", encoding="utf-8")

class AppIdentite(RobotApp):
    def manifest(self):
        return AppManifest(app_id=APP_ID, name="Identité", version=VERSION,
                           author="Atelier Dupont", author_email="t@t.com", description="d",
                           category=AppCategory.UTILITIES,
                           permissions=[Permission.ACTUATORS_WRITE_ARMS],
                           pricing_model=PricingModel.ONE_TIME, base_price_eur=10.0)

    def on_start(self, ctx):
        return True

    def on_stop(self, ctx):
        pass

    def run_mission(self, ctx, params):
        statuts = ctx.move_arms({"left_elbow_joint": ActuatorCommand(
            mode=ControlMode.POSITION, position=0.05)})
        return MissionResult(success=all(s.name == "OK" for s in statuts.values()),
                             mission_id="", app_id="", duration_s=0.0, items_processed=1)
'''
N = [0]


def paquet(cle, version="1.0.0", app_id="com.test.identite", permissions=None,
           prix=(PricingModel.ONE_TIME, 10.0)):
    """(source, manifeste déclaré, signature, témoin) d'une app, comme les
    prépare un développeur sur sa machine."""
    N[0] += 1
    temoin = DOSSIER / f"temoin_{N[0]}"
    source = (MODELE.replace("TEMOIN", repr(str(temoin))).replace("APP_ID", repr(app_id))
              .replace("VERSION", repr(version)))
    manifeste = manifeste_texte(AppManifest(
        app_id=app_id, name="Identité", version=version, author="Atelier Dupont",
        author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
        permissions=[Permission.ACTUATORS_WRITE_ARMS] if permissions is None else permissions,
        pricing_model=prix[0], base_price_eur=prix[1]))
    return source, manifeste, sign_app_package(manifeste, source, cle.private_key_b64), temoin


class Operateur:
    """Tient le rôle de l'humain : note ce qu'on lui montre, puis accepte."""
    recoit_details = True

    def __init__(self, attente_s=0.0):
        self.vus, self.attente_s = [], attente_s

    def __call__(self, manifest, permissions, details=None):
        self.vus.append((manifest.version, details))
        time.sleep(self.attente_s)
        return True


def runtime(registre, operateur=None, **kw):
    return silencieux(PlayStoreRuntime, ROBOT, WD, profil=Profil.PRODUCTION, cles_approuvees=CLES,
                      registre_apps=registre, confirmer_permissions=operateur or Operateur(), **kw)[0]


def installer(rt, cle, version="1.0.0", methode="install_source", deja=None, **kw):
    """Installe un paquet neuf, ou `deja` (un paquet déjà installé, à l'octet près)."""
    source, manifeste, signature, _ = deja or paquet(cle, version, **kw)
    return silencieux(getattr(rt, methode), source, manifeste=manifeste, code_signature=signature,
                      developer_public_key=cle.public_key_b64, origine="identite")


def nouveau_registre():
    N[0] += 1
    chemin = DOSSIER / f"registre_{N[0]}.json"
    creer_registre_apps(chemin)
    return chemin


section("1. Le registre des apps est exigé, et vérifié au démarrage")
try:
    silencieux(PlayStoreRuntime, ROBOT, WD, profil=Profil.PRODUCTION, cles_approuvees=CLES,
               confirmer_permissions=Operateur())
    check("PRODUCTION sans registre des apps : refusé", False)
except ValueError as exc:
    check("PRODUCTION sans registre des apps : refusé", "registre" in str(exc), str(exc))


def refuse_registre(label, contenu=None, droits=0o600, lien=False, absent=False):
    N[0] += 1
    chemin = DOSSIER / f"registre_abime_{N[0]}.json"
    if not absent:
        cible = DOSSIER / f"cible_{N[0]}.json" if lien else chemin
        cible.write_text(contenu or '{"format": 1, "apps": {}}', encoding="utf-8")
        os.chmod(cible, droits)
        if lien:
            chemin.symlink_to(cible)
    try:
        runtime(chemin)
        check(label, False, "démarré quand même")
    except RegistreAppsInvalide:
        check(label, True)


refuse_registre("Registre inexistant (faute de chemin, fichier supprimé)", absent=True)
refuse_registre("Registre qui n'est pas du JSON", "pas du json")
refuse_registre("Registre avec une clé JSON en double",
                '{"format": 1, "apps": {}, "apps": {}}')
refuse_registre("Entrée dont la clé n'est pas une clé Ed25519",
                json.dumps({"format": 1, "apps": {"com.x.y": {"cle": "abc", "version": "1.0.0"}}}))
refuse_registre("Entrée dont la version est ambiguë (« 1_0.0.0 »)",
                json.dumps({"format": 1, "apps": {"com.x.y": {"cle": A.public_key_b64,
                                                              "version": "1_0.0.0"}}}))
refuse_registre("Registre modifiable par le groupe", droits=0o660)
refuse_registre("Registre qui est un lien symbolique", lien=True)

section("2. Un app_id est lié à la clé qui l'a installé la première fois")
REG = nouveau_registre()
rt1 = runtime(REG)
PAQUET_A = paquet(A)
ok, _ = installer(rt1, A, deja=PAQUET_A)
check("A installe com.test.identite v1.0.0", ok is True)
lien = lire_registre_apps(REG).get("com.test.identite") or {}
check("… le registre, sur disque, lie l'app_id à A",
      {k: lien.get(k) for k in ("cle", "version")} == {"cle": A.public_key_b64,
                                                       "version": "1.0.0"}, str(lien))
check("… avec l'empreinte du paquet installé (manifeste et code)",
      lien.get("empreinte") == hashlib.sha256(message_paquet(PAQUET_A[1], PAQUET_A[0])).hexdigest(),
      str(lien))
rt2 = runtime(REG)  # redémarrage de l'OS : rien n'est gardé en mémoire
ok, sortie = installer(rt2, B, "2.0.0")
check("Après redémarrage, B ne peut pas installer le même app_id",
      ok is False and "com.test.identite" not in rt2._apps)
check("… et le message montre la clé à laquelle il est lié",
      EMPREINTE_A in sortie and EMPREINTE_B in sortie, sortie[-400:])
ok, _ = installer(rt2, A, deja=PAQUET_A)
check("A le réinstalle (même version) après redémarrage", ok is True)
rt3 = runtime(REG)  # nouveau redémarrage : l'app n'est pas installée dans ce runtime
ok, sortie = installer(rt3, A)
check("Même version, signée par A, mais un autre contenu : refusée (il faut une "
      "version plus récente)", ok is False and "com.test.identite" not in rt3._apps
      and "AUTRE contenu" in sortie, sortie[-300:])
ok, _ = installer(rt2, B, "2.0.0", methode="update_source")
check("B ne peut pas non plus le mettre à jour", ok is False
      and rt2._apps["com.test.identite"].manifest.version == "1.0.0")

section("3. La confirmation montre la clé signataire, pas seulement l'auteur")
operateur = Operateur()
rt = runtime(nouveau_registre(), operateur)
ok, sortie = installer(rt, B)
details = operateur.vus[-1][1] if operateur.vus else None
check("L'humain reçoit l'empreinte de la clé qui a signé (B)",
      details is not None and details.signataire == EMPREINTE_B, str(details))
check("… le nom inscrit pour elle dans le fichier des clés approuvées, et ce fichier",
      details is not None and details.nom_signataire == "Dev B"
      and details.fichier_cles == str(CLES), str(details))
check("… et l'état de la liaison (première installation de cet app_id)",
      details is not None and "première installation" in (details.liaison or ""), str(details))
check("Le journal présente l'auteur comme DÉCLARÉ, et la clé qui a signé",
      "Auteur déclaré : Atelier Dupont" in sortie and EMPREINTE_B in sortie and "Dev B" in sortie,
      sortie[-400:])
vrai_input = builtins.input
builtins.input = lambda invite="": (print(invite), "o")[1]
try:
    _, ecran = silencieux(confirmer_au_terminal, rt._apps["com.test.identite"].manifest,
                          [Permission.ACTUATORS_WRITE_ARMS], details)
finally:
    builtins.input = vrai_input
check("Au terminal : empreinte, nom du signataire, auteur déclaré et liaison",
      EMPREINTE_B in ecran and "Dev B" in ecran and "auteur déclaré" in ecran
      and "première installation" in ecran, ecran)

section("4. Mise à jour en production : même clé, version plus récente")
rt = runtime(nouveau_registre())
installer(rt, A, "1.0.0")
silencieux(rt.start_app, "com.test.identite")
res, _ = silencieux(rt.run_mission, "com.test.identite", {})
check("v1.0.0 démarrée, mission réussie et facturée une fois (achat unique)",
      res is not None and res.success and rt._apps["com.test.identite"].one_time_billed,
      getattr(res, "error_message", None))
for version, cle, raison in [("1.0.0", A, "même version"), ("0.9.9", A, "version plus ancienne"),
                             ("2.0.0", B, "autre clé")]:
    ok, _ = installer(rt, cle, version, methode="update_source")
    check(f"update_source refusé : {raison}", ok is False
          and rt._apps["com.test.identite"].manifest.version == "1.0.0")
ok, sortie = installer(rt, A, "1.1.0", methode="update_source")
entree = rt._apps["com.test.identite"]
check("update_source A v1.1.0 : accepté", ok is True and entree.manifest.version == "1.1.0",
      sortie[-300:])
check("… ancienne version arrêtée, nouvelle non redémarrée d'office", entree.is_started is False)
check("… achat unique déjà payé : pas refacturé", entree.one_time_billed is True)
check("… le registre retient la nouvelle version",
      lire_registre_apps(rt._chemin_registre)["com.test.identite"]["version"] == "1.1.0")


class AppObjet(RobotApp):
    def manifest(self):
        return AppManifest(app_id="com.test.identite", name="Objet", version="9.0.0",
                           author="T", author_email="t@t.com", description="d",
                           category=AppCategory.UTILITIES)

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0)


ok, _ = silencieux(rt.update_app, AppObjet())
check("update_app(objet) toujours refusé en production", ok is False)

section("5. La version la plus récente survit au redémarrage")
REG = nouveau_registre()
PAQUET_V2 = paquet(A, "2.0.0")
ok, _ = installer(runtime(REG), A, deja=PAQUET_V2)
check("A installe v2.0.0", ok is True)
rt2 = runtime(REG)
ok, sortie = installer(rt2, A, "1.0.0")
check("Après redémarrage, v1.0.0 (plus ancienne) : refusée", ok is False
      and "2.0.0" in sortie, sortie[-300:])
ok, _ = installer(rt2, A, deja=PAQUET_V2)
check("v2.0.0 (la même) : réinstallée", ok is True)
ok, _ = installer(runtime(REG), A, "2.1.0")
check("v2.1.0 (plus récente) : installée, et retenue",
      ok is True and lire_registre_apps(REG)["com.test.identite"]["version"] == "2.1.0")

section("6. Aucune ligne de l'app ne tourne avant l'humain")


class OperateurVigilant(Operateur):
    def __init__(self, temoin):
        super().__init__()
        self.temoin, self.temoin_vu = temoin, None

    def __call__(self, manifest, permissions, details=None):
        self.temoin_vu = self.temoin.exists()
        self.permissions = [p.name for p in permissions]
        return True


source, manifeste, signature, temoin = paquet(A)
vigilant = OperateurVigilant(temoin)
rt = runtime(nouveau_registre(), vigilant)
ok, _ = silencieux(rt.install_source, source, manifeste=manifeste, code_signature=signature,
                   developer_public_key=A.public_key_b64, origine="identite")
check("Au moment où l'humain décide, le code de l'app n'a jamais tourné",
      ok is True and vigilant.temoin_vu is False, f"témoin vu : {vigilant.temoin_vu}")
check("… et il décide sur les permissions du manifeste déclaré",
      vigilant.permissions == ["ACTUATORS_WRITE_ARMS"])
silencieux(rt.start_app, "com.test.identite")
check("Le code ne tourne qu'ensuite, dans le processus de l'app", temoin.exists())

section("7. La signature lie le manifeste déclaré et le code")
rt = runtime(nouveau_registre())
src1, man1, sig1, _ = paquet(A, "1.0.0")
src2, man2, sig2, _ = paquet(A, "2.0.0")
sans_permission = man1.replace('"ACTUATORS_WRITE_ARMS"', "")
essais = [
    ("Manifeste modifié après signature (permission retirée)", src1, sans_permission, sig1),
    ("Code modifié après signature", src1 + "\n# ajout\n", man1, sig1),
    ("Manifeste d'un paquet avec le code d'un autre", src1, man2, sig1),
    ("Même recombinaison, signature de l'autre paquet", src1, man2, sig2),
    ("Signature du code seul (ancien format)", src1, man1, sign_app_code(src1, A.private_key_b64)),
]
for label, src, man, sig in essais:
    ok, sortie = silencieux(rt.install_source, src, manifeste=man, code_signature=sig,
                            developer_public_key=A.public_key_b64, origine="identite")
    check(f"{label} : refusé", ok is False and "Signature invalide" in sortie, sortie[-200:])
try:
    rt.install_source(src1, code_signature=sig1, developer_public_key=A.public_key_b64)
    check("Sans manifeste déclaré : refusé", False)
except TypeError:
    check("Sans manifeste déclaré : refusé (le code n'est jamais exécuté pour le lire)", True)
check("Rien n'a été installé", rt._apps == {})

section("8. Versions et app_id ambigus refusés")
rt = runtime(nouveau_registre())
for version in (" 1.0.0", "1_0.0.0", "١.0.0", "1.0.0-beta", "+1.0.0", "01.0.0"):
    ok, _ = installer(rt, A, version)
    check(f"Version {version!r} : refusée", ok is False)
installer(rt, A)
for sosie, quoi in [("com.test.identite ", "espace final"), ("Com.Test.Identite", "majuscules"),
                    ("com.test.identitе", "lettre cyrillique"),
                    ("com.test.identite\u0301", "accent combinant")]:
    ok, _ = installer(rt, B, app_id=sosie)
    check(f"Sosie de l'app_id de A ({quoi}), signé par B : refusé", ok is False)
source, manifeste, signature, _ = paquet(A, "2.0.0")
ok, _ = silencieux(rt.install_source, source, manifeste=manifeste.replace("Identité", "\ud800"),
                   code_signature=signature, developer_public_key=A.public_key_b64)
check("Manifeste qui ne s'encode pas en UTF-8 : refusé, sans exception", ok is False)

section("9. Registre : fichier temporaire resté d'une coupure, install() hors production")
REG = nouveau_registre()
(REG.parent / f".{REG.name}.{os.getpid()}.tmp").write_text("reste d'une coupure")
ok, sortie = installer(runtime(REG), A)
check("Un fichier temporaire laissé par une coupure ne bloque pas le registre", ok is True,
      sortie[-200:])


class AppNonSignee(RobotApp):
    def manifest(self):
        return AppManifest(app_id="com.test.identite", name="Copie", version="9.0.0",
                           author="Atelier Dupont", author_email="t@t.com", description="d",
                           category=AppCategory.UTILITIES)

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0)


dev = silencieux(PlayStoreRuntime, ROBOT, WD, profil=Profil.DEVELOPPEMENT, cles_approuvees=CLES,
                 registre_apps=REG, confirmer_permissions=Operateur())[0]
ok, sortie = silencieux(dev.install, AppNonSignee())
check("Hors production, avec registre : une app non signée ne prend pas un app_id lié",
      ok is False and "com.test.identite" not in dev._apps, sortie[-200:])

class AppVersion(RobotApp):
    VERSION = "2.0.0"

    def manifest(self):
        return AppManifest(app_id="com.test.version", name="Version", version=self.VERSION,
                           author="T", author_email="t@t.com", description="d",
                           category=AppCategory.UTILITIES)

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0)


AppVersion.VERSION = "2.0.0"
silencieux(dev.install, AppVersion())
silencieux(dev.start_app, "com.test.version")
AppVersion.VERSION = "1.0.0"
ok, sortie = silencieux(dev.update_app, AppVersion(), allow_downgrade=True)
entree = dev._apps["com.test.version"]
check("Avec registre, update_app(allow_downgrade=True) vers une version plus ancienne : refusé "
      "avant d'arrêter l'app", ok is False and entree.manifest.version == "2.0.0"
      and entree.is_started is True and "allow_downgrade" in sortie, sortie[-300:])

import argparse  # noqa: E402
from axis_os.cli.main import cmd_init  # noqa: E402
from axis_os.store.sdk.manifest import _APP_ID  # noqa: E402
cwd = os.getcwd()
os.chdir(tempfile.mkdtemp(prefix="axis_init_"))
try:
    code, sortie = silencieux(cmd_init, argparse.Namespace(name="Mon App", author="李雷",
                                                           email="d@d.d"))
    genere = Path("mon_app/app.py").read_text(encoding="utf-8") if code == 0 else ""
    check("axis-os init, auteur sans lettre latine : un app_id valide est généré",
          code == 0 and 'app_id="com.anonymous.mon_app"' in genere, sortie[-200:])
    code, sortie = silencieux(cmd_init, argparse.Namespace(name="Робот", author="Dev",
                                                           email="d@d.d"))
    check("axis-os init, nom sans lettre latine : refus clair, rien de créé",
          code == 1 and "app_id" in sortie, sortie[-200:])
finally:
    os.chdir(cwd)

section("10. Pendant une mise à jour")


class OperateurPresse(Operateur):
    """Pendant qu'on lui demande de confirmer, quelqu'un d'autre agit."""
    def __init__(self, action):
        super().__init__()
        self.action, self.resultat = action, None

    def __call__(self, manifest, permissions, details=None):
        if manifest.version != "1.0.0":
            self.resultat = self.action()
        return True


REG = nouveau_registre()
autre = paquet(B, "1.0.0", app_id="com.test.autre")
pendant = OperateurPresse(lambda: silencieux(rt_maj.start_app, "com.test.identite")[0])
rt_maj = runtime(REG, pendant)
installer(rt_maj, A, "1.0.0")
silencieux(rt_maj.start_app, "com.test.identite")
ok, _ = installer(rt_maj, A, "1.1.0", methode="update_source")
check("Pendant la confirmation d'une mise à jour, l'ancienne version ne peut pas être "
      "redémarrée", ok is True and pendant.resultat is False
      and rt_maj._apps["com.test.identite"].manifest.version == "1.1.0"
      and rt_maj._apps["com.test.identite"].is_started is False, str(pendant.resultat))
pendant.action = lambda: silencieux(rt_maj.install_source, autre[0], manifeste=autre[1],
                                    code_signature=autre[2],
                                    developer_public_key=B.public_key_b64)[0]
ok, _ = installer(rt_maj, A, "1.2.0", methode="update_source")
check("… ni une autre installation depuis la source commencer",
      ok is True and pendant.resultat is False)
AppVersion.VERSION = "1.0.0"
pendant_dev = OperateurPresse(lambda: silencieux(rt_dev.install, AppVersion())[0])
rt_dev = silencieux(PlayStoreRuntime, ROBOT, WD, profil=Profil.DEVELOPPEMENT, cles_approuvees=CLES,
                    registre_apps=nouveau_registre(), confirmer_permissions=pendant_dev)[0]
installer(rt_dev, A, "1.0.0")
ok, _ = installer(rt_dev, A, "1.1.0", methode="update_source")
check("… ni, hors production, une installation d'objet app (install), qui écrit aussi le "
      "registre", ok is True and pendant_dev.resultat is False
      and "com.test.version" not in rt_dev._apps, str(pendant_dev.resultat))

import axis_os.store.runtime.profil as module_profil  # noqa: E402
vrai_open = module_profil.os.open


def open_sans_dossier(chemin, drapeaux, *a, **kw):
    if drapeaux & os.O_DIRECTORY:
        raise OSError(5, "Input/output error")
    return vrai_open(chemin, drapeaux, *a, **kw)


REG = nouveau_registre()
rt_fsync = runtime(REG)
module_profil.os.open = open_sans_dossier
try:
    ok, sortie = installer(rt_fsync, A, "1.0.0")
finally:
    module_profil.os.open = vrai_open
check("fsync du dossier en échec après l'écriture du registre : installation gardée, "
      "avec un avertissement, disque et mémoire d'accord",
      ok is True and "durabilité" in sortie
      and lire_registre_apps(REG).get("com.test.identite") == rt_fsync._registre.get(
          "com.test.identite"), sortie[-200:])

section("11. Deux installations simultanées du même app_id")
REG = nouveau_registre()
rt = runtime(REG, Operateur(attente_s=0.3))
resultats = {}


def lancer(nom, cle):
    resultats[nom] = installer(rt, cle)[0]


fils = [threading.Thread(target=lancer, args=(nom, cle)) for nom, cle in (("A", A), ("B", B))]
for f in fils:
    f.start()
for f in fils:
    f.join(timeout=30)
gagnant = [nom for nom, ok in resultats.items() if ok]
lien = lire_registre_apps(REG).get("com.test.identite")
check("Une seule des deux réussit", len(gagnant) == 1, str(resultats))
check("… et le registre lie l'app_id à sa clé",
      len(gagnant) == 1 and lien is not None
      and lien["cle"] == {"A": A, "B": B}[gagnant[0]].public_key_b64, str(lien))

section("12. Prix : montré à l'humain, et confirmé quand il change")
operateur = Operateur()
rt = runtime(nouveau_registre(), operateur)
installer(rt, A, "1.0.0")
details = operateur.vus[-1][1] if operateur.vus else None
check("À l'installation, l'humain voit le prix (commission comprise) et son modèle",
      getattr(details, "prix", None) == "14.29 € (achat unique)"
      and getattr(details, "prix_precedent", None) is None, str(details))
LECTURE = [Permission.ACTUATORS_READ]   # aucune permission DANGEROUS : rien d'autre à confirmer
operateur = Operateur()
rt = runtime(nouveau_registre(), operateur)
installer(rt, A, "1.0.0", permissions=LECTURE)
ok, _ = installer(rt, A, "1.1.0", methode="update_source", permissions=LECTURE)
check("Mise à jour au même prix, sans permission DANGEROUS : rien à confirmer",
      ok is True and operateur.vus == [], str(operateur.vus))
ok, _ = installer(rt, A, "1.2.0", methode="update_source", permissions=LECTURE,
                  prix=(PricingModel.ONE_TIME, 5.0))
check("… ni pour une baisse de prix", ok is True and operateur.vus == [], str(operateur.vus))
ok, _ = installer(rt, A, "1.3.0", methode="update_source", permissions=LECTURE,
                  prix=(PricingModel.PAY_PER_MISSION, 1.0))
details = operateur.vus[-1][1] if operateur.vus else None
check("Passage d'un achat unique à un paiement par mission : l'humain doit l'accepter",
      ok is True and getattr(details, "prix", None) == "1.43 € (à chaque mission)"
      and getattr(details, "prix_precedent", None) == "7.14 € (achat unique)", str(details))
ok, _ = installer(rt, A, "1.4.0", methode="update_source", permissions=LECTURE,
                  prix=(PricingModel.PAY_PER_MISSION, 3.0))
details = operateur.vus[-1][1] if len(operateur.vus) == 2 else None
check("Hausse du prix : l'humain doit l'accepter",
      ok is True and getattr(details, "prix_precedent", None) == "1.43 € (à chaque mission)",
      str(details))


class OperateurQuiRefuse(Operateur):
    def __call__(self, manifest, permissions, details=None):
        super().__call__(manifest, permissions, details)
        return False


rt = runtime(nouveau_registre(), OperateurQuiRefuse())
installer(rt, A, "1.0.0", permissions=LECTURE)
ok, _ = installer(rt, A, "1.1.0", methode="update_source", permissions=LECTURE,
                  prix=(PricingModel.SUBSCRIPTION, 10.0))
check("Nouveau prix refusé par l'humain : la mise à jour n'a pas lieu",
      ok is False and rt._apps["com.test.identite"].manifest.version == "1.0.0"
      and rt._apps["com.test.identite"].manifest.pricing_model == PricingModel.ONE_TIME)
rt = silencieux(PlayStoreRuntime, ROBOT, WD, profil=Profil.DEVELOPPEMENT, cles_approuvees=CLES,
                registre_apps=nouveau_registre())[0]
installer(rt, A, "1.0.0", permissions=LECTURE)
ok, _ = installer(rt, A, "1.1.0", methode="update_source", permissions=LECTURE,
                  prix=(PricingModel.ONE_TIME, 50.0))
check("Sans confirmation configurée, un nouveau prix est refusé par défaut",
      ok is False and rt._apps["com.test.identite"].manifest.version == "1.0.0")
reponses = []
vrai_input = builtins.input
builtins.input = lambda question: reponses.append(question) or "n"
try:
    source, manifeste, _, _ = paquet(A, "2.0.0", permissions=LECTURE,
                                     prix=(PricingModel.SUBSCRIPTION, 10.0))
    from axis_os.store.runtime.manifeste_json import manifeste_depuis_texte  # noqa: E402
    from axis_os.store.runtime.play_store import DetailsConfirmation, confirmer_au_terminal  # noqa: E402
    try:
        details = DetailsConfirmation("PRODUCTION", EMPREINTE_A, "Dev A", str(CLES),
                                      prix_precedent="14.29 € (achat unique)")
    except TypeError:   # code d'avant cette correction : pas de champ prix_precedent
        details = DetailsConfirmation("PRODUCTION", EMPREINTE_A, "Dev A", str(CLES))
    accepte, sortie = silencieux(confirmer_au_terminal, manifeste_depuis_texte(manifeste), [],
                                 details)
finally:
    builtins.input = vrai_input
check("Au terminal : le nouveau prix, l'ancien, et une question sur le prix (pas sur des "
      "permissions vides)", accepte is False and "PRIX MODIFIÉ" in sortie
      and "achat unique" in sortie and "abonnement" in sortie
      and reponses and "nouveau prix" in reponses[0], sortie[-400:])

section("13. Une mise à jour garde les compteurs de l'app")
rt = runtime(nouveau_registre())
installer(rt, A, "1.0.0")
silencieux(rt.start_app, "com.test.identite")
res, _ = silencieux(rt.run_mission, "com.test.identite", {})
avant = rt._apps["com.test.identite"]
compteurs = (avant.mission_count, avant.total_revenue)
check("Une mission facturée (achat unique) avant la mise à jour",
      res is not None and res.success and compteurs[0] == 1 and compteurs[1] > 0, str(compteurs))
ok, _ = installer(rt, A, "1.1.0", methode="update_source")
apres = rt._apps["com.test.identite"]
check("Après update_source : missions et revenu conservés (rapport financier juste)",
      ok is True and (apres.mission_count, apres.total_revenue) == compteurs,
      str((apres.mission_count, apres.total_revenue)))
AppVersion.VERSION = "1.0.0"
rt = silencieux(PlayStoreRuntime, ROBOT, WD, profil=Profil.DEVELOPPEMENT,
                confirmer_permissions=Operateur())[0]
silencieux(rt.install, AppVersion())
rt._apps["com.test.version"].mission_count = 4
rt._apps["com.test.version"].total_revenue = 12.5
AppVersion.VERSION = "1.1.0"
ok, _ = silencieux(rt.update_app, AppVersion())
apres = rt._apps.get("com.test.version")
check("Après update_app (objet, hors production) : de même",
      ok is True and apres is not None and (apres.mission_count, apres.total_revenue) == (4, 12.5),
      str(apres and (apres.mission_count, apres.total_revenue)))

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
