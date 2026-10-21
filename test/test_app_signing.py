"""
AXIS-OS — Tests Signature des apps
=====================================
Valide axis_os/store/sdk/signing.py (génération de clés, signature,
vérification) et son intégration dans PlayStoreRuntime.install() — le
mécanisme qui empêche un fichier .py quelconque de se faire passer
pour une app AXIS-OS légitime, et détecte toute modification du code
après signature.

Les scénarios d'intégration (sections 3-5) créent de VRAIS fichiers
sur disque plutôt que des classes définies en mémoire : inspect.
getsource() — utilisé par PlayStoreRuntime pour récupérer le code à
vérifier — échoue sur du code sans fichier source réel
(OSError: source code not available), donc un test fidèle au
comportement réel doit passer par de vrais fichiers, pas une classe
inline.
"""

import sys
import tempfile
from pathlib import Path
sys.path.insert(0, ".")

from axis_os.store.sdk.signing import (
    generate_developer_keypair, sign_app_code, verify_app_signature,
    sign_app_file, verify_app_file, SignatureError, DeveloperKeypair,
)
from axis_os.store.sdk.manifest import AppManifest, AppCategory
from axis_os.store.runtime.play_store import PlayStoreRuntime, accepter_tout_SIMULATION
from axis_os.store.runtime.profil import Profil
from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer
from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL
from axis_os.hal.watchdog.system import WatchdogSystem

FAIL = 0


def check(description: str, condition: bool) -> None:
    global FAIL
    status = "✓" if condition else "✗"
    print(f"  [{status}] {description}")
    if not condition:
        FAIL += 1


def section(title: str) -> None:
    print(f"\n{'=' * 66}\n  {title}\n{'=' * 66}")


def make_runtime() -> PlayStoreRuntime:
    robot = NormalizedRobotHAL(UnitreeG1HAL("g1_signing_test"), build_g1_normalizer())
    robot.initialize_all()
    wd = WatchdogSystem(robot)
    return PlayStoreRuntime(robot, wd, confirmer_permissions=accepter_tout_SIMULATION,
                            profil=Profil.SIMULATION)


# ══════════════════════════════════════════════════════════════════

section("1 — generate_developer_keypair() : format et unicité")

kp1 = generate_developer_keypair()
kp2 = generate_developer_keypair()
check("Retourne un DeveloperKeypair", isinstance(kp1, DeveloperKeypair))
check("Clé privée non vide", len(kp1.private_key_b64) > 0)
check("Clé publique non vide", len(kp1.public_key_b64) > 0)
check("Deux appels produisent des clés DIFFÉRENTES (aléatoire)",
      kp1.private_key_b64 != kp2.private_key_b64)

# ══════════════════════════════════════════════════════════════════

section("2 — sign_app_code() / verify_app_signature() : cas de base")

kp = generate_developer_keypair()
code = "def run_mission(ctx, params): return True"
sig = sign_app_code(code, kp.private_key_b64)

check("Signature valide sur code intact",
      verify_app_signature(code, sig, kp.public_key_b64))
check("Signature invalide si code modifié (1 caractère)",
      not verify_app_signature(code + " ", sig, kp.public_key_b64))
check("Signature invalide avec la mauvaise clé publique",
      not verify_app_signature(code, sig, generate_developer_keypair().public_key_b64))

try:
    verify_app_signature(code, sig, "clé-invalide-pas-du-base64!!!")
    check("Clé publique malformée lève SignatureError", False)
except SignatureError:
    check("Clé publique malformée lève SignatureError", True)

try:
    sign_app_code(code, "clé-privée-invalide!!!")
    check("Clé privée malformée lève SignatureError", False)
except SignatureError:
    check("Clé privée malformée lève SignatureError", True)

try:
    verify_app_signature(code, "signature-invalide-pas-du-base64!!!", kp.public_key_b64)
    check("Signature malformée lève SignatureError", False)
except SignatureError:
    check("Signature malformée lève SignatureError", True)

# ══════════════════════════════════════════════════════════════════

section("3 — sign_app_file() / verify_app_file()")

with tempfile.TemporaryDirectory() as tmp:
    path = Path(tmp) / "app.py"
    path.write_text("class MyApp:\n    pass\n")

    kp = generate_developer_keypair()
    sig = sign_app_file(path, kp.private_key_b64)
    check("Signature de fichier valide", verify_app_file(path, sig, kp.public_key_b64))

    path.write_text("class MyApp:\n    pass\n# ligne ajoutée après signature\n")
    check("Fichier modifié après signature -> invalide",
          not verify_app_file(path, sig, kp.public_key_b64))

# ══════════════════════════════════════════════════════════════════

section("4 — AppManifest.validate() : code_signature / developer_public_key")

m_none = AppManifest(
    app_id="com.test.a", name="A", version="1.0.0", author="T",
    author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
)
check("Aucun des deux champs -> aucune erreur (rétrocompatible)", m_none.validate() == [])

m_both = AppManifest(
    app_id="com.test.b", name="B", version="1.0.0", author="T",
    author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
    code_signature="sig", developer_public_key="key",
)
check("Les deux champs renseignés -> aucune erreur de cohérence",
      not any("ENSEMBLE" in e for e in m_both.validate()))

m_only_sig = AppManifest(
    app_id="com.test.c", name="C", version="1.0.0", author="T",
    author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
    code_signature="sig",
)
errors = m_only_sig.validate()
check("Signature SANS clé publique -> erreur détectée",
      any("ENSEMBLE" in e for e in errors))

m_only_key = AppManifest(
    app_id="com.test.d", name="D", version="1.0.0", author="T",
    author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
    developer_public_key="key",
)
errors2 = m_only_key.validate()
check("Clé publique SANS signature -> erreur détectée",
      any("ENSEMBLE" in e for e in errors2))

# ══════════════════════════════════════════════════════════════════

section("5 — PlayStoreRuntime.install() : intégration bout en bout")

with tempfile.TemporaryDirectory() as tmp:
    sys.path.insert(0, tmp)

    kp = generate_developer_keypair()

    # --- App correctement signée ---
    signed_path = Path(tmp) / "app_signed_ok.py"
    signed_path.write_text(f'''
import sys
sys.path.insert(0, ".")
import inspect
from axis_os.store.sdk.signing import sign_app_code
from axis_os.store.sdk.manifest import AppManifest, AppCategory
from axis_os.store.sdk.app_base import RobotApp

_PRIVATE_KEY = "{kp.private_key_b64}"
_PUBLIC_KEY = "{kp.public_key_b64}"

class SignedOkApp(RobotApp):
    def manifest(self):
        # Signe le FICHIER SOURCE ENTIER, pas seulement la classe —
        # cohérent avec PlayStoreRuntime.install() qui vérifie
        # désormais inspect.getsourcefile(type(app)) dans son
        # intégralité (voir play_store.py, correctif E1 d'un audit
        # de sécurité : signer uniquement la classe laissait tout
        # code au niveau module hors de la portée de la signature).
        source_file = inspect.getsourcefile(type(self))
        with open(source_file, "r", encoding="utf-8") as f:
            source = f.read()
        sig = sign_app_code(source, _PRIVATE_KEY)
        return AppManifest(
            app_id="com.test.signedok", name="SignedOkApp", version="1.0.0",
            author="T", author_email="t@t.com", description="d",
            category=AppCategory.UTILITIES,
            code_signature=sig, developer_public_key=_PUBLIC_KEY,
        )
    def on_start(self, ctx): pass
    def on_stop(self, ctx): pass
    def run_mission(self, ctx, params): pass
''')
    import importlib
    mod_signed = importlib.import_module("app_signed_ok")
    runtime1 = make_runtime()
    check("App correctement signée s'installe",
          runtime1.install(mod_signed.SignedOkApp()))

    # --- App non signée (rétrocompatibilité) ---
    unsigned_path = Path(tmp) / "app_unsigned.py"
    unsigned_path.write_text('''
from axis_os.store.sdk.manifest import AppManifest, AppCategory
from axis_os.store.sdk.app_base import RobotApp

class UnsignedApp(RobotApp):
    def manifest(self):
        return AppManifest(
            app_id="com.test.unsigned", name="UnsignedApp", version="1.0.0",
            author="T", author_email="t@t.com", description="d",
            category=AppCategory.UTILITIES,
        )
    def on_start(self, ctx): pass
    def on_stop(self, ctx): pass
    def run_mission(self, ctx, params): pass
''')
    mod_unsigned = importlib.import_module("app_unsigned")
    runtime2 = make_runtime()
    check("App SANS signature s'installe normalement (rétrocompatible)",
          runtime2.install(mod_unsigned.UnsignedApp()))

    # --- App avec signature calculée sur un AUTRE contenu (falsifiée) ---
    tampered_path = Path(tmp) / "app_tampered.py"
    other_kp = generate_developer_keypair()
    fake_sig = sign_app_code("ceci n'est pas le vrai code de cette classe",
                               other_kp.private_key_b64)
    tampered_path.write_text(f'''
from axis_os.store.sdk.manifest import AppManifest, AppCategory
from axis_os.store.sdk.app_base import RobotApp

class TamperedApp(RobotApp):
    def manifest(self):
        return AppManifest(
            app_id="com.test.tampered", name="TamperedApp", version="1.0.0",
            author="T", author_email="t@t.com", description="d",
            category=AppCategory.UTILITIES,
            code_signature="{fake_sig}",
            developer_public_key="{other_kp.public_key_b64}",
        )
    def on_start(self, ctx): pass
    def on_stop(self, ctx): pass
    def run_mission(self, ctx, params): pass
''')
    mod_tampered = importlib.import_module("app_tampered")
    runtime3 = make_runtime()
    check("App avec signature ne correspondant pas au code réel REFUSÉE",
          not runtime3.install(mod_tampered.TamperedApp()))

    sys.path.remove(tmp)

# ══════════════════════════════════════════════════════════════════

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)

if FAIL == 0:
    print("\n  ✓ Génération de clés Ed25519 correcte")
    print("  ✓ Signature/vérification détecte toute modification de code")
    print("  ✓ Validation de manifest cohérente (signature+clé ensemble)")
    print("  ✓ PlayStoreRuntime refuse une app dont le code ne correspond")
    print("    pas à sa signature déclarée")
    print("  ✓ Rétrocompatible avec les apps non signées")

sys.exit(0 if FAIL == 0 else 1)
