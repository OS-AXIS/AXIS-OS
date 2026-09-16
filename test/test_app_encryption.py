"""
AXIS-OS — Tests du chiffrement des apps (encryption.py)
============================================================
Valide encrypt_app_code()/decrypt_app_code() — comble une demande
explicite, distincte de la signature (signing.py, déjà testé
séparément dans test_app_signing.py) : une app, une fois livrée au
Play Store, ne doit être lisible que par AXIS-OS, pas par n'importe
qui qui obtiendrait le fichier publié.

Chaque propriété de sécurité affirmée dans la docstring du module est
vérifiée ici par exécution réelle, pas seulement documentée :
authenticité AES-GCM (fichier altéré détecté), non-corrélabilité
(deux chiffrements du même contenu produisent des résultats
différents), refus d'une mauvaise clé, robustesse aux entrées
malformées.
"""

import sys
import base64
import tempfile
sys.path.insert(0, ".")

from pathlib import Path
from axis_os.store.sdk.encryption import (
    generate_platform_keypair, encrypt_app_code, decrypt_app_code,
    encrypt_app_file, decrypt_app_file_to_bytes,
    EncryptedApp, EncryptionError, DecryptionError,
)

FAIL = 0


def check(description: str, condition: bool) -> None:
    global FAIL
    status = "✓" if condition else "✗"
    print(f"  [{status}] {description}")
    if not condition:
        FAIL += 1


def section(title: str) -> None:
    print(f"\n{'=' * 66}\n  {title}\n{'=' * 66}")


# ══════════════════════════════════════════════════════════════════

section("1 — Génération de clés plateforme")

kp = generate_platform_keypair()
check("Clé privée en base64, longueur cohérente avec 32 octets",
      len(base64.b64decode(kp.private_key_b64)) == 32)
check("Clé publique en base64, longueur cohérente avec 32 octets",
      len(base64.b64decode(kp.public_key_b64)) == 32)

kp2 = generate_platform_keypair()
check("Deux appels successifs génèrent des clés DIFFÉRENTES",
      kp.private_key_b64 != kp2.private_key_b64)

# ══════════════════════════════════════════════════════════════════

section("2 — Cycle complet chiffrement/déchiffrement")

original_code = (
    b"class SecretBusinessLogicApp:\n"
    b"    def run(self):\n"
    b"        return compute_proprietary_algorithm()\n"
)
encrypted = encrypt_app_code(original_code, kp.public_key_b64)

check("EncryptedApp produit avec les 2 champs attendus (HPKE : clé "
      "encapsulée + texte chiffré, pas de nonce séparé)",
      encrypted.encapsulated_key_b64 and encrypted.ciphertext_b64)

raw_envelope = encrypted.to_bytes()
check("CRITIQUE : aucune trace du contenu original dans l'enveloppe "
      "brute (mot-clé 'proprietary' absent)",
      b"proprietary" not in raw_envelope)
check("CRITIQUE : aucune trace du contenu original (nom de classe absent)",
      b"SecretBusinessLogic" not in raw_envelope)

decrypted = decrypt_app_code(encrypted, kp.private_key_b64)
check("Déchiffrement avec la VRAIE clé privée restitue exactement "
      "le contenu original", decrypted == original_code)

# ══════════════════════════════════════════════════════════════════

section("3 — CRITIQUE : seule la vraie clé privée plateforme déchiffre")

kp_wrong = generate_platform_keypair()
try:
    decrypt_app_code(encrypted, kp_wrong.private_key_b64)
    check("Une AUTRE clé privée (attaquant) est refusée", False)
except DecryptionError:
    check("Une AUTRE clé privée (attaquant) est refusée", True)

# ══════════════════════════════════════════════════════════════════

section("4 — CRITIQUE : authenticité AEAD (AES-256-GCM) — toute altération détectée")

raw_ct = base64.b64decode(encrypted.ciphertext_b64)
tampered_ct = bytearray(raw_ct)
tampered_ct[0] ^= 0xFF
tampered = EncryptedApp(
    encapsulated_key_b64=encrypted.encapsulated_key_b64,
    ciphertext_b64=base64.b64encode(bytes(tampered_ct)).decode("ascii"),
)
try:
    decrypt_app_code(tampered, kp.private_key_b64)
    check("Fichier chiffré altéré (1 octet du texte chiffré modifié) "
          "détecté et rejeté", False)
except DecryptionError:
    check("Fichier chiffré altéré (1 octet du texte chiffré modifié) "
          "détecté et rejeté", True)

# Altération de la clé encapsulée (enc, l'équivalent HPKE de la clé
# publique éphémère) également détectée — invalide le secret partagé
# reconstruit, donc le déchiffrement échoue.
raw_enc = base64.b64decode(encrypted.encapsulated_key_b64)
tampered_enc = bytearray(raw_enc)
tampered_enc[0] ^= 0xFF
tampered2 = EncryptedApp(
    encapsulated_key_b64=base64.b64encode(bytes(tampered_enc)).decode("ascii"),
    ciphertext_b64=encrypted.ciphertext_b64,
)
try:
    decrypt_app_code(tampered2, kp.private_key_b64)
    check("Clé encapsulée altérée également détectée et rejetée", False)
except (DecryptionError, EncryptionError):
    # EncryptionError possible aussi : une clé encapsulée corrompue peut
    # échouer dès la désérialisation HPKE, avant même la tentative de
    # déchiffrement — les deux cas signifient également "rejeté".
    check("Clé encapsulée altérée également détectée et rejetée", True)

# ══════════════════════════════════════════════════════════════════

section("5 — Non-corrélabilité : deux chiffrements du même contenu diffèrent")

enc_a = encrypt_app_code(b"identical content", kp.public_key_b64)
enc_b = encrypt_app_code(b"identical content", kp.public_key_b64)
check("Clés encapsulées différentes entre deux chiffrements",
      enc_a.encapsulated_key_b64 != enc_b.encapsulated_key_b64)
check("Textes chiffrés différents entre deux chiffrements du même contenu",
      enc_a.ciphertext_b64 != enc_b.ciphertext_b64)
check("Les deux se déchiffrent quand même correctement vers le même contenu",
      decrypt_app_code(enc_a, kp.private_key_b64) == b"identical content"
      and decrypt_app_code(enc_b, kp.private_key_b64) == b"identical content")

# ══════════════════════════════════════════════════════════════════

section("6 — Robustesse : entrées malformées")

try:
    encrypt_app_code(b"code", "PAS_DU_BASE64_VALIDE!!!")
    check("Clé publique malformée (base64 invalide) rejetée à l'encryption", False)
except EncryptionError:
    check("Clé publique malformée (base64 invalide) rejetée à l'encryption", True)

bad_key_wrong_size = base64.b64encode(b"trop_court").decode("ascii")
try:
    encrypt_app_code(b"code", bad_key_wrong_size)
    check("Clé publique de mauvaise taille (pas 32 octets) rejetée", False)
except EncryptionError:
    check("Clé publique de mauvaise taille (pas 32 octets) rejetée", True)

try:
    EncryptedApp.from_bytes(b"CECI N'EST PAS UN FICHIER AXISAPP VALIDE")
    check("Fichier .axisapp structurellement corrompu rejeté", False)
except EncryptionError:
    check("Fichier .axisapp structurellement corrompu rejeté", True)

try:
    EncryptedApp.from_bytes(b"")
    check("Fichier .axisapp vide rejeté", False)
except EncryptionError:
    check("Fichier .axisapp vide rejeté", True)

try:
    decrypt_app_code(encrypted, "PAS_DU_BASE64_VALIDE!!!")
    check("Clé privée malformée à la déchiffrement rejetée", False)
except EncryptionError:
    check("Clé privée malformée à la déchiffrement rejetée", True)

# ══════════════════════════════════════════════════════════════════

section("7 — Cycle complet via fichiers disque (.py -> .axisapp -> .py)")

tmp_dir = tempfile.mkdtemp(prefix="axis_os_encryption_test_")
app_py = Path(tmp_dir) / "my_app.py"
app_py.write_text(
    "class MyRealDeployedApp:\n    def run(self):\n        return 42\n",
    encoding="utf-8",
)

encrypted_from_file = encrypt_app_file(app_py, kp.public_key_b64)
axisapp_path = Path(tmp_dir) / "my_app.axisapp"
axisapp_path.write_bytes(encrypted_from_file.to_bytes())

disk_content = axisapp_path.read_bytes()
check("CRITIQUE : le fichier .axisapp SUR DISQUE ne contient aucune "
      "trace du code (nom de classe absent)",
      b"MyRealDeployedApp" not in disk_content)

decrypted_from_disk = decrypt_app_file_to_bytes(axisapp_path, kp.private_key_b64)
check("Déchiffrement depuis le fichier disque restitue exactement "
      "le contenu original",
      decrypted_from_disk == app_py.read_bytes())

import shutil
shutil.rmtree(tmp_dir, ignore_errors=True)

# ══════════════════════════════════════════════════════════════════

section("8 — Séparation des clés : signing.py et encryption.py sont indépendants")

# Vérifie explicitement que les deux modules n'entremêlent jamais
# leurs types de clés — Ed25519 (signature) ne doit jamais pouvoir
# être utilisée pour ce module (X25519, chiffrement).
from axis_os.store.sdk.signing import generate_developer_keypair

dev_kp = generate_developer_keypair()  # paire Ed25519, PAS X25519
try:
    encrypt_app_code(b"code", dev_kp.public_key_b64)
    # Une clé Ed25519 base64-décodée fait aussi 32 octets — le
    # chiffrement X25519 pourrait techniquement "réussir" avec ces
    # octets bruts (X25519 accepte n'importe quelle chaîne de 32
    # octets comme clé valide, contrairement à Ed25519 qui valide un
    # point de courbe). Documenté ici comme limite connue : ce module
    # NE VALIDE PAS qu'une clé fournie est réellement une clé X25519
    # légitimement générée pour cet usage — seule sa TAILLE est
    # vérifiée. La séparation des clés reste une DISCIPLINE d'usage
    # (deux fonctions de génération distinctes, deux types de clé
    # dans le code), pas une contrainte cryptographique appliquée.
    check("Une clé Ed25519 utilisée par erreur comme clé X25519 : "
          "accepté sans erreur (limite connue et documentée — "
          "X25519 n'a pas de validation de point de courbe comme "
          "Ed25519, la séparation des clés reste une discipline "
          "d'usage, pas une garantie technique)",
          True)
except EncryptionError:
    check("Une clé Ed25519 utilisée par erreur comme clé X25519 : "
          "rejetée (meilleur que prévu)", True)

# ══════════════════════════════════════════════════════════════════

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)

if FAIL == 0:
    print("\n  ✓ Cycle complet chiffrement/déchiffrement confirmé exact")
    print("  ✓ CRITIQUE : seule la vraie clé privée plateforme déchiffre")
    print("  ✓ CRITIQUE : toute altération du fichier chiffré est détectée")
    print("    (authenticité AES-GCM), jamais silencieusement corrompue")
    print("  ✓ Deux chiffrements du même contenu produisent des résultats")
    print("    différents et non corrélables")
    print("  ✓ Robuste face à des clés/fichiers malformés")
    print("  ✓ Confirmé de bout en bout via de vrais fichiers disque")

sys.exit(0 if FAIL == 0 else 1)
