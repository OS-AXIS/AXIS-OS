"""
AXIS-OS — Tests Structured Log : chaînage de hash
====================================================
Valide l'ajout de hash-chaining à StructuredLogger (voir docstring de
axis_os/hal/core/structured_log.py pour la motivation : obligations de
transparence EU AI Act dès août 2026, logique "flight recorder"
appliquée aux logs de sécurité robotique).

Couvre : chaîne valide sur écritures normales, détection de
modification de contenu, détection de suppression de ligne, continuité
de chaîne entre deux instances (simule un redémarrage de process),
rétrocompatibilité avec des logs pré-chaînage, et thread-safety sous
écriture concurrente.
"""

import sys
import json
import tempfile
import threading
from pathlib import Path
sys.path.insert(0, ".")

from axis_os.hal.core.structured_log import (
    StructuredLogger, LogCategory, GENESIS_HASH, ChainVerificationResult,
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

section("1 — Chaîne valide sur écritures normales")

with tempfile.TemporaryDirectory() as tmp:
    logger = StructuredLogger(Path(tmp))
    for i in range(5):
        ok = logger.log(LogCategory.ESTOP, {"reason": f"TEST_{i}"})
        check(f"log() #{i} réussit", ok)

    result = logger.verify_chain(LogCategory.ESTOP)
    check("Chaîne de 5 entrées valide", result.valid)
    check("entries_checked == 5", result.entries_checked == 5)
    check("break_at_line == None sur chaîne valide", result.break_at_line is None)

    result_empty = logger.verify_chain(LogCategory.SANDBOX_EVENT)
    check("Catégorie jamais écrite -> valid=True, 0 entrées (rien à vérifier)",
          result_empty.valid and result_empty.entries_checked == 0)

# ══════════════════════════════════════════════════════════════════

section("2 — Détection : contenu d'une entrée modifié après écriture")

with tempfile.TemporaryDirectory() as tmp:
    logger = StructuredLogger(Path(tmp))
    logger.log(LogCategory.ESTOP, {"reason": "A"})
    logger.log(LogCategory.ESTOP, {"reason": "B"})
    logger.log(LogCategory.ESTOP, {"reason": "C"})

    path = Path(tmp) / "estop.jsonl"
    lines = path.read_text().splitlines()
    entry = json.loads(lines[1])
    entry["data"]["reason"] = "FALSIFIÉ"
    lines[1] = json.dumps(entry)
    path.write_text("\n".join(lines) + "\n")

    result = logger.verify_chain(LogCategory.ESTOP)
    check("Falsification de contenu détectée (valid=False)", not result.valid)
    check("Rupture localisée à la ligne 2", result.break_at_line == 2)
    check("La 1ère entrée reste confirmée valide", result.entries_checked == 1)
    check("Raison mentionne le hash", "hash" in result.reason.lower())

# ══════════════════════════════════════════════════════════════════

section("3 — Détection : suppression d'une ligne")

with tempfile.TemporaryDirectory() as tmp:
    logger = StructuredLogger(Path(tmp))
    logger.log(LogCategory.ESTOP, {"reason": "A"})
    logger.log(LogCategory.ESTOP, {"reason": "B"})
    logger.log(LogCategory.ESTOP, {"reason": "C"})

    path = Path(tmp) / "estop.jsonl"
    lines = path.read_text().splitlines()
    del lines[1]  # supprime l'entrée "B"
    path.write_text("\n".join(lines) + "\n")

    result = logger.verify_chain(LogCategory.ESTOP)
    check("Suppression de ligne détectée (valid=False)", not result.valid)
    check("Rupture localisée à la ligne 2 (nouvel index après suppression)",
          result.break_at_line == 2)
    check("Raison mentionne prev_hash incohérent", "prev_hash" in result.reason)

# ══════════════════════════════════════════════════════════════════

section("4 — Continuité de chaîne entre deux instances (redémarrage)")

with tempfile.TemporaryDirectory() as tmp:
    logger1 = StructuredLogger(Path(tmp))
    logger1.log(LogCategory.WATCHDOG_ALERT, {"reason": "A"})
    logger1.log(LogCategory.WATCHDOG_ALERT, {"reason": "B"})

    # Nouvelle instance : simule un process qui redémarre et reprend
    # l'écriture sur un fichier de log déjà existant.
    logger2 = StructuredLogger(Path(tmp))
    logger2.log(LogCategory.WATCHDOG_ALERT, {"reason": "C"})

    result = logger2.verify_chain(LogCategory.WATCHDOG_ALERT)
    check("Chaîne continue sur 3 entrées malgré 2 instances différentes",
          result.valid and result.entries_checked == 3)

# ══════════════════════════════════════════════════════════════════

section("5 — Rétrocompatibilité : logs pré-chaînage (sans hash/prev_hash)")

with tempfile.TemporaryDirectory() as tmp:
    path = Path(tmp) / "estop.jsonl"
    old_entry = {
        "timestamp_ns": 123, "timestamp_iso": "2026-01-01T00:00:00",
        "category": "estop", "robot_id": "g1", "data": {"reason": "OLD"},
    }
    path.write_text(json.dumps(old_entry) + "\n")

    logger = StructuredLogger(Path(tmp))
    ok = logger.log(LogCategory.ESTOP, {"reason": "NEW"})
    check("Écriture réussit sur un fichier legacy sans hash (pas de crash)", ok)

    recent = logger.read_recent(LogCategory.ESTOP, n=10)
    check("read_recent lit les 2 entrées (legacy + nouvelle)", len(recent) == 2)

    result = logger.verify_chain(LogCategory.ESTOP)
    check("verify_chain signale honnêtement l'absence de hash (pas un crash)",
          not result.valid and result.break_at_line == 1)

# ══════════════════════════════════════════════════════════════════

section("6 — Thread-safety sous écriture concurrente")

with tempfile.TemporaryDirectory() as tmp:
    logger = StructuredLogger(Path(tmp))

    def writer(thread_id: int) -> None:
        for i in range(20):
            logger.log(LogCategory.SANDBOX_EVENT, {"thread": thread_id, "i": i})

    threads = [threading.Thread(target=writer, args=(t,)) for t in range(5)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    result = logger.verify_chain(LogCategory.SANDBOX_EVENT)
    check("100 écritures concurrentes (5 threads × 20) -> chaîne intacte",
          result.valid and result.entries_checked == 100)

# ══════════════════════════════════════════════════════════════════

section("7 — GENESIS_HASH : première entrée de chaque catégorie")

with tempfile.TemporaryDirectory() as tmp:
    logger = StructuredLogger(Path(tmp))
    logger.log(LogCategory.CONVENTION_ERROR, {"x": 1})
    path = Path(tmp) / "convention_error.jsonl"
    first_entry = json.loads(path.read_text().splitlines()[0])
    check("prev_hash de la toute première entrée == GENESIS_HASH",
          first_entry["prev_hash"] == GENESIS_HASH)

# ══════════════════════════════════════════════════════════════════

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)

if FAIL == 0:
    print("\n  ✓ Chaîne de hash valide sur écritures normales")
    print("  ✓ Modification de contenu détectée et localisée précisément")
    print("  ✓ Suppression de ligne détectée (rupture de prev_hash)")
    print("  ✓ Continuité garantie entre redémarrages de process")
    print("  ✓ Rétrocompatible avec les logs écrits avant ce changement")
    print("  ✓ Thread-safe sous écriture concurrente")

sys.exit(0 if FAIL == 0 else 1)
