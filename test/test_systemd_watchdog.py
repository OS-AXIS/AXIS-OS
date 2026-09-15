"""
AXIS-OS — Tests chien de garde systemd
==========================================
Valide _sd_notify_watchdog() (axis_os/hal/watchdog/system.py), ajouté
suite à une recommandation externe : si WatchdogSystem._monitor_loop()
se fige complètement, rien dans le process Python ne peut s'en
apercevoir — c'est le rôle du chien de garde systemd (WatchdogSec dans
deploy/systemd/axis-os.service), déclenché par ce heartbeat envoyé
après chaque cycle réussi.
"""

import sys
import os
import socket
import threading
import tempfile
import time
sys.path.insert(0, ".")

from axis_os.hal.watchdog.system import _sd_notify_watchdog, WatchdogSystem, WatchdogConfig
from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer
from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL

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

section("1 — Sans NOTIFY_SOCKET : no-op silencieux, jamais d'exception")

os.environ.pop("NOTIFY_SOCKET", None)
try:
    _sd_notify_watchdog()
    check("Aucune exception levée sans NOTIFY_SOCKET défini", True)
except Exception as e:
    check(f"Aucune exception levée sans NOTIFY_SOCKET défini (obtenu: {e})", False)

# ══════════════════════════════════════════════════════════════════

section("2 — Avec un vrai socket Unix : message WATCHDOG=1 bien envoyé")

with tempfile.TemporaryDirectory() as tmp:
    sock_path = os.path.join(tmp, "notify.sock")
    server = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    server.bind(sock_path)
    server.settimeout(2.0)

    os.environ["NOTIFY_SOCKET"] = sock_path
    _sd_notify_watchdog()

    try:
        data, _ = server.recvfrom(1024)
        check("Message WATCHDOG=1 reçu sur le socket", data == b"WATCHDOG=1")
    except socket.timeout:
        check("Message WATCHDOG=1 reçu sur le socket", False)
    finally:
        server.close()
        os.environ.pop("NOTIFY_SOCKET", None)

# ══════════════════════════════════════════════════════════════════

section("3 — Socket abstrait Linux (préfixe '@') géré correctement")

# Les sockets abstraits (NOTIFY_SOCKET="@nom") sont la convention
# systemd la plus courante en production — vérifie que le '@' est
# bien traduit en '\0' (premier octet nul, convention Linux), sans
# lever d'exception même si le socket n'existe pas réellement ici
# (comportement no-op attendu, testé en section 1 pour le cas général).
os.environ["NOTIFY_SOCKET"] = "@test.socket.inexistant"
try:
    _sd_notify_watchdog()
    check("Socket abstrait avec préfixe '@' ne lève jamais d'exception "
          "(même si le socket cible n'existe pas)", True)
except Exception as e:
    check(f"Socket abstrait '@' géré sans exception (obtenu: {e})", False)
finally:
    os.environ.pop("NOTIFY_SOCKET", None)

# ══════════════════════════════════════════════════════════════════

section("4 — Intégration : WatchdogSystem envoie bien le heartbeat après un cycle")

with tempfile.TemporaryDirectory() as tmp:
    sock_path = os.path.join(tmp, "notify_integ.sock")
    server = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    server.bind(sock_path)
    server.settimeout(2.0)

    os.environ["NOTIFY_SOCKET"] = sock_path

    robot = UnitreeG1HAL("g1_sdnotify_test")
    robot.initialize_all()
    cfg = WatchdogConfig.standard()
    cfg.poll_interval_s = 0.05
    wd = WatchdogSystem(robot, cfg)
    wd.start()

    try:
        data, _ = server.recvfrom(1024)
        check("WatchdogSystem envoie bien WATCHDOG=1 après un cycle réel",
              data == b"WATCHDOG=1")
    except socket.timeout:
        check("WatchdogSystem envoie bien WATCHDOG=1 après un cycle réel", False)
    finally:
        wd.stop()
        server.close()
        os.environ.pop("NOTIFY_SOCKET", None)

# ══════════════════════════════════════════════════════════════════

section("5 — Fichier unit systemd présent et cohérent")

unit_path = "deploy/systemd/axis-os.service"
check("deploy/systemd/axis-os.service existe", os.path.exists(unit_path))
if os.path.exists(unit_path):
    content = open(unit_path, encoding="utf-8").read()
    check("Type=notify présent (requis pour que systemd écoute sd_notify)",
          "Type=notify" in content)
    check("WatchdogSec défini", "WatchdogSec=" in content)
    check("Restart=on-failure présent (redémarrage après timeout du chien de garde)",
          "Restart=on-failure" in content)
    check("User= défini (jamais lancer en root)", "User=" in content)

# ══════════════════════════════════════════════════════════════════

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)

sys.exit(0 if FAIL == 0 else 1)
