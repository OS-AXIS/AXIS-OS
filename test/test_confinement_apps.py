"""
Confinement des apps : on vérifie que les portes du bac à sable sont
fermées, puis que chaque vérification détecte une porte rouverte exprès.

  1. Le canal JSON refuse ce qui n'est pas un message valide (octets non
     JSON, NaN, infini, champ inconnu, trame de 2 Mio) et reste utilisable.
  2. Le processus d'une app n'hérite d'aucun descripteur de l'OS.
  3. Il ne voit pas l'environnement de l'OS.
  4. Une mission complète ne fait jamais désérialiser de pickle à l'OS.
  5. Chaque vérification ci-dessus échoue si la protection est retirée.
"""
import contextlib
import io
import json
import os
import pickle
import socket
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer  # noqa: E402
from axis_os.hal.core.normalized_robot import NormalizedRobotHAL  # noqa: E402
from axis_os.hal.core.types import ActuatorCommand, ControlMode  # noqa: E402
from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL  # noqa: E402
from axis_os.hal.watchdog.system import WatchdogSystem  # noqa: E402
from axis_os.store.runtime import canal_ipc, processus_app  # noqa: E402
from axis_os.store.runtime.canal_ipc import (  # noqa: E402
    Canal, CanalCorrompu, MessageRefuse, TAILLE_MAX_TRAME, _ENTETE,
)
from axis_os.store.runtime.hal_bridge import HALBridgeServer  # noqa: E402
from axis_os.store.runtime.play_store import PlayStoreRuntime, accepter_tout_SIMULATION  # noqa: E402
from axis_os.store.runtime.profil import Profil  # noqa: E402
from axis_os.store.runtime.sandbox import SandboxRunner  # noqa: E402
from axis_os.store.sdk.app_base import MissionResult, RobotApp, RobotContext  # noqa: E402
from axis_os.store.sdk.manifest import (  # noqa: E402
    AppCategory, AppManifest, Permission, ResourceQuota,
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
    with contextlib.redirect_stdout(io.StringIO()):
        return f(*a, **kw)


def robot(nom):
    with contextlib.redirect_stdout(io.StringIO()):
        r = NormalizedRobotHAL(UnitreeG1HAL(nom), build_g1_normalizer())
        r.initialize_all()
    return r


def trame(corps: bytes) -> bytes:
    return _ENTETE.pack(len(corps)) + corps


def refuse(canal, pair, corps):
    """Envoie une trame brute ; True si elle est refusée et que le canal
    accepte ensuite un message valide."""
    pair.sendall(trame(corps))
    try:
        canal.recevoir(timeout=2.0)
        return False
    except MessageRefuse:
        pass
    pair.sendall(trame(b'{"ok":1}'))
    return canal.recevoir(timeout=2.0) == {"ok": 1}


# Fonctions exécutées dans le processus de l'app par SandboxRunner.run().
def descripteurs_ouverts():
    cibles = []
    for nom in os.listdir("/proc/self/fd"):
        try:
            cibles.append(os.readlink(f"/proc/self/fd/{nom}"))
        except OSError:
            pass                # le descripteur du listage lui-même, déjà fermé
    return cibles


def variables_visibles():
    return sorted(os.environ)


def descripteurs_suspects(cibles):
    """Ce qui n'est ni un tube/terminal (stdio), ni le pont, ni /dev/null."""
    sockets = [c for c in cibles if c.startswith("socket:")]
    autres = [c for c in cibles if not c.startswith(("socket:", "pipe:", "/dev/"))
              and "/proc/" not in c]
    return sockets, autres


class AppBras(RobotApp):
    def manifest(self):
        return AppManifest(app_id="com.test.confinement", name="Confinement", version="1.0.0",
                           author="T", author_email="t@t.com", description="d",
                           category=AppCategory.UTILITIES,
                           permissions=[Permission.ACTUATORS_WRITE_ARMS])

    def on_start(self, ctx):
        return True

    def on_stop(self, ctx):
        pass

    def run_mission(self, ctx, params):
        statuts = ctx.move_arms({"left_elbow_joint": ActuatorCommand(
            mode=ControlMode.POSITION, position=0.05)})
        return MissionResult(success=all(s.name == "OK" for s in statuts.values()),
                             mission_id="", app_id="", duration_s=0.0, items_processed=1)


@contextlib.contextmanager
def pickle_interdit():
    """Note chaque désérialisation pickle tentée dans le processus de l'OS."""
    appels = []
    origine = (pickle.loads, pickle.load, pickle.Unpickler)

    def note(*a, **kw):
        appels.append("appel")
        raise AssertionError("pickle utilisé par l'OS")

    pickle.loads = pickle.load = pickle.Unpickler = note
    try:
        yield appels
    finally:
        pickle.loads, pickle.load, pickle.Unpickler = origine


def mission_complete():
    r = robot("g1_confinement")
    rt = silencieux(PlayStoreRuntime, r, WatchdogSystem(r), profil=Profil.SIMULATION,
                    confirmer_permissions=accepter_tout_SIMULATION)
    ok = silencieux(rt.install, AppBras()) and silencieux(rt.start_app, "com.test.confinement")
    res = silencieux(rt.run_mission, "com.test.confinement", {})
    return ok and res is not None and res.success


def lancer(fn):
    return silencieux(SandboxRunner("com.test.confinement",
                                    ResourceQuota(max_runtime_s=20.0)).run, fn)


if __name__ == "__main__":
    section("1. Le canal JSON refuse ce qui n'est pas un message valide")
    a, b = socket.socketpair()
    canal = Canal(a)
    check("Octets binaires non JSON : refusés, canal toujours utilisable",
          refuse(canal, b, b"\x80\x04\x95" + bytes(range(256))))
    check("UTF-8 invalide : refusé", refuse(canal, b, b'{"a":"\xff"}'))
    check("Constante NaN : refusée", refuse(canal, b, b'{"x":NaN}'))
    check("Nombre infini (1e999) : refusé", refuse(canal, b, b'{"x":1e999}'))
    check("Clé en double : refusée", refuse(canal, b, b'{"x":1,"x":2}'))
    check("Liste au lieu d'un objet : refusée", refuse(canal, b, b"[1,2]"))
    check("Imbrication excessive : refusée",
          refuse(canal, b, b'{"x":' + b"[" * 40 + b"]" * 40 + b"}"))
    try:
        canal.envoyer({"x": "a" * (2 * 2**20)})
        check("Envoi d'un message de 2 Mio : refusé", False, "aucune exception")
    except ValueError:
        check("Envoi d'un message de 2 Mio : refusé", True)
    b.sendall(_ENTETE.pack(2 * 2**20))
    try:
        canal.recevoir(timeout=2.0)
        check("Trame annoncée de 2 Mio : refusée sans être lue", False, "acceptée")
    except CanalCorrompu:
        check("Trame annoncée de 2 Mio : refusée sans être lue", True)
    b.close()
    check(f"Taille maximale d'une trame : 1 Mio ({TAILLE_MAX_TRAME} octets)",
          TAILLE_MAX_TRAME == 2**20)

    r = robot("g1_confinement_schema")
    ctx = RobotContext.for_testing(r, WatchdogSystem(r), AppBras().manifest())
    serveur = HALBridgeServer(ctx)
    rep = silencieux(serveur.traiter, {"action": "now", "payload": {"intrus": 1},
                                       "request_id": 7})
    check("Champ inconnu dans une commande du pont : MALFORMED_COMMAND",
          rep is not None and rep["success"] is False
          and rep["error"].startswith("MALFORMED_COMMAND"), str(rep))
    rep = silencieux(serveur.traiter, {"action": "now", "payload": {}, "request_id": 8})
    check("… et le serveur répond encore à la commande valide suivante",
          rep is not None and rep["success"] is True, str(rep))

    section("2. Le processus de l'app n'hérite d'aucun descripteur de l'OS")
    fuite = tempfile.NamedTemporaryFile(prefix="axis_fuite_")
    os.set_inheritable(fuite.fileno(), True)
    res = lancer(descripteurs_ouverts)
    sockets, autres = descripteurs_suspects(res.result or [])
    check("L'app s'exécute", res.success, str(res.error))
    check("Un seul socket ouvert : le pont", len(sockets) == 1, str(sockets))
    check("Aucun fichier de l'OS ouvert (même rendu héritable)", autres == [], str(autres))

    section("3. L'app ne voit pas l'environnement de l'OS")
    os.environ["AXIS_TEST_SECRET"] = "valeur-de-test"
    res = lancer(variables_visibles)
    check("L'app s'exécute", res.success, str(res.error))
    check("AXIS_TEST_SECRET absente", "AXIS_TEST_SECRET" not in (res.result or []))
    check("Seules PATH et la langue sont transmises",
          set(res.result or []) <= {"PATH", "LANG", "LC_ALL", "LC_CTYPE", "PWD"},
          str(res.result))

    section("4. Une mission complète sans aucun pickle côté OS")
    with pickle_interdit() as appels:
        reussie = mission_complete()
    check("La mission réussit et bouge le bras par le pont", reussie)
    check("Aucune désérialisation pickle dans le processus de l'OS", appels == [], str(appels))

    section("5. Chaque vérification détecte une protection retirée")
    # Canal : décodage qui laisserait passer NaN et l'infini.
    garde = (canal_ipc._constante_interdite, canal_ipc._flottant_fini)
    canal_ipc._constante_interdite, canal_ipc._flottant_fini = float, float
    try:
        a, b = socket.socketpair()
        check("Sans la garde, NaN serait accepté (le test 1 le verrait)",
              not refuse(Canal(a), b, b'{"x":NaN}'))
        b.close()
    finally:
        canal_ipc._constante_interdite, canal_ipc._flottant_fini = garde

    # Descripteurs : lancement qui transmettrait aussi un fichier de l'OS.
    popen = processus_app.subprocess.Popen

    def popen_fuyant(*a, pass_fds=(), **kw):
        return popen(*a, pass_fds=tuple(pass_fds) + (fuite.fileno(),), **kw)

    processus_app.subprocess.Popen = popen_fuyant
    try:
        res = lancer(descripteurs_ouverts)
        _, autres = descripteurs_suspects(res.result or [])
        check("Sans close_fds, le fichier hérité serait vu (le test 2 le verrait)",
              res.success and any("axis_fuite_" in c for c in autres), str(res.result))
    finally:
        processus_app.subprocess.Popen = popen
    fuite.close()

    # Environnement : lancement avec l'environnement complet de l'OS.
    env = processus_app.environnement_minimal
    processus_app.environnement_minimal = lambda: dict(os.environ)
    try:
        res = lancer(variables_visibles)
        check("Avec l'environnement complet, le secret serait vu (le test 3 le verrait)",
              "AXIS_TEST_SECRET" in (res.result or []), str(res.error))
    finally:
        processus_app.environnement_minimal = env
        del os.environ["AXIS_TEST_SECRET"]

    # Pickle : une désérialisation côté OS est bien notée.
    with pickle_interdit() as appels:
        with contextlib.suppress(AssertionError):
            pickle.loads(json.dumps(1).encode())
    check("Une désérialisation pickle côté OS serait détectée (le test 4 le verrait)",
          appels == ["appel"])

    print(f"\n{'Tous les tests passent.' if FAIL == 0 else f'{FAIL} échec(s).'}")
    sys.exit(0 if FAIL == 0 else 1)
