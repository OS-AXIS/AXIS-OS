"""
Connecteur GR00T — la réponse du serveur est une donnée NON FIABLE.

Audit du 2026-10-10 (I3). Vérifié ici, contre un faux serveur qui renvoie
exactement les octets voulus :
  - endpoint hors de la machine refusé par défaut (ZeroMQ circule en clair) ;
  - message plus gros que taille_max_octets refusé (ZMQ_MAXMSGSIZE) ;
  - archive npz refusée là où un tableau .npy est attendu ;
  - dtypes non réels refusés (complexes, structures, dates) ; les actions
    ne peuvent être que des nombres réels ;
  - le format de NVIDIA (__ndarray_class__ + .npy) et du texte dans `info`
    restent acceptés.
"""
import io
import sys
import threading
import time
from pathlib import Path

import msgpack
import msgpack_numpy as mnp
import numpy as np
import zmq

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.store.sdk.ai.gr00t_connector import GR00TConnector, endpoint_local  # noqa: E402

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


def leve(exc_type, fn):
    try:
        fn()
    except exc_type:
        return True
    except Exception:                                   # noqa: BLE001
        return False
    return False


ARMS = [f"left_j{i}" for i in range(7)] + [f"right_j{i}" for i in range(7)]
ETAT = {j: 0.0 for j in ARMS}
IMG = np.zeros((24, 32, 3), dtype=np.uint8)


def npy(a):
    """Format de NVIDIA : {"__ndarray_class__": True, "as_npy": <octets .npy>}."""
    b = io.BytesIO()
    np.save(b, a, allow_pickle=False)
    return {"__ndarray_class__": True, "as_npy": b.getvalue()}


def action_ok(**remplace):
    a = {"bras_gauche": np.zeros((1, 16, 7), np.float32),
         "bras_droit": np.zeros((1, 16, 7), np.float32),
         "base": np.zeros((1, 16, 3), np.float32)}
    a.update(remplace)
    return a


class Serveur(threading.Thread):
    """REP : ping -> ok ; get_action -> self.reponse() (octets bruts)."""

    def __init__(self):
        super().__init__(daemon=True)
        self.ctx = zmq.Context()
        self.sock = self.ctx.socket(zmq.REP)
        self.port = self.sock.bind_to_random_port("tcp://127.0.0.1")
        self.sock.setsockopt(zmq.RCVTIMEO, 100)
        self.run_ = True
        self.reponse = lambda: msgpack.packb([action_ok(), {}], default=mnp.encode)

    def run(self):
        while self.run_:
            try:
                req = msgpack.unpackb(self.sock.recv(), raw=False)
            except zmq.error.Again:
                continue
            if req.get("endpoint") == "ping":
                self.sock.send(msgpack.packb({"status": "ok"}))
            else:
                self.sock.send(self.reponse())

    def stop(self):
        self.run_ = False
        self.join(timeout=2)
        self.sock.close(linger=0)
        self.ctx.term()


section("1 — Endpoint : boucle locale par défaut")

for ep in ("tcp://127.0.0.1:5555", "tcp://localhost:5555", "tcp://[::1]:5555",
           "ipc:///tmp/gr00t.sock"):
    check(f"{ep} : local, accepté", endpoint_local(ep)
          and not leve(ValueError, lambda: GR00TConnector(ep, arm_joints=ARMS)))
for ep in ("tcp://10.0.0.2:5555", "tcp://0.0.0.0:5555", "tcp://gpu.example.com:5555",
           "tcp://127.0.0.1.example.com:5555"):
    check(f"{ep} : hors de la machine, REFUSÉ par défaut",
          leve(ValueError, lambda: GR00TConnector(ep, arm_joints=ARMS)))
check("… accepté seulement sur choix explicite (autoriser_distant=True)",
      not leve(ValueError, lambda: GR00TConnector("tcp://10.0.0.2:5555", arm_joints=ARMS,
                                                  autoriser_distant=True)))

srv = Serveur()
srv.start()


def connecteur(**kw):
    kw.setdefault("request_timeout_ms", 1000)
    c = GR00TConnector(f"tcp://127.0.0.1:{srv.port}", arm_joints=ARMS, **kw)
    assert c.connect()
    return c


def predire(c):
    return c.predict_action(IMG, "x", ETAT)


section("2 — Référence : réponses valides acceptées")

c = connecteur()
check("Format msgpack_numpy (celui du faux serveur historique) : accepté",
      len(predire(c).target_angles) == 14)
srv.reponse = lambda: msgpack.packb(
    [{k: npy(v) for k, v in action_ok().items()}, {"note": npy(np.array(["ok"]))}])
check("Format de NVIDIA (__ndarray_class__ + .npy), texte dans info : accepté",
      len(predire(c).target_angles) == 14)

section("3 — Taille maximale de message (ZMQ_MAXMSGSIZE)")

gros = np.zeros((1, 16, 7), np.float32)
srv.reponse = lambda: msgpack.packb([action_ok(), {"bourrage": b"x" * 2_000_000}],
                                    default=mnp.encode)
cp = connecteur(taille_max_octets=1_000_000, request_timeout_ms=500)
check("Réponse de 2 Mo avec une borne de 1 Mo : refusée (RuntimeError)",
      leve(RuntimeError, lambda: predire(cp)))
cg = connecteur()
check("La même réponse sous la borne par défaut (8 Mo) : acceptée",
      len(predire(cg).target_angles) == 14)
check("Borne nulle ou négative refusée à la construction",
      leve(ValueError, lambda: GR00TConnector(arm_joints=ARMS, taille_max_octets=0)))

section("4 — Archive npz refusée là où un tableau est attendu")


def npz():
    b = io.BytesIO()
    np.savez(b, a=np.zeros((1, 16, 7), np.float32))
    return {"__ndarray_class__": True, "as_npy": b.getvalue()}


srv.reponse = lambda: msgpack.packb([{**{k: npy(v) for k, v in action_ok().items()},
                                      "bras_gauche": npz()}, {}])
check("npz dans 'bras_gauche' : refusé", leve((RuntimeError, ValueError), lambda: predire(c)))
# Le refus ci-dessus pourrait venir du contrôle de dtype des actions : on
# isole celui de _dec avec un npz dans `info`, actions valides (revue).
srv.reponse = lambda: msgpack.packb([{k: npy(v) for k, v in action_ok().items()},
                                     {"cache": npz()}])
check("npz dans info, actions valides : refusé par le décodeur lui-même",
      leve(ValueError, lambda: predire(c)))
srv.reponse = lambda: msgpack.packb([action_ok(), {"x": {"__ndarray_class__": True}}],
                                    default=mnp.encode)
check("__ndarray_class__ sans 'as_npy' : refusé", leve((RuntimeError, ValueError), lambda: predire(c)))

section("5 — Dtypes non réels refusés")

cas = {
    "complexes (msgpack_numpy)": lambda: msgpack.packb(
        [action_ok(bras_gauche=np.zeros((1, 16, 7), np.complex64)), {}], default=mnp.encode),
    "complexes (format NVIDIA)": lambda: msgpack.packb(
        [{**{k: npy(v) for k, v in action_ok().items()},
          "bras_gauche": npy(np.zeros((1, 16, 7), np.complex128))}, {}]),
    "dates (format NVIDIA)": lambda: msgpack.packb(
        [action_ok(), {"t": npy(np.array(["2026-10-10"], dtype="datetime64[D]"))}],
        default=mnp.encode),
    "structure (msgpack_numpy)": lambda: msgpack.packb(
        [action_ok(), {"s": np.zeros(2, dtype=[("a", "f4"), ("b", "i4")])}], default=mnp.encode),
    "scalaire complexe dans info": lambda: msgpack.packb([action_ok(), {"z": 1 + 2j}],
                                                          default=mnp.encode),
    "action en booléens": lambda: msgpack.packb(
        [action_ok(base=np.zeros((1, 16, 3), bool)), {}], default=mnp.encode),
    "action en texte": lambda: msgpack.packb(
        [action_ok(base=[[["a", "b", "c"]] * 16]), {}], default=mnp.encode),
    "action finie en float64 mais infinie en float32 (1e39)": lambda: msgpack.packb(
        [action_ok(base=np.full((1, 16, 3), 1e39)), {}], default=mnp.encode),
}
for nom, rep in cas.items():
    srv.reponse = rep
    check(f"{nom} : refusé", leve((RuntimeError, ValueError, TypeError), lambda: predire(c)))

srv.reponse = lambda: msgpack.packb([action_ok(), {}], default=mnp.encode)
check("Après les refus, le connecteur refonctionne", len(predire(c).target_angles) == 14)
srv.reponse = lambda: msgpack.packb(["pas un dict", {}])
check("Action qui n'est pas un dictionnaire : refusée", leve(RuntimeError, lambda: predire(c)))

srv.stop()

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
