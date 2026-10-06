"""
Connecteur GR00T : protocole ZeroMQ + msgpack-numpy de NVIDIA, contre un FAUX
serveur qui valide l'observation comme le fait Gr00tPolicy.check_observation
(clés, dtypes, formes (B, T, ...)) — sans GPU ni modèle.

LIMITE : ce n'est pas un vrai serveur GR00T. Le vrai essai se fait sur la
machine GPU (run_gr00t_server.py).
"""
import sys
import threading
import time
from pathlib import Path

import msgpack
import msgpack_numpy as mnp
import numpy as np
import zmq

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.store.sdk.ai.gr00t_connector import GR00TConnector, GR00TAction, CLE_LANGUE  # noqa: E402
from axis_os.hal.core.types import ControlMode  # noqa: E402

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
ETAT = {j: 0.1 * k for k, j in enumerate(ARMS)}
IMG = np.full((120, 160, 3), 77, dtype=np.uint8)


class FauxServeur(threading.Thread):
    """REP, mêmes endpoints que PolicyServer : ping / get_action / reset."""

    def __init__(self, token=None):
        super().__init__(daemon=True)
        self.ctx = zmq.Context()
        self.sock = self.ctx.socket(zmq.REP)
        self.port = self.sock.bind_to_random_port("tcp://127.0.0.1")
        self.sock.setsockopt(zmq.RCVTIMEO, 100)
        self.token, self.run_ = token, True
        self.mode, self.vues, self.resets, self.delai = "ok", [], 0, 0.0

    def _action(self, obs):
        # Valide comme check_observation : sinon le vrai serveur lèverait.
        v = obs["video"][CLE_VIDEO_]
        assert v.dtype == np.uint8 and v.ndim == 5 and v.shape[:2] == (1, 1) and v.shape[-1] == 3
        for k in ("bras_gauche", "bras_droit"):
            s = obs["state"][k]
            assert s.dtype == np.float32 and s.shape == (1, 1, 7), s.shape
        lang = obs["language"][CLE_LANGUE]
        assert isinstance(lang, list) and len(lang) == 1 and len(lang[0]) == 1 and isinstance(lang[0][0], str)
        self.vues.append(obs)
        t = np.arange(16, dtype=np.float32)[None, :, None]
        a = {"bras_gauche": 1.0 + t + np.zeros((1, 16, 7), np.float32),
             "bras_droit": 100.0 + t + np.zeros((1, 16, 7), np.float32),
             "base": np.tile(np.array([0.2, 0.0, -0.1], np.float32), (1, 16, 1))}
        if self.mode == "nan":
            a["bras_gauche"][0, 0, 3] = np.nan
        if self.mode == "forme":
            a["base"] = a["base"][:, :, :2]
        if self.mode == "cle":
            del a["base"]
        return [a, {}]

    def run(self):
        while self.run_:
            try:
                msg = self.sock.recv()
            except zmq.error.Again:
                continue
            req = msgpack.unpackb(msg, object_hook=mnp.decode, raw=False)
            if self.token and req.get("api_token") != self.token:
                rep = {"error": "Unauthorized: Invalid API token"}
            else:
                ep = req.get("endpoint", "get_action")
                try:
                    if ep == "ping":
                        rep = {"status": "ok", "message": "Server is running"}
                    elif ep == "reset":
                        self.resets += 1
                        rep = {}
                    elif ep == "get_action":
                        if self.mode == "erreur":
                            raise ValueError("modèle indisponible")
                        if self.mode == "lent":
                            time.sleep(self.delai)
                        rep = self._action(req["data"]["observation"])
                    else:
                        raise ValueError(f"Unknown endpoint: {ep}")
                except Exception as e:                  # noqa: BLE001
                    rep = {"error": str(e)}
            if self.mode == "pickle" and req.get("endpoint") == "get_action":
                out = msgpack.packb({b"nd": True, b"kind": b"O", b"type": "|O", b"shape": [1],
                                     b"data": b"x"}, use_bin_type=True)
            else:
                out = msgpack.packb(rep, default=mnp.encode)
            self.sock.send(out)

    def stop(self):
        self.run_ = False
        self.join(timeout=2)
        self.sock.close(linger=0)
        self.ctx.term()


CLE_VIDEO_ = "ego_view"


def connecteur(srv, **kw):
    kw.setdefault("request_timeout_ms", 1500)
    return GR00TConnector(f"tcp://127.0.0.1:{srv.port}", arm_joints=ARMS, **kw)


section("Construction")
check("arm_joints de mauvaise taille refusé", leve(ValueError, lambda: GR00TConnector(arm_joints=ARMS[:10])))
check("arm_joints dans le mauvais ordre refusé (droit avant gauche)",
      leve(ValueError, lambda: GR00TConnector(arm_joints=ARMS[7:] + ARMS[:7])))
check("step hors horizon refusé", leve(ValueError, lambda: GR00TConnector(arm_joints=ARMS, step=16)))
check("predict_action avant connect() lève RuntimeError",
      leve(RuntimeError, lambda: GR00TConnector(arm_joints=ARMS).predict_action(IMG, "x", ETAT)))

section("Connexion et prédiction")
srv = FauxServeur()
srv.start()
c = connecteur(srv, image_shape=(120, 160))
check("connect() : ping réel du serveur", c.connect() and c.is_connected())
t0 = time.monotonic_ns()
a = c.predict_action(IMG, "pick up the red box", ETAT)
check("le serveur a reçu une observation valide (formes/dtypes de check_observation)", len(srv.vues) == 1)
check("consigne transmise telle quelle",
      srv.vues[0]["language"][CLE_LANGUE] == [["pick up the red box"]])
check("état envoyé = positions des 14 joints, gauche puis droit",
      np.allclose(srv.vues[0]["state"]["bras_gauche"][0, 0], [ETAT[j] for j in ARMS[:7]])
      and np.allclose(srv.vues[0]["state"]["bras_droit"][0, 0], [ETAT[j] for j in ARMS[7:]]))
check("angles : pas 0 du chunk, mappés sur les vrais joint_id",
      isinstance(a, GR00TAction) and a.target_angles["left_j0"] == 1.0 and a.target_angles["right_j6"] == 100.0
      and len(a.target_angles) == 14)
check("vitesses de base séparées des angles", a.base_velocity == (np.float32(0.2).item(), 0.0, np.float32(-0.1).item())
      and not any("base" in j for j in a.target_angles))
check("chunk complet conservé (16 pas)", a.chunk["bras_gauche"].shape == (16, 7) and a.chunk["base"].shape == (16, 3))
check("horodatage = capture (avant l'envoi), latence mesurée",
      t0 <= a.timestamp_ns <= time.monotonic_ns() and a.latency_s >= 0)
cmds = a.to_commands()
check("to_commands() hérité : ActuatorCommand en POSITION, prêt pour le garde",
      cmds["left_j0"].mode == ControlMode.POSITION and cmds["left_j0"].position == 1.0)
c5 = connecteur(srv, step=5)
c5.connect()
check("step=5 exécute le 6e pas du chunk", c5.predict_action(IMG, "x", ETAT).target_angles["left_j0"] == 6.0)
c5.disconnect()
c.reset()
check("reset() atteint le serveur", srv.resets == 1)

section("Entrées refusées AVANT d'être envoyées")
n = len(srv.vues)
check("state absent", leve(ValueError, lambda: c.predict_action(IMG, "x", None)))
check("state incomplet", leve(ValueError, lambda: c.predict_action(IMG, "x", {"left_j0": 0.0})))
check("state avec NaN", leve(ValueError, lambda: c.predict_action(IMG, "x", {**ETAT, "left_j1": float("nan")})))
check("image float refusée", leve(TypeError, lambda: c.predict_action(IMG.astype(np.float32), "x", ETAT)))
check("image de mauvaise résolution refusée (pas de redimensionnement implicite)",
      leve(ValueError, lambda: c.predict_action(IMG[:60], "x", ETAT)))
check("rien n'est parti vers le serveur", len(srv.vues) == n)

section("Réponses défectueuses")
for mode, label in (("nan", "action avec NaN"), ("forme", "mauvaise forme"), ("cle", "clé manquante"),
                    ("erreur", "erreur du serveur"), ("pickle", "tableau dtype object (pickle)")):
    srv.mode = mode
    check(f"{label} → rejetée", leve((RuntimeError, ValueError), lambda: c.predict_action(IMG, "x", ETAT)))
srv.mode = "ok"
check("après les erreurs, le connecteur refonctionne", c.predict_action(IMG, "x", ETAT).target_angles["left_j0"] == 1.0)

section("Timeout : le socket REQ est recréé")
srv.mode, srv.delai = "lent", 0.6
cl = connecteur(srv, request_timeout_ms=200)
cl.connect()
check("timeout → RuntimeError explicite", leve(RuntimeError, lambda: cl.predict_action(IMG, "x", ETAT)))
time.sleep(0.8)                                  # laisse le serveur finir la requête perdue
srv.mode = "ok"
check("l'appel suivant repart (REQ non bloqué)", cl.predict_action(IMG, "x", ETAT).target_angles["left_j0"] == 1.0)
cl.disconnect()
check("serveur injoignable : connect() rend False, sans lever",
      GR00TConnector("tcp://127.0.0.1:1", arm_joints=ARMS, request_timeout_ms=200).connect() is False)

section("Jeton d'API")
srv.stop()
srv2 = FauxServeur(token="secret")
srv2.start()
check("sans jeton : refusé", connecteur(srv2).connect() is False)
check("avec le bon jeton : accepté", connecteur(srv2, api_token="secret").connect())
srv2.stop()

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
