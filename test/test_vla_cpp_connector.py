"""
AXIS-OS — Tests VLACppConnector
===================================
Valide le client ZeroMQ/Protobuf de axis_os/store/sdk/ai/connector.py,
compatible avec le protocole EXACT du serveur d'inférence vla.cpp
(https://github.com/VinRobotics/vla.cpp, licence Apache 2.0) — voir
axis_os/store/sdk/ai/proto/vla.proto pour le schéma source, copié tel
quel depuis le dépôt et compilé avec protoc (vla_pb2.py généré).

Les tests d'intégration (sections 3+) montent un VRAI serveur ZeroMQ
minimal en thread, parlant le protocole vla.proto — pas de mock du
socket : on valide que le client fonctionne vraiment sur le réseau
(loopback), pas seulement que son code s'exécute sans erreur.
"""

import sys
import threading
import time
sys.path.insert(0, ".")

import zmq
from axis_os.store.sdk.ai.proto import vla_pb2
from axis_os.store.sdk.ai.connector import VLACppConnector, VLAAction

FAIL = 0
_next_port = [5700]  # port unique par test pour éviter les collisions


def check(description: str, condition: bool) -> None:
    global FAIL
    status = "✓" if condition else "✗"
    print(f"  [{status}] {description}")
    if not condition:
        FAIL += 1


def section(title: str) -> None:
    print(f"\n{'=' * 66}\n  {title}\n{'=' * 66}")


def next_endpoint() -> str:
    _next_port[0] += 1
    return f"tcp://127.0.0.1:{_next_port[0]}"


def start_fake_server(endpoint: str, handler) -> threading.Thread:
    """Lance un serveur ZeroMQ REP minimal en thread daemon, qui
    traite UNE requête avec `handler(PredictRequest) -> PredictResponse`
    puis se ferme. Suffisant pour un test par requête, pas un vrai
    serveur persistant."""
    def _run():
        ctx = zmq.Context.instance()
        sock = ctx.socket(zmq.REP)
        sock.bind(endpoint)
        raw = sock.recv()
        req = vla_pb2.PredictRequest()
        req.ParseFromString(raw)
        resp = handler(req)
        sock.send(resp.SerializeToString())
        sock.close()

    t = threading.Thread(target=_run, daemon=True)
    t.start()
    time.sleep(0.2)  # laisse le temps au bind() de se faire avant que le client connecte
    return t


# ══════════════════════════════════════════════════════════════════

section("1 — Erreurs sans connexion / sans tokenizer")

c = VLACppConnector(endpoint=next_endpoint())
try:
    c.predict_action(b"fake", "test")
    check("predict_action() sans connect() lève RuntimeError", False)
except RuntimeError:
    check("predict_action() sans connect() lève RuntimeError", True)

c.connect()
try:
    c.predict_action({"height": 1, "width": 1, "data": b"\x00\x00\x00"}, "test")
    check("predict_action() sans lang_tokenizer lève RuntimeError", False)
except RuntimeError:
    check("predict_action() sans lang_tokenizer lève RuntimeError", True)
c.disconnect()
check("disconnect() -> is_connected() == False", not c.is_connected())

# ══════════════════════════════════════════════════════════════════

section("2 — Format d'image non reconnu")

c2 = VLACppConnector(endpoint=next_endpoint(), lang_tokenizer=lambda s: [1])
c2.connect()
try:
    c2.predict_action(12345, "test")  # ni dict, ni objet avec tobytes/shape
    check("Image de type inattendu lève TypeError", False)
except TypeError:
    check("Image de type inattendu lève TypeError", True)
c2.disconnect()

# ══════════════════════════════════════════════════════════════════

section("3 — Round-trip complet avec un vrai serveur ZeroMQ (protocole réel)")

endpoint = next_endpoint()

def echo_handler(req):
    resp = vla_pb2.PredictResponse()
    resp.request_id = req.request_id
    resp.action_dim = 3
    resp.chunk_size = 1
    resp.action_chunk.extend([0.1, 0.2, 0.3])
    return resp

start_fake_server(endpoint, echo_handler)

client = VLACppConnector(
    endpoint=endpoint,
    joint_order=["left_elbow_joint", "right_elbow_joint", "left_wrist_roll_joint"],
    lang_tokenizer=lambda s: [ord(c) for c in s[:10]],
)
check("connect() réussit", client.connect())

fake_image = {"height": 2, "width": 2, "data": b"\x00" * (2 * 2 * 3)}
action = client.predict_action(fake_image, "attrape la tasse")

check("Retourne un VLAAction", isinstance(action, VLAAction))
check("target_angles mappé sur les VRAIS joint_id (joint_order)",
      set(action.target_angles.keys()) == {"left_elbow_joint", "right_elbow_joint",
                                             "left_wrist_roll_joint"})
check("Valeurs d'action correctement décodées",
      abs(action.target_angles["left_elbow_joint"] - 0.1) < 1e-5 and
      abs(action.target_angles["right_elbow_joint"] - 0.2) < 1e-5)

client.disconnect()

# ══════════════════════════════════════════════════════════════════

section("4 — Sans joint_order : noms génériques")

endpoint2 = next_endpoint()
start_fake_server(endpoint2, echo_handler)

client2 = VLACppConnector(endpoint=endpoint2, lang_tokenizer=lambda s: [1, 2])
client2.connect()
action2 = client2.predict_action({"height": 1, "width": 1, "data": b"\x00\x00\x00"}, "x")
check("Sans joint_order -> noms génériques joint_0/joint_1/joint_2",
      set(action2.target_angles.keys()) == {"joint_0", "joint_1", "joint_2"})
client2.disconnect()

# ══════════════════════════════════════════════════════════════════

section("5 — Mismatch de dimension (joint_order incohérent avec le modèle serveur)")

endpoint3 = next_endpoint()
start_fake_server(endpoint3, echo_handler)  # renvoie toujours 3 valeurs

client3 = VLACppConnector(
    endpoint=endpoint3,
    joint_order=["only_one_joint"],  # 1 seul nom pour 3 valeurs retournées -> erreur
    lang_tokenizer=lambda s: [1],
)
client3.connect()
try:
    client3.predict_action({"height": 1, "width": 1, "data": b"\x00\x00\x00"}, "x")
    check("Mismatch dimension action/joint_order lève RuntimeError", False)
except RuntimeError:
    check("Mismatch dimension action/joint_order lève RuntimeError", True)
client3.disconnect()

# ══════════════════════════════════════════════════════════════════

section("6 — Le serveur retourne une erreur (champ 'error' du protocole)")

endpoint4 = next_endpoint()

def error_handler(req):
    resp = vla_pb2.PredictResponse()
    resp.request_id = req.request_id
    resp.error = "Model checkpoint mismatch: expected 23 DOF, got 29"
    return resp

start_fake_server(endpoint4, error_handler)

client4 = VLACppConnector(endpoint=endpoint4, lang_tokenizer=lambda s: [1])
client4.connect()
try:
    client4.predict_action({"height": 1, "width": 1, "data": b"\x00\x00\x00"}, "x")
    check("Erreur serveur (champ 'error') propagée en RuntimeError", False)
except RuntimeError as e:
    check("Erreur serveur (champ 'error') propagée en RuntimeError",
          "DOF" in str(e))
client4.disconnect()

# ══════════════════════════════════════════════════════════════════

section("7 — Timeout : aucun serveur ne répond")

client5 = VLACppConnector(
    endpoint=next_endpoint(),  # personne n'écoute sur ce port
    lang_tokenizer=lambda s: [1],
    request_timeout_ms=400,
)
client5.connect()
start = time.time()
try:
    client5.predict_action({"height": 1, "width": 1, "data": b"\x00\x00\x00"}, "x")
    check("Timeout lève RuntimeError", False)
except RuntimeError:
    elapsed = time.time() - start
    check(f"Timeout lève RuntimeError après ~{elapsed:.1f}s (attendu ~0.4s)",
          0.3 < elapsed < 2.0)
client5.disconnect()

# ══════════════════════════════════════════════════════════════════

section("8 — chunk_size/action_dim == 0 : action vide, pas de crash")

endpoint6 = next_endpoint()

def empty_handler(req):
    resp = vla_pb2.PredictResponse()
    resp.request_id = req.request_id
    resp.chunk_size = 0
    resp.action_dim = 0
    return resp

start_fake_server(endpoint6, empty_handler)
client6 = VLACppConnector(endpoint=endpoint6, lang_tokenizer=lambda s: [1])
client6.connect()
action6 = client6.predict_action({"height": 1, "width": 1, "data": b"\x00\x00\x00"}, "x")
check("Réponse vide -> VLAAction avec target_angles vide (pas de crash)",
      action6.target_angles == {})
client6.disconnect()

# ══════════════════════════════════════════════════════════════════

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)

if FAIL == 0:
    print("\n  ✓ Protocole ZeroMQ/Protobuf compatible vla.cpp validé de bout en bout")
    print("  ✓ Erreurs serveur et timeouts correctement propagés")
    print("  ✓ Mapping vers les vrais joint_id du robot fonctionne")
    print("  ✓ Cas limites (réponse vide, mismatch dimension) gérés sans crash")

sys.exit(0 if FAIL == 0 else 1)
