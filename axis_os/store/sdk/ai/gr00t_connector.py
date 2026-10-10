"""
AXIS-OS — Connecteur GR00T (NVIDIA Isaac GR00T N1.7)
======================================================
Branche le modèle GR00T entraîné sur nos démonstrations (axis_os/vla/) dans
une app, derrière le contrat IVLAConnector.

Protocole EXACT, lu dans le code source de NVIDIA (gr00t/policy/server_client.py,
dépôt Isaac-GR00T) — pas une supposition :
  - ZeroMQ REQ/REP synchrone, messages msgpack + msgpack_numpy ;
  - requête   {"endpoint": "get_action", "data": {"observation": ..., "options": None}}
              (+ "api_token" si le serveur en exige un) ;
  - réponse   [action, info] ; en cas d'erreur {"error": "..."} ;
  - endpoints utilisés ici : "ping", "get_action", "reset".
Le serveur se lance, dans l'environnement GR00T (pas dans AXIS-OS) :

    uv run python gr00t/eval/run_gr00t_server.py \\
        --model-path ~/vla_gr00t_g1/checkpoint-2000 \\
        --embodiment-tag NEW_EMBODIMENT \\
        --modality-config-path <AXIS-OS>/axis_os/vla/config_gr00t_g1.py \\
        --port 5555

FORMAT (celui de axis_os/vla/config_gr00t_g1.py et de la collecte — l'entraînement
et l'inférence DOIVENT parler pareil, sinon le modèle reçoit des entrées qu'il
n'a jamais vues, sans aucune erreur) :
  observation  video   {"ego_view": uint8 (1, 1, H, W, 3)}   caméra de tête
               state   {"bras_gauche": float32 (1, 1, 7),
                        "bras_droit":  float32 (1, 1, 7)}     positions articulaires
               language {"annotation.human.task_description": [[consigne]]}
  action       {"bras_gauche": (1, 16, 7), "bras_droit": (1, 16, 7),
                "base": (1, 16, 3)}   16 pas à 10 i/s ; bras en positions ABSOLUES
                (le serveur a déjà décodé le relatif d'après l'état envoyé),
                base = vitesses (vx, vy, wz).

SÉCURITÉ : ce connecteur ne contourne rien. Il rejette seulement ce qui n'a
aucun sens (forme inattendue, NaN/inf) ; les angles produits passent ensuite
par SetpointFollower puis JointSafetyGuard, comme toute autre source de
commandes. Les vitesses de base (`base_velocity`) NE sont PAS des angles : c'est
à l'appelant de les envoyer à la locomotion, qui les borne et les fait valider
elle-même.

LA RÉPONSE DU SERVEUR EST UNE DONNÉE NON FIABLE (audit du 2026-10-10) :
  - jamais de pickle : tableaux de dtype object refusés, à l'aller comme au
    retour (msgpack_numpy passerait par pickle.loads) ; np.load toujours
    avec allow_pickle=False, et seulement un tableau (pas une archive npz) ;
  - seulement des nombres réels (bool, entiers, flottants) ou du texte :
    complexes, structures et dates refusés ; les ACTIONS, elles, ne sont
    acceptées que réelles (entiers ou flottants) ;
  - taille de message bornée (ZMQ_MAXMSGSIZE) : un serveur défaillant ne
    peut pas saturer la mémoire du robot ;
  - serveur en BOUCLE LOCALE par défaut : ZeroMQ circule en clair. Sur un
    réseau, les images de la caméra seraient lisibles et les actions
    falsifiables par n'importe qui sur le chemin. Un serveur distant exige
    autoriser_distant=True, choix explicite de l'appelant.

Portée et limites honnêtes :
  - Testé contre un faux serveur qui reproduit le protocole (test/
    test_vla_gr00t_connector.py), PAS encore contre un vrai serveur GR00T :
    le premier essai sur la machine GPU est la vraie validation.
  - L'image doit avoir la MÊME résolution et le même traitement qu'à la
    collecte (caméra de tête réduite de moitié, RGB uint8) : ce connecteur ne
    redimensionne pas. `image_shape` permet de le vérifier.
  - Dépendances (pyzmq, msgpack, msgpack-numpy) importées à connect(), pour ne
    rien imposer aux apps qui n'utilisent pas ce connecteur.

Licence : Apache 2.0 (SDK public)
"""
from __future__ import annotations

import re
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Sequence, Tuple

import numpy as np

from axis_os.store.sdk.ai.connector import IVLAConnector, VLAAction

CLE_LANGUE = "annotation.human.task_description"
CLE_VIDEO = "ego_view"
N_BRAS = 7          # articulations par bras (bras_gauche, bras_droit)
N_BASE = 3          # vx, vy, wz
HORIZON = 16        # pas prédits par appel (config_gr00t_g1.py)
# Taille maximale d'un message reçu. Une réponse normale pèse quelques Ko
# (3 tableaux de 16 pas) : 8 Mo laissent une marge énorme tout en
# empêchant un serveur défaillant d'envoyer des Go au robot.
TAILLE_MAX_OCTETS = 8 * 1024 * 1024
# Types numpy acceptés dans une réponse : bool, entiers signés/non signés,
# flottants — et du texte (U, S), inoffensif (taille fixe, jamais de
# pickle), que le dictionnaire `info` du serveur peut légitimement contenir.
# Refusés : objets (pickle), complexes, structures, dates. Les ACTIONS,
# elles, ne peuvent être que des nombres réels (voir _vers_action).
_DTYPES_ACCEPTES = "biufUS"
# Endpoints en boucle locale : rien ne sort de la machine.
_LOCAL = re.compile(r"^(ipc|inproc)://.+$"
                    r"|^tcp://(127(\.\d{1,3}){3}|localhost|\[::1\]):\d+$")


def endpoint_local(endpoint: str) -> bool:
    """Vrai si l'endpoint ZeroMQ reste sur la machine (127.x.x.x, localhost,
    [::1], ipc://, inproc://)."""
    return bool(_LOCAL.match(endpoint or ""))


def _verifier_reel(obj: Any) -> Any:
    """Refuse tout tableau ou scalaire numpy qui n'est ni un nombre réel ni du
    texte (objets, complexes, structures, dates)."""
    if isinstance(obj, (np.ndarray, np.generic)) and obj.dtype.kind not in _DTYPES_ACCEPTES:
        raise ValueError(f"tableau de dtype {obj.dtype} refusé dans la réponse "
                         "(nombres réels ou texte seulement)")
    if isinstance(obj, complex):
        raise ValueError("nombre complexe refusé dans la réponse")
    return obj


@dataclass
class GR00TAction(VLAAction):
    """VLAAction + ce que GR00T produit en plus des angles.

    target_angles : le pas `step` du chunk, par joint_id (bras uniquement).
    base_velocity : (vx, vy, wz) du même pas — à envoyer à la locomotion, PAS au
                    suiveur d'angles.
    chunk         : les 16 pas complets {"bras_gauche": (16,7), ...}, pour un
                    exécuteur qui voudrait rejouer plusieurs pas ; ignorés par
                    défaut (un seul pas exécuté par appel : le plus prudent).
    latency_s     : durée aller-retour de la requête (mesurée).
    """
    base_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    chunk: Dict[str, np.ndarray] = field(default_factory=dict)
    latency_s: float = 0.0


def _verifier_bras(arm_joints: Sequence[str]) -> None:
    if len(arm_joints) != 2 * N_BRAS:
        raise ValueError(f"arm_joints doit contenir {2 * N_BRAS} noms "
                         f"({N_BRAS} gauche puis {N_BRAS} droit), reçu {len(arm_joints)}")
    g, d = arm_joints[:N_BRAS], arm_joints[N_BRAS:]
    if not all(n.startswith("left_") for n in g) or not all(n.startswith("right_") for n in d):
        raise ValueError("arm_joints : les 7 premiers doivent commencer par 'left_', les 7 "
                         "suivants par 'right_' — même ordre que la collecte (bras_j).")


class GR00TConnector(IVLAConnector):
    def __init__(self, endpoint: str = "tcp://127.0.0.1:5555",
                 arm_joints: Sequence[str] = (), api_token: Optional[str] = None,
                 image_shape: Optional[Tuple[int, int]] = None,
                 connect_timeout_ms: int = 5000, request_timeout_ms: int = 3000,
                 step: int = 0, taille_max_octets: int = TAILLE_MAX_OCTETS,
                 autoriser_distant: bool = False):
        """
        arm_joints  : 14 joint_id du robot, bras gauche (7) puis droit (7), dans
                      l'ordre de la collecte (`cfg.body_joints[cfg.num_actions:]`).
        image_shape : (hauteur, largeur) attendues ; une image différente est
                      refusée au lieu d'être envoyée telle quelle.
        step        : pas du chunk exécuté (0 = le plus prudent).
        taille_max_octets : au-delà, la réponse est refusée par ZeroMQ lui-même
                      (ZMQ_MAXMSGSIZE) et l'appel échoue en timeout.
        autoriser_distant : False (défaut) = seul un serveur en boucle locale est
                      accepté. ZeroMQ circule EN CLAIR : un serveur distant
                      exposerait les images de la caméra et accepterait des
                      actions falsifiées par quiconque est sur le chemin.
        """
        _verifier_bras(arm_joints)
        if not 0 <= step < HORIZON:
            raise ValueError(f"step doit être dans [0, {HORIZON - 1}]")
        if not autoriser_distant and not endpoint_local(endpoint):
            raise ValueError(
                f"endpoint '{endpoint}' hors de la machine refusé : ZeroMQ circule "
                "en clair (images de la caméra lisibles, actions falsifiables). "
                "Utiliser tcp://127.0.0.1:<port> (tunnel SSH si le serveur est "
                "ailleurs), ou autoriser_distant=True en connaissance de cause.")
        if taille_max_octets <= 0:
            raise ValueError("taille_max_octets doit être > 0")
        self._taille_max = int(taille_max_octets)
        self._endpoint, self._arm_joints = endpoint, list(arm_joints)
        self._token, self._image_shape = api_token, image_shape
        self._connect_ms, self._request_ms, self._step = connect_timeout_ms, request_timeout_ms, step
        self._ctx = self._sock = None
        self._connected = False

    @property
    def arm_joints(self) -> list:
        """Les 14 joint_id, bras gauche puis droit (copie)."""
        return list(self._arm_joints)

    # ---- sérialisation (même contrat que MsgSerializer de NVIDIA) -------
    @staticmethod
    def _enc(obj):
        import msgpack_numpy as mnp
        if isinstance(obj, np.ndarray) and obj.dtype.kind == "O":
            raise TypeError("tableau de dtype object refusé (msgpack_numpy passerait par pickle)")
        return mnp.encode(obj)

    @staticmethod
    def _dec(obj):
        import io
        import msgpack_numpy as mnp
        if isinstance(obj, dict):
            if obj.get("__ndarray_class__", obj.get(b"__ndarray_class__")):
                payload = obj.get("as_npy", obj.get(b"as_npy"))
                if not isinstance(payload, (bytes, bytearray)):
                    raise ValueError("tableau mal formé : 'as_npy' manquant ou illisible")
                arr = np.load(io.BytesIO(payload), allow_pickle=False)
                if not isinstance(arr, np.ndarray):
                    # Une archive .npz se charge aussi par np.load : refusée,
                    # le protocole n'envoie que des tableaux (format .npy).
                    close = getattr(arr, "close", None)
                    if close is not None:
                        close()
                    raise ValueError("archive npz refusée dans la réponse "
                                     "(un tableau .npy est attendu)")
                return _verifier_reel(arr)
            if obj.get(b"nd", obj.get("nd")) and obj.get(b"kind", obj.get("kind")) in (b"O", "O"):
                raise ValueError("tableau de dtype object refusé dans la réponse (pickle)")
        return _verifier_reel(mnp.decode(obj))

    def _ouvrir_socket(self) -> None:
        import zmq
        if self._sock is not None:
            self._sock.close(linger=0)
        self._sock = self._ctx.socket(zmq.REQ)
        self._sock.setsockopt(zmq.LINGER, 0)
        # Message plus gros que la borne : ZeroMQ coupe la connexion au lieu de
        # l'allouer, et l'appel échoue en timeout (voir _appel).
        self._sock.setsockopt(zmq.MAXMSGSIZE, self._taille_max)
        self._sock.setsockopt(zmq.RCVTIMEO, self._request_ms)
        self._sock.setsockopt(zmq.SNDTIMEO, self._connect_ms)
        self._sock.connect(self._endpoint)

    def _appel(self, endpoint: str, data: Optional[dict] = None) -> Any:
        import msgpack
        import zmq
        requete: Dict[str, Any] = {"endpoint": endpoint}
        if data is not None:
            requete["data"] = data
        if self._token:
            requete["api_token"] = self._token
        try:
            self._sock.send(msgpack.packb(requete, default=self._enc))
            brut = self._sock.recv()
        except zmq.error.Again as exc:
            # Un REQ resté en attente d'une réponse qui ne viendra jamais est
            # inutilisable : on le recrée pour que l'appel suivant puisse partir.
            self._ouvrir_socket()
            raise RuntimeError(f"Timeout GR00T sur '{self._endpoint}' "
                               f"(> {self._request_ms} ms, endpoint '{endpoint}' ; "
                               f"ou réponse > {self._taille_max} octets, refusée)") from exc
        rep = msgpack.unpackb(brut, object_hook=self._dec, raw=False)
        if isinstance(rep, dict) and "error" in rep:
            raise RuntimeError(f"Erreur du serveur GR00T : {rep['error']}")
        return rep

    # ---- IVLAConnector ----------------------------------------------------
    def connect(self) -> bool:
        """Ouvre le socket ET vérifie que le serveur répond (endpoint 'ping')."""
        try:
            import zmq
            self._ctx = zmq.Context.instance()
            self._ouvrir_socket()
            rep = self._appel("ping")
            self._connected = isinstance(rep, dict) and rep.get("status") == "ok"
        except Exception as exc:                      # noqa: BLE001
            print(f"[GR00TConnector] Échec de connexion à '{self._endpoint}' : {exc}")
            self._connected = False
        if self._connected:
            print(f"[GR00TConnector] Serveur GR00T joint sur '{self._endpoint}'")
        return self._connected

    def disconnect(self) -> None:
        if self._sock is not None:
            self._sock.close(linger=0)
            self._sock = None
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def reset(self) -> None:
        """À appeler au début de chaque épisode / mission."""
        self._exiger_connecte()
        self._appel("reset", {"options": None})

    def _exiger_connecte(self) -> None:
        if not self._connected or self._sock is None:
            raise RuntimeError("GR00TConnector non connecté — appelez connect() d'abord.")

    def predict_action(self, image: Any, instruction: str,
                       state: Optional[Dict[str, float]] = None) -> GR00TAction:
        """
        image       : tableau numpy (H, W, 3) uint8, RGB, comme à la collecte.
        instruction : en anglais, comme à l'entraînement ("pick up the red box" —
                      voir axis_os.vla.collecte.consigne).
        state       : {joint_id: position} pour les 14 articulations des bras.
                      OBLIGATOIRE : GR00T décode ses actions relatives à partir
                      de l'état, un état absent ou faux décale tous les bras.
        """
        self._exiger_connecte()
        if not state:
            raise ValueError("state obligatoire (positions des 14 articulations des bras)")
        manquants = [j for j in self._arm_joints if j not in state]
        if manquants:
            raise ValueError(f"state incomplet, manquent : {manquants}")
        img = np.asarray(image)
        if img.dtype != np.uint8 or img.ndim != 3 or img.shape[2] != 3:
            raise TypeError(f"image attendue (H, W, 3) uint8, reçu {img.dtype} {img.shape}")
        if self._image_shape is not None and tuple(img.shape[:2]) != tuple(self._image_shape):
            raise ValueError(f"image {img.shape[:2]} ≠ {tuple(self._image_shape)} attendu "
                             "(résolution de la collecte) — pas de redimensionnement implicite")
        v = np.asarray([state[j] for j in self._arm_joints], dtype=np.float32)
        if not np.all(np.isfinite(v)):
            raise ValueError("state contient NaN/inf")

        obs = {
            "video": {CLE_VIDEO: img[None, None]},
            "state": {"bras_gauche": v[None, None, :N_BRAS], "bras_droit": v[None, None, N_BRAS:]},
            "language": {CLE_LANGUE: [[str(instruction)]]},
        }
        # Horodatage = instant de la CAPTURE (juste avant l'envoi), pas de la
        # réponse : la décision a au moins cet âge. L'erreur va donc dans le sens
        # conservateur pour l'échéance de fraîcheur de SetpointFollower.
        t_ns = time.monotonic_ns()
        rep = self._appel("get_action", {"observation": obs, "options": None})
        latence = (time.monotonic_ns() - t_ns) / 1e9
        return self._vers_action(rep, t_ns, latence)

    def _vers_action(self, rep: Any, t_ns: int, latence: float) -> GR00TAction:
        try:
            action = rep[0]
        except (TypeError, KeyError, IndexError) as exc:
            raise RuntimeError(f"réponse GR00T inattendue : {type(rep).__name__}") from exc
        if not isinstance(action, dict):
            raise RuntimeError(f"réponse GR00T inattendue : action de type {type(action).__name__}")
        attendu = {"bras_gauche": N_BRAS, "bras_droit": N_BRAS, "base": N_BASE}
        chunk: Dict[str, np.ndarray] = {}
        for cle, dim in attendu.items():
            if cle not in action:
                raise RuntimeError(f"réponse GR00T sans la clé '{cle}' — le modèle chargé "
                                   "n'a pas été entraîné avec config_gr00t_g1.py ?")
            brut = np.asarray(action[cle])
            # Des nombres réels seulement : un booléen, une chaîne ou un
            # complexe converti en float32 donnerait un angle sans aucun sens.
            if brut.dtype.kind not in "iuf":
                raise RuntimeError(f"'{cle}' de type {brut.dtype}, attendu des nombres réels")
            # Converti AVANT le test de finitude : 1e39, fini en float64,
            # devient inf en float32 — il est donc rejeté aussi.
            a = brut.astype(np.float32)
            if a.ndim != 3 or a.shape[0] != 1 or a.shape[2] != dim or a.shape[1] <= self._step:
                raise RuntimeError(f"'{cle}' de forme {a.shape}, attendu (1, ≥{self._step + 1}, {dim})")
            if not np.all(np.isfinite(a)):
                raise RuntimeError(f"'{cle}' contient NaN/inf — action rejetée")
            chunk[cle] = a[0]
        s = self._step
        angles = np.concatenate([chunk["bras_gauche"][s], chunk["bras_droit"][s]])
        vx, vy, wz = (float(x) for x in chunk["base"][s])
        return GR00TAction(
            target_angles={j: float(x) for j, x in zip(self._arm_joints, angles)},
            gripper_closed=False, confidence=None, timestamp_ns=t_ns,
            base_velocity=(vx, vy, wz), chunk=chunk, latency_s=latence,
        )
