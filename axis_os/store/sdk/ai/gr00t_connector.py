"""
AXIS-OS — Connecteur GR00T (NVIDIA Isaac GR00T N1.7)
======================================================
GR00T est le modèle d'IA de NVIDIA pour robots humanoïdes. C'est un modèle
VLA (vision-langage-action) : il reçoit une image de caméra et une consigne
en langage naturel, et répond par les mouvements à exécuter. Ce fichier
permet à une app d'utiliser le modèle GR00T que nous avons entraîné sur nos
propres démonstrations (voir axis_os/vla/), derrière IVLAConnector, le
contrat commun à tous les connecteurs VLA (défini dans connector.py).

Le modèle ne tourne pas dans AXIS-OS mais dans un serveur séparé, fourni par
NVIDIA, en général sur une machine équipée d'un GPU (carte graphique de
calcul). Ce connecteur en est le client.

Protocole d'échange
-------------------
Ce protocole a été lu dans le code source de NVIDIA
(gr00t/policy/server_client.py, dépôt Isaac-GR00T), et non supposé :
  - transport ZeroMQ (bibliothèque de messagerie réseau) en mode REQ/REP
    (requête/réponse) synchrone : chaque requête attend sa réponse avant
    la suivante ;
  - messages encodés en msgpack (format binaire compact, proche du JSON),
    avec l'extension msgpack_numpy pour transporter des tableaux numpy ;
  - requête   {"endpoint": "get_action",
               "data": {"observation": ..., "options": None}},
              plus "api_token" si le serveur exige un jeton d'accès ;
  - réponse   [action, info] ; en cas d'erreur, {"error": "..."} ;
  - fonctions du serveur appelées ici (ses « endpoints ») : "ping",
    "get_action" et "reset".
Attention aux deux sens du mot « endpoint » dans ce fichier : dans une
requête, c'est le nom de la fonction appelée ; pour ZeroMQ, c'est l'adresse
du serveur (par exemple tcp://127.0.0.1:5555).

Le serveur se lance dans l'environnement logiciel de GR00T (pas dans celui
d'AXIS-OS) :

    uv run python gr00t/eval/run_gr00t_server.py \\
        --model-path ~/vla_gr00t_g1/checkpoint-2000 \\
        --embodiment-tag NEW_EMBODIMENT \\
        --modality-config-path <AXIS-OS>/axis_os/vla/config_gr00t_g1.py \\
        --port 5555

(--model-path désigne le checkpoint, c'est-à-dire les poids du modèle
sauvegardés pendant l'entraînement ; NEW_EMBODIMENT est l'étiquette que
GR00T réserve à un robot qui ne fait pas partie de ceux prévus d'origine.)

Format des données
------------------
C'est celui de axis_os/vla/config_gr00t_g1.py et de la collecte des
démonstrations. L'entraînement et l'inférence (l'utilisation du modèle
entraîné) doivent employer exactement le même format : sinon le modèle
reçoit des entrées qu'il n'a jamais vues, et aucune erreur ne le signale.
  observation  video    {"ego_view": uint8 (1, 1, H, W, 3)}  caméra de tête
               state    {"bras_gauche": float32 (1, 1, 7),
                         "bras_droit":  float32 (1, 1, 7)}
                        positions articulaires des deux bras
               language {"annotation.human.task_description": [[consigne]]}
  action       {"bras_gauche": (1, 16, 7), "bras_droit": (1, 16, 7),
                "base": (1, 16, 3)}
Lecture des formes : la première dimension (1) est le lot, ici une seule
observation ; la deuxième compte les instants (un seul pour l'observation,
16 pas pour l'action) ; H et W sont la hauteur et la largeur de l'image, 3
ses canaux de couleur ; 7 est le nombre d'articulations par bras, 3 celui
des vitesses de la base. uint8 désigne des entiers de 0 à 255, float32
des nombres à virgule sur 32 bits.
L'action couvre 16 pas, à 10 pas par seconde. Les bras y sont donnés en
positions ABSOLUES : le modèle prédit des positions relatives à l'état
envoyé, et le serveur les a déjà converties. La base est donnée en vitesses
(vx, vy, wz) : avance, déplacement latéral et rotation sur place.

Sécurité
--------
Ce connecteur ne contourne aucune protection. Il rejette seulement ce qui
n'a aucun sens : forme de tableau inattendue, NaN ou inf (valeur indéfinie
ou infinie). Les angles produits passent ensuite par SetpointFollower (le
suiveur de consigne, qui rejoint la cible par petits pas et fige le robot
si la consigne devient trop ancienne), puis par JointSafetyGuard (le garde
qui vérifie chaque commande d'articulation et refuse celles qui sortent
de ses limites), comme ceux
de n'importe quelle autre source de commandes. Les passer au suiveur avec
`set_target(..., origine=ORIGINE_IA)` : le contexte de l'app exige alors
la permission ACTUATORS_AI_DRIVEN pour chaque pas. Les vitesses de base
(`base_velocity`) ne sont PAS des angles : c'est à l'appelant de les
transmettre au service de locomotion, qui les borne et les fait valider
lui-même ; ce chemin-là ne porte pas la marque d'origine (voir
SECURITY.md, limite n° 2).

La réponse du serveur est une donnée non fiable
-----------------------------------------------
Un audit de sécurité mené le 2026-10-10 a conduit à traiter tout ce que
renvoie le serveur comme suspect : le serveur peut être défaillant, ou un
message falsifié en route. Ces protections sont vérifiées par
test/test_vla_gr00t_durcissement.py :
  - jamais de pickle (le format de sérialisation natif de Python, dont la
    lecture peut exécuter du code) : les tableaux numpy de type « object »
    sont refusés, à l'envoi comme à la réception, car msgpack_numpy les
    relirait avec pickle.loads ; np.load est toujours appelé avec
    allow_pickle=False et n'accepte qu'un tableau seul, pas une archive
    .npz (qui regroupe plusieurs tableaux) ;
  - seuls des nombres réels (booléens, entiers, flottants) ou du texte sont
    acceptés : nombres complexes, structures et dates sont refusés ; les
    ACTIONS, elles, ne sont acceptées que sous forme de nombres réels
    (entiers ou flottants) ;
  - la taille d'un message est bornée (option ZMQ_MAXMSGSIZE de ZeroMQ) :
    un serveur défaillant ne peut pas saturer la mémoire du robot ;
  - par défaut, seul un serveur en boucle locale (sur la même machine) est
    accepté, car ZeroMQ circule en clair, sans chiffrement. Sur un réseau,
    n'importe qui placé sur le chemin pourrait lire les images de la caméra
    et falsifier les actions. Un serveur distant exige
    autoriser_distant=True, un choix explicite de l'appelant.

Portée et limites connues
-------------------------
  - Testé contre un faux serveur qui reproduit le protocole
    (test/test_vla_gr00t_connector.py, test/test_vla_gr00t_durcissement.py).
    Utilisé aussi contre le vrai serveur GR00T, sur la machine GPU, pour
    évaluer le modèle de prise (mode --evaluer-vla de
    sim/validation/test_isaac_t1_tri_usine.py, lancé par
    deploy/machine_gpu/evaluer_vla.sh) ; le constat de cet essai est
    rapporté dans axis_os/services/policy/vla_service.py (paramètre
    pas_executes, essai d'octobre 2026). Les protections ajoutées le
    2026-10-10 (taille de message bornée, archives .npz et nombres non
    réels refusés, serveur local par défaut, jeton d'accès) sont
    vérifiées contre le faux serveur ; aucun essai documenté contre le
    vrai serveur ne les couvre encore.
  - L'image doit avoir la même résolution et le même traitement qu'à la
    collecte (image de la caméra de tête réduite de moitié, couleurs RGB,
    c'est-à-dire rouge, vert, bleu, en uint8) : ce connecteur ne
    redimensionne pas. Le paramètre `image_shape` permet de le vérifier.
  - Les dépendances (pyzmq, msgpack, msgpack-numpy) ne sont importées qu'à
    l'appel de connect(), pour ne rien imposer aux apps qui n'utilisent pas
    ce connecteur.

Licence : Apache 2.0 (SDK public)
"""
from __future__ import annotations

import re
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Sequence, Tuple

import numpy as np

from axis_os.store.sdk.ai.connector import IVLAConnector, VLAAction

# Noms des entrées du modèle, fixés à l'entraînement (config_gr00t_g1.py) :
# consigne en langage naturel et image de la caméra de tête.
CLE_LANGUE = "annotation.human.task_description"
CLE_VIDEO = "ego_view"
N_BRAS = 7          # articulations par bras (bras_gauche, bras_droit)
N_BASE = 3          # vitesses de la base : vx, vy, wz
HORIZON = 16        # pas prédits par appel (config_gr00t_g1.py)
# Taille maximale d'un message reçu. Elle empêche un serveur défaillant
# d'envoyer des gigaoctets au robot et de saturer sa mémoire. Une réponse
# normale pèse quelques kilooctets (3 tableaux de 16 pas) : la borne de
# 8 Mo laisse une marge énorme.
TAILLE_MAX_OCTETS = 8 * 1024 * 1024
# Types d'éléments numpy (« dtype ») acceptés dans une réponse, par leur
# code : b = booléen, i = entier signé, u = entier non signé, f = flottant,
# U et S = texte (Unicode ou octets). Le texte est inoffensif (taille fixe,
# jamais de pickle) et le dictionnaire `info` du serveur peut légitimement
# en contenir.
# Refusés : objets (qui passeraient par pickle), complexes, structures,
# dates. Les ACTIONS, elles, ne peuvent être que des nombres réels (voir
# _vers_action).
_DTYPES_ACCEPTES = "biufUS"
# Adresses ZeroMQ en boucle locale, d'où rien ne sort de la machine :
# ipc:// (communication entre processus de la même machine), inproc://
# (à l'intérieur d'un même processus), ou tcp:// vers 127.x.x.x, localhost
# ou [::1] (la boucle locale en IPv6).
_LOCAL = re.compile(r"^(ipc|inproc)://.+$"
                    r"|^tcp://(127(\.\d{1,3}){3}|localhost|\[::1\]):\d+$")


def endpoint_local(endpoint: str) -> bool:
    """Vrai si l'adresse ZeroMQ (« endpoint ») reste sur la machine :
    127.x.x.x, localhost, [::1], ipc:// ou inproc://. Une adresse vide ou
    absente est considérée comme distante."""
    return bool(_LOCAL.match(endpoint or ""))


def _verifier_reel(obj: Any) -> Any:
    """Refuse tout tableau ou scalaire numpy qui n'est ni un nombre réel ni du
    texte (objets, complexes, structures, dates), ainsi que les nombres
    complexes Python. Appliqué par _dec au résultat du décodage de chaque
    dictionnaire de la réponse ; renvoie la valeur inchangée si elle est
    acceptée."""
    if isinstance(obj, (np.ndarray, np.generic)) and obj.dtype.kind not in _DTYPES_ACCEPTES:
        raise ValueError(f"tableau de dtype {obj.dtype} refusé dans la réponse "
                         "(nombres réels ou texte seulement)")
    if isinstance(obj, complex):
        raise ValueError("nombre complexe refusé dans la réponse")
    return obj


@dataclass
class GR00TAction(VLAAction):
    """VLAAction, complétée de ce que GR00T produit en plus des angles.

    GR00T prédit à chaque appel un « chunk », un bloc de 16 pas successifs ;
    un seul de ces pas est exécuté par défaut.

    target_angles : les angles du pas `step` du chunk, par joint_id
                    (identifiant d'articulation du robot), bras uniquement.
    base_velocity : (vx, vy, wz) du même pas, à envoyer au service de
                    locomotion, PAS au suiveur d'angles.
    chunk         : les 16 pas complets {"bras_gauche": (16, 7), ...}, pour
                    un exécuteur qui voudrait en jouer plusieurs. Ils sont
                    ignorés par défaut : exécuter un seul pas par appel est
                    le plus prudent.
    latency_s     : durée aller-retour de la requête, mesurée.
    """
    base_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    chunk: Dict[str, np.ndarray] = field(default_factory=dict)
    latency_s: float = 0.0


def _verifier_bras(arm_joints: Sequence[str]) -> None:
    """Vérifie que `arm_joints` liste 14 articulations : les 7 du bras gauche
    (noms en left_...) puis les 7 du bras droit (noms en right_...).

    Pourquoi : les 14 valeurs échangées avec le modèle n'ont pas de nom,
    seulement une position dans le tableau. Elles sont associées aux
    articulations dans l'ordre de cette liste, qui doit donc être celui de
    la collecte des démonstrations.
    """
    if len(arm_joints) != 2 * N_BRAS:
        raise ValueError(f"arm_joints doit contenir {2 * N_BRAS} noms "
                         f"({N_BRAS} gauche puis {N_BRAS} droit), reçu {len(arm_joints)}")
    g, d = arm_joints[:N_BRAS], arm_joints[N_BRAS:]
    if not all(n.startswith("left_") for n in g) or not all(n.startswith("right_") for n in d):
        raise ValueError("arm_joints : les 7 premiers doivent commencer par 'left_', les 7 "
                         "suivants par 'right_' — même ordre que la collecte (bras_j).")


class GR00TConnector(IVLAConnector):
    """Client d'un serveur GR00T, derrière le contrat IVLAConnector.

    Usage : connect() une fois, reset() au début de chaque épisode, puis
    predict_action() à chaque décision ; disconnect() pour finir.
    """

    def __init__(self, endpoint: str = "tcp://127.0.0.1:5555",
                 arm_joints: Sequence[str] = (), api_token: Optional[str] = None,
                 image_shape: Optional[Tuple[int, int]] = None,
                 connect_timeout_ms: int = 5000, request_timeout_ms: int = 3000,
                 step: int = 0, taille_max_octets: int = TAILLE_MAX_OCTETS,
                 autoriser_distant: bool = False):
        """
        endpoint    : adresse ZeroMQ du serveur GR00T (boucle locale par
                      défaut, voir `autoriser_distant`).
        arm_joints  : 14 joint_id du robot, bras gauche (7) puis droit (7),
                      dans l'ordre de la collecte
                      (`cfg.body_joints[cfg.num_actions:]`).
        api_token   : jeton d'accès, ajouté à chaque requête si le serveur en
                      exige un.
        image_shape : (hauteur, largeur) attendues ; une image différente est
                      refusée au lieu d'être envoyée telle quelle.
        connect_timeout_ms : délai maximal pour envoyer une requête.
        request_timeout_ms : délai maximal d'attente d'une réponse.
        step        : pas du chunk exécuté (0, le premier, est le plus
                      prudent).
        taille_max_octets : au-delà, la réponse est refusée par ZeroMQ
                      lui-même (option ZMQ_MAXMSGSIZE) et l'appel échoue
                      comme un dépassement de délai.
        autoriser_distant : à False (par défaut), seul un serveur en boucle
                      locale est accepté. Pourquoi : ZeroMQ circule EN
                      CLAIR ; avec un serveur distant, quiconque est sur le
                      chemin pourrait lire les images de la caméra et
                      falsifier les actions. Pour un serveur sur une autre
                      machine, la voie prévue est un tunnel SSH (connexion
                      chiffrée) vers tcp://127.0.0.1:<port>.
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
    # Encodage et décodage des messages msgpack, dans le format du code de
    # NVIDIA (sa classe MsgSerializer), avec les refus décrits en tête de
    # fichier.
    @staticmethod
    def _enc(obj):
        """Encode un objet que msgpack ne connaît pas (un tableau numpy).
        Un tableau de type « object » est refusé dès l'envoi : msgpack_numpy
        le transmettrait sous forme de pickle."""
        import msgpack_numpy as mnp
        if isinstance(obj, np.ndarray) and obj.dtype.kind == "O":
            raise TypeError("tableau de dtype object refusé (msgpack_numpy passerait par pickle)")
        return mnp.encode(obj)

    @staticmethod
    def _dec(obj):
        """Décode chaque dictionnaire reçu, en refusant tout ce qui passerait
        par pickle ou ne serait pas un nombre réel ou du texte.

        Deux formats de tableau sont reconnus : celui de NVIDIA (le tableau
        voyage comme un fichier .npy dans le champ "as_npy") et celui de
        msgpack_numpy. Les clés sont cherchées sous forme de texte comme
        d'octets.
        """
        import io
        import msgpack_numpy as mnp
        if isinstance(obj, dict):
            # Format de NVIDIA : relu par np.load, sans jamais autoriser
            # pickle.
            if obj.get("__ndarray_class__", obj.get(b"__ndarray_class__")):
                payload = obj.get("as_npy", obj.get(b"as_npy"))
                if not isinstance(payload, (bytes, bytearray)):
                    raise ValueError("tableau mal formé : 'as_npy' manquant ou illisible")
                arr = np.load(io.BytesIO(payload), allow_pickle=False)
                if not isinstance(arr, np.ndarray):
                    # Piège : np.load charge aussi une archive .npz (plusieurs
                    # tableaux), alors que le protocole n'envoie que des
                    # tableaux seuls (format .npy). L'archive est fermée,
                    # puis refusée.
                    close = getattr(arr, "close", None)
                    if close is not None:
                        close()
                    raise ValueError("archive npz refusée dans la réponse "
                                     "(un tableau .npy est attendu)")
                return _verifier_reel(arr)
            # Format de msgpack_numpy : le type "O" (objets Python) serait
            # relu par pickle dans mnp.decode, il est donc refusé avant.
            if obj.get(b"nd", obj.get("nd")) and obj.get(b"kind", obj.get("kind")) in (b"O", "O"):
                raise ValueError("tableau de dtype object refusé dans la réponse (pickle)")
        return _verifier_reel(mnp.decode(obj))

    def _ouvrir_socket(self) -> None:
        """(Re)crée le socket REQ (le point de connexion ZeroMQ côté client)
        vers le serveur, en fermant l'ancien s'il existe. LINGER à 0 : à la
        fermeture, les messages encore en attente sont abandonnés au lieu de
        bloquer le programme."""
        import zmq
        if self._sock is not None:
            self._sock.close(linger=0)
        self._sock = self._ctx.socket(zmq.REQ)
        self._sock.setsockopt(zmq.LINGER, 0)
        # Borne de taille : face à un message plus gros, ZeroMQ coupe la
        # connexion au lieu de réserver la mémoire nécessaire, et l'appel
        # échoue comme un dépassement de délai (voir _appel).
        self._sock.setsockopt(zmq.MAXMSGSIZE, self._taille_max)
        self._sock.setsockopt(zmq.RCVTIMEO, self._request_ms)
        self._sock.setsockopt(zmq.SNDTIMEO, self._connect_ms)
        self._sock.connect(self._endpoint)

    def _appel(self, endpoint: str, data: Optional[dict] = None) -> Any:
        """Appelle la fonction `endpoint` du serveur et renvoie sa réponse
        décodée. Lève RuntimeError si le délai est dépassé, si la réponse
        dépasse la taille maximale, ou si le serveur signale une erreur ;
        ValueError si le décodage refuse le contenu (voir _dec)."""
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
            # Un socket REQ doit alterner strictement envoi et réception :
            # resté en attente d'une réponse qui ne viendra jamais, il est
            # inutilisable. On le recrée pour que l'appel suivant puisse
            # partir.
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
        """Ouvre le socket ET vérifie que le serveur répond (fonction
        « ping »). Avec ZeroMQ, ouvrir la connexion réussit même si aucun
        serveur n'écoute : seul un échange réel prouve qu'il est là.
        Renvoie False en cas d'échec au lieu de lever une exception ; une
        erreur de connexion est affichée avec sa cause."""
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
        """Ferme le socket ; connect() devra être rappelé avant tout nouvel
        appel."""
        if self._sock is not None:
            self._sock.close(linger=0)
            self._sock = None
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def reset(self) -> None:
        """À appeler au début de chaque épisode ou mission (une exécution
        complète de la tâche) : demande au serveur de repartir de zéro."""
        self._exiger_connecte()
        self._appel("reset", {"options": None})

    def _exiger_connecte(self) -> None:
        """Lève RuntimeError si connect() n'a pas abouti."""
        if not self._connected or self._sock is None:
            raise RuntimeError("GR00TConnector non connecté — appelez connect() d'abord.")

    def predict_action(self, image: Any, instruction: str,
                       state: Optional[Dict[str, float]] = None) -> GR00TAction:
        """Envoie une observation au serveur et renvoie l'action du pas `step`.

        image       : tableau numpy (H, W, 3) uint8, en RGB, comme à la
                      collecte.
        instruction : en anglais, comme à l'entraînement ("pick up the red
                      box" ; voir axis_os.vla.collecte.consigne).
        state       : {joint_id: position} pour les 14 articulations des bras.
                      OBLIGATOIRE : GR00T convertit ses actions relatives en
                      positions absolues à partir de cet état ; un état
                      absent ou faux décale tous les mouvements des bras.

        Lève ValueError ou TypeError si une entrée est incomplète ou mal
        formée. Lève RuntimeError si le connecteur n'est pas connecté ou si
        la réponse du serveur est refusée (ValueError quand le refus a lieu
        dès le décodage, voir _dec).
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
        # La décision du modèle a au moins l'âge de l'observation envoyée.
        # L'horodatage est donc pris à la capture, juste avant l'envoi, et non
        # à l'arrivée de la réponse : l'erreur va ainsi dans le sens prudent
        # pour l'échéance de fraîcheur de SetpointFollower (qui fige le robot
        # quand une consigne est trop ancienne).
        t_ns = time.monotonic_ns()
        rep = self._appel("get_action", {"observation": obs, "options": None})
        latence = (time.monotonic_ns() - t_ns) / 1e9
        return self._vers_action(rep, t_ns, latence)

    def _vers_action(self, rep: Any, t_ns: int, latence: float) -> GR00TAction:
        """Vérifie la réponse du serveur et la convertit en GR00TAction.

        Toute réponse inattendue (clé absente, type ou forme imprévus, NaN ou
        inf) lève RuntimeError au lieu de produire une action.
        """
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
            # Des nombres réels seulement : converti en float32, un booléen,
            # une chaîne ou un complexe donnerait un angle dépourvu de sens.
            if brut.dtype.kind not in "iuf":
                raise RuntimeError(f"'{cle}' de type {brut.dtype}, attendu des nombres réels")
            # Piège : 1e39 est fini en float64 (64 bits) mais devient inf en
            # float32. La conversion a donc lieu AVANT le test de finitude,
            # pour que ce cas soit rejeté lui aussi.
            a = brut.astype(np.float32)
            if a.ndim != 3 or a.shape[0] != 1 or a.shape[2] != dim or a.shape[1] <= self._step:
                raise RuntimeError(f"'{cle}' de forme {a.shape}, attendu (1, ≥{self._step + 1}, {dim})")
            if not np.all(np.isfinite(a)):
                raise RuntimeError(f"'{cle}' contient NaN/inf — action rejetée")
            chunk[cle] = a[0]   # sans la dimension de lot : (pas, dim)
        # Seul le pas `step` est exécuté ; le chunk complet reste joint à
        # l'action. Ce format d'action ne comporte ni commande de pince ni
        # score de confiance, d'où gripper_closed=False et confidence=None.
        s = self._step
        angles = np.concatenate([chunk["bras_gauche"][s], chunk["bras_droit"][s]])
        vx, vy, wz = (float(x) for x in chunk["base"][s])
        return GR00TAction(
            target_angles={j: float(x) for j, x in zip(self._arm_joints, angles)},
            gripper_closed=False, confidence=None, timestamp_ns=t_ns,
            base_velocity=(vx, vy, wz), chunk=chunk, latency_s=latence,
        )
