"""
AXIS-OS — VLA Connector
==========================
Interface optionnelle pour brancher un modèle Vision-Language-Action
(style Figure Helix, OpenVLA, RT-2) dans une app Play Store.

Principe validé avec l'utilisateur :
  - L'IA n'est JAMAIS dans l'OS/HAL. Elle est optionnelle, dans le SDK.
  - Une app scriptée (PackageSorting) n'a besoin d'aucun de ces fichiers.
  - Une app qui veut de l'IA connecte IVLAConnector à son modèle de choix.
  - Le HAL/Safety/Watchdog protègent le robot quelle que soit la source
    des commandes (script ou IA) — rien ici ne contourne la sécurité.

Usage typique dans une app :

    from axis_os.store.sdk.ai.connector import OpenVLAConnector

    class MyAIApp(RobotApp):
        def on_start(self, ctx):
            self._vla = OpenVLAConnector()
            return self._vla.connect()

        def run_mission(self, ctx, params):
            image = params.get("camera_frame")
            action = self._vla.predict_action(image, "range le carton rouge")
            # Les angles de l'IA passent quand même par le Safety Guard
            ctx.move_arms(action.to_commands(arm_joint_ids))

Licence : Apache 2.0 (SDK public)
"""

from __future__ import annotations
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from axis_os.hal.core.types import ActuatorCommand, ControlMode


@dataclass
class VLAAction:
    """
    Résultat générique produit par un modèle VLA.

    target_angles : dict {joint_id: angle_normalisé} — pas une liste brute,
                    pour éviter toute ambiguïté sur l'ordre des joints.
    gripper_closed : consigne de préhension simple (ouvert/fermé).
    confidence     : score de confiance du modèle, si disponible (0.0-1.0).
    """
    target_angles:  Dict[str, float] = field(default_factory=dict)
    gripper_closed: bool             = False
    confidence:     Optional[float]  = None

    def to_commands(self, velocity: float = 1.0) -> Dict[str, ActuatorCommand]:
        """
        Convertit l'action IA en commandes HAL normalisées.
        Ces commandes passeront quand même par le JointSafetyGuard —
        une hallucination du modèle ne peut pas casser le robot.
        """
        return {
            joint_id: ActuatorCommand(
                mode=ControlMode.POSITION,
                position=angle,
                velocity=velocity,
            )
            for joint_id, angle in self.target_angles.items()
        }


class IVLAConnector(ABC):
    """
    Contrat universel pour tout modèle Vision-Language-Action.
    Une app peut brancher n'importe quel modèle (OpenVLA, Helix, GPT-4V,
    Llama local) tant qu'il implémente ce contrat.
    """

    @abstractmethod
    def connect(self) -> bool:
        """Initialise le modèle (chargement RAM/VRAM ou connexion API)."""
        ...

    @abstractmethod
    def disconnect(self) -> None:
        """Libère les ressources du modèle."""
        ...

    @abstractmethod
    def is_connected(self) -> bool:
        """Health check — le modèle est-il prêt à répondre ?"""
        ...

    @abstractmethod
    def predict_action(self, image: Any, instruction: str) -> VLAAction:
        """
        Prend une image (caméra) et une consigne en langage naturel.
        Retourne l'action physique à effectuer.
        Lève RuntimeError si le connecteur n'est pas connecté.
        """
        ...


class OpenVLAConnector(IVLAConnector):
    """
    Connecteur pour OpenVLA (ou tout modèle compatible).

    Mode stub (par défaut) : aucune dépendance externe, retourne une
    action neutre déterministe — permet de développer et tester une
    app IA sans avoir le vrai modèle installé.

    Mode production : en Sprint 4, ce connecteur chargera le vrai
    modèle via transformers/torch. L'interface ne changera pas —
    les apps qui utilisent OpenVLAConnector n'auront rien à modifier.
    """

    def __init__(self, endpoint: str = "local_stub"):
        self._endpoint     = endpoint
        self._connected    = False

    def connect(self) -> bool:
        print(f"[VLAConnector] Connexion à OpenVLA sur '{self._endpoint}'")
        # Sprint 4 : chargement réel du modèle HuggingFace/PyTorch ici.
        self._connected = True
        return True

    def disconnect(self) -> None:
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def predict_action(self, image: Any, instruction: str) -> VLAAction:
        if not self._connected:
            raise RuntimeError(
                "OpenVLAConnector non connecté — appelez connect() d'abord."
            )

        print(f"[OpenVLA] Analyse de l'image pour la consigne : '{instruction}'")

        # Stub déterministe : action neutre, aucun mouvement réel.
        # Remplacé par une vraie inférence modèle en Sprint 4.
        should_grasp = any(
            kw in instruction.lower() for kw in ["attrape", "prends", "saisis", "trie"]
        )

        return VLAAction(
            target_angles={},   # neutre : le stub ne bouge rien par défaut
            gripper_closed=should_grasp,
            confidence=1.0 if should_grasp else 0.5,
        )


class VLACppConnector(IVLAConnector):
    """
    Connecteur pour un serveur vla-server (projet vla.cpp, licence
    Apache 2.0 — https://github.com/VinRobotics/vla.cpp), un moteur
    d'inférence C++ (basé sur llama.cpp) qui fait tourner des modèles
    VLA au format GGUF unifié (SmolVLA, π0, OpenVLA-OFT, GR00T N1.x,
    Evo-1...), avec des benchmarks publiés sur matériel embarqué
    (NVIDIA Jetson AGX Orin, Jetson Orin Nano 8GB) — le même type de
    processeur que le module Jetson Orin NX du Unitree G1 EDU.

    Protocole EXACT (pas une supposition) : ZeroMQ REQ/REP synchrone,
    messages Protobuf définis par le schéma officiel du projet
    (voir axis_os/store/sdk/ai/proto/vla.proto, copié depuis le
    dépôt source et compilé avec protoc — vla_pb2.py généré, ne pas
    éditer à la main).

    Choisi plutôt qu'un chargement direct PyTorch/transformers dans
    ce connecteur pour plusieurs raisons documentées par le projet
    source lui-même : le serveur ne dépend PAS de Python/PyTorch à
    l'inférence (utile pour un déploiement embarqué léger sur le
    Jetson du robot), reste éventuellement RÉSIDENT en mémoire
    (charge le modèle une fois, répond à de nombreuses requêtes),
    et découple explicitement l'inférence (côté serveur) du contrôle
    temps réel (côté client, ici IVLAConnector) — la gestion des
    "chunks" d'action qui se chevauchent, des observations périmées,
    et du safe-holding en cas de retard d'inférence reste la
    responsabilité du CLIENT, exactement le rôle de ce connecteur.

    Portée volontairement limitée à cette version (honnête sur ses
    manques) :
      - request_id : compteur local simple (auto-incrémenté par
        instance), pas de garantie d'unicité inter-process — suffisant
        pour un client à usage unique par robot, à revoir si plusieurs
        clients partagent un même serveur.
      - images : un seul encodage supporté (RGB_U8 brut), pas de
        compression JPEG côté client — le schéma proto le permet
        (Image.Encoding.JPEG), non implémenté ici par simplicité.
      - lang_tokens : `instruction` (texte brut) N'EST PAS tokenisée
        ici — le schéma proto attend des tokens entiers déjà encodés
        par LE MÊME tokenizer que celui utilisé à l'entraînement du
        modèle GGUF chargé côté serveur. Cette classe ne connaît pas
        ce tokenizer (il dépend du modèle exact déployé) — voir
        `lang_tokenizer` au constructeur : à fournir explicitement par
        l'app, ou lève une erreur claire si l'instruction ne peut pas
        être envoyée telle quelle.
      - `state` (proprioception) : liste de floats vide par défaut —
        une app qui veut fournir l'état articulaire réel du robot doit
        le passer explicitement (voir predict_action(..., state=...)),
        ce connecteur ne va pas le lire lui-même dans le HAL (il reste
        volontairement découplé du HAL, comme IVLAConnector l'a
        toujours été).
      - `action_chunk` → `VLAAction.target_angles` : mapping simple
        chunk[i] → nom de joint générique "joint_{i}", car le schéma
        proto ne transporte AUCUN nom de joint (juste un vecteur de
        floats) — c'est à l'app appelante de fournir le VRAI mapping
        vers les joint_id réels du robot (voir `joint_order` au
        constructeur).

    Ne remplace PAS OpenVLAConnector — les deux coexistent, une app
    choisit celui qui correspond à son déploiement (stub de dev vs
    vrai serveur d'inférence local).
    """

    def __init__(
        self,
        endpoint: str = "tcp://127.0.0.1:5555",
        joint_order: Optional[List[str]] = None,
        lang_tokenizer: Optional[Any] = None,
        connect_timeout_ms: int = 5000,
        request_timeout_ms: int = 2000,
    ):
        """
        `endpoint` : adresse ZeroMQ du vla-server (ex: le serveur
        tournant localement sur le Jetson du robot — voir --bind dans
        la documentation vla.cpp).
        `joint_order` : liste ordonnée des joint_id du robot,
        correspondant à l'ordre attendu par le modèle chargé côté
        serveur (le schéma proto ne transporte que des indices, pas
        des noms — voir docstring de classe). Obligatoire pour que
        predict_action() puisse construire un VLAAction utilisable ;
        sans lui, les actions retournées utilisent des noms
        génériques "joint_0", "joint_1"... à mapper soi-même.
        `lang_tokenizer` : callable optionnel `str -> List[int]` pour
        convertir l'instruction en tokens compatibles avec le modèle
        serveur. Sans lui, predict_action() lève une erreur claire
        plutôt que d'envoyer une instruction non tokenisée qui
        produirait un résultat incohérent côté serveur.
        """
        self._endpoint = endpoint
        self._joint_order = joint_order
        self._lang_tokenizer = lang_tokenizer
        self._connect_timeout_ms = connect_timeout_ms
        self._request_timeout_ms = request_timeout_ms
        self._context = None
        self._socket = None
        self._connected = False
        self._next_request_id = 1

    def connect(self) -> bool:
        """
        Ouvre le socket ZeroMQ REQ vers vla-server. N'effectue PAS de
        vraie requête de test ici (REQ/REP n'a pas de "ping" natif
        dans le schéma actuel) — la première vraie erreur de
        connectivité apparaîtra au premier predict_action(), avec un
        message explicite plutôt qu'un échec silencieux ici.
        """
        import zmq
        try:
            self._context = zmq.Context.instance()
            self._socket = self._context.socket(zmq.REQ)
            self._socket.setsockopt(zmq.LINGER, 0)
            self._socket.setsockopt(zmq.RCVTIMEO, self._request_timeout_ms)
            self._socket.setsockopt(zmq.SNDTIMEO, self._connect_timeout_ms)
            self._socket.connect(self._endpoint)
            self._connected = True
            print(f"[VLACppConnector] Connecté (ZeroMQ REQ) à '{self._endpoint}'")
            return True
        except Exception as exc:
            print(f"[VLACppConnector] Échec de connexion à '{self._endpoint}' : {exc}")
            self._connected = False
            return False

    def disconnect(self) -> None:
        if self._socket is not None:
            self._socket.close()
            self._socket = None
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def predict_action(
        self,
        image: Any,
        instruction: str,
        state: Optional[List[float]] = None,
    ) -> VLAAction:
        """
        `image` : soit des bytes RGB bruts déjà au format attendu
        (height*width*3 octets), soit un objet exposant `.tobytes()`,
        `.height`/`.width` (compatible numpy array HxWx3 uint8, sans
        dépendance numpy imposée ici — duck typing volontaire).
        `state` : vecteur de proprioception optionnel (voir docstring
        de classe) — liste vide envoyée si omis.

        Lève RuntimeError si non connecté, ou si aucun lang_tokenizer
        n'a été fourni au constructeur (voir docstring de classe :
        cette classe ne devine jamais un tokenizer par défaut).
        """
        if not self._connected or self._socket is None:
            raise RuntimeError(
                "VLACppConnector non connecté — appelez connect() d'abord."
            )
        if self._lang_tokenizer is None:
            raise RuntimeError(
                "VLACppConnector nécessite un lang_tokenizer (callable "
                "str -> List[int]) fourni au constructeur, correspondant "
                "au tokenizer du modèle chargé côté serveur. Aucune "
                "valeur par défaut n'est devinée : un mauvais tokenizer "
                "produirait une action incohérente sans erreur visible."
            )

        from axis_os.store.sdk.ai.proto import vla_pb2
        import zmq

        height, width, data = self._extract_image_bytes(image)

        req = vla_pb2.PredictRequest()
        req.request_id = self._next_request_id
        self._next_request_id += 1
        req.lang_tokens.extend(self._lang_tokenizer(instruction))
        if state:
            req.state.extend(state)

        img = req.images.add()
        img.encoding = vla_pb2.Image.RGB_U8
        img.height = height
        img.width = width
        img.data = data

        try:
            self._socket.send(req.SerializeToString())
            raw_response = self._socket.recv()
        except zmq.error.Again as exc:
            raise RuntimeError(
                f"Timeout en attente de vla-server sur '{self._endpoint}' "
                f"(> {self._request_timeout_ms}ms) — serveur indisponible "
                f"ou surchargé."
            ) from exc

        resp = vla_pb2.PredictResponse()
        resp.ParseFromString(raw_response)

        if resp.error:
            raise RuntimeError(f"vla-server a retourné une erreur : {resp.error}")

        return self._response_to_action(resp)

    def _extract_image_bytes(self, image: Any) -> tuple:
        """Duck typing volontaire (voir docstring predict_action) —
        pas de dépendance numpy imposée à ce module pour un simple
        objet image."""
        if hasattr(image, "tobytes") and hasattr(image, "shape"):
            h, w = image.shape[0], image.shape[1]
            return h, w, image.tobytes()
        if isinstance(image, dict) and "height" in image and "width" in image and "data" in image:
            return image["height"], image["width"], image["data"]
        raise TypeError(
            "Format d'image non reconnu — attendu un objet avec "
            "'.tobytes()' + '.shape' (ex: numpy array HxWx3), ou un "
            "dict {'height', 'width', 'data'}."
        )

    def _response_to_action(self, resp) -> VLAAction:
        """
        Convertit action_chunk (vecteur plat de floats) en dict
        {joint_id: angle}. Le schéma proto peut transporter PLUSIEURS
        pas de temps (chunk_size > 1) — seul le PREMIER pas est utilisé
        ici (comportement le plus sûr par défaut pour du contrôle
        immédiat) ; le reste du chunk est ignoré volontairement plutôt
        que d'exécuter plusieurs actions à l'aveugle sans que l'app
        appelante l'ait explicitement demandé.
        """
        if resp.chunk_size == 0 or resp.action_dim == 0:
            return VLAAction(target_angles={}, confidence=None)

        first_step = list(resp.action_chunk[:resp.action_dim])

        if self._joint_order is not None:
            if len(first_step) != len(self._joint_order):
                raise RuntimeError(
                    f"vla-server a retourné {len(first_step)} valeurs "
                    f"d'action, mais joint_order en attend "
                    f"{len(self._joint_order)} — le modèle chargé côté "
                    f"serveur ne correspond probablement pas au robot "
                    f"configuré ici."
                )
            target_angles = dict(zip(self._joint_order, first_step))
        else:
            target_angles = {f"joint_{i}": v for i, v in enumerate(first_step)}

        return VLAAction(target_angles=target_angles, confidence=None)
