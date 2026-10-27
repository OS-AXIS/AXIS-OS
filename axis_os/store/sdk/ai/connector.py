"""
AXIS-OS — Connecteurs VLA
=========================
Interface optionnelle pour brancher un modèle VLA (modèle
vision-langage-action : il reçoit une image de caméra et une consigne en
langage naturel, et répond par les mouvements à exécuter), du type Figure
Helix, OpenVLA ou RT-2, dans une app du Play Store (la boutique d'apps
installables sur le robot).

Principes de conception :
  - L'IA ne fait JAMAIS partie de l'OS ni du HAL (la couche d'abstraction
    matérielle, seul chemin vers les moteurs). Elle est optionnelle et vit
    dans le SDK (le kit de développement fourni aux apps).
  - Une app scriptée, comme l'app de tri de colis PackageSorting, n'a
    besoin d'aucun élément de ce fichier.
  - Une app qui veut de l'IA branche le modèle de son choix derrière le
    contrat IVLAConnector.
  - Quelle que soit l'origine des commandes (script ou IA), elles
    traversent les mêmes protections : le HAL, le garde de sécurité des
    articulations (JointSafetyGuard) et le Watchdog (la surveillance temps
    réel des articulations, qui déclenche l'arrêt d'urgence en cas de
    danger). Rien dans ce fichier ne contourne la sécurité.

Contenu du fichier :
  - VLAAction : l'action produite par un modèle, dans un format commun ;
  - IVLAConnector : le contrat que tout connecteur doit respecter ;
  - OpenVLAConnector : un bouchon de développement, sans vrai modèle ;
  - VLACppConnector : le client d'un vrai serveur d'inférence vla.cpp.
Le connecteur du modèle GR00T de NVIDIA (GR00TConnector), cité plus bas
comme référence, se trouve dans gr00t_connector.py.

Usage typique dans une app :

    from axis_os.store.sdk.ai.connector import OpenVLAConnector

    class MyAIApp(RobotApp):
        def on_start(self, ctx):
            self._vla = OpenVLAConnector()
            return self._vla.connect()

        def run_mission(self, ctx, params):
            image = params.get("camera_frame")
            action = self._vla.predict_action(image, "range le carton rouge")
            # Les angles proposés par l'IA passent quand même par le garde
            # de sécurité des articulations avant d'atteindre les moteurs.
            ctx.move_arms(action.to_commands())

Licence : Apache 2.0 (SDK public)
"""

from __future__ import annotations
import math
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from axis_os.hal.core.types import ActuatorCommand, ControlMode, ORIGINE_IA


@dataclass
class VLAAction:
    """
    Action produite par un modèle VLA, dans un format commun à tous les
    connecteurs.

    target_angles  : dict {joint_id: angle_normalisé}. Un dict plutôt
                     qu'une liste brute, pour qu'il n'y ait aucune
                     ambiguïté sur l'ordre des articulations.
    gripper_closed : consigne de préhension simple (pince ouverte ou
                     fermée).
    confidence     : score de confiance du modèle, entre 0.0 et 1.0, s'il
                     en fournit un.
    timestamp_ns   : instant (horloge time.monotonic_ns, qui ne recule
                     jamais) où la POLITIQUE (le modèle qui décide des
                     mouvements) a produit cette action, et non l'instant
                     où on la reçoit.
                     Cette distinction est le fondement de l'échéance de
                     fraîcheur de SetpointFollower (le suiveur de
                     consigne, dans axis_os/services/policy/), qui cesse
                     de suivre une consigne trop ancienne. Horodater à la
                     réception mesurerait la latence réseau au lieu de
                     l'âge réel de la décision : une politique figée dont
                     les paquets arrivent encore passerait pour fraîche.
                     0 signifie « non horodatée » : le suiveur retombe
                     alors sur l'instant de réception, avec l'imprécision
                     décrite ci-dessus.
    """
    target_angles:  Dict[str, float] = field(default_factory=dict)
    gripper_closed: bool             = False
    confidence:     Optional[float]  = None
    timestamp_ns:   int              = 0

    def move_to_pose(self, ctx, duration_s: float = 2.0, step_s: float = 0.05) -> bool:
        """Rejoint target_angles par ctx.move_to_pose(), avec la marque
        ORIGINE_IA : l'OS exige alors ACTUATORS_AI_DRIVEN, comme pour
        to_commands()."""
        return ctx.move_to_pose(dict(self.target_angles), duration_s, step_s,
                                origine=ORIGINE_IA)

    def to_commands(self, velocity: float = 1.0) -> Dict[str, ActuatorCommand]:
        """
        Convertit l'action en commandes HAL normalisées, en mode position,
        une par articulation de target_angles ; velocity est appliquée
        telle quelle à chacune.

        Ces commandes passeront quand même par JointSafetyGuard (le garde
        qui vérifie chaque commande avant de l'envoyer au moteur) : une
        hallucination du modèle (une sortie aberrante présentée comme
        valable) ne peut pas casser le robot.
        """
        return {
            joint_id: ActuatorCommand(
                mode=ControlMode.POSITION,
                position=angle,
                velocity=velocity,
                origine=ORIGINE_IA,
            )
            for joint_id, angle in self.target_angles.items()
        }


class IVLAConnector(ABC):
    """
    Contrat commun à tout connecteur de modèle VLA.

    Une app peut brancher n'importe quel modèle (OpenVLA, Helix, GPT-4V,
    un Llama exécuté localement...) tant que son connecteur implémente ces
    quatre méthodes.
    """

    @abstractmethod
    def connect(self) -> bool:
        """
        Prépare le modèle : chargement en mémoire (RAM, ou VRAM, la
        mémoire de la carte graphique) ou connexion à une API distante (un
        service de modèle joint par le réseau).
        Retourne True si le modèle est prêt.
        """
        ...

    @abstractmethod
    def disconnect(self) -> None:
        """Libère les ressources du modèle."""
        ...

    @abstractmethod
    def is_connected(self) -> bool:
        """Vérification de santé : le modèle est-il prêt à répondre ?"""
        ...

    @abstractmethod
    def predict_action(self, image: Any, instruction: str) -> VLAAction:
        """
        Reçoit une image de caméra et une consigne en langage naturel, et
        retourne l'action physique à effectuer.

        Lève RuntimeError si le connecteur n'est pas connecté.
        """
        ...


class OpenVLAConnector(IVLAConnector):
    """
    Connecteur pour OpenVLA (un modèle VLA open source) ou tout modèle
    compatible.

    Mode bouchon (« stub », par défaut) : aucune dépendance externe. Le
    connecteur retourne une action neutre et déterministe, ce qui permet
    de développer et de tester une app IA sans avoir installé le vrai
    modèle.

    Mode production (prévu, pas encore codé) : ce connecteur chargera le
    vrai modèle avec les bibliothèques transformers et PyTorch.
    L'interface ne changera pas : les apps qui utilisent OpenVLAConnector
    n'auront rien à modifier.
    """

    def __init__(self, endpoint: str = "local_stub"):
        self._endpoint     = endpoint
        self._connected    = False

    def connect(self) -> bool:
        print(f"[VLAConnector] Connexion à OpenVLA sur '{self._endpoint}'")
        # Prévu : le vrai modèle sera chargé ici, téléchargé depuis
        # HuggingFace (la plateforme de partage de modèles) et exécuté avec
        # PyTorch. En attendant, le bouchon se déclare simplement connecté.
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

        # Bouchon déterministe : action neutre, aucun mouvement réel. Il sera
        # remplacé par une vraie inférence du modèle (le calcul de sa
        # réponse). Seule la pince réagit : elle se ferme si la consigne, mise
        # en minuscules, contient l'un des mots-clés ci-dessous (attrape,
        # prends, saisis, trie) ; la confiance vaut alors 1.0, sinon 0.5.
        should_grasp = any(
            kw in instruction.lower() for kw in ["attrape", "prends", "saisis", "trie"]
        )

        return VLAAction(
            target_angles={},   # neutre : le bouchon ne bouge rien
            gripper_closed=should_grasp,
            confidence=1.0 if should_grasp else 0.5,
            timestamp_ns=time.monotonic_ns(),
        )


class VLACppConnector(IVLAConnector):
    """
    Connecteur vers un serveur vla-server, du projet vla.cpp (licence
    Apache 2.0, https://github.com/VinRobotics/vla.cpp).

    vla.cpp est un moteur d'inférence (le programme qui exécute un modèle
    déjà entraîné) écrit en C++ et basé sur llama.cpp (un moteur du même
    genre, conçu pour les modèles de langage). Il fait tourner des
    modèles VLA au format GGUF unifié (un format de fichier de poids de
    modèle issu de llama.cpp) : SmolVLA, π0, OpenVLA-OFT, GR00T N1.x,
    Evo-1... Le projet publie des mesures de performance sur du matériel
    embarqué (NVIDIA Jetson AGX Orin, Jetson Orin Nano 8GB), le même type
    de processeur que le module Jetson Orin NX du Unitree G1 EDU (le robot
    humanoïde visé par AXIS-OS).

    Protocole : c'est le protocole EXACT du projet, pas une supposition.
    ZeroMQ (une bibliothèque de messagerie réseau) en mode REQ/REP
    synchrone (une requête, puis sa réponse, chacune à son tour), avec des
    messages Protobuf (un format binaire de messages décrit par un schéma)
    définis par le schéma officiel du projet. Ce schéma a été copié depuis
    le dépôt source dans axis_os/store/sdk/ai/proto/vla.proto et compilé
    avec protoc (le compilateur Protobuf) ; vla_pb2.py est généré, il ne
    faut pas l'éditer à la main.

    Pourquoi passer par ce serveur plutôt que de charger directement le
    modèle avec PyTorch/transformers dans ce connecteur ? Pour plusieurs
    raisons documentées par le projet source lui-même :
      - le serveur ne dépend ni de Python ni de PyTorch pendant
        l'inférence, ce qui convient à un déploiement embarqué léger sur
        le Jetson du robot ;
      - il peut rester résident en mémoire : il charge le modèle une fois,
        puis répond à de nombreuses requêtes ;
      - il sépare explicitement l'inférence (côté serveur) du contrôle
        temps réel (côté client, ici IVLAConnector). La gestion des
        « chunks » d'action (des séquences de plusieurs pas de mouvement)
        qui se chevauchent, des observations périmées et du maintien en
        position sûre (« safe-holding ») quand l'inférence prend du retard
        reste à la charge du CLIENT : c'est exactement le rôle de ce
        connecteur.

    Limites connues de cette version, assumées et volontaires :
      - request_id : simple compteur local, incrémenté par instance, sans
        garantie d'unicité entre processus. Suffisant pour un seul client
        par robot ; à revoir si plusieurs clients partagent le même
        serveur.
      - images : un seul encodage pris en charge (RGB_U8, des pixels RGB
        bruts sur un octet par canal), sans compression JPEG côté client.
        Le schéma proto la permet (Image.Encoding.JPEG) ; elle n'est pas
        implémentée ici, par simplicité.
      - lang_tokens : l'instruction (texte brut) n'est pas convertie en
        tokens ici. Le schéma proto attend des tokens (les entiers qui
        représentent le texte pour un modèle de langage) déjà produits par
        le MÊME tokenizer (l'outil de découpage du texte en tokens) que
        celui utilisé pour entraîner le modèle GGUF chargé côté serveur.
        Cette classe ne connaît pas ce tokenizer, qui dépend du modèle
        exact déployé : l'app doit le fournir explicitement via le
        paramètre lang_tokenizer du constructeur, sinon predict_action()
        lève une erreur claire au lieu d'envoyer l'instruction telle
        quelle.
      - state (la proprioception, c'est-à-dire l'état articulaire mesuré
        du robot) : liste de floats vide par défaut. Une app qui veut
        fournir l'état articulaire réel doit le passer explicitement
        (predict_action(..., state=...)). Ce connecteur ne va pas le lire
        lui-même dans le HAL : il reste volontairement découplé du HAL,
        comme IVLAConnector l'a toujours été.
      - action_chunk → VLAAction.target_angles : correspondance simple,
        chunk[i] → nom d'articulation générique "joint_{i}", car le schéma
        proto ne transporte AUCUN nom d'articulation (seulement un vecteur
        de floats). C'est à l'app appelante de fournir la vraie
        correspondance avec les joint_id du robot (paramètre joint_order
        du constructeur).

    Ne remplace pas OpenVLAConnector : les deux coexistent, et une app
    choisit celui qui correspond à son déploiement (bouchon de
    développement, ou vrai serveur d'inférence local).
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
        endpoint : adresse ZeroMQ du vla-server, par exemple celle du
            serveur qui tourne localement sur le Jetson du robot (voir
            l'option --bind dans la documentation de vla.cpp).
        joint_order : liste ordonnée des joint_id du robot, dans l'ordre
            attendu par le modèle chargé côté serveur (le schéma proto ne
            transporte que des indices, le rang de chaque valeur dans un
            vecteur, et pas des noms : voir la docstring de la classe).
            Indispensable pour que predict_action() construise une
            VLAAction directement utilisable ; sans lui, les actions
            retournées portent des noms génériques "joint_0", "joint_1"...,
            à faire correspondre soi-même.
        lang_tokenizer : fonction optionnelle str -> List[int] qui
            convertit l'instruction en tokens compatibles avec le modèle du
            serveur. Sans elle, predict_action() lève une erreur claire
            plutôt que d'envoyer une instruction non convertie, qui
            produirait un résultat incohérent côté serveur.
        connect_timeout_ms : délai maximal d'envoi d'une requête, en
            millisecondes (option SNDTIMEO de ZeroMQ). Malgré son nom, il
            ne borne pas l'établissement de la connexion, mais l'envoi.
        request_timeout_ms : délai maximal d'attente de la réponse, en
            millisecondes (option RCVTIMEO de ZeroMQ).
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
        Ouvre la socket ZeroMQ REQ (le point de communication côté client)
        vers vla-server, sans envoyer de requête de test : le mode REQ/REP
        n'a pas de « ping » natif dans le schéma actuel. La première vraie
        erreur de connectivité apparaîtra donc au premier predict_action(),
        avec un message explicite, plutôt qu'un échec silencieux ici.

        Retourne False, sans lever d'exception, si la socket ne peut pas
        être ouverte. Seule l'absence de la bibliothèque zmq lève une
        erreur (ImportError), car son import est fait avant le bloc try.
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
        image : soit un objet qui expose .tobytes() et .shape, par exemple
            un tableau numpy (numpy est la bibliothèque de calcul numérique
            de Python) de forme HxWx3, c'est-à-dire hauteur × largeur × 3
            canaux de couleur, en entiers uint8 d'un octet chacun ; soit un
            dict {'height', 'width', 'data'} dont 'data' contient les
            octets RGB bruts (height*width*3 octets). Ce duck typing (on
            regarde ce que l'objet sait faire, pas son type) est
            volontaire : ce module n'impose pas de dépendance à numpy.
        instruction : la consigne en langage naturel, convertie en tokens
            par le lang_tokenizer fourni au constructeur.
        state : vecteur de proprioception optionnel (voir la docstring de
            la classe) ; une liste vide est envoyée s'il est omis.

        Lève RuntimeError si le connecteur n'est pas connecté, ou si aucun
        lang_tokenizer n'a été fourni au constructeur (cette classe ne
        devine jamais un tokenizer par défaut, voir la docstring de la
        classe). Lève aussi RuntimeError si le serveur ne répond pas à
        temps, renvoie une erreur ou une action invalide, et TypeError si
        le format de l'image n'est pas reconnu.
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

        # Horodatage « à la capture », c'est-à-dire juste avant l'envoi de la
        # requête, comme dans GR00TConnector : la décision a au moins cet âge.
        # Un audit de sécurité (2026-10-10) a relevé que l'ancienne version
        # horodatait à la réception, ce qui rajeunissait l'action de toute la
        # latence réseau : une erreur dans le mauvais sens pour l'échéance de
        # fraîcheur de SetpointFollower. Vérifié par
        # test/test_suiveur_non_fini.py (serveur volontairement lent).
        t_capture_ns = time.monotonic_ns()
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

        return self._response_to_action(resp, t_capture_ns)

    def _extract_image_bytes(self, image: Any) -> tuple:
        """
        Retourne (hauteur, largeur, octets) à partir de l'image reçue.

        Duck typing volontaire (voir la docstring de predict_action) : ce
        module n'impose pas de dépendance à numpy pour un simple objet
        image. Lève TypeError si le format n'est pas reconnu.
        """
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

    def _response_to_action(self, resp, t_capture_ns: Optional[int] = None) -> VLAAction:
        """
        Convertit action_chunk (vecteur plat de floats) en dict
        {joint_id: angle}.

        Le schéma proto peut transporter PLUSIEURS pas de temps
        (chunk_size > 1). Seul le premier pas est utilisé ici : c'est le
        comportement le plus sûr par défaut pour du contrôle immédiat. Le
        reste du chunk est ignoré volontairement, plutôt que d'exécuter
        plusieurs actions à l'aveugle sans que l'app appelante l'ait
        explicitement demandé.

        Une réponse vide (chunk_size ou action_dim à 0) donne une action
        neutre. Lève RuntimeError si le premier pas est incomplet, s'il
        contient NaN (une valeur numérique invalide, « pas un nombre ») ou
        inf (l'infini), ou si sa longueur ne correspond pas à joint_order.
        t_capture_ns : horodatage pris avant l'envoi de la requête (voir
        predict_action) ; à défaut, l'instant présent.
        """
        t_ns = t_capture_ns if t_capture_ns is not None else time.monotonic_ns()
        if resp.chunk_size == 0 or resp.action_dim == 0:
            return VLAAction(target_angles={}, confidence=None, timestamp_ns=t_ns)

        first_step = list(resp.action_chunk[:resp.action_dim])
        if len(first_step) != resp.action_dim:
            raise RuntimeError(
                f"vla-server annonce action_dim={resp.action_dim} mais n'envoie "
                f"que {len(first_step)} valeurs — action rejetée"
            )
        # NaN et inf sont rejetés ici, comme dans GR00TConnector. Le même
        # audit de sécurité a montré qu'une cible NaN faisait dériver
        # l'articulation jusqu'à sa butée dans SetpointFollower : un pas de
        # -budget (le déplacement maximal autorisé par cycle de contrôle) à
        # chaque cycle, que le garde de sécurité ne refuse pas puisque
        # chaque pas reste fini et petit. Détail dans l'en-tête de
        # axis_os/services/policy/setpoint_follower.py.
        if not all(math.isfinite(v) for v in first_step):
            raise RuntimeError("vla-server a retourné une action contenant "
                               "NaN/inf — action rejetée")

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

        # Horodaté à la capture (avant l'envoi de la requête) : le protocole
        # vla-server ne transporte pas l'instant de l'inférence, mais l'image
        # a forcément été prise avant. L'action paraît donc plus VIEILLE
        # qu'elle ne l'est, jamais plus jeune : l'erreur va dans le sens
        # prudent pour l'échéance de fraîcheur. Avant l'audit de sécurité du
        # 2026-10-10, l'horodatage était pris à la réception, ce qui rognait
        # la marge de fraîcheur de toute la latence réseau.
        return VLAAction(target_angles=target_angles, confidence=None,
                         timestamp_ns=t_ns)
