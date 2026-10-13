"""
AXIS-OS — Détecteur à vocabulaire ouvert, basé sur OWL-ViT (Google).

OWL-ViT est un modèle de vision publié par Google : il repère dans une
image des objets décrits par un texte libre (« vocabulaire ouvert »), sans
liste de classes fixée à l'avance. Ce fichier en fait la première
implémentation réelle d'IObjectDetector, le contrat des détecteurs défini
dans service.py. Le service de perception ne change pas d'une ligne : c'est
tout l'intérêt de la séparation entre le code (stable, versionné avec
l'OS) et le modèle (remplaçable), décidée au §6.4 du document
d'architecture (docs/architecture_services.md).

Pourquoi OWL-ViT, honnêtement : ce n'est PAS le détecteur le plus précis
disponible. Parmi les autres détecteurs à vocabulaire ouvert, Grounding
DINO fait mieux et YOLO-World va plus vite. Il est retenu pour trois
raisons qui n'ont rien à voir avec ses performances brutes :

  1. Sa licence, Apache 2.0, qui est permissive. L'écosystème Ultralytics
     (détecteurs YOLO26, YOLOE) est sous AGPL-3.0, une licence qui oblige
     à publier sous la même licence le code source de tout logiciel qui
     l'intègre dès qu'il est distribué, ou même seulement proposé à
     travers un réseau. Cela complique tout déploiement commercial fermé,
     pour AXIS-OS comme pour les développeurs tiers du Play Store (la
     boutique d'apps d'AXIS-OS).
  2. Une installation immédiate via `transformers`, la bibliothèque de
     modèles de Hugging Face (l'entreprise dont le site, huggingface.co,
     héberge les modèles publiés) : la chaîne complète se valide donc
     tout de suite sur le GPU (carte graphique de calcul) d'une machine
     de développement.
  3. NanoOWL, la version optimisée par NVIDIA pour ses cartes embarquées
     Jetson, EST ce modèle. Passer au matériel embarqué sera un
     changement d'implémentation, pas de famille de modèle : donc ni
     réentraînement ni revalidation complète.

Un vocabulaire réellement ouvert : le modèle accepte des descriptions
libres (« un verre », « une boîte en carton ») sans avoir été entraîné sur
ces classes. C'est ce qui permet à une app tierce de chercher un objet que
personne n'avait prévu. Sans cette propriété, chaque nouvel objet
exigerait un réentraînement, ce qui ne passe pas à l'échelle d'un
catalogue d'apps.
"""
from __future__ import annotations

import re
import time
from typing import List, Optional

from axis_os.hal.core.types import CameraFrame
from axis_os.services.perception.service import Detection, IObjectDetector

# Poids épinglés : les poids (les paramètres appris du modèle) sont figés
# à une version précise. Pourquoi : un audit de sécurité du 10 octobre
# 2026 a relevé que, sans révision précisée, from_pretrained (la fonction
# de transformers qui télécharge le modèle) chargeait la dernière version
# publiée. Le comportement de détection, donc ce que le robot voit et ce
# qu'il prend pour un obstacle ou un carton, pouvait alors changer sans
# qu'une ligne d'AXIS-OS ne bouge, et un dépôt de modèle compromis aurait
# été chargé tel quel.
# Révision retenue : le commit (version enregistrée du dépôt) du
# 7 septembre 2023 de google/owlvit-base-patch32, celui qui ajoute les
# poids au format safetensors, un format qui ne contient que des données
# (relevé par le mainteneur sur huggingface.co, le site qui héberge le
# modèle). La même valeur figure dans deploy/machine_gpu/installer.sh,
# qui pré-télécharge les poids ; test/test_deploiement.py vérifie que les
# deux concordent, et test/test_owlvit_epinglage.py vérifie le refus de
# charger des poids non épinglés. Changer de révision impose de revalider
# en simulation (voir docs/validation_perception.md, « Limites connues »).
MODELE_OWLVIT = "google/owlvit-base-patch32"
OWLVIT_REVISION = "85fa9bec488a03a73b7f29206b1550551d0a7556"
# Une révision épinglée est un hash de commit complet (empreinte unique
# d'une version, 40 caractères hexadécimaux). Un nom de branche (« main »)
# ou d'étiquette peut être déplacé vers d'autres poids : il ne fige rien.
_HASH_COMMIT = re.compile(r"^[0-9a-f]{40}$")
# Marqueur qui distingue « aucune révision donnée » (le modèle par défaut
# prend alors OWLVIT_REVISION) d'une révision None passée explicitement.
_PAR_DEFAUT = object()

# Requêtes par défaut quand l'appelant ne précise rien (describe_scene()).
# Volontairement génériques : ce sont des amorces, pas une liste fermée
# — le modèle accepte n'importe quelle description à l'appel suivant.
_DEFAULT_QUERIES = [
    "a person", "a box", "a cup", "a bottle", "a chair",
    "a table", "a door", "a tool",
]


class OwlViTDetector(IObjectDetector):
    """Détecteur à vocabulaire ouvert via OWL-ViT.

    Exemple :
        detector = OwlViTDetector(threshold=0.1)
        detector.load()                       # coûteux, une seule fois
        svc = PerceptionService(camera, tree, detector)
    """

    def __init__(
        self,
        model_name: str = MODELE_OWLVIT,
        revision=_PAR_DEFAUT,
        threshold: float = 0.1,
        device: Optional[str] = None,
        max_detections: int = 20,
        autoriser_non_epingle_SIMULATION: bool = False,
    ):
        """
        model_name : identifiant du modèle sur Hugging Face (par défaut
            MODELE_OWLVIT).
        revision : hash de commit Hugging Face (40 caractères) des POIDS.
            Par défaut : OWLVIT_REVISION pour le modèle par défaut. Un
            autre modèle n'a PAS de révision par défaut (celle-ci ne lui
            correspond pas) et doit en recevoir une. Sans révision
            épinglée (None, nom de branche), load() REFUSE de charger.
        threshold : seuil de confiance. Bas par défaut (0.1), car les
            scores d'OWL-ViT ne sont PAS calibrés comme ceux d'un
            détecteur à classes fermées : un 0.15 peut être une bonne
            détection. À régler sur des images réelles, jamais deviné.
        device : None (défaut) choisit au chargement "cuda" (le GPU
            NVIDIA) s'il est disponible, sinon "cpu" (le processeur
            central). Le CPU fonctionne, mais compte en secondes par
            image, pas en millisecondes.
        max_detections : nombre maximal de détections rendues par image,
            les plus confiantes d'abord.
        autoriser_non_epingle_SIMULATION : lève ce refus, pour essayer
            d'autres poids EN SIMULATION seulement ; un avertissement est
            alors affiché à chaque chargement. Jamais sur un vrai robot.

        Les poids sont toujours chargés au format safetensors
        (use_safetensors=True). Pourquoi : un fichier .bin de PyTorch est
        un pickle (le format de sérialisation natif de Python), qui peut
        exécuter du code au moment du chargement.
        """
        if revision is _PAR_DEFAUT:
            revision = OWLVIT_REVISION if model_name == MODELE_OWLVIT else None
        self._model_name = model_name
        self._revision = revision
        self._autoriser_non_epingle = autoriser_non_epingle_SIMULATION
        self._threshold = threshold
        self._device = device
        self._max_detections = max_detections

        self._model = None
        self._processor = None
        self._load_error: Optional[str] = None
        self._last_inference_error: Optional[str] = None
        self._inferences = 0
        self._total_ms = 0.0

    # ---- chargement ------------------------------------------------------

    def load(self) -> bool:
        """Charge le modèle ; rend True s'il est prêt.

        Séparé du constructeur DÉLIBÉRÉMENT : l'opération prend plusieurs
        secondes et télécharge des poids au premier appel. Un constructeur
        qui ferait ce travail empêcherait de construire un service pour en
        inspecter la configuration sans payer le chargement.

        En cas d'échec (poids non épinglés, dépendances absentes, réseau,
        mémoire), rend False et garde la raison dans stats["load_error"].
        """
        if self._model is not None:
            return True
        epingle = isinstance(self._revision, str) and bool(_HASH_COMMIT.match(self._revision))
        if not epingle and not self._autoriser_non_epingle:
            # Refus AVANT tout téléchargement, car des poids non figés
            # peuvent changer ce que voit le robot sans qu'AXIS-OS change.
            self._load_error = (
                f"poids NON épinglés (revision={self._revision!r}) — chargement "
                f"refusé. Donner un hash de commit complet (40 caractères), ou "
                f"autoriser_non_epingle_SIMULATION=True en simulation seulement.")
            print(f"  [owlvit] {self._load_error}")
            return False
        try:
            import torch
            from transformers import OwlViTForObjectDetection, OwlViTProcessor
        except ImportError as exc:
            self._load_error = (
                f"dépendances absentes ({exc}) — "
                f"`pip install transformers torch pillow`"
            )
            print(f"  [owlvit] {self._load_error}")
            return False

        try:
            if self._device is None:
                self._device = "cuda" if torch.cuda.is_available() else "cpu"
            kw = {"revision": self._revision} if self._revision else {}
            self._processor = OwlViTProcessor.from_pretrained(
                self._model_name, **kw)
            # use_safetensors=True : jamais le fichier .bin (un pickle, qui
            # peut exécuter du code). Si la révision n'a pas de version
            # safetensors, le chargement échoue, et bruyamment.
            self._model = OwlViTForObjectDetection.from_pretrained(
                self._model_name, use_safetensors=True, **kw).to(self._device)
            self._model.eval()
        except Exception as exc:        # noqa: BLE001 — réseau, mémoire...
            self._load_error = f"chargement impossible : {exc}"
            print(f"  [owlvit] {self._load_error}")
            self._model = None
            return False

        if self._device == "cpu":
            print("  [owlvit] AVERTISSEMENT : exécution sur CPU — compter "
                  "des SECONDES par image, pas des millisecondes. "
                  "Inutilisable dans une boucle de contrôle.")
        print(f"  [owlvit] {self._model_name} chargé sur {self._device} "
              f"(révision {self._revision or 'NON ÉPINGLÉE'})")
        if not epingle:
            print("  [owlvit] AVERTISSEMENT (SIMULATION) : poids NON épinglés "
                  f"(revision={self._revision!r}) — le comportement de détection "
                  "peut changer sans modification d'AXIS-OS. Jamais sur un vrai "
                  "robot.")
        return True

    @property
    def is_loaded(self) -> bool:
        return self._model is not None

    @property
    def is_open_vocabulary(self) -> bool:
        return True

    # ---- détection -------------------------------------------------------

    def detect(self, frame: CameraFrame,
               query: Optional[str] = None) -> List[Detection]:
        """Détecte dans `frame`. `query` est une description libre ; sans
        elle, les requêtes génériques de _DEFAULT_QUERIES sont utilisées.

        Rend une liste VIDE si le modèle n'est pas chargé, plutôt que de
        lever une exception : le service de perception traite déjà
        l'absence de détection comme un cas normal, et une exception ici
        ferait échouer une mission pour une raison d'infrastructure.
        Même chose pour une image illisible ou d'un encodage autre que
        rgb8 (couleur rouge-vert-bleu, 8 bits par composante), et pour
        une inférence (le passage de l'image dans le modèle) qui échoue :
        dans ce dernier cas, l'erreur est mémorisée dans
        stats["last_inference_error"] pour que le service de perception
        puisse signaler la panne.
        """
        if self._model is None or frame is None:
            return []

        try:
            import torch
            from PIL import Image
        except ImportError:
            return []

        if frame.encoding != "rgb8":
            # Ne pas deviner : une image de profondeur (encodage depth16,
            # 16 bits par pixel) interprétée comme une image couleur RGB
            # produirait des détections absurdes présentées comme
            # valides.
            print(f"  [owlvit] encodage '{frame.encoding}' non supporté "
                  f"(rgb8 attendu) — aucune détection.")
            return []

        try:
            image = Image.frombytes(
                "RGB", (frame.width, frame.height), frame.data)
        except Exception as exc:        # noqa: BLE001
            print(f"  [owlvit] image illisible : {exc}")
            return []

        queries = [query] if query else list(_DEFAULT_QUERIES)

        t0 = time.perf_counter()
        try:
            inputs = self._processor(
                text=[queries], images=image, return_tensors="pt"
            ).to(self._device)
            with torch.no_grad():
                outputs = self._model(**inputs)

            target_sizes = torch.tensor(
                [[frame.height, frame.width]]).to(self._device)
            results = self._post_process(outputs, target_sizes, queries)[0]
        except Exception as exc:        # noqa: BLE001
            # L'erreur est mémorisée, pas seulement affichée : une
            # inférence qui échoue en silence rend zéro détection, ce qui
            # est INDISCERNABLE d'une scène vide. C'est exactement ce qui
            # s'est produit lors du premier essai avec transformers 5.x :
            # find() rendait ok=True avec 0 détection alors que le
            # post-traitement (voir _post_process) plantait à chaque image.
            self._last_inference_error = str(exc)
            print(f"  [owlvit] inférence échouée : {exc}")
            return []
        finally:
            # Durée et nombre d'inférences, comptés même en cas d'échec
            # (moyenne dans stats["mean_ms"]).
            self._total_ms += (time.perf_counter() - t0) * 1000
            self._inferences += 1

        detections: List[Detection] = []
        # Selon la version de transformers, `labels` contient des indices
        # (entiers) dans la liste des requêtes, ou directement le texte ;
        # `text_labels` existe dans les versions 5.x. On traite tous les
        # cas sans rien supposer.
        labels = results.get("text_labels", results.get("labels"))
        for score, raw_label, box in zip(
                results["scores"], labels, results["boxes"]):
            x0, y0, x1, y1 = (float(v) for v in box.tolist())
            if isinstance(raw_label, str):
                label = raw_label
            else:
                idx = int(raw_label)
                label = queries[idx] if idx < len(queries) else str(idx)
            detections.append(Detection(
                label=label,
                confidence=float(score),
                # OWL-ViT rend deux coins opposés (x0, y0, x1, y1) ;
                # Detection attend (x, y, largeur, hauteur). Confondre les
                # deux décale le centre de la boîte, donc la direction
                # calculée, sans qu'aucune vérification ne s'en aperçoive.
                bbox=(int(x0), int(y0), int(x1 - x0), int(y1 - y0)),
                # Horodatage de l'IMAGE, pas de l'inférence : la scène
                # date du moment de la capture.
                timestamp_ns=frame.timestamp_ns,
            ))

        detections.sort(key=lambda d: -d.confidence)
        return detections[:self._max_detections]

    def _post_process(self, outputs, target_sizes, queries):
        """Post-traitement (conversion de la sortie brute du modèle en
        scores, labels et boîtes), quelle que soit la version de
        transformers.

        Constaté en exécution réelle avec transformers 5.17 :
        `post_process_object_detection` a migré du Processor (l'objet qui
        prépare texte et image pour le modèle) vers l'ImageProcessor (sa
        partie consacrée à l'image), et le Processor expose désormais
        `post_process_grounded_object_detection`, variante dédiée aux
        modèles à vocabulaire ouvert.

        On essaie les trois emplacements plutôt que d'épingler une
        version : ce détecteur doit survivre à une mise à jour de
        transformers sans casser la perception du robot. L'ordre suit la
        préférence : la variante « grounded » (rattachée aux textes de la
        requête) d'abord, puisque c'est celle qui correspond au
        vocabulaire ouvert. Lève RuntimeError si aucune n'existe.
        """
        # 1. Variante moderne, dédiée au vocabulaire ouvert.
        fn = getattr(self._processor,
                     "post_process_grounded_object_detection", None)
        if fn is not None:
            try:
                return fn(outputs=outputs, threshold=self._threshold,
                          target_sizes=target_sizes, text_labels=[queries])
            except TypeError:
                # La signature varie selon la version : certaines
                # n'acceptent pas text_labels.
                return fn(outputs=outputs, threshold=self._threshold,
                          target_sizes=target_sizes)

        # 2. Emplacement historique, sur le Processor.
        fn = getattr(self._processor, "post_process_object_detection", None)
        if fn is not None:
            return fn(outputs=outputs, threshold=self._threshold,
                      target_sizes=target_sizes)

        # 3. transformers 5.x : déplacé sur l'ImageProcessor.
        image_proc = getattr(self._processor, "image_processor", None)
        fn = getattr(image_proc, "post_process_object_detection", None)
        if fn is not None:
            return fn(outputs=outputs, threshold=self._threshold,
                      target_sizes=target_sizes)

        raise RuntimeError(
            "aucune méthode de post-traitement trouvée sur le processor "
            f"({type(self._processor).__name__}) ni sur son image_processor "
            "— version de transformers incompatible."
        )

    @property
    def stats(self) -> dict:
        """État et mesures du détecteur : modèle, révision des poids,
        matériel, chargement, nombre d'inférences, durée moyenne en
        millisecondes, dernières erreurs de chargement et d'inférence.
        Le service de perception lit "last_inference_error" pour
        distinguer une panne d'une scène vide."""
        return {
            "model": self._model_name,
            "revision": self._revision or "NON ÉPINGLÉE",
            "device": self._device,
            "loaded": self.is_loaded,
            "inferences": self._inferences,
            "mean_ms": (self._total_ms / self._inferences
                        if self._inferences else 0.0),
            "load_error": self._load_error,
            "last_inference_error": self._last_inference_error,
        }
