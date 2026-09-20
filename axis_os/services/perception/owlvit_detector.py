"""
AXIS-OS — Détecteur à vocabulaire ouvert, basé sur OWL-ViT (Google).

Première implémentation réelle d'IObjectDetector. Le service de
perception ne change pas d'une ligne : c'est tout l'intérêt de la
séparation code/modèle décidée en §6.4 du document d'architecture.

POURQUOI OWL-ViT, honnêtement : ce n'est PAS le détecteur le plus
précis disponible — Grounding DINO fait mieux, YOLO-World va plus
vite. Il est retenu pour trois raisons qui n'ont rien à voir avec ses
performances brutes :

  1. Licence Apache 2.0. L'écosystème Ultralytics (YOLO26, YOLOE) est
     en AGPL-3.0, ce qui contamine la question du déploiement
     commercial fermé — pour AXIS-OS ET pour les développeurs tiers
     du Play Store.
  2. Installation immédiate via `transformers`, donc la chaîne
     complète se valide tout de suite sur GPU de développement.
  3. NanoOWL, l'optimisation Jetson de NVIDIA, EST ce modèle. Passer
     au matériel embarqué sera un changement d'implémentation, pas de
     famille de modèle — donc pas un réentraînement ni une
     revalidation complète.

VOCABULAIRE RÉELLEMENT OUVERT : le modèle accepte des descriptions
libres ("un verre", "une boîte en carton") sans avoir été entraîné sur
ces classes. C'est ce qui permet à une app tierce de chercher un objet
que personne n'avait prévu — sans cette propriété, chaque nouvel objet
exigerait un réentraînement, ce qui ne passe pas à l'échelle d'un
catalogue d'apps.
"""
from __future__ import annotations

import time
from typing import List, Optional

from axis_os.hal.core.types import CameraFrame
from axis_os.services.perception.service import Detection, IObjectDetector

# Requêtes par défaut quand l'appelant ne précise rien (describe_scene).
# Volontairement génériques : ce sont des amorces, pas une liste fermée
# — le modèle accepte n'importe quelle description à l'appel suivant.
_DEFAULT_QUERIES = [
    "a person", "a box", "a cup", "a bottle", "a chair",
    "a table", "a door", "a tool",
]


class OwlViTDetector(IObjectDetector):
    """Détecteur à vocabulaire ouvert via OWL-ViT.

        detector = OwlViTDetector(threshold=0.1)
        detector.load()                       # coûteux, une seule fois
        svc = PerceptionService(camera, tree, detector)
    """

    def __init__(
        self,
        model_name: str = "google/owlvit-base-patch32",
        threshold: float = 0.1,
        device: Optional[str] = None,
        max_detections: int = 20,
    ):
        """
        threshold : seuil de confiance. Bas par défaut (0.1) car les
            scores d'OWL-ViT ne sont PAS calibrés comme ceux d'un
            détecteur à classes fermées — un 0.15 peut être une bonne
            détection. À régler sur des images réelles, jamais deviné.
        device : "cuda" si disponible, sinon "cpu". CPU fonctionne mais
            compte en secondes par image, pas en millisecondes.
        """
        self._model_name = model_name
        self._threshold = threshold
        self._device = device
        self._max_detections = max_detections

        self._model = None
        self._processor = None
        self._load_error: Optional[str] = None
        self._inferences = 0
        self._total_ms = 0.0

    # ---- chargement ------------------------------------------------------

    def load(self) -> bool:
        """Charge le modèle. Séparé du constructeur DÉLIBÉRÉMENT :
        l'opération prend plusieurs secondes et télécharge des poids au
        premier appel. Un constructeur qui ferait ça rendrait
        impossible de construire un service pour en inspecter la
        configuration sans payer le chargement."""
        if self._model is not None:
            return True
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
            self._processor = OwlViTProcessor.from_pretrained(self._model_name)
            self._model = OwlViTForObjectDetection.from_pretrained(
                self._model_name).to(self._device)
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
        print(f"  [owlvit] {self._model_name} chargé sur {self._device}")
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
        """Détecte dans `frame`. `query` est une description libre.

        Rend une liste VIDE si le modèle n'est pas chargé, plutôt que de
        lever : le service de perception traite déjà l'absence de
        détection comme un cas normal, et une exception ici ferait
        échouer une mission pour une raison d'infrastructure.
        """
        if self._model is None or frame is None:
            return []

        try:
            import torch
            from PIL import Image
        except ImportError:
            return []

        if frame.encoding != "rgb8":
            # Ne pas deviner : un depth16 interprété comme du RGB
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
            results = self._processor.post_process_object_detection(
                outputs=outputs, threshold=self._threshold,
                target_sizes=target_sizes,
            )[0]
        except Exception as exc:        # noqa: BLE001
            print(f"  [owlvit] inférence échouée : {exc}")
            return []
        finally:
            self._total_ms += (time.perf_counter() - t0) * 1000
            self._inferences += 1

        detections: List[Detection] = []
        for score, label_idx, box in zip(
                results["scores"], results["labels"], results["boxes"]):
            x0, y0, x1, y1 = (float(v) for v in box.tolist())
            detections.append(Detection(
                label=queries[int(label_idx)],
                confidence=float(score),
                # OWL-ViT rend (x0,y0,x1,y1) ; Detection attend
                # (x, y, largeur, hauteur). Confondre les deux décale
                # le centre de la boîte, donc la direction calculée —
                # sans qu'aucune vérification ne s'en aperçoive.
                bbox=(int(x0), int(y0), int(x1 - x0), int(y1 - y0)),
                # Horodatage de l'IMAGE, pas de l'inférence : la scène
                # date du moment de la capture.
                timestamp_ns=frame.timestamp_ns,
            ))

        detections.sort(key=lambda d: -d.confidence)
        return detections[:self._max_detections]

    @property
    def stats(self) -> dict:
        return {
            "model": self._model_name,
            "device": self._device,
            "loaded": self.is_loaded,
            "inferences": self._inferences,
            "mean_ms": (self._total_ms / self._inferences
                        if self._inferences else 0.0),
            "load_error": self._load_error,
        }
