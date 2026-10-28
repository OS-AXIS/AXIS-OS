"""
AXIS-OS — Détecteur de personnes réel : RT-DETR (R18), par transformers.

Choix, et pourquoi :
  - Licence. Le code (github.com/lyuwenyu/RT-DETR) et les poids publiés
    sur Hugging Face (PekingU/rtdetr_r18vd) sont sous Apache 2.0, qui
    permet l'usage commercial. Les détecteurs YOLO d'Ultralytics sont sous
    AGPL-3.0 et sont exclus (voir owlvit_detector.py et THIRD_PARTY.md).
    À ne pas confondre : RT-DETR n'est PAS un modèle Ultralytics, même si
    Ultralytics en distribue une copie ; on charge ici celle des auteurs.
  - Léger et rapide : 20 millions de paramètres, entraîné sur COCO (dont
    la classe « person ») ; des dizaines d'images par seconde sur un GPU
    embarqué, sans post-traitement NMS (suppression des boîtes en double)
    à régler.
  - Déjà dans le dépôt : il passe par transformers (Apache 2.0), comme
    OWL-ViT et Whisper ; aucune nouvelle dépendance.
  - Il ne fait que des boîtes. Il ne reconnaît personne : aucun visage,
    aucune identité.

Poids épinglés, comme ceux d'OWL-ViT : une révision précise (hash de commit
complet) et le format safetensors, qui ne contient que des données. Sans
révision épinglée, load() refuse de charger, sauf en simulation
(autoriser_non_epingle_SIMULATION). Les tests et la CI n'en téléchargent
aucun : ils utilisent des détections factices (FakePersonDetector).
"""
from __future__ import annotations

import re
import time
from typing import List, Optional

from axis_os.hal.core.types import CameraFrame

from .detector import IPersonDetector, PersonDetection

MODELE_RTDETR = "PekingU/rtdetr_r18vd"
# Commit du 1er juillet 2024 de PekingU/rtdetr_r18vd sur Hugging Face : un
# seul fichier de poids, model.safetensors (relevé sur huggingface.co).
# Changer de révision impose de revalider la détection (sim/validation/).
RTDETR_REVISION = "ac77a11ff0170a41b771c03264987f8ce2b0d753"
_HASH_COMMIT = re.compile(r"^[0-9a-f]{40}$")
_PAR_DEFAUT = object()


class RTDETRPersonDetector(IPersonDetector):
    """Détecteur de personnes RT-DETR (voir l'en-tête du module).

        det = RTDETRPersonDetector()
        det.load()                     # télécharge les poids épinglés
        svc = PeopleService(det, camera=..., frame_tree=..., source_pose=...)
    """

    def __init__(self, model_name: str = MODELE_RTDETR, revision=_PAR_DEFAUT,
                 threshold: float = 0.5, device: Optional[str] = None,
                 max_detections: int = 20,
                 autoriser_non_epingle_SIMULATION: bool = False):
        if revision is _PAR_DEFAUT:
            revision = RTDETR_REVISION if model_name == MODELE_RTDETR else None
        self._model_name = model_name
        self._revision = revision
        self._autoriser = autoriser_non_epingle_SIMULATION
        self._threshold = float(threshold)
        self._device = device
        self._max = int(max_detections)
        self._model = None
        self._processor = None
        self._person_ids: set = set()
        self._load_error: Optional[str] = None
        self._last_inference_error: Optional[str] = None
        self._inferences = 0
        self._total_ms = 0.0

    def load(self) -> bool:
        """Charge le modèle ; rend True s'il est prêt. Refuse des poids non
        épinglés AVANT tout téléchargement."""
        if self._model is not None:
            return True
        epingle = isinstance(self._revision, str) and bool(_HASH_COMMIT.match(self._revision))
        if not epingle and not self._autoriser:
            self._load_error = (f"poids NON épinglés (revision={self._revision!r}) — "
                                f"chargement refusé. Donner un hash de commit complet "
                                f"(40 caractères), ou autoriser_non_epingle_SIMULATION=True "
                                f"en simulation seulement.")
            print(f"  [rtdetr] {self._load_error}")
            return False
        try:
            import torch
            from transformers import RTDetrForObjectDetection, RTDetrImageProcessor
        except ImportError as exc:
            self._load_error = f"dépendances absentes ({exc}) — `pip install transformers torch`"
            print(f"  [rtdetr] {self._load_error}")
            return False
        try:
            if self._device is None:
                self._device = "cuda" if torch.cuda.is_available() else "cpu"
            kw = {"revision": self._revision} if self._revision else {}
            self._processor = RTDetrImageProcessor.from_pretrained(self._model_name, **kw)
            self._model = RTDetrForObjectDetection.from_pretrained(
                self._model_name, use_safetensors=True, **kw).to(self._device)
            self._model.eval()
        except Exception as exc:       # noqa: BLE001 — réseau, mémoire...
            self._load_error = f"chargement impossible : {exc}"
            print(f"  [rtdetr] {self._load_error}")
            self._model = None
            return False
        self._person_ids = self._ids_personne(getattr(self._model.config, "id2label", {}))
        if not self._person_ids:
            self._load_error = "le modèle n'a pas de classe « person »"
            self._model = None
            return False
        if not epingle:
            print("  [rtdetr] AVERTISSEMENT (SIMULATION) : poids NON épinglés — jamais sur "
                  "un vrai robot.")
        return True

    @staticmethod
    def _ids_personne(id2label) -> set:
        """Indices des classes « person » du modèle (COCO : 0)."""
        return {int(k) for k, v in dict(id2label).items() if str(v).strip().lower() == "person"}

    @property
    def is_loaded(self) -> bool:
        return self._model is not None

    def detect(self, frame: Optional[CameraFrame]) -> List[PersonDetection]:
        if self._model is None or frame is None:
            return []
        if frame.encoding != "rgb8":
            return []
        try:
            import torch
            from PIL import Image
            image = Image.frombytes("RGB", (frame.width, frame.height), frame.data)
        except Exception as exc:        # noqa: BLE001 — image illisible
            self._last_inference_error = f"image illisible : {exc}"
            return []
        t0 = time.perf_counter()
        try:
            entrees = self._processor(images=image, return_tensors="pt").to(self._device)
            with torch.no_grad():
                sorties = self._model(**entrees)
            tailles = torch.tensor([[frame.height, frame.width]]).to(self._device)
            res = self._processor.post_process_object_detection(
                sorties, target_sizes=tailles, threshold=self._threshold)[0]
        except Exception as exc:        # noqa: BLE001 — inférence en panne : dit, pas tu
            self._last_inference_error = str(exc)
            return []
        finally:
            self._total_ms += (time.perf_counter() - t0) * 1000.0
            self._inferences += 1
        self._last_inference_error = None
        return self._personnes(res, frame.timestamp_ns)

    def _personnes(self, res, timestamp_ns: int) -> List[PersonDetection]:
        """Résultat du post-traitement -> détections de la classe person."""
        out = []
        for score, label, boite in zip(res["scores"], res["labels"], res["boxes"]):
            if int(label) not in self._person_ids:
                continue
            x0, y0, x1, y1 = (float(v) for v in boite.tolist())
            # RT-DETR rend deux coins (x0, y0, x1, y1) ; PersonDetection
            # attend (x, y, largeur, hauteur).
            out.append(PersonDetection(bbox=(x0, y0, x1 - x0, y1 - y0), score=float(score),
                                       timestamp_ns=timestamp_ns))
        out.sort(key=lambda d: -d.score)
        return out[:self._max]

    @property
    def stats(self) -> dict:
        return {"model": self._model_name, "revision": self._revision or "NON ÉPINGLÉE",
                "license": "Apache-2.0", "device": self._device, "loaded": self.is_loaded,
                "inferences": self._inferences,
                "mean_ms": self._total_ms / self._inferences if self._inferences else 0.0,
                "load_error": self._load_error,
                "last_inference_error": self._last_inference_error}
