"""
AXIS-OS — Masque d'un objet à partir du cadre du détecteur, avec SAM 2.1 (Meta).

Le cadre d'OWL-ViT déborde sur la table, le fond et les mains : ses points
3D ne sont pas que ceux du carton. SAM 2.1 (« Segment Anything », Meta)
découpe l'objet dans le cadre, pixel par pixel. Code et poids sous licence
Apache-2.0 (facebook/sam2.1-hiera-tiny), chargés par `transformers`, déjà
la dépendance d'OWL-ViT : rien de plus à installer. Le plus petit des
modèles (tiny) : la perception tourne sur le Jetson du robot.

Mêmes règles que le détecteur (owlvit_detector.py) : poids épinglés à un
commit, au format safetensors seulement ; chargement refusé sinon. Sans
masque (modèle absent, échec), la perception garde le cadre : elle ne
devine rien, l'ajustement de la boîte rend alors un résidu plus grand.
"""
from __future__ import annotations

import time
from typing import Optional, Sequence

import numpy as np

from axis_os.hal.core.types import CameraFrame
from axis_os.services.perception.owlvit_detector import _HASH_COMMIT

MODELE_SAM2 = "facebook/sam2.1-hiera-tiny"
# Commit du 15 août 2025 : le premier avec la configuration de transformers
# (processor_config.json) et les poids en safetensors.
SAM2_REVISION = "de431c4043854a71d8101e17995dfe596bf101a5"


class Sam2Segmenteur:
    """masque(image, cadre) -> tableau booléen H × W, ou None."""

    def __init__(self, model_name: str = MODELE_SAM2, revision: Optional[str] = SAM2_REVISION,
                 device: Optional[str] = None):
        self._model_name, self._revision, self._device = model_name, revision, device
        self._model = self._processor = None
        self.erreur: Optional[str] = None
        self.stats = {"masques": 0, "echecs": 0, "ms_moyen": 0.0}

    def load(self) -> bool:
        if self._model is not None:
            return True
        if not (isinstance(self._revision, str) and _HASH_COMMIT.match(self._revision)):
            self.erreur = f"poids NON épinglés (revision={self._revision!r}) — chargement refusé"
            return False
        try:
            import torch
            from transformers import Sam2Model, Sam2Processor
            if self._device is None:
                self._device = "cuda" if torch.cuda.is_available() else "cpu"
            self._processor = Sam2Processor.from_pretrained(self._model_name,
                                                            revision=self._revision)
            self._model = Sam2Model.from_pretrained(self._model_name, revision=self._revision,
                                                    use_safetensors=True).to(self._device)
            self._model.eval()
        except Exception as exc:        # noqa: BLE001 — dépendances, réseau, mémoire
            self.erreur = f"chargement impossible : {exc}"
            self._model = None
            return False
        print(f"  [sam2] {self._model_name} chargé sur {self._device} (révision {self._revision})")
        return True

    @property
    def is_loaded(self) -> bool:
        return self._model is not None

    def masque(self, frame: CameraFrame, bbox: Sequence[float]) -> Optional[np.ndarray]:
        """Masque de l'objet du cadre `bbox` (x, y, largeur, hauteur), ou None
        (modèle absent, image illisible, échec : l'erreur est gardée)."""
        if self._model is None or frame is None or frame.encoding != "rgb8":
            return None
        try:
            import torch
            from PIL import Image
            t0 = time.perf_counter()
            image = Image.frombytes("RGB", (frame.width, frame.height), frame.data)
            x, y, w, h = (float(v) for v in bbox)
            entree = self._processor(images=image, input_boxes=[[[x, y, x + w, y + h]]],
                                     return_tensors="pt").to(self._device)
            with torch.no_grad():
                sortie = self._model(**entree, multimask_output=False)
            masques = self._processor.post_process_masks(sortie.pred_masks.cpu(),
                                                         entree["original_sizes"])[0]
            m = np.asarray(masques[0, 0], dtype=bool)
            n = self.stats["masques"]
            self.stats["ms_moyen"] = (self.stats["ms_moyen"] * n
                                      + 1000 * (time.perf_counter() - t0)) / (n + 1)
            self.stats["masques"] = n + 1
            return m if m.shape == (frame.height, frame.width) and m.any() else None
        except Exception as exc:        # noqa: BLE001 — jamais une panne de la perception
            self.erreur = f"masque impossible : {exc}"
            self.stats["echecs"] += 1
            return None
