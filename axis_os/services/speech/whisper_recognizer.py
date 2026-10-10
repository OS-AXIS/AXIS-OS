"""
AXIS-OS — Transcription par Whisper (OpenAI), via transformers.

Première implémentation réelle d'ISpeechRecognizer. Le service de parole
ne change pas d'une ligne : même séparation code/modèle que pour la
perception.

POURQUOI WHISPER : licence MIT, multilingue sans configuration,
robuste au bruit, et disponible en plusieurs tailles — du modèle
`tiny` (39 M paramètres, exploitable sur un Jetson) au `large`. La
taille se choisit selon le matériel, sans changer une ligne d'AXIS-OS.

CE QUI N'EST PAS FAIT ICI, volontairement : la détection d'activité
vocale. Ce composant transcrit une durée fixe d'audio. Un robot qui
écoute en permanence a besoin de savoir QUAND quelqu'un parle, ce qui
est un problème distinct (et bien plus léger qu'un modèle de
transcription).
"""
from __future__ import annotations

from typing import Optional, Tuple

from axis_os.hal.core.types import AudioChunk
from axis_os.services.speech.service import ISpeechRecognizer

WHISPER_SAMPLE_RATE = 16000
"""Whisper attend du 16 kHz. Un audio à une autre fréquence donne une
transcription fausse SANS ERREUR : les mots sont déformés, le modèle
rend quand même quelque chose de plausible. D'où le rééchantillonnage
explicite ci-dessous."""


class WhisperRecognizer(ISpeechRecognizer):

    def __init__(self, model_name: str = "openai/whisper-small",
                 language: str = "fr", device: Optional[str] = None,
                 revision: Optional[str] = None):
        """
        revision : identifiant de commit des poids. None = dernière
            version publiée, donc un comportement qui peut changer sans
            qu'AXIS-OS bouge. (OWL-ViT, lui, est épinglé et refuse de
            charger sans révision depuis l'audit du 2026-10-10 — voir
            owlvit_detector.OWLVIT_REVISION ; Whisper reste à épingler.)
        """
        self._model_name = model_name
        self._language = language
        self._device = device
        self._revision = revision
        self._model = None
        self._processor = None
        self._load_error: Optional[str] = None

    @property
    def language(self) -> str:
        return self._language

    @property
    def is_loaded(self) -> bool:
        return self._model is not None

    def load(self) -> bool:
        if self._model is not None:
            return True
        try:
            import torch
            from transformers import (
                WhisperForConditionalGeneration, WhisperProcessor,
            )
        except ImportError as exc:
            self._load_error = f"dépendances absentes ({exc})"
            print(f"  [whisper] {self._load_error}")
            return False
        try:
            if self._device is None:
                self._device = "cuda" if torch.cuda.is_available() else "cpu"
            kw = {"revision": self._revision} if self._revision else {}
            self._processor = WhisperProcessor.from_pretrained(
                self._model_name, **kw)
            self._model = WhisperForConditionalGeneration.from_pretrained(
                self._model_name, **kw).to(self._device)
            self._model.eval()
        except Exception as exc:        # noqa: BLE001
            self._load_error = f"chargement impossible : {exc}"
            print(f"  [whisper] {self._load_error}")
            self._model = None
            return False
        print(f"  [whisper] {self._model_name} chargé sur {self._device}")
        if self._revision is None:
            print("  [whisper] AVERTISSEMENT : poids NON épinglés.")
        return True

    # ---- transcription -------------------------------------------------

    @staticmethod
    def to_float_mono(audio: AudioChunk):
        """AudioChunk (PCM entier) -> tableau de flottants mono à 16 kHz.

        Trois conversions, chacune silencieuse si omise :
          - entiers signés vers flottants dans [-1, 1] ;
          - plusieurs canaux vers un seul (moyenne) ;
          - rééchantillonnage vers 16 kHz.
        Une seule oubliée donne une transcription fausse sans erreur.
        """
        import numpy as np
        if audio.sample_width == 2:
            dt, echelle = np.int16, 32768.0
        elif audio.sample_width == 4:
            dt, echelle = np.int32, 2147483648.0
        elif audio.sample_width == 1:
            dt, echelle = np.uint8, 128.0
        else:
            raise ValueError(
                f"largeur d'échantillon non gérée : {audio.sample_width} "
                f"octets — refus plutôt qu'une transcription fausse")

        x = np.frombuffer(audio.data, dtype=dt).astype(np.float32)
        if audio.sample_width == 1:
            x = x - echelle
        x = x / echelle

        if audio.channels > 1:
            n = (x.size // audio.channels) * audio.channels
            x = x[:n].reshape(-1, audio.channels).mean(axis=1)

        if audio.sample_rate != WHISPER_SAMPLE_RATE and x.size:
            # Rééchantillonnage linéaire : suffisant pour de la parole,
            # et sans dépendance supplémentaire.
            n_out = int(round(x.size * WHISPER_SAMPLE_RATE / audio.sample_rate))
            if n_out > 1:
                x = np.interp(np.linspace(0, x.size - 1, n_out),
                              np.arange(x.size), x).astype(np.float32)
        return x

    def transcribe(self, audio: AudioChunk) -> Tuple[str, float]:
        if self._model is None or audio is None:
            return "", 0.0
        try:
            import numpy as np
            import torch
        except ImportError:
            return "", 0.0

        try:
            x = self.to_float_mono(audio)
        except ValueError as exc:
            print(f"  [whisper] {exc}")
            return "", 0.0
        if x.size < WHISPER_SAMPLE_RATE // 10:      # moins de 100 ms
            return "", 0.0

        # Silence : inutile de faire tourner le modèle, et surtout il
        # rendrait une phrase plausible sur du bruit de fond.
        if float(np.abs(x).max()) < 1e-3:
            return "", 0.0

        try:
            inputs = self._processor(
                x, sampling_rate=WHISPER_SAMPLE_RATE, return_tensors="pt")
            feats = inputs.input_features.to(self._device)
            with torch.no_grad():
                ids = self._model.generate(
                    feats,
                    language=self._language,
                    task="transcribe",
                    output_scores=True,
                    return_dict_in_generate=True,
                )
            texte = self._processor.batch_decode(
                ids.sequences, skip_special_tokens=True)[0].strip()
            # Confiance : moyenne des probabilités du jeton retenu à
            # chaque pas. Grossier, mais permet d'ignorer une
            # transcription hasardeuse plutôt que de l'exécuter.
            conf = 1.0
            if getattr(ids, "scores", None):
                probs = []
                for s in ids.scores:
                    p = torch.softmax(s[0].float(), dim=-1)
                    probs.append(float(p.max()))
                conf = sum(probs) / len(probs) if probs else 1.0
            return texte, float(conf)
        except Exception as exc:        # noqa: BLE001
            print(f"  [whisper] transcription échouée : {exc}")
            return "", 0.0
