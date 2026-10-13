"""
AXIS-OS — Transcription par Whisper (OpenAI), via transformers.

Whisper est un modèle de reconnaissance de la parole publié par OpenAI ;
transformers est la bibliothèque Python de Hugging Face (la plateforme
qui héberge les modèles publiés) qui sert à le télécharger et à
l'exécuter, en s'appuyant sur torch (PyTorch, la bibliothèque de calcul
qui fait tourner le modèle).

Ce fichier est la première implémentation réelle d'ISpeechRecognizer
(voir axis_os/services/speech/service.py). Le service de parole n'a pas
changé d'une ligne pour l'accueillir : c'est la même séparation entre
le code et le modèle que pour la perception.

POURQUOI WHISPER : licence MIT (très permissive), multilingue sans
configuration, robuste au bruit, et disponible en plusieurs tailles, du
modèle `tiny` (39 M paramètres, exploitable sur un Jetson, la carte de
calcul embarquée de NVIDIA) au `large`. La taille se choisit selon le
matériel, sans changer une ligne d'AXIS-OS.

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
"""Fréquence d'échantillonnage attendue par Whisper : 16 kHz (16 000
échantillons par seconde). Un audio à une autre fréquence donne une
transcription fausse SANS ERREUR : les mots sont déformés, mais le
modèle rend quand même quelque chose de plausible. D'où le
rééchantillonnage explicite dans to_float_mono() ci-dessous."""


class WhisperRecognizer(ISpeechRecognizer):
    """Reconnaisseur de parole fondé sur Whisper.

    Le constructeur ne charge rien : c'est load() qui télécharge et
    charge les poids (les paramètres appris du modèle), une opération
    coûteuse à faire une fois. Tant que le modèle n'est pas chargé,
    transcribe() rend un texte vide.
    """

    def __init__(self, model_name: str = "openai/whisper-small",
                 language: str = "fr", device: Optional[str] = None,
                 revision: Optional[str] = None):
        """
        model_name : nom du modèle sur Hugging Face.
        language : langue imposée à la transcription.
        device : "cuda" (carte graphique NVIDIA) ou "cpu" ; None laisse
            load() choisir la carte graphique si elle est disponible.
        revision : identifiant de commit des poids, c'est-à-dire une
            version figée du modèle publié. None = dernière version
            publiée, donc un comportement qui peut changer sans
            qu'AXIS-OS bouge. OWL-ViT (le modèle de détection d'objets
            du service de perception), lui, est épinglé : depuis un
            audit de sécurité du 10 octobre 2026, il refuse de charger
            sans révision figée (voir
            axis_os/services/perception/owlvit_detector.py,
            OWLVIT_REVISION). Whisper reste
            à épingler ; load() affiche un avertissement tant que ce
            n'est pas fait.
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
        """Charge le préprocesseur de Whisper (WhisperProcessor, qui
        prépare l'audio pour le modèle et décode le texte produit) et
        les poids du modèle.

        Rend True si le modèle est prêt (y compris s'il l'était déjà),
        False sinon : dépendances absentes ou chargement impossible.
        L'erreur est alors gardée dans _load_error et affichée, sans
        lever d'exception, pour qu'un modèle absent ne fasse pas
        planter l'appelant.
        """
        if self._model is not None:
            return True
        # Imports différés : torch et transformers sont des dépendances
        # facultatives, dont l'absence doit donner un échec propre.
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
            # Sans révision explicite, aucun argument n'est passé :
            # transformers prend alors la dernière version publiée.
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

        PCM désigne l'audio brut, une suite d'échantillons entiers.
        Trois conversions sont nécessaires, et chacune passe inaperçue
        si on l'omet :
          - entiers vers flottants dans [-1, 1] (16 et 32 bits sont
            signés ; 8 bits est non signé, centré sur 128) ;
          - plusieurs canaux vers un seul (moyenne) ;
          - rééchantillonnage vers 16 kHz.
        Une seule conversion oubliée donne une transcription fausse sans
        erreur. Une largeur d'échantillon non gérée lève donc ValueError
        plutôt que de produire une transcription fausse.
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
            # Les échantillons de queue qui ne forment pas une trame
            # complète (un échantillon pour chaque canal) sont ignorés
            # avant de faire la moyenne.
            n = (x.size // audio.channels) * audio.channels
            x = x[:n].reshape(-1, audio.channels).mean(axis=1)

        if audio.sample_rate != WHISPER_SAMPLE_RATE and x.size:
            # Rééchantillonnage par interpolation linéaire : suffisant
            # pour de la parole, et sans dépendance supplémentaire.
            n_out = int(round(x.size * WHISPER_SAMPLE_RATE / audio.sample_rate))
            if n_out > 1:
                x = np.interp(np.linspace(0, x.size - 1, n_out),
                              np.arange(x.size), x).astype(np.float32)
        return x

    def transcribe(self, audio: AudioChunk) -> Tuple[str, float]:
        """Rend (texte, confiance).

        Rend ("", 0.0), c'est-à-dire « rien compris », plutôt que de
        lever une exception ou d'inventer une phrase, dans tous ces
        cas : modèle non chargé, audio absent, dépendances absentes,
        format audio non géré, audio de moins de 100 ms, silence, ou
        erreur pendant la transcription.
        """
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

        # Silence (amplitude maximale sous 1e-3, soit 0,1 % de la pleine
        # échelle) : inutile de faire tourner le modèle, et surtout il
        # rendrait une phrase plausible sur du bruit de fond.
        if float(np.abs(x).max()) < 1e-3:
            return "", 0.0

        try:
            inputs = self._processor(
                x, sampling_rate=WHISPER_SAMPLE_RATE, return_tensors="pt")
            feats = inputs.input_features.to(self._device)
            # output_scores=True conserve les scores de chaque pas de
            # génération, nécessaires au calcul de la confiance.
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
            # Confiance : moyenne, sur tous les pas de génération, de la
            # probabilité du jeton retenu (le jeton est l'unité de texte
            # produite par le modèle ; softmax convertit les scores en
            # probabilités). Grossier, mais suffisant pour ignorer une
            # transcription hasardeuse plutôt que de l'exécuter (seuil
            # min_confidence de SpeechService). Sans scores disponibles,
            # la confiance vaut 1.0.
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
