"""
AXIS-OS — Faux drivers ICamera/IMicrophone/ISpeaker
========================================================
"Pas de driver caméra réel pour le G1, seulement le faux qui rejoue
des images depuis le disque" — décision explicite, pas une limite
technique : un vrai driver caméra G1 dépendrait d'un vrai matériel
non disponible ici (voir la même remarque déjà faite pour
G1DDSActuator, jamais testé sur matériel réel). Ce faux driver existe
pour que ICamera/IMicrophone/ISpeaker (axis_os/hal/core/interfaces.py)
soient testables dès aujourd'hui, sans attendre un vrai matériel ni
un vrai PerceptionService (délibérément non implémenté ce soir —
« une coquille sans modèle donnerait une fausse impression
d'avancement »).

FakeCamera REJOUE de vraies images depuis un dossier disque — pas des
données synthétiques générées en mémoire. Une app ou un futur service
qui teste contre ce faux driver exerce donc le VRAI chemin de lecture
de fichier, décodage minimal, découpage en CameraFrame — seule la
provenance des octets (disque plutôt que capteur physique) diffère
d'un vrai driver.

FakeMicrophone/FakeSpeaker restent synthétiques (silence/tonalité
déterministe) — aucune source d'enregistrements audio de référence
disponible dans ce dépôt, contrairement aux images (facilement
fournissables par l'utilisateur pour FakeCamera).

Licence : Apache 2.0
"""

from __future__ import annotations
import itertools
import math
import struct
from pathlib import Path
from typing import List, Optional

from axis_os.hal.core.interfaces import ICamera, IMicrophone, ISpeaker
from axis_os.hal.core.types import (
    CameraFrame, CameraIntrinsics, AudioChunk, HALStatus, SensorType,
)


class FakeCamera(ICamera):
    """
    Rejoue les fichiers d'un dossier disque, dans l'ordre alphabétique,
    en boucle — chaque appel à capture() avance d'une image. Formats
    supportés : PPM (P6, brut, sans dépendance externe — pas de
    Pillow/OpenCV requis pour ce faux driver minimal) et tout fichier
    déjà encodé en octets bruts RGB de taille connue via
    `raw_width`/`raw_height` (voir __init__).

    Dossier vide ou introuvable : capture() retourne toujours None,
    jamais d'exception — cohérent avec le contrat ICamera (Optional
    [CameraFrame]), un capteur qui ne répond pas n'est pas une erreur
    de programmation.
    """

    def __init__(
        self,
        sensor_id: str,
        image_dir: Path,
        raw_width: Optional[int] = None,
        raw_height: Optional[int] = None,
        intrinsics: Optional[CameraIntrinsics] = None,
    ):
        super().__init__(sensor_id, SensorType.CAMERA_RGB)
        self._image_dir = Path(image_dir)
        self._raw_width = raw_width
        self._raw_height = raw_height
        self._intrinsics = intrinsics
        self._files: List[Path] = []
        self._cycle = None

    def get_intrinsics(self) -> Optional[CameraIntrinsics]:
        """None par défaut, et c'est VOLONTAIRE : une caméra factice
        n'a pas de calibration réelle, et en inventer une donnerait à
        un service de perception l'illusion de pouvoir calculer des
        positions 3D exactes. Passer `intrinsics=` explicitement pour
        tester le chemin calibré."""
        return self._intrinsics

    def initialize(self) -> HALStatus:
        if not self._image_dir.exists():
            # Pas une erreur fatale : un dossier vide/absent est un
            # cas d'usage légitime pour tester le comportement
            # "aucune image disponible" — voir capture().
            self._files = []
        else:
            self._files = sorted(
                p for p in self._image_dir.iterdir()
                if p.suffix.lower() in (".ppm", ".raw")
            )
        self._cycle = itertools.cycle(self._files) if self._files else None
        self._initialized = True
        return HALStatus.OK

    def shutdown(self) -> HALStatus:
        self._initialized = False
        return HALStatus.OK

    def is_healthy(self) -> bool:
        return self._initialized and len(self._files) > 0

    def capture(self) -> Optional[CameraFrame]:
        if not self._initialized or self._cycle is None:
            return None
        path = next(self._cycle)
        try:
            if path.suffix.lower() == ".ppm":
                return self._read_ppm(path)
            return self._read_raw(path)
        except (OSError, ValueError):
            # Fichier corrompu/illisible — même principe que
            # MemoryService._read_category() : ne jamais planter un
            # capteur pour un fichier individuel défaillant, passer
            # à l'image suivante au prochain appel plutôt que de
            # propager l'exception jusqu'à l'appelant.
            return None

    def _read_ppm(self, path: Path) -> CameraFrame:
        """Décodage PPM (P6) minimal — format texte+binaire simple,
        sans dépendance externe. Suffisant pour rejouer de vraies
        images fournies par l'utilisateur sans imposer Pillow/OpenCV
        à ce faux driver."""
        with open(path, "rb") as f:
            magic = f.readline().strip()
            if magic != b"P6":
                raise ValueError(f"Format PPM non supporté (attendu P6) : {magic!r}")
            dims_line = f.readline().strip()
            while dims_line.startswith(b"#"):  # commentaires PPM
                dims_line = f.readline().strip()
            width, height = map(int, dims_line.split())
            maxval_line = f.readline().strip()
            data = f.read(width * height * 3)
        return CameraFrame(width=width, height=height, encoding="rgb8", data=data)

    def _read_raw(self, path: Path) -> CameraFrame:
        if self._raw_width is None or self._raw_height is None:
            raise ValueError(
                "Fichier .raw nécessite raw_width/raw_height explicites "
                "à la construction de FakeCamera"
            )
        with open(path, "rb") as f:
            data = f.read(self._raw_width * self._raw_height * 3)
        return CameraFrame(width=self._raw_width, height=self._raw_height,
                          encoding="rgb8", data=data)


class FakeMicrophone(IMicrophone):
    """
    Silence numérique déterministe (échantillons à zéro) — pas de
    bruit aléatoire, pour que deux appels avec les mêmes paramètres
    produisent des résultats identiques et testables.
    """

    def __init__(self, sensor_id: str, sample_rate: int = 16000):
        super().__init__(sensor_id, SensorType.MICROPHONE)
        self._sample_rate = sample_rate

    def initialize(self) -> HALStatus:
        self._initialized = True
        return HALStatus.OK

    def shutdown(self) -> HALStatus:
        self._initialized = False
        return HALStatus.OK

    def is_healthy(self) -> bool:
        return self._initialized

    def capture(self, duration_s: float) -> Optional[AudioChunk]:
        if not self._initialized:
            return None
        n_samples = max(0, int(self._sample_rate * duration_s))
        data = b"\x00\x00" * n_samples  # 16-bit PCM, silence
        return AudioChunk(sample_rate=self._sample_rate, channels=1,
                          sample_width=2, data=data)


class FakeSpeaker(ISpeaker):
    """
    N'émet aucun vrai son (pas de matériel audio à piloter en test) —
    enregistre simplement les AudioChunk reçus pour inspection par un
    test, via last_played et play_count.
    """

    def __init__(self, speaker_id: str):
        super().__init__(speaker_id)
        self.last_played: Optional[AudioChunk] = None
        self.play_count = 0

    def initialize(self) -> HALStatus:
        self._initialized = True
        return HALStatus.OK

    def shutdown(self) -> HALStatus:
        self._initialized = False
        return HALStatus.OK

    def play(self, audio: AudioChunk) -> HALStatus:
        if not self._initialized:
            return HALStatus.ERROR_NOT_INIT
        self.last_played = audio
        self.play_count += 1
        return HALStatus.OK


class QueuedMicrophone(IMicrophone):
    """Micro qui rejoue une file de segments audio préparés.

    Permet de tester toute la chaîne voix -> intention -> action sans
    matériel ni enregistrement en direct : on fournit les phrases, dans
    l'ordre où elles seront « prononcées ».

    Quand la file est vide, rend du SILENCE plutôt que None : un micro
    réel ne cesse pas de fonctionner parce que personne ne parle, et le
    service doit voir passer du silence comme cas normal.
    """

    def __init__(self, sensor_id: str, chunks=None, sample_rate: int = 16000):
        super().__init__(sensor_id, SensorType.MICROPHONE)
        self._file = list(chunks or [])
        self._sample_rate = sample_rate
        self.captures = 0

    def initialize(self) -> HALStatus:
        self._initialized = True
        return HALStatus.OK

    def shutdown(self) -> HALStatus:
        self._initialized = False
        return HALStatus.OK

    def is_healthy(self) -> bool:
        return self._initialized

    def enqueue(self, chunk: AudioChunk) -> None:
        self._file.append(chunk)

    @property
    def remaining(self) -> int:
        return len(self._file)

    def capture(self, duration_s: float):
        if not self._initialized:
            return None
        self.captures += 1
        if self._file:
            return self._file.pop(0)
        n = max(1, int(self._sample_rate * duration_s))
        return AudioChunk(sample_rate=self._sample_rate, channels=1,
                          sample_width=2, data=b"\x00\x00" * n)


def wav_to_chunk(path) -> AudioChunk:
    """Fichier WAV -> AudioChunk, sans conversion.

    La fréquence et le nombre de canaux du fichier sont conservés TELS
    QUELS : c'est au consommateur de convertir s'il en a besoin. Un
    micro réel ne rééchantillonne pas non plus, et masquer cette étape
    ici cacherait un bug de conversion côté transcription.
    """
    import wave
    with wave.open(str(path), "rb") as w:
        return AudioChunk(
            sample_rate=w.getframerate(), channels=w.getnchannels(),
            sample_width=w.getsampwidth(),
            data=w.readframes(w.getnframes()))
