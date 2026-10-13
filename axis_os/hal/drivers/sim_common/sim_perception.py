"""
AXIS-OS — Faux drivers ICamera/IMicrophone/ISpeaker
========================================================
Faux pilotes (drivers) de caméra, de micro et de haut-parleur, pour la
simulation et les tests.

Quand ce fichier a été écrit, il n'y avait pas de vrai driver caméra
pour le G1 (le robot humanoïde Unitree G1), seulement ce faux qui
rejoue des images depuis le disque. C'était une décision explicite, pas
une limite technique : un vrai driver caméra G1 dépendait d'un vrai
matériel, non disponible ici. La même remarque vaut pour G1DDSActuator,
le pilote des moteurs du G1 via DDS (Data Distribution Service, le
système de messages qui commande les moteurs du robot), jamais testé
sur matériel réel. Un vrai driver caméra existe depuis, pour la caméra
Intel RealSense du G1 (axis_os/hal/drivers/unitree_g1/g1_camera.py) ;
ce faux driver reste l'outil des tests et de la simulation.

Ce faux driver existe pour que les interfaces ICamera, IMicrophone et
ISpeaker de la HAL (Hardware Abstraction Layer, la couche d'abstraction
matérielle, voir axis_os/hal/core/interfaces.py) soient testables
immédiatement, sans attendre un vrai matériel ni un vrai
PerceptionService. Ce service de perception avait été délibérément
laissé de côté au moment où ce fichier a été écrit (« une coquille sans
modèle donnerait une fausse impression d'avancement ») ; il existe
depuis, dans axis_os/services/perception/service.py.

FakeCamera rejoue de vraies images lues dans un dossier du disque, et
non des données synthétiques générées en mémoire. Une app ou un service
testé contre ce faux driver exerce donc le vrai chemin : lecture de
fichier, décodage minimal, découpage en CameraFrame (le type « image »
de la HAL). Seule la provenance des octets (disque plutôt que capteur
physique) diffère d'un vrai driver.

FakeMicrophone et FakeSpeaker restent synthétiques (silence
déterministe, simple enregistrement de ce qui est joué) : ce dépôt ne
contient aucun enregistrement audio de référence, contrairement aux
images, que l'utilisateur peut facilement fournir à FakeCamera.
QueuedMicrophone rejoue, lui, des segments audio préparés à l'avance,
par exemple lus depuis des fichiers WAV avec wav_to_chunk.

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
    Rejoue en boucle les fichiers d'un dossier du disque, dans l'ordre
    alphabétique : chaque appel à capture() avance d'une image.

    Formats acceptés, sans dépendance externe (ni Pillow ni OpenCV, les
    bibliothèques de traitement d'image habituelles, ne sont requises
    pour ce faux driver minimal) :
      - .ppm : image PPM binaire (variante « P6 », un format très simple :
        un court en-tête texte suivi des pixels bruts, trois octets
        rouge, vert, bleu (RGB) par pixel) ;
      - .raw : octets RGB bruts, dont la taille doit être donnée par
        raw_width et raw_height (voir __init__).

    Dossier vide ou introuvable : capture() rend toujours None, jamais
    d'exception. C'est cohérent avec le contrat ICamera (le résultat est
    Optional[CameraFrame]) : un capteur qui ne répond pas n'est pas une
    erreur de programmation.
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
        """Paramètres de calibration de la caméra (intrinsèques : focale,
        centre optique). None par défaut, et c'est volontaire : une caméra
        factice n'a pas de calibration réelle, et en inventer une
        donnerait à un service de perception l'illusion de pouvoir
        calculer des positions 3D exactes. Pour tester le chemin calibré,
        passer intrinsics= explicitement à la construction."""
        return self._intrinsics

    def initialize(self) -> HALStatus:
        if not self._image_dir.exists():
            # Pas une erreur fatale : un dossier vide ou absent est un cas
            # d'usage légitime, pour tester le comportement « aucune image
            # disponible » (voir capture()).
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
            # Fichier corrompu ou illisible : même principe que
            # MemoryService._read_category()
            # (axis_os/services/memory/service.py). Un capteur ne doit
            # jamais planter à cause d'un seul fichier défaillant : on
            # rend None plutôt que de propager l'exception jusqu'à
            # l'appelant, et l'appel suivant passe à l'image suivante.
            return None

    def _read_ppm(self, path: Path) -> CameraFrame:
        """Décodage minimal d'une image PPM P6, sans dépendance externe.
        Suffisant pour rejouer de vraies images fournies par
        l'utilisateur sans imposer Pillow ou OpenCV à ce faux driver.
        La ligne de valeur maximale de l'en-tête est lue mais pas
        exploitée : chaque composante est lue sur un octet (images
        8 bits)."""
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
        # Un fichier .raw n'a pas d'en-tête : sans largeur ni hauteur
        # fournies, impossible de savoir comment découper les octets.
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
    Rend un silence numérique déterministe (tous les échantillons à zéro)
    et non un bruit aléatoire : deux appels avec les mêmes paramètres
    produisent ainsi des résultats identiques, donc testables.
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
        # Silence en PCM 16 bits (échantillons audio bruts, non
        # compressés, de deux octets chacun), mono.
        data = b"\x00\x00" * n_samples  # silence
        return AudioChunk(sample_rate=self._sample_rate, channels=1,
                          sample_width=2, data=data)


class FakeSpeaker(ISpeaker):
    """
    N'émet aucun son réel (pas de matériel audio à piloter en test) :
    garde simplement une trace des AudioChunk reçus, qu'un test peut
    inspecter via last_played et play_count.
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

    Il permet de tester toute la chaîne voix -> intention -> action (de
    la phrase prononcée à l'action du robot) sans matériel ni
    enregistrement en direct : on fournit les phrases dans l'ordre où
    elles seront « prononcées ».

    Quand la file est vide, il rend du silence plutôt que None : un vrai
    micro ne cesse pas de fonctionner parce que personne ne parle, et le
    service doit voir passer du silence comme un cas normal.
    """

    def __init__(self, sensor_id: str, chunks=None, sample_rate: int = 16000,
                 sleeper=None):
        """sleeper : fonction d'attente, time.sleep par défaut. En
        simulation, on passe une attente en temps simulé, pour qu'écouter
        « deux secondes » dure deux secondes simulées et non deux
        secondes de temps réel."""
        super().__init__(sensor_id, SensorType.MICROPHONE)
        import time as _time
        self._sleeper = sleeper or _time.sleep
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
        # On attend toujours la durée d'écoute demandée, comme un vrai
        # micro qui enregistre en temps réel, qu'il y ait une phrase ou du
        # silence.
        #
        # Deux défauts constatés sans cette attente :
        #   - silence rendu instantanément : une app qui écoute en boucle
        #     tournait à pleine vitesse, avec 117 547 « phrases ignorées »
        #     en une seule mission ;
        #   - phrase rendue instantanément : toutes les phrases de la file
        #     étaient « prononcées » à l'instant zéro, avant même le
        #     début de la vidéo, et leur son était tronqué.
        self._sleeper(duration_s)
        if self._file:
            return self._file.pop(0)
        n = max(1, int(self._sample_rate * duration_s))
        return AudioChunk(sample_rate=self._sample_rate, channels=1,
                          sample_width=2, data=b"\x00\x00" * n)


def wav_to_chunk(path) -> AudioChunk:
    """Fichier WAV -> AudioChunk, sans conversion.

    La fréquence d'échantillonnage et le nombre de canaux du fichier sont
    conservés tels quels : c'est au consommateur de convertir s'il en a
    besoin. Un vrai micro ne rééchantillonne pas non plus, et masquer
    cette étape ici cacherait un bug de conversion côté transcription
    (le passage de la voix au texte).
    """
    import wave
    with wave.open(str(path), "rb") as w:
        return AudioChunk(
            sample_rate=w.getframerate(), channels=w.getnchannels(),
            sample_width=w.getsampwidth(),
            data=w.readframes(w.getnframes()))
