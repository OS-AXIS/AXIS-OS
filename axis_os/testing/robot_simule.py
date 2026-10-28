"""
AXIS-OS — Robot simulé léger, pour tester une app en quelques lignes
=====================================================================
Un G1 simulé (le driver UnitreeG1HAL sans matériel, avec ses garde-fous
d'articulation), posé sur une base qui glisse sans jambes, et entouré des
VRAIS services d'AXIS-OS : locomotion, gardien de proximité, navigation,
manipulation. Seuls le monde (la pose de la base, les objets, les
personnes) et la perception sont simulés, sans physique ni rendu : pas de
GPU, pas d'Isaac Sim, quelques secondes par essai.

    with SimulatedRobot(objects=[SimulatedObject("red box", (1.0, 0.0, 0.9))]) as robot:
        runtime = robot.make_runtime()
        ...

Les services sont cadencés ici, à 50 Hz, par la boucle du robot simulé,
comme sur un vrai robot : une app ne les cadence jamais elle-même.

Ce que ce robot ne simule PAS : l'équilibre, les chocs, la dynamique des
bras chargés, le glissement d'un objet dans les mains. Une app qui marche
ici doit encore être essayée en simulation complète (sim/) avant un vrai
robot.

Licence : Apache 2.0 (open-source — SDK public)
"""
from __future__ import annotations

import contextlib
import io
import math
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from axis_os.hal.core.frames import FRAME_WORLD
from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer
from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
from axis_os.hal.core.types import ActuatorCommand, BatteryState, ControlMode, HALStatus, Vector3
from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL
from axis_os.hal.watchdog.system import WatchdogSystem
from axis_os.services.base import ServiceHealth, ServiceResult
from axis_os.services.locomotion.proximite import GardienProximite
from axis_os.services.locomotion.service import (
    BaseState, ILocomotionAdapter, LocomotionService, Twist,
)
from axis_os.services.manipulation.service import Etat, ManipulationService
from axis_os.services.navigation.service import Carte, NavigationService, Pose2D
from axis_os.services.navigation.site_map import SiteMap
from axis_os.services.navigation.zones import GardienZones
from axis_os.services.people.appearance import appearance_descriptor, synthetic_person_image
from axis_os.services.people.detector import IPersonDetector, PersonDetection
from axis_os.services.people.follow import FollowService
from axis_os.services.people.service import PeopleService
from axis_os.services.speech.service import ISpeechRecognizer, SpeechService


@dataclass
class SimulatedObject:
    """Un objet du monde simulé : son nom (ce que la perception reconnaît),
    son centre dans le repère monde (m) et ses dimensions : largeur (prise
    entre les mains), profondeur, hauteur (m)."""
    label: str
    position: Tuple[float, float, float]
    size: Tuple[float, float, float] = (0.24, 0.20, 0.20)
    confidence: float = 0.9


class SimulatedPerson:
    """Une personne du monde simulé : sa position au sol (m, repère monde),
    les couleurs de son haut et de son bas (RGB, ce que voit l'apparence),
    et un trajet scripté, parcouru à la cadence du robot simulé.

        p = SimulatedPerson(2.0, 0.0, torso=(200, 30, 30))
        p.walk_to(4.0, 0.0, speed_mps=0.4).wait(3.0).walk_to(4.0, 2.0)

    Elle est vue par le lidar du gardien de proximité (une colonne de 0.2 à
    1.7 m), et par la caméra simulée si elle est dans son champ, à portée,
    et pas cachée par un obstacle de la carte ou par une autre personne."""

    RADIUS_M = 0.25

    def __init__(self, x: float, y: float, *, torso=(40, 60, 200), legs=(40, 40, 40)):
        self.x, self.y = float(x), float(y)
        self.torso, self.legs = tuple(torso), tuple(legs)
        self._etapes: List[tuple] = []
        self._attente: Optional[float] = None

    def walk_to(self, x: float, y: float, speed_mps: float = 0.5) -> "SimulatedPerson":
        self._etapes.append(("to", float(x), float(y), float(speed_mps)))
        return self

    def wait(self, seconds: float) -> "SimulatedPerson":
        self._etapes.append(("wait", float(seconds)))
        return self

    @property
    def moving(self) -> bool:
        return bool(self._etapes)

    def _avancer(self, dt: float) -> None:
        while self._etapes and dt > 0:
            e = self._etapes[0]
            if e[0] == "wait":
                reste = e[1] - dt
                if reste > 0:
                    self._etapes[0] = ("wait", reste)
                    return
                dt = -reste
                self._etapes.pop(0)
                continue
            _, tx, ty, v = e
            d = math.hypot(tx - self.x, ty - self.y)
            pas = v * dt
            if pas >= d:
                self.x, self.y = tx, ty
                dt -= d / v if v > 0 else dt
                self._etapes.pop(0)
            else:
                self.x += (tx - self.x) * pas / d
                self.y += (ty - self.y) * pas / d
                return


class SimulatedPersonDetector(IPersonDetector):
    """Détecteur de personnes simulé : la caméra de la tête voit les
    personnes de son champ (FOV_RAD), à moins de RANGE_M, que ni un obstacle
    de la carte ni une personne plus proche ne cache. La position est
    bruitée (comme une vraie mesure) ; l'apparence est calculée par le même
    code que sur une vraie image, à partir d'une silhouette synthétique."""

    FOV_RAD = 1.2
    RANGE_M = 6.0

    def __init__(self, robot: "SimulatedRobot", seed: int = 0):
        self._robot = robot
        self._rng = np.random.default_rng(seed)

    @property
    def needs_frame(self) -> bool:
        return False

    def _cachee(self, p: SimulatedPerson) -> bool:
        rx, ry = self._robot.x, self._robot.y
        for o in self._robot.carte.obstacles:
            if (o.zmax is None or o.zmax > 1.0) and _segment_coupe_rectangle(
                    rx, ry, p.x, p.y, o):
                return True
        d = math.hypot(p.x - rx, p.y - ry)
        for q in self._robot.people:
            if q is p or math.hypot(q.x - rx, q.y - ry) >= d:
                continue
            # Distance de q à la ligne de vue robot -> p.
            ux, uy = (p.x - rx) / d, (p.y - ry) / d
            t = (q.x - rx) * ux + (q.y - ry) * uy
            if 0 < t < d and abs(-(q.x - rx) * uy + (q.y - ry) * ux) < SimulatedPerson.RADIUS_M:
                return True
        return False

    def detect(self, frame) -> List[PersonDetection]:
        out = []
        for p in list(self._robot.people):
            rel = self._robot._vers_robot((p.x, p.y, 0.0))
            d = math.hypot(rel[0], rel[1])
            if d > self.RANGE_M or d < 0.2 or abs(math.atan2(rel[1], rel[0])) > self.FOV_RAD / 2:
                continue
            if self._cachee(p):
                continue
            sigma = 0.03 + 0.02 * d
            image = synthetic_person_image(p.torso, p.legs, noise=6.0, rng=self._rng)
            apparence = appearance_descriptor(image, (0, 0, image.shape[1], image.shape[0]))
            out.append(PersonDetection(
                bbox=(0.0, 0.0, 40.0, 100.0), score=0.9,
                position_world=(p.x + float(self._rng.normal(0, sigma)),
                                p.y + float(self._rng.normal(0, sigma)), 0.0),
                appearance=tuple(float(v) for v in apparence)))
        return out


def _segment_coupe_rectangle(x0, y0, x1, y1, o) -> bool:
    """Le segment (x0, y0) -> (x1, y1) traverse-t-il le rectangle `o` ?"""
    n = max(2, int(math.hypot(x1 - x0, y1 - y0) / 0.05))
    return any(o.contient(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n)
               for k in range(1, n))


class _MicroSimule:
    """Micro du robot simulé : capture() rend dès qu'une phrase entendue
    (SimulatedRobot.hear) est due, sinon à la fin de la durée demandée."""

    def __init__(self, robot: "SimulatedRobot"):
        self._robot = robot
        self._ok = False

    def initialize(self) -> HALStatus:
        self._ok = True
        return HALStatus.OK

    def shutdown(self) -> HALStatus:
        self._ok = False
        return HALStatus.OK

    def is_healthy(self) -> bool:
        return self._ok

    def capture(self, duration_s: float):
        from axis_os.hal.core.types import AudioChunk
        fin = self._robot.clock() + max(0.0, duration_s)
        while self._robot.clock() < fin and not self._robot._phrase_due():
            time.sleep(0.005)
        return AudioChunk(sample_rate=16000, channels=1, sample_width=2, data=b"")


class _TranscriptionSimulee(ISpeechRecognizer):
    """Rend la prochaine phrase due (confiance 1), ou rien (silence)."""

    def __init__(self, robot: "SimulatedRobot"):
        self._robot = robot

    def transcribe(self, audio):
        texte = self._robot._prendre_phrase()
        return (texte, 1.0) if texte else ("", 0.0)

    @property
    def language(self) -> str:
        return "fr"


class _HALSimulee(UnitreeG1HAL):
    """Le G1 simulé, avec une batterie dont le test fixe la charge (None :
    le driver ne la mesure pas, comme le G1 simulé ordinaire)."""
    batterie: Optional[float] = None

    def get_battery(self) -> Optional[BatteryState]:
        if self.batterie is None:
            return None
        return BatteryState(percent=float(self.batterie), charging=False)


class _BaseGlissante(ILocomotionAdapter):
    """Adaptateur de locomotion du robot simulé : la base avance de la
    vitesse demandée, sans jambes ; les cibles de bras écrites par la
    manipulation (pose_bras) sont transmises aux articulations quand elles
    changent, jamais réécrites sinon, pour ne pas écraser un geste
    commandé par une app."""

    def __init__(self, robot: "SimulatedRobot", dt: float, max_twist: Twist):
        self._robot, self._dt, self._max = robot, dt, max_twist
        self.pose_bras: Dict[str, float] = {}
        self._envoyees: Dict[str, float] = {}

    @property
    def control_dt(self) -> float:
        return self._dt

    @property
    def max_twist(self) -> Twist:
        return self._max

    def reset(self) -> None:
        pass

    def step(self, state: BaseState, twist: Twist) -> Dict[str, float]:
        self._robot._avancer(twist)
        nouvelles = {j: v for j, v in self.pose_bras.items() if self._envoyees.get(j) != v}
        self._envoyees.update(nouvelles)
        return nouvelles

    def is_fallen(self, state: BaseState) -> bool:
        return self._robot.fallen


class SimulatedPerception:
    """Perception simulée : « voit » les objets du monde dont le nom
    contient la requête, avec leur position exacte (la caméra de profondeur
    idéale), dans le repère demandé. Même interface que PerceptionService
    pour ce qu'en lit le SDK (find, describe_scene)."""

    def __init__(self, robot: "SimulatedRobot"):
        self._robot = robot
        self._demarre = False

    @property
    def service_id(self) -> str:
        return "perception"

    def start(self) -> HALStatus:
        self._demarre = True
        return HALStatus.OK

    def stop(self) -> None:
        self._demarre = False

    def health(self) -> ServiceHealth:
        return ServiceHealth(running=self._demarre, healthy=True, detail="simulée")

    def _visibles(self, query: Optional[str]) -> List[SimulatedObject]:
        q = (query or "").strip().lower()
        return [o for o in self._robot.objects
                if o is not self._robot.held and (not q or q in o.label.lower())]

    def find(self, query: Optional[str], now_ns: Optional[int] = None,
             frame: str = "base") -> ServiceResult:
        from axis_os.services.perception.service import Detection
        if not self._demarre:
            return ServiceResult(ok=False, reason="service non démarré")
        dets = []
        for o in sorted(self._visibles(query), key=lambda o: -o.confidence):
            p = np.asarray(o.position, dtype=float)
            q = p if frame == FRAME_WORLD else self._robot._vers_robot(p)
            n = float(np.linalg.norm(q)) or 1.0
            d = Detection(label=o.label, confidence=o.confidence, bbox=(0, 0, 1, 1),
                          direction=Vector3(q[0] / n, q[1] / n, q[2] / n),
                          position_3d=Vector3(float(q[0]), float(q[1]), float(q[2])),
                          timestamp_ns=time.monotonic_ns(),
                          frame="world" if frame == FRAME_WORLD else "base")
            dets.append(d)
        return ServiceResult(ok=True, value=dets)

    def describe_scene(self) -> ServiceResult:
        noms = [o.label for o in self._visibles(None)]
        return ServiceResult(ok=True, value=("je vois : " + ", ".join(noms)) if noms
                             else "rien de reconnaissable")


class SimulatedRobot:
    """Robot simulé léger (voir l'en-tête du module).

    Les personnes sont détectées un pas sur PEOPLE_EVERY (10 Hz à 50 Hz).

    carte            : la carte du site (Carte) ; par défaut une pièce vide
                       de 10 m sur 10 m, centrée sur l'origine.
    start            : pose de départ (x, y, cap), repère monde.
    proximity_guard  : avec un gardien de proximité, qui voit les personnes
                       ajoutées par add_person().
    manipulation     : avec le service de manipulation (prendre, poser).
    objects          : objets du monde (SimulatedObject).
    battery_percent  : charge de la batterie, ou None (inconnue).
    grasp_succeeds   : faux pour simuler une prise qui ne tient pas.
    people           : personnes du monde (SimulatedPerson), vues par le
                       gardien de proximité et par la caméra simulée.
    speech           : avec un service de parole ; hear() lui fait entendre
                       une phrase.
    seed             : graine du bruit des mesures simulées.
    site_map         : carte du sens (lieux nommés, zones) ; par défaut une
                       carte vide, que le test complète par publish().
    time_scale       : le temps du robot passe `time_scale` fois plus vite
                       que le temps réel (2 : un essai de 20 s en dure 10).
                       Tout ce qui mesure le temps du robot (services, apps
                       par ctx.now() et ctx.sleep()) suit cette horloge.
    """

    PEOPLE_EVERY = 5

    def __init__(self, *, carte: Optional[Carte] = None,
                 start: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 proximity_guard: bool = True, manipulation: bool = True,
                 objects=(), battery_percent: Optional[float] = None,
                 grasp_succeeds: bool = True, tick_hz: float = 50.0,
                 max_twist: Twist = Twist(0.4, 0.2, 0.8), people=(),
                 speech: bool = False, seed: int = 0, time_scale: float = 1.0,
                 site_map: Optional[SiteMap] = None):
        if time_scale == 1.0:
            self.clock = time.monotonic
        else:
            t0 = time.monotonic()
            self.clock = lambda: t0 + (time.monotonic() - t0) * time_scale
        self._echelle = float(time_scale)
        self.carte = carte or Carte(-5.0, -5.0, 5.0, 5.0)
        with contextlib.redirect_stdout(io.StringIO()):
            self.raw_hal = _HALSimulee(f"g1_simule_{id(self):x}")
            self.raw_hal.batterie = battery_percent
            self.hal = NormalizedRobotHAL(self.raw_hal, build_g1_normalizer())
            self.hal.initialize_all()
        self.watchdog = WatchdogSystem(self.hal)
        self.x, self.y, self.heading = (float(v) for v in start)
        self.fallen = False
        self.people: List[SimulatedPerson] = list(people)
        self.objects: List[SimulatedObject] = list(objects)
        self.held: Optional[SimulatedObject] = None
        self.grasp_succeeds = grasp_succeeds
        self.errors: List[str] = []
        self.ticks = 0
        self._dt = 1.0 / tick_hz
        self._dernier_pas: Optional[float] = None
        # Instant de la mesure lidar : le début du pas en cours, toujours
        # antérieur à l'instant où le gardien l'évalue (un horodatage pris
        # après lui paraîtrait venir du futur et arrêterait le robot).
        self._t_mesure = self.clock()
        self.adapter = _BaseGlissante(self, self._dt, max_twist)
        self.guard = (GardienProximite(self._scan, self._pose_xyc,
                                       obstacles=self.carte.obstacles, horloge=self.clock)
                      if proximity_guard else None)
        self.site_map = site_map if site_map is not None else SiteMap()
        self.zone_guard = GardienZones(self.site_map, self._pose_xyc)
        self.locomotion = LocomotionService(self.adapter, self._etat_base, self._envoyer,
                                            clock=self.clock, gardien=self.guard,
                                            zones=self.zone_guard)
        self.navigation = NavigationService(self.carte, self._pose2d, self.locomotion,
                                            horloge=self.clock, site_map=self.site_map)
        self.manipulation = (ManipulationService(self.adapter.pose_bras, self._mains,
                                                 self._vers_robot, self._prise_tient,
                                                 gardien=self.guard, horloge=self.clock)
                             if manipulation else None)
        self.perception = SimulatedPerception(self)
        # Personnes : les VRAIS services de détection-suivi et de suivi, sur
        # un détecteur simulé (pas d'image, pas de modèle).
        self.people_service = PeopleService(SimulatedPersonDetector(self, seed),
                                            source_pose=self._pose_xyc, horloge=self.clock)
        self.follow_service = FollowService(self.people_service, self.locomotion, self.guard,
                                            source_pose=self._pose_xyc, horloge=self.clock,
                                            navigation=self.navigation)
        self._phrases: List[Tuple[float, str]] = []
        self._verrou_phrases = threading.Lock()
        self.speech = (SpeechService(_MicroSimule(self), _TranscriptionSimulee(self))
                       if speech else None)
        # Synthèse vocale factice : `voice.spoken` garde ce que le robot a dit.
        from axis_os.services.speech.synthesis import FakeSpeechSynthesizer
        self.voice = FakeSpeechSynthesizer()
        self._arret = threading.Event()
        self._fil: Optional[threading.Thread] = None

    # ---- monde -----------------------------------------------------------

    def add_person(self, x: float, y: float, **kw) -> SimulatedPerson:
        """Une personne debout en (x, y), que voient le gardien de proximité
        et la caméra (voir SimulatedPerson pour `kw` et pour la faire
        marcher)."""
        p = SimulatedPerson(x, y, **kw)
        self.people.append(p)
        return p

    def hear(self, text: str, after_s: float = 0.0) -> None:
        """Le micro entendra `text` dans `after_s` secondes du robot (exige
        speech=True)."""
        with self._verrou_phrases:
            self._phrases.append((self.clock() + max(0.0, after_s), str(text)))
            self._phrases.sort()

    def _phrase_due(self) -> bool:
        with self._verrou_phrases:
            return bool(self._phrases) and self._phrases[0][0] <= self.clock()

    def _prendre_phrase(self) -> str:
        with self._verrou_phrases:
            if self._phrases and self._phrases[0][0] <= self.clock():
                return self._phrases.pop(0)[1]
            return ""

    def clear_people(self) -> None:
        self.people.clear()

    def set_battery(self, percent: Optional[float]) -> None:
        """Charge de la batterie (None : inconnue)."""
        self.raw_hal.batterie = percent

    def fall(self) -> None:
        """Simule une chute : la locomotion la détecte au pas suivant."""
        self.fallen = True

    @property
    def pose(self) -> Tuple[float, float, float]:
        return self.x, self.y, self.heading

    # ---- cycle de vie ----------------------------------------------------

    def start(self) -> "SimulatedRobot":
        """Démarre les services et la boucle qui les cadence."""
        self.locomotion.start()
        self.navigation.start()
        if self.manipulation is not None:
            self.manipulation.start()
        self.perception.start()
        self.people_service.start()
        self.follow_service.start()
        if self.speech is not None:
            self.speech.start()
        self._arret.clear()
        self._fil = threading.Thread(target=self._boucle, daemon=True, name="robot-simule")
        self._fil.start()
        return self

    def stop(self) -> None:
        self._arret.set()
        if self._fil is not None:
            self._fil.join(timeout=2.0)
            self._fil = None

    def __enter__(self) -> "SimulatedRobot":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()

    def make_runtime(self, **kw):
        """Un PlayStoreRuntime en profil SIMULATION, branché sur ce robot et
        ses services. Les permissions DANGEROUS y sont accordées sans humain
        (accepter_tout_SIMULATION) : c'est un banc d'essai, jamais un robot
        réel. Les mots-clés passés remplacent les réglages par défaut."""
        from axis_os.store.runtime.play_store import PlayStoreRuntime, accepter_tout_SIMULATION
        from axis_os.store.runtime.profil import Profil
        reglages = dict(locomotion_service=self.locomotion, perception_service=self.perception,
                        navigation_service=self.navigation,
                        manipulation_service=self.manipulation,
                        speech_synthesizer=self.voice, speech_service=self.speech,
                        people_service=self.people_service,
                        follow_service=self.follow_service,
                        site_map=self.site_map,
                        clock=self.clock, profil=Profil.SIMULATION,
                        confirmer_permissions=accepter_tout_SIMULATION)
        reglages.update(kw)
        with contextlib.redirect_stdout(io.StringIO()):
            return PlayStoreRuntime(self.hal, self.watchdog, **reglages)

    def _boucle(self) -> None:
        """La boucle du robot : les personnes bougent ; détection des
        personnes (à 10 Hz), suivi, navigation, locomotion, manipulation,
        puis l'objet porté suit les mains. Une exception d'un service est
        notée dans `errors`, sans arrêter la boucle."""
        t_prec = self.clock()
        while not self._arret.is_set():
            t0 = time.monotonic()
            self._t_mesure = self.clock()
            for p in list(self.people):
                p._avancer(self._t_mesure - t_prec)
            t_prec = self._t_mesure
            services = [self.follow_service, self.navigation, self.locomotion,
                        self.manipulation]
            if self.ticks % self.PEOPLE_EVERY == 0:
                services.insert(0, self.people_service)
            for service in services:
                if service is None:
                    continue
                try:
                    service.tick()
                except Exception as exc:        # noqa: BLE001 — noté pour le test
                    if len(self.errors) < 100:
                        self.errors.append(f"{service.service_id} : {exc!r}"[:300])
            self._porter()
            self.ticks += 1
            self._arret.wait(max(0.0, self._dt / self._echelle - (time.monotonic() - t0)))

    # ---- base, capteurs et mains simulés ----------------------------------

    def _avancer(self, twist: Twist) -> None:
        """Intègre la vitesse de la base depuis le pas précédent. Sous
        arrêt d'urgence, la base ne bouge plus (moteurs coupés)."""
        maintenant = self.clock()
        dt = 0.0 if self._dernier_pas is None else min(0.1, maintenant - self._dernier_pas)
        self._dernier_pas = maintenant
        if self.hal.estop_active or self.fallen:
            return
        c, s = math.cos(self.heading), math.sin(self.heading)
        self.x += (twist.vx * c - twist.vy * s) * dt
        self.y += (twist.vx * s + twist.vy * c) * dt
        self.heading += twist.wz * dt

    def _etat_base(self) -> BaseState:
        return BaseState(quat_wxyz=(1.0, 0.0, 0.0, 0.0), ang_vel_body=(0.0, 0.0, 0.0),
                         joint_pos={}, joint_vel={})

    def _envoyer(self, cibles: Dict[str, float]) -> Dict[str, HALStatus]:
        """Cibles des bras (convention brute du G1, celle des postures de la
        manipulation) -> articulations, à travers leurs garde-fous."""
        return self.raw_hal.send_joint_commands(
            {j: ActuatorCommand(mode=ControlMode.POSITION, position=v) for j, v in cibles.items()})

    def _pose_xyc(self):
        return self.x, self.y, self.heading

    def _pose2d(self) -> Pose2D:
        return Pose2D(self.x, self.y, self.heading, horodatage=self.clock())

    def _scan(self):
        """Balayage lidar : quelques points de sol (ignorés par le gardien),
        et chaque personne, en colonne de 0.2 à 1.7 m."""
        pts = [(self.x + 3.0 * math.cos(a), self.y + 3.0 * math.sin(a), 0.0)
               for a in np.linspace(0.0, 2 * math.pi, 8, endpoint=False)]
        for p in list(self.people):
            pts += [(p.x, p.y, z) for z in (0.2, 0.9, 1.7)]
        return np.asarray(pts, dtype=float), self._t_mesure

    def _vers_robot(self, p) -> np.ndarray:
        """Point du repère monde -> repère du robot (x devant, y à gauche)."""
        p = np.asarray(p, dtype=float)
        dx, dy = p[0] - self.x, p[1] - self.y
        c, s = math.cos(self.heading), math.sin(self.heading)
        return np.array([c * dx + s * dy, -s * dx + c * dy, p[2]])

    def _vers_monde(self, xr: float, yr: float, z: float) -> np.ndarray:
        c, s = math.cos(self.heading), math.sin(self.heading)
        return np.array([self.x + c * xr - s * yr, self.y + s * xr + c * yr, z])

    def _mains(self):
        """Position des mains dans le repère monde, d'après les angles
        mesurés des épaules : le roulis écarte les mains (33 cm à 0.20 rad,
        valeur de Postures), le tangage les lève (6 cm pour 0.20 rad)."""
        etat = self.raw_hal.get_full_state()
        roulis = etat["left_shoulder_roll_joint"].position
        tangage = etat["left_shoulder_pitch_joint"].position
        ecart = min(0.80, max(0.05, 0.33 + (roulis - 0.20)))
        z = 0.90 + (-0.30 - tangage) * 0.30
        return (self._vers_monde(0.30, ecart / 2, z), self._vers_monde(0.30, -ecart / 2, z))

    def _prise_tient(self) -> bool:
        """Vérification de prise : l'objet visé est-il entre les mains, et
        la prise doit-elle réussir ? Si oui, il est porté."""
        objet = self.manipulation.objet if self.manipulation is not None else None
        if objet is None or not self.grasp_succeeds:
            return False
        proche = min(self.objects, default=None,
                     key=lambda o: math.dist(o.position, objet.centre))
        if proche is None or math.dist(proche.position, objet.centre) > 0.15:
            return False
        self.held = proche
        return True

    def _porter(self) -> None:
        """L'objet porté suit le milieu des mains ; une fois les bras
        rouverts (plus d'objet en main), il reste où il a été posé."""
        if self.held is None or self.manipulation is None:
            return
        g, d = self._mains()
        milieu = (g + d) / 2.0
        if self.manipulation.objet is None and self.manipulation.etat in (Etat.LIBRE, Etat.LACHER):
            self.held.position = (float(milieu[0]), float(milieu[1]),
                                  float(milieu[2]))
            self.held = None
            return
        self.held.position = (float(milieu[0]), float(milieu[1]), float(milieu[2]))
