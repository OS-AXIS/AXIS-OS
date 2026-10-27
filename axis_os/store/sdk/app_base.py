"""
AXIS-OS Robotics Play Store — SDK des apps, classe de base
==========================================================
Point de départ de tout développeur d'app : ce fichier fait partie du SDK
(le kit de développement fourni aux auteurs d'apps). Le Robotics Play
Store est la boutique d'apps d'AXIS-OS : il installe les apps, vérifie
leur signature et leurs permissions, puis les lance. Une app est une
classe qui étend RobotApp (en bas de ce fichier).

L'app n'accède au robot qu'à travers l'objet RobotContext qu'elle reçoit :
c'est sa cloison. Chaque méthode qui lit un capteur ou fait bouger le
robot vérifie d'abord que la permission correspondante est déclarée dans
le manifeste de l'app (sa fiche d'identité : nom, permissions, quotas ;
voir manifest.py), puis passe par la HAL (la couche d'abstraction
matérielle, seul chemin vers les moteurs) et ses garde-fous de sécurité.

Quand l'app s'exécute dans son sandbox (le « bac à sable » : un processus
séparé, lancé rien que pour elle), elle reçoit à la place un HALBridgeClient
(axis_os/store/runtime/hal_bridge.py) : mêmes méthodes, mais chaque appel
est relayé par IPC (la communication entre deux processus) vers un
RobotContext resté du côté de l'OS. test/test_context_parity.py vérifie
que les deux exposent exactement les mêmes méthodes publiques.

Limite connue : ces protections tiennent face à une app buggée, pas
encore face à une app délibérément hostile, car l'isolation des apps
n'est pas encore étanche. Le constat et le plan de correction sont dans
docs/isolation_apps.md et SECURITY.md.

Licence : Apache 2.0 (open-source — SDK public)
"""

from __future__ import annotations
from abc import ABC, abstractmethod
from dataclasses import dataclass, field, replace
from typing import Optional, Dict, Any
import math
import secrets
import time

from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
from axis_os.hal.core.types import (
    ActuatorCommand, ActuatorFeedback, HALStatus, ControlMode,
    JointGroup, infer_joint_group, ORIGINE_IA,
)
from axis_os.hal.watchdog.system import WatchdogSystem
from .manifest import AppManifest, Permission
from .version import API_VERSION
from .lifecycle import (
    CONFIRMATION_MAX_S, PROGRESS_HISTORY_MAX, PROGRESS_MESSAGE_MAX, QUESTION_MAX,
    STOP_MARGIN_S, STOP_SOURCES, EventPoller, MissionCancelled, Progress,
)
from .animation import (
    LEG_MATCH_TOL, Animation, AnimationPlayer, check_animation, load_animation,
    load_library_animation,
)
from .stream import (
    BusyError, CapabilityMissingError, TargetStream, TargetStreamEngine, stream_params,
)
from .results import (
    ABSENT, AVAILABLE, DEGRADED, EXCEPTIONS_PAR_CODE, ActionResult, AxisRefusal,
    BatteryInfo, InvalidArgumentError, RefusalCode, RobotInfo, SafetyStatus,
    object_from_target, refused, require_language, require_number, require_text,
    surface_height_from_target,
)


class PermissionError(AxisRefusal):
    """
    Levée quand une app appelle une méthode de RobotContext dont la
    permission n'est pas déclarée dans son manifeste.

    Attention au nom : cette classe porte le même nom que l'exception
    standard de Python, mais c'est une classe distincte, définie ici.
    Pour l'intercepter, il faut importer celle de ce module.

    Code de refus : RefusalCode.PERMISSION_DENIED (voir results.py).
    """
    code = RefusalCode.PERMISSION_DENIED


class EstopActiveError(AxisRefusal):
    """
    Levée quand une app tente d'envoyer une commande de mouvement alors
    qu'un e-stop (arrêt d'urgence) global est actif sur le robot.

    Pourquoi : le test de stress complet du système
    (test/test_ultimate_stress.py) a révélé un trou de sécurité. Une
    mission déjà en cours au moment où le Watchdog (le chien de garde
    logiciel qui surveille en continu l'état du robot) déclenchait un
    e-stop continuait auparavant jusqu'au bout : emergency_stop_all()
    coupait bien le matériel, mais rien n'empêchait le code de l'app de
    continuer à envoyer de nouvelles commandes de mouvement entre-temps.

    Correction : l'état e-stop est vérifié à chaque appel de move_arms(),
    move_legs(), move_all() et walk() (voir _require_no_estop()), et non
    plus une seule fois, au lancement de la mission par
    PlayStoreRuntime.run_mission(). Test : test/test_estop_interrupt.py.

    Depuis, l'arrêt d'urgence interrompt aussi la mission elle-même, et un
    on_start ou un on_stop en cours : dans le processus de l'app, tout
    appel au robot (sauf log, stop_walking, emergency_stop et
    get_recent_alerts) lève alors cette exception,
    l'app a une demi-seconde pour rendre son résultat, puis son processus
    est arrêté (voir `arret` dans SandboxRunner.run_app, et
    test/test_arret_urgence_mission.py).

    Code de refus : RefusalCode.ESTOP_ACTIVE (voir results.py).
    """
    code = RefusalCode.ESTOP_ACTIVE


EXCEPTIONS_PAR_CODE[RefusalCode.PERMISSION_DENIED.value] = PermissionError
EXCEPTIONS_PAR_CODE[RefusalCode.ESTOP_ACTIVE.value] = EstopActiveError


class UnauthorizedContextError(Exception):
    """
    Levée quand un RobotContext est construit sans jeton de session
    valide.

    Le trou que ce jeton ferme : auparavant, rien n'empêchait un code
    disposant d'un vrai NormalizedRobotHAL et d'un vrai WatchdogSystem
    (par exemple du code qui importe axis_os directement, hors de tout
    Play Store) de construire son propre RobotContext avec un AppManifest
    arbitraire, donc avec n'importe quelles permissions, sans jamais
    passer par install() ni par la vérification de signature
    (signing.py).

    Or tout le code d'une app passe par RobotContext : RobotApp est
    entièrement abstraite, et sans ctx.move_arms() et les autres
    méthodes, le code d'une app ne fait littéralement rien. Exiger ce
    jeton rend donc une app réellement inutilisable hors d'AXIS-OS. Le
    seul chemin de construction légitime est PlayStoreRuntime._make_ctx(),
    qui fournit le jeton : voir _RuntimeSessionToken ci-dessous, et ses
    limites.
    """
    pass


class _RuntimeSessionToken:
    """
    Jeton d'identité, créé une seule fois par instance de
    PlayStoreRuntime (voir PlayStoreRuntime.__init__) et transmis à
    chaque RobotContext qu'elle construit. Il est reconnu par référence
    d'objet (« est-ce le même objet ? »), jamais par une valeur
    comparable qu'on pourrait recopier.

    Limite assumée : en Python pur, sans extension compilée, aucune
    barrière n'empêche un code écrit délibérément pour contourner ce
    jeton d'y parvenir, car tout ce que définit un module Python reste
    accessible au code qui l'importe. Deux versions précédentes de ce
    mécanisme (une comparaison de valeur, puis un registre tenu au niveau
    du module) ont chacune semblé fermer ce trou sans y parvenir
    réellement, pour cette même raison.

    Ce mécanisme ne prétend donc PAS arrêter un développeur qui étudie le
    code source d'AXIS-OS et écrit délibérément du code pour le
    contourner : c'est hors de portée en Python pur.
    docs/hardware_estop_notes.md fait le même constat de fond pour la
    sécurité physique : aucune barrière logicielle n'est jamais absolue.

    Ce qu'il empêche réellement : qu'une app innocente, ou du code qui
    importe RobotContext sans intention malveillante particulière,
    obtienne un RobotContext fonctionnel par accident ou en suivant l'API
    publique évidente. C'était le besoin exprimé à l'origine : qu'une app
    écrite pour AXIS-OS ne « marche » que dans AXIS-OS. L'appel
    RobotContext(robot, watchdog, manifest), sans rien de plus, échoue
    désormais explicitement, avec un message clair, au lieu de sembler
    fonctionner. Test : test/test_context_lockdown.py.
    """
    __slots__ = ()

    def _verify(self, candidate: object) -> bool:
        """Vrai seulement si `candidate` est ce jeton-ci, le même objet."""
        return candidate is self


class RobotContext:
    """
    Interface sécurisée entre une app et le robot.

    Attention : ne jamais instancier directement hors de
    PlayStoreRuntime._make_ctx() ; voir UnauthorizedContextError pour le
    contexte complet. `_session_token` est requis et vérifié dès la
    construction, jamais optionnel : un appelant qui n'a pas accès au
    jeton d'une vraie instance de PlayStoreRuntime ne peut pas produire
    de RobotContext utilisable, quelles que soient les permissions qu'il
    tenterait de déclarer dans un AppManifest construit à la main. Pour
    un test unitaire, voir RobotContext.for_testing().

    Règles communes aux méthodes :
    - chaque méthode qui lit un capteur ou commande le robot exige une
      permission déclarée dans le manifeste, et lève PermissionError
      (celle de ce module) sans elle ;
    - les méthodes de mouvement (move_arms(), move_legs(), move_all(),
      move_to_pose() et walk()) lèvent EstopActiveError si un e-stop est
      actif ; stop_walking() et emergency_stop() ne font pas ce contrôle ;
    - un service absent du robot (pas de caméra, de micro, de jambes ou
      de mémoire persistante) n'est pas une erreur : la méthode rend une
      réponse vide ou explicite (None, [], False, ou un texte qui le dit),
      le plus souvent avec une ligne dans le journal de mission.
    """

    def __init__(
        self,
        robot: NormalizedRobotHAL,
        watchdog: WatchdogSystem,
        manifest: AppManifest,
        _session_token: Optional[_RuntimeSessionToken] = None,
        _runtime_token_holder: Optional[_RuntimeSessionToken] = None,
    ):
        if _runtime_token_holder is None or not _runtime_token_holder._verify(_session_token):
            raise UnauthorizedContextError(
                "RobotContext ne peut être construit que par "
                "PlayStoreRuntime._make_ctx() — voir UnauthorizedContextError. "
                "Pour un test qui a besoin d'un RobotContext isolé, sans "
                "PlayStoreRuntime complet, voir RobotContext.for_testing()."
            )
        self._robot    = robot
        self._watchdog = watchdog
        self._manifest = manifest
        self._log_entries: list[str] = []
        self._taille_journal = 0
        self._journal_tronque = False
        self._t0 = time.monotonic()
        # Services système, injectés par le runtime (voir
        # _attach_services()). None signifie que le service n'existe pas
        # sur ce robot : l'API le dit clairement plutôt que d'échouer de
        # façon obscure.
        self._perception = None
        self._locomotion = None
        self._speech = None
        self._memory = None
        self._navigation = None
        self._manipulation = None
        # Vrai une fois le hook terminé (voir _fermer()) : plus aucune action
        # longue lancée par ce contexte ne continue.
        self._ferme = False
        # Cycle de vie (voir axis_os/store/sdk/lifecycle.py) : arrêt demandé
        # par un humain ou par l'OS, progression rapportée, interface
        # humaine (côté OS), thread des événements (créé au premier on_*()).
        self._arret_demande: Optional[str] = None
        self._progression: list = []
        self._humain = None
        self._evenements: Optional[EventPoller] = None
        # Flux de consignes ouvert (TargetStreamEngine), un à la fois.
        self._flux: Optional[TargetStreamEngine] = None
        self._flux_id = 0
        # Synthèse vocale (ISpeechSynthesizer, côté OS) et réglages choisis
        # par l'utilisateur pour cette app (voir _set_voice, _set_settings).
        self._voix = None
        self._reglages: Dict[str, Any] = {}
        # Animation en cours (AnimationPlayer), une à la fois.
        self._animation: Optional[AnimationPlayer] = None
        # Horloge du robot : l'heure réelle par défaut, remplacée par
        # l'heure simulée en simulation (voir _set_clock() et now()).
        self._clock = time.monotonic
        # Groupes d'articulations déclarés par la HAL (bras, jambes ou
        # autre), résolus à la première utilisation (voir _group_of()).
        self._joint_groups: Optional[Dict[str, JointGroup]] = None
        # Profil d'exécution de l'OS (voir robot_info()) et échéance du
        # hook en cours sur l'horloge du robot (voir safety_status()),
        # posés par le runtime.
        self._profil_nom: Optional[str] = None
        self._echeance: Optional[float] = None

    @classmethod
    def for_testing(
        cls, robot: NormalizedRobotHAL, watchdog: WatchdogSystem, manifest: AppManifest,
    ) -> "RobotContext":
        """
        Construit un RobotContext sans passer par un PlayStoreRuntime
        complet. Le nom explicite `for_testing` rend chaque usage visible
        et intentionnel dans le code appelant : ce n'est jamais un
        contournement accidentel de UnauthorizedContextError.

        Réservé aux tests unitaires qui veulent isoler RobotContext sans
        construire tout un PlayStoreRuntime. Jamais destiné au code de
        production ni à une app réelle, qui doivent toujours passer par
        PlayStoreRuntime.install() puis run_mission().
        """
        token = _RuntimeSessionToken()
        return cls(robot, watchdog, manifest, _session_token=token, _runtime_token_holder=token)

    def get_joint_state(self, joint_id: str) -> Optional[ActuatorFeedback]:
        """État mesuré d'une articulation (position, vitesse, couple…), ou
        None si `joint_id` est inconnu. Permission : ACTUATORS_READ."""
        self._require(Permission.ACTUATORS_READ)
        return self._robot.get_full_state().get(joint_id)

    def get_full_state(self) -> Dict[str, ActuatorFeedback]:
        """État mesuré de toutes les articulations, indexé par identifiant
        d'articulation. Permission : ACTUATORS_READ."""
        self._require(Permission.ACTUATORS_READ)
        return self._robot.get_full_state()

    def get_imu(self):
        """Centrale inertielle du robot (IMU : le capteur qui mesure
        l'orientation et les accélérations du corps), telle que la fournit
        la HAL. Permission : SENSORS_IMU."""
        self._require(Permission.SENSORS_IMU)
        return self._robot.get_imu()

    def _group_of(self, joint_id: str) -> JointGroup:
        """Groupe d'une articulation (bras, jambes ou autre), tel que le
        déclare le driver du robot (le pilote logiciel propre à chaque
        modèle de robot).

        Pourquoi : le SDK devinait autrefois le groupe à partir de
        mots-clés anglais dans le nom de l'articulation (« shoulder »,
        « knee »…), ce qui supposait un humanoïde nommé à l'anglaise (voir
        JointGroup dans axis_os/hal/core/types.py). Il le demande
        désormais à la HAL, qui seule sait ce que son robot possède. Pour
        un driver qui ne déclare pas encore ses groupes, ou une HAL qui ne
        fournit pas la définition de ses articulations, il se rabat sur
        infer_joint_group() (l'ancienne déduction par mots-clés), afin de
        ne rien casser pendant la transition.

        Les groupes sont lus une fois, au premier appel, puis gardés.
        """
        if self._joint_groups is None:
            groups: Dict[str, JointGroup] = {}
            try:
                for jdef in self._robot.get_joint_definitions():
                    groups[jdef.joint_id] = (
                        jdef.group if jdef.group is not None
                        else infer_joint_group(jdef.joint_id)
                    )
            except Exception:       # noqa: BLE001 — HAL sans définitions
                pass
            self._joint_groups = groups
        g = self._joint_groups.get(joint_id)
        return g if g is not None else infer_joint_group(joint_id)

    def _route(self, commands: Dict[str, ActuatorCommand],
               group: JointGroup) -> Dict[str, ActuatorCommand]:
        """Ne garde que les commandes destinées aux articulations du
        groupe `group` ; les autres sont écartées sans erreur."""
        return {jid: cmd for jid, cmd in commands.items()
                if self._group_of(jid) is group}

    # ---- Services système -------------------------------------------

    def _attach_services(self, perception=None, locomotion=None,
                         speech=None, memory=None, navigation=None,
                         manipulation=None) -> None:
        """Injection des services par le runtime.

        Méthode privée (préfixe _), donc hors de la surface publique que
        compare le test de parité (test/test_context_parity.py). Ce n'est
        pas une API d'app : une app ne choisit pas ses services, elle les
        reçoit."""
        self._perception = perception
        self._locomotion = locomotion
        self._speech = speech
        self._memory = memory
        self._navigation = navigation
        self._manipulation = manipulation

    def _fermer(self) -> None:
        """Appelée par le runtime à la fin de chaque hook, même si le
        processus de l'app a été tué : une action longue encore servie
        (go_to, move_to_pose…) s'arrête au pas suivant."""
        self._ferme = True
        if self._evenements is not None:
            self._evenements.stop()
        if self._animation is not None:
            self._animation.stop()
        if self._flux is not None:
            # Arrêt sûr du flux (la marche ralentit puis s'arrête), sans
            # attendre : son fil termine seul.
            self._flux.close(attendre_s=0.0)

    def _demander_arret(self, source: str) -> None:
        """Appelée par le runtime (PlayStoreRuntime.request_stop) : un
        humain ou l'OS demande l'arrêt de la mission. Coopératif :
        should_stop() devient vrai, et les actions longues en cours
        (go_to, pick…) rendent la main avec CANCELLED."""
        if source not in STOP_SOURCES:
            raise ValueError(f"source d'arrêt inconnue : {source!r}")
        self._arret_demande = source

    def _set_voice(self, voix) -> None:
        """Injection par le runtime de la synthèse vocale (côté OS)."""
        self._voix = voix

    def _set_settings(self, reglages: Dict[str, Any]) -> None:
        """Injection par le runtime des réglages choisis par l'utilisateur
        pour cette app (déjà validés contre sa déclaration)."""
        self._reglages = dict(reglages)

    def _set_human_interface(self, humain) -> None:
        """Injection par le runtime de l'interface humaine (IHumanInterface),
        côté OS seulement. None : aucun humain pour répondre."""
        self._humain = humain

    def _motif_interruption(self) -> Optional[tuple]:
        """(code, message) si une action longue doit s'interrompre : arrêt
        d'urgence, hook terminé ou arrêt demandé ; None sinon."""
        if self._watchdog.estop_fired or self._robot.estop_active:
            return RefusalCode.ESTOP_ACTIVE, "arrêt d'urgence"
        if self._ferme:
            return RefusalCode.CANCELLED, "fin du hook de l'app"
        if self._arret_demande is not None:
            return RefusalCode.CANCELLED, f"arrêt demandé ({self._arret_demande})"
        return None

    def _motif_arret(self) -> Optional[str]:
        """Pourquoi la mission doit s'arrêter : « estop », « human » ou
        « os » (hook terminé, arrêt demandé par l'OS, ou moins de
        STOP_MARGIN_S restantes au hook) ; None sinon."""
        if self._watchdog.estop_fired or self._robot.estop_active:
            return "estop"
        if self._arret_demande is not None:
            return self._arret_demande
        if self._ferme:
            return "os"
        if self._echeance is not None and self._echeance - self.now() < STOP_MARGIN_S:
            return "os"
        return None

    def _set_os_info(self, profil_nom: Optional[str]) -> None:
        """Injection par le runtime du nom de son profil (SIMULATION,
        DEVELOPPEMENT, PRODUCTION) : privée, hors de la surface publique."""
        self._profil_nom = profil_nom

    def _set_deadline(self, echeance: Optional[float]) -> None:
        """Injection par le runtime de l'échéance du hook en cours (heure du
        robot à laquelle le quota de durée est atteint)."""
        self._echeance = echeance

    def _set_clock(self, clock) -> None:
        """Injection de l'horloge par le runtime : privée, hors de la
        surface publique. Sans horloge fournie, on revient à l'heure
        réelle (time.monotonic)."""
        self._clock = clock or time.monotonic

    def now(self) -> float:
        """Heure du robot, en secondes (l'heure simulée en simulation).

        Pourquoi ne pas mesurer le temps avec time.monotonic() : une app
        qui écoutait « deux secondes » et marchait « trois secondes » en
        temps réel (celui de la montre), dans une simulation ralentie par
        le rendu vidéo, prenait ses décisions sur une perception décalée
        et perdait le colis qu'elle visait.

        À utiliser à la place de time.monotonic() pour mesurer une durée
        dans une app. En simulation, c'est l'heure simulée ; sur le vrai
        robot, l'heure réelle. Le même code se comporte donc de la même
        façon dans les deux cas. C'est l'équivalent de l'option
        use_sim_time de ROS (Robot Operating System, le cadre logiciel de
        référence en robotique), qui fait suivre aux programmes l'horloge
        du simulateur.
        """
        return float(self._clock())

    def sleep(self, secondes: float) -> None:
        """Attend `secondes` du temps du robot (voir now()).

        Sur l'heure réelle, c'est un simple time.sleep(). Sur une horloge
        simulée, l'attente suit cette horloge (vérifiée toutes les 2 ms) :
        si la simulation est ralentie, l'attente dure plus longtemps en
        temps réel. Une durée négative compte pour zéro."""
        if self._clock is time.monotonic:
            time.sleep(max(0.0, secondes))
            return
        fin = self._clock() + max(0.0, secondes)
        while self._clock() < fin:
            time.sleep(0.002)

    def remember(self, key: str, value) -> bool:
        """Enregistre une donnée qui survit au redémarrage du robot.

        Rend True si la donnée est enregistrée. Rend False si le robot n'a
        pas de mémoire persistante, ou si le service de mémoire refuse, par
        exemple parce que le quota de l'app est atteint
        (MemoryService.MAX_CLES_PAR_APP clés, MAX_OCTETS_PAR_APP octets) ;
        la raison est alors notée dans le journal. Permission :
        STORAGE_PERSISTENT.

        L'espace de stockage est celui de cette app : l'identifiant
        utilisé est celui de son manifeste, qu'elle ne choisit pas. Une
        app ne peut donc, par cette API, ni lire ni écraser les données
        d'une autre, même en devinant sa clé. Ce cloisonnement est logique
        et n'est pas encore imposé par le système d'exploitation : voir
        docs/isolation_apps.md.
        """
        self._require(Permission.STORAGE_PERSISTENT)
        if self._memory is None:
            self.log("remember : ce robot n'a pas de mémoire persistante.")
            return False
        r = self._memory.app_store(self._manifest.app_id, key, value)
        if not r.ok:
            self.log(f"remember : {r.reason}")
        return bool(r.ok)

    def recall(self, key: str, default=None):
        """Relit une donnée enregistrée par remember(), ou rend `default`
        si elle est absente.

        Une clé inconnue est un cas normal (premier démarrage, mémoire
        effacée), pas une erreur. Un robot sans mémoire persistante rend
        aussi `default`. Permission : STORAGE_PERSISTENT.
        """
        self._require(Permission.STORAGE_PERSISTENT)
        if self._memory is None:
            return default
        r = self._memory.app_load(self._manifest.app_id, key)
        return r.value if r.ok else default

    def listen(self, duration_s: float = 3.0):
        """Écoute pendant `duration_s` secondes et rend une intention
        reconnue, ou None.

        Pourquoi une intention et jamais un mouvement : une phrase ne
        commande pas un moteur, elle demande quelque chose à du code
        responsable. L'objet Intent rendu porte un nom d'action et des
        paramètres ; c'est l'app qui décide ce que l'intention déclenche.

        None est un résultat normal et fréquent : silence, bruit, phrase
        hors du vocabulaire déclaré (voir declare_intent()), ou robot
        sans service de parole. Une app doit traiter ce cas comme
        ordinaire, pas comme une erreur. Seules les intentions déclarées
        par cette app peuvent lui revenir. Permission : SENSORS_MICROPHONE.
        """
        self._require(Permission.SENSORS_MICROPHONE)
        if self._speech is None:
            self.log("listen : ce robot n'a pas de service de parole.")
            return None
        r = self._speech.listen(duration_s, app_id=self._manifest.app_id)
        if r.reason:
            self.log(f"listen : {r.reason}")
        return r.value if r.ok else None

    def declare_intent(self, name: str, keywords, params=None,
                       description: str = "") -> bool:
        """Déclare une intention que cette app sait traiter.

        C'est le modèle des intentions (« intents ») d'Android : l'app
        annonce ce qu'elle sait faire, l'OS choisit qui appeler. Installer
        une app ajoute du vocabulaire au robot, sans réentraîner aucun
        modèle.

        `keywords` est une liste de groupes de synonymes, tous requis pour
        que l'intention corresponde ; pour `params` et `description`, voir
        IntentDeclaration (axis_os/services/speech/service.py).

        Rend True si l'intention est enregistrée. Rend False si le robot
        n'a pas de service de parole, si cette app a déjà déclaré ce nom
        d'intention ou si elle a atteint son plafond
        (IntentRegistry.MAX_INTENTIONS) ; la raison est notée dans le
        journal. Les intentions d'une app sont rangées sous son app_id :
        une autre app ne peut ni les recevoir ni en réserver le nom.
        Permission : SENSORS_MICROPHONE.
        """
        self._require(Permission.SENSORS_MICROPHONE)
        if self._speech is None:
            return False
        from axis_os.services.speech.service import IntentDeclaration
        try:
            self._speech.registry.register(IntentDeclaration(
                name=name, keywords=keywords, params=params or {},
                description=description), app_id=self._manifest.app_id)
            return True
        except ValueError as exc:
            self.log(f"declare_intent : {exc}")
            return False

    def look_for(self, query: Optional[str] = None,
                 frame: str = "base") -> list:
        """Cherche un objet décrit en langage naturel (par exemple
        « a red box »).

        Rend une liste de Detection, triées par confiance. Chacune porte
        une direction dans le repère du robot, et une position 3D si une
        caméra de profondeur est disponible ; sinon None, jamais une
        distance inventée.

        `frame` choisit le repère (le système de coordonnées) : "base"
        (défaut), lié au robot, pour se diriger vers un objet ; "world",
        fixe, pour mémoriser sa position. Un repère lié au robot change
        quand le robot bouge : y enregistrer un inventaire n'a pas de
        sens (une app d'inventaire a ainsi fusionné deux colis situés à
        90° l'un de l'autre, voir docs/CREER_UNE_APP.md).

        Liste vide si le service de perception est absent : un robot sans
        caméra n'est pas une erreur de programmation. Liste vide aussi si
        la perception échoue (la raison est notée dans le journal).
        Permission : SENSORS_CAMERA.
        """
        self._require(Permission.SENSORS_CAMERA)
        if self._perception is None:
            self.log("look_for : aucun service de perception sur ce robot.")
            return []
        from axis_os.hal.core.frames import FRAME_BASE, FRAME_WORLD
        repere = FRAME_WORLD if frame == "world" else FRAME_BASE
        r = self._perception.find(query, frame=repere)
        if not r.ok:
            self.log(f"look_for : {r.reason}")
            return []
        if r.reason:
            self.log(f"look_for : {r.reason}")
        return r.value or []

    def describe_scene(self) -> str:
        """Ce que le robot voit, en clair, sous forme de texte.

        Sans perception, ou si elle échoue, rend un texte qui le dit
        plutôt qu'une erreur. Permission : SENSORS_CAMERA."""
        self._require(Permission.SENSORS_CAMERA)
        if self._perception is None:
            return "aucune perception disponible"
        r = self._perception.describe_scene()
        return r.value if r.ok else f"perception indisponible : {r.reason}"

    # ---- État, capacités et diagnostic --------------------------------

    # Profils de l'OS, tels que l'app les lit (noms anglais de l'API).
    _PROFILS = {"SIMULATION": "simulation", "DEVELOPPEMENT": "development",
                "PRODUCTION": "production"}

    def _services(self) -> Dict[str, str]:
        """Services système présents sur ce robot (AVAILABLE ou ABSENT)."""
        presents = {"locomotion": self._locomotion, "perception": self._perception,
                    "speech_in": self._speech, "memory": self._memory,
                    "navigation": self._navigation, "manipulation": self._manipulation,
                    "speech_out": self._voix}
        services = {nom: AVAILABLE if s is not None else ABSENT
                    for nom, s in presents.items()}
        services["battery"] = AVAILABLE if self._lire_batterie() is not None else ABSENT
        return services

    def _lire_batterie(self):
        """BatteryState mesuré, ou None (inconnue, ou HAL sans batterie)."""
        lire = getattr(self._robot, "get_battery", None)
        return lire() if lire is not None else None

    def _capacites(self) -> Dict[str, str]:
        """État de chaque capacité matérielle : AVAILABLE, DEGRADED, ou
        ABSENT (non déclarée, ou hors service en ce moment)."""
        from axis_os.hal.core.capabilities import CapabilityStatus, RobotCapability
        try:
            caps = self._robot.get_capabilities()
        except Exception:       # noqa: BLE001 — HAL sans capacités déclarées
            caps = None
        etats = {}
        for c in RobotCapability:
            s = caps.status_of(c) if caps is not None else None
            etats[c.name.lower()] = (AVAILABLE if s is CapabilityStatus.AVAILABLE
                                     else DEGRADED if s is CapabilityStatus.DEGRADED
                                     else ABSENT)
        return etats

    def _batterie(self) -> BatteryInfo:
        b = self._lire_batterie()
        if b is None:
            return BatteryInfo(known=False)
        return BatteryInfo(known=True, percent=float(b.percent), charging=b.charging,
                           voltage=b.voltage)

    def robot_info(self) -> RobotInfo:
        """Le robot qui exécute l'app : modèle, fabricant, version d'AXIS-OS
        et de l'API du SDK, profil (« simulation », « development » ou
        « production »), capacités matérielles et leur état (AVAILABLE,
        DEGRADED, ABSENT), services système présents, limites utiles et
        batterie (inconnue si le driver ne la mesure pas : jamais une
        valeur inventée).

        Limites : vitesse de marche maximale vers l'avant et sur le côté
        (m/s), rotation maximale (rad/s), toutes celles du service de
        locomotion (None sans locomotion), et le rapport de vitesse
        articulaire du manifeste (max_velocity_ratio).

        Permission : SYSTEM_INFO. Servie aussi pendant un arrêt d'urgence :
        c'est de la lecture seule."""
        self._require(Permission.SYSTEM_INFO)
        import axis_os
        info = self._robot.get_robot_info()
        twist = getattr(self._locomotion, "max_twist", None)
        limits = {
            "max_walk_speed_mps": getattr(twist, "vx", None),
            "max_lateral_speed_mps": getattr(twist, "vy", None),
            "max_turn_rate_radps": getattr(twist, "wz", None),
            "max_velocity_ratio": self._manifest.max_velocity_ratio,
        }
        return RobotInfo(
            model=str(info.get("model", "")), manufacturer=str(info.get("manufacturer", "")),
            os_version=axis_os.__version__, sdk_api_version=API_VERSION,
            profile=self._PROFILS.get(self._profil_nom or "", "unknown"),
            capabilities=self._capacites(), services=self._services(),
            limits=limits, battery=self._batterie())

    def has_capability(self, name: str) -> bool:
        """Vrai si le robot peut servir `name` maintenant : une capacité
        matérielle (« legs », « camera », « speaker »… : voir
        robot_info().capabilities) déclarée ET disponible, ou un service
        système présent (« locomotion », « perception », « battery »… :
        voir robot_info().services).

        Pour désactiver proprement une fonction facultative au lieu de
        planter. Un nom inconnu lève InvalidArgumentError (une faute de
        frappe ne doit pas désactiver une fonction en silence).
        Permission : SYSTEM_INFO."""
        self._require(Permission.SYSTEM_INFO)
        nom = require_text("name", name, 64).strip().lower()
        capacites, services = self._capacites(), self._services()
        if nom in capacites:
            return capacites[nom] == AVAILABLE
        if nom in services:
            return services[nom] == AVAILABLE
        raise InvalidArgumentError(
            f"capacité inconnue : {nom!r} — capacités : {sorted(capacites)} ; "
            f"services : {sorted(services)}")

    def safety_status(self) -> SafetyStatus:
        """État de sécurité au moment de l'appel : arrêt d'urgence, gardien
        de proximité (« clear », « slowing », « stopped » ou
        « unavailable »), chute détectée (None si le robot ne marche pas),
        et quotas restants : durée du hook en cours (secondes du robot),
        clés et octets de mémoire persistante.

        Permission : SYSTEM_INFO. Servie aussi pendant un arrêt d'urgence."""
        self._require(Permission.SYSTEM_INFO)
        loco = self._locomotion
        proximite, distance, raison = "unavailable", None, ""
        if loco is not None and getattr(loco, "is_guarded", False):
            v = getattr(loco, "dernier_verdict", None)
            if v is not None:
                proximite = ("stopped" if v.facteur <= 0.0 else
                             "slowing" if v.facteur < 1.0 else "clear")
                distance = (None if v.distance is None or not math.isfinite(v.distance)
                            else float(v.distance))
                raison = str(v.raison)[:200]
        chute = None
        if loco is not None and hasattr(loco, "state"):
            chute = getattr(loco.state, "name", "") == "FALLEN"
        reste = None if self._echeance is None else max(0.0, self._echeance - self.now())
        cles = octets = None
        if self._memory is not None and hasattr(self._memory, "app_usage"):
            r = self._memory.app_usage(self._manifest.app_id)
            if r.ok:
                n, taille = r.value
                cles = max(0, self._memory.MAX_CLES_PAR_APP - n)
                octets = max(0, self._memory.MAX_OCTETS_PAR_APP - taille)
        return SafetyStatus(
            estop_active=bool(self._watchdog.estop_fired or self._robot.estop_active),
            proximity=proximite, proximity_distance_m=distance, proximity_reason=raison,
            fall_detected=chute, time_remaining_s=reste,
            storage_keys_remaining=cles, storage_bytes_remaining=octets)

    # Version de l'API du SDK que sert cet OS (SemVer, voir
    # axis_os/store/sdk/version.py et AppManifest.min_sdk_version).
    api_version: str = API_VERSION

    # ---- Services de haut niveau : navigation, manipulation, repos -----

    # Durée maximale d'une action longue (go_to, pick, place). Dans le
    # processus de l'app, elle est aussi bornée par le temps qui reste au
    # hook (voir HALBridgeServer.traiter).
    MAX_ACTION_S = 600.0
    # Pas d'attente d'une action longue (secondes du robot).
    _PAS_ATTENTE_S = 0.02

    def go_to(self, x: float, y: float, theta: Optional[float] = None, *,
              timeout_s: float) -> ActionResult:
        """Va au point (x, y) de la carte (mètres, repère monde), et s'y
        oriente selon `theta` (radians) s'il est donné. BLOQUANT : rend la
        main à l'arrivée, ou dès que l'issue est connue.

        Le service de navigation planifie un chemin qui contourne les
        obstacles de la carte, puis le suit par des consignes de vitesse au
        service de locomotion : la marche est donc bornée comme celle de
        walk() (vitesse maximale du robot) et plafonnée par le gardien de
        proximité, qui ralentit puis arrête le robot devant une personne.
        Un obstacle immobile sur le chemin est contourné si un autre
        passage existe.

        Rend un ActionResult dont `status` vaut :
          - ARRIVED : arrivé ;
          - BLOCKED : pas de chemin, ou plus de passage (code BLOCKED) ;
          - REFUSED : arrêt d'urgence survenu (ESTOP_ACTIVE), annulation
            (CANCELLED), robot à terre (SAFETY_LIMIT), pas de navigation
            sur ce robot (CAPABILITY_MISSING) ;
          - TIMEOUT : délai dépassé (code TIMEOUT, ou PROXIMITY_GUARD si le
            gardien tenait le robot arrêté à ce moment).
        Le robot est arrêté (en équilibre) pour toute issue autre
        qu'ARRIVED.

        Lève InvalidArgumentError (coordonnée ou délai non fini, délai hors
        de ]0, MAX_ACTION_S]) et EstopActiveError si un arrêt d'urgence est
        déjà actif. Permission : NAVIGATION_GOTO."""
        self._require(Permission.NAVIGATION_GOTO)
        x = require_number("x", x, mini=-1e4, maxi=1e4)
        y = require_number("y", y, mini=-1e4, maxi=1e4)
        if theta is not None:
            theta = require_number("theta", theta, mini=-2 * math.pi, maxi=2 * math.pi)
        delai = require_number("timeout_s", timeout_s, mini=0.0, maxi=self.MAX_ACTION_S,
                               strict_min=True)
        self._require_no_estop()
        nav = self._navigation
        if nav is None:
            return refused("REFUSED", RefusalCode.CAPABILITY_MISSING,
                           "pas de service de navigation sur ce robot")
        from axis_os.services.navigation.service import EtatNavigation, Pose2D
        # cap None : le suiveur ne s'aligne sur aucun cap à l'arrivée.
        r = nav.aller_vers(Pose2D(x, y, theta))
        if not r.ok:
            return refused("BLOCKED", RefusalCode.BLOCKED, str(r.reason))
        echeance = self.now() + delai
        while True:
            motif = self._motif_interruption()
            if motif is not None:
                nav.annuler()
                return refused("REFUSED", motif[0], f"navigation interrompue : {motif[1]}")
            if getattr(getattr(self._locomotion, "state", None), "name", "") == "FALLEN":
                nav.annuler()
                return refused("REFUSED", RefusalCode.SAFETY_LIMIT, "chute détectée")
            if nav.etat is EtatNavigation.ARRIVE:
                return ActionResult(ok=True, status="ARRIVED", message="arrivé")
            if nav.etat is EtatNavigation.ECHEC:
                nav.annuler()
                return refused("BLOCKED", RefusalCode.BLOCKED, nav.raison or "pas de chemin")
            if self.now() >= echeance:
                v = getattr(self._locomotion, "dernier_verdict", None)
                garde = v is not None and v.facteur <= 0.0
                nav.annuler()
                return refused("TIMEOUT",
                               RefusalCode.PROXIMITY_GUARD if garde else RefusalCode.TIMEOUT,
                               f"délai de {delai:g} s dépassé" +
                               (f" — {v.raison}" if garde else ""))
            self.sleep(self._PAS_ATTENTE_S)

    def _attendre_manipulation(self, fini, echec, delai: float, geste: str) -> ActionResult:
        """Attend l'issue d'un geste du service de manipulation : `fini()`
        vrai -> réussite ; `echec()` rend un message -> échec. Ne commande
        rien sous arrêt d'urgence."""
        manip = self._manipulation
        echeance = self.now() + delai
        while True:
            motif = self._motif_interruption()
            if motif is not None:
                return refused("REFUSED", motif[0], f"{geste} interrompu : {motif[1]}")
            if fini():
                return ActionResult(ok=True, status="DONE", message=f"{geste} terminé")
            msg = echec()
            if msg:
                return refused("FAILED", self._code_refus_manipulation(msg), msg)
            if self.now() >= echeance:
                return refused("TIMEOUT", RefusalCode.TIMEOUT,
                               f"{geste} : délai de {delai:g} s dépassé "
                               f"(état {getattr(manip.etat, 'name', manip.etat)})")
            self.sleep(self._PAS_ATTENTE_S)

    @staticmethod
    def _code_refus_manipulation(raison: str) -> RefusalCode:
        """Code d'un refus du service de manipulation, d'après sa raison."""
        if raison.startswith(("prise refusée", "plateau trop haut", "dépose impossible")):
            return RefusalCode.OUT_OF_REACH
        if raison.startswith("exemption refusée"):
            return RefusalCode.SAFETY_LIMIT
        if raison.startswith("occupé"):
            return RefusalCode.BUSY
        if raison.startswith("service non démarré"):
            return RefusalCode.CAPABILITY_MISSING
        return RefusalCode.FAILED

    def pick(self, target, *, size_m=None, timeout_s: float = 15.0) -> ActionResult:
        """Prend un objet à deux mains. BLOQUANT jusqu'à l'issue du geste.

        `target` : une Detection vue par look_for(..., frame="world"), avec
        sa position 3D, et alors `size_m` = (largeur, profondeur, hauteur)
        en mètres, que l'image ne donne pas ; ou une ObjectSpec (centre et
        taille, repère monde).

        Le service de manipulation, seul écrivain des bras, ne serre que si
        l'objet est entre les mains (sinon OUT_OF_REACH : c'est à l'app
        de placer le robot, par go_to), demande au gardien de proximité une
        exemption bornée autour de l'objet, puis vérifie que l'objet a suivi
        avant de le dire tenu. Chaque consigne de bras passe par le
        garde-fou des articulations.

        `status` : HELD (tenu), REFUSED (OUT_OF_REACH, BUSY, SAFETY_LIMIT,
        CAPABILITY_MISSING, ESTOP_ACTIVE, CANCELLED), FAILED (prise non
        confirmée : les bras se rouvrent progressivement) ou TIMEOUT.

        Lève InvalidArgumentError (cible ou taille invalide, délai hors de
        ]0, MAX_ACTION_S]) et EstopActiveError si un arrêt d'urgence est
        actif. Permission : MANIPULATION."""
        self._require(Permission.MANIPULATION)
        spec = object_from_target(target, size_m)
        delai = require_number("timeout_s", timeout_s, mini=0.0, maxi=self.MAX_ACTION_S,
                               strict_min=True)
        self._require_no_estop()
        manip = self._manipulation
        if manip is None:
            return refused("REFUSED", RefusalCode.CAPABILITY_MISSING,
                           "pas de service de manipulation sur ce robot")
        from axis_os.services.manipulation.service import Etat, Objet
        largeur, profondeur, hauteur = spec.size
        r = manip.prendre(Objet(spec.center, largeur, profondeur, hauteur))
        if not r.ok:
            return refused("REFUSED", self._code_refus_manipulation(str(r.reason)),
                           str(r.reason))
        res = self._attendre_manipulation(
            lambda: manip.etat is Etat.TENU,
            lambda: manip.raison or "prise non confirmée" if manip.etat is Etat.ECHEC else "",
            delai, "prise")
        if res.ok:
            return ActionResult(ok=True, status="HELD", message="objet tenu")
        if res.status in ("FAILED", "TIMEOUT") and not self._motif_interruption():
            # Bras rouverts progressivement : ne jamais rester serrés sur un
            # objet mal pris.
            manip.reinitialiser()
        return res

    def place(self, target=None, *, timeout_s: float = 15.0) -> ActionResult:
        """Pose l'objet tenu. BLOQUANT jusqu'à l'issue du geste.

        `target` : None pour redescendre à hauteur de port et ouvrir les
        bras ; un nombre, la hauteur du plateau (m, repère monde) ; une
        ObjectSpec du plateau (sa face supérieure) ; ou une Detection vue
        dans le repère monde (la hauteur du point vu). Avec un plateau,
        l'objet descend jusqu'à l'effleurer avant que les bras s'ouvrent :
        jamais lâché de haut.

        `status` : PLACED, REFUSED (OUT_OF_REACH : plateau trop haut ;
        FAILED : rien en main ; CAPABILITY_MISSING, ESTOP_ACTIVE,
        CANCELLED), FAILED (code OUT_OF_REACH : bras au plus bas sans que
        l'objet touche le plateau ; il reste tenu) ou TIMEOUT.

        Lève InvalidArgumentError et EstopActiveError comme pick().
        Permission : MANIPULATION."""
        self._require(Permission.MANIPULATION)
        hauteur = surface_height_from_target(target)
        delai = require_number("timeout_s", timeout_s, mini=0.0, maxi=self.MAX_ACTION_S,
                               strict_min=True)
        self._require_no_estop()
        manip = self._manipulation
        if manip is None:
            return refused("REFUSED", RefusalCode.CAPABILITY_MISSING,
                           "pas de service de manipulation sur ce robot")
        from axis_os.services.manipulation.service import Etat
        r = manip.poser(hauteur)
        if not r.ok:
            return refused("REFUSED", self._code_refus_manipulation(str(r.reason)),
                           str(r.reason))
        res = self._attendre_manipulation(
            lambda: manip.etat is Etat.LIBRE,
            lambda: manip.raison if manip.etat is Etat.TENU and manip.raison else "",
            delai, "pose")
        if res.ok:
            return ActionResult(ok=True, status="PLACED", message="objet posé")
        return res

    def rest_pose(self, *, duration_s: float = 2.0) -> ActionResult:
        """Ramène le robot dans sa posture de repos sûre, définie par son
        driver (jamais par l'app) : pour le G1, les bras le long du corps.
        Jamais les jambes d'un bipède debout.

        Le geste passe par move_to_pose() : mêmes permissions, mêmes pas
        interpolés sous le garde-fou des articulations, même refus sous
        arrêt d'urgence (EstopActiveError).

        `status` : AT_REST, FAILED (un pas refusé par le garde-fou, code
        SAFETY_LIMIT), REFUSED (CAPABILITY_MISSING : le driver ne déclare
        pas de posture de repos).

        Lève InvalidArgumentError si `duration_s` n'est pas dans [0.5, 30].
        Permissions : celles de move_to_pose() pour les articulations de la
        posture (ACTUATORS_READ et ACTUATORS_WRITE_ARMS pour le G1)."""
        duree = require_number("duration_s", duration_s, mini=0.5, maxi=30.0)
        lire = getattr(self._robot, "get_rest_pose", None)
        pose = lire() if lire is not None else None
        if not pose:
            # Les permissions restent exigées avant de répondre : une app ne
            # doit pas apprendre par ce biais ce qu'elle n'a pas le droit de
            # commander.
            self._require(Permission.ACTUATORS_READ)
            self._require(Permission.ACTUATORS_WRITE_ARMS)
            return refused("REFUSED", RefusalCode.CAPABILITY_MISSING,
                           "ce robot ne déclare pas de posture de repos")
        manip = self._manipulation
        etat = getattr(getattr(manip, "etat", None), "name", "LIBRE")
        if etat not in ("LIBRE", "ECHEC"):
            # Les bras appartiennent au service de manipulation pendant un
            # geste : ne pas lâcher un objet tenu en les ramenant au repos.
            return refused("REFUSED", RefusalCode.BUSY,
                           f"bras occupés par la manipulation (état {etat}) : place() d'abord")
        if self.move_to_pose(dict(pose), duration_s=duree):
            return ActionResult(ok=True, status="AT_REST", message="posture de repos")
        return refused("FAILED", RefusalCode.SAFETY_LIMIT,
                       "un pas vers la posture de repos a été refusé (voir le journal)")

    # ---- Cycle de vie : annulation, progression, événements, humain ------

    def should_stop(self) -> bool:
        """Vrai si la mission doit s'arrêter au plus vite : arrêt demandé
        par un humain ou par l'OS (PlayStoreRuntime.request_stop), moins
        de STOP_MARGIN_S restantes au hook, ou arrêt d'urgence.

        Coopératif : à l'app de ranger (poser l'objet, s'arrêter) et de
        rendre la main. Cela ne remplace pas l'arrêt forcé : l'arrêt
        d'urgence coupe tout de toute façon, et le quota de durée tue le
        processus d'une app qui ne rend pas la main.
        Permission : MISSION_LIFECYCLE."""
        self._require(Permission.MISSION_LIFECYCLE)
        return self._motif_arret() is not None

    def check_cancelled(self) -> None:
        """Lève si la mission doit s'arrêter (voir should_stop()) :
        EstopActiveError sous arrêt d'urgence, MissionCancelled (code
        CANCELLED) sinon. Sans effet si rien n'est demandé. L'OS note la
        mission comme annulée. Permission : MISSION_LIFECYCLE."""
        self._require(Permission.MISSION_LIFECYCLE)
        motif = self._motif_arret()
        if motif == "estop":
            raise EstopActiveError("arrêt d'urgence actif")
        if motif is not None:
            raise MissionCancelled(f"arrêt demandé ({motif})")

    def report_progress(self, fraction: float, message: str = "") -> None:
        """Rapporte l'avancement de la mission : `fraction` dans [0, 1],
        `message` court (coupé à PROGRESS_MESSAGE_MAX caractères, ses
        caractères de contrôle remplacés par des espaces). L'OS le lit par
        PlayStoreRuntime.get_progress() ; seules les
        PROGRESS_HISTORY_MAX dernières valeurs sont gardées.
        Lève InvalidArgumentError (fraction non finie, hors de [0, 1],
        message qui n'est pas un texte). Permission : MISSION_LIFECYCLE."""
        self._require(Permission.MISSION_LIFECYCLE)
        f = require_number("fraction", fraction, mini=0.0, maxi=1.0)
        m = require_text("message", message, PROGRESS_MESSAGE_MAX, allow_empty=True)
        self._progression.append(Progress(f, m, self.now()))
        del self._progression[:-PROGRESS_HISTORY_MAX]

    def _peut_abonner(self, kind: str, threshold: Optional[float] = None) -> bool:
        """Vérifie le droit de s'abonner à un événement (permission, et
        batterie connue pour « low_battery ») ; rend False si l'événement
        ne peut pas se produire sur ce robot."""
        if kind == "estop":
            self._require(Permission.MISSION_LIFECYCLE)
            return True
        if kind == "alert":
            self._require(Permission.WATCHDOG_READ)
            return True
        if kind == "low_battery":
            self._require(Permission.SYSTEM_INFO)
            require_number("threshold_percent", threshold, mini=1.0, maxi=99.0)
            return self._lire_batterie() is not None
        raise InvalidArgumentError(f"événement inconnu : {kind!r}")

    def _etat_evenements(self, since_index: Optional[int]) -> Dict[str, Any]:
        """Ce que le thread des événements lit : arrêt d'urgence, alertes
        depuis `since_index` (avec WATCHDOG_READ), charge de la batterie
        (avec SYSTEM_INFO)."""
        alertes, suivant = [], 0
        if self._manifest.has_permission(Permission.WATCHDOG_READ):
            depuis = 0 if since_index is None else since_index
            vues = self._watchdog.get_recent_alerts(depuis)
            suivant = depuis + len(vues)
            if since_index is not None:
                alertes = [{"kind": a.kind.name, "severity": a.severity.name,
                            "message": a.message, "joint_id": a.joint_id} for a in vues]
        pct = None
        if self._manifest.has_permission(Permission.SYSTEM_INFO):
            b = self._lire_batterie()
            pct = None if b is None else float(b.percent)
        return {"estop": bool(self._watchdog.estop_fired or self._robot.estop_active),
                "alerts": alertes, "next_index": suivant, "battery_percent": pct}

    def _abonner(self, kind: str, callback, threshold: Optional[float] = None) -> None:
        if self._evenements is None:
            self._evenements = EventPoller(self._etat_evenements, self.log)
            if self._ferme:
                self._evenements.stop()
        self._evenements.add(kind, callback, threshold)

    @staticmethod
    def _exiger_appelable(callback) -> None:
        if not callable(callback):
            raise InvalidArgumentError("callback doit être une fonction")

    def on_estop(self, callback) -> None:
        """Appelle `callback()` une fois, dès qu'un arrêt d'urgence survient
        pendant le hook : au plus EventPoller.PERIOD_S secondes après, ou
        aussitôt qu'un appel de l'app est refusé pour cette raison. Dans le
        processus de l'app, il reste alors peu de temps avant qu'il soit
        arrêté (SandboxRunner.DELAI_ARRET_URGENCE_S) : un callback court,
        qui note ou journalise. Le robot est déjà arrêté par l'OS.
        Permission : MISSION_LIFECYCLE."""
        self._exiger_appelable(callback)
        self._peut_abonner("estop")
        self._abonner("estop", callback)

    def on_alert(self, callback) -> None:
        """Appelle `callback(alerte)` pour chaque nouvelle alerte du
        watchdog (dict : kind, severity, message, joint_id), à partir de
        l'abonnement. Fondé sur la même lecture que subscribe_alerts().
        Permission : WATCHDOG_READ."""
        self._exiger_appelable(callback)
        self._peut_abonner("alert")
        self._abonner("alert", callback)

    def on_low_battery(self, callback, *, threshold_percent: float = 20.0) -> bool:
        """Appelle `callback(pourcentage)` quand la charge mesurée passe
        sous `threshold_percent` (une fois par passage). Rend False, sans
        s'abonner, si le driver ne mesure pas la batterie : l'OS n'invente
        pas de charge. Lève InvalidArgumentError si le seuil n'est pas dans
        [1, 99]. Permission : SYSTEM_INFO."""
        self._exiger_appelable(callback)
        if not self._peut_abonner("low_battery", threshold_percent):
            self.log("on_low_battery : batterie inconnue sur ce robot — pas d'abonnement")
            return False
        self._abonner("low_battery", callback, float(threshold_percent))
        return True

    def ask_confirmation(self, question: str, *, timeout_s: float = 30.0) -> ActionResult:
        """Pose une question fermée (oui/non) à l'humain présent, au nom de
        l'app (le nom affiché vient du manifeste, jamais de l'app). BLOQUANT
        jusqu'à la réponse ou au délai.

        `status` : CONFIRMED (oui), DECLINED (non, code HUMAN_DECLINED),
        TIMEOUT (pas de réponse), REFUSED (code HUMAN_UNAVAILABLE : aucun
        humain pour répondre sur ce robot). Seule une réponse explicite
        « oui » vaut confirmation : l'app ne peut jamais répondre à la
        place de l'humain, et l'absence de réponse n'est jamais un oui.

        Lève InvalidArgumentError (question vide ou qui n'est pas un texte,
        délai hors de ]0, CONFIRMATION_MAX_S]) et EstopActiveError sous
        arrêt d'urgence. Permission : HUMAN_CONFIRMATION."""
        self._require(Permission.HUMAN_CONFIRMATION)
        q = require_text("question", question, QUESTION_MAX)
        delai = require_number("timeout_s", timeout_s, mini=0.0, maxi=CONFIRMATION_MAX_S,
                               strict_min=True)
        self._require_no_estop()
        if self._humain is None:
            return refused("REFUSED", RefusalCode.HUMAN_UNAVAILABLE,
                           "aucun humain pour répondre sur ce robot")
        nom = require_text("nom de l'app", self._manifest.name or self._manifest.app_id, 64)
        try:
            reponse = self._humain.confirm(nom, q, delai)
        except Exception as exc:       # noqa: BLE001 — interface en panne : pas de réponse
            return refused("REFUSED", RefusalCode.HUMAN_UNAVAILABLE,
                           f"interface humaine indisponible : {type(exc).__name__}")
        if reponse is True:
            return ActionResult(ok=True, status="CONFIRMED", message="confirmé par l'humain")
        if reponse is False:
            return refused("DECLINED", RefusalCode.HUMAN_DECLINED, "refusé par l'humain")
        return refused("TIMEOUT", RefusalCode.TIMEOUT, f"pas de réponse en {delai:g} s")

    # ---- Flux de consignes (téléopération locale) -----------------------

    def stream_targets(self, kind: str, *, deadman_ms: float = 200.0,
                       max_rate_hz: float = 50.0, max_age_ms: float = 150.0,
                       max_speed: Optional[float] = None) -> TargetStream:
        """Ouvre un flux de consignes (voir axis_os/store/sdk/stream.py) :
        « arms », cibles des articulations des bras ; « walk », vitesses de
        marche (vx, vy, wz). Local seulement (pont de l'app, jamais le
        réseau).

        deadman_ms  : au-delà, sans consigne valide homme mort tenu, arrêt
                      sûr (marche : ralentir puis s'arrêter ; bras : sur
                      place). Dans [50, 1000].
        max_rate_hz : consignes acceptées par seconde au plus (au-delà :
                      RATE_LIMITED). Dans [1, 100].
        max_age_ms  : âge maximal d'une consigne à son arrivée (au-delà :
                      STALE_TARGET). Dans [10, 1000].
        max_speed   : vitesse propre au flux. « arms » : rad/s, dans
                      ]0, 2] (1 par défaut) ; « walk » : fraction de la
                      vitesse maximale du robot, dans ]0, 0.5] (0.3 par
                      défaut). Une consigne au-delà est rejetée
                      (OUT_OF_BOUNDS), jamais rognée.

        Permissions : TELEOP_STREAM, plus celles des commandes remplacées
        (« arms » : ACTUATORS_READ et ACTUATORS_WRITE_ARMS ; « walk » :
        LOCOMOTION_WALK et ACTUATORS_WRITE_LEGS). Lève
        InvalidArgumentError, EstopActiveError, CapabilityMissingError (pas
        de bras ou pas de marche) et BusyError (un flux déjà ouvert)."""
        self._require(Permission.TELEOP_STREAM)
        params = stream_params(kind, deadman_ms, max_rate_hz, max_age_ms, max_speed)
        sid, info = self._flux_ouvrir(params)
        return TargetStream(self, sid, params["kind"], info)

    def _flux_ouvrir(self, params: Dict[str, Any]) -> tuple:
        # Jamais plus de droits que la somme des commandes que le flux
        # remplace : on exige les leurs.
        if params["kind"] == "arms":
            self._require(Permission.ACTUATORS_READ)
            self._require(Permission.ACTUATORS_WRITE_ARMS)
        else:
            self._require(Permission.LOCOMOTION_WALK)
            self._require(Permission.ACTUATORS_WRITE_LEGS)
        self._require_no_estop()
        if self._flux is not None and self._flux.state != "CLOSED":
            raise BusyError("un flux est déjà ouvert : le fermer d'abord")
        if params["kind"] == "arms":
            bras = {d.joint_id: (d.limits.pos_min, d.limits.pos_max)
                    for d in self._robot.get_joint_definitions()
                    if self._group_of(d.joint_id) is JointGroup.MANIPULATION}
            if not bras:
                raise CapabilityMissingError("ce robot n'a pas de bras pilotables")
            moteur = TargetStreamEngine(self, params, joints=bras)
        else:
            if self._locomotion is None:
                raise CapabilityMissingError("pas de service de locomotion sur ce robot")
            bras = {}
            moteur = TargetStreamEngine(self, params, max_twist=self._locomotion.max_twist)
        self._flux_id += 1
        self._flux = moteur
        return self._flux_id, {"joints": sorted(bras), "limits": dict(moteur.limits)}

    def _flux_moteur(self, stream_id: int) -> TargetStreamEngine:
        self._require(Permission.TELEOP_STREAM)
        if self._flux is None or stream_id != self._flux_id:
            raise InvalidArgumentError("flux inconnu (fermé, ou ouvert par un autre contexte)")
        return self._flux

    def _flux_push(self, stream_id: int, cible, deadman: bool, t: Optional[float]):
        moteur = self._flux_moteur(stream_id)
        self._require_no_estop()
        return moteur.push(cible, deadman, t)

    def _flux_resume(self, stream_id: int) -> ActionResult:
        return self._flux_moteur(stream_id).resume()

    def _flux_metrics(self, stream_id: int):
        return self._flux_moteur(stream_id).metrics()

    def _flux_close(self, stream_id: int) -> None:
        self._flux_moteur(stream_id).close()

    # ---- Animations -------------------------------------------------------

    ANIMATION_TIMEOUT_MAX_S = 120.0

    def _modele_robot(self) -> str:
        try:
            return str(self._robot.get_robot_info().get("model", ""))
        except Exception:              # noqa: BLE001 — modèle inconnu : aucune animation compatible
            return ""

    def play_animation(self, animation, *, wait: bool = True,
                       timeout_s: Optional[float] = None) -> ActionResult:
        """Joue une animation (voir axis_os/store/sdk/animation.py) : le nom
        d'une animation fournie avec le robot (« wave », « point » pour le
        G1), un dict au format « axis-animation », ou une Animation. Un
        chemin de fichier n'est pas accepté : l'app ne lit pas les
        fichiers de l'OS ; elle charge les siens avec load_animation().

        Tout est vérifié AVANT de jouer : format, robot compatible,
        articulations, butées, vitesses, accélérations ; une articulation de
        jambe exige la mention « stationary_legs_still », des jambes
        immobiles, le robot à l'arrêt et une posture mesurée identique —
        et l'OS ne la commande jamais. Chaque pas passe ensuite par
        move_arms() ; le geste se met en pause si le gardien de proximité
        arrête le robot.

        wait=True : bloque jusqu'à la fin (status PLAYED), ou au délai
        `timeout_s` (par défaut la durée du geste plus 2 s, au plus
        ANIMATION_TIMEOUT_MAX_S ; au-delà, le geste est arrêté : TIMEOUT).
        wait=False : rend la main aussitôt (PLAYING) ; stop_animation()
        l'arrête. Autres issues : REFUSED (ESTOP_ACTIVE, CANCELLED,
        SAFETY_LIMIT, BUSY), STOPPED, FAILED (SAFETY_LIMIT : un pas refusé
        par le garde-fou).

        Permissions : ANIMATION_PLAY, plus celles de move_arms()
        (ACTUATORS_READ, ACTUATORS_WRITE_ARMS). Lève InvalidArgumentError
        (animation invalide pour ce robot, avec la liste des problèmes) et
        EstopActiveError."""
        self._require(Permission.ANIMATION_PLAY)
        if not isinstance(wait, bool):
            raise InvalidArgumentError("wait doit être un booléen")
        if timeout_s is not None:
            timeout_s = require_number("timeout_s", timeout_s, mini=0.0,
                                       maxi=self.ANIMATION_TIMEOUT_MAX_S, strict_min=True)
        modele = self._modele_robot()
        if isinstance(animation, str):
            anim = load_library_animation(animation, modele)
        elif isinstance(animation, Animation):
            # Revalidée depuis ses données : un objet construit à la main
            # ne passe pas sans contrôle.
            anim = load_animation(animation.to_json())
        elif isinstance(animation, dict):
            anim = load_animation(animation)
        else:
            raise InvalidArgumentError("animation : un nom d'animation du robot, un dict au "
                                       "format axis-animation, ou une Animation")
        self._require(Permission.ACTUATORS_READ)
        self._require(Permission.ACTUATORS_WRITE_ARMS)
        self._require_no_estop()
        defs = self._robot.get_joint_definitions()
        groupes = {d.joint_id: self._group_of(d.joint_id) for d in defs}
        if JointGroup.MANIPULATION not in groupes.values():
            return refused("REFUSED", RefusalCode.CAPABILITY_MISSING,
                           "ce robot n'a pas de bras à animer")
        check_animation(anim, defs, robot_model=modele)
        jambes = [j for j in anim.joints if groupes.get(j) is JointGroup.LOCOMOTION]
        if jambes:
            if getattr(self._locomotion, "is_moving", False):
                return refused("REFUSED", RefusalCode.SAFETY_LIMIT,
                               "le robot marche : une animation qui décrit les jambes "
                               "exige le robot à l'arrêt")
            etat = self._robot.get_full_state()
            ecarts = [j for j in jambes
                      if abs(etat[j].position - anim.position_at(j, 0.0)) > LEG_MATCH_TOL]
            if ecarts:
                return refused("REFUSED", RefusalCode.SAFETY_LIMIT,
                               f"posture mesurée des jambes différente de celle que décrit "
                               f"l'animation : {ecarts[:3]}")
        if self._animation is not None and not self._animation.done:
            return refused("REFUSED", RefusalCode.BUSY, "une animation est déjà en cours")
        if self._flux is not None and self._flux.kind == "arms" and self._flux.state != "CLOSED":
            return refused("REFUSED", RefusalCode.BUSY, "un flux de consignes des bras est ouvert")
        etat_manip = getattr(getattr(self._manipulation, "etat", None), "name", "LIBRE")
        if etat_manip not in ("LIBRE", "ECHEC"):
            return refused("REFUSED", RefusalCode.BUSY,
                           f"bras occupés par la manipulation (état {etat_manip})")
        bras = [j for j in anim.joints if groupes.get(j) is JointGroup.MANIPULATION]
        lecteur = AnimationPlayer(self, anim, bras)
        self._animation = lecteur
        if not wait:
            return ActionResult(ok=True, status="PLAYING", message=f"« {anim.name} » en cours",
                                data={"duration_s": round(lecteur.total_s, 3)})
        attente = timeout_s if timeout_s is not None else min(
            self.ANIMATION_TIMEOUT_MAX_S, lecteur.total_s + 2.0)
        debut = time.monotonic()
        fin = debut + attente
        while not lecteur.done:
            maintenant = time.monotonic()
            if maintenant >= fin or maintenant - debut >= self.ANIMATION_TIMEOUT_MAX_S:
                break
            lecteur.wait(0.05)
            if lecteur.paused and timeout_s is None:
                # En pause devant une personne : le délai par défaut ne
                # court pas (jamais au-delà d'ANIMATION_TIMEOUT_MAX_S).
                fin += 0.05
        res = lecteur.result
        if res is None:
            lecteur.stop()
            lecteur.wait(1.0)
            return refused("TIMEOUT", RefusalCode.TIMEOUT,
                           f"animation non terminée en {attente:g} s : arrêtée")
        return res

    def stop_animation(self) -> ActionResult:
        """Arrête l'animation en cours : les bras s'arrêtent là où ils
        sont (status STOPPED) ; IDLE s'il n'y en a pas. Permission :
        ANIMATION_PLAY."""
        self._require(Permission.ANIMATION_PLAY)
        lecteur = self._animation
        if lecteur is None or lecteur.done:
            return ActionResult(ok=True, status="IDLE", message="aucune animation en cours")
        lecteur.stop()
        lecteur.wait(1.0)
        return ActionResult(ok=True, status="STOPPED", message="animation arrêtée")

    # ---- Parole et réglages ----------------------------------------------

    SPEAK_MAX_CHARS = 500

    def speak(self, text: str, *, language: str = "en") -> ActionResult:
        """Fait dire `text` au robot, dans `language` (code « en », « fr »,
        « fr-FR »…). Le texte est coupé à SPEAK_MAX_CHARS caractères et ses
        caractères de contrôle remplacés par des espaces. BLOQUANT le temps
        de le dire.

        `status` : SPOKEN, REFUSED (CAPABILITY_MISSING : pas de synthèse
        vocale sur ce robot, ou langue non prise en charge), FAILED. Lève
        InvalidArgumentError (texte vide ou qui n'est pas un texte, code de
        langue invalide) et EstopActiveError sous arrêt d'urgence.
        Permission : AUDIO_OUTPUT."""
        self._require(Permission.AUDIO_OUTPUT)
        texte = require_text("text", text, self.SPEAK_MAX_CHARS)
        langue = require_language(language)
        self._require_no_estop()
        voix = self._voix
        if voix is None:
            return refused("REFUSED", RefusalCode.CAPABILITY_MISSING,
                           "pas de synthèse vocale sur ce robot")
        langues = tuple(getattr(voix, "languages", ()) or ())
        if langue not in langues and langue.split("-")[0] not in langues:
            return refused("REFUSED", RefusalCode.CAPABILITY_MISSING,
                           f"langue « {langue} » non prise en charge — {list(langues)}")
        try:
            dit = voix.say(texte, langue)
        except Exception as exc:       # noqa: BLE001 — synthèse en panne : échec, pas de plantage
            return refused("FAILED", RefusalCode.FAILED, f"synthèse vocale : {type(exc).__name__}")
        if dit is not True:
            return refused("FAILED", RefusalCode.FAILED, "la synthèse vocale n'a rien dit")
        return ActionResult(ok=True, status="SPOKEN", message="dit")

    def get_setting(self, name: str):
        """Valeur d'un réglage déclaré dans le manifeste (settings) : celle
        que l'utilisateur a choisie, sinon la valeur par défaut déclarée.
        Lève InvalidArgumentError si le réglage n'est pas déclaré.
        Permission : APP_SETTINGS."""
        self._require(Permission.APP_SETTINGS)
        nom = require_text("name", name, 32)
        spec = next((s for s in self._manifest.settings if s.name == nom), None)
        if spec is None:
            raise InvalidArgumentError(f"réglage « {nom} » non déclaré dans le manifeste — "
                                       f"déclarés : {[s.name for s in self._manifest.settings]}")
        if nom in self._reglages:
            try:
                return spec.check(self._reglages[nom])
            except ValueError:
                pass                   # déclaration changée depuis : valeur par défaut
        return spec.default

    def walk(self, vx: float = 0.0, vy: float = 0.0, wz: float = 0.0, *,
             origine: Optional[str] = None) -> bool:
        """Déclare une vitesse de déplacement. NON BLOQUANT.

        Pourquoi l'app ne pilote pas la marche pas à pas : une app
        sandboxée ne peut pas garantir d'envoyer une consigne 50 fois par
        seconde (50 Hz) ; elle peut être lente, bloquée, ou arrêtée par
        son quota (les limites de calcul, de mémoire et de durée imposées
        à l'app). Elle déclare donc une intention, et l'OS fait tourner la
        boucle de commande à la fréquence qu'exige le robot.

        `vx` : vitesse vers l'avant, `vy` : vers la gauche (m/s) ; `wz` :
        rotation autour de la verticale (rad/s). Une consigne trop grande
        est bornée par le service de locomotion. Une consigne non finie
        (NaN, « not a number », c'est-à-dire une valeur numérique
        invalide, ou une valeur infinie) est refusée, et le robot s'arrête
        en équilibre (voir LocomotionService.set_velocity et
        test/test_locomotion_non_finie.py).

        La consigne doit être renouvelée régulièrement. Si l'app cesse de
        le faire, le service la ramène à zéro et le robot s'arrête en
        gardant l'équilibre : la sécurité ne dépend pas de l'app.

        `origine` : ORIGINE_IA quand la vitesse vient d'un modèle d'IA ;
        c'est ce que pose LocomotionAction.walk() du SDK. Elle exige alors
        ACTUATORS_AI_DRIVEN (voir _exiger_origine).

        Rend False si le service refuse (robot à terre, service arrêté,
        consigne non finie) ou si le robot ne sait pas se déplacer.
        Permissions : LOCOMOTION_WALK et ACTUATORS_WRITE_LEGS. Lève
        EstopActiveError si un e-stop est actif.
        """
        self._require(Permission.LOCOMOTION_WALK)
        self._require(Permission.ACTUATORS_WRITE_LEGS)
        self._exiger_origine(origine)
        self._require_no_estop()
        if self._locomotion is None:
            self.log("walk : ce robot ne sait pas se déplacer.")
            return False
        r = self._locomotion.set_velocity(vx, vy, wz)
        if r.reason:
            self.log(f"walk : {r.reason}")
        return bool(r.ok)

    def stop_walking(self) -> bool:
        """Arrêt en équilibre (vitesse nulle). Ce n'est pas un arrêt
        d'urgence : voir emergency_stop().

        Rend False si le robot ne sait pas se déplacer ou si le service
        refuse. Permission : LOCOMOTION_WALK seule ; contrairement à
        walk(), aucune vérification d'e-stop n'est faite ici."""
        self._require(Permission.LOCOMOTION_WALK)
        if self._locomotion is None:
            return False
        return bool(self._locomotion.halt().ok)

    def move_arms(self, commands: Dict[str, ActuatorCommand]) -> Dict[str, HALStatus]:
        """Envoie des consignes aux articulations des bras (groupe
        MANIPULATION : bras, pinces).

        Les consignes destinées à une articulation d'un autre groupe sont
        écartées sans erreur (voir _route()). La vitesse de chaque
        consigne est réduite selon le manifeste (voir
        _apply_velocity_cap()), puis la HAL et son garde-fou vérifient
        chaque consigne et peuvent la refuser. Rend le statut de chaque
        articulation commandée (HALStatus.OK si acceptée) : à lire.

        Permission : ACTUATORS_WRITE_ARMS. Lève EstopActiveError si un
        e-stop est actif.
        """
        self._require(Permission.ACTUATORS_WRITE_ARMS)
        self._exiger_pilotage_ia(commands)
        self._require_no_estop()
        filtered = self._apply_velocity_cap(
            self._route(commands, JointGroup.MANIPULATION))
        return self._robot.send_joint_commands(filtered)

    def move_legs(self, commands: Dict[str, ActuatorCommand]) -> Dict[str, HALStatus]:
        """Envoie des consignes aux articulations des jambes (groupe
        LOCOMOTION : jambes, pattes, roues).

        Mêmes règles que move_arms() : consignes d'un autre groupe
        écartées sans erreur, vitesse réduite selon le manifeste,
        garde-fou de la HAL, statut rendu pour chaque articulation
        commandée.

        Permission : ACTUATORS_WRITE_LEGS. Lève EstopActiveError si un
        e-stop est actif.
        """
        self._require(Permission.ACTUATORS_WRITE_LEGS)
        self._exiger_pilotage_ia(commands)
        self._require_no_estop()
        filtered = self._apply_velocity_cap(
            self._route(commands, JointGroup.LOCOMOTION))
        return self._robot.send_joint_commands(filtered)

    def move_all(self, commands: Dict[str, ActuatorCommand]) -> Dict[str, HALStatus]:
        """Envoie des consignes à n'importe quelle articulation, y compris
        celles du groupe OTHER (torse, tête…), que seule cette méthode
        atteint.

        Vitesse réduite selon le manifeste, puis garde-fou de la HAL,
        comme move_arms() ; rend le statut de chaque articulation.

        Permission : ACTUATORS_WRITE_ALL. Lève EstopActiveError si un
        e-stop est actif.
        """
        self._require(Permission.ACTUATORS_WRITE_ALL)
        self._exiger_pilotage_ia(commands)
        self._require_no_estop()
        commands = self._apply_velocity_cap(commands)
        return self._robot.send_joint_commands(commands)

    def move_to_pose(
        self,
        target: Dict[str, float],
        duration_s: float = 2.0,
        step_s: float = 0.05,
        *,
        origine: Optional[str] = None,
    ) -> bool:
        """
        Déplace progressivement les articulations vers `target` (dict
        joint_id -> position cible), en environ `duration_s` secondes, par
        petits pas interpolés, un toutes les `step_s` secondes.

        Pourquoi cette méthode existe : pour corriger un défaut critique
        relevé par un audit de sécurité externe
        (docs/audit_externe_2026-09-14.md), le projet a ajouté
        JointLimits.position_delta_max, l'écart maximal accepté entre la
        position mesurée d'une articulation et sa consigne ; au-delà, le
        moteur produirait un couple supérieur à sa limite. Un second audit
        externe a ensuite relevé (voir CHANGELOG.md) que cette borne
        protège correctement contre un saut de position dangereux, mais
        ne laissait à une app aucun moyen d'atteindre en un seul appel une
        cible légitime située plus loin. Exemple concret : les
        bras ont `position_delta_max ≈ 0.62 rad` ; si le G1 (le robot
        humanoïde Unitree G1) démarre bras repliés (~1.2 rad) et que
        `on_start()` commande directement la position HOME (~0 rad) par
        `move_arms()`, le garde-fou refuse ce saut, à raison, et rien ne
        permettait alors à l'app d'atteindre HOME. `move_to_pose()` est ce
        chemin manquant : chaque pas reste sous `position_delta_max`.

        Point de départ : la position réelle de chaque articulation, lue
        par `get_full_state()` au moment de l'appel, jamais une position
        supposée ou mise en cache. Le trajet est ensuite calculé depuis ce
        point de départ ; la position n'est pas relue entre les pas. Si
        une articulation bouge de façon inattendue en cours de route
        (autre app, perturbation physique), le garde-fou de la HAL, qui
        compare chaque consigne à la position mesurée, peut refuser le pas
        suivant ; la méthode s'arrête alors (False).

        Chaque articulation est routée vers `move_arms()` ou `move_legs()`
        selon le groupe que déclare le driver (même routage que ces
        méthodes, voir `_group_of()`) : un `target` qui mélange bras et
        jambes est géré correctement, chaque famille recevant sa propre
        commande à chaque pas. Les vérifications de ces deux méthodes
        (permission, e-stop, vitesse) s'appliquent donc à chaque pas.

        Rend False, et s'arrête aussitôt, dès qu'un pas échoue pour au
        moins une articulation : limite dépassée malgré l'interpolation,
        e-stop survenu pendant l'envoi d'un pas, articulation introuvable,
        ou articulation ni bras ni jambe (torse, tête… : utiliser
        `move_all()`), ces deux derniers cas étant détectés avant tout
        mouvement. La méthode ne continue jamais après un échec partiel.
        Rend True quand tous les pas, jusqu'à la cible, ont été acceptés
        pour toutes les articulations demandées.

        Permissions : ACTUATORS_READ, plus ACTUATORS_WRITE_ARMS et/ou
        ACTUATORS_WRITE_LEGS selon les articulations visées. Un e-stop
        déjà actif au début d'un pas lève EstopActiveError, comme
        `move_arms()`. L'attente entre deux pas se fait en temps réel
        (time.sleep), pas sur l'heure du robot de `sleep()`.

        `origine` : ORIGINE_IA quand la cible vient d'un modèle d'IA (c'est
        ce que posent VLAAction.move_to_pose() et LocomotionAction.move_to_pose()
        du SDK) ; ACTUATORS_AI_DRIVEN est alors exigée avant tout
        mouvement, et chaque pas porte la marque.
        """
        import time as _time

        self._exiger_origine(origine)
        state = self.get_full_state()
        current: Dict[str, float] = {}
        for jid in target:
            fb = state.get(jid)
            if fb is None:
                self.log(f"move_to_pose: joint '{jid}' introuvable — abandon.")
                return False
            current[jid] = fb.position

        n_steps = max(1, int(duration_s / step_s))
        for step in range(1, n_steps + 1):
            fraction = step / n_steps
            step_commands: Dict[str, ActuatorCommand] = {}
            for jid, target_pos in target.items():
                interpolated = current[jid] + (target_pos - current[jid]) * fraction
                step_commands[jid] = ActuatorCommand(
                    mode=ControlMode.POSITION, position=interpolated, origine=origine,
                )

            arm_cmds = self._route(step_commands, JointGroup.MANIPULATION)
            leg_cmds = self._route(step_commands, JointGroup.LOCOMOTION)
            unrouted = set(step_commands) - set(arm_cmds) - set(leg_cmds)
            if unrouted:
                self.log(f"move_to_pose: joint(s) du groupe OTHER (torse, tête…) "
                        f"non routable(s) par move_arms/move_legs : "
                        f"{sorted(unrouted)} — abandon. Utiliser move_all().")
                return False

            results: Dict[str, HALStatus] = {}
            if arm_cmds:
                results.update(self.move_arms(arm_cmds))
            if leg_cmds:
                results.update(self.move_legs(leg_cmds))

            failed = [jid for jid, s in results.items() if s != HALStatus.OK]
            if failed:
                self.log(f"move_to_pose: pas {step}/{n_steps} échoué pour "
                        f"{failed} — abandon (interpolation trop agressive, "
                        f"e-stop, ou autre violation de sécurité).")
                return False

            _time.sleep(step_s)
            if self._ferme:
                # Hook terminé (processus de l'app arrêté) : le geste qu'il
                # avait demandé ne continue pas sans lui.
                self.log("move_to_pose: fin du hook — abandon.")
                return False

        return True

    def emergency_stop(self) -> HALStatus:
        """
        Déclenche un arrêt d'urgence global du robot.

        Pourquoi : un audit de sécurité externe
        (docs/audit_externe_2026-09-14.md) a relevé qu'aucune app n'avait
        de moyen de déclencher elle-même un e-stop, alors que la
        permission Permission.EMERGENCY_STOP existait déjà dans
        l'énumération des permissions. Concrètement, une app de patrouille
        qui détectait un humain dans la trajectoire du robot ne pouvait
        pas arrêter le mouvement en cours : seuls le Watchdog et le
        Supervisor (le second gardien, qui surveille le Watchdog lui-même)
        pouvaient le faire, et aucun des deux n'est visible ni pilotable
        par le code d'une app.

        Requiert Permission.EMERGENCY_STOP, qui n'est délibérément ni
        incluse dans les permissions accordées par défaut, ni requise par
        les autres méthodes de mouvement : c'est un pouvoir à part, qu'une
        app doit demander explicitement dans son manifeste. C'est une
        permission DANGEROUS, confirmée par une personne à l'installation
        (voir PERMISSION_LEVELS dans manifest.py). Le déclenchement est
        noté dans le journal ; rend le statut renvoyé par la HAL.
        """
        self._require(Permission.EMERGENCY_STOP)
        self.log("EMERGENCY STOP déclenché par l'app")
        return self._robot.emergency_stop_all()

    def subscribe_alerts(self, callback) -> None:
        """Abonne `callback` aux alertes de sécurité émises par le
        Watchdog. Permission : WATCHDOG_READ."""
        self._require(Permission.WATCHDOG_READ)
        self._watchdog.subscribe(callback)

    MAX_LIGNES_JOURNAL = 10_000
    MAX_CARACTERES_LIGNE = 1_000
    MAX_CARACTERES_JOURNAL = 1_000_000

    def log(self, message: str) -> None:
        """Ajoute une ligne au journal de mission et l'affiche.

        La ligne est préfixée par le temps réel écoulé depuis la création
        de ce contexte et par l'identifiant de l'app. Aucune permission
        requise."""
        # Le journal vit dans le processus de l'OS pendant tout le hook, et
        # ses lignes s'affichent sur sa console : une app qui écrirait sans
        # fin ferait grossir la mémoire de l'OS hors de ses propres quotas,
        # ou réécrirait l'écran par des séquences d'échappement. Chaque
        # ligne est donc neutralisée et coupée, et le journal est borné en
        # lignes comme en volume ; une seule ligne signale la troncature.
        if self._journal_tronque:
            return
        brut = str(message)[:self.MAX_CARACTERES_LIGNE]
        texte = "".join(c if c.isprintable() else "?" for c in brut)
        elapsed = time.monotonic() - self._t0
        entry   = f"[T+{elapsed:06.2f}s] [{self._manifest.app_id}] {texte}"
        if (len(self._log_entries) >= self.MAX_LIGNES_JOURNAL
                or self._taille_journal + len(entry) > self.MAX_CARACTERES_JOURNAL):
            self._journal_tronque = True
            self._log_entries.append(f"[{self._manifest.app_id}] … journal tronqué")
            return
        self._taille_journal += len(entry)
        self._log_entries.append(entry)
        print(f"  {entry}")

    def get_mission_log(self) -> list[str]:
        """Copie du journal de mission (les lignes écrites par log()).
        Aucune permission requise."""
        return list(self._log_entries)

    def _exiger_pilotage_ia(self, commands: Dict[str, ActuatorCommand]) -> None:
        """Exige ACTUATORS_AI_DRIVEN dès qu'une consigne vient d'un modèle
        d'IA (ActuatorCommand.origine == ORIGINE_IA, posée par les actions
        du SDK : VLAAction, GR00TAction, LocomotionAction).

        Pourquoi : la permission était confirmée à l'installation mais
        jamais vérifiée à l'exécution (SECURITY.md, limite n° 2) ; une app
        qui faisait tourner un modèle sans la déclarer n'était pas
        arrêtée. Le contrôle a lieu ici, dans le processus de l'OS, quel
        que soit le chemin (app sandboxée ou contexte direct). Limite : une
        app qui fabrique elle-même ses consignes à partir de la sortie d'un
        modèle, sans passer par les actions du SDK, ne porte pas la marque
        et n'est pas détectée ; l'OS ne peut pas distinguer un nombre
        calculé par un modèle d'un nombre écrit en dur."""
        if any(getattr(c, "origine", None) == ORIGINE_IA for c in commands.values()):
            self._require(Permission.ACTUATORS_AI_DRIVEN)

    def _exiger_origine(self, origine: Optional[str]) -> None:
        """Même règle que _exiger_pilotage_ia pour walk() et move_to_pose(),
        qui reçoivent une vitesse ou une cible plutôt que des commandes.
        Une origine inconnue est refusée plutôt qu'ignorée."""
        if origine is None:
            return
        if origine != ORIGINE_IA:
            raise ValueError(f"origine inconnue : {str(origine)[:40]!r}")
        self._require(Permission.ACTUATORS_AI_DRIVEN)

    def _require(self, permission: Permission) -> None:
        """Lève PermissionError si `permission` n'est pas déclarée dans le
        manifeste de l'app. Appelée en tête de chaque méthode protégée."""
        if not self._manifest.has_permission(permission):
            raise PermissionError(
                f"Permission refusée : {permission.name} "
                f"non déclarée dans '{self._manifest.app_id}'"
            )

    def _require_no_estop(self) -> None:
        """
        Vérifie qu'aucun e-stop n'est actif avant chaque tentative de
        mouvement, et pas seulement une fois au début de la mission : une
        mission déjà en cours doit s'arrêter net à la prochaine commande
        de mouvement dès qu'un e-stop survient, au lieu de continuer
        jusqu'à la fin de sa boucle.

        Pourquoi deux sources : avant ce double contrôle (correctif issu
        de l'audit de sécurité externe du 2026-09-14, voir CHANGELOG.md),
        un e-stop déclenché par le Supervisor, précisément le cas où le
        Watchdog est mort ou bloqué, restait invisible ici : les apps
        continuaient de voir le robot comme disponible. Les deux sources
        consultées sont :
        - watchdog.estop_fired, positionné par le Watchdog lui-même ;
        - self._robot.estop_active, positionné par
          IRobotHAL.emergency_stop_all(), donc aussi par le Supervisor
          quand il appelle cette méthode directement.
        """
        if self._watchdog.estop_fired or self._robot.estop_active:
            raise EstopActiveError(
                f"Commande de mouvement refusée : e-stop actif sur le "
                f"robot — app '{self._manifest.app_id}'"
            )

    def _apply_velocity_cap(
        self, commands: Dict[str, ActuatorCommand]
    ) -> Dict[str, ActuatorCommand]:
        """
        Ramène la vitesse de chaque commande à max_velocity_ratio fois sa
        valeur d'origine, en gardant son signe. Ce rapport est fixé dans
        le manifeste de l'app : strictement positif et au plus 1 (vérifié
        par AppManifest.validate()), 0,8 par défaut. La marche demandée
        par walk() n'est pas concernée : sa vitesse est bornée par le
        service de locomotion.

        Pourquoi une copie : un audit de sécurité, en exécutant réellement
        le code, a trouvé la version précédente cassée. Elle faisait
        `cmd.velocity *= ratio` en place sur les objets de l'appelant, au
        lieu de plafonner. Une app qui construit son dict de commandes une
        fois et le réutilise à chaque cycle (usage courant) voyait sa
        vitesse décroître géométriquement d'un appel à l'autre (ratio^n)
        au lieu de rester à une valeur stable : un mouvement qui « traîne »
        de plus en plus, en silence, sans qu'aucune erreur ne soit levée.

        Rend donc un nouveau dict, avec de nouveaux objets ActuatorCommand,
        plutôt que de modifier ceux de l'appelant : une app ne doit jamais
        voir ses propres objets changés par effet de bord après un appel à
        move_arms(), move_legs() ou move_all(). Une commande sans vitesse
        (None ou 0) est reprise telle quelle.
        """
        ratio = self._manifest.max_velocity_ratio
        result = {}
        for jid, cmd in commands.items():
            if cmd.velocity is None or cmd.velocity == 0:
                result[jid] = cmd
                continue
            capped_velocity = math.copysign(abs(cmd.velocity) * ratio, cmd.velocity)
            result[jid] = replace(cmd, velocity=capped_velocity)
        return result


@dataclass
class MissionResult:
    """
    Résultat d'une mission, rendu par RobotApp.run_mission().

    `success` et `error_message` disent si la mission a réussi et, sinon,
    pourquoi : un échec clair vaut mieux qu'un résultat plausible inventé
    (voir docs/CREER_UNE_APP.md). `items_processed` compte les éléments
    traités, `metrics` reçoit les mesures propres à l'app. `log` est
    rempli par l'OS avec le journal de mission (ctx.log()).

    Si la mission lève PermissionError ou EstopActiveError dans le
    sandbox, l'OS construit lui-même un MissionResult d'échec.

    `interrompue_par_arret_urgence` n'est posé que par l'OS (PlayStoreRuntime),
    quand un arrêt d'urgence est survenu pendant la mission ; elle est alors
    un échec, jamais facturée.
    """
    success:          bool
    mission_id:       str
    app_id:           str
    duration_s:       float
    items_processed:  int              = 0
    error_message:    Optional[str]    = None
    metrics:          Dict[str, Any]   = field(default_factory=dict)
    log:              list[str]        = field(default_factory=list)
    interrompue_par_arret_urgence: bool = False

    def __str__(self) -> str:
        status = "✓ SUCCESS" if self.success else (
            "✗ INTERROMPUE (arrêt d'urgence)" if self.interrompue_par_arret_urgence
            else "✗ FAILED")
        return (f"[Mission {self.mission_id}] {status} | "
                f"app={self.app_id} | durée={self.duration_s:.2f}s | "
                f"items={self.items_processed}")


class RobotApp(ABC):
    """Classe de base pour toute application du Robotics Play Store.

    Une app implémente quatre méthodes, appelées par l'OS
    (PlayStoreRuntime) :
    - manifest() : rend l'AppManifest de l'app (identité, permissions,
      quotas…) ;
    - on_start(ctx) : prépare l'app et rend True si elle est prête ; si
      elle rend False, l'app n'est pas démarrée et l'OS refuse ses
      missions ;
    - run_mission(ctx, params) : exécute une mission et rend un
      MissionResult ;
    - on_stop(ctx) : arrête l'app.

    on_start(), run_mission() et on_stop() s'exécutent dans le sandbox
    de l'app ; `ctx` y est un HALBridgeClient, qui offre exactement les
    mêmes méthodes que RobotContext. C'est le seul accès de l'app au
    robot.

    Définir la classe de l'app au niveau du module, jamais à l'intérieur
    d'une fonction : chaque hook tourne dans un processus Python neuf, qui
    recharge l'app depuis son module ou sa source (voir
    store/runtime/hote_app.py) et ne peut pas retrouver une classe locale.
    Le constructeur doit pouvoir être appelé sans argument : seuls les
    attributs qui sont des données (nombres, chaînes, listes, dicts,
    énumérations) sont recopiés depuis l'instance installée.
    """

    @abstractmethod
    def manifest(self) -> AppManifest: ...

    @abstractmethod
    def on_start(self, ctx: RobotContext) -> bool: ...

    @abstractmethod
    def run_mission(self, ctx: RobotContext, params: Dict[str, Any]) -> MissionResult: ...

    @abstractmethod
    def on_stop(self, ctx: RobotContext) -> None: ...
