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


class PermissionError(Exception):
    """
    Levée quand une app appelle une méthode de RobotContext dont la
    permission n'est pas déclarée dans son manifeste.

    Attention au nom : cette classe porte le même nom que l'exception
    standard de Python, mais c'est une classe distincte, définie ici.
    Pour l'intercepter, il faut importer celle de ce module.
    """
    pass


class EstopActiveError(Exception):
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
    """
    pass


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
        self._t0 = time.monotonic()
        # Services système, injectés par le runtime (voir
        # _attach_services()). None signifie que le service n'existe pas
        # sur ce robot : l'API le dit clairement plutôt que d'échouer de
        # façon obscure.
        self._perception = None
        self._locomotion = None
        self._speech = None
        self._memory = None
        # Horloge du robot : l'heure réelle par défaut, remplacée par
        # l'heure simulée en simulation (voir _set_clock() et now()).
        self._clock = time.monotonic
        # Groupes d'articulations déclarés par la HAL (bras, jambes ou
        # autre), résolus à la première utilisation (voir _group_of()).
        self._joint_groups: Optional[Dict[str, JointGroup]] = None

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
                         speech=None, memory=None) -> None:
        """Injection des services par le runtime.

        Méthode privée (préfixe _), donc hors de la surface publique que
        compare le test de parité (test/test_context_parity.py). Ce n'est
        pas une API d'app : une app ne choisit pas ses services, elle les
        reçoit."""
        self._perception = perception
        self._locomotion = locomotion
        self._speech = speech
        self._memory = memory

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
        pas de mémoire persistante, ou si le service de mémoire refuse
        (la raison est alors notée dans le journal). Permission :
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
        ordinaire, pas comme une erreur. Permission : SENSORS_MICROPHONE.
        """
        self._require(Permission.SENSORS_MICROPHONE)
        if self._speech is None:
            self.log("listen : ce robot n'a pas de service de parole.")
            return None
        r = self._speech.listen(duration_s)
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
        n'a pas de service de parole, ou si ce nom d'intention est déjà
        déclaré (le conflit est noté dans le journal). Permission :
        SENSORS_MICROPHONE.
        """
        self._require(Permission.SENSORS_MICROPHONE)
        if self._speech is None:
            return False
        from axis_os.services.speech.service import IntentDeclaration
        try:
            self._speech.registry.register(IntentDeclaration(
                name=name, keywords=keywords, params=params or {},
                description=description))
            return True
        except ValueError as exc:
            # Déjà déclarée par une autre app : ce n'est pas une erreur
            # de programmation, c'est un conflit à arbitrer.
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

    def walk(self, vx: float = 0.0, vy: float = 0.0, wz: float = 0.0) -> bool:
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

        Rend False si le service refuse (robot à terre, service arrêté,
        consigne non finie) ou si le robot ne sait pas se déplacer.
        Permissions : LOCOMOTION_WALK et ACTUATORS_WRITE_LEGS. Lève
        EstopActiveError si un e-stop est actif.
        """
        self._require(Permission.LOCOMOTION_WALK)
        self._require(Permission.ACTUATORS_WRITE_LEGS)
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
        """
        import math as _math
        import time as _time

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
                    mode=ControlMode.POSITION, position=interpolated,
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

    def log(self, message: str) -> None:
        """Ajoute une ligne au journal de mission et l'affiche.

        La ligne est préfixée par le temps réel écoulé depuis la création
        de ce contexte et par l'identifiant de l'app. Aucune permission
        requise."""
        elapsed = time.monotonic() - self._t0
        entry   = f"[T+{elapsed:06.2f}s] [{self._manifest.app_id}] {message}"
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
    """
    success:          bool
    mission_id:       str
    app_id:           str
    duration_s:       float
    items_processed:  int              = 0
    error_message:    Optional[str]    = None
    metrics:          Dict[str, Any]   = field(default_factory=dict)
    log:              list[str]        = field(default_factory=list)

    def __str__(self) -> str:
        status = "✓ SUCCESS" if self.success else "✗ FAILED"
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
    d'une fonction : le sandbox transmet l'app au processus isolé en la
    sérialisant avec pickle (le format de Python qui transforme un objet
    en octets pour le faire passer d'un processus à l'autre), et une
    classe locale ne se sérialise pas : l'app plante alors au démarrage
    sans explication claire (voir docs/CREER_UNE_APP.md).
    """

    @abstractmethod
    def manifest(self) -> AppManifest: ...

    @abstractmethod
    def on_start(self, ctx: RobotContext) -> bool: ...

    @abstractmethod
    def run_mission(self, ctx: RobotContext, params: Dict[str, Any]) -> MissionResult: ...

    @abstractmethod
    def on_stop(self, ctx: RobotContext) -> None: ...
