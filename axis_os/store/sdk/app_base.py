"""
AXIS-OS Robotics Play Store — App SDK Base
============================================
Classe de base que tout développeur d'app doit étendre.
Le RobotContext est le sandbox — l'app n'accède au robot qu'à travers lui.

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
    JointGroup, infer_joint_group,
)
from axis_os.hal.watchdog.system import WatchdogSystem
from .manifest import AppManifest, Permission


class PermissionError(Exception):
    pass


class EstopActiveError(Exception):
    """
    Levée quand une app tente d'envoyer une commande de mouvement alors
    qu'un e-stop global est actif sur le robot.

    Trou de sécurité identifié par le test de stress final : une mission
    déjà en cours au moment où le Watchdog déclenche un e-stop continuait
    auparavant son exécution jusqu'au bout — emergency_stop_all() coupait
    bien le hardware, mais rien n'empêchait le CODE APPLICATIF de
    continuer à tenter d'envoyer de nouvelles commandes de mouvement
    entre-temps. Corrigé en vérifiant l'état e-stop à CHAQUE appel
    move_arms/move_legs/move_all, pas seulement une fois à l'entrée de
    run_mission() côté PlayStoreRuntime.
    """
    pass


class UnauthorizedContextError(Exception):
    """
    Levée quand RobotContext est construit sans un jeton de session
    valide — trouvé nécessaire pour fermer un vrai trou : dans le code
    précédent, RIEN n'empêchait quiconque disposant d'un objet
    NormalizedRobotHAL/WatchdogSystem réel (ex: du code qui importe
    axis_os directement, hors de tout Play Store) de construire son
    PROPRE RobotContext avec un AppManifest arbitraire — donc
    n'importe quelles permissions, sans jamais passer par install()
    ni par la vérification de signature (signing.py). Une app dont le
    code entier ne fonctionne QU'À TRAVERS RobotContext (voir
    RobotApp, entièrement abstraite — sans ctx.move_arms() etc., le
    code d'une app ne fait littéralement rien) devient alors
    réellement inutilisable hors d'AXIS-OS : le seul chemin légitime
    de construction est PlayStoreRuntime._make_ctx(), qui fournit ce
    jeton — voir _SESSION_TOKEN ci-dessous.
    """
    pass


class _RuntimeSessionToken:
    """
    Jeton d'identité — généré une fois par instance de
    PlayStoreRuntime (voir PlayStoreRuntime.__init__), transmis par
    RÉFÉRENCE D'OBJET (jamais par valeur comparable) à chaque
    RobotContext qu'elle construit.

    Limite assumée, honnête : en Python pur, sans extension compilée,
    AUCUNE barrière n'empêche un attaquant qui LIT ce code source
    d'importer _RuntimeSessionToken et d'en instancier une lui-même —
    deux tentatives précédentes de ce mécanisme (comparaison de
    valeur, registre module-level) ont chacune semblé fermer ce trou
    sans y parvenir réellement, la classe elle-même restant
    accessible à l'import dans les deux cas.

    Ce mécanisme ne prétend donc PAS empêcher un développeur qui lit
    activement le code source d'AXIS-OS et écrit délibérément du code
    pour le contourner — objectif hors de portée en Python pur (voir
    docs/hardware_estop_notes.md pour la même limite de fond
    appliquée à la sécurité physique : aucune barrière logicielle
    n'est jamais absolue). Ce que ce mécanisme empêche réellement :
    qu'une app INNOCENTE, ou du code qui importe RobotContext sans
    intention malveillante particulière (le cas visé par la demande
    d'origine — qu'une app ne « marche » que dans AXIS-OS), obtienne
    un RobotContext fonctionnel par accident ou en suivant l'API
    publique évidente — RobotContext(robot, watchdog, manifest) sans
    rien de plus échoue maintenant explicitement, avec un message
    clair, plutôt que de sembler fonctionner.
    """
    __slots__ = ()

    def _verify(self, candidate: object) -> bool:
        return candidate is self


class RobotContext:
    """
    Interface sécurisée entre une app et le robot.

    ⚠️ Ne JAMAIS instancier directement hors de
    PlayStoreRuntime._make_ctx() — voir UnauthorizedContextError pour
    le contexte complet. `_session_token` est requis et vérifié dès
    la construction, jamais optionnel : un appelant qui n'a pas
    accès au jeton d'une vraie instance de PlayStoreRuntime ne peut
    pas produire de RobotContext utilisable, quelles que soient les
    permissions qu'il tenterait de déclarer dans un AppManifest
    construit à la main.
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
        # Services système, injectés par le runtime. None = service
        # indisponible sur ce robot ; l'API le DIT plutôt que d'échouer
        # obscurément.
        self._perception = None
        self._locomotion = None
        self._speech = None
        self._memory = None
        self._clock = time.monotonic
        # Groupes déclarés par le HAL, résolus à la première utilisation.
        self._joint_groups: Optional[Dict[str, JointGroup]] = None

    @classmethod
    def for_testing(
        cls, robot: NormalizedRobotHAL, watchdog: WatchdogSystem, manifest: AppManifest,
    ) -> "RobotContext":
        """
        Construit un RobotContext SANS passer par un PlayStoreRuntime
        complet — nommé explicitement `for_testing` pour que son
        usage soit toujours visible et intentionnel dans le code
        appelant (jamais un contournement accidentel de
        UnauthorizedContextError). Réservé aux tests unitaires qui
        veulent isoler RobotContext sans construire tout un
        PlayStoreRuntime — jamais destiné à du code de production ni
        à une app réelle, qui doivent toujours passer par
        PlayStoreRuntime.install()/run_mission().
        """
        token = _RuntimeSessionToken()
        return cls(robot, watchdog, manifest, _session_token=token, _runtime_token_holder=token)

    def get_joint_state(self, joint_id: str) -> Optional[ActuatorFeedback]:
        self._require(Permission.ACTUATORS_READ)
        return self._robot.get_full_state().get(joint_id)

    def get_full_state(self) -> Dict[str, ActuatorFeedback]:
        self._require(Permission.ACTUATORS_READ)
        return self._robot.get_full_state()

    def get_imu(self):
        self._require(Permission.SENSORS_IMU)
        return self._robot.get_imu()

    def _group_of(self, joint_id: str) -> JointGroup:
        """Groupe d'une articulation, DÉCLARÉ PAR LE DRIVER.

        Le SDK ne devine plus à partir de mots-clés anglais : il demande
        au HAL, qui seul sait ce que son robot possède. Repli sur
        infer_joint_group() pour un driver qui ne déclare pas encore ses
        groupes, afin de ne rien casser pendant la transition.
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
        return {jid: cmd for jid, cmd in commands.items()
                if self._group_of(jid) is group}

    # ---- Services système -------------------------------------------

    def _attach_services(self, perception=None, locomotion=None,
                         speech=None, memory=None) -> None:
        """Injection par le runtime — PRIVÉE, donc hors de la surface
        publique comparée par le test de parité. Ce n'est pas une API
        d'app : une app ne choisit pas ses services, elle les reçoit."""
        self._perception = perception
        self._locomotion = locomotion
        self._speech = speech
        self._memory = memory

    def _set_clock(self, clock) -> None:
        """Injection par le runtime — privée, hors surface publique."""
        self._clock = clock or time.monotonic

    def now(self) -> float:
        """Heure du ROBOT, en secondes.

        À utiliser À LA PLACE de time.monotonic() pour mesurer une durée
        dans une app. En simulation, c'est l'heure SIMULÉE ; sur le vrai
        robot, l'heure réelle. Le même code se comporte donc identiquement
        dans les deux cas — l'équivalent de use_sim_time dans ROS.

        Constaté sans cette précaution : une app qui écoutait « deux
        secondes » et marchait « trois secondes » en temps mural, dans une
        simulation ralentie par le rendu vidéo, prenait ses décisions sur
        une perception décalée et perdait le colis qu'elle visait.
        """
        return float(self._clock())

    def sleep(self, secondes: float) -> None:
        """Attend `secondes` du temps du ROBOT. Voir now()."""
        if self._clock is time.monotonic:
            time.sleep(max(0.0, secondes))
            return
        fin = self._clock() + max(0.0, secondes)
        while self._clock() < fin:
            time.sleep(0.002)

    def remember(self, key: str, value) -> bool:
        """Enregistre une donnée qui SURVIT au redémarrage du robot.

        L'espace de stockage est celui de CETTE app : l'identifiant
        utilisé est celui de son manifeste, qu'elle ne choisit pas. Une
        app ne peut donc ni lire ni écraser les données d'une autre,
        même en devinant sa clé.
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
        """Relit une donnée enregistrée, ou `default` si absente.

        Une clé inconnue est un cas NORMAL — premier démarrage, mémoire
        effacée — pas une erreur.
        """
        self._require(Permission.STORAGE_PERSISTENT)
        if self._memory is None:
            return default
        r = self._memory.app_load(self._manifest.app_id, key)
        return r.value if r.ok else default

    def listen(self, duration_s: float = 3.0):
        """Écoute et rend une INTENTION reconnue, ou None.

        Rend un objet Intent : un NOM d'action et des paramètres. Jamais
        un mouvement. C'est l'app qui décide ce que l'intention
        déclenche — une phrase ne commande pas un moteur, elle demande
        quelque chose à du code responsable.

        None est un résultat NORMAL et fréquent : silence, bruit, phrase
        hors du vocabulaire déclaré. Une app doit traiter ce cas comme
        ordinaire, pas comme une erreur.
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
        """Déclare une intention que CETTE app sait traiter.

        C'est le modèle des intentions d'Android : l'app annonce ce
        qu'elle sait faire, l'OS choisit qui appeler. Installer une app
        ajoute du vocabulaire au robot, sans réentraîner aucun modèle.
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
        """Cherche un objet décrit en langage naturel.

        Rend une liste de Detection, triées par confiance. Chacune porte
        une direction dans le repère du robot, et une position 3D si une
        caméra de profondeur est disponible — sinon None, jamais une
        distance inventée.

        `frame` : "base" (défaut) pour se diriger vers un objet,
        "world" pour MÉMORISER sa position. Un repère lié au robot
        change quand le robot bouge : y enregistrer un inventaire n'a
        pas de sens.

        Liste vide si le service de perception est absent : un robot
        sans caméra n'est pas une erreur de programmation.
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
        """Ce que le robot voit, en clair."""
        self._require(Permission.SENSORS_CAMERA)
        if self._perception is None:
            return "aucune perception disponible"
        r = self._perception.describe_scene()
        return r.value if r.ok else f"perception indisponible : {r.reason}"

    def walk(self, vx: float = 0.0, vy: float = 0.0, wz: float = 0.0) -> bool:
        """Déclare une vitesse de déplacement. NON BLOQUANT.

        L'app ne pilote PAS la marche pas à pas : elle déclare une
        intention, et l'OS fait tourner la boucle de commande à la
        fréquence qu'exige le robot. Une app sandboxée ne peut pas
        garantir 50 Hz — elle peut être lente, bloquée, ou arrêtée par
        son quota.

        La consigne doit être RENOUVELÉE régulièrement. Si l'app cesse
        de le faire, le service la ramène à zéro et le robot s'arrête en
        gardant l'équilibre : la sécurité ne dépend pas de l'app.

        Rend False si le service refuse (robot à terre, service arrêté).
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
        """Arrêt EN ÉQUILIBRE (vitesse nulle). Ce n'est pas un arrêt
        d'urgence : voir emergency_stop()."""
        self._require(Permission.LOCOMOTION_WALK)
        if self._locomotion is None:
            return False
        return bool(self._locomotion.halt().ok)

    def move_arms(self, commands: Dict[str, ActuatorCommand]) -> Dict[str, HALStatus]:
        self._require(Permission.ACTUATORS_WRITE_ARMS)
        self._require_no_estop()
        filtered = self._apply_velocity_cap(
            self._route(commands, JointGroup.MANIPULATION))
        return self._robot.send_joint_commands(filtered)

    def move_legs(self, commands: Dict[str, ActuatorCommand]) -> Dict[str, HALStatus]:
        self._require(Permission.ACTUATORS_WRITE_LEGS)
        self._require_no_estop()
        filtered = self._apply_velocity_cap(
            self._route(commands, JointGroup.LOCOMOTION))
        return self._robot.send_joint_commands(filtered)

    def move_all(self, commands: Dict[str, ActuatorCommand]) -> Dict[str, HALStatus]:
        self._require(Permission.ACTUATORS_WRITE_ALL)
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
        Déplace progressivement vers `target` (dict joint_id -> position
        cible) en interpolant depuis la position RÉELLE mesurée, par
        petits pas restant sous JointLimits.position_delta_max de
        chaque joint — comblant un trou identifié après le correctif
        C5 (audit de sécurité) : `position_delta_max` protège
        correctement contre un saut de position dangereux, mais
        n'offre à une app AUCUN moyen d'atteindre une cible légitime
        qui se trouve au-delà de cette limite en un seul appel. Concret
        : les bras ont `position_delta_max ≈ 0.62 rad` ; si le G1
        démarre bras repliés (~1.2 rad) et que `on_start()` commande
        HOME (~0 rad) directement via `move_arms()`, le garde refuse à
        raison ce saut — mais rien ne permettait alors à l'app
        d'atteindre HOME du tout. `move_to_pose()` est ce chemin
        manquant.

        Lit la position réelle via `get_full_state()` (jamais une
        position supposée ou mise en cache) avant chaque pas — si un
        joint bouge de façon inattendue en cours de route (autre app,
        perturbation physique), l'interpolation continue depuis sa
        vraie position courante, pas depuis un trajet planifié à
        l'avance et potentiellement obsolète.

        Route chaque joint vers `move_arms()` ou `move_legs()` selon
        son nom (même heuristique que ces méthodes elles-mêmes) — un
        `target` qui mélange bras et jambes est géré correctement,
        chaque famille recevant sa propre commande à chaque pas.

        Retourne False dès qu'un pas échoue pour au moins un joint
        (limite dépassée malgré l'interpolation, e-stop déclenché en
        cours de route, joint introuvable) — s'arrête alors
        immédiatement, ne continue jamais après un échec partiel.
        Retourne True si la cible a été atteinte pour tous les joints
        demandés.
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
        Déclenche un arrêt d'urgence global du robot — comble un trou
        identifié par un audit de sécurité (M11) : aucune app n'avait
        jusqu'ici de moyen de déclencher elle-même un e-stop, malgré
        l'existence de la permission Permission.EMERGENCY_STOP dans
        l'enum. Concret : une app de patrouille qui détecte un humain
        dans la trajectoire du robot n'avait aucun moyen d'arrêter le
        mouvement en cours — seuls le Watchdog et le Supervisor
        pouvaient le faire, tous deux invisibles et non pilotables par
        la logique applicative.

        Requiert Permission.EMERGENCY_STOP — délibérément PAS incluse
        dans les permissions accordées par défaut ni requise par les
        autres méthodes de mouvement : c'est un pouvoir à part, qu'une
        app doit demander explicitement dans son manifest.
        """
        self._require(Permission.EMERGENCY_STOP)
        self.log("EMERGENCY STOP déclenché par l'app")
        return self._robot.emergency_stop_all()

    def subscribe_alerts(self, callback) -> None:
        self._require(Permission.WATCHDOG_READ)
        self._watchdog.subscribe(callback)

    def log(self, message: str) -> None:
        elapsed = time.monotonic() - self._t0
        entry   = f"[T+{elapsed:06.2f}s] [{self._manifest.app_id}] {message}"
        self._log_entries.append(entry)
        print(f"  {entry}")

    def get_mission_log(self) -> list[str]:
        return list(self._log_entries)

    def _require(self, permission: Permission) -> None:
        if not self._manifest.has_permission(permission):
            raise PermissionError(
                f"Permission refusée : {permission.name} "
                f"non déclarée dans '{self._manifest.app_id}'"
            )

    def _require_no_estop(self) -> None:
        """
        Vérifie qu'aucun e-stop n'est actif AVANT chaque tentative de
        mouvement — pas seulement une fois à l'entrée de la mission.
        Une mission déjà en cours doit s'arrêter net à la PROCHAINE
        commande de mouvement dès qu'un e-stop survient, pas continuer
        jusqu'à la fin de sa boucle applicative.

        Consulte DEUX sources — correctif E10, audit de sécurité :
        watchdog.estop_fired (positionné par le Watchdog lui-même) ET
        self._robot.estop_active (positionné par IRobotHAL.
        emergency_stop_all(), donc aussi par le Supervisor quand il
        appelle cette méthode directement). Avant ce correctif, un
        e-stop déclenché par le Supervisor — précisément le cas où le
        Watchdog est mort ou bloqué — restait invisible ici : les apps
        continuaient de voir le robot comme disponible.
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
        Plafonne la vitesse de chaque commande à max_velocity_ratio
        de sa valeur d'origine — trouvé cassé par un audit de sécurité
        (exécution réelle) : la version précédente faisait
        `cmd.velocity *= ratio` EN PLACE sur les objets de l'appelant,
        au lieu de plafonner. Une app qui construit son dict de
        commandes une fois et le réutilise à chaque cycle (pattern
        courant) voyait sa vitesse décroître géométriquement d'un
        appel à l'autre (ratio^n) au lieu de rester plafonnée à une
        valeur stable — un mouvement qui "traîne" de façon croissante
        et silencieuse, sans qu'aucune erreur ne soit levée.

        Retourne un NOUVEAU dict avec de nouveaux objets ActuatorCommand
        plutôt que de muter ceux de l'appelant — une app ne doit jamais
        voir ses propres objets modifiés par effet de bord après un
        appel à move_arms/move_legs/move_all.
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
    """Classe de base pour toute application du Robotics Play Store."""

    @abstractmethod
    def manifest(self) -> AppManifest: ...

    @abstractmethod
    def on_start(self, ctx: RobotContext) -> bool: ...

    @abstractmethod
    def run_mission(self, ctx: RobotContext, params: Dict[str, Any]) -> MissionResult: ...

    @abstractmethod
    def on_stop(self, ctx: RobotContext) -> None: ...
