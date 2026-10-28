"""
AXIS-OS — couche de sécurité du HAL, version 2
================================================
Garde-fou articulaire : chaque commande destinée à un moteur passe par ce
validateur avant d'être exécutée. Il se place, dans le HAL (la couche
d'abstraction matérielle, seul chemin vers les moteurs), entre la partie
de l'OS qui décide des mouvements (l'orchestrateur) et chaque actionneur
(le moteur d'une articulation).

Règle absolue : en cas de doute, on coupe. La sécurité passe avant la
performance.

Ce que la version 2 a corrigé, à la suite d'une revue extérieure de ce
fichier :
  - Les valeurs NaN (une valeur numérique invalide, « pas un nombre ») et
    infinies sont détectées explicitement, sous la règle INVALID_VALUE.
    Avant, elles étaient rejetées implicitement sous la règle
    POSITION_LIMIT, avec un message trompeur : en Python,
    pos_min <= NaN <= pos_max vaut False, donc la commande était déjà
    bloquée, mais le diagnostic était faux.
  - validate_limits() vérifie qu'une configuration de limites (JointLimits)
    est cohérente (min < max, valeurs finies) avant qu'elle serve à valider
    quoi que ce soit : des limites mal configurées ne doivent jamais passer
    inaperçues.

Ce que la version 2 n'a volontairement pas ajouté : les limites
d'accélération et de slew rate (vitesse maximale de variation d'une
consigne), et la comparaison entre la commande et le retour du moteur.
Toutes demandent de connaître la commande précédente, que
JointSafetyGuard ne conserve pas lui-même. La revue de ce fichier et celle
du Watchdog (le système qui surveille en continu l'état du robot) ont
pointé la même dépendance manquante : signe qu'il fallait la traiter une
seule fois, proprement, plutôt qu'à moitié ici. Elle l'a été depuis dans
un composant partagé, MotionStateTracker (hal/core/motion_state.py), sur
lequel s'appuie le contrôle d'accélération de validate_command().

Licence : Apache 2.0 (open-source)
"""

from __future__ import annotations
import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, List

from ..core.types import (
    ActuatorCommand, ActuatorFeedback,
    JointLimits, ControlMode, HALStatus
)
from ..core.motion_state import MotionStateTracker
from ..core.structured_log import StructuredLogger, LogCategory, get_default_logger


class SafetyErrorKind(Enum):
    """
    Type d'une violation de sécurité, pour que le code appelant puisse
    réagir selon la nature du problème (par exemple, un arrêt d'urgence sur
    une surcharge) sans analyser un texte. Même principe que
    SandboxErrorKind (erreurs du sandbox, le processus séparé dans lequel
    tourne une app) et ConventionErrorKind (erreurs dans la table des
    conventions d'articulations du robot).

    Les types les moins évidents :
      - INVALID_VALUE : valeur NaN ou infinie, ou gain d'impédance
        (raideur ou amortissement) négatif.
      - POSITION_DELTA_LIMIT : consigne de position trop éloignée de la
        position mesurée ; le contrôleur PD (proportionnel-dérivé) intégré
        au moteur produirait alors un couple excessif. Ajouté après un
        audit de sécurité ; voir JointLimits.position_delta_max.
      - TORQUE_OVERLOAD, CURRENT_OVERLOAD : couple ou courant réellement
        mesuré (et non commandé) au-delà de 110 % de son maximum
        (torque_max ou current_max). CURRENT_OVERLOAD a été ajouté après un
        audit de sécurité.
      - INVALID_LIMITS_CONFIG : la configuration JointLimits elle-même est
        incohérente. Ce fichier n'émet aujourd'hui aucune violation de ce
        type : une configuration incohérente est signalée par
        validate_limits(), puis par une exception à la construction de
        JointSafetyGuard.
      - ACCELERATION_LIMIT : accélération trop forte entre deux commandes
        successives. La commande est bloquée, jamais lissée.
    """
    INVALID_VALUE      = auto()
    POSITION_LIMIT      = auto()
    POSITION_DELTA_LIMIT = auto()
    VELOCITY_LIMIT       = auto()
    TORQUE_LIMIT          = auto()
    CURRENT_LIMIT           = auto()
    TEMPERATURE_CRITICAL      = auto()
    TORQUE_OVERLOAD             = auto()
    CURRENT_OVERLOAD              = auto()
    INVALID_LIMITS_CONFIG         = auto()
    ACCELERATION_LIMIT              = auto()
    THERMAL_DERATING                  = auto()


@dataclass
class SafetyViolation:
    """
    Une commande refusée, ou une anomalie relevée dans un retour du moteur :
    l'articulation concernée, la règle enfreinte, la valeur fautive et la
    limite dépassée.

    rule est le nom lisible de la règle (par exemple « POSITION_LIMIT »),
    conservé pour la compatibilité avec le code existant ; kind en donne
    le type structuré (SafetyErrorKind).
    """
    joint_id:    str
    rule:        str            # nom lisible, gardé pour la compatibilité
    value:       float
    limit:       float
    kind:        Optional[SafetyErrorKind] = None
    timestamp_ns: int = 0

    def __post_init__(self):
        # 0 signifie « non fourni » : la violation est alors horodatée à sa
        # création, en nanosecondes.
        if self.timestamp_ns == 0:
            import time
            self.timestamp_ns = time.time_ns()

    def __str__(self) -> str:
        return (
            f"[SAFETY VIOLATION] joint={self.joint_id} | "
            f"rule={self.rule} | value={self.value} | limit={self.limit}"
        )


@dataclass
class LimitsValidationResult:
    """
    Résultat de validate_limits() : valid vaut True si la configuration est
    utilisable ; sinon errors décrit chaque incohérence trouvée. Ajouté à
    la suite de la revue extérieure décrite en tête de ce fichier.
    """
    valid:  bool
    errors: List[str]

    def __str__(self) -> str:
        if self.valid:
            return "[LIMITS] Configuration valide."
        return "[LIMITS] " + "; ".join(self.errors)


def validate_limits(joint_id: str, limits: JointLimits) -> LimitsValidationResult:
    """
    Vérifie qu'une configuration de limites (JointLimits) est cohérente
    avant qu'elle serve à valider des commandes. Demandé par la revue
    extérieure décrite en tête de ce fichier : des limites mal configurées
    fausseraient en silence tous les contrôles qui s'appuient dessus.

    Contrôles effectués :
      - pos_min, pos_max, vel_max, torque_max, current_max et
        temperature_max sont finis (ni NaN ni infini) ; les limites
        facultatives accel_max et position_delta_max ne sont pas
        vérifiées ici ;
      - pos_min < pos_max ;
      - vel_max, torque_max et current_max sont strictement positifs : ce
        sont des amplitudes, physiquement jamais négatives ;
      - joint_id n'est pas vide.

    Ne lève pas d'exception : renvoie la liste des erreurs. C'est
    JointSafetyGuard qui refuse de se construire si elle n'est pas vide
    (sauf avec strict_init=False).
    """
    errors: List[str] = []

    for name, val in [("pos_min", limits.pos_min), ("pos_max", limits.pos_max),
                      ("vel_max", limits.vel_max), ("torque_max", limits.torque_max),
                      ("current_max", limits.current_max),
                      ("temperature_max", limits.temperature_max)]:
        if not math.isfinite(val):
            errors.append(f"{name}={val} n'est pas une valeur finie")

    if math.isfinite(limits.pos_min) and math.isfinite(limits.pos_max):
        if limits.pos_min >= limits.pos_max:
            errors.append(f"pos_min({limits.pos_min}) >= pos_max({limits.pos_max})")

    for name, val in [("vel_max", limits.vel_max), ("torque_max", limits.torque_max),
                      ("current_max", limits.current_max)]:
        if math.isfinite(val) and val <= 0:
            errors.append(f"{name}={val} doit être strictement positif (amplitude)")

    if joint_id.strip() == "":
        errors.append("joint_id vide")

    return LimitsValidationResult(valid=(len(errors) == 0), errors=errors)


def seuils_thermiques(limits: JointLimits) -> tuple:
    """(début du bridage, seuil critique), en °C, d'une articulation. Le
    seuil critique est le plus bas de JointSafetyGuard.TEMPERATURE_CRITICAL_C
    et de limits.temperature_max : une limite déclarée plus basse rend le
    garde-fou plus strict, jamais plus permissif. Le bridage commence
    THERMAL_DERATING_SPAN_C plus bas."""
    critique = JointSafetyGuard.TEMPERATURE_CRITICAL_C
    tmax = limits.temperature_max
    if math.isfinite(tmax) and 0.0 < tmax < critique:
        critique = tmax
    return critique - JointSafetyGuard.THERMAL_DERATING_SPAN_C, critique


def facteur_thermique(temperature: float, limits: JointLimits) -> float:
    """Part des vitesses et couples maximaux permise à cette température :
    1 sous le début du bridage, puis jusqu'à THERMAL_DERATING_MIN au seuil
    critique, 0 au-delà ou pour une température non finie."""
    if not math.isfinite(temperature):
        return 0.0
    debut, critique = seuils_thermiques(limits)
    if temperature >= critique:
        return 0.0
    if temperature <= debut:
        return 1.0
    mini = JointSafetyGuard.THERMAL_DERATING_MIN
    return 1.0 - (1.0 - mini) * (temperature - debut) / (critique - debut)


class JointSafetyGuard:
    """
    Garde-fou de sécurité d'une seule articulation (joint). Chaque IActuator
    (l'interface commune des pilotes de moteur) en crée un dans son
    __init__ et lui soumet chaque commande avant de l'envoyer au moteur.

    Les limites sont vérifiées par validate_limits() dès la construction :
    avec des limites incohérentes, le constructeur lève une exception
    immédiatement, plutôt que de laisser passer en silence des commandes
    mal contraintes. strict_init=False désactive ce refus (le résultat de
    la vérification est alors ignoré).

    Le gardien accepte ou refuse une commande ; il ne la modifie jamais.
    """

    # Seuils de température moteur, en degrés Celsius. Au seuil critique
    # (le plus bas de TEMPERATURE_CRITICAL_C et de limits.temperature_max,
    # voir seuils_thermiques), toute commande est refusée et le retour du
    # moteur est signalé (voir check_feedback_health). Avant, ce seuil
    # était fixe et limits.temperature_max n'était pas lue.
    # Entre le début du bridage (THERMAL_DERATING_SPAN_C sous le seuil
    # critique, 65 °C pour 80 °C : TEMPERATURE_WARNING_C) et le seuil
    # critique, vitesse, couple et écart de position commandés sont
    # plafonnés à une part de vel_max, torque_max et position_delta_max qui
    # descend jusqu'à THERMAL_DERATING_MIN : la commande qui dépasse est
    # refusée (THERMAL_DERATING), jamais réduite.
    TEMPERATURE_CRITICAL_C = 80.0
    TEMPERATURE_WARNING_C  = 65.0
    THERMAL_DERATING_SPAN_C = 15.0
    THERMAL_DERATING_MIN = 0.5

    def __init__(self, joint_id: str, limits: JointLimits, strict_init: bool = True,
                motion_tracker: Optional[MotionStateTracker] = None,
                logger: Optional[StructuredLogger] = None):
        result = validate_limits(joint_id, limits)
        if not result.valid and strict_init:
            raise ValueError(
                f"JointSafetyGuard({joint_id}) refusé : {result}"
            )
        self.joint_id = joint_id
        self.limits   = limits
        self._last_violation: Optional[SafetyViolation] = None
        # Journal structuré : même principe que WatchdogSystem, il est
        # injecté, ou à défaut c'est l'instance partagée par défaut qui
        # sert. Chaque violation détectée par ce gardien y est conservée
        # durablement (le journal est écrit dans des fichiers, voir
        # hal/core/structured_log.py), alors même que la commande a été
        # refusée et n'a jamais été exécutée sur le matériel. But :
        # repérer après coup une app qui envoie systématiquement des
        # commandes invalides, même si aucune de ces commandes n'a jamais
        # atteint les moteurs.
        self._struct_logger = logger or get_default_logger()

        # MotionStateTracker (l'historique des commandes de chaque
        # articulation) : optionnel, injecté par IRobotHAL (l'interface du
        # HAL pour un robot complet), jamais créé ici. C'est un choix de
        # conception : ce composant est générique et partagé, voir
        # hal/core/motion_state.py. Sans lui (None), la limitation
        # d'accélération est désactivée ; le reste du comportement du
        # gardien ne change pas.
        self._motion_tracker = motion_tracker

    def _record_violation(self, v: SafetyViolation) -> SafetyViolation:
        """
        Point unique d'enregistrement d'une violation (dernière violation
        et journal structuré). Avant, chaque cas de rejet de
        validate_command() répétait lui-même self._last_violation = v ;
        centraliser garantit qu'aucune règle ajoutée plus tard n'oubliera
        cet enregistrement.
        """
        self._last_violation = v
        self._struct_logger.log(
            LogCategory.SAFETY_VIOLATION, v, robot_id=None
        )
        return v

    def validate_command(
        self,
        command: ActuatorCommand,
        feedback: Optional[ActuatorFeedback] = None
    ) -> tuple[bool, Optional[SafetyViolation]]:
        """
        Accepte ou refuse une commande avant son envoi au moteur.

        feedback est le dernier retour du moteur (position, vitesse,
        couple, courant et température mesurés). Il peut manquer, par
        exemple au tout premier appel, avant toute lecture des capteurs :
        les contrôles qui en dépendent sont alors sautés.

        Renvoie (True, None) si la commande est acceptée, sinon
        (False, violation) pour la première règle enfreinte. Ordre des
        contrôles : valeurs invalides, position, écart de position,
        vitesse, couple, courant, gains d'impédance, température (critique,
        non finie, bridage), accélération.
        """

        # Valeurs NaN ou infinies, signalées par la revue extérieure
        # décrite en tête de fichier. Vérifiées explicitement AVANT toute
        # comparaison, pour donner le bon diagnostic : sinon un NaN est
        # rejeté sous POSITION_LIMIT, ce qui égare sur la vraie cause.
        for field_name, val in [("position", command.position),
                                ("velocity", command.velocity),
                                ("torque", command.torque),
                                ("current", command.current)]:
            if val is not None and not math.isfinite(val):
                v = SafetyViolation(
                    joint_id=self.joint_id, rule="INVALID_VALUE",
                    value=val, limit=0.0, kind=SafetyErrorKind.INVALID_VALUE,
                )
                self._record_violation(v)
                return False, v

        # Position : vérifiée dès que command.position est renseignée,
        # QUEL QUE SOIT command.mode. Un audit de sécurité, en exécutant
        # réellement le code, a montré que l'ancienne version ne la
        # vérifiait qu'en mode POSITION. C'était le seul champ soumis au
        # mode (vitesse, couple et courant sont vérifiés sans condition,
        # plus bas), alors que certains pilotes transmettent la position
        # (q) au moteur quel que soit le mode. C'était le cas de
        # G1DDSActuator.send_command, le pilote du robot Unitree G1 réel
        # qui parle aux moteurs par DDS (Data Distribution Service, le
        # système de messages par lequel on commande les moteurs du G1).
        # Ce pilote a depuis été corrigé de la même façon, mais ce
        # gardien doit rester la barrière de référence : un futur pilote
        # ne doit pas pouvoir la contourner simplement parce qu'une
        # valeur non voulue reste dans un champ que son mode n'utilise
        # pas. Détails : docs/audit_externe_2026-09-14.md et CHANGELOG.md,
        # section « Corrections critiques et élevées (audit externe du
        # 2026-09-14) ».
        if command.position is not None:
            if not (self.limits.pos_min <= command.position <= self.limits.pos_max):
                v = SafetyViolation(
                    joint_id=self.joint_id, rule="POSITION_LIMIT",
                    value=command.position,
                    limit=self.limits.pos_max if command.position > 0
                          else self.limits.pos_min,
                    kind=SafetyErrorKind.POSITION_LIMIT,
                )
                self._record_violation(v)
                return False, v

            # Écart entre la consigne et la position mesurée, ajouté après
            # un audit de sécurité. Pourquoi : en contrôle de position, le
            # couple réellement produit n'est pas command.torque, mais
            # celui du contrôleur PD (proportionnel-dérivé) intégré au
            # moteur, qui croît avec cet écart. Une consigne pourtant
            # comprise entre pos_min et pos_max peut donc produire un
            # couple bien supérieur à torque_max si elle est trop loin de
            # la position actuelle. Justification physique complète :
            # JointLimits.position_delta_max ; historique : CHANGELOG.md,
            # section « audit externe, vague 2 ».
            # Le contrôle ne s'applique que si la limite est configurée et
            # qu'un retour réel (et fini) du moteur est disponible, ce qui
            # n'est pas le cas au tout premier appel, avant toute lecture
            # des capteurs. Sinon il ne fait rien et le comportement reste
            # celui d'avant, compatible avec tous les appelants existants.
            if (self.limits.position_delta_max is not None
                    and feedback is not None
                    and math.isfinite(feedback.position)):
                delta = abs(command.position - feedback.position)
                if delta > self.limits.position_delta_max:
                    v = SafetyViolation(
                        joint_id=self.joint_id, rule="POSITION_DELTA_LIMIT",
                        value=delta, limit=self.limits.position_delta_max,
                        kind=SafetyErrorKind.POSITION_DELTA_LIMIT,
                    )
                    self._record_violation(v)
                    return False, v

        # Vitesse, en valeur absolue (la limite vaut dans les deux sens).
        if command.velocity is not None:
            if abs(command.velocity) > self.limits.vel_max:
                v = SafetyViolation(
                    joint_id=self.joint_id, rule="VELOCITY_LIMIT",
                    value=abs(command.velocity), limit=self.limits.vel_max,
                    kind=SafetyErrorKind.VELOCITY_LIMIT,
                )
                self._record_violation(v)
                return False, v

        # Couple, en valeur absolue.
        if command.torque is not None:
            if abs(command.torque) > self.limits.torque_max:
                v = SafetyViolation(
                    joint_id=self.joint_id, rule="TORQUE_LIMIT",
                    value=abs(command.torque), limit=self.limits.torque_max,
                    kind=SafetyErrorKind.TORQUE_LIMIT,
                )
                self._record_violation(v)
                return False, v

        # Courant, en valeur absolue.
        if command.current is not None:
            if abs(command.current) > self.limits.current_max:
                v = SafetyViolation(
                    joint_id=self.joint_id, rule="CURRENT_LIMIT",
                    value=abs(command.current), limit=self.limits.current_max,
                    kind=SafetyErrorKind.CURRENT_LIMIT,
                )
                self._record_violation(v)
                return False, v

        # Gains d'impédance : stiffness (la raideur) et damping
        # (l'amortissement). Un audit de sécurité a montré qu'ils n'étaient
        # jamais validés : NaN ou des valeurs aberrantes (par exemple 1e9)
        # passaient ce gardien sans bruit. Il n'existe pas de limite
        # physique connue par articulation pour ces gains : contrairement à
        # torque_max ou vel_max, l'URDF (le fichier qui décrit les
        # articulations du robot et leurs limites) ne fournit rien de tel.
        # La validation se limite donc à ce que tout gain PD doit
        # respecter : être fini et non négatif (un gain négatif inverserait
        # le sens de la correction du contrôleur). Refuser ici, même sans
        # borne supérieure connue, vaut mieux que ne rien vérifier du tout.
        # Limite connue : une valeur finie et positive mais aberrante,
        # comme 1e9, passe donc encore. Historique : CHANGELOG.md, section
        # « audit externe, vague 3 ».
        for field_name, val in (("stiffness", command.stiffness),
                                ("damping", command.damping)):
            if val is not None:
                if not math.isfinite(val):
                    v = SafetyViolation(
                        joint_id=self.joint_id, rule="INVALID_VALUE",
                        value=val, limit=0.0, kind=SafetyErrorKind.INVALID_VALUE,
                    )
                    self._record_violation(v)
                    return False, v
                if val < 0:
                    v = SafetyViolation(
                        joint_id=self.joint_id, rule=f"{field_name.upper()}_NEGATIVE",
                        value=val, limit= -1.0, kind=SafetyErrorKind.INVALID_VALUE,
                    )
                    self._record_violation(v)
                    return False, v

        # Température mesurée : à partir du seuil critique, toute commande
        # est refusée ; entre le début du bridage et lui, vitesse et couple
        # commandés sont plafonnés (voir TEMPERATURE_CRITICAL_C). Une
        # température non finie est un danger, pas une absence de donnée :
        # la commande est refusée (avant, elle était ignorée ici).
        if feedback is not None:
            temp = feedback.temperature
            debut, critique = seuils_thermiques(self.limits)
            if not math.isfinite(temp):
                v = SafetyViolation(
                    joint_id=self.joint_id, rule="TEMPERATURE_UNKNOWN",
                    value=temp, limit=critique, kind=SafetyErrorKind.INVALID_VALUE,
                )
                self._record_violation(v)
                return False, v
            if temp >= critique:
                v = SafetyViolation(
                    joint_id=self.joint_id, rule="TEMPERATURE_CRITICAL",
                    value=temp, limit=critique,
                    kind=SafetyErrorKind.TEMPERATURE_CRITICAL,
                )
                self._record_violation(v)
                return False, v
            if temp > debut:
                k = facteur_thermique(temp, self.limits)
                bornes = [(command.velocity, self.limits.vel_max),
                          (command.torque, self.limits.torque_max)]
                # En position (le G1), le couple vient du PD du moteur et
                # croît avec l'écart : c'est lui qu'il faut brider.
                if (self.limits.position_delta_max is not None
                        and command.position is not None
                        and math.isfinite(feedback.position)):
                    bornes.append((command.position - feedback.position,
                                   self.limits.position_delta_max))
                for val, maxi in bornes:
                    if val is not None and abs(val) > maxi * k:
                        v = SafetyViolation(
                            joint_id=self.joint_id, rule="THERMAL_DERATING",
                            value=abs(val), limit=maxi * k,
                            kind=SafetyErrorKind.THERMAL_DERATING,
                        )
                        self._record_violation(v)
                        return False, v

        # Accélération (slew rate) : la commande est bloquée, jamais
        # lissée. C'est une décision de conception arrêtée : le gardien
        # reste une barrière qui accepte ou refuse, jamais un contrôleur de
        # mouvement qui modifierait la commande. L'accélération est déduite
        # de l'écart de vitesse avec la commande précédente (voir
        # JointLimits.accel_max). Il faut un MotionStateTracker injecté et
        # limits.accel_max défini ; sans l'un ou l'autre, ce contrôle ne
        # fait rien et le comportement reste celui d'avant. Il ne
        # s'applique qu'en mode POSITION, et seulement quand un historique
        # existe (pas à la toute première commande).
        if (self._motion_tracker is not None and self.limits.accel_max is not None
                and command.mode == ControlMode.POSITION):
            delta = self._motion_tracker.compute_command_delta(self.joint_id, command)
            if delta is not None and delta.has_history and delta.implied_acceleration is not None:
                accel_abs = abs(delta.implied_acceleration)
                if accel_abs > self.limits.accel_max:
                    v = SafetyViolation(
                        joint_id=self.joint_id, rule="ACCELERATION_LIMIT",
                        value=accel_abs, limit=self.limits.accel_max,
                        kind=SafetyErrorKind.ACCELERATION_LIMIT,
                    )
                    self._record_violation(v)
                    return False, v

        return True, None

    def check_feedback_health(
        self, feedback: ActuatorFeedback
    ) -> list[SafetyViolation]:
        """
        Vérifie un retour du moteur (feedback), indépendamment de toute
        commande. Un retour qui contient une valeur non finie est lui-même
        une anomalie : un capteur qui renvoie NaN ne fournit pas une donnée
        exploitable.

        Renvoie la liste des violations trouvées, vide si tout va bien.
        Elles ne sont pas enregistrées ici : c'est l'appelant qui décide de
        la réaction. Les pilotes du G1 (hal/drivers/unitree_g1/robot.py et
        g1_dds_actuator.py) déclenchent un arrêt d'urgence sur
        TORQUE_OVERLOAD ou CURRENT_OVERLOAD.
        """
        violations = []

        for field_name in ("position", "velocity", "torque", "current", "temperature"):
            val = getattr(feedback, field_name)
            if not math.isfinite(val):
                violations.append(SafetyViolation(
                    joint_id=self.joint_id, rule="INVALID_VALUE",
                    value=val, limit=0.0, kind=SafetyErrorKind.INVALID_VALUE,
                ))
                return violations  # retour inexploitable : on s'arrête là

        critique = seuils_thermiques(self.limits)[1]
        if feedback.temperature >= critique:
            violations.append(SafetyViolation(
                joint_id=self.joint_id, rule="TEMPERATURE_CRITICAL",
                value=feedback.temperature, limit=critique,
                kind=SafetyErrorKind.TEMPERATURE_CRITICAL,
            ))
        # Couple mesuré au-delà de 110 % de torque_max : signe qu'un moteur
        # force contre un obstacle, par exemple un bras coincé (voir
        # hal/drivers/unitree_g1/robot.py).
        if abs(feedback.torque) > self.limits.torque_max * 1.1:
            violations.append(SafetyViolation(
                joint_id=self.joint_id, rule="TORQUE_OVERLOAD",
                value=abs(feedback.torque), limit=self.limits.torque_max * 1.1,
                kind=SafetyErrorKind.TORQUE_OVERLOAD,
            ))
        # Courant mesuré au-delà de 110 % de current_max, ajouté après un
        # audit de sécurité. Un second audit extérieur a ensuite montré que
        # sur le G1 réel, feedback.current vaut TOUJOURS 0.0 : MotorState_
        # (le message d'état des moteurs du G1) n'expose pas le courant.
        # Sans condition, ce contrôle ne pouvait donc jamais se déclencher
        # sur le vrai robot, tout en donnant l'apparence d'une protection
        # active. Il n'est émis que si la valeur est explicitement
        # documentée comme une vraie mesure (voir
        # JointLimits.current_is_measured). Historique : CHANGELOG.md,
        # sections « audit externe, vague 3 » et « suite au deuxième audit
        # externe ».
        if self.limits.current_is_measured and abs(feedback.current) > self.limits.current_max * 1.1:
            violations.append(SafetyViolation(
                joint_id=self.joint_id, rule="CURRENT_OVERLOAD",
                value=abs(feedback.current), limit=self.limits.current_max * 1.1,
                kind=SafetyErrorKind.CURRENT_OVERLOAD,
            ))
        return violations

    @property
    def last_violation(self) -> Optional[SafetyViolation]:
        """Dernière violation enregistrée par validate_command(), ou None."""
        return self._last_violation
