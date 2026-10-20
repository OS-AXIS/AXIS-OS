"""
AXIS-OS — Moniteur de sécurité indépendant
==========================================
Pourquoi ce module existe : un garde-fou qui dépend du même code que ce
qu'il surveille se trompe en même temps que lui. Ce moniteur apporte
donc une redondance « diverse » (un second contrôle bâti par un autre
chemin logiciel) et non une simple duplication (la même vérification
écrite deux fois) ; la distinction compte. C'est l'une des cinq
recommandations de renforcement formulées à la suite du second audit
externe du code (voir CHANGELOG.md, section « 5 recommandations
externes de renforcement », du 2026-09-15).

Les contrôles existants, JointSafetyGuard (le garde-fou qui vérifie
chaque commande d'articulation avant qu'elle n'atteigne le moteur) et
WatchdogSystem (la surveillance en temps réel du robot complet), sont
construits par le même chemin logiciel que la commande elle-même, ou en
dépendent : kinematic_normalizer (la conversion entre la convention du
fabricant et la convention commune d'AXIS-OS, sens et décalage de
chaque articulation) et les pilotes de moteur du robot humanoïde
Unitree G1, G1DDSActuator (robot réel, piloté par DDS, le système de
messages par lequel on parle aux moteurs du G1) et G1SimulatedActuator
(robot simulé). Si ce chemin contient une erreur de conception, le
garde-fou qui en dépend est faux avec lui. C'est exactement le défaut
qu'a relevé l'audit externe du 2026-09-14 (voir CHANGELOG.md et
docs/audit_externe_2026-09-14.md) : JointSafetyGuard validait un couple
dont kinematic_normalizer avait déjà faussé le signe, sans rien
signaler (sur les 9 articulations du G1 dont le sens est inversé par
rapport à la convention d'AXIS-OS, le couple partait au moteur réel
avec le signe opposé à celui demandé). Le garde-fou ne pouvait pas
détecter cette erreur, puisqu'il partageait la même vision faussée du
monde que le composant qu'il devait surveiller.

Ce que fait le module : il lit le retour des moteurs
(feedback.position/velocity/torque) en valeurs brutes, c'est-à-dire
dans le repère mécanique du fabricant, directement depuis
IRobotHAL.get_full_state(). IRobotHAL est l'interface que chaque robot
fournit au HAL (la couche d'abstraction matérielle, seul chemin vers
les moteurs). Cette lecture ne passe jamais par NormalizedRobotHAL (la
vue normalisée du robot présentée aux apps) ni par KinematicNormalizer.
Les valeurs sont comparées aux limites du fichier URDF officiel (le
fichier qui décrit les articulations du robot et leurs limites), lues
directement par urdf_loader.py, jamais via robot.py ni
_build_g1_joint_definitions(), qui pourraient un jour diverger du
fichier source. Dans cette version, seule la position est comparée aux
limites (voir _check_once()).

Le module n'importe ni axis_os.hal.core.kinematic_normalizer ni
axis_os.hal.drivers.unitree_g1.robot. test/test_independent_monitor.py
le vérifie explicitement par une assertion sur les imports du module
lui-même (il interdit aussi g1_dds_actuator), et pas seulement par un
test de comportement : une future réorganisation du code ne peut donc
pas réintroduire ce couplage sans bruit et vider ce module de sa valeur
de redondance.

Ce moniteur peut déclencher lui-même un e-stop (arrêt d'urgence) par
IRobotHAL.emergency_stop_all(), sans jamais consulter JointSafetyGuard
ni WatchdogSystem pour en décider. Seul ce type de redondance attrape
les erreurs de CONCEPTION, c'est-à-dire une mauvaise hypothèse partagée
par tout le chemin de commande. La redondance par duplication (la même
vérification réécrite deux fois) reproduit fidèlement le même bug si
l'hypothèse de départ était fausse.

Limite assumée : ce moniteur ne peut pas distinguer une position hors
limites due à un vrai problème mécanique (butée franchie) d'une
position hors limites due à une erreur dans sa propre lecture du
fichier URDF. Il partage cette dépendance avec tout le reste du
système. C'est un choix pragmatique, pas un oubli : le fichier URDF
officiel reste la source de vérité la plus fiable disponible.

Comme tout le logiciel de ce dépôt, ce moniteur n'est pas une fonction
de sécurité au sens des normes (ISO 13849, IEC 61508) : seul un arrêt
d'urgence matériel câblé, qui coupe directement l'alimentation des
moteurs, peut l'être. Voir docs/hardware_estop_notes.md.

Licence : Apache 2.0
"""

from __future__ import annotations
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Union

from axis_os.hal.core.interfaces import IRobotHAL
from axis_os.hal.core.urdf_loader import load_urdf
from axis_os.hal.core.structured_log import (
    StructuredLogger, LogCategory,
)


@dataclass(frozen=True)
class IndependentMonitorAlert:
    """
    Alerte émise quand une articulation sort de ses limites.

    joint_id : nom de l'articulation (le même dans le HAL et dans
        le fichier URDF).
    reason : motif, par exemple "POSITION_OUT_OF_BOUNDS_INDEPENDENT".
    measured : valeur brute mesurée, dans le repère du fabricant.
    limit : borne franchie, après application de margin_ratio.
    timestamp_ns : heure de détection en nanosecondes (time.time_ns()).
    """
    joint_id: str
    reason: str
    measured: float
    limit: float
    timestamp_ns: int


class IndependentSafetyMonitor:
    """
    Surveillance des positions d'articulation, indépendante du chemin
    de commande.

    Tourne dans son propre thread (un fil d'exécution parallèle) et
    interroge à intervalle régulier IRobotHAL.get_full_state()
    directement, jamais via NormalizedRobotHAL. Les limites viennent du
    fichier URDF officiel, lu par urdf_loader.load_urdf(), et jamais de
    axis_os.hal.drivers.unitree_g1.robot._build_g1_joint_definitions() :
    cette fonction pourrait diverger du fichier source sans que ce
    moniteur puisse le remarquer autrement.

    Usage :
        monitor = IndependentSafetyMonitor(hal, "chemin/vers/robot.urdf")
        monitor.subscribe(lambda alerte: ...)
        monitor.start()
        ...
        monitor.stop()
    """

    def __init__(
        self,
        robot_hal: IRobotHAL,
        urdf_path: Union[str, Path],
        poll_interval_s: float = 0.05,
        margin_ratio: float = 1.0,
        logger: Optional[StructuredLogger] = None,
    ):
        """
        robot_hal : le HAL du robot, lu directement (valeurs brutes).
        urdf_path : chemin du fichier URDF officiel du robot.
        poll_interval_s : pause entre deux vérifications, en secondes
            (0.05 par défaut, soit au plus 20 vérifications par seconde).
        logger : journal structuré où sont écrites les alertes ; un
            StructuredLogger neuf est créé s'il n'est pas fourni.

        margin_ratio : multiplicateur appliqué aux limites URDF. Avec 1.0
        (valeur par défaut), ce sont strictement les limites officielles,
        sans aucune tolérance ; en dessous de 1.0, l'intervalle autorisé
        se resserre (test/test_independent_monitor.py utilise 0.5), et
        au-dessus il s'élargit. Ce
        réglage est volontairement séparé de toute marge appliquée
        ailleurs dans le système (par exemple le seuil TORQUE_OVERLOAD à
        110 % dans JointSafetyGuard) : ce moniteur applique sa propre
        politique, décidée indépendamment. C'est une autre facette de la
        redondance diverse recherchée ici.
        """
        self._hal = robot_hal
        self._model = load_urdf(urdf_path)
        # Bornes effectives (pos_min, pos_max) de chaque articulation,
        # après margin_ratio. Pour que le ratio agisse dans le même sens
        # des deux côtés, une borne basse négative est multipliée et une
        # borne basse positive ou nulle est divisée. La borne haute est
        # toujours multipliée : cela n'agit dans le bon sens que si elle
        # est positive, ce qui est le cas de toutes les articulations du
        # fichier URDF du G1.
        self._limits_by_joint: Dict[str, tuple] = {
            j.joint_id: (j.limits.pos_min * margin_ratio if j.limits.pos_min < 0
                        else j.limits.pos_min / margin_ratio,
                        j.limits.pos_max * margin_ratio)
            for j in self._model.get_joints()
        }
        self._poll_interval_s = poll_interval_s
        self._logger = logger or StructuredLogger()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._estop_triggered = False
        # Protège _alerts et _estop_triggered, écrits par le thread du
        # moniteur et lus depuis d'autres threads.
        self._lock = threading.Lock()
        self._alerts: List[IndependentMonitorAlert] = []
        self._subscribers: List = []

    def subscribe(self, callback) -> None:
        """
        Enregistre callback, une fonction de rappel appelée à chaque
        alerte avec l'IndependentMonitorAlert : callback(alerte) -> None.

        Une exception levée par un abonné est ignorée : elle n'interrompt
        jamais la boucle de surveillance (voir _trigger()). Les abonnés
        sont appelés dans le thread du moniteur, avant l'appel à l'e-stop :
        un abonné lent retarde donc l'arrêt d'urgence.
        """
        self._subscribers.append(callback)

    def start(self) -> None:
        """
        Lance la surveillance dans un thread séparé ; sans effet si elle
        tourne déjà. Le thread est marqué « daemon » : il n'empêche pas
        le programme de se terminer.
        """
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._poll_loop, daemon=True,
            name="axis-os-independent-monitor",
        )
        self._thread.start()

    def stop(self) -> None:
        """
        Demande l'arrêt de la boucle et attend la fin du thread,
        2 secondes au plus. Un e-stop déjà déclenché n'est pas annulé.
        """
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    @property
    def estop_triggered(self) -> bool:
        """True dès que ce moniteur a déclenché un e-stop. Cette classe
        ne le remet jamais à False."""
        with self._lock:
            return self._estop_triggered

    def get_alerts(self) -> List[IndependentMonitorAlert]:
        """Copie des alertes émises depuis la création du moniteur, de la
        plus ancienne à la plus récente."""
        with self._lock:
            return list(self._alerts)

    def _poll_loop(self) -> None:
        """Boucle du thread : une vérification, puis une pause de
        poll_interval_s, jusqu'à l'appel de stop()."""
        while self._running:
            try:
                self._check_once()
            except Exception as exc:
                # Ce moniteur ne doit jamais s'arrêter ni échouer en
                # silence. Avaler l'exception et continuer comme si de
                # rien n'était serait exactement le genre de défaillance
                # silencieuse que ce module existe pour combattre
                # ailleurs dans le système. Toute erreur, même
                # inattendue, est donc affichée, puis la surveillance
                # reprend au tour suivant.
                print(f"  [INDEPENDENT MONITOR] Erreur interne (non fatale, "
                      f"continue) : {type(exc).__name__}: {exc}")
            time.sleep(self._poll_interval_s)

    def _check_once(self) -> None:
        """
        Un tour de vérification : compare la position brute de chaque
        articulation aux bornes calculées dans __init__. La vitesse et le
        couple ne sont pas contrôlés ici pour l'instant.
        """
        feedbacks = self._hal.get_full_state()
        for joint_id, fb in feedbacks.items():
            limits = self._limits_by_joint.get(joint_id)
            if limits is None:
                continue  # articulation hors de l'URDF chargé (ex. : capteur monté)
            pos_min, pos_max = limits
            if not (pos_min <= fb.position <= pos_max):
                self._trigger(joint_id, "POSITION_OUT_OF_BOUNDS_INDEPENDENT",
                              fb.position,
                              pos_max if fb.position > pos_max else pos_min)

    def _trigger(self, joint_id: str, reason: str, measured: float, limit: float) -> None:
        """
        Enregistre l'alerte, l'écrit dans le journal, prévient les
        abonnés, puis déclenche l'e-stop si ce n'est pas déjà fait.

        Tant qu'une articulation reste hors limites, chaque tour de
        boucle produit une nouvelle alerte ; l'arrêt d'urgence, lui,
        n'est demandé qu'une fois, au premier dépassement.
        """
        alert = IndependentMonitorAlert(
            joint_id=joint_id, reason=reason, measured=measured,
            limit=limit, timestamp_ns=time.time_ns(),
        )
        with self._lock:
            self._alerts.append(alert)
            already_triggered = self._estop_triggered
            self._estop_triggered = True

        self._logger.log(LogCategory.SAFETY_VIOLATION, {
            "source": "independent_monitor", "joint_id": joint_id,
            "reason": reason, "measured": measured, "limit": limit,
        })

        for cb in self._subscribers:
            try:
                cb(alert)
            except Exception:
                pass  # un abonné défaillant ne doit jamais bloquer ce moniteur

        # La décision ne consulte ni JointSafetyGuard ni WatchdogSystem :
        # c'est tout l'intérêt de ce moniteur.
        if not already_triggered:
            print(f"  [INDEPENDENT MONITOR] ⚠️  {reason} sur {joint_id} : "
                  f"{measured:.4f} (limite {limit:.4f}) — e-stop déclenché, "
                  f"INDÉPENDAMMENT de JointSafetyGuard/WatchdogSystem.")
            self._hal.emergency_stop_all()
