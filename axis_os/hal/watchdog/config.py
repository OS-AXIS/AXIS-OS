"""
AXIS-OS — Watchdog Configuration
===================================
Profils de surveillance par déploiement.

Le watchdog (chien de garde) est le thread qui surveille en continu
chaque articulation (joint) du robot et déclenche l'e-stop (arrêt
d'urgence) quand une mesure devient dangereuse ; il est défini dans
axis_os/hal/watchdog/system.py. Ce fichier regroupe ses seuils et trois
profils prêts à l'emploi : standard, strict et simulation.

Licence : Apache 2.0 (open-source)
"""

from __future__ import annotations
from dataclasses import dataclass


@dataclass
class WatchdogConfig:
    """
    Seuils et réglages du WatchdogSystem. Les durées sont en secondes
    (suffixe _s) ou en millisecondes (suffixe _ms), les températures en
    degrés Celsius (suffixe _c).

    Vocabulaire : le feedback est la mesure que chaque moteur renvoie à
    chaque cycle (position, vitesse, couple, température, état). Les
    sévérités d'alerte vont de INFO à FATAL (voir events.py) ; une
    alerte FATAL peut déclencher l'e-stop automatique. Les limites de
    chaque joint (JointLimits) viennent du HAL (la couche d'abstraction
    matérielle, seul chemin vers les moteurs).
    """
    # Période visée entre deux cycles de surveillance (10 ms, soit
    # 100 cycles par seconde).
    poll_interval_s:           float = 0.01
    # Sans nouveau feedback d'un joint depuis comm_timeout_s : alerte
    # WARNING (COMM_TIMEOUT). Depuis comm_lost_s : la communication est
    # considérée perdue, alerte FATAL (COMM_LOST) et e-stop si
    # auto_estop_on_comm_lost est vrai.
    comm_timeout_s:            float = 0.05
    comm_lost_s:               float = 0.20
    # Période de l'alerte HEARTBEAT (signal de vie, sévérité INFO) qui
    # résume l'activité du watchdog.
    heartbeat_interval_s:      float = 5.0
    # Température moteur : WARNING à partir du premier seuil, FATAL (et
    # e-stop si auto_estop_on_temp_critical est vrai) à partir du second.
    temp_warning_c:            float = 65.0
    temp_critical_c:           float = 80.0
    # Multiplicateurs appliqués aux limites de chaque joint (torque_max,
    # vel_max dans JointLimits) : 1.10 signifie « alerte au-delà de
    # 110 % de la limite ».
    torque_overload_ratio:     float = 1.10
    velocity_overrun_ratio:    float = 1.10
    # Pas encore utilisé par WatchdogSystem : le courant mesuré est
    # surveillé par JointSafetyGuard (le garde-fou de chaque joint), dans
    # check_feedback_health() (axis_os/hal/safety/limits.py), avec son
    # propre seuil fixe de 110 % de current_max, et seulement quand le
    # pilote fournit une vraie mesure du courant (ce n'est pas le cas
    # sur le G1 réel ; voir ce fichier).
    current_overload_ratio:    float = 1.10
    # Nombre de mesures consécutives hors limite exigées avant une
    # alerte CRITICAL, pour éviter les faux positifs : un pic isolé ne
    # suffit pas. S'applique au couple, à la vitesse, à la position hors
    # limites et à l'écart entre commande et feedback.
    confirm_samples:           int   = 3
    # Durée maximale d'un cycle de surveillance. Au-delà, le watchdog se
    # déclare lui-même dégradé (alerte CRITICAL) : trop lent, il ne
    # surveille plus le robot de façon fiable.
    max_cycle_time_ms:         float = 100.0
    position_drift_max_rad:    float = 0.15
    """
    Écart maximal toléré, en radians, entre la dernière commande de
    position envoyée et le feedback réel, avant de considérer que le
    joint ne suit plus sa consigne (moteur bloqué, calibration qui a
    dérivé, etc.) : alerte CRITICAL (POSITION_DRIFT) après
    confirm_samples dépassements consécutifs.

    Ce contrôle a besoin d'un MotionStateTracker (le composant qui
    garde, pour chaque joint, la dernière commande et le dernier
    feedback) exposé par le HAL ; sans lui, il n'a aucun effet (voir
    WatchdogSystem._check_feedback_drift).
    """
    # Déclenchement automatique de l'e-stop sur communication perdue,
    # température critique ou joint en défaut (état FAULT signalé par
    # le moteur). Si l'un d'eux est désactivé, l'alerte FATAL
    # correspondante est quand même émise, mais le watchdog n'arrête
    # pas le robot de lui-même.
    auto_estop_on_comm_lost:   bool  = True
    auto_estop_on_temp_critical: bool = True
    auto_estop_on_joint_fault: bool  = True
    # Nom du profil, repris dans les messages et dans get_stats().
    profile_name:              str   = "standard"

    @classmethod
    def standard(cls) -> "WatchdogConfig":
        """Profil par défaut : les valeurs ci-dessus."""
        return cls(profile_name="standard")

    @classmethod
    def strict(cls) -> "WatchdogConfig":
        """
        Profil aux seuils plus serrés : cycle de 5 ms, avertissement de
        communication après 20 ms et perte après 100 ms, température en
        alerte dès 55 °C (critique à 70 °C), tolérance de 5 % au lieu de
        10 % sur le couple et la vitesse (et sur le courant, réglage pas
        encore utilisé par le watchdog).
        """
        return cls(
            poll_interval_s=0.005, comm_timeout_s=0.02, comm_lost_s=0.1,
            temp_warning_c=55.0, temp_critical_c=70.0,
            torque_overload_ratio=1.05, velocity_overrun_ratio=1.05,
            current_overload_ratio=1.05, profile_name="strict"
        )

    @classmethod
    def simulation(cls) -> "WatchdogConfig":
        """
        Profil de simulation : cycle de 20 ms, délais de communication
        relâchés (avertissement après 1 s, perte après 5 s), seuils de
        température fixés à 999 °C (donc jamais atteints) et aucun e-stop
        automatique. Les alertes restent émises, mais le watchdog
        n'arrête jamais le robot de lui-même. Les tests s'en servent pour
        observer les alertes sans arrêt automatique (voir
        test/test_watchdog_v2.py).
        """
        return cls(
            poll_interval_s=0.02, comm_timeout_s=1.0, comm_lost_s=5.0,
            temp_warning_c=999.0, temp_critical_c=999.0,
            auto_estop_on_comm_lost=False,
            auto_estop_on_temp_critical=False,
            auto_estop_on_joint_fault=False,
            profile_name="simulation"
        )
