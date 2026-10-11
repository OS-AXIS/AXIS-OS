/*
 * AXIS-OS — Moniteur de fraîcheur (point 1, recommandation externe)
 * ======================================================================
 * Démon SÉPARÉ du process principal AXIS-OS, écrit en C plutôt qu'en
 * Python — recommandation explicite : ce moniteur doit survivre à la
 * mort de l'interpréteur Python lui-même (deadlock, crash, OOM killer
 * qui cible le plus gros process...), donc il ne doit PARTAGER AUCUN
 * mode de défaillance avec lui. Un moniteur écrit en Python, même dans
 * un process séparé, resterait vulnérable à un bug de l'interpréteur
 * Python lui-même (fuite mémoire du GC, deadlock GIL dans une lib
 * tierce partagée) — improbable, mais c'est précisément la classe de
 * risque que ce composant existe pour éliminer.
 *
 * Rôle : écoute le canal DDS "rt/lowcmd" (le même que G1DDSHub publie,
 * voir axis_os/hal/drivers/unitree_g1/g1_dds_actuator.py). Si aucun
 * message n'est vu pendant FRESHNESS_TIMEOUT_MS alors que le robot est
 * censé être actif, publie lui-même UNE commande d'amortissement
 * (kp=0, kd=DAMPING_KD faible, tau=0, mode=1 Enable) sur les 35
 * moteurs — le robot se relâche doucement au lieu de rester bloqué sur
 * la dernière consigne reçue, qui pourrait être dangereuse si le
 * process principal est mort en plein mouvement.
 *
 * Complémentaire au chien de garde systemd (voir
 * deploy/systemd/axis-os.service) : celui-ci REDÉMARRE le process
 * mort, mais laisse s'écouler jusqu'à WatchdogSec=2s avant de le
 * détecter, PUIS le temps du redémarrage lui-même — pendant tout cet
 * intervalle, le robot reste sur la dernière consigne DDS reçue si
 * rien d'autre n'intervient. Ce moniteur agit BEAUCOUP plus vite
 * (FRESHNESS_TIMEOUT_MS, quelques dizaines de ms) et directement au
 * niveau du bus DDS, sans dépendre d'aucune détection système.
 *
 * ⚠️ SQUELETTE FONCTIONNEL, PAS ENCORE BRANCHÉ SUR UN VRAI BUS DDS —
 * les fonctions dds_subscribe_lowcmd()/dds_publish_damping() sont
 * volontairement isolées et marquées TODO ci-dessous : les brancher
 * sur la vraie CycloneDDS C library + l'IDL officiel unitree_hg
 * (structures LowCmd_/MotorCmd_, identiques à ce que G1DDSHub utilise
 * côté Python — voir sa docstring pour la référence complète) est un
 * vrai travail d'intégration matérielle, non testable dans cet
 * environnement de développement (pas de CycloneDDS C installé ici,
 * pas de vrai bus DDS qui tourne). La LOGIQUE de timeout et
 * d'amortissement ci-dessous est en revanche complète et testée
 * (voir test_freshness_monitor.c) — c'est elle qui compte le plus
 * pour la revue, l'intégration DDS réelle étant plus mécanique une
 * fois la vraie lib disponible sur la cible.
 *
 * Licence : Apache 2.0
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <string.h>
#include "dds/dds.h"
#include "idl/LowCmd_.h"

/* Configuration — voir docstring de module pour la justification des
 * valeurs par défaut. Ajustables sans recompiler via variables
 * d'environnement (voir main()). */
#define DEFAULT_FRESHNESS_TIMEOUT_MS 200
#define DEFAULT_DAMPING_KD 2.0
#define DEFAULT_POLL_INTERVAL_MS 20
#define NUM_MOTORS 35  /* voir MotorCmd_[35] dans le protocole officiel */

typedef struct {
    long timeout_ms;
    double damping_kd;
    long poll_interval_ms;
    bool robot_active;       /* mis à jour par dds_subscribe_lowcmd() */
    long last_message_ms;    /* horodatage monotone du dernier message vu */
} monitor_state_t;

/* Horodatage monotone en millisecondes — jamais l'horloge murale
 * (CLOCK_REALTIME), qui peut sauter en arrière (NTP, changement
 * d'heure) et fausserait le calcul de fraîcheur. */
static long monotonic_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long)ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

/*
 * TODO (intégration matérielle) : brancher sur la vraie CycloneDDS C
 * library — créer un DDS_Subscriber sur le topic "rt/lowcmd" (type
 * unitree_hg::msg::dds_::LowCmd_, même IDL que G1DDSHub côté Python),
 * et à chaque message reçu :
 *   - state->last_message_ms = monotonic_ms();
 *   - state->robot_active = true;
 * `robot_active` devrait passer à false si le process principal se
 * ferme proprement (message explicite, ou canal fermé côté éditeur) —
 * à définir précisément une fois la vraie intégration commencée, pour
 * ne jamais déclencher l'amortissement sur un arrêt VOLONTAIRE.
 *
 * IMPLÉMENTÉ le 2026-09-15 : le fichier IDL officiel unitree_hg
 * n'étant distribué qu'en C++ par Unitree (unitree_sdk2, pas de
 * version C ni fichier .idl source publié tel quel), le schéma
 * LowCmd_/MotorCmd_ a été reconstruit ici en .idl à partir de la
 * structure confirmée identique dans DEUX sources officielles lues
 * ce soir (unitree_sdk2_python généré par idlc pour Python, et
 * unitree_sdk2 généré par idlc pour C++ — mêmes champs, mêmes types,
 * même ordre) - voir le dossier idl/ (fichiers .idl), compilés
 * avec le VRAI idlc de CycloneDDS 0.10.x (voir README.md pour
 * reproduire). ⚠️ Jamais testé contre un vrai bus DDS ni un vrai G1
 * — seule la COMPILATION est vérifiée ici, pas le comportement réseau
 * réel (voir la limite documentée dans README.md).
 */
static dds_entity_t g_participant = 0;
static dds_entity_t g_reader = 0;
static unitree_hg_msg_dds__LowCmd_ *g_sample = NULL;

static int dds_subscribe_lowcmd(monitor_state_t *state) {
    (void)state;

    g_participant = dds_create_participant(DDS_DOMAIN_DEFAULT, NULL, NULL);
    if (g_participant < 0) {
        fprintf(stderr, "[freshness_monitor] dds_create_participant a échoué : %s\n",
                dds_strretcode(-g_participant));
        return -1;
    }

    dds_entity_t topic = dds_create_topic(
        g_participant, &unitree_hg_msg_dds__LowCmd__desc, "rt/lowcmd", NULL, NULL);
    if (topic < 0) {
        fprintf(stderr, "[freshness_monitor] dds_create_topic a échoué : %s\n",
                dds_strretcode(-topic));
        return -1;
    }

    dds_qos_t *qos = dds_create_qos();
    dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, DDS_SECS(1));
    g_reader = dds_create_reader(g_participant, topic, qos, NULL);
    dds_delete_qos(qos);
    if (g_reader < 0) {
        fprintf(stderr, "[freshness_monitor] dds_create_reader a échoué : %s\n",
                dds_strretcode(-g_reader));
        return -1;
    }

    g_sample = unitree_hg_msg_dds__LowCmd___alloc();
    fprintf(stderr, "[freshness_monitor] Abonné à 'rt/lowcmd' — en attente de messages.\n");
    return 0;
}

/* Appelée à chaque cycle de la boucle principale (voir main()) —
 * poll non bloquant, cohérent avec poll_interval_ms plutôt qu'un
 * callback DDS asynchrone : garde la logique de timeout simple et
 * entièrement dans le fil d'exécution principal, plus facile à
 * raisonner et à tester que de la concurrence supplémentaire pour
 * un composant dont la fiabilité est précisément l'enjeu. */
static void dds_poll_lowcmd(monitor_state_t *state) {
    void *samples[1] = { g_sample };
    dds_sample_info_t infos[1];
    dds_return_t rc = dds_take(g_reader, samples, infos, 1, 1);
    if (rc < 0) {
        fprintf(stderr, "[freshness_monitor] dds_take a échoué : %s\n",
                dds_strretcode(-rc));
        return;
    }
    if (rc > 0 && infos[0].valid_data) {
        state->last_message_ms = monotonic_ms();
        state->robot_active = true;
    }
}

/*
 * CRC32 — algorithme EXACT copié depuis le SDK officiel Unitree
 * (unitree_sdk2/include/unitree/dds_wrapper/common/crc.h,
 * crc32_core()) — reproduit à l'identique ici plutôt que réinventé :
 * un CRC divergent, même correct algorithmiquement en soi, serait
 * silencieusement rejeté par le firmware du vrai robot (voir la
 * docstring de dds_publish_damping ci-dessous).
 *
 * Copyright (c) 2016-2023 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics")
 * Licence BSD 3-Clause : voir LICENSE_UNITREE_SDK2.txt dans ce dossier.
 */
static uint32_t crc32_core(const uint32_t *ptr, uint32_t len) {
    uint32_t xbit = 0, data = 0, CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++) {
        xbit = 1u << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++) {
            if (CRC32 & 0x80000000) {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            } else {
                CRC32 <<= 1;
            }
            if (data & xbit) {
                CRC32 ^= dwPolynomial;
            }
            xbit >>= 1;
        }
    }
    return CRC32;
}

/*
 * TODO (intégration matérielle) : brancher sur la vraie CycloneDDS C
 * library — publier UN message LowCmd_ avec, pour chacun des 35
 * MotorCmd_ :
 *   mode = 1 (Enable, jamais 0 — voir la justification identique dans
 *             G1DDSActuator.emergency_stop() côté Python : éviter une
 *             roue libre incontrôlée sur un membre en charge)
 *   q = 0.0, dq = 0.0, tau = 0.0
 *   kp = 0.0
 *   kd = damping_kd (voir DEFAULT_DAMPING_KD — non nul, contrairement
 *        à un e-stop classique : un amortissement doux plutôt qu'un
 *        blocage brutal, cohérent avec l'intention "se relâche
 *        doucement" de ce composant précis, différent d'un e-stop)
 * CRC calculé exactement comme G1DDSHub.publish() le fait côté Python
 * (unitree_sdk2py.utils.crc.CRC) — indispensable, un message sans CRC
 * valide est silencieusement ignoré par le firmware du robot.
 *
 * IMPLÉMENTÉ le 2026-09-15 — voir dds_subscribe_lowcmd() pour le
 * contexte complet sur la reconstruction du schéma IDL. Écrit un
 * publisher DDS sur "rt/lowcmd" avec les 35 moteurs en mode
 * amortissement, CRC calculé avec l'algorithme officiel exact
 * (crc32_core ci-dessus, copié depuis unitree_sdk2/include/unitree/
 * dds_wrapper/common/crc.h). ⚠️ Jamais testé contre un vrai bus DDS
 * ni un vrai G1 — voir la limite documentée dans README.md.
 */
static dds_entity_t g_writer = 0;

static int dds_publish_damping(double kd) {
    if (g_writer == 0) {
        dds_entity_t topic = dds_create_topic(
            g_participant, &unitree_hg_msg_dds__LowCmd__desc, "rt/lowcmd", NULL, NULL);
        if (topic < 0) {
            fprintf(stderr, "[freshness_monitor] dds_create_topic (writer) a échoué : %s\n",
                    dds_strretcode(-topic));
            return -1;
        }
        g_writer = dds_create_writer(g_participant, topic, NULL, NULL);
        if (g_writer < 0) {
            fprintf(stderr, "[freshness_monitor] dds_create_writer a échoué : %s\n",
                    dds_strretcode(-g_writer));
            g_writer = 0;
            return -1;
        }
    }

    unitree_hg_msg_dds__LowCmd_ cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.mode_pr = 0;
    /* mode_machine : lu depuis le premier LowState_ reçu dans une
     * vraie intégration (voir G1DDSHub._on_low_state() côté Python,
     * même principe — ne jamais le deviner). Laissé à 0 ici tant que
     * ce démon ne lit pas encore rt/lowstate (portée volontairement
     * limitée à rt/lowcmd pour ce composant, voir docstring de
     * module) — à corriger avant tout usage réel. */
    cmd.mode_machine = 0;

    for (int i = 0; i < NUM_MOTORS; i++) {
        cmd.motor_cmd[i].mode = 1;  /* Enable, jamais 0 — voir docstring ci-dessus */
        cmd.motor_cmd[i].q = 0.0f;
        cmd.motor_cmd[i].dq = 0.0f;
        cmd.motor_cmd[i].tau = 0.0f;
        cmd.motor_cmd[i].kp = 0.0f;
        cmd.motor_cmd[i].kd = (float)kd;
        cmd.motor_cmd[i].reserve = 0;
    }

    cmd.crc = crc32_core((uint32_t *)&cmd, (sizeof(cmd) >> 2) - 1);

    dds_return_t rc = dds_write(g_writer, &cmd);
    if (rc != DDS_RETCODE_OK) {
        fprintf(stderr, "[freshness_monitor] dds_write a échoué : %s\n",
                dds_strretcode(-rc));
        return -1;
    }
    return 0;
}

/*
 * Logique de détection de fraîcheur — testée indépendamment de toute
 * vraie communication DDS (voir test_freshness_monitor.c), car c'est
 * elle qui contient le vrai risque de bug (erreur de comparaison de
 * seuil, oubli de réarmement après une publication d'amortissement,
 * etc.) — pas l'appel réseau lui-même.
 *
 * Retourne true si un amortissement doit être publié MAINTENANT (le
 * dernier message vu date de plus de timeout_ms, ET le robot est
 * censé être actif) — jamais true si robot_active est false (arrêt
 * volontaire du process principal, rien à amortir).
 */
static bool should_publish_damping(const monitor_state_t *state, long now_ms) {
    if (!state->robot_active) {
        return false;
    }
    long age_ms = now_ms - state->last_message_ms;
    return age_ms > state->timeout_ms;
}

static long env_long_or_default(const char *name, long default_value) {
    const char *val = getenv(name);
    if (val == NULL || val[0] == '\0') {
        return default_value;
    }
    return atol(val);
}

int main(void) {
    monitor_state_t state;
    state.timeout_ms = env_long_or_default(
        "AXIS_OS_FRESHNESS_TIMEOUT_MS", DEFAULT_FRESHNESS_TIMEOUT_MS);
    state.damping_kd = (double)env_long_or_default(
        "AXIS_OS_DAMPING_KD_X100", (long)(DEFAULT_DAMPING_KD * 100)) / 100.0;
    state.poll_interval_ms = env_long_or_default(
        "AXIS_OS_FRESHNESS_POLL_MS", DEFAULT_POLL_INTERVAL_MS);
    state.robot_active = false;
    state.last_message_ms = monotonic_ms();

    fprintf(stderr,
        "[freshness_monitor] Démarrage — timeout=%ldms kd=%.2f poll=%ldms\n",
        state.timeout_ms, state.damping_kd, state.poll_interval_ms);

    if (dds_subscribe_lowcmd(&state) != 0) {
        fprintf(stderr, "[freshness_monitor] Échec abonnement DDS — arrêt.\n");
        return 1;
    }

    bool damping_active = false;
    while (1) {
        dds_poll_lowcmd(&state);
        long now = monotonic_ms();
        if (should_publish_damping(&state, now)) {
            if (!damping_active) {
                fprintf(stderr,
                    "[freshness_monitor] ⚠️  Aucun message depuis %ldms "
                    "(seuil=%ldms) — publication de l'amortissement.\n",
                    now - state.last_message_ms, state.timeout_ms);
                dds_publish_damping(state.damping_kd);
                damping_active = true;
            }
        } else {
            damping_active = false;
        }
        usleep((useconds_t)(state.poll_interval_ms * 1000));
    }

    return 0;
}
