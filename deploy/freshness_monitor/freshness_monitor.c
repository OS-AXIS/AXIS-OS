/*
 * AXIS-OS — Moniteur de fraîcheur
 * ======================================================================
 * Pourquoi ce démon existe : si le processus principal d'AXIS-OS meurt
 * en plein mouvement, les moteurs du robot gardent la dernière consigne
 * reçue, qui peut être dangereuse. Ce moniteur s'en aperçoit et relâche
 * le robot en douceur. C'est la première des cinq recommandations de
 * renforcement formulées après le second audit externe du code
 * (2026-09-15, voir CHANGELOG.md).
 *
 * Pourquoi en C, dans un démon séparé : il doit survivre à la mort de
 * l'interpréteur Python lui-même (blocage, plantage, ou « OOM killer »,
 * le mécanisme du noyau Linux qui tue le plus gros processus quand la
 * mémoire manque), donc ne partager aucun mode de défaillance avec lui.
 * Un moniteur écrit en Python, même dans un processus séparé, resterait
 * exposé à un bug de l'interpréteur (fuite mémoire du ramasse-miettes,
 * blocage du GIL, le verrou global de Python, dans une bibliothèque
 * tierce partagée). C'est improbable, mais c'est précisément la classe
 * de risque que ce composant existe pour éliminer.
 *
 * Ce qu'il fait : il écoute le canal DDS « rt/lowcmd » (DDS, Data
 * Distribution Service, est le système de messages du Unitree G1 ; ce
 * canal est celui où G1DDSHub publie les commandes des moteurs, voir
 * axis_os/hal/drivers/unitree_g1/g1_dds_actuator.py). Si aucun message
 * n'est vu pendant FRESHNESS_TIMEOUT_MS alors que le robot est censé
 * être actif, il publie lui-même une commande d'amortissement sur les
 * 35 moteurs : kp = 0 (aucun rappel vers une position), kd = DAMPING_KD
 * (un amortissement faible, qui freine le mouvement), tau = 0 (aucun
 * couple), mode = 1 (moteur activé).
 * Le robot se relâche doucement au lieu de rester bloqué sur la
 * dernière consigne reçue.
 *
 * Complémentaire du chien de garde systemd (voir
 * deploy/systemd/axis-os.service) : celui-ci redémarre le processus
 * mort, mais laisse passer jusqu'à WatchdogSec = 2 s avant de le
 * détecter, puis le temps du redémarrage lui-même. Pendant tout cet
 * intervalle, le robot reste sur la dernière consigne DDS reçue si rien
 * d'autre n'intervient. Ce moniteur agit beaucoup plus vite
 * (FRESHNESS_TIMEOUT_MS, 200 ms par défaut), directement sur le bus
 * DDS, sans dépendre d'aucune détection système.
 *
 * État (2026-09-15) : l'abonnement et la publication DDS sont écrits
 * pour CycloneDDS en C (voir dds_subscribe_lowcmd() et
 * dds_publish_damping()), avec le schéma des messages reconstruit dans
 * idl/. Le code compile, mais il n'a jamais été essayé sur un vrai bus
 * DDS ni sur un vrai G1 (voir la limite décrite dans README.md). La
 * logique de délai, elle, est testée (should_publish_damping(), voir
 * test_freshness_monitor.c). Deux points restent à régler avant tout
 * usage réel, détaillés plus bas : mode_machine laissé à 0, et un arrêt
 * volontaire du processus principal pas encore distingué d'une panne.
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

/* Valeurs par défaut, réglables sans recompiler par des variables
 * d'environnement (voir main() et README.md). */
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

/* Horodatage monotone en millisecondes. Jamais l'horloge murale
 * (CLOCK_REALTIME) : elle peut sauter en arrière (mise à l'heure par
 * NTP, changement d'heure) et fausserait le calcul de fraîcheur. */
static long monotonic_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long)ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

/*
 * Abonnement au canal « rt/lowcmd » (type
 * unitree_hg::msg::dds_::LowCmd_, le même schéma que G1DDSHub côté
 * Python). À chaque message reçu, dds_poll_lowcmd() note l'heure
 * (state->last_message_ms) et marque le robot actif
 * (state->robot_active = true).
 *
 * Limite à lever avant tout usage réel : robot_active ne repasse
 * jamais à false. Il le devrait quand le processus principal se ferme
 * proprement (message explicite, ou canal fermé côté émetteur), pour ne
 * jamais déclencher l'amortissement sur un arrêt volontaire. Le moyen
 * reste à définir avec l'intégration sur le vrai robot.
 *
 * Origine du schéma (2026-09-15) : Unitree ne publie pas de fichier
 * .idl source pour unitree_hg (IDL, Interface Definition Language,
 * décrit la structure des messages DDS) ; son SDK C++ (unitree_sdk2)
 * ne distribue que le code déjà généré. Le schéma LowCmd_/MotorCmd_ a
 * donc été reconstruit dans idl/ à partir de deux générations
 * officielles indépendantes, celle du SDK Python (unitree_sdk2_python)
 * et celle du SDK C++, vérifiées identiques (mêmes champs, mêmes types,
 * même ordre). Il a été compilé avec l'outil idlc de CycloneDDS 0.10.x
 * (voir README.md pour le reproduire). Seule la compilation est
 * vérifiée : ce code n'a jamais été essayé sur un vrai bus DDS ni sur
 * un vrai G1 (voir la limite décrite dans README.md).
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

/* Appelée à chaque cycle de la boucle principale (voir main()). Elle
 * interroge le canal sans attendre (lecture non bloquante, au rythme
 * de poll_interval_ms) plutôt que de recevoir un rappel DDS
 * asynchrone : toute la logique de délai reste dans le seul fil
 * d'exécution principal. C'est plus simple à raisonner et à tester que
 * d'ajouter de la concurrence, pour un composant dont la fiabilité est
 * précisément l'enjeu. */
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
 * CRC32 (somme de contrôle de chaque message moteur) : l'algorithme
 * exact du SDK officiel d'Unitree
 * (unitree_sdk2/include/unitree/dds_wrapper/common/crc.h,
 * crc32_core()), recopié à l'identique plutôt que réinventé. Un CRC
 * différent, même juste en soi, serait rejeté sans bruit par le
 * firmware du vrai robot (voir le commentaire de dds_publish_damping()
 * ci-dessous).
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
 * Publie un message LowCmd_ sur « rt/lowcmd » qui met les 35 moteurs
 * en amortissement. Pour chaque MotorCmd_ :
 *   mode = 1 (moteur activé, jamais 0 : un moteur désactivé laisse le
 *             membre en roue libre, incontrôlé s'il porte une charge ;
 *             même raison que dans G1DDSActuator.emergency_stop() côté
 *             Python)
 *   q = 0.0, dq = 0.0, tau = 0.0 (consignes de position, de vitesse et
 *             de couple)
 *   kp = 0.0 (aucun rappel vers une position)
 *   kd = damping_kd (voir DEFAULT_DAMPING_KD). Non nul, contrairement
 *        à un arrêt d'urgence classique : on veut un amortissement doux
 *        plutôt qu'un blocage brutal, puisque ce composant doit laisser
 *        le robot « se relâcher doucement ».
 * Le CRC est calculé comme G1DDSHub.publish() le fait côté Python
 * (unitree_sdk2py.utils.crc.CRC), avec crc32_core() ci-dessus. C'est
 * indispensable : un message sans CRC valide est ignoré sans bruit par
 * le firmware du robot.
 *
 * Écrit le 2026-09-15 ; l'origine du schéma est expliquée au-dessus de
 * dds_subscribe_lowcmd(). Jamais essayé sur un vrai bus DDS ni sur un
 * vrai G1 : voir la limite décrite dans README.md.
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
    /* mode_machine (champ de l'état du robot, recopié dans chaque
     * commande) : sur le vrai robot, il doit être lu dans le premier
     * message LowState_ reçu, jamais deviné (même principe que
     * G1DDSHub._on_low_state() côté Python). Il est laissé à 0 tant que
     * ce démon ne lit pas « rt/lowstate » : sa portée se limite
     * volontairement à « rt/lowcmd ». À corriger avant tout usage
     * réel. */
    cmd.mode_machine = 0;

    for (int i = 0; i < NUM_MOTORS; i++) {
        cmd.motor_cmd[i].mode = 1;  /* activé, jamais 0 : voir plus haut */
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
 * Logique de détection de fraîcheur. Elle est testée sans aucune
 * communication DDS (voir test_freshness_monitor.c), parce que c'est
 * elle qui porte le vrai risque de bug (erreur de comparaison au seuil,
 * oubli de réarmement après un amortissement…), pas l'appel réseau.
 *
 * Retourne true si un amortissement doit être publié maintenant : le
 * dernier message vu date de plus de timeout_ms et le robot est censé
 * être actif. Jamais true si robot_active vaut false (arrêt volontaire
 * du processus principal : rien à amortir).
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
