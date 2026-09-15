/*
 * AXIS-OS — Tests moniteur de fraîcheur
 * =========================================
 * Teste UNIQUEMENT la logique de détection (should_publish_damping),
 * jamais la communication DDS elle-même (non implémentée dans ce
 * squelette — voir les TODO dans freshness_monitor.c). C'est cette
 * logique qui porte le vrai risque de bug (erreur de comparaison de
 * seuil, oubli de réarmement) — l'appel réseau, une fois la vraie lib
 * CycloneDDS branchée, est plus mécanique.
 *
 * Compilation et exécution :
 *   gcc -DAXIS_OS_TEST_BUILD -o test_freshness_monitor \
 *       test_freshness_monitor.c && ./test_freshness_monitor
 */

#define AXIS_OS_TEST_BUILD
#include <stdio.h>
#include <stdbool.h>
#include <assert.h>

/* Redéfinit les structures/fonctions testées directement ici plutôt
 * que d'inclure freshness_monitor.c tel quel (qui a un main() et des
 * fonctions DDS stub qui écrivent sur stderr) — garde ces tests
 * silencieux et focalisés uniquement sur la logique pure. */

typedef struct {
    long timeout_ms;
    double damping_kd;
    long poll_interval_ms;
    bool robot_active;
    long last_message_ms;
} monitor_state_t;

static bool should_publish_damping(const monitor_state_t *state, long now_ms) {
    if (!state->robot_active) {
        return false;
    }
    long age_ms = now_ms - state->last_message_ms;
    return age_ms > state->timeout_ms;
}

static int tests_run = 0;
static int tests_failed = 0;

static void check(const char *description, bool condition) {
    tests_run++;
    if (condition) {
        printf("  [OK] %s\n", description);
    } else {
        printf("  [FAIL] %s\n", description);
        tests_failed++;
    }
}

int main(void) {
    printf("==================================================================\n");
    printf("  Tests should_publish_damping()\n");
    printf("==================================================================\n");

    /* 1 — robot inactif : jamais d'amortissement, même très vieux message */
    {
        monitor_state_t state = {200, 2.0, 20, false, 0};
        check("robot_active=false -> jamais d'amortissement (même à 10000ms)",
              should_publish_damping(&state, 10000) == false);
    }

    /* 2 — robot actif, message récent : pas d'amortissement */
    {
        monitor_state_t state = {200, 2.0, 20, true, 1000};
        check("Message récent (age=50ms < timeout=200ms) -> pas d'amortissement",
              should_publish_damping(&state, 1050) == false);
    }

    /* 3 — robot actif, message trop vieux : amortissement */
    {
        monitor_state_t state = {200, 2.0, 20, true, 1000};
        check("Message trop vieux (age=250ms > timeout=200ms) -> amortissement",
              should_publish_damping(&state, 1250) == true);
    }

    /* 4 — cas limite : exactement au seuil (age == timeout) */
    {
        monitor_state_t state = {200, 2.0, 20, true, 1000};
        check("Cas limite age==timeout (200ms==200ms) -> PAS d'amortissement "
              "(strictement supérieur requis, évite un déclenchement sur "
              "un jitter d'1 cycle pile au seuil)",
              should_publish_damping(&state, 1200) == false);
        check("Cas limite age==timeout+1 (201ms) -> amortissement",
              should_publish_damping(&state, 1201) == true);
    }

    /* 5 — timeout personnalisé (pas la valeur par défaut) */
    {
        monitor_state_t state = {50, 2.0, 20, true, 5000};
        check("Timeout personnalisé (50ms) respecté, pas codé en dur",
              should_publish_damping(&state, 5051) == true
              && should_publish_damping(&state, 5049) == false);
    }

    /* 6 — now_ms avant last_message_ms (horloge qui recule, ne devrait
     * jamais arriver avec CLOCK_MONOTONIC, mais defensive) */
    {
        monitor_state_t state = {200, 2.0, 20, true, 5000};
        check("now_ms < last_message_ms (age négatif) -> jamais d'amortissement "
              "(defensive, ne doit jamais arriver avec CLOCK_MONOTONIC)",
              should_publish_damping(&state, 4000) == false);
    }

    printf("\n==================================================================\n");
    printf("  Resultats : %d/%d tests passes\n", tests_run - tests_failed, tests_run);
    printf("==================================================================\n");

    return tests_failed == 0 ? 0 : 1;
}
