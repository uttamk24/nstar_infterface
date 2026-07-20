/**
 * @file  ttc_nstar.c
 * @brief N-STAR TTC transponder interface — application entry point.
 *
 * Follows the core module pattern:
 *   - pubSubMessageProcessor() handles all incoming ZeroMQ topics
 *   - main() runs startup then drives the pub-sub event loop
 *
 * Callbacks from the NSTAR core (frame received, TX complete, fault,
 * lock acquired/lost) are defined here and registered at NSTAR_Init().
 */

#include "ttc_nstar.h"
#include <stdio.h>
#include <string.h>

/* =========================================================================
 * BEGIN: NSTAR Core Callbacks
 * =========================================================================
 * Called from internal NSTAR threads — do NOT call NSTAR_* functions
 * from inside these callbacks.
 */

static void onFrameReceived(const uint8_t *buf, size_t len)
{
    (void)buf;
    TTC_NSTAR_Debug_L1_D("/ => Frame received (%zu bytes)\n", len);
}

static void onTXComplete(size_t bytesSent)
{
    TTC_NSTAR_Debug_L1_D("/ => TX complete (%zu bytes)\n", bytesSent);
}

static void onFault(NSTAR_FaultSource_t source)
{
    TTC_NSTAR_Debug_L1_D("X => FAULT source=%d\n", (int)source);
}

static void onLockAcquired(void)
{
    TTC_NSTAR_Debug_L1("/ => RX lock acquired\n");
}

static void onLockLost(void)
{
    TTC_NSTAR_Debug_L1("X => RX lock lost\n");
}

/* =========================================================================
 * END: NSTAR Core Callbacks
 * =========================================================================
 */

/* =========================================================================
 * BEGIN: Pub-Sub Message Processor
 * =========================================================================
 */


/* Telecommand handler — stub until TC function IDs are defined */
void TELECOMMAND_HANDLER_TTC_NSTAR(const uint8_t *message, size_t message_size)
{
    (void)message;
    TTC_NSTAR_Debug_L1_D("/ => TC received (%zu bytes) — handler not yet implemented\n", message_size);
}

void pubSubMessageProcessor(const char *topic, const uint8_t *message,
                             size_t message_size,
                             size_t topic_size __attribute__((unused)))
{
    /* Telecommands from Command Ingest */
    if (strcmp(topic, "CI_TTC_NSTAR_PUT:TC") == 0)
    {
        TTC_NSTAR_Debug_L1("CI_TTC_NSTAR_PUT:TC\n");
        TELECOMMAND_HANDLER_TTC_NSTAR(message, message_size);
    }

    /* Scheduler heartbeat — update shared memory health snapshot */
    else if (strcmp(topic, "SCH_APP_PING:OBC_TM_IC") == 0)
    {
        TTC_NSTAR_Debug_L1("SCH_APP_PING:OBC_TM_IC\n");
        /* TODO: writeTMtoSHM() */
        (void)message;
    }

    /* Mission mode broadcast */
    else if (strcmp(topic, "MMP_APPS_PUT:BROADCAST_MC") == 0)
    {
        TTC_NSTAR_Debug_L1("MMP_APPS_PUT:BROADCAST_MC\n");
        /* TODO: update ttcNSTARMode */
        (void)message;
    }

    /* Startup sync */
    else if (strcmp(topic, "APPS_APPS:BROADCAST") == 0)
    {
        TTC_NSTAR_Debug_L1("APPS_APPS:BROADCAST\n");
    }
}

/* =========================================================================
 * END: Pub-Sub Message Processor
 * =========================================================================
 */

/* =========================================================================
 * BEGIN: Main
 * =========================================================================
 */

int main(int argc __attribute__((unused)), char *argv[] __attribute__((unused)))
{
    printf("\n\n TTC_NSTAR Will Start\n");
    TTC_NSTAR_Debug_L1("  TTC_NSTAR_DEBUG_L1 => Activated\n");
    TTC_NSTAR_Debug_L2("  TTC_NSTAR_DEBUG_L2 => Activated\n");

    /* Hardware configuration — fd values assigned once real HAL is available */
    NSTAR_Config_t cfg = {
        .uartFd          = -1,   /* TODO: open("/dev/ttyS1", O_RDWR|O_NOCTTY) */
        .gpioLockDetect  = -1,   /* TODO: GPIO fd for LOCK_DETECT              */
        .gpioDataValid   = -1,   /* TODO: GPIO fd for DATA_VALID               */
        .gpioFaultN      = -1,   /* TODO: GPIO fd for FAULT_N                  */
        .gpioResetN      = -1,   /* TODO: GPIO fd for RESET_N                  */
        .dataFd          = -1,   /* TODO: open point — data interface TBD      */
    };

    NSTAR_Callbacks_t cbs = {
        .onFrameReceived = onFrameReceived,
        .onTXComplete    = onTXComplete,
        .onFault         = onFault,
        .onLockAcquired  = onLockAcquired,
        .onLockLost      = onLockLost,
    };

    NSTAR_Ctx_t *ctx = NULL;
    NSTAR_Result_t rc = NSTAR_Init(&cfg, &cbs, &ctx);
    if (rc != NSTAR_OK)
    {
        printf("X => NSTAR_Init failed: %d\n", rc);
        return 1;
    }

    rc = NSTAR_StartupSequence(ctx);
    if (rc != NSTAR_OK)
    {
        printf("X => NSTAR_StartupSequence failed: %d\n", rc);
        NSTAR_Deinit(ctx);
        return 1;
    }

    TTC_NSTAR_Debug_L1("/ => NSTAR module READY\n");

    while (1)
    {
        /* TODO: replace with receive_zmq_pubsub_message() + pubSubMessageProcessor()
         *       once ZeroMQ pub-sub config is wired up.
         *       For standalone HAL testing, insert test sequences here. */
        nstarSleepMS(1000);
    }

    NSTAR_Deinit(ctx);
    return 0;
}

/* =========================================================================
 * END: Main
 * =========================================================================
 */
