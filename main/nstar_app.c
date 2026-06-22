/**
 * @file  nstar_app.c
 * @brief Application entry point — Stage 1 skeleton.
 */
#include "nstar.h"
#include <stdio.h>

static void on_frame_received(const uint8_t *buf, size_t len)
{
    (void)buf; (void)len;
    printf("app: frame received (%zu bytes)\n", len);
}

static void on_tx_complete(size_t bytes_sent)
{
    printf("app: TX complete (%zu bytes)\n", bytes_sent);
}

static void on_fault(nstar_fault_source_t source)
{
    printf("app: FAULT source=%d\n", (int)source);
}

static void on_lock_acquired(void) { printf("app: lock acquired\n"); }
static void on_lock_lost(void)     { printf("app: lock lost\n");     }

int main(void)
{
    printf("N-STAR interface module — Stage 1 build OK\n");
    printf("Frame size: %u bytes\n", NSTAR_FRAME_SIZE_BYTES);
#ifdef NSTAR_CRC_ENABLED
    printf("CRC: ENABLED\n");
#else
    printf("CRC: DISABLED\n");
#endif
    return 0;
}
