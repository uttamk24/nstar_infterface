/**
 * @file  nstar_app.c
 * @brief Application entry point — Stage 1 skeleton.
 */
#include "nstar.h"
#include <stdio.h>

static void onFrameReceived(const uint8_t *buf, size_t len)
{
    (void)buf; (void)len;
    printf("app: frame received (%zu bytes)\n", len);
}

static void onTXComplete(size_t bytesSent)
{
    printf("app: TX complete (%zu bytes)\n", bytesSent);
}

static void onFault(nstarFaultSource_t source)
{
    printf("app: FAULT source=%d\n", (int)source);
}

static void onLockAcquired(void) { printf("app: lock acquired\n"); }
static void onLockLost(void)     { printf("app: lock lost\n");     }

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
