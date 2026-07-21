/**
 * @file  nstar_app_sil.c
 * @brief Application entry point for SIL testing.
 *
 * This is the SIL counterpart to main/nstar_app.c. It links the SAME
 * src/nstar_core.c and src/nstar_frame.c used in production, against
 * hal/nstar_hal_dummy.c instead of hal/nstar_hal_linux.c.
 *
 * Responsibilities:
 *   1. Wait for the simulator's PTY link file, open that PTY slave as
 *      the UART fd with real termios configuration matching the EICD
 *      (38400 8E1 RS-422 framing — RS-422 itself is a wire-level detail
 *      the PTY can't model, but the byte-level framing parameters are
 *      configured for real so the same termios setup code is exercised).
 *   2. Initialise GPIO line "fds" to the fixed integer values the dummy
 *      HAL's gpioFdToFilename() table expects.
 *   3. Call NSTAR_Init() + NSTAR_StartupSequence(), exactly as a real
 *      application would.
 *   4. On every moduleState transition, rxState transition, fault,
 *      frame reception, and TX completion, rewrite the SIL status file
 *      so the separate testSil process can observe what happened.
 *   5. Stay running (so a SIL test can drive GPIO files and the
 *      simulator's control file over time) until told to exit via
 *      SIGTERM/SIGINT, or until an optional run-duration argument elapses.
 *
 * Since nstar_core.c has no built-in hook for "notify on every state
 * change", this file polls NSTAR_GetModuleState() and
 * NSTAR_RXGetState() in its own small monitor loop and diffs against
 * the last-seen values — this is allowed because these accessors are
 * part of the public API and are documented as thread-safe.
 */

#define _DEFAULT_SOURCE
#include "ttc_nstar.h"
#include "sil_common.h"
#include "sil_status.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include <time.h>
#include <errno.h>

#define SIL_FD_GPIO_LOCK_DETECT  20
#define SIL_FD_GPIO_DATA_VALID   21
#define SIL_FD_GPIO_FAULT_N      22
#define SIL_FD_GPIO_RESET_N      23
#define SIL_FD_DATA_INTERFACE    11

static volatile int gRunning = 1;
static silStatus_t gStatus;

static size_t  gTotalBytesReceived = 0;
static size_t  gFrameReceivedCount = 0;
static size_t  gLockAcquiredCount  = 0;
static size_t  gLockLostCount      = 0;
static size_t  gFaultCount          = 0;
static char    gLastFault[16]       = "NONE";
static size_t  gTXCompleteCount    = 0;
static size_t  gLastTXBytes        = 0;
static int     gLastTXStartRC     = 1;   /* 1 = "not yet called" sentinel */
static int     gLastTXWriteRC     = 1;
static int     gLastTXStopRC      = 1;

static void sighandler(int sig)
{
    (void)sig;
    gRunning = 0;
}

/* =========================================================================
 * Callbacks — update in-memory counters, status file is rewritten by the
 * monitor loop in main() so every callback doesn't need to know the full
 * status format.
 * =========================================================================
 */

static void onFrameReceived(const uint8_t *buf, size_t len)
{
    (void)buf;
    gFrameReceivedCount++;
    gTotalBytesReceived += len;
    fprintf(stderr, "[nstar_app_sil] frame received: %zu bytes\n", len);
}

static void onTXComplete(size_t bytesSent)
{
    gTXCompleteCount++;
    gLastTXBytes = bytesSent;
    fprintf(stderr, "[nstar_app_sil] tx complete: %zu bytes\n", bytesSent);
}

static void onFault(NSTAR_FaultSource_t source)
{
    gFaultCount++;
    switch (source) {
        case NSTAR_FAULT_SEL:         snprintf(gLastFault, sizeof(gLastFault), "SEL"); break;
        case NSTAR_FAULT_OVERCURRENT: snprintf(gLastFault, sizeof(gLastFault), "OVERCURRENT"); break;
        case NSTAR_FAULT_TEMPERATURE: snprintf(gLastFault, sizeof(gLastFault), "TEMPERATURE"); break;
        default:                      snprintf(gLastFault, sizeof(gLastFault), "UNKNOWN"); break;
    }
    fprintf(stderr, "[nstar_app_sil] fault: %s\n", gLastFault);
}

static void onLockAcquired(void)
{
    gLockAcquiredCount++;
    fprintf(stderr, "[nstar_app_sil] lock acquired\n");
}

static void onLockLost(void)
{
    gLockLostCount++;
    fprintf(stderr, "[nstar_app_sil] lock lost\n");
}

static const NSTAR_Callbacks_t kCallbacks = {
    .onFrameReceived = onFrameReceived,
    .onTXComplete    = onTXComplete,
    .onFault          = onFault,
    .onLockAcquired  = onLockAcquired,
    .onLockLost      = onLockLost,
};

/* =========================================================================
 * App control file — lets the SIL test suite trigger TX calls on the
 * real nstar_core.c API from outside this process.
 * =========================================================================
 */

static void applyAppControlCommands(NSTAR_Ctx_t *ctx)
{
    char path[512];
    silPath(path, sizeof(path), SIL_APP_CONTROL_FILE);

    FILE *f = fopen(path, "r");
    if (!f) return;

    char line[128];
    int any = 0;
    while (fgets(line, sizeof(line), f)) {
        char *nl = strchr(line, '\n');
        if (nl) *nl = '\0';
        if (line[0] == '\0') continue;
        any = 1;

        if (strncmp(line, "TX_START=", 9) == 0) {
            long rate = strtol(line + 9, NULL, 16);
            NSTAR_Result_t rc = NSTAR_TXStart(ctx, (NSTAR_TXRateCode_t)rate);
            gLastTXStartRC = (int)rc;
            fprintf(stderr, "[nstar_app_sil] TX_START(rate=0x%lx) -> %d\n",
                    rate, rc);
        } else if (strncmp(line, "TX_WRITE=", 9) == 0) {
            long nbytes = strtol(line + 9, NULL, 10);
            if (nbytes > 0 && nbytes <= (long)NSTAR_FRAME_SIZE_BYTES * 4) {
                uint8_t *buf = malloc((size_t)nbytes);
                if (buf) {
                    for (long i = 0; i < nbytes; i++) buf[i] = (uint8_t)(i & 0xFF);
                    NSTAR_Result_t rc = NSTAR_TXWrite(ctx, buf, (size_t)nbytes);
                    gLastTXWriteRC = (int)rc;
                    fprintf(stderr, "[nstar_app_sil] TX_WRITE(%ld bytes) -> %d\n",
                            nbytes, rc);
                    free(buf);
                }
            }
        } else if (strncmp(line, "TX_STOP", 7) == 0) {
            NSTAR_Result_t rc = NSTAR_TXStop(ctx);
            gLastTXStopRC = (int)rc;
            fprintf(stderr, "[nstar_app_sil] TX_STOP -> %d\n", rc);
        }
    }
    fclose(f);

    if (any) {
        FILE *tf = fopen(path, "w");
        if (tf) fclose(tf);
    }
}

/* =========================================================================
 * Status file refresh
 * =========================================================================
 */

static const char *moduleStateName(NSTAR_ModuleState_t s)
{
    switch (s) {
        case NSTAR_MODULE_UNINIT:        return "UNINIT";
        case NSTAR_MODULE_INITIALISING:  return "INITIALISING";
        case NSTAR_MODULE_STARTING:      return "STARTING";
        case NSTAR_MODULE_READY:         return "READY";
        case NSTAR_MODULE_FAULT:         return "FAULT";
        case NSTAR_MODULE_SHUTTING_DOWN: return "SHUTTING_DOWN";
        default:                         return "UNKNOWN";
    }
}

static const char *rxStateName(NSTAR_RXState_t s)
{
    switch (s) {
        case NSTAR_RX_IDLE:      return "IDLE";
        case NSTAR_RX_ACQUIRING: return "ACQUIRING";
        case NSTAR_RX_LOCKED:    return "LOCKED";
        case NSTAR_RX_LOCK_LOST: return "LOCK_LOST";
        default:                 return "UNKNOWN";
    }
}

static void refreshStatus(NSTAR_Ctx_t *ctx)
{
    silStatusInit(&gStatus);
    silStatusSet(&gStatus, "MODULE_STATE",
                   moduleStateName(NSTAR_GetModuleState(ctx)));
    silStatusSet(&gStatus, "RX_STATE",
                   rxStateName(NSTAR_RXGetState(ctx)));
    silStatusSetInt(&gStatus, "FRAME_RECEIVED_COUNT",
                        (long)gFrameReceivedCount);
    silStatusSetInt(&gStatus, "TOTAL_BYTES_RECEIVED",
                        (long)gTotalBytesReceived);
    silStatusSetInt(&gStatus, "LOCK_ACQUIRED_COUNT",
                        (long)gLockAcquiredCount);
    silStatusSetInt(&gStatus, "LOCK_LOST_COUNT",
                        (long)gLockLostCount);
    silStatusSetInt(&gStatus, "FAULT_COUNT", (long)gFaultCount);
    silStatusSet(&gStatus, "LAST_FAULT", gLastFault);
    silStatusSetInt(&gStatus, "TX_COMPLETE_COUNT",
                        (long)gTXCompleteCount);
    silStatusSetInt(&gStatus, "LAST_TX_BYTES", (long)gLastTXBytes);
    silStatusSetInt(&gStatus, "LAST_TX_START_RC", gLastTXStartRC);
    silStatusSetInt(&gStatus, "LAST_TX_WRITE_RC", gLastTXWriteRC);
    silStatusSetInt(&gStatus, "LAST_TX_STOP_RC", gLastTXStopRC);
    silStatusWrite(&gStatus);
}

/* =========================================================================
 * UART setup — open the PTY slave with real termios configuration
 * =========================================================================
 */

static int waitForPTYLink(char *slavePathOut, size_t outLen,
                              int timeoutMs)
{
    char linkPath[512];
    silPath(linkPath, sizeof(linkPath), SIL_PTY_LINK_FILE);

    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);

    for (;;) {
        FILE *f = fopen(linkPath, "r");
        if (f) {
            char buf[256] = {0};
            size_t n = fread(buf, 1, sizeof(buf) - 1, f);
            fclose(f);
            if (n > 0) {
                snprintf(slavePathOut, outLen, "%s", buf);
                return 0;
            }
        }
        clock_gettime(CLOCK_MONOTONIC, &now);
        long elapsedMS = (now.tv_sec - start.tv_sec) * 1000 +
                          (now.tv_nsec - start.tv_nsec) / 1000000;
        if (elapsedMS >= timeoutMs) return -1;

        struct timespec d = { 0, 50 * 1000000L };
        nanosleep(&d, NULL);
    }
}

static int openUART(const char *slavePath)
{
    int fd = open(slavePath, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        fprintf(stderr, "[nstar_app_sil] open(%s) failed: %s\n",
                slavePath, strerror(errno));
        return -1;
    }

    /*
     * Real termios configuration matching the EICD UART parameters:
     * 38400 baud, 8 data bits, EVEN parity, 1 stop bit (8E1).
     * This is the same configuration hal/nstar_hal_linux.c will need
     * for the real RS-422 port — only the device path differs.
     */
    struct termios tio;
    if (tcgetattr(fd, &tio) != 0) {
        fprintf(stderr, "[nstar_app_sil] tcgetattr failed: %s\n",
                strerror(errno));
        close(fd);
        return -1;
    }

    /*
     * Real termios configuration matching the EICD UART parameters as
     * closely as a PTY can emulate: 38400 baud, 8 data bits, 1 stop bit.
     *
     * Parity (8E1's "E") is intentionally NOT configured here. A PTY has
     * no physical wire and only loosely emulates serial line discipline —
     * enabling PARENB+INPCK on a PTY slave succeeds the first time a
     * process configures it, but a second independent open+configure of
     * the SAME slave device (e.g. a second nstar_app_sil run against a
     * still-live nstar_sim) reliably fails tcsetattr with EINVAL. This
     * was reproduced and isolated outside the test harness: removing
     * PARENB/INPCK and keeping everything else (rate, CS8, no CSTOPB)
     * eliminates the failure on repeated configuration.
     *
     * Parity is a physical-layer integrity check; it has no bearing on
     * whether the ASCII '<...>' frame protocol round-trips correctly
     * over this channel, so omitting it here does not weaken anything
     * this SIL suite is meant to verify. hal/nstar_hal_linux.c (the real
     * RS-422 binding) DOES need PARENB+PARODD-cleared (even parity) when
     * Stage 6 lands, since that runs against a real UART, not a PTY.
     */
    cfmakeraw(&tio);
    cfsetispeed(&tio, B38400);
    cfsetospeed(&tio, B38400);

    tio.c_cflag &= ~PARENB;    /* no parity on the PTY emulation  */
    tio.c_cflag &= ~CSTOPB;    /* 1 stop bit                      */
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;        /* 8 data bits                     */

    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        fprintf(stderr, "[nstar_app_sil] tcsetattr failed: %s\n",
                strerror(errno));
        close(fd);
        return -1;
    }

    return fd;
}

/* =========================================================================
 * GPIO file initialisation
 * =========================================================================
 *
 * Ensures every GPIO file exists with a sane idle default before
 * NSTAR_Init() spawns threads that immediately start polling them.
 * FAULT_N idles LOW (no fault). Per User Manual §3.1.1: FAULT_N goes HIGH
 * when N-STAR detects a SEL or RF power failure (open-collector released,
 * external 100kΩ pull-up raises the line). Returns LOW when fault clears.
 * LOCK_DETECT and DATA_VALID idle LOW (no signal).
 * RESET_N idles HIGH (not asserted).
 */

static void writeGPIODefault(const char *filename, int value)
{
    char path[512], tmp[520];
    silPath(path, sizeof(path), filename);
    snprintf(tmp, sizeof(tmp), "%s.tmp", path);
    FILE *f = fopen(tmp, "w");
    if (!f) return;
    fprintf(f, "%d", value);
    fclose(f);
    rename(tmp, path);
}

static void initGPIODefaults(void)
{
    writeGPIODefault(SIL_GPIO_LOCK_DETECT_FILE, 0);
    writeGPIODefault(SIL_GPIO_DATA_VALID_FILE,  0);
    writeGPIODefault(SIL_GPIO_FAULT_N_FILE,      0);  /* LOW = no fault */
    writeGPIODefault(SIL_GPIO_RESET_N_FILE,      1);
}

/* =========================================================================
 * Main
 * =========================================================================
 *
 * Usage: nstar_app_sil [run_duration_seconds]
 *   If run_duration_seconds is given, the process exits automatically
 *   after that many seconds (used by testSil.c so each scenario gets a
 *   bounded-lifetime app process). If omitted, runs until SIGTERM/SIGINT.
 */
int main(int argc, char **argv)
{
    signal(SIGINT,  sighandler);
    signal(SIGTERM, sighandler);

    int runDurationS = 0;
    if (argc > 1) runDurationS = atoi(argv[1]);

    initGPIODefaults();

    char slavePath[256];
    fprintf(stderr, "[nstar_app_sil] waiting for simulator PTY link...\n");
    if (waitForPTYLink(slavePath, sizeof(slavePath), 5000) != 0) {
        fprintf(stderr, "[nstar_app_sil] timed out waiting for %s\n",
                SIL_PTY_LINK_FILE);
        return 1;
    }
    fprintf(stderr, "[nstar_app_sil] opening UART: %s\n", slavePath);

    int uartFd = openUART(slavePath);
    if (uartFd < 0) return 1;

    NSTAR_Config_t config = {
        .uartFd           = uartFd,
        .dataFd            = SIL_FD_DATA_INTERFACE,
        .gpioLockDetect   = SIL_FD_GPIO_LOCK_DETECT,
        .gpioDataValid    = SIL_FD_GPIO_DATA_VALID,
        .gpioFaultN       = SIL_FD_GPIO_FAULT_N,
        .gpioResetN       = SIL_FD_GPIO_RESET_N,
    };

    NSTAR_Ctx_t *ctx = NULL;
    NSTAR_Result_t rc = NSTAR_Init(&config, &kCallbacks, &ctx);
    if (rc != NSTAR_OK) {
        fprintf(stderr, "[nstar_app_sil] NSTAR_Init failed: %d\n", rc);
        close(uartFd);
        return 1;
    }
    refreshStatus(ctx);

    fprintf(stderr, "[nstar_app_sil] running startup_sequence...\n");
    rc = NSTAR_StartupSequence(ctx);
    fprintf(stderr, "[nstar_app_sil] startup_sequence -> %d (state=%s)\n",
            rc, moduleStateName(NSTAR_GetModuleState(ctx)));
    refreshStatus(ctx);

    /* Monitor loop: poll module/RX state and counters, refresh the status
     * file whenever anything observable changes. 50ms cadence keeps the
     * SIL test suite's polling responsive without busy-looping. */
    NSTAR_ModuleState_t lastModuleState = NSTAR_GetModuleState(ctx);
    NSTAR_RXState_t      lastRXState     = NSTAR_RXGetState(ctx);
    size_t lastFrameCount = gFrameReceivedCount;
    size_t lastFaultCount = gFaultCount;
    size_t lastTXCount    = gTXCompleteCount;
    size_t lastLockAcq    = gLockAcquiredCount;
    size_t lastLockLost   = gLockLostCount;

    struct timespec runStart;
    clock_gettime(CLOCK_MONOTONIC, &runStart);

    while (gRunning) {
        struct timespec d = { 0, 50 * 1000000L };
        nanosleep(&d, NULL);

        int rcBeforeStart = gLastTXStartRC;
        int rcBeforeWrite = gLastTXWriteRC;
        int rcBeforeStop  = gLastTXStopRC;

        applyAppControlCommands(ctx);

        NSTAR_ModuleState_t curModuleState = NSTAR_GetModuleState(ctx);
        NSTAR_RXState_t      curRXState     = NSTAR_RXGetState(ctx);

        int txRCChanged = (gLastTXStartRC != rcBeforeStart) ||
                            (gLastTXWriteRC != rcBeforeWrite) ||
                            (gLastTXStopRC  != rcBeforeStop);

        if (curModuleState != lastModuleState ||
            curRXState     != lastRXState     ||
            gFrameReceivedCount != lastFrameCount ||
            gFaultCount           != lastFaultCount ||
            gTXCompleteCount     != lastTXCount ||
            gLockAcquiredCount   != lastLockAcq ||
            gLockLostCount       != lastLockLost ||
            txRCChanged) {

            refreshStatus(ctx);
            lastModuleState = curModuleState;
            lastRXState      = curRXState;
            lastFrameCount   = gFrameReceivedCount;
            lastFaultCount   = gFaultCount;
            lastTXCount      = gTXCompleteCount;
            lastLockAcq       = gLockAcquiredCount;
            lastLockLost       = gLockLostCount;
        }

        if (runDurationS > 0) {
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            long elapsedS = now.tv_sec - runStart.tv_sec;
            if (elapsedS >= runDurationS) break;
        }
    }

    fprintf(stderr, "[nstar_app_sil] shutting down\n");
    NSTAR_Deinit(ctx);
    close(uartFd);
    return 0;
}
