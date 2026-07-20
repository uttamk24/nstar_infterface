/**
 * @file  testRx.c
 * @brief Stage 4 unit tests — RX pipeline.
 *
 * Tests cover:
 *   - NSTAR_RXConfigure()
 *   - NSTAR_RXGetStatus() register decode
 *   - NSTAR_RXGetLinkQuality() multi-register decode
 *   - rxThreadFunc() state machine via GPIO edge sequences
 *
 * Threading notes:
 *   NSTAR_Init() now spawns the rxThread, faultThread, and healthThread.
 *   Tests that exercise the rxThread queue GPIO edges via the mock and then
 *   wait for the onFrameReceived / onLockAcquired / onLockLost
 *   callbacks using a pthread_cond_t.
 *
 *   setUp() calls NSTAR_Init() which starts all three threads.
 *   The fault and health threads block on GPIO edges / sleep — they will
 *   not interfere with RX tests because their GPIO fds differ.
 */

#include "unity/unity.h"
#include "ttc_nstar.h"
#include "ttc_nstar_hal_mock.h"
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>

/* =========================================================================
 * Synchronisation helpers for threaded tests
 * =========================================================================
 */

typedef struct {
    pthread_mutex_t mu;
    pthread_cond_t  cv;
    int             fired;
    /* captured callback data */
    uint8_t         frameBuf[NSTAR_FRAME_SIZE_BYTES];
    size_t          frameLen;
    int             lockAcquired;
    int             lockLost;
    NSTAR_FaultSource_t faultSource;
    int             faultFired;
} cbSync_t;

static cbSync_t gSync;

static void syncInit(void)
{
    pthread_mutex_init(&gSync.mu, NULL);
    pthread_cond_init(&gSync.cv, NULL);
    memset(&gSync.frameBuf, 0, sizeof(gSync.frameBuf));
    gSync.fired         = 0;
    gSync.frameLen     = 0;
    gSync.lockAcquired = 0;
    gSync.lockLost     = 0;
    gSync.faultFired   = 0;
}

static void syncDestroy(void)
{
    pthread_mutex_destroy(&gSync.mu);
    pthread_cond_destroy(&gSync.cv);
}

/** Wait up to timeoutMs for gSync.fired to become non-zero. */
static int syncWait(int timeoutMs)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec  += timeoutMs / 1000;
    ts.tv_nsec += (timeoutMs % 1000) * 1000000L;
    if (ts.tv_nsec >= 1000000000L) { ts.tv_sec++; ts.tv_nsec -= 1000000000L; }

    pthread_mutex_lock(&gSync.mu);
    int rc = 0;
    while (!gSync.fired && rc == 0) {
        rc = pthread_cond_timedwait(&gSync.cv, &gSync.mu, &ts);
    }
    int result = gSync.fired;
    pthread_mutex_unlock(&gSync.mu);
    return result;
}

/**
 * Wait up to timeoutMs specifically for onFrameReceived to fire.
 * onLockAcquired and onLockLost also call syncSignal() and set the
 * generic gSync.fired flag, so a plain syncWait() can return early on
 * the wrong event. This variant loops past those, re-arming the wait,
 * until frameLen becomes non-zero or the deadline passes.
 */
static int syncWaitForFrame(int timeoutMs)
{
    struct timespec deadline;
    clock_gettime(CLOCK_REALTIME, &deadline);
    deadline.tv_sec  += timeoutMs / 1000;
    deadline.tv_nsec += (timeoutMs % 1000) * 1000000L;
    if (deadline.tv_nsec >= 1000000000L) {
        deadline.tv_sec++; deadline.tv_nsec -= 1000000000L;
    }

    pthread_mutex_lock(&gSync.mu);
    while (gSync.frameLen == 0) {
        int rc = pthread_cond_timedwait(&gSync.cv, &gSync.mu, &deadline);
        if (rc != 0) break;   /* timed out */
        /* Spurious wake from a different callback (lockAcquired/lockLost):
         * loop back and keep waiting until frameLen is set or deadline hits. */
    }
    int got = (gSync.frameLen != 0);
    pthread_mutex_unlock(&gSync.mu);
    return got;
}

static void syncSignal(void)
{
    pthread_mutex_lock(&gSync.mu);
    gSync.fired = 1;
    pthread_cond_signal(&gSync.cv);
    pthread_mutex_unlock(&gSync.mu);
}

/* =========================================================================
 * Callbacks
 * =========================================================================
 */

static void onFrameReceived(const uint8_t *buf, size_t len)
{
    if (len > sizeof(gSync.frameBuf)) len = sizeof(gSync.frameBuf);
    memcpy(gSync.frameBuf, buf, len);
    gSync.frameLen = len;
    syncSignal();
}

static void onTXComplete(size_t n) { (void)n; }

static void onFault(NSTAR_FaultSource_t src)
{
    gSync.faultSource = src;
    gSync.faultFired  = 1;
    syncSignal();
}

static void onLockAcquired(void)
{
    gSync.lockAcquired = 1;
    syncSignal();
}

static void onLockLost(void)
{
    gSync.lockLost = 1;
    syncSignal();
}

static const NSTAR_Callbacks_t kCb = {
    .onFrameReceived = onFrameReceived,
    .onTXComplete    = onTXComplete,
    .onFault          = onFault,
    .onLockAcquired  = onLockAcquired,
    .onLockLost      = onLockLost,
};

/* =========================================================================
 * Fixtures
 * =========================================================================
 */

#define FD_UART   10
#define FD_DATA   11
#define FD_LD     20   /* LOCK_DETECT  */
#define FD_DV     21   /* DATA_VALID   */
#define FD_FN     22   /* FAULT_N      */
#define FD_RST    23   /* RESET_N      */

static const NSTAR_Config_t kCfg = {
    .uartFd          = FD_UART,
    .dataFd          = FD_DATA,
    .gpioLockDetect = FD_LD,
    .gpioDataValid  = FD_DV,
    .gpioFaultN     = FD_FN,
    .gpioResetN     = FD_RST,
};

static NSTAR_Ctx_t *gCtx = NULL;

static void qr(const char *s)
{
    nstarMockUARTQueueResponse((const uint8_t *)s, strlen(s));
}

/* Queue the 3-response sequence for a successful startup_sequence
 * (V, R 0x06, W 0x10 — the redundant R 0x08/R 0x09 reads were removed
 * from nstar_core.c since FPGA_OPTION is already in the V response). */
static void queueStartupHappy(void)
{
    qr("<V12:010018230042620307:F832>");
    qr("<R02:62:7D57>");
    qr("<A02:00:466C>");
}

void setUp(void)
{
    nstarMockReset();
    syncInit();
    /* Pre-set DATA_VALID and FAULT_N to idle state */
    nstarMockGPIOSetValue(FD_DV, 0);
    nstarMockGPIOSetValue(FD_FN, 1);  /* FAULT_N idle = HIGH */
    NSTAR_Result_t rc = NSTAR_Init(&kCfg, &kCb, &gCtx);
    TEST_ASSERT_EQUAL(NSTAR_OK, rc);

    /*
     * NSTAR_Init() leaves the module in STARTING, not READY.
     * NSTAR_RXConfigure() requires READY. Run startup_sequence() here
     * so all RX tests start from READY, matching the real init flow.
     */
    queueStartupHappy();
    rc = NSTAR_StartupSequence(gCtx);
    TEST_ASSERT_EQUAL(NSTAR_OK, rc);
    TEST_ASSERT_EQUAL(NSTAR_MODULE_READY, NSTAR_GetModuleState(gCtx));
}

void tearDown(void)
{
    if (gCtx) { NSTAR_Deinit(gCtx); gCtx = NULL; }
    syncDestroy();
}

/* =========================================================================
 * NSTAR_RXConfigure tests
 * =========================================================================
 */

void testRxConfigureNullCtxReturnsNotInit(void)
{
    TEST_ASSERT_EQUAL(NSTAR_ERR_NOT_INIT,
        NSTAR_RXConfigure(NULL, NSTAR_RX_RATE_16K));
}

/**
 * NSTAR_RXConfigure sends W 0x22 = rateCode.
 * For NSTAR_RX_RATE_16K (0x1F), expect frame <W04:221F:CCCC>.
 */
void testRxConfigureSendsCorrectRegisterWrite(void)
{
    qr("<A02:00:466C>");  /* W 0x22 ACK */
    TEST_ASSERT_EQUAL(NSTAR_OK,
        NSTAR_RXConfigure(gCtx, NSTAR_RX_RATE_16K));

    size_t wlen = 0;
    const uint8_t *w = nstarMockUARTGetWritten(&wlen);

    /* <W04:221F:...> — addr=0x22, val=0x1F */
    int found = 0;
    for (size_t i = 0; i + 9 < wlen; i++) {
        if (w[i]=='<' && w[i+1]=='W' &&
            w[i+5]=='2' && w[i+6]=='2' &&
            w[i+7]=='1' && w[i+8]=='F') {
            found = 1; break;
        }
    }
    TEST_ASSERT_TRUE_MESSAGE(found,
        "Expected W frame addr=0x22 val=0x1F in UART writes");
}

/* =========================================================================
 * NSTAR_RXGetStatus tests
 * =========================================================================
 */

void testRxGetStatusNullOutReturnsParamError(void)
{
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM, NSTAR_RXGetStatus(gCtx, NULL));
}

/**
 * RX_STATUS = 0x0F → all four lock bits set.
 */
void testRxGetStatusAllBitsSet(void)
{
    qr("<R02:0F:0B6A>");
    NSTAR_RXStatus_t st;
    memset(&st, 0, sizeof(st));
    TEST_ASSERT_EQUAL(NSTAR_OK, NSTAR_RXGetStatus(gCtx, &st));
    TEST_ASSERT_TRUE(st.carrierDetect);
    TEST_ASSERT_TRUE(st.carrierLock);
    TEST_ASSERT_TRUE(st.bitLock);
    TEST_ASSERT_TRUE(st.dataValid);
    TEST_ASSERT_EQUAL_HEX8(0x0F, st.raw);
}

/**
 * RX_STATUS = 0x01 → only carrierDetect set, others clear.
 */
void testRxGetStatusCarrierOnly(void)
{
    qr("<R02:01:9AA4>");
    NSTAR_RXStatus_t st;
    memset(&st, 0, sizeof(st));
    TEST_ASSERT_EQUAL(NSTAR_OK, NSTAR_RXGetStatus(gCtx, &st));
    TEST_ASSERT_TRUE(st.carrierDetect);
    TEST_ASSERT_FALSE(st.carrierLock);
    TEST_ASSERT_FALSE(st.bitLock);
    TEST_ASSERT_FALSE(st.dataValid);
}

/* =========================================================================
 * NSTAR_RXGetLinkQuality tests
 * =========================================================================
 */

void testRxGetLinkQualityNullOutReturnsParamError(void)
{
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM,
        NSTAR_RXGetLinkQuality(gCtx, NULL));
}

/**
 * Eb = 0x001000 = 4096, N0 = 0x000100 = 256
 * Eb/N0 = 20*log10(4096/256) - 3 = 20*log10(16) - 3 = 20*1.204 - 3 = 21.1 dB
 * IQ power = 0x0040 = 64, AGC = 0x0020 = 32
 * RSSI = 64 - 32 = 32 dBm
 * Freq shift = 0x000080 = 128 / 8 = 16 Hz
 */
void testRxGetLinkQualityDecodesCorrectly(void)
{
    /* Eb: 3 individual register reads */
    qr("<R02:00:A995>"); qr("<R02:10:9EA5>"); qr("<R02:00:A995>");
    /* N0: 3 reads */
    qr("<R02:00:A995>"); qr("<R02:01:9AA4>"); qr("<R02:00:A995>");
    /* IQ power: 2 reads */
    qr("<R02:00:A995>"); qr("<R02:40:7555>");
    /* AGC: 2 reads */
    qr("<R02:00:A995>"); qr("<R02:20:C7F5>");
    /* Freq shift: 3 reads */
    qr("<R02:00:A995>"); qr("<R02:00:A995>"); qr("<R02:80:0034>");

    NSTAR_LinkQuality_t lq;
    memset(&lq, 0, sizeof(lq));
    TEST_ASSERT_EQUAL(NSTAR_OK, NSTAR_RXGetLinkQuality(gCtx, &lq));

    /* Eb/N0 ≈ 21.1 dB — accept ±0.5 for float precision */
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 21.1f, lq.ebNoDB);

    /* RSSI = 64 - 32 = 32 */
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 32.0f, lq.rssiDBM);

    /* Frequency shift = 128/8 = 16 Hz */
    TEST_ASSERT_EQUAL_INT32(16, lq.freqShiftHz);
}

/**
 * Negative frequency shift — sign extension of 24-bit to 32-bit.
 * 0xFFFF80 = -128 (two's complement 24-bit) → /8 = -16 Hz
 */
void testRxGetLinkQualityNegativeFreqShift(void)
{
    /* Eb, N0, IQ, AGC — reuse simple values */
    qr("<R02:00:A995>"); qr("<R02:10:9EA5>"); qr("<R02:00:A995>");
    qr("<R02:00:A995>"); qr("<R02:01:9AA4>"); qr("<R02:00:A995>");
    qr("<R02:00:A995>"); qr("<R02:40:7555>");
    qr("<R02:00:A995>"); qr("<R02:20:C7F5>");

    /* Freq shift: 0xFFFF80 = -128 signed-24, /8 = -16 Hz */
    /* Compute CRC for each byte frame */
    qr("<R02:FF:61C2>"); qr("<R02:FF:61C2>"); qr("<R02:80:0034>");

    NSTAR_LinkQuality_t lq;
    memset(&lq, 0, sizeof(lq));
    TEST_ASSERT_EQUAL(NSTAR_OK, NSTAR_RXGetLinkQuality(gCtx, &lq));
    TEST_ASSERT_EQUAL_INT32(-16, lq.freqShiftHz);
}

/* =========================================================================
 * RX thread state machine tests
 * =========================================================================
 */

/**
 * LOCK_DETECT rising → onLockAcquired callback fires.
 * Queue: LOCK_DETECT rising edge, then DATA_VALID edge times out.
 */
void testRxThreadLockDetectFiresLockAcquiredCb(void)
{
    /* Reset sync state for this specific check */
    gSync.lockAcquired = 0;
    gSync.fired         = 0;

    /* Queue LOCK_DETECT rising edge */
    nstarMockGPIOQueueEdge(FD_LD, NSTAR_GPIO_EDGE_RISING, 0);

    /* Wait for onLockAcquired to fire (up to 1 s) */
    int got = syncWait(1000);
    TEST_ASSERT_TRUE_MESSAGE(got, "onLockAcquired callback did not fire");
    TEST_ASSERT_TRUE(gSync.lockAcquired);
}

/**
 * Full lock sequence: LOCK_DETECT rising, DATA_VALID rising,
 * data available → onFrameReceived fires with correct bytes.
 */
void testRxThreadFullLockSequenceDeliversFrame(void)
{
    /* Supply 64 bytes to the data interface */
    uint8_t payload[64];
    for (int i = 0; i < 64; i++) payload[i] = (uint8_t)i;
    nstarMockDataSupplyRead(payload, sizeof(payload));

    /* Set DATA_VALID HIGH so gpio_read() returns 1 after lock */
    nstarMockGPIOSetValue(FD_DV, 1);

    /* Queue GPIO edges: LOCK_DETECT rising, then DATA_VALID rising */
    nstarMockGPIOQueueEdge(FD_LD, NSTAR_GPIO_EDGE_RISING, 0);
    nstarMockGPIOQueueEdge(FD_DV, NSTAR_GPIO_EDGE_RISING, 0);

    /* Wait specifically for onFrameReceived (not lockAcquired) */
    int got = syncWaitForFrame(2000);
    TEST_ASSERT_TRUE_MESSAGE(got, "onFrameReceived callback did not fire");
    TEST_ASSERT_EQUAL_UINT(64, gSync.frameLen);
    TEST_ASSERT_EQUAL_MEMORY(payload, gSync.frameBuf, 64);
}

/**
 * DATA_VALID falls mid-session → onLockLost fires.
 * Supply one frame of data; after it is read, DATA_VALID goes LOW.
 */
void testRxThreadDataValidFallingFiresLockLostCb(void)
{
    /*
     * Supply first batch: thread reads this, fires onFrameReceived.
     * After frame callback, test drops DATA_VALID to 0.
     * Thread then calls gpio_read() = 0, exits read loop, fires onLockLost.
     */
    uint8_t payload[32];
    memset(payload, 0xAB, sizeof(payload));
    nstarMockDataSupplyRead(payload, sizeof(payload));

    /* DATA_VALID HIGH so gpio_read returns 1 during the read loop */
    nstarMockGPIOSetValue(FD_DV, 1);

    nstarMockGPIOQueueEdge(FD_LD, NSTAR_GPIO_EDGE_RISING, 0);
    nstarMockGPIOQueueEdge(FD_DV, NSTAR_GPIO_EDGE_RISING, 0);

    /* Wait specifically for onFrameReceived (not lockAcquired) */
    int got = syncWaitForFrame(2000);
    TEST_ASSERT_TRUE_MESSAGE(got, "Frame callback did not fire");
    TEST_ASSERT_EQUAL_UINT(32, gSync.frameLen);

    /*
     * Drop DATA_VALID BEFORE resetting sync state.
     * The thread will see gpio_read()=0 on its next loop iteration
     * and call onLockLost, which calls syncSignal().
     * We then reset gSync.fired under the mutex so we can wait again.
     */
    nstarMockGPIOSetValue(FD_DV, 0);

    /* Reset sync state under the mutex to avoid race */
    pthread_mutex_lock(&gSync.mu);
    gSync.fired     = 0;
    gSync.lockLost = 0;
    pthread_mutex_unlock(&gSync.mu);

    /* Wait for onLockLost (up to 2 s) */
    got = syncWait(2000);
    TEST_ASSERT_TRUE_MESSAGE(got, "onLockLost callback did not fire");
    TEST_ASSERT_TRUE(gSync.lockLost);
}

/* =========================================================================
 * Test runner
 * =========================================================================
 */

int main(void)
{
    UNITY_BEGIN();

    /* rx_configure */
    RUN_TEST(testRxConfigureNullCtxReturnsNotInit);
    RUN_TEST(testRxConfigureSendsCorrectRegisterWrite);

    /* rx_get_status */
    RUN_TEST(testRxGetStatusNullOutReturnsParamError);
    RUN_TEST(testRxGetStatusAllBitsSet);
    RUN_TEST(testRxGetStatusCarrierOnly);

    /* rx_get_link_quality */
    RUN_TEST(testRxGetLinkQualityNullOutReturnsParamError);
    RUN_TEST(testRxGetLinkQualityDecodesCorrectly);
    RUN_TEST(testRxGetLinkQualityNegativeFreqShift);

    /* rxThread state machine */
    RUN_TEST(testRxThreadLockDetectFiresLockAcquiredCb);
    RUN_TEST(testRxThreadFullLockSequenceDeliversFrame);
    RUN_TEST(testRxThreadDataValidFallingFiresLockLostCb);

    return UNITY_END();
}
