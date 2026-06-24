/**
 * @file  testTx.c
 * @brief Stage 3 unit tests — TX pipeline.
 *
 * Tests cover nstarTXStart(), nstarTXWrite(), nstarTXStop(),
 * and nstarTXGetStatus() against the mock HAL.
 *
 * TX sequence under test:
 *   nstarTXStart():
 *     W 0x22 = rateCode  (set TX data rate)
 *     clock_start()        (assert CLK_TX)
 *     sleep 100 ms         (stabilise)
 *     R 0x40               (read TX_STATUS, check b4=1)
 *     W 0x40 = 0x01        (TX_MODE = Modulation)
 *
 *   nstarTXWrite():
 *     data_write() in NSTAR_FRAME_SIZE_BYTES chunks
 *
 *   nstarTXStop():
 *     W 0x40 = 0x00        (TX_MODE = Standby)
 *     clock_stop()
 *     onTXComplete(bytesSent)
 */

#include "unity/unity.h"
#include "nstar.h"
#include "nstar_hal_mock.h"
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

/* =========================================================================
 * Fixtures
 * =========================================================================
 */

static nstarCtx_t *gCtx      = NULL;
static size_t       gTXCbBytes = 0;
static int          gTXCbFired = 0;

static void onTXComplete(size_t bytes)
{
    gTXCbFired = 1;
    gTXCbBytes = bytes;
}
static void onFrameReceived(const uint8_t *b, size_t l) { (void)b;(void)l; }
static void onFault(nstarFaultSource_t s)               { (void)s; }
static void onLockAcquired(void)                         {}
static void onLockLost(void)                             {}

static const nstarCallbacks_t kCb = {
    .onFrameReceived = onFrameReceived,
    .onTXComplete    = onTXComplete,
    .onFault          = onFault,
    .onLockAcquired  = onLockAcquired,
    .onLockLost      = onLockLost,
};

static const nstarConfig_t kCfg = {
    .uartFd          = 10,
    .dataFd          = 11,
    .gpioLockDetect = 20,
    .gpioDataValid  = 21,
    .gpioFaultN     = 22,
    .gpioResetN     = 23,
};

static void qr(const char *s)
{
    nstarMockUARTQueueResponse((const uint8_t *)s, strlen(s));
}

/* Queue the 3-response sequence for a successful startup_sequence:
 *   V identity (includes FPGA_OPTION), R 0x06 (FPGA_TYPE), W 0x10 ACK.
 * The redundant R 0x08/R 0x09 reads were removed from nstar_core.c —
 * FPGA_OPTION is already in the V response (IRD Annexe A: 0x08/0x09
 * IS FPGA_OPTION), so re-reading it via R was both wasted round-trips
 * and, worse, the result was being discarded entirely without caching
 * either copy. Now cached on ctx via the V response only. */
static void queueStartupHappy(void)
{
    qr("<V12:010018230042620307:F832>");
    qr("<R02:62:7D57>");
    qr("<A02:00:466C>");
}

/* Queue the standard 3-response sequence for a successful tx_start:
 *   W 0x22 ACK, R 0x40 (clock detected), W 0x40=0x01 ACK */
static void queueTXStartHappy(void)
{
    qr("<A02:00:466C>");   /* W 0x22 rate ACK                        */
    qr("<R02:10:9EA5>");   /* R 0x40 TX_STATUS: b4=1 clock detected  */
    qr("<A02:00:466C>");   /* W 0x40=0x01 modulation ACK             */
}

/* Queue the sequence for a successful tx_stop:
 *   W 0x40=0x00 ACK */
static void queueTXStopHappy(void)
{
    qr("<A02:00:466C>");   /* W 0x40=0x00 standby ACK */
}

void setUp(void)
{
    nstarMockReset();
    gTXCbFired = 0;
    gTXCbBytes = 0;
    nstarResult_t rc = nstarInit(&kCfg, &kCb, &gCtx);
    TEST_ASSERT_EQUAL(NSTAR_OK, rc);

    /*
     * nstarInit() leaves the module in STARTING, not READY.
     * nstarTXStart() and nstarRXConfigure() require READY.
     * Run startup_sequence() here so all TX tests start from READY,
     * matching the application's real init flow.
     */
    queueStartupHappy();
    rc = nstarStartupSequence(gCtx);
    TEST_ASSERT_EQUAL(NSTAR_OK, rc);
    TEST_ASSERT_EQUAL(NSTAR_MODULE_READY, nstarGetModuleState(gCtx));
}

void tearDown(void)
{
    if (gCtx) { nstarDeinit(gCtx); gCtx = NULL; }
}

/* =========================================================================
 * nstarTXGetStatus tests
 * =========================================================================
 */

void testTxGetStatusNullCtxReturnsParamError(void)
{
    nstarTXStatus_t st;
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM, nstarTXGetStatus(NULL, &st));
}

void testTxGetStatusNullOutReturnsParamError(void)
{
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM, nstarTXGetStatus(gCtx, NULL));
}

/**
 * TX_STATUS = 0x10 → mode=STANDBY (b1:b0=00), clockDetected=1 (b4=1).
 */
void testTxGetStatusClockDetected(void)
{
    qr("<R02:10:9EA5>");
    nstarTXStatus_t st;
    memset(&st, 0, sizeof(st));
    TEST_ASSERT_EQUAL(NSTAR_OK, nstarTXGetStatus(gCtx, &st));
    TEST_ASSERT_TRUE(st.clockDetected);
    TEST_ASSERT_EQUAL(NSTAR_TX_STANDBY, st.currentMode);
    TEST_ASSERT_EQUAL_HEX8(0x10, st.raw);
}

/**
 * TX_STATUS = 0x11 → mode=MODULATION (b1:b0=01), clockDetected=1.
 */
void testTxGetStatusModulationActive(void)
{
    qr("<R02:11:AD94>");
    nstarTXStatus_t st;
    memset(&st, 0, sizeof(st));
    TEST_ASSERT_EQUAL(NSTAR_OK, nstarTXGetStatus(gCtx, &st));
    TEST_ASSERT_TRUE(st.clockDetected);
    TEST_ASSERT_EQUAL(NSTAR_TX_MODULATION, st.currentMode);
}

/**
 * TX_STATUS = 0x00 → no clock, standby.
 */
void testTxGetStatusNoClock(void)
{
    qr("<R02:00:A995>");
    nstarTXStatus_t st;
    memset(&st, 0, sizeof(st));
    TEST_ASSERT_EQUAL(NSTAR_OK, nstarTXGetStatus(gCtx, &st));
    TEST_ASSERT_FALSE(st.clockDetected);
    TEST_ASSERT_EQUAL(NSTAR_TX_STANDBY, st.currentMode);
}

/* =========================================================================
 * nstarTXStart tests
 * =========================================================================
 */

void testTxStartNullCtxReturnsNotInit(void)
{
    TEST_ASSERT_EQUAL(NSTAR_ERR_NOT_INIT,
        nstarTXStart(NULL, NSTAR_TX_RATE_32K));
}

/**
 * Happy path: start TX at 32 kbps.
 * Verify: returns OK, clock is running, txActive is set.
 */
void testTxStartSuccess(void)
{
    queueTXStartHappy();
    TEST_ASSERT_EQUAL(NSTAR_OK,
        nstarTXStart(gCtx, NSTAR_TX_RATE_32K));
    TEST_ASSERT_TRUE(nstarMockDataClockIsRunning());
}

/**
 * Verify the UART frames sent during tx_start in the correct order:
 *   Frame 1: W 0x22 = rateCode   (TX data rate)
 *   Frame 2: R 0x40               (read TX_STATUS)
 *   Frame 3: W 0x40 = 0x01        (TX_MODE = Modulation)
 *
 * setUp() already ran startup_sequence(), which wrote its own frames
 * (V, R, R, R, W) to the same cumulative mock write buffer. We record
 * the buffer length before calling tx_start() and only inspect bytes
 * written after that point.
 */
void testTxStartSendsFramesInCorrectOrder(void)
{
    size_t baselineLen = 0;
    nstarMockUARTGetWritten(&baselineLen);

    queueTXStartHappy();
    nstarTXStart(gCtx, NSTAR_TX_RATE_32K);

    size_t wlen = 0;
    const uint8_t *wFull = nstarMockUARTGetWritten(&wlen);
    const uint8_t *w = wFull + baselineLen;
    size_t newLen = wlen - baselineLen;

    /* Frame 1 (the first frame written by tx_start) must be a W command */
    TEST_ASSERT_TRUE_MESSAGE(newLen >= 2, "No new bytes written by tx_start");
    TEST_ASSERT_EQUAL_CHAR('<', (char)w[0]);
    TEST_ASSERT_EQUAL_CHAR('W', (char)w[1]);

    /* Frame 3 must contain 4001 (TX_MODE reg=0x40, val=0x01) */
    /* Scan only the new bytes for <W04:4001: */
    int foundModulation = 0;
    for (size_t i = 0; i + 9 < newLen; i++) {
        if (w[i]=='<' && w[i+1]=='W' &&
            w[i+5]=='4' && w[i+6]=='0' &&
            w[i+7]=='0' && w[i+8]=='1') {
            foundModulation = 1;
            break;
        }
    }
    TEST_ASSERT_TRUE_MESSAGE(foundModulation,
        "Expected W frame 0x40=0x01 (TX_MODE=Modulation) in UART writes");
}

/**
 * TX rate is written to register 0x22.
 * For NSTAR_TX_RATE_32K (0x0F), frame data must contain "220F".
 */
void testTxStartSetsCorrectRateRegister(void)
{
    queueTXStartHappy();
    nstarTXStart(gCtx, NSTAR_TX_RATE_32K);

    size_t wlen = 0;
    const uint8_t *w = nstarMockUARTGetWritten(&wlen);

    /* Look for W frame with "220F" (addr=0x22, val=0x0F) */
    int found = 0;
    for (size_t i = 0; i + 9 < wlen; i++) {
        if (w[i]=='<' && w[i+1]=='W' &&
            w[i+5]=='2' && w[i+6]=='2' &&
            w[i+7]=='0' && w[i+8]=='F') {
            found = 1;
            break;
        }
    }
    TEST_ASSERT_TRUE_MESSAGE(found,
        "Expected W frame addr=0x22 val=0x0F in UART writes");
}

/**
 * Clock not detected (TX_STATUS b4=0) → NSTAR_ERR_NO_CLOCK.
 * Clock must also be stopped after the failure.
 */
void testTxStartNoClockReturnsNoClockError(void)
{
    qr("<A02:00:466C>");   /* W 0x22 rate ACK                         */
    qr("<R02:00:A995>");   /* R 0x40 TX_STATUS: b4=0 clock NOT found  */

    TEST_ASSERT_EQUAL(NSTAR_ERR_NO_CLOCK,
        nstarTXStart(gCtx, NSTAR_TX_RATE_32K));

    /* Clock must be de-asserted after failure */
    TEST_ASSERT_FALSE(nstarMockDataClockIsRunning());
}

/**
 * UART timeout on rate-write → tx_start returns NSTAR_ERR_TIMEOUT.
 */
void testTxStartRateWriteTimeoutReturnsTimeout(void)
{
    nstarMockUARTForceTimeout(1);
    TEST_ASSERT_EQUAL(NSTAR_ERR_TIMEOUT,
        nstarTXStart(gCtx, NSTAR_TX_RATE_32K));
}

/**
 * Calling tx_start when already active → NSTAR_ERR_BUSY.
 */
void testTxStartWhenAlreadyActiveReturnsBusy(void)
{
    queueTXStartHappy();
    TEST_ASSERT_EQUAL(NSTAR_OK, nstarTXStart(gCtx, NSTAR_TX_RATE_32K));
    TEST_ASSERT_EQUAL(NSTAR_ERR_BUSY,
        nstarTXStart(gCtx, NSTAR_TX_RATE_32K));
}

/* =========================================================================
 * nstarTXWrite tests
 * =========================================================================
 */

void testTxWriteNullCtxReturnsNotInit(void)
{
    uint8_t buf[4] = {0};
    TEST_ASSERT_EQUAL(NSTAR_ERR_NOT_INIT,
        nstarTXWrite(NULL, buf, 4));
}

void testTxWriteNullBufReturnsParamError(void)
{
    queueTXStartHappy();
    nstarTXStart(gCtx, NSTAR_TX_RATE_32K);
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM,
        nstarTXWrite(gCtx, NULL, 10));
}

/**
 * Write without calling tx_start → NSTAR_ERR_BUSY.
 */
void testTxWriteWithoutStartReturnsBusy(void)
{
    uint8_t buf[16] = {0xAA};
    TEST_ASSERT_EQUAL(NSTAR_ERR_BUSY,
        nstarTXWrite(gCtx, buf, sizeof(buf)));
}

/**
 * Happy path: write 100 bytes, verify all arrive at data interface.
 */
void testTxWriteDeliversAllBytes(void)
{
    queueTXStartHappy();
    nstarTXStart(gCtx, NSTAR_TX_RATE_32K);

    uint8_t payload[100];
    for (int i = 0; i < 100; i++) payload[i] = (uint8_t)i;

    TEST_ASSERT_EQUAL(NSTAR_OK,
        nstarTXWrite(gCtx, payload, sizeof(payload)));

    size_t writtenLen = 0;
    const uint8_t *written = nstarMockDataGetWritten(&writtenLen);
    TEST_ASSERT_EQUAL_UINT(100, writtenLen);
    TEST_ASSERT_EQUAL_MEMORY(payload, written, 100);
}

/**
 * Write exactly NSTAR_FRAME_SIZE_BYTES — should be one chunk.
 */
void testTxWriteExactFrameSize(void)
{
    queueTXStartHappy();
    nstarTXStart(gCtx, NSTAR_TX_RATE_32K);

    uint8_t payload[NSTAR_FRAME_SIZE_BYTES];
    memset(payload, 0xAB, sizeof(payload));

    TEST_ASSERT_EQUAL(NSTAR_OK,
        nstarTXWrite(gCtx, payload, sizeof(payload)));

    size_t wlen = 0;
    const uint8_t *w = nstarMockDataGetWritten(&wlen);
    TEST_ASSERT_EQUAL_UINT(NSTAR_FRAME_SIZE_BYTES, wlen);
    TEST_ASSERT_EQUAL_MEMORY(payload, w, NSTAR_FRAME_SIZE_BYTES);
}

/**
 * Write 2 × NSTAR_FRAME_SIZE_BYTES + 1 byte — should split into 3 chunks
 * but all data arrives intact at the data interface.
 */
void testTxWriteMultiChunkPreservesAllBytes(void)
{
    queueTXStartHappy();
    nstarTXStart(gCtx, NSTAR_TX_RATE_32K);

    size_t total = (NSTAR_FRAME_SIZE_BYTES * 2) + 1;
    uint8_t *payload = malloc(total);
    TEST_ASSERT_NOT_NULL(payload);
    for (size_t i = 0; i < total; i++) payload[i] = (uint8_t)(i & 0xFF);

    TEST_ASSERT_EQUAL(NSTAR_OK, nstarTXWrite(gCtx, payload, total));

    size_t wlen = 0;
    const uint8_t *w = nstarMockDataGetWritten(&wlen);
    TEST_ASSERT_EQUAL_UINT(total, wlen);
    TEST_ASSERT_EQUAL_MEMORY(payload, w, total);
    free(payload);
}

/**
 * Multiple sequential tx_write calls accumulate bytes correctly.
 */
void testTxWriteMultipleCallsAccumulate(void)
{
    queueTXStartHappy();
    nstarTXStart(gCtx, NSTAR_TX_RATE_32K);

    uint8_t a[50], b[30];
    memset(a, 0xAA, sizeof(a));
    memset(b, 0xBB, sizeof(b));

    TEST_ASSERT_EQUAL(NSTAR_OK, nstarTXWrite(gCtx, a, sizeof(a)));
    TEST_ASSERT_EQUAL(NSTAR_OK, nstarTXWrite(gCtx, b, sizeof(b)));

    size_t wlen = 0;
    const uint8_t *w = nstarMockDataGetWritten(&wlen);
    TEST_ASSERT_EQUAL_UINT(80, wlen);
    TEST_ASSERT_EQUAL_MEMORY(a, w,      50);
    TEST_ASSERT_EQUAL_MEMORY(b, w + 50, 30);
}

/* =========================================================================
 * nstarTXStop tests
 * =========================================================================
 */

void testTxStopWithoutStartIsOk(void)
{
    /* Calling stop when not active should return OK (idempotent) */
    TEST_ASSERT_EQUAL(NSTAR_OK, nstarTXStop(gCtx));
    TEST_ASSERT_FALSE(gTXCbFired);
}

void testTxStopNullCtxReturnsNotInit(void)
{
    TEST_ASSERT_EQUAL(NSTAR_ERR_NOT_INIT, nstarTXStop(NULL));
}

/**
 * Happy path stop: sends Standby command, stops clock, fires callback.
 */
void testTxStopSendsStandbyFiresCallback(void)
{
    queueTXStartHappy();
    nstarTXStart(gCtx, NSTAR_TX_RATE_32K);

    /* Write some data so bytesSent > 0 */
    uint8_t data[64];
    memset(data, 0xCD, sizeof(data));
    nstarTXWrite(gCtx, data, sizeof(data));

    queueTXStopHappy();
    TEST_ASSERT_EQUAL(NSTAR_OK, nstarTXStop(gCtx));

    /* Callback must have fired */
    TEST_ASSERT_TRUE(gTXCbFired);
    TEST_ASSERT_EQUAL_UINT(64, gTXCbBytes);

    /* Clock must be stopped */
    TEST_ASSERT_FALSE(nstarMockDataClockIsRunning());
}

/**
 * Verify that Standby frame (W 0x40=0x00) is in UART writes after stop.
 */
void testTxStopSendsStandbyFrame(void)
{
    queueTXStartHappy();
    nstarTXStart(gCtx, NSTAR_TX_RATE_32K);
    queueTXStopHappy();
    nstarTXStop(gCtx);

    size_t wlen = 0;
    const uint8_t *w = nstarMockUARTGetWritten(&wlen);

    /* Scan for W frame with 4000 (addr=0x40, val=0x00) */
    int found = 0;
    for (size_t i = 0; i + 9 < wlen; i++) {
        if (w[i]=='<' && w[i+1]=='W' &&
            w[i+5]=='4' && w[i+6]=='0' &&
            w[i+7]=='0' && w[i+8]=='0') {
            found = 1;
            break;
        }
    }
    TEST_ASSERT_TRUE_MESSAGE(found,
        "Expected W frame addr=0x40 val=0x00 (Standby) in UART writes");
}

/**
 * After stop, byte counter is reset — subsequent start/stop reports 0.
 */
void testTxStopResetsByteCounter(void)
{
    /* Session 1 */
    queueTXStartHappy();
    nstarTXStart(gCtx, NSTAR_TX_RATE_32K);
    uint8_t d[100]; memset(d, 0, sizeof(d));
    nstarTXWrite(gCtx, d, 100);
    queueTXStopHappy();
    nstarTXStop(gCtx);
    TEST_ASSERT_EQUAL_UINT(100, gTXCbBytes);

    /* Session 2 — fresh start should report only session-2 bytes */
    nstarMockReset();
    gTXCbBytes = 0;
    gTXCbFired = 0;

    queueTXStartHappy();
    nstarTXStart(gCtx, NSTAR_TX_RATE_32K);
    nstarTXWrite(gCtx, d, 40);
    queueTXStopHappy();
    nstarTXStop(gCtx);
    TEST_ASSERT_EQUAL_UINT(40, gTXCbBytes);
}

/**
 * Full TX session: start → write → stop — verify byte count in callback
 * matches total written.
 */
void testTxFullSessionByteCountMatches(void)
{
    queueTXStartHappy();
    TEST_ASSERT_EQUAL(NSTAR_OK, nstarTXStart(gCtx, NSTAR_TX_RATE_256K));

    uint8_t chunk[512];
    memset(chunk, 0x55, sizeof(chunk));

    /* Write 5 chunks = 2560 bytes */
    for (int i = 0; i < 5; i++) {
        TEST_ASSERT_EQUAL(NSTAR_OK,
            nstarTXWrite(gCtx, chunk, sizeof(chunk)));
    }

    queueTXStopHappy();
    TEST_ASSERT_EQUAL(NSTAR_OK, nstarTXStop(gCtx));

    TEST_ASSERT_TRUE(gTXCbFired);
    TEST_ASSERT_EQUAL_UINT(5 * 512, gTXCbBytes);
}

/* =========================================================================
 * Test runner
 * =========================================================================
 */

int main(void)
{
    UNITY_BEGIN();

    /* tx_get_status */
    RUN_TEST(testTxGetStatusNullCtxReturnsParamError);
    RUN_TEST(testTxGetStatusNullOutReturnsParamError);
    RUN_TEST(testTxGetStatusClockDetected);
    RUN_TEST(testTxGetStatusModulationActive);
    RUN_TEST(testTxGetStatusNoClock);

    /* tx_start */
    RUN_TEST(testTxStartNullCtxReturnsNotInit);
    RUN_TEST(testTxStartSuccess);
    RUN_TEST(testTxStartSendsFramesInCorrectOrder);
    RUN_TEST(testTxStartSetsCorrectRateRegister);
    RUN_TEST(testTxStartNoClockReturnsNoClockError);
    RUN_TEST(testTxStartRateWriteTimeoutReturnsTimeout);
    RUN_TEST(testTxStartWhenAlreadyActiveReturnsBusy);

    /* tx_write */
    RUN_TEST(testTxWriteNullCtxReturnsNotInit);
    RUN_TEST(testTxWriteNullBufReturnsParamError);
    RUN_TEST(testTxWriteWithoutStartReturnsBusy);
    RUN_TEST(testTxWriteDeliversAllBytes);
    RUN_TEST(testTxWriteExactFrameSize);
    RUN_TEST(testTxWriteMultiChunkPreservesAllBytes);
    RUN_TEST(testTxWriteMultipleCallsAccumulate);

    /* tx_stop */
    RUN_TEST(testTxStopNullCtxReturnsNotInit);
    RUN_TEST(testTxStopWithoutStartIsOk);
    RUN_TEST(testTxStopSendsStandbyFiresCallback);
    RUN_TEST(testTxStopSendsStandbyFrame);
    RUN_TEST(testTxStopResetsByteCounter);
    RUN_TEST(testTxFullSessionByteCountMatches);

    return UNITY_END();
}
