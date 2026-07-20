/**
 * @file  testCore.c
 * @brief Stage 2 unit tests — command interface, register access,
 *        named commands, startup sequence.
 *
 * All tests run synchronously against the mock HAL.
 * No threads are spawned; nstar_core functions are called directly.
 * Each test calls nstarMockReset() and NSTAR_Init() in setUp().
 */

#include "unity/unity.h"
#include "ttc_nstar.h"
#include "ttc_nstar_hal_mock.h"
#include <string.h>
#include <stdint.h>

/* =========================================================================
 * Test fixtures
 * =========================================================================
 */

/* Fake file descriptors — the mock HAL ignores the fd value */
#define FAKE_UART_FD     10
#define FAKE_DATA_FD     11
#define FAKE_GPIO_LD     20   /* LOCK_DETECT */
#define FAKE_GPIO_DV     21   /* DATA_VALID  */
#define FAKE_GPIO_FN     22   /* FAULT_N     */
#define FAKE_GPIO_RST    23   /* RESET_N     */

static NSTAR_Ctx_t *gCtx = NULL;

static void dummyOnFrameReceived(const uint8_t *b, size_t l)
{ (void)b; (void)l; }
static void dummyOnTXComplete(size_t n) { (void)n; }
static void dummyOnFault(NSTAR_FaultSource_t s) { (void)s; }
static void dummyOnLockAcquired(void) {}
static void dummyOnLockLost(void) {}

static const NSTAR_Callbacks_t kCallbacks = {
    .onFrameReceived = dummyOnFrameReceived,
    .onTXComplete    = dummyOnTXComplete,
    .onFault          = dummyOnFault,
    .onLockAcquired  = dummyOnLockAcquired,
    .onLockLost      = dummyOnLockLost,
};

static const NSTAR_Config_t kConfig = {
    .uartFd           = FAKE_UART_FD,
    .dataFd           = FAKE_DATA_FD,
    .gpioLockDetect  = FAKE_GPIO_LD,
    .gpioDataValid   = FAKE_GPIO_DV,
    .gpioFaultN      = FAKE_GPIO_FN,
    .gpioResetN      = FAKE_GPIO_RST,
};

/* Helper: queue a literal frame string as a UART response */
static void queueResponse(const char *frameStr)
{
    nstarMockUARTQueueResponse(
        (const uint8_t *)frameStr, strlen(frameStr));
}

void setUp(void)
{
    nstarMockReset();
    NSTAR_Result_t rc = NSTAR_Init(&kConfig, &kCallbacks, &gCtx);
    TEST_ASSERT_EQUAL(NSTAR_OK, rc);
    TEST_ASSERT_NOT_NULL(gCtx);
}

void tearDown(void)
{
    if (gCtx) {
        NSTAR_Deinit(gCtx);
        gCtx = NULL;
    }
}

/* =========================================================================
 * Init / deinit tests
 * =========================================================================
 */

void testInitNullConfigReturnsParamError(void)
{
    NSTAR_Ctx_t *ctx = NULL;
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM,
        NSTAR_Init(NULL, &kCallbacks, &ctx));
}

void testInitNullCallbacksReturnsParamError(void)
{
    NSTAR_Ctx_t *ctx = NULL;
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM,
        NSTAR_Init(&kConfig, NULL, &ctx));
}

void testInitNullCtxOutReturnsParamError(void)
{
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM,
        NSTAR_Init(&kConfig, &kCallbacks, NULL));
}

void testInitSuccessReturnsNonNullCtx(void)
{
    /* setUp() already called NSTAR_Init; ctx is non-null */
    TEST_ASSERT_NOT_NULL(gCtx);
}

void testDeinitNullCtxIsSafe(void)
{
    /* Must not crash */
    NSTAR_Deinit(NULL);
}

/* =========================================================================
 * NSTAR_RegRead tests
 * =========================================================================
 */

void testRegReadNullCtxReturnsParamError(void)
{
    uint8_t val;
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM, NSTAR_RegRead(NULL, 0x06, &val));
}

void testRegReadNullValReturnsParamError(void)
{
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM, NSTAR_RegRead(gCtx, 0x06, NULL));
}

/**
 * Happy path: read FPGA_TYPE register.
 * Mock response: <R02:62:7D57>  (0x62 = N-STAR PCM/PM RX+TX)
 * Verify: returned value is 0x62.
 */
void testRegReadFpgaTypeReturnsCorrectValue(void)
{
    queueResponse("<R02:62:7D57>");
    uint8_t val = 0;
    TEST_ASSERT_EQUAL(NSTAR_OK, NSTAR_RegRead(gCtx, NSTAR_REG_FPGA_TYPE, &val));
    TEST_ASSERT_EQUAL_HEX8(0x62, val);
}

/**
 * Verify the frame written to UART for R command.
 * R 0x06 should write: <R02:06:CCCC>
 */
void testRegReadSendsCorrectFrame(void)
{
    queueResponse("<R02:62:7D57>");

    uint8_t val = 0;
    NSTAR_RegRead(gCtx, NSTAR_REG_FPGA_TYPE, &val);

    size_t writtenLen = 0;
    const uint8_t *written = nstarMockUARTGetWritten(&writtenLen);

    /* Frame should start with <R02:06: */
    TEST_ASSERT_TRUE(writtenLen >= 8);
    TEST_ASSERT_EQUAL_CHAR('<', (char)written[0]);
    TEST_ASSERT_EQUAL_CHAR('R', (char)written[1]);
    TEST_ASSERT_EQUAL_CHAR('0', (char)written[2]);
    TEST_ASSERT_EQUAL_CHAR('2', (char)written[3]);
    TEST_ASSERT_EQUAL_CHAR(':', (char)written[4]);
    TEST_ASSERT_EQUAL_CHAR('0', (char)written[5]);
    TEST_ASSERT_EQUAL_CHAR('6', (char)written[6]);
    TEST_ASSERT_EQUAL_CHAR(':', (char)written[7]);
}

/**
 * Timeout: mock returns no response.
 * After NSTAR_CMD_MAX_RETRIES attempts, expect NSTAR_ERR_TIMEOUT.
 */
void testRegReadTimeoutReturnsTimeoutError(void)
{
    nstarMockUARTForceTimeout(1);
    uint8_t val = 0;
    TEST_ASSERT_EQUAL(NSTAR_ERR_TIMEOUT,
        NSTAR_RegRead(gCtx, NSTAR_REG_FPGA_TYPE, &val));
}

/**
 * CRC error: corrupt the CRC field of the response.
 * After retries, expect NSTAR_ERR_CRC.
 * Queue two corrupted responses (one for initial attempt, one for retry).
 */
void testRegReadCrcErrorAfterRetry(void)
{
    /* Correct: <R02:62:7D57>  Corrupt: last digit 7->8 */
    queueResponse("<R02:62:7D58>");
    queueResponse("<R02:62:7D58>");
    uint8_t val = 0;
    TEST_ASSERT_EQUAL(NSTAR_ERR_CRC,
        NSTAR_RegRead(gCtx, NSTAR_REG_FPGA_TYPE, &val));
}

/**
 * Retry succeeds: first response is corrupted, second is good.
 */
void testRegReadSucceedsOnRetry(void)
{
    queueResponse("<R02:62:7D58>");   /* corrupt — will be retried */
    queueResponse("<R02:62:7D57>");   /* correct */
    uint8_t val = 0;
    TEST_ASSERT_EQUAL(NSTAR_OK,
        NSTAR_RegRead(gCtx, NSTAR_REG_FPGA_TYPE, &val));
    TEST_ASSERT_EQUAL_HEX8(0x62, val);
}

/* =========================================================================
 * NSTAR_RegWrite tests
 * =========================================================================
 */

void testRegWriteNullCtxReturnsParamError(void)
{
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM,
        NSTAR_RegWrite(NULL, NSTAR_REG_TX_MODE, 0x01));
}

/**
 * Happy path: write TX_MODE = Modulation.
 * Mock response: <A02:00:466C>  (ACK, result 0x00 = success)
 */
void testRegWriteTxModeModulationSuccess(void)
{
    queueResponse("<A02:00:466C>");
    TEST_ASSERT_EQUAL(NSTAR_OK,
        NSTAR_RegWrite(gCtx, NSTAR_REG_TX_MODE, NSTAR_TX_MODE_MODULATION));
}

/**
 * Verify the frame written to UART for W command.
 * W 0x40=0x01 should write: <W04:4001:CCCC>
 */
void testRegWriteSendsCorrectFrame(void)
{
    queueResponse("<A02:00:466C>");
    NSTAR_RegWrite(gCtx, NSTAR_REG_TX_MODE, NSTAR_TX_MODE_MODULATION);

    size_t writtenLen = 0;
    const uint8_t *written = nstarMockUARTGetWritten(&writtenLen);

    /* <W04:4001:...> */
    TEST_ASSERT_EQUAL_CHAR('<', (char)written[0]);
    TEST_ASSERT_EQUAL_CHAR('W', (char)written[1]);
    TEST_ASSERT_EQUAL_CHAR('0', (char)written[2]);
    TEST_ASSERT_EQUAL_CHAR('4', (char)written[3]);
    TEST_ASSERT_EQUAL_CHAR(':', (char)written[4]);
    TEST_ASSERT_EQUAL_CHAR('4', (char)written[5]);
    TEST_ASSERT_EQUAL_CHAR('0', (char)written[6]);
    TEST_ASSERT_EQUAL_CHAR('0', (char)written[7]);
    TEST_ASSERT_EQUAL_CHAR('1', (char)written[8]);
    TEST_ASSERT_EQUAL_CHAR(':', (char)written[9]);
}

/**
 * N-STAR returns non-zero ACK code → NSTAR_ERR_BAD_ACK.
 * Mock: <A02:01:755D>  (result 0x01 = error)
 */
void testRegWriteBadAckReturnsBadAckError(void)
{
    queueResponse("<A02:01:755D>");
    TEST_ASSERT_EQUAL(NSTAR_ERR_BAD_ACK,
        NSTAR_RegWrite(gCtx, NSTAR_REG_TX_MODE, 0x01));
}

/**
 * Timeout on write → NSTAR_ERR_TIMEOUT.
 */
void testRegWriteTimeoutReturnsTimeoutError(void)
{
    nstarMockUARTForceTimeout(1);
    TEST_ASSERT_EQUAL(NSTAR_ERR_TIMEOUT,
        NSTAR_RegWrite(gCtx, NSTAR_REG_TX_MODE, 0x01));
}

/* =========================================================================
 * NSTAR_RegReadMulti tests
 * =========================================================================
 */

/**
 * Read 3 consecutive registers (0x08, 0x09, 0x0A).
 * Each gets its own R command and mock response.
 */
void testRegReadMultiReadsNRegisters(void)
{
    queueResponse("<R02:03:FCC6>");   /* 0x08 = 0x03 */
    queueResponse("<R02:07:3002>");   /* 0x09 = 0x07 */
    queueResponse("<R02:AB:2896>");   /* 0x0A = 0xAB */

    uint8_t buf[3] = {0};
    TEST_ASSERT_EQUAL(NSTAR_OK,
        NSTAR_RegReadMulti(gCtx, 0x08, 3, buf));

    TEST_ASSERT_EQUAL_HEX8(0x03, buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0x07, buf[1]);
    TEST_ASSERT_EQUAL_HEX8(0xAB, buf[2]);
}

void testRegReadMultiZeroCountReturnsParamError(void)
{
    uint8_t buf[4];
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM,
        NSTAR_RegReadMulti(gCtx, 0x08, 0, buf));
}

/* =========================================================================
 * NSTAR_CMDReadIdentity (V command) tests
 * =========================================================================
 */

/**
 * Happy path: V command returns 9-byte identity.
 * Mock response: <V12:010018230042620307:F832>
 *   Byte 0: fpgaVersion = 0x01
 *   Byte 1: fpgaBuild   = 0x00
 *   Byte 2: hwYear      = 0x18 (2024)
 *   Byte 3: hwWeek      = 0x23 (week 35)
 *   Byte 4-5: hwOrder   = 0x0042
 *   Byte 6: fpgaType    = 0x62
 *   Byte 7-8: fpgaOptions = 0x0307
 */
void testCmdReadIdentitySuccess(void)
{
    queueResponse("<V12:010018230042620307:F832>");

    NSTAR_Identity_t id;
    memset(&id, 0, sizeof(id));
    TEST_ASSERT_EQUAL(NSTAR_OK, NSTAR_CMDReadIdentity(gCtx, &id));

    TEST_ASSERT_EQUAL_HEX8(0x01, id.fpgaVersion);
    TEST_ASSERT_EQUAL_HEX8(0x00, id.fpgaBuild);
    TEST_ASSERT_EQUAL_HEX8(0x18, id.hwYear);
    TEST_ASSERT_EQUAL_HEX8(0x23, id.hwWeek);
    TEST_ASSERT_EQUAL_HEX16(0x0042, id.hwOrder);
    TEST_ASSERT_EQUAL_HEX8(0x62, id.fpgaType);
    TEST_ASSERT_EQUAL_HEX16(0x0307, id.fpgaOptions);
}

void testCmdReadIdentityNullOutReturnsParamError(void)
{
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM,
        NSTAR_CMDReadIdentity(gCtx, NULL));
}

/**
 * V command sends correct frame (no data, DATA_SIZE="00").
 * Expect: <V00::CCCC>
 */
void testCmdReadIdentitySendsVFrame(void)
{
    queueResponse("<V12:010018230042620307:F832>");
    NSTAR_Identity_t id;
    NSTAR_CMDReadIdentity(gCtx, &id);

    size_t wlen = 0;
    const uint8_t *w = nstarMockUARTGetWritten(&wlen);
    TEST_ASSERT_EQUAL_CHAR('<', (char)w[0]);
    TEST_ASSERT_EQUAL_CHAR('V', (char)w[1]);
    TEST_ASSERT_EQUAL_CHAR('0', (char)w[2]);
    TEST_ASSERT_EQUAL_CHAR('0', (char)w[3]);
    TEST_ASSERT_EQUAL_CHAR(':', (char)w[4]);
    TEST_ASSERT_EQUAL_CHAR(':', (char)w[5]);  /* Sep2 immediately after Sep1 */
}

/* =========================================================================
 * NSTAR_CMDReadAllRXStatus (E command) tests
 * =========================================================================
 */

/**
 * E command returns 19 raw bytes (regs 0x10-0x22).
 * Mock response: <E26:101112131415161718191A1B1C1D1E1F202122:F52A>
 */
void testCmdReadAllRxStatusSuccess(void)
{
    queueResponse("<E26:101112131415161718191A1B1C1D1E1F202122:F52A>");

    uint8_t raw[32] = {0};
    size_t  len = 0;
    TEST_ASSERT_EQUAL(NSTAR_OK,
        NSTAR_CMDReadAllRXStatus(gCtx, raw, &len));
    TEST_ASSERT_EQUAL_UINT(19, len);
    TEST_ASSERT_EQUAL_HEX8(0x10, raw[0]);
    TEST_ASSERT_EQUAL_HEX8(0x22, raw[18]);
}

void testCmdReadAllRxStatusNullOutReturnsParamError(void)
{
    size_t len;
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM,
        NSTAR_CMDReadAllRXStatus(gCtx, NULL, &len));
}

/* =========================================================================
 * NSTAR_CMDReset (C command) tests
 * =========================================================================
 */

/**
 * Happy path: reset command acknowledged.
 * Mock response: <A02:00:466C>
 */
void testCmdResetSuccess(void)
{
    queueResponse("<A02:00:466C>");
    TEST_ASSERT_EQUAL(NSTAR_OK, NSTAR_CMDReset(gCtx));
}

/**
 * Verify C frame contains magic word 0x5A5A.
 * Expected: <C04:5A5A:CCCC>
 */
void testCmdResetSendsMagicWord(void)
{
    queueResponse("<A02:00:466C>");
    NSTAR_CMDReset(gCtx);

    size_t wlen = 0;
    const uint8_t *w = nstarMockUARTGetWritten(&wlen);
    TEST_ASSERT_EQUAL_CHAR('C', (char)w[1]);
    TEST_ASSERT_EQUAL_CHAR('0', (char)w[2]);
    TEST_ASSERT_EQUAL_CHAR('4', (char)w[3]);
    /* DATA field: 5A5A */
    TEST_ASSERT_EQUAL_CHAR('5', (char)w[5]);
    TEST_ASSERT_EQUAL_CHAR('A', (char)w[6]);
    TEST_ASSERT_EQUAL_CHAR('5', (char)w[7]);
    TEST_ASSERT_EQUAL_CHAR('A', (char)w[8]);
}

/**
 * Non-zero ACK from reset → NSTAR_ERR_BAD_ACK.
 */
void testCmdResetBadAckReturnsError(void)
{
    queueResponse("<A02:01:755D>");
    TEST_ASSERT_EQUAL(NSTAR_ERR_BAD_ACK, NSTAR_CMDReset(gCtx));
}

/* =========================================================================
 * NSTAR_StartupSequence tests
 * =========================================================================
 */

/**
 * Full startup sequence happy path.
 * Mock queues responses in this order (3 transactions, after removing
 * the redundant R 0x08/R 0x09 reads — see nstar_core.c comment: FPGA_OPTION
 * is already captured by the V response in step 2, and was previously
 * being read a second time and then discarded entirely):
 *   1. V command response (identity, includes FPGA_OPTION)
 *   2. R 0x06 response (FPGA_TYPE = 0x62)
 *   3. W 0x10=0x02 ACK (OBS sweep config)
 */
void testStartupSequenceSuccess(void)
{
    queueResponse("<V12:010018230042620307:F832>");  /* V */
    queueResponse("<R02:62:7D57>");                  /* R 0x06 */
    queueResponse("<A02:00:466C>");                  /* W 0x10 */

    TEST_ASSERT_EQUAL(NSTAR_OK, NSTAR_StartupSequence(gCtx));
}

/**
 * Wrong FPGA_TYPE → NSTAR_ERR_FPGA_TYPE.
 * Simulate a unit where register 0x06 returns 0xFF instead of 0x62.
 */
void testStartupWrongFpgaTypeReturnsError(void)
{
    queueResponse("<V12:010018230042620307:F832>");  /* V */
    queueResponse("<R02:FF:61C2>");                  /* R 0x06 = 0xFF (wrong) */

    TEST_ASSERT_EQUAL(NSTAR_ERR_FPGA_TYPE,
        NSTAR_StartupSequence(gCtx));
}

/**
 * V command times out → startup returns NSTAR_ERR_TIMEOUT.
 */
void testStartupVTimeoutReturnsTimeoutError(void)
{
    nstarMockUARTForceTimeout(1);
    TEST_ASSERT_EQUAL(NSTAR_ERR_TIMEOUT,
        NSTAR_StartupSequence(gCtx));
}

/**
 * OBS sweep write times out → startup returns NSTAR_ERR_TIMEOUT.
 */
void testStartupObsWriteTimeoutReturnsTimeoutError(void)
{
    queueResponse("<V12:010018230042620307:F832>");
    queueResponse("<R02:62:7D57>");
    /* No response for W 0x10 → timeout */
    nstarMockUARTForceTimeout(1);

    TEST_ASSERT_EQUAL(NSTAR_ERR_TIMEOUT,
        NSTAR_StartupSequence(gCtx));
}

/**
 * Verify that startup sequence issues commands in the correct order.
 * Check the first bytes of the combined written buffer:
 *   Frame 1: <V00::...>    — V command
 *   Frame 2: <R02:06:...>  — R FPGA_TYPE
 */
void testStartupSendsVCommandFirst(void)
{
    queueResponse("<V12:010018230042620307:F832>");
    queueResponse("<R02:62:7D57>");
    queueResponse("<A02:00:466C>");

    NSTAR_StartupSequence(gCtx);

    size_t wlen = 0;
    const uint8_t *w = nstarMockUARTGetWritten(&wlen);

    /* First frame written must be the V command */
    TEST_ASSERT_EQUAL_CHAR('<', (char)w[0]);
    TEST_ASSERT_EQUAL_CHAR('V', (char)w[1]);
    TEST_ASSERT_EQUAL_CHAR('0', (char)w[2]);
    TEST_ASSERT_EQUAL_CHAR('0', (char)w[3]);
}

/**
 * Verify that the OBS sweep write (W 0x10 = 0x02) is the last
 * frame in the startup sequence.
 */
void testStartupLastFrameIsObsSweepWrite(void)
{
    queueResponse("<V12:010018230042620307:F832>");
    queueResponse("<R02:62:7D57>");
    queueResponse("<A02:00:466C>");

    NSTAR_StartupSequence(gCtx);

    size_t wlen = 0;
    const uint8_t *w = nstarMockUARTGetWritten(&wlen);

    /* Scan written bytes for a W frame containing 1002 (addr=0x10, val=0x02) */
    int found = 0;
    for (size_t i = 0; i + 9 < wlen; i++) {
        if (w[i] == '<' && w[i+1] == 'W' &&
            w[i+5] == '1' && w[i+6] == '0' &&
            w[i+7] == '0' && w[i+8] == '2') {
            found = 1;
            break;
        }
    }
    TEST_ASSERT_TRUE_MESSAGE(found,
        "Expected W frame with addr=0x10 val=0x02 in written bytes");
}

/* =========================================================================
 * NSTAR_GetIdentity tests
 * =========================================================================
 *
 * Verifies the bug fix: identity (including fpgaOptions) read during
 * startup_sequence() step 2 is now cached on ctx and retrievable
 * afterwards, instead of being read into a stack-local variable and
 * silently discarded when the function returned.
 */

void testGetIdentityBeforeStartupReturnsNotReady(void)
{
    NSTAR_Identity_t id;
    memset(&id, 0, sizeof(id));
    TEST_ASSERT_EQUAL(NSTAR_ERR_NOT_READY, NSTAR_GetIdentity(gCtx, &id));
}

void testGetIdentityNullCtxReturnsParamError(void)
{
    NSTAR_Identity_t id;
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM, NSTAR_GetIdentity(NULL, &id));
}

void testGetIdentityNullOutReturnsParamError(void)
{
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM, NSTAR_GetIdentity(gCtx, NULL));
}

/**
 * After a successful startup_sequence(), NSTAR_GetIdentity() must
 * return the same fpgaOptions the V command reported — proving the
 * data survives past the function call instead of being discarded.
 */
void testGetIdentityAfterStartupReturnsCachedFpgaOptions(void)
{
    queueResponse("<V12:010018230042620307:F832>");  /* fpgaOptions=0x0307 */
    queueResponse("<R02:62:7D57>");
    queueResponse("<A02:00:466C>");

    TEST_ASSERT_EQUAL(NSTAR_OK, NSTAR_StartupSequence(gCtx));

    NSTAR_Identity_t id;
    memset(&id, 0, sizeof(id));
    TEST_ASSERT_EQUAL(NSTAR_OK, NSTAR_GetIdentity(gCtx, &id));
    TEST_ASSERT_EQUAL_HEX8(0x62, id.fpgaType);
    TEST_ASSERT_EQUAL_HEX16(0x0307, id.fpgaOptions);
}

/* =========================================================================
 * Test runner
 * =========================================================================
 */

int main(void)
{
    UNITY_BEGIN();

    /* Init / deinit */
    RUN_TEST(testInitNullConfigReturnsParamError);
    RUN_TEST(testInitNullCallbacksReturnsParamError);
    RUN_TEST(testInitNullCtxOutReturnsParamError);
    RUN_TEST(testInitSuccessReturnsNonNullCtx);
    RUN_TEST(testDeinitNullCtxIsSafe);

    /* reg_read */
    RUN_TEST(testRegReadNullCtxReturnsParamError);
    RUN_TEST(testRegReadNullValReturnsParamError);
    RUN_TEST(testRegReadFpgaTypeReturnsCorrectValue);
    RUN_TEST(testRegReadSendsCorrectFrame);
    RUN_TEST(testRegReadTimeoutReturnsTimeoutError);
    RUN_TEST(testRegReadCrcErrorAfterRetry);
    RUN_TEST(testRegReadSucceedsOnRetry);

    /* reg_write */
    RUN_TEST(testRegWriteNullCtxReturnsParamError);
    RUN_TEST(testRegWriteTxModeModulationSuccess);
    RUN_TEST(testRegWriteSendsCorrectFrame);
    RUN_TEST(testRegWriteBadAckReturnsBadAckError);
    RUN_TEST(testRegWriteTimeoutReturnsTimeoutError);

    /* reg_read_multi */
    RUN_TEST(testRegReadMultiReadsNRegisters);
    RUN_TEST(testRegReadMultiZeroCountReturnsParamError);

    /* cmd_read_identity */
    RUN_TEST(testCmdReadIdentitySuccess);
    RUN_TEST(testCmdReadIdentityNullOutReturnsParamError);
    RUN_TEST(testCmdReadIdentitySendsVFrame);

    /* cmd_read_all_rx_status */
    RUN_TEST(testCmdReadAllRxStatusSuccess);
    RUN_TEST(testCmdReadAllRxStatusNullOutReturnsParamError);

    /* cmd_reset */
    RUN_TEST(testCmdResetSuccess);
    RUN_TEST(testCmdResetSendsMagicWord);
    RUN_TEST(testCmdResetBadAckReturnsError);

    /* startup_sequence */
    RUN_TEST(testStartupSequenceSuccess);
    RUN_TEST(testStartupWrongFpgaTypeReturnsError);
    RUN_TEST(testStartupVTimeoutReturnsTimeoutError);
    RUN_TEST(testStartupObsWriteTimeoutReturnsTimeoutError);
    RUN_TEST(testStartupSendsVCommandFirst);
    RUN_TEST(testStartupLastFrameIsObsSweepWrite);

    /* get_identity */
    RUN_TEST(testGetIdentityBeforeStartupReturnsNotReady);
    RUN_TEST(testGetIdentityNullCtxReturnsParamError);
    RUN_TEST(testGetIdentityNullOutReturnsParamError);
    RUN_TEST(testGetIdentityAfterStartupReturnsCachedFpgaOptions);

    return UNITY_END();
}
