/**
 * @file  ttc_nstar.c
 * @brief N-STAR TTC transponder interface — core logic.
 *
 * Stage 2 covers:
 *   - Context lifecycle (init / deinit)
 *   - UART command encode → send → receive → decode pipeline
 *   - Register read / write / read_multi
 *   - Named commands: V (identity), E (all RX status), C (reset)
 *   - Startup sequence
 *
 * Threading model for Stage 2:
 *   All UART operations are synchronous (no command thread yet).
 *   nstarCMDQExecute() is called directly from the caller's thread.
 *   The mutex is present but uncontested at this stage.
 *   The command thread, RX thread, fault thread, and health thread
 *   are added in Stages 3-5 when the pipelines that need them are built.
 *
 * Stages 3-5 stubs remain at the bottom — compile-only, return NSTAR_OK.
 */

#include "ttc_nstar.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <pthread.h>

/* =========================================================================
 * Internal context
 * =========================================================================
 */

struct NSTAR_Ctx {
    NSTAR_Config_t    config;
    NSTAR_Callbacks_t callbacks;
    int               initialised;
    pthread_mutex_t   uartMutex;    /* serialises all uartFd access       */

    /* Module-level FSM */
    volatile NSTAR_ModuleState_t moduleState;
    pthread_mutex_t   stateMutex;   /* protects moduleState                */

    /* Cached identity, populated during startup_sequence() step 2 (V cmd).
     * fpgaOptions here is the authoritative copy — there is no separate
     * runtime read of registers 0x08/0x09, since the V command's response
     * already contains the same FPGA_OPTION bytes (IRD Annexe A: 0x08/0x09
     * IS FPGA_OPTION). Re-reading via R would be redundant. */
    NSTAR_Identity_t  identity;
    int               identityValid;

    /* TX state (Stage 3) */
    int               txActive;     /* 1 while TX_MODE=Modulation is set   */
    size_t            txBytesSent; /* cumulative bytes written this session*/

    /* Thread lifecycle (Stages 4-5) */
    volatile int      stopFlag;     /* set to 1 by NSTAR_Deinit()          */
    pthread_t         rxThread;
    pthread_t         faultThread;
    pthread_t         healthThread;
    int               threadsStarted;

    /* RX state (Stage 4) */
    volatile NSTAR_RXState_t rxState;
};

/* =========================================================================
 * Module-level FSM helpers
 * =========================================================================
 */

/** Internal: set moduleState under stateMutex. */
static void setModuleState(NSTAR_Ctx_t *ctx, NSTAR_ModuleState_t newState)
{
    pthread_mutex_lock(&ctx->stateMutex);
    ctx->moduleState = newState;
    pthread_mutex_unlock(&ctx->stateMutex);
}

/** Internal: read moduleState under stateMutex. */
static NSTAR_ModuleState_t getModuleState(NSTAR_Ctx_t *ctx)
{
    pthread_mutex_lock(&ctx->stateMutex);
    NSTAR_ModuleState_t s = ctx->moduleState;
    pthread_mutex_unlock(&ctx->stateMutex);
    return s;
}

NSTAR_ModuleState_t NSTAR_GetModuleState(NSTAR_Ctx_t *ctx)
{
    if (!ctx) return NSTAR_MODULE_UNINIT;
    return getModuleState(ctx);
}

NSTAR_RXState_t NSTAR_RXGetState(NSTAR_Ctx_t *ctx)
{
    if (!ctx) return NSTAR_RX_IDLE;
    return ctx->rxState;   /* volatile single-word read, no lock needed */
}

/**
 * Return the FPGA identity cached during the most recent successful
 * startup_sequence() (covers both initial init and post-fault recovery).
 * @return NSTAR_OK with *out populated, or NSTAR_ERR_NOT_READY if
 *         startup_sequence() has never completed successfully yet.
 */
NSTAR_Result_t NSTAR_GetIdentity(NSTAR_Ctx_t *ctx, NSTAR_Identity_t *out)
{
    if (!ctx || !out) return NSTAR_ERR_PARAM;
    if (!ctx->identityValid) return NSTAR_ERR_NOT_READY;
    *out = ctx->identity;
    return NSTAR_OK;
}

/* =========================================================================
 * Internal helpers
 * =========================================================================
 */

/**
 * Send one UART frame and receive the response.
 *
 * Encodes the frame, writes it to uartFd, then reads the response
 * with NSTAR_CMD_TIMEOUT_MS timeout.  Retries once on timeout or
 * CRC failure (NSTAR_CMD_MAX_RETRIES).
 *
 * @param ctx         Context handle (uartFd must be open).
 * @param cmdId      Command letter ('R','W','V','E','C').
 * @param dataIn     Raw payload bytes for the command (may be NULL).
 * @param dataLen    Number of raw bytes in dataIn.
 * @param respBuf    Buffer to receive the decoded response payload.
 * @param respLen    Set to the number of decoded bytes in respBuf.
 * @param respCmdId Set to the CMD_ID character in the response frame.
 * @return            NSTAR_OK, NSTAR_ERR_TIMEOUT, NSTAR_ERR_CRC,
 *                    NSTAR_ERR_BAD_FRAME, or NSTAR_ERR_HAL.
 */
/* Send a pre-encoded frame and read back one response frame. */
static NSTAR_Result_t cmdqSendReceive(NSTAR_Ctx_t *ctx,
                                      const uint8_t *txFrame, size_t txLen,
                                      uint8_t *rxFrame,
                                      uint8_t *respBuf, size_t *respLen,
                                      char *respCmdId)
{
    ssize_t written = nstarUARTWrite(ctx->config.uartFd, txFrame, txLen);
    if (written < 0 || (size_t)written != txLen) {
        return NSTAR_ERR_HAL;
    }
    ssize_t nread = nstarUARTRead(ctx->config.uartFd,
                                         rxFrame, NSTAR_FRAME_BUF_MAX,
                                         NSTAR_CMD_TIMEOUT_MS);
    if (nread <= 0) {
        return NSTAR_ERR_TIMEOUT;
    }
    return nstarFrameDecode(rxFrame, (size_t)nread,
                               respCmdId, respBuf, respLen);
}

/* Encode-send-receive-decode with retry on timeout or CRC error. */
static NSTAR_Result_t cmdqExecute(NSTAR_Ctx_t *ctx,
                                    char cmdId,
                                    const uint8_t *dataIn, size_t dataLen,
                                    uint8_t *respBuf, size_t *respLen,
                                    char *respCmdId)
{
    uint8_t txFrame[NSTAR_FRAME_BUF_MAX];
    uint8_t rxFrame[NSTAR_FRAME_BUF_MAX];
    size_t  txLen = 0;

    NSTAR_Result_t rc = nstarFrameEncode(cmdId, dataIn, dataLen,
                                            txFrame, &txLen);
    if (rc != NSTAR_OK) return rc;

    int attempts = 0;
    do {
        attempts++;
        rc = cmdqSendReceive(ctx, txFrame, txLen, rxFrame,
                              respBuf, respLen, respCmdId);
        if (rc == NSTAR_ERR_TIMEOUT || rc == NSTAR_ERR_CRC) {
            if (attempts <= (int)NSTAR_CMD_MAX_RETRIES) continue;
        }
        return rc;
    } while (attempts <= (int)NSTAR_CMD_MAX_RETRIES);
    return NSTAR_ERR_TIMEOUT;
}

/**
 * Issue a command and validate the response CMD_ID matches expectedRespId.
 * Convenience wrapper over cmdqExecute().
 */
static NSTAR_Result_t cmdTransact(NSTAR_Ctx_t *ctx,
                                    char cmdId,
                                    const uint8_t *dataIn, size_t dataLen,
                                    char expectedRespId,
                                    uint8_t *respData, size_t *respDataLen)
{
    char   respCmdId = 0;
    size_t respLen    = 0;
    uint8_t localBuf[NSTAR_FRAME_BUF_MAX / 2];

    uint8_t *outBuf = respData  ? respData  : localBuf;
    size_t  *outLen = respDataLen ? respDataLen : &respLen;

    pthread_mutex_lock(&ctx->uartMutex);
    NSTAR_Result_t rc = cmdqExecute(ctx, cmdId, dataIn, dataLen,
                                      outBuf, outLen, &respCmdId);
    pthread_mutex_unlock(&ctx->uartMutex);

    if (rc != NSTAR_OK) return rc;

    /* Response CMD_ID validation:
     * N-STAR echoes 'R' for read responses, 'A' for write ACKs,
     * 'V' for identity, 'E' for all-status. Accept either the echo
     * or 'A' (ACK) for write commands. */
    if (expectedRespId != 0 &&
        respCmdId != expectedRespId &&
        respCmdId != 'A') {
        return NSTAR_ERR_BAD_FRAME;
    }

    return NSTAR_OK;
}

/* =========================================================================
 * Lifecycle
 * =========================================================================
 */

/* Forward declarations for thread functions */
static void *rxThreadFunc(void *arg);
static void *faultThreadFunc(void *arg);
static void *healthThreadFunc(void *arg);

NSTAR_Result_t NSTAR_Init(const NSTAR_Config_t *config,
                           const NSTAR_Callbacks_t *callbacks,
                           NSTAR_Ctx_t **ctxOut)
{
    if (!config || !callbacks || !ctxOut) return NSTAR_ERR_PARAM;

    NSTAR_Ctx_t *ctx = calloc(1, sizeof(*ctx));
    if (!ctx) return NSTAR_ERR_HAL;

    ctx->config       = *config;
    ctx->callbacks    = *callbacks;
    ctx->stopFlag    = 0;
    ctx->rxState     = NSTAR_RX_IDLE;
    ctx->moduleState = NSTAR_MODULE_UNINIT;

    if (pthread_mutex_init(&ctx->uartMutex, NULL) != 0) {
        free(ctx);
        return NSTAR_ERR_HAL;
    }

    if (pthread_mutex_init(&ctx->stateMutex, NULL) != 0) {
        pthread_mutex_destroy(&ctx->uartMutex);
        free(ctx);
        return NSTAR_ERR_HAL;
    }

    setModuleState(ctx, NSTAR_MODULE_INITIALISING);

    ctx->initialised = 1;

    /* Spawn background threads */
    if (pthread_create(&ctx->rxThread,     NULL, rxThreadFunc,     ctx) != 0 ||
        pthread_create(&ctx->faultThread,  NULL, faultThreadFunc,  ctx) != 0 ||
        pthread_create(&ctx->healthThread, NULL, healthThreadFunc, ctx) != 0) {
        ctx->stopFlag = 1;
        pthread_mutex_destroy(&ctx->stateMutex);
        pthread_mutex_destroy(&ctx->uartMutex);
        free(ctx);
        return NSTAR_ERR_HAL;
    }

    ctx->threadsStarted = 1;

    /*
     * Threads are running but startup_sequence() has not been called yet.
     * The caller (or the application's init flow) is expected to call
     * NSTAR_StartupSequence() next, which will transition STARTING->READY.
     */
    setModuleState(ctx, NSTAR_MODULE_STARTING);

    *ctxOut = ctx;
    return NSTAR_OK;
}

void NSTAR_Deinit(NSTAR_Ctx_t *ctx)
{
    if (!ctx) return;

    setModuleState(ctx, NSTAR_MODULE_SHUTTING_DOWN);

    /* Signal all threads to stop */
    ctx->stopFlag = 1;

    if (ctx->threadsStarted) {
        pthread_join(ctx->rxThread,     NULL);
        pthread_join(ctx->faultThread,  NULL);
        pthread_join(ctx->healthThread, NULL);
    }

    pthread_mutex_destroy(&ctx->uartMutex);
    pthread_mutex_destroy(&ctx->stateMutex);
    free(ctx);
}

/* =========================================================================
 * Startup sequence
 * =========================================================================
 *
 * Step 1: Wait NSTAR_POWERUP_WAIT_MS for oscillator (User Manual §3.1)
 * Step 2: V command — read FPGA identity (version, build, HW serial,
 *         FPGA_TYPE, FPGA_OPTION). Cached on ctx->identity; retrievable
 *         afterwards via NSTAR_GetIdentity(). FPGA_OPTION here is the
 *         ONLY read of that data — registers 0x08/0x09 are not re-read
 *         separately, since the V response already contains identical
 *         bytes (IRD Annexe A: 0x08/0x09 IS FPGA_OPTION). An earlier
 *         version of this function re-read 0x08/0x09 via R commands and
 *         then discarded the result entirely without caching either
 *         copy — removed as both redundant and a real data-loss bug.
 * Step 3: R 0x06 — verify FPGA_TYPE == 0x62 (N-STAR PCM/PM, RX+TX)
 * Step 4: W 0x10 = 0x02 — configure 2-pass OBS sweep
 */
NSTAR_Result_t NSTAR_StartupSequence(NSTAR_Ctx_t *ctx)
{
    if (!ctx || !ctx->initialised) return NSTAR_ERR_NOT_INIT;

    /* Exempt from READY guard — this function is what creates READY. */
    setModuleState(ctx, NSTAR_MODULE_STARTING);
    nstarSleepMS(NSTAR_POWERUP_WAIT_MS);

    /* Step 2 — read and cache identity (V command contains FPGA_OPTION) */
    NSTAR_Identity_t identity;
    memset(&identity, 0, sizeof(identity));
    NSTAR_Result_t rc = NSTAR_CMDReadIdentity(ctx, &identity);
    if (rc != NSTAR_OK) { setModuleState(ctx, NSTAR_MODULE_FAULT); return rc; }
    ctx->identity      = identity;
    ctx->identityValid = 1;

    /* Step 3 — verify FPGA_TYPE */
    uint8_t fpgaType = 0;
    rc = NSTAR_RegRead(ctx, NSTAR_REG_FPGA_TYPE, &fpgaType);
    if (rc != NSTAR_OK) { setModuleState(ctx, NSTAR_MODULE_FAULT); return rc; }
    if (fpgaType != NSTAR_FPGA_TYPE_EXPECTED) {
        setModuleState(ctx, NSTAR_MODULE_FAULT);
        return NSTAR_ERR_FPGA_TYPE;
    }

    /* Step 4 — configure 2-pass OBS sweep */
    rc = NSTAR_RegWrite(ctx, NSTAR_REG_RX_NB_SWEEP, NSTAR_RX_SWEEP_2PASS);
    if (rc != NSTAR_OK) { setModuleState(ctx, NSTAR_MODULE_FAULT); return rc; }

    setModuleState(ctx, NSTAR_MODULE_READY);
    return NSTAR_OK;
}

/* =========================================================================
 * Register access
 * =========================================================================
 */

NSTAR_Result_t NSTAR_RegRead(NSTAR_Ctx_t *ctx, uint8_t addr,
                               uint8_t *valOut)
{
    if (!ctx || !valOut) return NSTAR_ERR_PARAM;

    /*
     * R command: DATA = address byte only.
     * N-STAR responds with the register value as 1 byte.
     */
    uint8_t resp[NSTAR_FRAME_BUF_MAX / 2];
    size_t  respLen = 0;

    NSTAR_Result_t rc = cmdTransact(ctx, 'R', &addr, 1,
                                      'R', resp, &respLen);
    if (rc != NSTAR_OK) return rc;
    if (respLen < 1) return NSTAR_ERR_BAD_FRAME;

    *valOut = resp[0];
    return NSTAR_OK;
}

NSTAR_Result_t NSTAR_RegWrite(NSTAR_Ctx_t *ctx, uint8_t addr, uint8_t val)
{
    if (!ctx) return NSTAR_ERR_PARAM;

    /*
     * W command: DATA = address byte + value byte.
     * N-STAR responds with an ACK frame containing result code (0 = OK).
     */
    uint8_t payload[2] = { addr, val };
    uint8_t resp[NSTAR_FRAME_BUF_MAX / 2];
    size_t  respLen = 0;

    NSTAR_Result_t rc = cmdTransact(ctx, 'W', payload, 2,
                                      'A', resp, &respLen);
    if (rc != NSTAR_OK) return rc;

    /* ACK payload: 1 byte result code. 0x00 = success. */
    if (respLen >= 1 && resp[0] != 0x00) return NSTAR_ERR_BAD_ACK;

    return NSTAR_OK;
}

NSTAR_Result_t NSTAR_RegReadMulti(NSTAR_Ctx_t *ctx, uint8_t startAddr,
                                     uint8_t n, uint8_t *bufOut)
{
    if (!ctx || !bufOut || n == 0) return NSTAR_ERR_PARAM;

    /* Issue n individual R commands and collect results. */
    for (uint8_t i = 0; i < n; i++) {
        NSTAR_Result_t rc = NSTAR_RegRead(ctx,
                                            (uint8_t)(startAddr + i),
                                            &bufOut[i]);
        if (rc != NSTAR_OK) return rc;
    }
    return NSTAR_OK;
}

/* =========================================================================
 * Named commands
 * =========================================================================
 */

NSTAR_Result_t NSTAR_CMDReadIdentity(NSTAR_Ctx_t *ctx,
                                        NSTAR_Identity_t *out)
{
    if (!ctx || !out) return NSTAR_ERR_PARAM;

    /*
     * V command: no data payload.
     * Response: up to 12 bytes —
     *   [0]    FPGA_VERSION
     *   [1]    FPGA_BUILD
     *   [2]    HW_ID_YEAR
     *   [3]    HW_ID_WEEK
     *   [4-5]  HW_ID_ORDER (MSB first)
     *   [6]    FPGA_TYPE
     *   [7-8]  FPGA_OPTION (MSB first)
     *   [9-11] reserved
     */
    uint8_t resp[NSTAR_FRAME_BUF_MAX / 2];
    size_t  respLen = 0;

    NSTAR_Result_t rc = cmdTransact(ctx, 'V', NULL, 0,
                                      'V', resp, &respLen);
    if (rc != NSTAR_OK) return rc;
    if (respLen < 6) return NSTAR_ERR_BAD_FRAME;

    memset(out, 0, sizeof(*out));
    out->fpgaVersion = resp[0];
    out->fpgaBuild   = resp[1];
    out->hwYear      = resp[2];
    out->hwWeek      = resp[3];
    if (respLen >= 6) {
        out->hwOrder = (uint16_t)((resp[4] << 8) | resp[5]);
    }
    if (respLen >= 7) {
        out->fpgaType = resp[6];
    }
    if (respLen >= 9) {
        out->fpgaOptions = (uint16_t)((resp[7] << 8) | resp[8]);
    }
    return NSTAR_OK;
}

NSTAR_Result_t NSTAR_CMDReadAllRXStatus(NSTAR_Ctx_t *ctx,
                                             uint8_t *rawOut,
                                             size_t  *lenOut)
{
    if (!ctx || !rawOut || !lenOut) return NSTAR_ERR_PARAM;

    /*
     * E command: no data payload.
     * Response: registers 0x10-0x22 in one frame (19 bytes).
     */
    char   respCmdId = 0;
    size_t respLen    = 0;

    pthread_mutex_lock(&ctx->uartMutex);
    NSTAR_Result_t rc = cmdqExecute(ctx, 'E', NULL, 0,
                                      rawOut, &respLen, &respCmdId);
    pthread_mutex_unlock(&ctx->uartMutex);

    if (rc != NSTAR_OK) return rc;
    *lenOut = respLen;
    return NSTAR_OK;
}

NSTAR_Result_t NSTAR_CMDReset(NSTAR_Ctx_t *ctx)
{
    if (!ctx) return NSTAR_ERR_PARAM;

    /*
     * C command: DATA = magic word 0x5A5A (2 bytes).
     * N-STAR resets all registers to manufacture defaults.
     * Response: ACK with result code 0x00.
     */
    uint8_t magic[2] = { (uint8_t)(NSTAR_RESET_MAGIC >> 8),
                          (uint8_t)(NSTAR_RESET_MAGIC & 0xFF) };
    uint8_t resp[NSTAR_FRAME_BUF_MAX / 2];
    size_t  respLen = 0;

    NSTAR_Result_t rc = cmdTransact(ctx, 'C', magic, 2,
                                      'A', resp, &respLen);
    if (rc != NSTAR_OK) return rc;
    if (respLen >= 1 && resp[0] != 0x00) return NSTAR_ERR_BAD_ACK;
    return NSTAR_OK;
}

/* =========================================================================
 * TX pipeline (Stage 3)
 * =========================================================================
 *
 * Sequence (from flow diagram and EICD §5.11.1):
 *
 *   NSTAR_TXStart():
 *     1. Write TX data rate register (W 0x22 = rateCode)
 *     2. Assert TX clock via nstarDataClockStart()
 *     3. Wait NSTAR_TX_CLOCK_PRESTABLE_MS for clock to stabilise
 *     4. Read TX_STATUS (R 0x40) — verify b4 (clock detected) = 1
 *     5. Write TX_MODE = Modulation (W 0x40 = 0x01)
 *
 *   NSTAR_TXWrite():
 *     - Write pre-framed application data to data interface in chunks
 *       of NSTAR_FRAME_SIZE_BYTES.  Accumulates total bytes sent.
 *
 *   NSTAR_TXStop():
 *     1. Write TX_MODE = Standby (W 0x40 = 0x00)
 *     2. Stop TX clock via nstarDataClockStop()
 *     3. Fire onTXComplete callback with total bytes sent
 *     4. Reset txActive flag and byte counter
 *
 *   NSTAR_TXGetStatus():
 *     - Read TX_STATUS register, decode mode and clock-detected bit.
 *
 * No background thread: TX is driven synchronously by the application.
 * The data interface write (nstarDataWrite) is blocking.
 * Clock gap constraint (EICD §5.11.1): if the clock is absent for
 * >1 ms during active modulation, N-STAR enters standby automatically.
 * The application is responsible for keeping NSTAR_TXWrite() calls
 * frequent enough to avoid gaps.
 */

/* Assert clock, wait for stabilisation, verify N-STAR detects it.
 * Stops clock and returns error if any step fails. */
static NSTAR_Result_t txAssertAndVerifyClock(NSTAR_Ctx_t *ctx)
{
    NSTAR_Result_t rc = nstarDataClockStart(ctx->config.dataFd);
    if (rc != NSTAR_OK) return rc;

    nstarSleepMS(NSTAR_TX_CLOCK_PRESTABLE_MS);

    NSTAR_TXStatus_t status;
    memset(&status, 0, sizeof(status));
    rc = NSTAR_TXGetStatus(ctx, &status);
    if (rc != NSTAR_OK || !status.clockDetected) {
        nstarDataClockStop(ctx->config.dataFd);
        return (rc != NSTAR_OK) ? rc : NSTAR_ERR_NO_CLOCK;
    }
    return NSTAR_OK;
}

NSTAR_Result_t NSTAR_TXStart(NSTAR_Ctx_t *ctx,
                               NSTAR_TXRateCode_t rateCode)
{
    if (!ctx || !ctx->initialised) return NSTAR_ERR_NOT_INIT;
    if (getModuleState(ctx) != NSTAR_MODULE_READY) return NSTAR_ERR_NOT_READY;
    if (ctx->txActive) return NSTAR_ERR_BUSY;

    NSTAR_Result_t rc = NSTAR_RegWrite(ctx, NSTAR_REG_RX_DATA_RATE,
                                         (uint8_t)rateCode);
    if (rc != NSTAR_OK) return rc;

    rc = txAssertAndVerifyClock(ctx);
    if (rc != NSTAR_OK) return rc;

    rc = NSTAR_RegWrite(ctx, NSTAR_REG_TX_MODE, NSTAR_TX_MODE_MODULATION);
    if (rc != NSTAR_OK) {
        nstarDataClockStop(ctx->config.dataFd);
        return rc;
    }

    ctx->txActive    = 1;
    ctx->txBytesSent = 0;
    return NSTAR_OK;
}

NSTAR_Result_t NSTAR_TXWrite(NSTAR_Ctx_t *ctx,
                               const uint8_t *buf, size_t len)
{
    if (!ctx || !ctx->initialised) return NSTAR_ERR_NOT_INIT;
    if (!buf || len == 0)          return NSTAR_ERR_PARAM;
    if (!ctx->txActive)           return NSTAR_ERR_BUSY;

    /*
     * Write in NSTAR_FRAME_SIZE_BYTES chunks.
     * Each call to nstarDataWrite is blocking.
     * The caller must not allow gaps between calls exceeding
     * NSTAR_TX_CLOCK_GAP_MAX_MS (1 ms) or N-STAR will enter standby.
     */
    size_t remaining = len;
    const uint8_t *ptr = buf;

    while (remaining > 0) {
        size_t chunk = (remaining > NSTAR_FRAME_SIZE_BYTES)
                       ? NSTAR_FRAME_SIZE_BYTES
                       : remaining;

        ssize_t written = nstarDataWrite(ctx->config.dataFd,
                                                ptr, chunk);
        if (written < 0) return NSTAR_ERR_HAL;

        ptr                 += (size_t)written;
        remaining           -= (size_t)written;
        ctx->txBytesSent  += (size_t)written;
    }

    return NSTAR_OK;
}

NSTAR_Result_t NSTAR_TXStop(NSTAR_Ctx_t *ctx)
{
    if (!ctx || !ctx->initialised) return NSTAR_ERR_NOT_INIT;
    if (!ctx->txActive)           return NSTAR_OK;  /* idempotent */

    /* Step 1: Enter standby */
    NSTAR_RegWrite(ctx, NSTAR_REG_TX_MODE, NSTAR_TX_MODE_STANDBY);

    /* Step 2: Stop clock */
    nstarDataClockStop(ctx->config.dataFd);

    /* Step 3: Notify application */
    size_t bytes = ctx->txBytesSent;
    ctx->txActive     = 0;
    ctx->txBytesSent = 0;

    if (ctx->callbacks.onTXComplete) {
        ctx->callbacks.onTXComplete(bytes);
    }

    return NSTAR_OK;
}

NSTAR_Result_t NSTAR_TXGetStatus(NSTAR_Ctx_t *ctx,
                                    NSTAR_TXStatus_t *out)
{
    if (!ctx || !out) return NSTAR_ERR_PARAM;

    uint8_t raw = 0;
    NSTAR_Result_t rc = NSTAR_RegRead(ctx, NSTAR_REG_TX_MODE, &raw);
    if (rc != NSTAR_OK) return rc;

    memset(out, 0, sizeof(*out));
    out->raw            = raw;
    out->currentMode   = (NSTAR_TXMode_t)(raw & NSTAR_TX_STATUS_MODE_MASK);
    out->clockDetected = (raw & NSTAR_TX_STATUS_CLOCK_DETECTED) != 0;
    out->configSet     = (raw >> 2) & 0x03U;
    return NSTAR_OK;
}

/* =========================================================================
 * RX pipeline (Stage 4)
 * =========================================================================
 *
 * NSTAR_RXConfigure():
 *   Writes RX data rate to register 0x22. Call before contact window.
 *   Requires OPT-C31-RDF (ordered on Lumos EM).
 *
 * NSTAR_RXGetStatus():
 *   Reads RX_STATUS register (R 0x10), decodes four lock bits.
 *
 * NSTAR_RXGetLinkQuality():
 *   Reads Eb/N0, RSSI, and Doppler frequency shift registers.
 *   Only meaningful when rxState == NSTAR_RX_LOCKED.
 *
 * rxThreadFunc():
 *   State machine driven by GPIO interrupts:
 *
 *   IDLE ──[LOCK_DETECT rising]──► ACQUIRING
 *     └─[timeout]──────────────────► IDLE (log timeout)
 *
 *   ACQUIRING ──[DATA_VALID rising]──► LOCKED
 *     └─[timeout / LOCK_DETECT falls]─► IDLE
 *
 *   LOCKED ──[DATA_VALID HIGH, data available]──► read frame → callback
 *     └─[DATA_VALID LOW]──────────────────────────► LOCK_LOST
 *
 *   LOCK_LOST ──[cleanup done]──► IDLE
 *
 * The RX thread is the ONLY consumer of dataFd reads.
 * UART diagnostic commands (NSTAR_RXGetStatus, NSTAR_RXGetLinkQuality)
 * must NOT be called from within the RX thread — they use uartMutex.
 */

NSTAR_Result_t NSTAR_RXConfigure(NSTAR_Ctx_t *ctx,
                                   NSTAR_RXRateCode_t rateCode)
{
    if (!ctx || !ctx->initialised) return NSTAR_ERR_NOT_INIT;
    if (getModuleState(ctx) != NSTAR_MODULE_READY) return NSTAR_ERR_NOT_READY;

    /*
     * RX data rate is stored in the same register as TX (0x22).
     * OPT-C31-RDF must be ordered for this to be configurable.
     */
    return NSTAR_RegWrite(ctx, NSTAR_REG_RX_DATA_RATE, (uint8_t)rateCode);
}

NSTAR_Result_t NSTAR_RXGetStatus(NSTAR_Ctx_t *ctx,
                                    NSTAR_RXStatus_t *out)
{
    if (!ctx || !out) return NSTAR_ERR_PARAM;

    uint8_t raw = 0;
    NSTAR_Result_t rc = NSTAR_RegRead(ctx, NSTAR_REG_RX_STATUS, &raw);
    if (rc != NSTAR_OK) return rc;

    memset(out, 0, sizeof(*out));
    out->raw            = raw;
    out->carrierDetect = (raw & NSTAR_RX_STATUS_CARRIER_DETECT) != 0;
    out->carrierLock   = (raw & NSTAR_RX_STATUS_CARRIER_LOCK)   != 0;
    out->bitLock       = (raw & NSTAR_RX_STATUS_BIT_LOCK)        != 0;
    out->dataValid     = (raw & NSTAR_RX_STATUS_DATA_VALID)      != 0;
    out->sweepState    = (raw >> 5) & 0x03U;
    return NSTAR_OK;
}

NSTAR_Result_t NSTAR_RXGetLinkQuality(NSTAR_Ctx_t *ctx,
                                          NSTAR_LinkQuality_t *out)
{
    if (!ctx || !out) return NSTAR_ERR_PARAM;

    NSTAR_Result_t rc;
    uint8_t buf[3];
    memset(out, 0, sizeof(*out));

    /*
     * Eb/N0: Eb = R 0x1C-0x1E (24-bit, MSB first)
     *        N0 = R 0x1F-0x21 (24-bit, MSB first)
     * Eb/N0 (dB) = 20 × log10(Eb_raw / N0_raw) − 3
     * (IRD Annex C)
     */
    rc = NSTAR_RegReadMulti(ctx, NSTAR_REG_RX_DEMOD_EB_MSB, 3, buf);
    if (rc != NSTAR_OK) return rc;
    uint32_t ebRaw = ((uint32_t)buf[0] << 16) |
                      ((uint32_t)buf[1] <<  8) |
                       (uint32_t)buf[2];

    rc = NSTAR_RegReadMulti(ctx, NSTAR_REG_RX_DEMOD_N0_MSB, 3, buf);
    if (rc != NSTAR_OK) return rc;
    uint32_t n0Raw = ((uint32_t)buf[0] << 16) |
                      ((uint32_t)buf[1] <<  8) |
                       (uint32_t)buf[2];

    if (ebRaw > 0 && n0Raw > 0) {
        out->ebNoDB = 20.0f * log10f((float)ebRaw / (float)n0Raw) - 3.0f;
    }

    /*
     * RSSI: IQ_POWER = R 0x17-0x18 (16-bit MSB first) = RX signal power
     *       AGC      = R 0x19-0x1A (16-bit MSB first) = RX gain
     * RSSI (dBm) = RX_IQ_power − RX_GAIN_RF (IRD Annex C)
     */
    rc = NSTAR_RegReadMulti(ctx, NSTAR_REG_RX_IQ_POWER_MSB, 2, buf);
    if (rc != NSTAR_OK) return rc;
    float iqPower = (float)(((uint16_t)buf[0] << 8) | buf[1]);

    rc = NSTAR_RegReadMulti(ctx, NSTAR_REG_RX_AGC_MSB, 2, buf);
    if (rc != NSTAR_OK) return rc;
    float agcVal = (float)(((uint16_t)buf[0] << 8) | buf[1]);

    out->rssiDBM = iqPower - agcVal;

    /*
     * Frequency shift: R 0x14-0x16 (24-bit signed, MSB first)
     * Freq shift (Hz) = signed_24bit / 8  (IRD Annex C)
     */
    rc = NSTAR_RegReadMulti(ctx, NSTAR_REG_RX_FREQ_SHIFT_MSB, 3, buf);
    if (rc != NSTAR_OK) return rc;
    uint32_t raw24 = ((uint32_t)buf[0] << 16) |
                     ((uint32_t)buf[1] <<  8) |
                      (uint32_t)buf[2];
    /* Sign-extend 24-bit to 32-bit */
    int32_t signed24 = (raw24 & 0x800000U)
                       ? (int32_t)(raw24 | 0xFF000000U)
                       : (int32_t)raw24;
    out->freqShiftHz = signed24 / 8;

    return NSTAR_OK;
}

/* -------------------------------------------------------------------------
 * RX thread
 * ------------------------------------------------------------------------- */

/* Inner loop executed while RX is LOCKED. Reads frames until DATA_VALID drops. */
static void rxHandleLockedState(NSTAR_Ctx_t *ctx)
{
    uint8_t frameBuf[NSTAR_FRAME_SIZE_BYTES];
    while (!ctx->stopFlag) {
        int dv = nstarGPIORead(ctx->config.gpioDataValid);
        if (dv != 1) break;

        ssize_t nread = nstarDataRead(ctx->config.dataFd,
                                              frameBuf, sizeof(frameBuf));
        if (nread <= 0) {
            dv = nstarGPIORead(ctx->config.gpioDataValid);
            if (dv != 1) break;
            nstarSleepMS(1);
            continue;
        }
        if (ctx->callbacks.onFrameReceived) {
            ctx->callbacks.onFrameReceived(frameBuf, (size_t)nread);
        }
    }
}

/* Attempt to acquire signal lock from IDLE state. Returns 1 if LOCKED. */
static int rxAcquireLock(NSTAR_Ctx_t *ctx)
{
    ctx->rxState = NSTAR_RX_IDLE;
    NSTAR_Result_t rc = nstarGPIOWaitEdge(ctx->config.gpioLockDetect,
                                                  NSTAR_GPIO_EDGE_RISING, 500);
    if (ctx->stopFlag || rc == NSTAR_ERR_TIMEOUT || rc != NSTAR_OK) return 0;

    ctx->rxState = NSTAR_RX_ACQUIRING;
    if (ctx->callbacks.onLockAcquired) ctx->callbacks.onLockAcquired();

    rc = nstarGPIOWaitEdge(ctx->config.gpioDataValid,
                                    NSTAR_GPIO_EDGE_RISING, 3000);
    if (ctx->stopFlag || rc != NSTAR_OK) return 0;

    ctx->rxState = NSTAR_RX_LOCKED;
    return 1;
}

static void *rxThreadFunc(void *arg)
{
    NSTAR_Ctx_t *ctx = (NSTAR_Ctx_t *)arg;
    while (!ctx->stopFlag) {
        if (!rxAcquireLock(ctx)) continue;

        rxHandleLockedState(ctx);

        ctx->rxState = NSTAR_RX_LOCK_LOST;
        if (ctx->callbacks.onLockLost) ctx->callbacks.onLockLost();
    }
    return NULL;
}

/* =========================================================================
 * Health monitoring (Stage 5)
 * =========================================================================
 *
 * NSTAR_HealthRead():
 *   Reads PA and BB temperature ADC registers (two 16-bit values each,
 *   MSB first).  Converts via NSTAR_TEMP_FROM_ADC().
 *   Also reads FAULT_N GPIO to populate faultActive field.
 *
 * healthThreadFunc():
 *   Polls PA temperature every NSTAR_HEALTH_POLL_INTERVAL_MS.
 *   If T > NSTAR_PA_TEMP_WARN_CELSIUS: fires onFault(TEMPERATURE)
 *   and stops TX.
 *
 * faultThreadFunc():
 *   Blocks on FAULT_N RISING edge (fault onset — open-collector released,
 *   external pull-up drives line HIGH per User Manual §3.1.1 Figures 9 & 10).
 *   On fault:
 *     1. Stop TX if active.
 *     2. Wait FAULT_N FALLING edge (N-STAR auto-recovery, line returns LOW).
 *        Minimum fault duration guaranteed by hardware = 3 s.
 *     3. If not recovered within 5 s timeout: assert RESET_N (GPIO LOW 100 ms).
 *     4. Fire onFault(NSTAR_FAULT_SEL) callback.
 *     5. Re-run startup_sequence() to restore register configuration.
 */

/* Read a 2-byte MSB-first ADC register pair and convert to Celsius. */
static NSTAR_Result_t readADCTemp(NSTAR_Ctx_t *ctx, uint8_t msbAddr,
                                  uint16_t *rawOut, float *celsiusOut)
{
    uint8_t buf[2];
    NSTAR_Result_t rc = NSTAR_RegReadMulti(ctx, msbAddr, 2, buf);
    if (rc != NSTAR_OK) return rc;
    *rawOut     = (uint16_t)((buf[0] << 8) | buf[1]);
    *celsiusOut = NSTAR_TEMP_FROM_ADC(*rawOut);
    return NSTAR_OK;
}

NSTAR_Result_t NSTAR_HealthRead(NSTAR_Ctx_t *ctx, NSTAR_Health_t *out)
{
    if (!ctx || !out) return NSTAR_ERR_PARAM;
    memset(out, 0, sizeof(*out));

    NSTAR_Result_t rc = readADCTemp(ctx, NSTAR_REG_ADC_PA_TEMP_MSB,
                                    &out->paAdcRaw, &out->paTempCelsius);
    if (rc != NSTAR_OK) return rc;

    rc = readADCTemp(ctx, NSTAR_REG_ADC_BB_TEMP_MSB,
                      &out->bbAdcRaw, &out->bbTempCelsius);
    if (rc != NSTAR_OK) return rc;

    int faultPin = nstarGPIORead(ctx->config.gpioFaultN);
    /* FAULT_N is HIGH when fault is active (open-collector released by N-STAR,
     * external 100kΩ pull-up raises line).  FAULT_N is LOW in normal operation. */
    out->faultActive = (faultPin == 1);
    return NSTAR_OK;
}

/* -------------------------------------------------------------------------
 * Health thread
 * ------------------------------------------------------------------------- */

static void *healthThreadFunc(void *arg)
{
    NSTAR_Ctx_t *ctx = (NSTAR_Ctx_t *)arg;

    while (!ctx->stopFlag) {
        nstarSleepMS(NSTAR_HEALTH_POLL_INTERVAL_MS);
        if (ctx->stopFlag) break;

        /* Only poll when READY — avoids UART contention during startup/recovery */
        if (getModuleState(ctx) != NSTAR_MODULE_READY) continue;

        NSTAR_Health_t health;
        NSTAR_Result_t rc = NSTAR_HealthRead(ctx, &health);
        if (rc != NSTAR_OK) continue;   /* UART busy or timeout — try next cycle */

        if (health.paTempCelsius > NSTAR_PA_TEMP_WARN_CELSIUS) {
            /* Proactively stop TX before N-STAR auto-stops at 90°C */
            if (ctx->txActive) {
                NSTAR_TXStop(ctx);
            }
            if (ctx->callbacks.onFault) {
                ctx->callbacks.onFault(NSTAR_FAULT_TEMPERATURE);
            }
        }
    }

    return NULL;
}

/* -------------------------------------------------------------------------
 * Fault thread
 * ------------------------------------------------------------------------- */

#define FAULT_RECOVERY_TIMEOUT_MS   5000U   /* wait for FAULT_N to clear */
#define FAULT_RESET_HOLD_MS          100U   /* RESET_N assert duration   */

/* Assert RESET_N low for FAULT_RESET_HOLD_MS, then wait for FAULT_N to clear.
 *
 * After releasing RESET_N, N-STAR performs a cold boot and de-asserts FAULT_N
 * (line returns LOW — open-collector pulled down).  We wait for FALLING edge.
 */
static void faultHandleHardReset(NSTAR_Ctx_t *ctx)
{
    nstarGPIOWrite(ctx->config.gpioResetN, 0);
    nstarSleepMS(FAULT_RESET_HOLD_MS);
    nstarGPIOWrite(ctx->config.gpioResetN, 1);
    /* Wait for FAULT_N to return LOW after cold boot */
    nstarGPIOWaitEdge(ctx->config.gpioFaultN,
                          NSTAR_GPIO_EDGE_FALLING,
                          FAULT_RECOVERY_TIMEOUT_MS);
}

/* Handle one complete fault event: stop TX, wait for recovery, notify, reinit.
 *
 * FAULT_N polarity (User Manual §3.1.1, Figures 9 & 10):
 *   FAULT_N goes HIGH when a SEL/fault fires (open-collector released, external
 *   pull-up raises the line).  It returns LOW when the fault has cleared.
 *   So after the thread detects the RISING edge (fault start), we wait here for
 *   the FALLING edge (fault cleared / N-STAR recovered).
 */
static void faultHandleEvent(NSTAR_Ctx_t *ctx)
{
    setModuleState(ctx, NSTAR_MODULE_FAULT);
    if (ctx->txActive) NSTAR_TXStop(ctx);

    /* Wait for FAULT_N to return LOW — N-STAR auto-recovery within 5 s */
    NSTAR_Result_t rc = nstarGPIOWaitEdge(ctx->config.gpioFaultN,
                                                 NSTAR_GPIO_EDGE_FALLING,
                                                 FAULT_RECOVERY_TIMEOUT_MS);
    if (rc != NSTAR_OK) faultHandleHardReset(ctx);

    if (ctx->callbacks.onFault) ctx->callbacks.onFault(NSTAR_FAULT_SEL);
    NSTAR_StartupSequence(ctx);
}

static void *faultThreadFunc(void *arg)
{
    NSTAR_Ctx_t *ctx = (NSTAR_Ctx_t *)arg;
    while (!ctx->stopFlag) {
        /* Wait for FAULT_N to go HIGH — fault onset (RISING edge) */
        NSTAR_Result_t rc = nstarGPIOWaitEdge(ctx->config.gpioFaultN,
                                                      NSTAR_GPIO_EDGE_RISING,
                                                      500);
        if (ctx->stopFlag) break;
        if (rc != NSTAR_OK) continue;
        faultHandleEvent(ctx);
    }
    return NULL;
}
