/**
 * @file  nstar_core.c
 * @brief N-STAR interface module — core implementation.
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
 *   nstar_cmdq_execute() is called directly from the caller's thread.
 *   The mutex is present but uncontested at this stage.
 *   The command thread, RX thread, fault thread, and health thread
 *   are added in Stages 3-5 when the pipelines that need them are built.
 *
 * Stages 3-5 stubs remain at the bottom — compile-only, return NSTAR_OK.
 */

#include "nstar.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <pthread.h>

/* =========================================================================
 * Internal context
 * =========================================================================
 */

struct nstar_ctx {
    nstar_config_t    config;
    nstar_callbacks_t callbacks;
    int               initialised;
    pthread_mutex_t   uart_mutex;    /* serialises all uart_fd access       */

    /* Module-level FSM */
    volatile nstar_module_state_t module_state;
    pthread_mutex_t   state_mutex;   /* protects module_state                */

    /* Cached identity, populated during startup_sequence() step 2 (V cmd).
     * fpga_options here is the authoritative copy — there is no separate
     * runtime read of registers 0x08/0x09, since the V command's response
     * already contains the same FPGA_OPTION bytes (IRD Annexe A: 0x08/0x09
     * IS FPGA_OPTION). Re-reading via R would be redundant. */
    nstar_identity_t  identity;
    int               identity_valid;

    /* TX state (Stage 3) */
    int               tx_active;     /* 1 while TX_MODE=Modulation is set   */
    size_t            tx_bytes_sent; /* cumulative bytes written this session*/

    /* Thread lifecycle (Stages 4-5) */
    volatile int      stop_flag;     /* set to 1 by nstar_deinit()          */
    pthread_t         rx_thread;
    pthread_t         fault_thread;
    pthread_t         health_thread;
    int               threads_started;

    /* RX state (Stage 4) */
    volatile nstar_rx_state_t rx_state;
};

/* =========================================================================
 * Module-level FSM helpers
 * =========================================================================
 */

/** Internal: set module_state under state_mutex. */
static void set_module_state(nstar_ctx_t *ctx, nstar_module_state_t new_state)
{
    pthread_mutex_lock(&ctx->state_mutex);
    ctx->module_state = new_state;
    pthread_mutex_unlock(&ctx->state_mutex);
}

/** Internal: read module_state under state_mutex. */
static nstar_module_state_t get_module_state(nstar_ctx_t *ctx)
{
    pthread_mutex_lock(&ctx->state_mutex);
    nstar_module_state_t s = ctx->module_state;
    pthread_mutex_unlock(&ctx->state_mutex);
    return s;
}

nstar_module_state_t nstar_get_module_state(nstar_ctx_t *ctx)
{
    if (!ctx) return NSTAR_MODULE_UNINIT;
    return get_module_state(ctx);
}

nstar_rx_state_t nstar_rx_get_state(nstar_ctx_t *ctx)
{
    if (!ctx) return NSTAR_RX_IDLE;
    return ctx->rx_state;   /* volatile single-word read, no lock needed */
}

/**
 * Return the FPGA identity cached during the most recent successful
 * startup_sequence() (covers both initial init and post-fault recovery).
 * @return NSTAR_OK with *out populated, or NSTAR_ERR_NOT_READY if
 *         startup_sequence() has never completed successfully yet.
 */
nstar_result_t nstar_get_identity(nstar_ctx_t *ctx, nstar_identity_t *out)
{
    if (!ctx || !out) return NSTAR_ERR_PARAM;
    if (!ctx->identity_valid) return NSTAR_ERR_NOT_READY;
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
 * Encodes the frame, writes it to uart_fd, then reads the response
 * with NSTAR_CMD_TIMEOUT_MS timeout.  Retries once on timeout or
 * CRC failure (NSTAR_CMD_MAX_RETRIES).
 *
 * @param ctx         Context handle (uart_fd must be open).
 * @param cmd_id      Command letter ('R','W','V','E','C').
 * @param data_in     Raw payload bytes for the command (may be NULL).
 * @param data_len    Number of raw bytes in data_in.
 * @param resp_buf    Buffer to receive the decoded response payload.
 * @param resp_len    Set to the number of decoded bytes in resp_buf.
 * @param resp_cmd_id Set to the CMD_ID character in the response frame.
 * @return            NSTAR_OK, NSTAR_ERR_TIMEOUT, NSTAR_ERR_CRC,
 *                    NSTAR_ERR_BAD_FRAME, or NSTAR_ERR_HAL.
 */
static nstar_result_t cmdq_execute(nstar_ctx_t *ctx,
                                    char cmd_id,
                                    const uint8_t *data_in, size_t data_len,
                                    uint8_t *resp_buf, size_t *resp_len,
                                    char *resp_cmd_id)
{
    uint8_t tx_frame[NSTAR_FRAME_BUF_MAX];
    uint8_t rx_frame[NSTAR_FRAME_BUF_MAX];
    size_t  tx_len = 0;

    /* Encode command frame */
    nstar_result_t rc = nstar_frame_encode(cmd_id, data_in, data_len,
                                            tx_frame, &tx_len);
    if (rc != NSTAR_OK) return rc;

    int attempts = 0;

    do {
        attempts++;

        /* Write frame to UART */
        ssize_t written = nstar_hal_uart_write(ctx->config.uart_fd,
                                                tx_frame, tx_len);
        if (written < 0 || (size_t)written != tx_len) {
            return NSTAR_ERR_HAL;
        }

        /* Read response with timeout */
        ssize_t nread = nstar_hal_uart_read(ctx->config.uart_fd,
                                             rx_frame, sizeof(rx_frame),
                                             NSTAR_CMD_TIMEOUT_MS);
        if (nread <= 0) {
            /* Timeout — retry if attempts remain */
            if (attempts <= (int)NSTAR_CMD_MAX_RETRIES) continue;
            return NSTAR_ERR_TIMEOUT;
        }

        /* Decode response */
        rc = nstar_frame_decode(rx_frame, (size_t)nread,
                                 resp_cmd_id, resp_buf, resp_len);

        if (rc == NSTAR_ERR_CRC) {
            /* CRC failure — retry if attempts remain */
            if (attempts <= (int)NSTAR_CMD_MAX_RETRIES) continue;
            return NSTAR_ERR_CRC;
        }

        return rc;   /* NSTAR_OK or NSTAR_ERR_BAD_FRAME */

    } while (attempts <= (int)NSTAR_CMD_MAX_RETRIES);

    return NSTAR_ERR_TIMEOUT;
}

/**
 * Issue a command and validate the response CMD_ID matches expected_resp_id.
 * Convenience wrapper over cmdq_execute().
 */
static nstar_result_t cmd_transact(nstar_ctx_t *ctx,
                                    char cmd_id,
                                    const uint8_t *data_in, size_t data_len,
                                    char expected_resp_id,
                                    uint8_t *resp_data, size_t *resp_data_len)
{
    char   resp_cmd_id = 0;
    size_t resp_len    = 0;
    uint8_t local_buf[NSTAR_FRAME_BUF_MAX / 2];

    uint8_t *out_buf = resp_data  ? resp_data  : local_buf;
    size_t  *out_len = resp_data_len ? resp_data_len : &resp_len;

    pthread_mutex_lock(&ctx->uart_mutex);
    nstar_result_t rc = cmdq_execute(ctx, cmd_id, data_in, data_len,
                                      out_buf, out_len, &resp_cmd_id);
    pthread_mutex_unlock(&ctx->uart_mutex);

    if (rc != NSTAR_OK) return rc;

    /* Response CMD_ID validation:
     * N-STAR echoes 'R' for read responses, 'A' for write ACKs,
     * 'V' for identity, 'E' for all-status. Accept either the echo
     * or 'A' (ACK) for write commands. */
    if (expected_resp_id != 0 &&
        resp_cmd_id != expected_resp_id &&
        resp_cmd_id != 'A') {
        return NSTAR_ERR_BAD_FRAME;
    }

    return NSTAR_OK;
}

/* =========================================================================
 * Lifecycle
 * =========================================================================
 */

/* Forward declarations for thread functions */
static void *rx_thread_func(void *arg);
static void *fault_thread_func(void *arg);
static void *health_thread_func(void *arg);

nstar_result_t nstar_init(const nstar_config_t *config,
                           const nstar_callbacks_t *callbacks,
                           nstar_ctx_t **ctx_out)
{
    if (!config || !callbacks || !ctx_out) return NSTAR_ERR_PARAM;

    nstar_ctx_t *ctx = calloc(1, sizeof(*ctx));
    if (!ctx) return NSTAR_ERR_HAL;

    ctx->config       = *config;
    ctx->callbacks    = *callbacks;
    ctx->stop_flag    = 0;
    ctx->rx_state     = NSTAR_RX_IDLE;
    ctx->module_state = NSTAR_MODULE_UNINIT;

    if (pthread_mutex_init(&ctx->uart_mutex, NULL) != 0) {
        free(ctx);
        return NSTAR_ERR_HAL;
    }

    if (pthread_mutex_init(&ctx->state_mutex, NULL) != 0) {
        pthread_mutex_destroy(&ctx->uart_mutex);
        free(ctx);
        return NSTAR_ERR_HAL;
    }

    set_module_state(ctx, NSTAR_MODULE_INITIALISING);

    ctx->initialised = 1;

    /* Spawn background threads */
    if (pthread_create(&ctx->rx_thread,     NULL, rx_thread_func,     ctx) != 0 ||
        pthread_create(&ctx->fault_thread,  NULL, fault_thread_func,  ctx) != 0 ||
        pthread_create(&ctx->health_thread, NULL, health_thread_func, ctx) != 0) {
        ctx->stop_flag = 1;
        pthread_mutex_destroy(&ctx->state_mutex);
        pthread_mutex_destroy(&ctx->uart_mutex);
        free(ctx);
        return NSTAR_ERR_HAL;
    }

    ctx->threads_started = 1;

    /*
     * Threads are running but startup_sequence() has not been called yet.
     * The caller (or the application's init flow) is expected to call
     * nstar_startup_sequence() next, which will transition STARTING->READY.
     */
    set_module_state(ctx, NSTAR_MODULE_STARTING);

    *ctx_out = ctx;
    return NSTAR_OK;
}

void nstar_deinit(nstar_ctx_t *ctx)
{
    if (!ctx) return;

    set_module_state(ctx, NSTAR_MODULE_SHUTTING_DOWN);

    /* Signal all threads to stop */
    ctx->stop_flag = 1;

    if (ctx->threads_started) {
        pthread_join(ctx->rx_thread,     NULL);
        pthread_join(ctx->fault_thread,  NULL);
        pthread_join(ctx->health_thread, NULL);
    }

    pthread_mutex_destroy(&ctx->uart_mutex);
    pthread_mutex_destroy(&ctx->state_mutex);
    free(ctx);
}

/* =========================================================================
 * Startup sequence
 * =========================================================================
 *
 * Step 1: Wait NSTAR_POWERUP_WAIT_MS for oscillator (User Manual §3.1)
 * Step 2: V command — read FPGA identity (version, build, HW serial,
 *         FPGA_TYPE, FPGA_OPTION). Cached on ctx->identity; retrievable
 *         afterwards via nstar_get_identity(). FPGA_OPTION here is the
 *         ONLY read of that data — registers 0x08/0x09 are not re-read
 *         separately, since the V response already contains identical
 *         bytes (IRD Annexe A: 0x08/0x09 IS FPGA_OPTION). An earlier
 *         version of this function re-read 0x08/0x09 via R commands and
 *         then discarded the result entirely without caching either
 *         copy — removed as both redundant and a real data-loss bug.
 * Step 3: R 0x06 — verify FPGA_TYPE == 0x62 (N-STAR PCM/PM, RX+TX)
 * Step 4: W 0x10 = 0x02 — configure 2-pass OBS sweep
 */
nstar_result_t nstar_startup_sequence(nstar_ctx_t *ctx)
{
    if (!ctx || !ctx->initialised) return NSTAR_ERR_NOT_INIT;

    /*
     * startup_sequence drives the module INTO READY (or FAULT on failure).
     * It is exempt from the NSTAR_MODULE_READY guard applied to other
     * entry points, since it is what establishes READY in the first place.
     * It is called both from application init flow and from the fault
     * thread during recovery (where module_state is already FAULT).
     */
    set_module_state(ctx, NSTAR_MODULE_STARTING);

    nstar_result_t rc;

    /* Step 1 — power-up wait */
    nstar_hal_sleep_ms(NSTAR_POWERUP_WAIT_MS);

    /* Step 2 — read identity (V command), cache on ctx */
    nstar_identity_t identity;
    memset(&identity, 0, sizeof(identity));
    rc = nstar_cmd_read_identity(ctx, &identity);
    if (rc != NSTAR_OK) { set_module_state(ctx, NSTAR_MODULE_FAULT); return rc; }
    ctx->identity       = identity;
    ctx->identity_valid = 1;

    /* Step 3 — verify FPGA_TYPE */
    uint8_t fpga_type = 0;
    rc = nstar_reg_read(ctx, NSTAR_REG_FPGA_TYPE, &fpga_type);
    if (rc != NSTAR_OK) { set_module_state(ctx, NSTAR_MODULE_FAULT); return rc; }
    if (fpga_type != NSTAR_FPGA_TYPE_EXPECTED) {
        set_module_state(ctx, NSTAR_MODULE_FAULT);
        return NSTAR_ERR_FPGA_TYPE;
    }

    /* Step 4 — configure 2-pass OBS sweep */
    rc = nstar_reg_write(ctx, NSTAR_REG_RX_NB_SWEEP, NSTAR_RX_SWEEP_2PASS);
    if (rc != NSTAR_OK) { set_module_state(ctx, NSTAR_MODULE_FAULT); return rc; }

    set_module_state(ctx, NSTAR_MODULE_READY);
    return NSTAR_OK;
}

/* =========================================================================
 * Register access
 * =========================================================================
 */

nstar_result_t nstar_reg_read(nstar_ctx_t *ctx, uint8_t addr,
                               uint8_t *val_out)
{
    if (!ctx || !val_out) return NSTAR_ERR_PARAM;

    /*
     * R command: DATA = address byte only.
     * N-STAR responds with the register value as 1 byte.
     */
    uint8_t resp[NSTAR_FRAME_BUF_MAX / 2];
    size_t  resp_len = 0;

    nstar_result_t rc = cmd_transact(ctx, 'R', &addr, 1,
                                      'R', resp, &resp_len);
    if (rc != NSTAR_OK) return rc;
    if (resp_len < 1) return NSTAR_ERR_BAD_FRAME;

    *val_out = resp[0];
    return NSTAR_OK;
}

nstar_result_t nstar_reg_write(nstar_ctx_t *ctx, uint8_t addr, uint8_t val)
{
    if (!ctx) return NSTAR_ERR_PARAM;

    /*
     * W command: DATA = address byte + value byte.
     * N-STAR responds with an ACK frame containing result code (0 = OK).
     */
    uint8_t payload[2] = { addr, val };
    uint8_t resp[NSTAR_FRAME_BUF_MAX / 2];
    size_t  resp_len = 0;

    nstar_result_t rc = cmd_transact(ctx, 'W', payload, 2,
                                      'A', resp, &resp_len);
    if (rc != NSTAR_OK) return rc;

    /* ACK payload: 1 byte result code. 0x00 = success. */
    if (resp_len >= 1 && resp[0] != 0x00) return NSTAR_ERR_BAD_ACK;

    return NSTAR_OK;
}

nstar_result_t nstar_reg_read_multi(nstar_ctx_t *ctx, uint8_t start_addr,
                                     uint8_t n, uint8_t *buf_out)
{
    if (!ctx || !buf_out || n == 0) return NSTAR_ERR_PARAM;

    /* Issue n individual R commands and collect results. */
    for (uint8_t i = 0; i < n; i++) {
        nstar_result_t rc = nstar_reg_read(ctx,
                                            (uint8_t)(start_addr + i),
                                            &buf_out[i]);
        if (rc != NSTAR_OK) return rc;
    }
    return NSTAR_OK;
}

/* =========================================================================
 * Named commands
 * =========================================================================
 */

nstar_result_t nstar_cmd_read_identity(nstar_ctx_t *ctx,
                                        nstar_identity_t *out)
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
    size_t  resp_len = 0;

    nstar_result_t rc = cmd_transact(ctx, 'V', NULL, 0,
                                      'V', resp, &resp_len);
    if (rc != NSTAR_OK) return rc;
    if (resp_len < 6) return NSTAR_ERR_BAD_FRAME;

    memset(out, 0, sizeof(*out));
    out->fpga_version = resp[0];
    out->fpga_build   = resp[1];
    out->hw_year      = resp[2];
    out->hw_week      = resp[3];
    if (resp_len >= 6) {
        out->hw_order = (uint16_t)((resp[4] << 8) | resp[5]);
    }
    if (resp_len >= 7) {
        out->fpga_type = resp[6];
    }
    if (resp_len >= 9) {
        out->fpga_options = (uint16_t)((resp[7] << 8) | resp[8]);
    }
    return NSTAR_OK;
}

nstar_result_t nstar_cmd_read_all_rx_status(nstar_ctx_t *ctx,
                                             uint8_t *raw_out,
                                             size_t  *len_out)
{
    if (!ctx || !raw_out || !len_out) return NSTAR_ERR_PARAM;

    /*
     * E command: no data payload.
     * Response: registers 0x10-0x22 in one frame (19 bytes).
     */
    char   resp_cmd_id = 0;
    size_t resp_len    = 0;

    pthread_mutex_lock(&ctx->uart_mutex);
    nstar_result_t rc = cmdq_execute(ctx, 'E', NULL, 0,
                                      raw_out, &resp_len, &resp_cmd_id);
    pthread_mutex_unlock(&ctx->uart_mutex);

    if (rc != NSTAR_OK) return rc;
    *len_out = resp_len;
    return NSTAR_OK;
}

nstar_result_t nstar_cmd_reset(nstar_ctx_t *ctx)
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
    size_t  resp_len = 0;

    nstar_result_t rc = cmd_transact(ctx, 'C', magic, 2,
                                      'A', resp, &resp_len);
    if (rc != NSTAR_OK) return rc;
    if (resp_len >= 1 && resp[0] != 0x00) return NSTAR_ERR_BAD_ACK;
    return NSTAR_OK;
}

/* =========================================================================
 * TX pipeline (Stage 3)
 * =========================================================================
 *
 * Sequence (from flow diagram and EICD §5.11.1):
 *
 *   nstar_tx_start():
 *     1. Write TX data rate register (W 0x22 = rate_code)
 *     2. Assert TX clock via nstar_hal_data_clock_start()
 *     3. Wait NSTAR_TX_CLOCK_PRESTABLE_MS for clock to stabilise
 *     4. Read TX_STATUS (R 0x40) — verify b4 (clock detected) = 1
 *     5. Write TX_MODE = Modulation (W 0x40 = 0x01)
 *
 *   nstar_tx_write():
 *     - Write pre-framed application data to data interface in chunks
 *       of NSTAR_FRAME_SIZE_BYTES.  Accumulates total bytes sent.
 *
 *   nstar_tx_stop():
 *     1. Write TX_MODE = Standby (W 0x40 = 0x00)
 *     2. Stop TX clock via nstar_hal_data_clock_stop()
 *     3. Fire on_tx_complete callback with total bytes sent
 *     4. Reset tx_active flag and byte counter
 *
 *   nstar_tx_get_status():
 *     - Read TX_STATUS register, decode mode and clock-detected bit.
 *
 * No background thread: TX is driven synchronously by the application.
 * The data interface write (nstar_hal_data_write) is blocking.
 * Clock gap constraint (EICD §5.11.1): if the clock is absent for
 * >1 ms during active modulation, N-STAR enters standby automatically.
 * The application is responsible for keeping nstar_tx_write() calls
 * frequent enough to avoid gaps.
 */

nstar_result_t nstar_tx_start(nstar_ctx_t *ctx,
                               nstar_tx_rate_code_t rate_code)
{
    if (!ctx || !ctx->initialised) return NSTAR_ERR_NOT_INIT;
    if (get_module_state(ctx) != NSTAR_MODULE_READY) return NSTAR_ERR_NOT_READY;
    if (ctx->tx_active)           return NSTAR_ERR_BUSY;

    nstar_result_t rc;

    /* Step 1: Set TX data rate (OPT-C31-TDF ordered — rate is configurable) */
    rc = nstar_reg_write(ctx, NSTAR_REG_RX_DATA_RATE, (uint8_t)rate_code);
    if (rc != NSTAR_OK) return rc;

    /* Step 2: Assert TX clock on CLK_TX pins */
    rc = nstar_hal_data_clock_start(ctx->config.data_fd);
    if (rc != NSTAR_OK) return rc;

    /* Step 3: Wait for clock to stabilise */
    nstar_hal_sleep_ms(NSTAR_TX_CLOCK_PRESTABLE_MS);

    /* Step 4: Read TX_STATUS and verify clock detected (b4 = 1) */
    nstar_tx_status_t status;
    memset(&status, 0, sizeof(status));
    rc = nstar_tx_get_status(ctx, &status);
    if (rc != NSTAR_OK) {
        nstar_hal_data_clock_stop(ctx->config.data_fd);
        return rc;
    }
    if (!status.clock_detected) {
        nstar_hal_data_clock_stop(ctx->config.data_fd);
        return NSTAR_ERR_NO_CLOCK;
    }

    /* Step 5: Enable modulation */
    rc = nstar_reg_write(ctx, NSTAR_REG_TX_MODE, NSTAR_TX_MODE_MODULATION);
    if (rc != NSTAR_OK) {
        nstar_hal_data_clock_stop(ctx->config.data_fd);
        return rc;
    }

    ctx->tx_active     = 1;
    ctx->tx_bytes_sent = 0;
    return NSTAR_OK;
}

nstar_result_t nstar_tx_write(nstar_ctx_t *ctx,
                               const uint8_t *buf, size_t len)
{
    if (!ctx || !ctx->initialised) return NSTAR_ERR_NOT_INIT;
    if (!buf || len == 0)          return NSTAR_ERR_PARAM;
    if (!ctx->tx_active)           return NSTAR_ERR_BUSY;

    /*
     * Write in NSTAR_FRAME_SIZE_BYTES chunks.
     * Each call to nstar_hal_data_write is blocking.
     * The caller must not allow gaps between calls exceeding
     * NSTAR_TX_CLOCK_GAP_MAX_MS (1 ms) or N-STAR will enter standby.
     */
    size_t remaining = len;
    const uint8_t *ptr = buf;

    while (remaining > 0) {
        size_t chunk = (remaining > NSTAR_FRAME_SIZE_BYTES)
                       ? NSTAR_FRAME_SIZE_BYTES
                       : remaining;

        ssize_t written = nstar_hal_data_write(ctx->config.data_fd,
                                                ptr, chunk);
        if (written < 0) return NSTAR_ERR_HAL;

        ptr                 += (size_t)written;
        remaining           -= (size_t)written;
        ctx->tx_bytes_sent  += (size_t)written;
    }

    return NSTAR_OK;
}

nstar_result_t nstar_tx_stop(nstar_ctx_t *ctx)
{
    if (!ctx || !ctx->initialised) return NSTAR_ERR_NOT_INIT;
    if (!ctx->tx_active)           return NSTAR_OK;  /* idempotent */

    /* Step 1: Enter standby */
    nstar_reg_write(ctx, NSTAR_REG_TX_MODE, NSTAR_TX_MODE_STANDBY);

    /* Step 2: Stop clock */
    nstar_hal_data_clock_stop(ctx->config.data_fd);

    /* Step 3: Notify application */
    size_t bytes = ctx->tx_bytes_sent;
    ctx->tx_active     = 0;
    ctx->tx_bytes_sent = 0;

    if (ctx->callbacks.on_tx_complete) {
        ctx->callbacks.on_tx_complete(bytes);
    }

    return NSTAR_OK;
}

nstar_result_t nstar_tx_get_status(nstar_ctx_t *ctx,
                                    nstar_tx_status_t *out)
{
    if (!ctx || !out) return NSTAR_ERR_PARAM;

    uint8_t raw = 0;
    nstar_result_t rc = nstar_reg_read(ctx, NSTAR_REG_TX_MODE, &raw);
    if (rc != NSTAR_OK) return rc;

    memset(out, 0, sizeof(*out));
    out->raw            = raw;
    out->current_mode   = (nstar_tx_mode_t)(raw & NSTAR_TX_STATUS_MODE_MASK);
    out->clock_detected = (raw & NSTAR_TX_STATUS_CLOCK_DETECTED) != 0;
    out->config_set     = (raw >> 2) & 0x03U;
    return NSTAR_OK;
}

/* =========================================================================
 * RX pipeline (Stage 4)
 * =========================================================================
 *
 * nstar_rx_configure():
 *   Writes RX data rate to register 0x22. Call before contact window.
 *   Requires OPT-C31-RDF (ordered on Lumos EM).
 *
 * nstar_rx_get_status():
 *   Reads RX_STATUS register (R 0x10), decodes four lock bits.
 *
 * nstar_rx_get_link_quality():
 *   Reads Eb/N0, RSSI, and Doppler frequency shift registers.
 *   Only meaningful when rx_state == NSTAR_RX_LOCKED.
 *
 * rx_thread_func():
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
 * The RX thread is the ONLY consumer of data_fd reads.
 * UART diagnostic commands (nstar_rx_get_status, nstar_rx_get_link_quality)
 * must NOT be called from within the RX thread — they use uart_mutex.
 */

nstar_result_t nstar_rx_configure(nstar_ctx_t *ctx,
                                   nstar_rx_rate_code_t rate_code)
{
    if (!ctx || !ctx->initialised) return NSTAR_ERR_NOT_INIT;
    if (get_module_state(ctx) != NSTAR_MODULE_READY) return NSTAR_ERR_NOT_READY;

    /*
     * RX data rate is stored in the same register as TX (0x22).
     * OPT-C31-RDF must be ordered for this to be configurable.
     */
    return nstar_reg_write(ctx, NSTAR_REG_RX_DATA_RATE, (uint8_t)rate_code);
}

nstar_result_t nstar_rx_get_status(nstar_ctx_t *ctx,
                                    nstar_rx_status_t *out)
{
    if (!ctx || !out) return NSTAR_ERR_PARAM;

    uint8_t raw = 0;
    nstar_result_t rc = nstar_reg_read(ctx, NSTAR_REG_RX_STATUS, &raw);
    if (rc != NSTAR_OK) return rc;

    memset(out, 0, sizeof(*out));
    out->raw            = raw;
    out->carrier_detect = (raw & NSTAR_RX_STATUS_CARRIER_DETECT) != 0;
    out->carrier_lock   = (raw & NSTAR_RX_STATUS_CARRIER_LOCK)   != 0;
    out->bit_lock       = (raw & NSTAR_RX_STATUS_BIT_LOCK)        != 0;
    out->data_valid     = (raw & NSTAR_RX_STATUS_DATA_VALID)      != 0;
    out->sweep_state    = (raw >> 5) & 0x03U;
    return NSTAR_OK;
}

nstar_result_t nstar_rx_get_link_quality(nstar_ctx_t *ctx,
                                          nstar_link_quality_t *out)
{
    if (!ctx || !out) return NSTAR_ERR_PARAM;

    nstar_result_t rc;
    uint8_t buf[3];
    memset(out, 0, sizeof(*out));

    /*
     * Eb/N0: Eb = R 0x1C-0x1E (24-bit, MSB first)
     *        N0 = R 0x1F-0x21 (24-bit, MSB first)
     * Eb/N0 (dB) = 20 × log10(Eb_raw / N0_raw) − 3
     * (IRD Annex C)
     */
    rc = nstar_reg_read_multi(ctx, NSTAR_REG_RX_DEMOD_EB_MSB, 3, buf);
    if (rc != NSTAR_OK) return rc;
    uint32_t eb_raw = ((uint32_t)buf[0] << 16) |
                      ((uint32_t)buf[1] <<  8) |
                       (uint32_t)buf[2];

    rc = nstar_reg_read_multi(ctx, NSTAR_REG_RX_DEMOD_N0_MSB, 3, buf);
    if (rc != NSTAR_OK) return rc;
    uint32_t n0_raw = ((uint32_t)buf[0] << 16) |
                      ((uint32_t)buf[1] <<  8) |
                       (uint32_t)buf[2];

    if (eb_raw > 0 && n0_raw > 0) {
        out->eb_no_db = 20.0f * log10f((float)eb_raw / (float)n0_raw) - 3.0f;
    }

    /*
     * RSSI: IQ_POWER = R 0x17-0x18 (16-bit MSB first) = RX signal power
     *       AGC      = R 0x19-0x1A (16-bit MSB first) = RX gain
     * RSSI (dBm) = RX_IQ_power − RX_GAIN_RF (IRD Annex C)
     */
    rc = nstar_reg_read_multi(ctx, NSTAR_REG_RX_IQ_POWER_MSB, 2, buf);
    if (rc != NSTAR_OK) return rc;
    float iq_power = (float)(((uint16_t)buf[0] << 8) | buf[1]);

    rc = nstar_reg_read_multi(ctx, NSTAR_REG_RX_AGC_MSB, 2, buf);
    if (rc != NSTAR_OK) return rc;
    float agc_val = (float)(((uint16_t)buf[0] << 8) | buf[1]);

    out->rssi_dbm = iq_power - agc_val;

    /*
     * Frequency shift: R 0x14-0x16 (24-bit signed, MSB first)
     * Freq shift (Hz) = signed_24bit / 8  (IRD Annex C)
     */
    rc = nstar_reg_read_multi(ctx, NSTAR_REG_RX_FREQ_SHIFT_MSB, 3, buf);
    if (rc != NSTAR_OK) return rc;
    uint32_t raw24 = ((uint32_t)buf[0] << 16) |
                     ((uint32_t)buf[1] <<  8) |
                      (uint32_t)buf[2];
    /* Sign-extend 24-bit to 32-bit */
    int32_t signed24 = (raw24 & 0x800000U)
                       ? (int32_t)(raw24 | 0xFF000000U)
                       : (int32_t)raw24;
    out->freq_shift_hz = signed24 / 8;

    return NSTAR_OK;
}

/* -------------------------------------------------------------------------
 * RX thread
 * ------------------------------------------------------------------------- */

static void *rx_thread_func(void *arg)
{
    nstar_ctx_t *ctx = (nstar_ctx_t *)arg;

    while (!ctx->stop_flag) {

        /* ── IDLE: wait for LOCK_DETECT rising edge ── */
        ctx->rx_state = NSTAR_RX_IDLE;

        nstar_result_t rc = nstar_hal_gpio_wait_edge(
            ctx->config.gpio_lock_detect,
            NSTAR_GPIO_EDGE_RISING,
            500);   /* 500 ms poll interval; retry on timeout */

        if (ctx->stop_flag) break;
        if (rc == NSTAR_ERR_TIMEOUT) continue;   /* no lock — keep waiting */
        if (rc != NSTAR_OK) continue;

        /* ── ACQUIRING: carrier locked, wait for DATA_VALID rising ── */
        ctx->rx_state = NSTAR_RX_ACQUIRING;

        if (ctx->callbacks.on_lock_acquired) {
            ctx->callbacks.on_lock_acquired();
        }

        rc = nstar_hal_gpio_wait_edge(
            ctx->config.gpio_data_valid,
            NSTAR_GPIO_EDGE_RISING,
            3000);   /* OBS sweep lock time < 3 s (Config Sheet) */

        if (ctx->stop_flag) break;
        if (rc != NSTAR_OK) {
            /* DATA_VALID did not assert — back to IDLE */
            continue;
        }

        /* ── LOCKED: DATA_VALID asserted, begin reading frames ── */
        ctx->rx_state = NSTAR_RX_LOCKED;

        uint8_t frame_buf[NSTAR_FRAME_SIZE_BYTES];

        while (!ctx->stop_flag) {
            /* Check DATA_VALID still HIGH before each read */
            int dv = nstar_hal_gpio_read(ctx->config.gpio_data_valid);
            if (dv != 1) break;   /* DATA_VALID fell → LOCK_LOST */

            ssize_t nread = nstar_hal_data_read(
                ctx->config.data_fd,
                frame_buf,
                sizeof(frame_buf));

            if (nread <= 0) {
                /* No data available — re-check DATA_VALID before looping */
                dv = nstar_hal_gpio_read(ctx->config.gpio_data_valid);
                if (dv != 1) break;
                /* DATA_VALID still high but no data — yield and retry */
                nstar_hal_sleep_ms(1);
                continue;
            }

            /* Post complete frame to application */
            if (ctx->callbacks.on_frame_received) {
                ctx->callbacks.on_frame_received(frame_buf, (size_t)nread);
            }
        }

        /* ── LOCK_LOST: DATA_VALID fell, clean up ── */
        ctx->rx_state = NSTAR_RX_LOCK_LOST;

        if (ctx->callbacks.on_lock_lost) {
            ctx->callbacks.on_lock_lost();
        }
        /* Loop back to IDLE */
    }

    return NULL;
}

/* =========================================================================
 * Health monitoring (Stage 5)
 * =========================================================================
 *
 * nstar_health_read():
 *   Reads PA and BB temperature ADC registers (two 16-bit values each,
 *   MSB first).  Converts via NSTAR_TEMP_FROM_ADC().
 *   Also reads FAULT_N GPIO to populate fault_active field.
 *
 * health_thread_func():
 *   Polls PA temperature every NSTAR_HEALTH_POLL_INTERVAL_MS.
 *   If T > NSTAR_PA_TEMP_WARN_CELSIUS: fires on_fault(TEMPERATURE)
 *   and stops TX.
 *
 * fault_thread_func():
 *   Blocks on FAULT_N falling edge (active-low fault assertion).
 *   On fault:
 *     1. Stop TX if active.
 *     2. Wait FAULT_N rising edge (N-STAR auto-recovery).
 *     3. If not recovered within timeout: assert RESET_N (GPIO LOW).
 *     4. Fire on_fault(NSTAR_FAULT_SEL) callback.
 *     5. Re-run startup_sequence() to restore register configuration.
 */

nstar_result_t nstar_health_read(nstar_ctx_t *ctx, nstar_health_t *out)
{
    if (!ctx || !out) return NSTAR_ERR_PARAM;

    memset(out, 0, sizeof(*out));
    nstar_result_t rc;
    uint8_t buf[2];

    /*
     * PA temperature: ADC registers 0xC8 (MSB) + 0xC9 (LSB).
     * T(°C) = 0.06105 × raw − 50  (IRD Annex C)
     */
    rc = nstar_reg_read_multi(ctx, NSTAR_REG_ADC_PA_TEMP_MSB, 2, buf);
    if (rc != NSTAR_OK) return rc;
    out->pa_adc_raw     = (uint16_t)((buf[0] << 8) | buf[1]);
    out->pa_temp_celsius = NSTAR_TEMP_FROM_ADC(out->pa_adc_raw);

    /*
     * BB (BaseBand) temperature: ADC registers 0xC0 (MSB) + 0xC1 (LSB).
     */
    rc = nstar_reg_read_multi(ctx, NSTAR_REG_ADC_BB_TEMP_MSB, 2, buf);
    if (rc != NSTAR_OK) return rc;
    out->bb_adc_raw      = (uint16_t)((buf[0] << 8) | buf[1]);
    out->bb_temp_celsius = NSTAR_TEMP_FROM_ADC(out->bb_adc_raw);

    /* FAULT_N is active-LOW: read 0 means fault asserted */
    int fault_pin = nstar_hal_gpio_read(ctx->config.gpio_fault_n);
    out->fault_active = (fault_pin == 0);

    return NSTAR_OK;
}

/* -------------------------------------------------------------------------
 * Health thread
 * ------------------------------------------------------------------------- */

static void *health_thread_func(void *arg)
{
    nstar_ctx_t *ctx = (nstar_ctx_t *)arg;

    while (!ctx->stop_flag) {
        nstar_hal_sleep_ms(NSTAR_HEALTH_POLL_INTERVAL_MS);

        if (ctx->stop_flag) break;

        /*
         * Only poll health when the module is READY. During INITIALISING,
         * STARTING, or FAULT recovery, the UART is owned by the startup
         * sequence (or the fault thread's recovery flow) and polling here
         * would contend for the same mocked/real UART resource and could
         * read stale or meaningless register values mid-reconfiguration.
         */
        if (get_module_state(ctx) != NSTAR_MODULE_READY) continue;

        nstar_health_t health;
        nstar_result_t rc = nstar_health_read(ctx, &health);
        if (rc != NSTAR_OK) continue;   /* UART busy or timeout — try next cycle */

        if (health.pa_temp_celsius > NSTAR_PA_TEMP_WARN_CELSIUS) {
            /* Proactively stop TX before N-STAR auto-stops at 90°C */
            if (ctx->tx_active) {
                nstar_tx_stop(ctx);
            }
            if (ctx->callbacks.on_fault) {
                ctx->callbacks.on_fault(NSTAR_FAULT_TEMPERATURE);
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

static void *fault_thread_func(void *arg)
{
    nstar_ctx_t *ctx = (nstar_ctx_t *)arg;

    while (!ctx->stop_flag) {

        /* Block on FAULT_N falling edge (fault asserts LOW) */
        nstar_result_t rc = nstar_hal_gpio_wait_edge(
            ctx->config.gpio_fault_n,
            NSTAR_GPIO_EDGE_FALLING,
            500);   /* 500 ms poll interval */

        if (ctx->stop_flag) break;
        if (rc == NSTAR_ERR_TIMEOUT) continue;
        if (rc != NSTAR_OK)          continue;

        /* Fault detected — module is no longer READY */
        set_module_state(ctx, NSTAR_MODULE_FAULT);

        /* Step 1: Stop TX immediately if active */
        if (ctx->tx_active) {
            nstar_tx_stop(ctx);
        }

        /* Step 2: Wait for N-STAR to auto-recover (FAULT_N rising) */
        rc = nstar_hal_gpio_wait_edge(
            ctx->config.gpio_fault_n,
            NSTAR_GPIO_EDGE_RISING,
            FAULT_RECOVERY_TIMEOUT_MS);

        if (rc != NSTAR_OK) {
            /*
             * N-STAR did not self-recover within timeout.
             * Assert hardware RESET_N (active-LOW) to force a power cycle.
             */
            nstar_hal_gpio_write(ctx->config.gpio_reset_n, 0);   /* LOW */
            nstar_hal_sleep_ms(FAULT_RESET_HOLD_MS);
            nstar_hal_gpio_write(ctx->config.gpio_reset_n, 1);   /* HIGH */

            /* Wait again for FAULT_N to clear after hardware reset */
            nstar_hal_gpio_wait_edge(
                ctx->config.gpio_fault_n,
                NSTAR_GPIO_EDGE_RISING,
                FAULT_RECOVERY_TIMEOUT_MS);
        }

        /* Step 3: Notify application */
        if (ctx->callbacks.on_fault) {
            ctx->callbacks.on_fault(NSTAR_FAULT_SEL);
        }

        /* Step 4: Re-run startup sequence — N-STAR registers reset to defaults */
        nstar_startup_sequence(ctx);
    }

    return NULL;
}
