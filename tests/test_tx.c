/**
 * @file  test_tx.c
 * @brief Stage 3 unit tests — TX pipeline.
 *
 * Tests cover nstar_tx_start(), nstar_tx_write(), nstar_tx_stop(),
 * and nstar_tx_get_status() against the mock HAL.
 *
 * TX sequence under test:
 *   nstar_tx_start():
 *     W 0x22 = rate_code  (set TX data rate)
 *     clock_start()        (assert CLK_TX)
 *     sleep 100 ms         (stabilise)
 *     R 0x40               (read TX_STATUS, check b4=1)
 *     W 0x40 = 0x01        (TX_MODE = Modulation)
 *
 *   nstar_tx_write():
 *     data_write() in NSTAR_FRAME_SIZE_BYTES chunks
 *
 *   nstar_tx_stop():
 *     W 0x40 = 0x00        (TX_MODE = Standby)
 *     clock_stop()
 *     on_tx_complete(bytes_sent)
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

static nstar_ctx_t *g_ctx      = NULL;
static size_t       g_tx_cb_bytes = 0;
static int          g_tx_cb_fired = 0;

static void on_tx_complete(size_t bytes)
{
    g_tx_cb_fired = 1;
    g_tx_cb_bytes = bytes;
}
static void on_frame_received(const uint8_t *b, size_t l) { (void)b;(void)l; }
static void on_fault(nstar_fault_source_t s)               { (void)s; }
static void on_lock_acquired(void)                         {}
static void on_lock_lost(void)                             {}

static const nstar_callbacks_t k_cb = {
    .on_frame_received = on_frame_received,
    .on_tx_complete    = on_tx_complete,
    .on_fault          = on_fault,
    .on_lock_acquired  = on_lock_acquired,
    .on_lock_lost      = on_lock_lost,
};

static const nstar_config_t k_cfg = {
    .uart_fd          = 10,
    .data_fd          = 11,
    .gpio_lock_detect = 20,
    .gpio_data_valid  = 21,
    .gpio_fault_n     = 22,
    .gpio_reset_n     = 23,
};

static void qr(const char *s)
{
    nstar_mock_uart_queue_response((const uint8_t *)s, strlen(s));
}

/* Queue the 3-response sequence for a successful startup_sequence:
 *   V identity (includes FPGA_OPTION), R 0x06 (FPGA_TYPE), W 0x10 ACK.
 * The redundant R 0x08/R 0x09 reads were removed from nstar_core.c —
 * FPGA_OPTION is already in the V response (IRD Annexe A: 0x08/0x09
 * IS FPGA_OPTION), so re-reading it via R was both wasted round-trips
 * and, worse, the result was being discarded entirely without caching
 * either copy. Now cached on ctx via the V response only. */
static void queue_startup_happy(void)
{
    qr("<V12:010018230042620307:F832>");
    qr("<R02:62:7D57>");
    qr("<A02:00:466C>");
}

/* Queue the standard 3-response sequence for a successful tx_start:
 *   W 0x22 ACK, R 0x40 (clock detected), W 0x40=0x01 ACK */
static void queue_tx_start_happy(void)
{
    qr("<A02:00:466C>");   /* W 0x22 rate ACK                        */
    qr("<R02:10:9EA5>");   /* R 0x40 TX_STATUS: b4=1 clock detected  */
    qr("<A02:00:466C>");   /* W 0x40=0x01 modulation ACK             */
}

/* Queue the sequence for a successful tx_stop:
 *   W 0x40=0x00 ACK */
static void queue_tx_stop_happy(void)
{
    qr("<A02:00:466C>");   /* W 0x40=0x00 standby ACK */
}

void setUp(void)
{
    nstar_mock_reset();
    g_tx_cb_fired = 0;
    g_tx_cb_bytes = 0;
    nstar_result_t rc = nstar_init(&k_cfg, &k_cb, &g_ctx);
    TEST_ASSERT_EQUAL(NSTAR_OK, rc);

    /*
     * nstar_init() leaves the module in STARTING, not READY.
     * nstar_tx_start() and nstar_rx_configure() require READY.
     * Run startup_sequence() here so all TX tests start from READY,
     * matching the application's real init flow.
     */
    queue_startup_happy();
    rc = nstar_startup_sequence(g_ctx);
    TEST_ASSERT_EQUAL(NSTAR_OK, rc);
    TEST_ASSERT_EQUAL(NSTAR_MODULE_READY, nstar_get_module_state(g_ctx));
}

void tearDown(void)
{
    if (g_ctx) { nstar_deinit(g_ctx); g_ctx = NULL; }
}

/* =========================================================================
 * nstar_tx_get_status tests
 * =========================================================================
 */

void test_tx_get_status_null_ctx_returns_param_error(void)
{
    nstar_tx_status_t st;
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM, nstar_tx_get_status(NULL, &st));
}

void test_tx_get_status_null_out_returns_param_error(void)
{
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM, nstar_tx_get_status(g_ctx, NULL));
}

/**
 * TX_STATUS = 0x10 → mode=STANDBY (b1:b0=00), clock_detected=1 (b4=1).
 */
void test_tx_get_status_clock_detected(void)
{
    qr("<R02:10:9EA5>");
    nstar_tx_status_t st;
    memset(&st, 0, sizeof(st));
    TEST_ASSERT_EQUAL(NSTAR_OK, nstar_tx_get_status(g_ctx, &st));
    TEST_ASSERT_TRUE(st.clock_detected);
    TEST_ASSERT_EQUAL(NSTAR_TX_STANDBY, st.current_mode);
    TEST_ASSERT_EQUAL_HEX8(0x10, st.raw);
}

/**
 * TX_STATUS = 0x11 → mode=MODULATION (b1:b0=01), clock_detected=1.
 */
void test_tx_get_status_modulation_active(void)
{
    qr("<R02:11:AD94>");
    nstar_tx_status_t st;
    memset(&st, 0, sizeof(st));
    TEST_ASSERT_EQUAL(NSTAR_OK, nstar_tx_get_status(g_ctx, &st));
    TEST_ASSERT_TRUE(st.clock_detected);
    TEST_ASSERT_EQUAL(NSTAR_TX_MODULATION, st.current_mode);
}

/**
 * TX_STATUS = 0x00 → no clock, standby.
 */
void test_tx_get_status_no_clock(void)
{
    qr("<R02:00:A995>");
    nstar_tx_status_t st;
    memset(&st, 0, sizeof(st));
    TEST_ASSERT_EQUAL(NSTAR_OK, nstar_tx_get_status(g_ctx, &st));
    TEST_ASSERT_FALSE(st.clock_detected);
    TEST_ASSERT_EQUAL(NSTAR_TX_STANDBY, st.current_mode);
}

/* =========================================================================
 * nstar_tx_start tests
 * =========================================================================
 */

void test_tx_start_null_ctx_returns_not_init(void)
{
    TEST_ASSERT_EQUAL(NSTAR_ERR_NOT_INIT,
        nstar_tx_start(NULL, NSTAR_TX_RATE_32K));
}

/**
 * Happy path: start TX at 32 kbps.
 * Verify: returns OK, clock is running, tx_active is set.
 */
void test_tx_start_success(void)
{
    queue_tx_start_happy();
    TEST_ASSERT_EQUAL(NSTAR_OK,
        nstar_tx_start(g_ctx, NSTAR_TX_RATE_32K));
    TEST_ASSERT_TRUE(nstar_mock_data_clock_is_running());
}

/**
 * Verify the UART frames sent during tx_start in the correct order:
 *   Frame 1: W 0x22 = rate_code   (TX data rate)
 *   Frame 2: R 0x40               (read TX_STATUS)
 *   Frame 3: W 0x40 = 0x01        (TX_MODE = Modulation)
 *
 * setUp() already ran startup_sequence(), which wrote its own frames
 * (V, R, R, R, W) to the same cumulative mock write buffer. We record
 * the buffer length before calling tx_start() and only inspect bytes
 * written after that point.
 */
void test_tx_start_sends_frames_in_correct_order(void)
{
    size_t baseline_len = 0;
    nstar_mock_uart_get_written(&baseline_len);

    queue_tx_start_happy();
    nstar_tx_start(g_ctx, NSTAR_TX_RATE_32K);

    size_t wlen = 0;
    const uint8_t *w_full = nstar_mock_uart_get_written(&wlen);
    const uint8_t *w = w_full + baseline_len;
    size_t new_len = wlen - baseline_len;

    /* Frame 1 (the first frame written by tx_start) must be a W command */
    TEST_ASSERT_TRUE_MESSAGE(new_len >= 2, "No new bytes written by tx_start");
    TEST_ASSERT_EQUAL_CHAR('<', (char)w[0]);
    TEST_ASSERT_EQUAL_CHAR('W', (char)w[1]);

    /* Frame 3 must contain 4001 (TX_MODE reg=0x40, val=0x01) */
    /* Scan only the new bytes for <W04:4001: */
    int found_modulation = 0;
    for (size_t i = 0; i + 9 < new_len; i++) {
        if (w[i]=='<' && w[i+1]=='W' &&
            w[i+5]=='4' && w[i+6]=='0' &&
            w[i+7]=='0' && w[i+8]=='1') {
            found_modulation = 1;
            break;
        }
    }
    TEST_ASSERT_TRUE_MESSAGE(found_modulation,
        "Expected W frame 0x40=0x01 (TX_MODE=Modulation) in UART writes");
}

/**
 * TX rate is written to register 0x22.
 * For NSTAR_TX_RATE_32K (0x0F), frame data must contain "220F".
 */
void test_tx_start_sets_correct_rate_register(void)
{
    queue_tx_start_happy();
    nstar_tx_start(g_ctx, NSTAR_TX_RATE_32K);

    size_t wlen = 0;
    const uint8_t *w = nstar_mock_uart_get_written(&wlen);

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
void test_tx_start_no_clock_returns_no_clock_error(void)
{
    qr("<A02:00:466C>");   /* W 0x22 rate ACK                         */
    qr("<R02:00:A995>");   /* R 0x40 TX_STATUS: b4=0 clock NOT found  */

    TEST_ASSERT_EQUAL(NSTAR_ERR_NO_CLOCK,
        nstar_tx_start(g_ctx, NSTAR_TX_RATE_32K));

    /* Clock must be de-asserted after failure */
    TEST_ASSERT_FALSE(nstar_mock_data_clock_is_running());
}

/**
 * UART timeout on rate-write → tx_start returns NSTAR_ERR_TIMEOUT.
 */
void test_tx_start_rate_write_timeout_returns_timeout(void)
{
    nstar_mock_uart_force_timeout(1);
    TEST_ASSERT_EQUAL(NSTAR_ERR_TIMEOUT,
        nstar_tx_start(g_ctx, NSTAR_TX_RATE_32K));
}

/**
 * Calling tx_start when already active → NSTAR_ERR_BUSY.
 */
void test_tx_start_when_already_active_returns_busy(void)
{
    queue_tx_start_happy();
    TEST_ASSERT_EQUAL(NSTAR_OK, nstar_tx_start(g_ctx, NSTAR_TX_RATE_32K));
    TEST_ASSERT_EQUAL(NSTAR_ERR_BUSY,
        nstar_tx_start(g_ctx, NSTAR_TX_RATE_32K));
}

/* =========================================================================
 * nstar_tx_write tests
 * =========================================================================
 */

void test_tx_write_null_ctx_returns_not_init(void)
{
    uint8_t buf[4] = {0};
    TEST_ASSERT_EQUAL(NSTAR_ERR_NOT_INIT,
        nstar_tx_write(NULL, buf, 4));
}

void test_tx_write_null_buf_returns_param_error(void)
{
    queue_tx_start_happy();
    nstar_tx_start(g_ctx, NSTAR_TX_RATE_32K);
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM,
        nstar_tx_write(g_ctx, NULL, 10));
}

/**
 * Write without calling tx_start → NSTAR_ERR_BUSY.
 */
void test_tx_write_without_start_returns_busy(void)
{
    uint8_t buf[16] = {0xAA};
    TEST_ASSERT_EQUAL(NSTAR_ERR_BUSY,
        nstar_tx_write(g_ctx, buf, sizeof(buf)));
}

/**
 * Happy path: write 100 bytes, verify all arrive at data interface.
 */
void test_tx_write_delivers_all_bytes(void)
{
    queue_tx_start_happy();
    nstar_tx_start(g_ctx, NSTAR_TX_RATE_32K);

    uint8_t payload[100];
    for (int i = 0; i < 100; i++) payload[i] = (uint8_t)i;

    TEST_ASSERT_EQUAL(NSTAR_OK,
        nstar_tx_write(g_ctx, payload, sizeof(payload)));

    size_t written_len = 0;
    const uint8_t *written = nstar_mock_data_get_written(&written_len);
    TEST_ASSERT_EQUAL_UINT(100, written_len);
    TEST_ASSERT_EQUAL_MEMORY(payload, written, 100);
}

/**
 * Write exactly NSTAR_FRAME_SIZE_BYTES — should be one chunk.
 */
void test_tx_write_exact_frame_size(void)
{
    queue_tx_start_happy();
    nstar_tx_start(g_ctx, NSTAR_TX_RATE_32K);

    uint8_t payload[NSTAR_FRAME_SIZE_BYTES];
    memset(payload, 0xAB, sizeof(payload));

    TEST_ASSERT_EQUAL(NSTAR_OK,
        nstar_tx_write(g_ctx, payload, sizeof(payload)));

    size_t wlen = 0;
    const uint8_t *w = nstar_mock_data_get_written(&wlen);
    TEST_ASSERT_EQUAL_UINT(NSTAR_FRAME_SIZE_BYTES, wlen);
    TEST_ASSERT_EQUAL_MEMORY(payload, w, NSTAR_FRAME_SIZE_BYTES);
}

/**
 * Write 2 × NSTAR_FRAME_SIZE_BYTES + 1 byte — should split into 3 chunks
 * but all data arrives intact at the data interface.
 */
void test_tx_write_multi_chunk_preserves_all_bytes(void)
{
    queue_tx_start_happy();
    nstar_tx_start(g_ctx, NSTAR_TX_RATE_32K);

    size_t total = (NSTAR_FRAME_SIZE_BYTES * 2) + 1;
    uint8_t *payload = malloc(total);
    TEST_ASSERT_NOT_NULL(payload);
    for (size_t i = 0; i < total; i++) payload[i] = (uint8_t)(i & 0xFF);

    TEST_ASSERT_EQUAL(NSTAR_OK, nstar_tx_write(g_ctx, payload, total));

    size_t wlen = 0;
    const uint8_t *w = nstar_mock_data_get_written(&wlen);
    TEST_ASSERT_EQUAL_UINT(total, wlen);
    TEST_ASSERT_EQUAL_MEMORY(payload, w, total);
    free(payload);
}

/**
 * Multiple sequential tx_write calls accumulate bytes correctly.
 */
void test_tx_write_multiple_calls_accumulate(void)
{
    queue_tx_start_happy();
    nstar_tx_start(g_ctx, NSTAR_TX_RATE_32K);

    uint8_t a[50], b[30];
    memset(a, 0xAA, sizeof(a));
    memset(b, 0xBB, sizeof(b));

    TEST_ASSERT_EQUAL(NSTAR_OK, nstar_tx_write(g_ctx, a, sizeof(a)));
    TEST_ASSERT_EQUAL(NSTAR_OK, nstar_tx_write(g_ctx, b, sizeof(b)));

    size_t wlen = 0;
    const uint8_t *w = nstar_mock_data_get_written(&wlen);
    TEST_ASSERT_EQUAL_UINT(80, wlen);
    TEST_ASSERT_EQUAL_MEMORY(a, w,      50);
    TEST_ASSERT_EQUAL_MEMORY(b, w + 50, 30);
}

/* =========================================================================
 * nstar_tx_stop tests
 * =========================================================================
 */

void test_tx_stop_without_start_is_ok(void)
{
    /* Calling stop when not active should return OK (idempotent) */
    TEST_ASSERT_EQUAL(NSTAR_OK, nstar_tx_stop(g_ctx));
    TEST_ASSERT_FALSE(g_tx_cb_fired);
}

void test_tx_stop_null_ctx_returns_not_init(void)
{
    TEST_ASSERT_EQUAL(NSTAR_ERR_NOT_INIT, nstar_tx_stop(NULL));
}

/**
 * Happy path stop: sends Standby command, stops clock, fires callback.
 */
void test_tx_stop_sends_standby_fires_callback(void)
{
    queue_tx_start_happy();
    nstar_tx_start(g_ctx, NSTAR_TX_RATE_32K);

    /* Write some data so bytes_sent > 0 */
    uint8_t data[64];
    memset(data, 0xCD, sizeof(data));
    nstar_tx_write(g_ctx, data, sizeof(data));

    queue_tx_stop_happy();
    TEST_ASSERT_EQUAL(NSTAR_OK, nstar_tx_stop(g_ctx));

    /* Callback must have fired */
    TEST_ASSERT_TRUE(g_tx_cb_fired);
    TEST_ASSERT_EQUAL_UINT(64, g_tx_cb_bytes);

    /* Clock must be stopped */
    TEST_ASSERT_FALSE(nstar_mock_data_clock_is_running());
}

/**
 * Verify that Standby frame (W 0x40=0x00) is in UART writes after stop.
 */
void test_tx_stop_sends_standby_frame(void)
{
    queue_tx_start_happy();
    nstar_tx_start(g_ctx, NSTAR_TX_RATE_32K);
    queue_tx_stop_happy();
    nstar_tx_stop(g_ctx);

    size_t wlen = 0;
    const uint8_t *w = nstar_mock_uart_get_written(&wlen);

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
void test_tx_stop_resets_byte_counter(void)
{
    /* Session 1 */
    queue_tx_start_happy();
    nstar_tx_start(g_ctx, NSTAR_TX_RATE_32K);
    uint8_t d[100]; memset(d, 0, sizeof(d));
    nstar_tx_write(g_ctx, d, 100);
    queue_tx_stop_happy();
    nstar_tx_stop(g_ctx);
    TEST_ASSERT_EQUAL_UINT(100, g_tx_cb_bytes);

    /* Session 2 — fresh start should report only session-2 bytes */
    nstar_mock_reset();
    g_tx_cb_bytes = 0;
    g_tx_cb_fired = 0;

    queue_tx_start_happy();
    nstar_tx_start(g_ctx, NSTAR_TX_RATE_32K);
    nstar_tx_write(g_ctx, d, 40);
    queue_tx_stop_happy();
    nstar_tx_stop(g_ctx);
    TEST_ASSERT_EQUAL_UINT(40, g_tx_cb_bytes);
}

/**
 * Full TX session: start → write → stop — verify byte count in callback
 * matches total written.
 */
void test_tx_full_session_byte_count_matches(void)
{
    queue_tx_start_happy();
    TEST_ASSERT_EQUAL(NSTAR_OK, nstar_tx_start(g_ctx, NSTAR_TX_RATE_256K));

    uint8_t chunk[512];
    memset(chunk, 0x55, sizeof(chunk));

    /* Write 5 chunks = 2560 bytes */
    for (int i = 0; i < 5; i++) {
        TEST_ASSERT_EQUAL(NSTAR_OK,
            nstar_tx_write(g_ctx, chunk, sizeof(chunk)));
    }

    queue_tx_stop_happy();
    TEST_ASSERT_EQUAL(NSTAR_OK, nstar_tx_stop(g_ctx));

    TEST_ASSERT_TRUE(g_tx_cb_fired);
    TEST_ASSERT_EQUAL_UINT(5 * 512, g_tx_cb_bytes);
}

/* =========================================================================
 * Test runner
 * =========================================================================
 */

int main(void)
{
    UNITY_BEGIN();

    /* tx_get_status */
    RUN_TEST(test_tx_get_status_null_ctx_returns_param_error);
    RUN_TEST(test_tx_get_status_null_out_returns_param_error);
    RUN_TEST(test_tx_get_status_clock_detected);
    RUN_TEST(test_tx_get_status_modulation_active);
    RUN_TEST(test_tx_get_status_no_clock);

    /* tx_start */
    RUN_TEST(test_tx_start_null_ctx_returns_not_init);
    RUN_TEST(test_tx_start_success);
    RUN_TEST(test_tx_start_sends_frames_in_correct_order);
    RUN_TEST(test_tx_start_sets_correct_rate_register);
    RUN_TEST(test_tx_start_no_clock_returns_no_clock_error);
    RUN_TEST(test_tx_start_rate_write_timeout_returns_timeout);
    RUN_TEST(test_tx_start_when_already_active_returns_busy);

    /* tx_write */
    RUN_TEST(test_tx_write_null_ctx_returns_not_init);
    RUN_TEST(test_tx_write_null_buf_returns_param_error);
    RUN_TEST(test_tx_write_without_start_returns_busy);
    RUN_TEST(test_tx_write_delivers_all_bytes);
    RUN_TEST(test_tx_write_exact_frame_size);
    RUN_TEST(test_tx_write_multi_chunk_preserves_all_bytes);
    RUN_TEST(test_tx_write_multiple_calls_accumulate);

    /* tx_stop */
    RUN_TEST(test_tx_stop_null_ctx_returns_not_init);
    RUN_TEST(test_tx_stop_without_start_is_ok);
    RUN_TEST(test_tx_stop_sends_standby_fires_callback);
    RUN_TEST(test_tx_stop_sends_standby_frame);
    RUN_TEST(test_tx_stop_resets_byte_counter);
    RUN_TEST(test_tx_full_session_byte_count_matches);

    return UNITY_END();
}
