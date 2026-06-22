/**
 * @file  test_rx.c
 * @brief Stage 4 unit tests — RX pipeline.
 *
 * Tests cover:
 *   - nstar_rx_configure()
 *   - nstar_rx_get_status() register decode
 *   - nstar_rx_get_link_quality() multi-register decode
 *   - rx_thread_func() state machine via GPIO edge sequences
 *
 * Threading notes:
 *   nstar_init() now spawns the rx_thread, fault_thread, and health_thread.
 *   Tests that exercise the rx_thread queue GPIO edges via the mock and then
 *   wait for the on_frame_received / on_lock_acquired / on_lock_lost
 *   callbacks using a pthread_cond_t.
 *
 *   setUp() calls nstar_init() which starts all three threads.
 *   The fault and health threads block on GPIO edges / sleep — they will
 *   not interfere with RX tests because their GPIO fds differ.
 */

#include "unity/unity.h"
#include "nstar.h"
#include "nstar_hal_mock.h"
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
    uint8_t         frame_buf[NSTAR_FRAME_SIZE_BYTES];
    size_t          frame_len;
    int             lock_acquired;
    int             lock_lost;
    nstar_fault_source_t fault_source;
    int             fault_fired;
} cb_sync_t;

static cb_sync_t g_sync;

static void sync_init(void)
{
    pthread_mutex_init(&g_sync.mu, NULL);
    pthread_cond_init(&g_sync.cv, NULL);
    memset(&g_sync.frame_buf, 0, sizeof(g_sync.frame_buf));
    g_sync.fired         = 0;
    g_sync.frame_len     = 0;
    g_sync.lock_acquired = 0;
    g_sync.lock_lost     = 0;
    g_sync.fault_fired   = 0;
}

static void sync_destroy(void)
{
    pthread_mutex_destroy(&g_sync.mu);
    pthread_cond_destroy(&g_sync.cv);
}

/** Wait up to timeout_ms for g_sync.fired to become non-zero. */
static int sync_wait(int timeout_ms)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec  += timeout_ms / 1000;
    ts.tv_nsec += (timeout_ms % 1000) * 1000000L;
    if (ts.tv_nsec >= 1000000000L) { ts.tv_sec++; ts.tv_nsec -= 1000000000L; }

    pthread_mutex_lock(&g_sync.mu);
    int rc = 0;
    while (!g_sync.fired && rc == 0) {
        rc = pthread_cond_timedwait(&g_sync.cv, &g_sync.mu, &ts);
    }
    int result = g_sync.fired;
    pthread_mutex_unlock(&g_sync.mu);
    return result;
}

/**
 * Wait up to timeout_ms specifically for on_frame_received to fire.
 * on_lock_acquired and on_lock_lost also call sync_signal() and set the
 * generic g_sync.fired flag, so a plain sync_wait() can return early on
 * the wrong event. This variant loops past those, re-arming the wait,
 * until frame_len becomes non-zero or the deadline passes.
 */
static int sync_wait_for_frame(int timeout_ms)
{
    struct timespec deadline;
    clock_gettime(CLOCK_REALTIME, &deadline);
    deadline.tv_sec  += timeout_ms / 1000;
    deadline.tv_nsec += (timeout_ms % 1000) * 1000000L;
    if (deadline.tv_nsec >= 1000000000L) {
        deadline.tv_sec++; deadline.tv_nsec -= 1000000000L;
    }

    pthread_mutex_lock(&g_sync.mu);
    while (g_sync.frame_len == 0) {
        int rc = pthread_cond_timedwait(&g_sync.cv, &g_sync.mu, &deadline);
        if (rc != 0) break;   /* timed out */
        /* Spurious wake from a different callback (lock_acquired/lock_lost):
         * loop back and keep waiting until frame_len is set or deadline hits. */
    }
    int got = (g_sync.frame_len != 0);
    pthread_mutex_unlock(&g_sync.mu);
    return got;
}

static void sync_signal(void)
{
    pthread_mutex_lock(&g_sync.mu);
    g_sync.fired = 1;
    pthread_cond_signal(&g_sync.cv);
    pthread_mutex_unlock(&g_sync.mu);
}

/* =========================================================================
 * Callbacks
 * =========================================================================
 */

static void on_frame_received(const uint8_t *buf, size_t len)
{
    if (len > sizeof(g_sync.frame_buf)) len = sizeof(g_sync.frame_buf);
    memcpy(g_sync.frame_buf, buf, len);
    g_sync.frame_len = len;
    sync_signal();
}

static void on_tx_complete(size_t n) { (void)n; }

static void on_fault(nstar_fault_source_t src)
{
    g_sync.fault_source = src;
    g_sync.fault_fired  = 1;
    sync_signal();
}

static void on_lock_acquired(void)
{
    g_sync.lock_acquired = 1;
    sync_signal();
}

static void on_lock_lost(void)
{
    g_sync.lock_lost = 1;
    sync_signal();
}

static const nstar_callbacks_t k_cb = {
    .on_frame_received = on_frame_received,
    .on_tx_complete    = on_tx_complete,
    .on_fault          = on_fault,
    .on_lock_acquired  = on_lock_acquired,
    .on_lock_lost      = on_lock_lost,
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

static const nstar_config_t k_cfg = {
    .uart_fd          = FD_UART,
    .data_fd          = FD_DATA,
    .gpio_lock_detect = FD_LD,
    .gpio_data_valid  = FD_DV,
    .gpio_fault_n     = FD_FN,
    .gpio_reset_n     = FD_RST,
};

static nstar_ctx_t *g_ctx = NULL;

static void qr(const char *s)
{
    nstar_mock_uart_queue_response((const uint8_t *)s, strlen(s));
}

/* Queue the 3-response sequence for a successful startup_sequence
 * (V, R 0x06, W 0x10 — the redundant R 0x08/R 0x09 reads were removed
 * from nstar_core.c since FPGA_OPTION is already in the V response). */
static void queue_startup_happy(void)
{
    qr("<V12:010018230042620307:F832>");
    qr("<R02:62:7D57>");
    qr("<A02:00:466C>");
}

void setUp(void)
{
    nstar_mock_reset();
    sync_init();
    /* Pre-set DATA_VALID and FAULT_N to idle state */
    nstar_mock_gpio_set_value(FD_DV, 0);
    nstar_mock_gpio_set_value(FD_FN, 1);  /* FAULT_N idle = HIGH */
    nstar_result_t rc = nstar_init(&k_cfg, &k_cb, &g_ctx);
    TEST_ASSERT_EQUAL(NSTAR_OK, rc);

    /*
     * nstar_init() leaves the module in STARTING, not READY.
     * nstar_rx_configure() requires READY. Run startup_sequence() here
     * so all RX tests start from READY, matching the real init flow.
     */
    queue_startup_happy();
    rc = nstar_startup_sequence(g_ctx);
    TEST_ASSERT_EQUAL(NSTAR_OK, rc);
    TEST_ASSERT_EQUAL(NSTAR_MODULE_READY, nstar_get_module_state(g_ctx));
}

void tearDown(void)
{
    if (g_ctx) { nstar_deinit(g_ctx); g_ctx = NULL; }
    sync_destroy();
}

/* =========================================================================
 * nstar_rx_configure tests
 * =========================================================================
 */

void test_rx_configure_null_ctx_returns_not_init(void)
{
    TEST_ASSERT_EQUAL(NSTAR_ERR_NOT_INIT,
        nstar_rx_configure(NULL, NSTAR_RX_RATE_16K));
}

/**
 * nstar_rx_configure sends W 0x22 = rate_code.
 * For NSTAR_RX_RATE_16K (0x1F), expect frame <W04:221F:CCCC>.
 */
void test_rx_configure_sends_correct_register_write(void)
{
    qr("<A02:00:466C>");  /* W 0x22 ACK */
    TEST_ASSERT_EQUAL(NSTAR_OK,
        nstar_rx_configure(g_ctx, NSTAR_RX_RATE_16K));

    size_t wlen = 0;
    const uint8_t *w = nstar_mock_uart_get_written(&wlen);

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
 * nstar_rx_get_status tests
 * =========================================================================
 */

void test_rx_get_status_null_out_returns_param_error(void)
{
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM, nstar_rx_get_status(g_ctx, NULL));
}

/**
 * RX_STATUS = 0x0F → all four lock bits set.
 */
void test_rx_get_status_all_bits_set(void)
{
    qr("<R02:0F:0B6A>");
    nstar_rx_status_t st;
    memset(&st, 0, sizeof(st));
    TEST_ASSERT_EQUAL(NSTAR_OK, nstar_rx_get_status(g_ctx, &st));
    TEST_ASSERT_TRUE(st.carrier_detect);
    TEST_ASSERT_TRUE(st.carrier_lock);
    TEST_ASSERT_TRUE(st.bit_lock);
    TEST_ASSERT_TRUE(st.data_valid);
    TEST_ASSERT_EQUAL_HEX8(0x0F, st.raw);
}

/**
 * RX_STATUS = 0x01 → only carrier_detect set, others clear.
 */
void test_rx_get_status_carrier_only(void)
{
    qr("<R02:01:9AA4>");
    nstar_rx_status_t st;
    memset(&st, 0, sizeof(st));
    TEST_ASSERT_EQUAL(NSTAR_OK, nstar_rx_get_status(g_ctx, &st));
    TEST_ASSERT_TRUE(st.carrier_detect);
    TEST_ASSERT_FALSE(st.carrier_lock);
    TEST_ASSERT_FALSE(st.bit_lock);
    TEST_ASSERT_FALSE(st.data_valid);
}

/* =========================================================================
 * nstar_rx_get_link_quality tests
 * =========================================================================
 */

void test_rx_get_link_quality_null_out_returns_param_error(void)
{
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM,
        nstar_rx_get_link_quality(g_ctx, NULL));
}

/**
 * Eb = 0x001000 = 4096, N0 = 0x000100 = 256
 * Eb/N0 = 20*log10(4096/256) - 3 = 20*log10(16) - 3 = 20*1.204 - 3 = 21.1 dB
 * IQ power = 0x0040 = 64, AGC = 0x0020 = 32
 * RSSI = 64 - 32 = 32 dBm
 * Freq shift = 0x000080 = 128 / 8 = 16 Hz
 */
void test_rx_get_link_quality_decodes_correctly(void)
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

    nstar_link_quality_t lq;
    memset(&lq, 0, sizeof(lq));
    TEST_ASSERT_EQUAL(NSTAR_OK, nstar_rx_get_link_quality(g_ctx, &lq));

    /* Eb/N0 ≈ 21.1 dB — accept ±0.5 for float precision */
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 21.1f, lq.eb_no_db);

    /* RSSI = 64 - 32 = 32 */
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 32.0f, lq.rssi_dbm);

    /* Frequency shift = 128/8 = 16 Hz */
    TEST_ASSERT_EQUAL_INT32(16, lq.freq_shift_hz);
}

/**
 * Negative frequency shift — sign extension of 24-bit to 32-bit.
 * 0xFFFF80 = -128 (two's complement 24-bit) → /8 = -16 Hz
 */
void test_rx_get_link_quality_negative_freq_shift(void)
{
    /* Eb, N0, IQ, AGC — reuse simple values */
    qr("<R02:00:A995>"); qr("<R02:10:9EA5>"); qr("<R02:00:A995>");
    qr("<R02:00:A995>"); qr("<R02:01:9AA4>"); qr("<R02:00:A995>");
    qr("<R02:00:A995>"); qr("<R02:40:7555>");
    qr("<R02:00:A995>"); qr("<R02:20:C7F5>");

    /* Freq shift: 0xFFFF80 = -128 signed-24, /8 = -16 Hz */
    /* Compute CRC for each byte frame */
    qr("<R02:FF:61C2>"); qr("<R02:FF:61C2>"); qr("<R02:80:0034>");

    nstar_link_quality_t lq;
    memset(&lq, 0, sizeof(lq));
    TEST_ASSERT_EQUAL(NSTAR_OK, nstar_rx_get_link_quality(g_ctx, &lq));
    TEST_ASSERT_EQUAL_INT32(-16, lq.freq_shift_hz);
}

/* =========================================================================
 * RX thread state machine tests
 * =========================================================================
 */

/**
 * LOCK_DETECT rising → on_lock_acquired callback fires.
 * Queue: LOCK_DETECT rising edge, then DATA_VALID edge times out.
 */
void test_rx_thread_lock_detect_fires_lock_acquired_cb(void)
{
    /* Reset sync state for this specific check */
    g_sync.lock_acquired = 0;
    g_sync.fired         = 0;

    /* Queue LOCK_DETECT rising edge */
    nstar_mock_gpio_queue_edge(FD_LD, NSTAR_GPIO_EDGE_RISING, 0);

    /* Wait for on_lock_acquired to fire (up to 1 s) */
    int got = sync_wait(1000);
    TEST_ASSERT_TRUE_MESSAGE(got, "on_lock_acquired callback did not fire");
    TEST_ASSERT_TRUE(g_sync.lock_acquired);
}

/**
 * Full lock sequence: LOCK_DETECT rising, DATA_VALID rising,
 * data available → on_frame_received fires with correct bytes.
 */
void test_rx_thread_full_lock_sequence_delivers_frame(void)
{
    /* Supply 64 bytes to the data interface */
    uint8_t payload[64];
    for (int i = 0; i < 64; i++) payload[i] = (uint8_t)i;
    nstar_mock_data_supply_read(payload, sizeof(payload));

    /* Set DATA_VALID HIGH so gpio_read() returns 1 after lock */
    nstar_mock_gpio_set_value(FD_DV, 1);

    /* Queue GPIO edges: LOCK_DETECT rising, then DATA_VALID rising */
    nstar_mock_gpio_queue_edge(FD_LD, NSTAR_GPIO_EDGE_RISING, 0);
    nstar_mock_gpio_queue_edge(FD_DV, NSTAR_GPIO_EDGE_RISING, 0);

    /* Wait specifically for on_frame_received (not lock_acquired) */
    int got = sync_wait_for_frame(2000);
    TEST_ASSERT_TRUE_MESSAGE(got, "on_frame_received callback did not fire");
    TEST_ASSERT_EQUAL_UINT(64, g_sync.frame_len);
    TEST_ASSERT_EQUAL_MEMORY(payload, g_sync.frame_buf, 64);
}

/**
 * DATA_VALID falls mid-session → on_lock_lost fires.
 * Supply one frame of data; after it is read, DATA_VALID goes LOW.
 */
void test_rx_thread_data_valid_falling_fires_lock_lost_cb(void)
{
    /*
     * Supply first batch: thread reads this, fires on_frame_received.
     * After frame callback, test drops DATA_VALID to 0.
     * Thread then calls gpio_read() = 0, exits read loop, fires on_lock_lost.
     */
    uint8_t payload[32];
    memset(payload, 0xAB, sizeof(payload));
    nstar_mock_data_supply_read(payload, sizeof(payload));

    /* DATA_VALID HIGH so gpio_read returns 1 during the read loop */
    nstar_mock_gpio_set_value(FD_DV, 1);

    nstar_mock_gpio_queue_edge(FD_LD, NSTAR_GPIO_EDGE_RISING, 0);
    nstar_mock_gpio_queue_edge(FD_DV, NSTAR_GPIO_EDGE_RISING, 0);

    /* Wait specifically for on_frame_received (not lock_acquired) */
    int got = sync_wait_for_frame(2000);
    TEST_ASSERT_TRUE_MESSAGE(got, "Frame callback did not fire");
    TEST_ASSERT_EQUAL_UINT(32, g_sync.frame_len);

    /*
     * Drop DATA_VALID BEFORE resetting sync state.
     * The thread will see gpio_read()=0 on its next loop iteration
     * and call on_lock_lost, which calls sync_signal().
     * We then reset g_sync.fired under the mutex so we can wait again.
     */
    nstar_mock_gpio_set_value(FD_DV, 0);

    /* Reset sync state under the mutex to avoid race */
    pthread_mutex_lock(&g_sync.mu);
    g_sync.fired     = 0;
    g_sync.lock_lost = 0;
    pthread_mutex_unlock(&g_sync.mu);

    /* Wait for on_lock_lost (up to 2 s) */
    got = sync_wait(2000);
    TEST_ASSERT_TRUE_MESSAGE(got, "on_lock_lost callback did not fire");
    TEST_ASSERT_TRUE(g_sync.lock_lost);
}

/* =========================================================================
 * Test runner
 * =========================================================================
 */

int main(void)
{
    UNITY_BEGIN();

    /* rx_configure */
    RUN_TEST(test_rx_configure_null_ctx_returns_not_init);
    RUN_TEST(test_rx_configure_sends_correct_register_write);

    /* rx_get_status */
    RUN_TEST(test_rx_get_status_null_out_returns_param_error);
    RUN_TEST(test_rx_get_status_all_bits_set);
    RUN_TEST(test_rx_get_status_carrier_only);

    /* rx_get_link_quality */
    RUN_TEST(test_rx_get_link_quality_null_out_returns_param_error);
    RUN_TEST(test_rx_get_link_quality_decodes_correctly);
    RUN_TEST(test_rx_get_link_quality_negative_freq_shift);

    /* rx_thread state machine */
    RUN_TEST(test_rx_thread_lock_detect_fires_lock_acquired_cb);
    RUN_TEST(test_rx_thread_full_lock_sequence_delivers_frame);
    RUN_TEST(test_rx_thread_data_valid_falling_fires_lock_lost_cb);

    return UNITY_END();
}
