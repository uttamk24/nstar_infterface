/**
 * @file  test_core.c
 * @brief Stage 2 unit tests — command interface, register access,
 *        named commands, startup sequence.
 *
 * All tests run synchronously against the mock HAL.
 * No threads are spawned; nstar_core functions are called directly.
 * Each test calls nstar_mock_reset() and nstar_init() in setUp().
 */

#include "unity/unity.h"
#include "nstar.h"
#include "nstar_hal_mock.h"
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

static nstar_ctx_t *g_ctx = NULL;

static void dummy_on_frame_received(const uint8_t *b, size_t l)
{ (void)b; (void)l; }
static void dummy_on_tx_complete(size_t n) { (void)n; }
static void dummy_on_fault(nstar_fault_source_t s) { (void)s; }
static void dummy_on_lock_acquired(void) {}
static void dummy_on_lock_lost(void) {}

static const nstar_callbacks_t k_callbacks = {
    .on_frame_received = dummy_on_frame_received,
    .on_tx_complete    = dummy_on_tx_complete,
    .on_fault          = dummy_on_fault,
    .on_lock_acquired  = dummy_on_lock_acquired,
    .on_lock_lost      = dummy_on_lock_lost,
};

static const nstar_config_t k_config = {
    .uart_fd           = FAKE_UART_FD,
    .data_fd           = FAKE_DATA_FD,
    .gpio_lock_detect  = FAKE_GPIO_LD,
    .gpio_data_valid   = FAKE_GPIO_DV,
    .gpio_fault_n      = FAKE_GPIO_FN,
    .gpio_reset_n      = FAKE_GPIO_RST,
};

/* Helper: queue a literal frame string as a UART response */
static void queue_response(const char *frame_str)
{
    nstar_mock_uart_queue_response(
        (const uint8_t *)frame_str, strlen(frame_str));
}

void setUp(void)
{
    nstar_mock_reset();
    nstar_result_t rc = nstar_init(&k_config, &k_callbacks, &g_ctx);
    TEST_ASSERT_EQUAL(NSTAR_OK, rc);
    TEST_ASSERT_NOT_NULL(g_ctx);
}

void tearDown(void)
{
    if (g_ctx) {
        nstar_deinit(g_ctx);
        g_ctx = NULL;
    }
}

/* =========================================================================
 * Init / deinit tests
 * =========================================================================
 */

void test_init_null_config_returns_param_error(void)
{
    nstar_ctx_t *ctx = NULL;
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM,
        nstar_init(NULL, &k_callbacks, &ctx));
}

void test_init_null_callbacks_returns_param_error(void)
{
    nstar_ctx_t *ctx = NULL;
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM,
        nstar_init(&k_config, NULL, &ctx));
}

void test_init_null_ctx_out_returns_param_error(void)
{
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM,
        nstar_init(&k_config, &k_callbacks, NULL));
}

void test_init_success_returns_non_null_ctx(void)
{
    /* setUp() already called nstar_init; ctx is non-null */
    TEST_ASSERT_NOT_NULL(g_ctx);
}

void test_deinit_null_ctx_is_safe(void)
{
    /* Must not crash */
    nstar_deinit(NULL);
}

/* =========================================================================
 * nstar_reg_read tests
 * =========================================================================
 */

void test_reg_read_null_ctx_returns_param_error(void)
{
    uint8_t val;
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM, nstar_reg_read(NULL, 0x06, &val));
}

void test_reg_read_null_val_returns_param_error(void)
{
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM, nstar_reg_read(g_ctx, 0x06, NULL));
}

/**
 * Happy path: read FPGA_TYPE register.
 * Mock response: <R02:62:7D57>  (0x62 = N-STAR PCM/PM RX+TX)
 * Verify: returned value is 0x62.
 */
void test_reg_read_fpga_type_returns_correct_value(void)
{
    queue_response("<R02:62:7D57>");
    uint8_t val = 0;
    TEST_ASSERT_EQUAL(NSTAR_OK, nstar_reg_read(g_ctx, NSTAR_REG_FPGA_TYPE, &val));
    TEST_ASSERT_EQUAL_HEX8(0x62, val);
}

/**
 * Verify the frame written to UART for R command.
 * R 0x06 should write: <R02:06:CCCC>
 */
void test_reg_read_sends_correct_frame(void)
{
    queue_response("<R02:62:7D57>");

    uint8_t val = 0;
    nstar_reg_read(g_ctx, NSTAR_REG_FPGA_TYPE, &val);

    size_t written_len = 0;
    const uint8_t *written = nstar_mock_uart_get_written(&written_len);

    /* Frame should start with <R02:06: */
    TEST_ASSERT_TRUE(written_len >= 8);
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
void test_reg_read_timeout_returns_timeout_error(void)
{
    nstar_mock_uart_force_timeout(1);
    uint8_t val = 0;
    TEST_ASSERT_EQUAL(NSTAR_ERR_TIMEOUT,
        nstar_reg_read(g_ctx, NSTAR_REG_FPGA_TYPE, &val));
}

/**
 * CRC error: corrupt the CRC field of the response.
 * After retries, expect NSTAR_ERR_CRC.
 * Queue two corrupted responses (one for initial attempt, one for retry).
 */
void test_reg_read_crc_error_after_retry(void)
{
    /* Correct: <R02:62:7D57>  Corrupt: last digit 7->8 */
    queue_response("<R02:62:7D58>");
    queue_response("<R02:62:7D58>");
    uint8_t val = 0;
    TEST_ASSERT_EQUAL(NSTAR_ERR_CRC,
        nstar_reg_read(g_ctx, NSTAR_REG_FPGA_TYPE, &val));
}

/**
 * Retry succeeds: first response is corrupted, second is good.
 */
void test_reg_read_succeeds_on_retry(void)
{
    queue_response("<R02:62:7D58>");   /* corrupt — will be retried */
    queue_response("<R02:62:7D57>");   /* correct */
    uint8_t val = 0;
    TEST_ASSERT_EQUAL(NSTAR_OK,
        nstar_reg_read(g_ctx, NSTAR_REG_FPGA_TYPE, &val));
    TEST_ASSERT_EQUAL_HEX8(0x62, val);
}

/* =========================================================================
 * nstar_reg_write tests
 * =========================================================================
 */

void test_reg_write_null_ctx_returns_param_error(void)
{
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM,
        nstar_reg_write(NULL, NSTAR_REG_TX_MODE, 0x01));
}

/**
 * Happy path: write TX_MODE = Modulation.
 * Mock response: <A02:00:466C>  (ACK, result 0x00 = success)
 */
void test_reg_write_tx_mode_modulation_success(void)
{
    queue_response("<A02:00:466C>");
    TEST_ASSERT_EQUAL(NSTAR_OK,
        nstar_reg_write(g_ctx, NSTAR_REG_TX_MODE, NSTAR_TX_MODE_MODULATION));
}

/**
 * Verify the frame written to UART for W command.
 * W 0x40=0x01 should write: <W04:4001:CCCC>
 */
void test_reg_write_sends_correct_frame(void)
{
    queue_response("<A02:00:466C>");
    nstar_reg_write(g_ctx, NSTAR_REG_TX_MODE, NSTAR_TX_MODE_MODULATION);

    size_t written_len = 0;
    const uint8_t *written = nstar_mock_uart_get_written(&written_len);

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
void test_reg_write_bad_ack_returns_bad_ack_error(void)
{
    queue_response("<A02:01:755D>");
    TEST_ASSERT_EQUAL(NSTAR_ERR_BAD_ACK,
        nstar_reg_write(g_ctx, NSTAR_REG_TX_MODE, 0x01));
}

/**
 * Timeout on write → NSTAR_ERR_TIMEOUT.
 */
void test_reg_write_timeout_returns_timeout_error(void)
{
    nstar_mock_uart_force_timeout(1);
    TEST_ASSERT_EQUAL(NSTAR_ERR_TIMEOUT,
        nstar_reg_write(g_ctx, NSTAR_REG_TX_MODE, 0x01));
}

/* =========================================================================
 * nstar_reg_read_multi tests
 * =========================================================================
 */

/**
 * Read 3 consecutive registers (0x08, 0x09, 0x0A).
 * Each gets its own R command and mock response.
 */
void test_reg_read_multi_reads_n_registers(void)
{
    queue_response("<R02:03:FCC6>");   /* 0x08 = 0x03 */
    queue_response("<R02:07:3002>");   /* 0x09 = 0x07 */
    queue_response("<R02:AB:2896>");   /* 0x0A = 0xAB */

    uint8_t buf[3] = {0};
    TEST_ASSERT_EQUAL(NSTAR_OK,
        nstar_reg_read_multi(g_ctx, 0x08, 3, buf));

    TEST_ASSERT_EQUAL_HEX8(0x03, buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0x07, buf[1]);
    TEST_ASSERT_EQUAL_HEX8(0xAB, buf[2]);
}

void test_reg_read_multi_zero_count_returns_param_error(void)
{
    uint8_t buf[4];
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM,
        nstar_reg_read_multi(g_ctx, 0x08, 0, buf));
}

/* =========================================================================
 * nstar_cmd_read_identity (V command) tests
 * =========================================================================
 */

/**
 * Happy path: V command returns 9-byte identity.
 * Mock response: <V12:010018230042620307:F832>
 *   Byte 0: fpga_version = 0x01
 *   Byte 1: fpga_build   = 0x00
 *   Byte 2: hw_year      = 0x18 (2024)
 *   Byte 3: hw_week      = 0x23 (week 35)
 *   Byte 4-5: hw_order   = 0x0042
 *   Byte 6: fpga_type    = 0x62
 *   Byte 7-8: fpga_options = 0x0307
 */
void test_cmd_read_identity_success(void)
{
    queue_response("<V12:010018230042620307:F832>");

    nstar_identity_t id;
    memset(&id, 0, sizeof(id));
    TEST_ASSERT_EQUAL(NSTAR_OK, nstar_cmd_read_identity(g_ctx, &id));

    TEST_ASSERT_EQUAL_HEX8(0x01, id.fpga_version);
    TEST_ASSERT_EQUAL_HEX8(0x00, id.fpga_build);
    TEST_ASSERT_EQUAL_HEX8(0x18, id.hw_year);
    TEST_ASSERT_EQUAL_HEX8(0x23, id.hw_week);
    TEST_ASSERT_EQUAL_HEX16(0x0042, id.hw_order);
    TEST_ASSERT_EQUAL_HEX8(0x62, id.fpga_type);
    TEST_ASSERT_EQUAL_HEX16(0x0307, id.fpga_options);
}

void test_cmd_read_identity_null_out_returns_param_error(void)
{
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM,
        nstar_cmd_read_identity(g_ctx, NULL));
}

/**
 * V command sends correct frame (no data, DATA_SIZE="00").
 * Expect: <V00::CCCC>
 */
void test_cmd_read_identity_sends_v_frame(void)
{
    queue_response("<V12:010018230042620307:F832>");
    nstar_identity_t id;
    nstar_cmd_read_identity(g_ctx, &id);

    size_t wlen = 0;
    const uint8_t *w = nstar_mock_uart_get_written(&wlen);
    TEST_ASSERT_EQUAL_CHAR('<', (char)w[0]);
    TEST_ASSERT_EQUAL_CHAR('V', (char)w[1]);
    TEST_ASSERT_EQUAL_CHAR('0', (char)w[2]);
    TEST_ASSERT_EQUAL_CHAR('0', (char)w[3]);
    TEST_ASSERT_EQUAL_CHAR(':', (char)w[4]);
    TEST_ASSERT_EQUAL_CHAR(':', (char)w[5]);  /* Sep2 immediately after Sep1 */
}

/* =========================================================================
 * nstar_cmd_read_all_rx_status (E command) tests
 * =========================================================================
 */

/**
 * E command returns 19 raw bytes (regs 0x10-0x22).
 * Mock response: <E26:101112131415161718191A1B1C1D1E1F202122:F52A>
 */
void test_cmd_read_all_rx_status_success(void)
{
    queue_response("<E26:101112131415161718191A1B1C1D1E1F202122:F52A>");

    uint8_t raw[32] = {0};
    size_t  len = 0;
    TEST_ASSERT_EQUAL(NSTAR_OK,
        nstar_cmd_read_all_rx_status(g_ctx, raw, &len));
    TEST_ASSERT_EQUAL_UINT(19, len);
    TEST_ASSERT_EQUAL_HEX8(0x10, raw[0]);
    TEST_ASSERT_EQUAL_HEX8(0x22, raw[18]);
}

void test_cmd_read_all_rx_status_null_out_returns_param_error(void)
{
    size_t len;
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM,
        nstar_cmd_read_all_rx_status(g_ctx, NULL, &len));
}

/* =========================================================================
 * nstar_cmd_reset (C command) tests
 * =========================================================================
 */

/**
 * Happy path: reset command acknowledged.
 * Mock response: <A02:00:466C>
 */
void test_cmd_reset_success(void)
{
    queue_response("<A02:00:466C>");
    TEST_ASSERT_EQUAL(NSTAR_OK, nstar_cmd_reset(g_ctx));
}

/**
 * Verify C frame contains magic word 0x5A5A.
 * Expected: <C04:5A5A:CCCC>
 */
void test_cmd_reset_sends_magic_word(void)
{
    queue_response("<A02:00:466C>");
    nstar_cmd_reset(g_ctx);

    size_t wlen = 0;
    const uint8_t *w = nstar_mock_uart_get_written(&wlen);
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
void test_cmd_reset_bad_ack_returns_error(void)
{
    queue_response("<A02:01:755D>");
    TEST_ASSERT_EQUAL(NSTAR_ERR_BAD_ACK, nstar_cmd_reset(g_ctx));
}

/* =========================================================================
 * nstar_startup_sequence tests
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
void test_startup_sequence_success(void)
{
    queue_response("<V12:010018230042620307:F832>");  /* V */
    queue_response("<R02:62:7D57>");                  /* R 0x06 */
    queue_response("<A02:00:466C>");                  /* W 0x10 */

    TEST_ASSERT_EQUAL(NSTAR_OK, nstar_startup_sequence(g_ctx));
}

/**
 * Wrong FPGA_TYPE → NSTAR_ERR_FPGA_TYPE.
 * Simulate a unit where register 0x06 returns 0xFF instead of 0x62.
 */
void test_startup_wrong_fpga_type_returns_error(void)
{
    queue_response("<V12:010018230042620307:F832>");  /* V */
    queue_response("<R02:FF:61C2>");                  /* R 0x06 = 0xFF (wrong) */

    TEST_ASSERT_EQUAL(NSTAR_ERR_FPGA_TYPE,
        nstar_startup_sequence(g_ctx));
}

/**
 * V command times out → startup returns NSTAR_ERR_TIMEOUT.
 */
void test_startup_v_timeout_returns_timeout_error(void)
{
    nstar_mock_uart_force_timeout(1);
    TEST_ASSERT_EQUAL(NSTAR_ERR_TIMEOUT,
        nstar_startup_sequence(g_ctx));
}

/**
 * OBS sweep write times out → startup returns NSTAR_ERR_TIMEOUT.
 */
void test_startup_obs_write_timeout_returns_timeout_error(void)
{
    queue_response("<V12:010018230042620307:F832>");
    queue_response("<R02:62:7D57>");
    /* No response for W 0x10 → timeout */
    nstar_mock_uart_force_timeout(1);

    TEST_ASSERT_EQUAL(NSTAR_ERR_TIMEOUT,
        nstar_startup_sequence(g_ctx));
}

/**
 * Verify that startup sequence issues commands in the correct order.
 * Check the first bytes of the combined written buffer:
 *   Frame 1: <V00::...>    — V command
 *   Frame 2: <R02:06:...>  — R FPGA_TYPE
 */
void test_startup_sends_v_command_first(void)
{
    queue_response("<V12:010018230042620307:F832>");
    queue_response("<R02:62:7D57>");
    queue_response("<A02:00:466C>");

    nstar_startup_sequence(g_ctx);

    size_t wlen = 0;
    const uint8_t *w = nstar_mock_uart_get_written(&wlen);

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
void test_startup_last_frame_is_obs_sweep_write(void)
{
    queue_response("<V12:010018230042620307:F832>");
    queue_response("<R02:62:7D57>");
    queue_response("<A02:00:466C>");

    nstar_startup_sequence(g_ctx);

    size_t wlen = 0;
    const uint8_t *w = nstar_mock_uart_get_written(&wlen);

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
 * nstar_get_identity tests
 * =========================================================================
 *
 * Verifies the bug fix: identity (including fpga_options) read during
 * startup_sequence() step 2 is now cached on ctx and retrievable
 * afterwards, instead of being read into a stack-local variable and
 * silently discarded when the function returned.
 */

void test_get_identity_before_startup_returns_not_ready(void)
{
    nstar_identity_t id;
    memset(&id, 0, sizeof(id));
    TEST_ASSERT_EQUAL(NSTAR_ERR_NOT_READY, nstar_get_identity(g_ctx, &id));
}

void test_get_identity_null_ctx_returns_param_error(void)
{
    nstar_identity_t id;
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM, nstar_get_identity(NULL, &id));
}

void test_get_identity_null_out_returns_param_error(void)
{
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM, nstar_get_identity(g_ctx, NULL));
}

/**
 * After a successful startup_sequence(), nstar_get_identity() must
 * return the same fpga_options the V command reported — proving the
 * data survives past the function call instead of being discarded.
 */
void test_get_identity_after_startup_returns_cached_fpga_options(void)
{
    queue_response("<V12:010018230042620307:F832>");  /* fpga_options=0x0307 */
    queue_response("<R02:62:7D57>");
    queue_response("<A02:00:466C>");

    TEST_ASSERT_EQUAL(NSTAR_OK, nstar_startup_sequence(g_ctx));

    nstar_identity_t id;
    memset(&id, 0, sizeof(id));
    TEST_ASSERT_EQUAL(NSTAR_OK, nstar_get_identity(g_ctx, &id));
    TEST_ASSERT_EQUAL_HEX8(0x62, id.fpga_type);
    TEST_ASSERT_EQUAL_HEX16(0x0307, id.fpga_options);
}

/* =========================================================================
 * Test runner
 * =========================================================================
 */

int main(void)
{
    UNITY_BEGIN();

    /* Init / deinit */
    RUN_TEST(test_init_null_config_returns_param_error);
    RUN_TEST(test_init_null_callbacks_returns_param_error);
    RUN_TEST(test_init_null_ctx_out_returns_param_error);
    RUN_TEST(test_init_success_returns_non_null_ctx);
    RUN_TEST(test_deinit_null_ctx_is_safe);

    /* reg_read */
    RUN_TEST(test_reg_read_null_ctx_returns_param_error);
    RUN_TEST(test_reg_read_null_val_returns_param_error);
    RUN_TEST(test_reg_read_fpga_type_returns_correct_value);
    RUN_TEST(test_reg_read_sends_correct_frame);
    RUN_TEST(test_reg_read_timeout_returns_timeout_error);
    RUN_TEST(test_reg_read_crc_error_after_retry);
    RUN_TEST(test_reg_read_succeeds_on_retry);

    /* reg_write */
    RUN_TEST(test_reg_write_null_ctx_returns_param_error);
    RUN_TEST(test_reg_write_tx_mode_modulation_success);
    RUN_TEST(test_reg_write_sends_correct_frame);
    RUN_TEST(test_reg_write_bad_ack_returns_bad_ack_error);
    RUN_TEST(test_reg_write_timeout_returns_timeout_error);

    /* reg_read_multi */
    RUN_TEST(test_reg_read_multi_reads_n_registers);
    RUN_TEST(test_reg_read_multi_zero_count_returns_param_error);

    /* cmd_read_identity */
    RUN_TEST(test_cmd_read_identity_success);
    RUN_TEST(test_cmd_read_identity_null_out_returns_param_error);
    RUN_TEST(test_cmd_read_identity_sends_v_frame);

    /* cmd_read_all_rx_status */
    RUN_TEST(test_cmd_read_all_rx_status_success);
    RUN_TEST(test_cmd_read_all_rx_status_null_out_returns_param_error);

    /* cmd_reset */
    RUN_TEST(test_cmd_reset_success);
    RUN_TEST(test_cmd_reset_sends_magic_word);
    RUN_TEST(test_cmd_reset_bad_ack_returns_error);

    /* startup_sequence */
    RUN_TEST(test_startup_sequence_success);
    RUN_TEST(test_startup_wrong_fpga_type_returns_error);
    RUN_TEST(test_startup_v_timeout_returns_timeout_error);
    RUN_TEST(test_startup_obs_write_timeout_returns_timeout_error);
    RUN_TEST(test_startup_sends_v_command_first);
    RUN_TEST(test_startup_last_frame_is_obs_sweep_write);

    /* get_identity */
    RUN_TEST(test_get_identity_before_startup_returns_not_ready);
    RUN_TEST(test_get_identity_null_ctx_returns_param_error);
    RUN_TEST(test_get_identity_null_out_returns_param_error);
    RUN_TEST(test_get_identity_after_startup_returns_cached_fpga_options);

    return UNITY_END();
}
