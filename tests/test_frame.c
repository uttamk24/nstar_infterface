/**
 * @file  test_frame.c
 * @brief Stage 1 unit tests — CRC16, frame encode, frame decode.
 *
 * Frame format (IRD §3.3.2.1):
 *   < CMD_ID DATA_SIZE : DATA : CRC >
 *
 * DATA_SIZE = number of ASCII chars in DATA field = raw_bytes * 2
 * CRC input = '<' + CMD + SIZE + ':' + DATA + ':'  (Sep2 included, '>' excluded)
 *
 * Run with CRC enabled:   ./test_frame_crc_on
 * Run with CRC disabled:  ./test_frame_crc_off
 */

#include "unity/unity.h"
#include "nstar.h"
#include <string.h>
#include <stdint.h>

void setUp(void)    {}
void tearDown(void) {}

/* =========================================================================
 * CRC16-XMODEM tests
 * =========================================================================
 */

/** IRD verification string: "123456789" -> 0x31C3 */
void test_crc_ird_verification_vector(void)
{
    const uint8_t input[] = "123456789";
    TEST_ASSERT_EQUAL_HEX16(0x31C3, nstar_crc16_xmodem(input, 9));
}

/**
 * V command CRC (no data).
 * CRC input = b"<V00::" (6 bytes, Sep2 included).
 * Expected: 0x68D3  (IRD §3.3.2.1 test vector).
 */
void test_crc_v_command_no_data(void)
{
    const uint8_t input[] = { '<','V','0','0',':',':' };
    TEST_ASSERT_EQUAL_HEX16(0x68D3, nstar_crc16_xmodem(input, sizeof(input)));
}

/**
 * R command CRC (read register 0x10).
 * CRC input = b"<R02:10:" (8 bytes).
 * DATA_SIZE='02' = 2 ASCII chars for 1 raw byte (0x10).
 * Expected: 0x9EA5.
 */
void test_crc_read_register_0x10(void)
{
    const uint8_t input[] = { '<','R','0','2',':','1','0',':' };
    TEST_ASSERT_EQUAL_HEX16(0x9EA5, nstar_crc16_xmodem(input, sizeof(input)));
}

/** CRC over empty input returns initial value 0x0000. */
void test_crc_empty_input(void)
{
    TEST_ASSERT_EQUAL_HEX16(0x0000, nstar_crc16_xmodem(NULL, 0));
}

/** Any single-byte change in input must change the CRC. */
void test_crc_differs_on_byte_change(void)
{
    const uint8_t a[] = { '<','R','0','2',':','1','0',':' };
    const uint8_t b[] = { '<','R','0','2',':','1','1',':' };
    TEST_ASSERT_NOT_EQUAL(nstar_crc16_xmodem(a, sizeof(a)),
                           nstar_crc16_xmodem(b, sizeof(b)));
}

/* =========================================================================
 * Frame encode tests
 * =========================================================================
 */

/** NULL output buffer returns NSTAR_ERR_PARAM. */
void test_encode_null_buf_returns_param_error(void)
{
    size_t len;
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM,
                      nstar_frame_encode('V', NULL, 0, NULL, &len));
}

/** NULL data with non-zero length returns NSTAR_ERR_PARAM. */
void test_encode_null_data_nonzero_len_returns_param_error(void)
{
    uint8_t buf[NSTAR_FRAME_BUF_MAX];
    size_t len;
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM,
                      nstar_frame_encode('W', NULL, 2, buf, &len));
}

/**
 * V command — no data.
 * CRC on:  <V00::68D3>  (11 bytes)
 * CRC off: <V00::>      (7 bytes — Sep2 still present, no CRC chars)
 */
void test_encode_v_command_no_data(void)
{
    uint8_t buf[NSTAR_FRAME_BUF_MAX];
    size_t  len = 0;

    TEST_ASSERT_EQUAL(NSTAR_OK,
                      nstar_frame_encode('V', NULL, 0, buf, &len));

#ifdef NSTAR_CRC_ENABLED
    const char *expected = "<V00::68D3>";
    TEST_ASSERT_EQUAL_UINT(strlen(expected), len);
    TEST_ASSERT_EQUAL_MEMORY(expected, buf, len);
#else
    const char *expected = "<V00::>";
    TEST_ASSERT_EQUAL_UINT(strlen(expected), len);
    TEST_ASSERT_EQUAL_MEMORY(expected, buf, len);
#endif
}

/**
 * R command — read register 0x10 (1 raw byte, DATA_SIZE="02").
 * CRC on:  <R02:10:9EA5>  (13 bytes)
 * CRC off: <R02:10:>      (9 bytes)
 */
void test_encode_read_register_0x10(void)
{
    uint8_t buf[NSTAR_FRAME_BUF_MAX];
    size_t  len  = 0;
    uint8_t data = 0x10;

    TEST_ASSERT_EQUAL(NSTAR_OK,
                      nstar_frame_encode('R', &data, 1, buf, &len));

#ifdef NSTAR_CRC_ENABLED
    const char *expected = "<R02:10:9EA5>";
    TEST_ASSERT_EQUAL_UINT(strlen(expected), len);
    TEST_ASSERT_EQUAL_MEMORY(expected, buf, len);
#else
    const char *expected = "<R02:10:>";
    TEST_ASSERT_EQUAL_UINT(strlen(expected), len);
    TEST_ASSERT_EQUAL_MEMORY(expected, buf, len);
#endif
}

/**
 * W command — write TX_MODE=Modulation (addr=0x40, val=0x01).
 * DATA = {0x40, 0x01} -> "4001" (4 ASCII chars, DATA_SIZE="04").
 * CRC on:  <W04:4001:4D69>  (15 bytes)
 * CRC off: <W04:4001:>      (11 bytes)
 */
void test_encode_write_tx_mode_modulation(void)
{
    uint8_t buf[NSTAR_FRAME_BUF_MAX];
    size_t  len = 0;
    uint8_t data[2] = { 0x40, 0x01 };

    TEST_ASSERT_EQUAL(NSTAR_OK,
                      nstar_frame_encode('W', data, 2, buf, &len));

    /* Structure checks valid for both CRC modes */
    TEST_ASSERT_EQUAL_CHAR('<',  (char)buf[0]);
    TEST_ASSERT_EQUAL_CHAR('W',  (char)buf[1]);
    TEST_ASSERT_EQUAL_CHAR('0',  (char)buf[2]);
    TEST_ASSERT_EQUAL_CHAR('4',  (char)buf[3]);
    TEST_ASSERT_EQUAL_CHAR(':',  (char)buf[4]);
    TEST_ASSERT_EQUAL_CHAR('4',  (char)buf[5]);
    TEST_ASSERT_EQUAL_CHAR('0',  (char)buf[6]);
    TEST_ASSERT_EQUAL_CHAR('0',  (char)buf[7]);
    TEST_ASSERT_EQUAL_CHAR('1',  (char)buf[8]);
    TEST_ASSERT_EQUAL_CHAR(':',  (char)buf[9]);  /* Sep2 always present */
    TEST_ASSERT_EQUAL_CHAR('>',  (char)buf[len - 1]);

#ifdef NSTAR_CRC_ENABLED
    const char *expected = "<W04:4001:4D69>";
    TEST_ASSERT_EQUAL_UINT(strlen(expected), len);
    TEST_ASSERT_EQUAL_MEMORY(expected, buf, len);
#else
    TEST_ASSERT_EQUAL_UINT(11, len);
#endif
}

/** C command — reset with magic 0x5A5A. DATA field must be "5A5A". */
void test_encode_reset_command(void)
{
    uint8_t buf[NSTAR_FRAME_BUF_MAX];
    size_t  len = 0;
    uint8_t data[2] = { 0x5A, 0x5A };

    TEST_ASSERT_EQUAL(NSTAR_OK,
                      nstar_frame_encode('C', data, 2, buf, &len));
    TEST_ASSERT_EQUAL_CHAR('5', (char)buf[5]);
    TEST_ASSERT_EQUAL_CHAR('A', (char)buf[6]);
    TEST_ASSERT_EQUAL_CHAR('5', (char)buf[7]);
    TEST_ASSERT_EQUAL_CHAR('A', (char)buf[8]);
}

/**
 * E command — no data. DATA_SIZE="00".
 * CRC on:  <E00::825B>
 */
void test_encode_e_command_no_data(void)
{
    uint8_t buf[NSTAR_FRAME_BUF_MAX];
    size_t  len = 0;

    TEST_ASSERT_EQUAL(NSTAR_OK,
                      nstar_frame_encode('E', NULL, 0, buf, &len));

    TEST_ASSERT_EQUAL_CHAR('<', (char)buf[0]);
    TEST_ASSERT_EQUAL_CHAR('E', (char)buf[1]);
    TEST_ASSERT_EQUAL_CHAR('0', (char)buf[2]);
    TEST_ASSERT_EQUAL_CHAR('0', (char)buf[3]);
    TEST_ASSERT_EQUAL_CHAR(':', (char)buf[4]);
    TEST_ASSERT_EQUAL_CHAR(':', (char)buf[5]);  /* Sep2 immediately after Sep1 */
    TEST_ASSERT_EQUAL_CHAR('>', (char)buf[len - 1]);

#ifdef NSTAR_CRC_ENABLED
    const char *expected = "<E00::825B>";
    TEST_ASSERT_EQUAL_UINT(strlen(expected), len);
    TEST_ASSERT_EQUAL_MEMORY(expected, buf, len);
#endif
}

/**
 * DATA_SIZE field must encode ASCII char count correctly.
 * 16 raw bytes -> 32 ASCII chars -> DATA_SIZE = 0x20 -> "20".
 */
void test_encode_data_size_field_is_ascii_char_count(void)
{
    uint8_t buf[NSTAR_FRAME_BUF_MAX];
    uint8_t data[16];
    size_t  len = 0;
    memset(data, 0xAB, sizeof(data));

    TEST_ASSERT_EQUAL(NSTAR_OK,
                      nstar_frame_encode('W', data, 16, buf, &len));

    /* DATA_SIZE = 32 decimal = 0x20 -> "20" */
    TEST_ASSERT_EQUAL_CHAR('2', (char)buf[2]);
    TEST_ASSERT_EQUAL_CHAR('0', (char)buf[3]);
}

/* =========================================================================
 * Frame decode tests
 * =========================================================================
 */

/** NULL arguments return NSTAR_ERR_PARAM. */
void test_decode_null_args_return_param_error(void)
{
    char    cmd; uint8_t data[64]; size_t dlen;
    const uint8_t f[] = "<R02:10:9EA5>";

    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM,
        nstar_frame_decode(NULL, 13, &cmd, data, &dlen));
    TEST_ASSERT_EQUAL(NSTAR_ERR_PARAM,
        nstar_frame_decode(f, 13, NULL, data, &dlen));
}

/** Missing '<' returns NSTAR_ERR_BAD_FRAME. */
void test_decode_missing_start_delimiter(void)
{
    char cmd; uint8_t data[64]; size_t dlen;
    const uint8_t f[] = "R02:10:9EA5>";
    TEST_ASSERT_EQUAL(NSTAR_ERR_BAD_FRAME,
        nstar_frame_decode(f, sizeof(f)-1, &cmd, data, &dlen));
}

/** Missing '>' returns NSTAR_ERR_BAD_FRAME. */
void test_decode_missing_end_delimiter(void)
{
    char cmd; uint8_t data[64]; size_t dlen;
    const uint8_t f[] = "<R02:10:9EA5";
    TEST_ASSERT_EQUAL(NSTAR_ERR_BAD_FRAME,
        nstar_frame_decode(f, sizeof(f)-1, &cmd, data, &dlen));
}

/**
 * Valid R response: <R02:0F:0B6A>
 * DATA_SIZE=02 -> 2 ASCII chars -> 1 raw byte decoded.
 */
void test_decode_read_response_valid(void)
{
    char cmd = 0; uint8_t data[64] = {0}; size_t dlen = 0;
#ifdef NSTAR_CRC_ENABLED
    const uint8_t f[] = "<R02:0F:0B6A>";
#else
    const uint8_t f[] = "<R02:0F:>";
#endif
    TEST_ASSERT_EQUAL(NSTAR_OK,
        nstar_frame_decode(f, sizeof(f)-1, &cmd, data, &dlen));
    TEST_ASSERT_EQUAL_CHAR('R', cmd);
    TEST_ASSERT_EQUAL_UINT(1, dlen);
    TEST_ASSERT_EQUAL_HEX8(0x0F, data[0]);
}

/**
 * Valid ACK response: <A02:00:466C>
 * 1 raw byte (value 0x00 = ACK success).
 */
void test_decode_ack_response_valid(void)
{
    char cmd = 0; uint8_t data[64] = {0}; size_t dlen = 0;
#ifdef NSTAR_CRC_ENABLED
    const uint8_t f[] = "<A02:00:466C>";
#else
    const uint8_t f[] = "<A02:00:>";
#endif
    TEST_ASSERT_EQUAL(NSTAR_OK,
        nstar_frame_decode(f, sizeof(f)-1, &cmd, data, &dlen));
    TEST_ASSERT_EQUAL_CHAR('A', cmd);
    TEST_ASSERT_EQUAL_UINT(1, dlen);
    TEST_ASSERT_EQUAL_HEX8(0x00, data[0]);
}

/**
 * Valid V response with 6 bytes of identity data.
 * DATA_SIZE = 0x0C (12 ASCII chars = 6 raw bytes).
 * DATA = "01000F1000" would be 10 chars; use 6 bytes = 12 chars = "0C".
 * Let's use 3 raw bytes for simplicity: 3*2=6 ASCII, SIZE="06".
 */
void test_decode_v_response_6_bytes(void)
{
    char cmd = 0; uint8_t data[64] = {0}; size_t dlen = 0;
#ifdef NSTAR_CRC_ENABLED
    /* 3 bytes {0x01, 0x62, 0xFF}: DATA="0162FF", SIZE="06"
       CRC input = b"<V06:0162FF:"  */
    uint8_t crc_in[] = "<V06:0162FF:";
    uint16_t crc = nstar_crc16_xmodem(crc_in, sizeof(crc_in)-1);
    /* Build frame dynamically to avoid hardcoding CRC */
    char frame[64];
    snprintf(frame, sizeof(frame), "<V06:0162FF:%04X>", crc);
    TEST_ASSERT_EQUAL(NSTAR_OK,
        nstar_frame_decode((uint8_t*)frame, strlen(frame), &cmd, data, &dlen));
#else
    const uint8_t f[] = "<V06:0162FF:>";
    TEST_ASSERT_EQUAL(NSTAR_OK,
        nstar_frame_decode(f, sizeof(f)-1, &cmd, data, &dlen));
#endif
    TEST_ASSERT_EQUAL_CHAR('V', cmd);
    TEST_ASSERT_EQUAL_UINT(3, dlen);
    TEST_ASSERT_EQUAL_HEX8(0x01, data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x62, data[1]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, data[2]);
}

#ifdef NSTAR_CRC_ENABLED
/** CRC mismatch returns NSTAR_ERR_CRC. */
void test_decode_bad_crc_returns_crc_error(void)
{
    char cmd; uint8_t data[64]; size_t dlen;
    /* Correct: <R02:0F:0B6A>  Corrupt CRC last nibble -> 0B6B */
    const uint8_t f[] = "<R02:0F:0B6B>";
    TEST_ASSERT_EQUAL(NSTAR_ERR_CRC,
        nstar_frame_decode(f, sizeof(f)-1, &cmd, data, &dlen));
}

/** Corrupted DATA with old CRC returns NSTAR_ERR_CRC. */
void test_decode_corrupted_data_detected_by_crc(void)
{
    char cmd; uint8_t data[64]; size_t dlen;
    /* Valid for 0x0F is <R02:0F:0B6A>. Change 0F->1F, keep CRC. */
    const uint8_t f[] = "<R02:1F:0B6A>";
    TEST_ASSERT_EQUAL(NSTAR_ERR_CRC,
        nstar_frame_decode(f, sizeof(f)-1, &cmd, data, &dlen));
}
#endif

/** Round-trip: encode then decode recovers original data exactly. */
void test_roundtrip_encode_decode(void)
{
    uint8_t  orig[4]  = { 0xDE, 0xAD, 0xBE, 0xEF };
    uint8_t  frame[NSTAR_FRAME_BUF_MAX];
    size_t   frame_len = 0;
    char     cmd = 0;
    uint8_t  dec[64] = {0};
    size_t   dlen = 0;

    TEST_ASSERT_EQUAL(NSTAR_OK,
        nstar_frame_encode('W', orig, 4, frame, &frame_len));
    TEST_ASSERT_EQUAL(NSTAR_OK,
        nstar_frame_decode(frame, frame_len, &cmd, dec, &dlen));
    TEST_ASSERT_EQUAL_CHAR('W', cmd);
    TEST_ASSERT_EQUAL_UINT(4, dlen);
    TEST_ASSERT_EQUAL_MEMORY(orig, dec, 4);
}

/** Round-trip with zero data (no-data command style). */
void test_roundtrip_encode_decode_no_data(void)
{
    uint8_t frame[NSTAR_FRAME_BUF_MAX];
    size_t  frame_len = 0;
    char    cmd = 0;
    uint8_t dec[64] = {0};
    size_t  dlen = 0;

    TEST_ASSERT_EQUAL(NSTAR_OK,
        nstar_frame_encode('V', NULL, 0, frame, &frame_len));
    TEST_ASSERT_EQUAL(NSTAR_OK,
        nstar_frame_decode(frame, frame_len, &cmd, dec, &dlen));
    TEST_ASSERT_EQUAL_CHAR('V', cmd);
    TEST_ASSERT_EQUAL_UINT(0, dlen);
}

/** Round-trip with maximum raw data length (127 bytes). */
void test_roundtrip_max_data_length(void)
{
    uint8_t orig[127];
    uint8_t frame[NSTAR_FRAME_BUF_MAX];
    size_t  frame_len = 0;
    char    cmd;
    uint8_t dec[127];
    size_t  dlen = 0;

    for (int i = 0; i < 127; i++) orig[i] = (uint8_t)i;

    TEST_ASSERT_EQUAL(NSTAR_OK,
        nstar_frame_encode('W', orig, 127, frame, &frame_len));
    TEST_ASSERT_EQUAL(NSTAR_OK,
        nstar_frame_decode(frame, frame_len, &cmd, dec, &dlen));
    TEST_ASSERT_EQUAL_UINT(127, dlen);
    TEST_ASSERT_EQUAL_MEMORY(orig, dec, 127);
}

/** Sep2 must always be present in encoded frame (both CRC modes). */
void test_encode_sep2_always_present(void)
{
    uint8_t buf[NSTAR_FRAME_BUF_MAX];
    size_t  len = 0;

    TEST_ASSERT_EQUAL(NSTAR_OK,
        nstar_frame_encode('V', NULL, 0, buf, &len));
    /* For <V00::...> Sep2 is at index 5 */
    TEST_ASSERT_EQUAL_CHAR(':', (char)buf[5]);
}

/* =========================================================================
 * Test runner
 * =========================================================================
 */
int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_crc_ird_verification_vector);
    RUN_TEST(test_crc_v_command_no_data);
    RUN_TEST(test_crc_read_register_0x10);
    RUN_TEST(test_crc_empty_input);
    RUN_TEST(test_crc_differs_on_byte_change);

    RUN_TEST(test_encode_null_buf_returns_param_error);
    RUN_TEST(test_encode_null_data_nonzero_len_returns_param_error);
    RUN_TEST(test_encode_v_command_no_data);
    RUN_TEST(test_encode_read_register_0x10);
    RUN_TEST(test_encode_write_tx_mode_modulation);
    RUN_TEST(test_encode_reset_command);
    RUN_TEST(test_encode_e_command_no_data);
    RUN_TEST(test_encode_data_size_field_is_ascii_char_count);

    RUN_TEST(test_decode_null_args_return_param_error);
    RUN_TEST(test_decode_missing_start_delimiter);
    RUN_TEST(test_decode_missing_end_delimiter);
    RUN_TEST(test_decode_read_response_valid);
    RUN_TEST(test_decode_ack_response_valid);
    RUN_TEST(test_decode_v_response_6_bytes);
#ifdef NSTAR_CRC_ENABLED
    RUN_TEST(test_decode_bad_crc_returns_crc_error);
    RUN_TEST(test_decode_corrupted_data_detected_by_crc);
#endif

    RUN_TEST(test_roundtrip_encode_decode);
    RUN_TEST(test_roundtrip_encode_decode_no_data);
    RUN_TEST(test_roundtrip_max_data_length);
    RUN_TEST(test_encode_sep2_always_present);

    return UNITY_END();
}
