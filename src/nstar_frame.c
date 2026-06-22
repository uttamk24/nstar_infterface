/**
 * @file  nstar_frame.c
 * @brief UART frame encoder, decoder, and CRC16-XMODEM implementation.
 *
 * Zero hardware dependency — no HAL calls, no threading, no I/O.
 *
 * Frame format (CRC enabled, IRD §3.3.2.1):
 *   < CMD_ID DATA_SIZE : DATA : CRC >
 *
 * DATA_SIZE: 2 ASCII hex chars encoding the NUMBER OF ASCII CHARS in DATA.
 *   e.g. 1 raw byte  -> 2 ASCII chars in DATA -> DATA_SIZE = "02"
 *        2 raw bytes -> 4 ASCII chars in DATA -> DATA_SIZE = "04"
 *
 * CRC input: '<' + CMD_ID + DATA_SIZE + ':' + DATA + ':'
 *   i.e. everything from '<' up to AND INCLUDING Sep2.
 *   Only the CRC field itself and '>' are excluded.
 *
 * Verified against IRD §3.3.2.1 test vectors:
 *   crc("<V00::")  = 0x68D3  (V command, no data)
 *   crc("<R02:10:") = 0x9EA5  (R command, addr 0x10)
 */

#include "nstar.h"
#include <stdio.h>
#include <string.h>

/* =========================================================================
 * Internal helpers
 * =========================================================================
 */

static char nibble_to_hex(uint8_t n)
{
    return (n < 10) ? (char)('0' + n) : (char)('A' + n - 10);
}

static int hex_to_nibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static void hex_encode(const uint8_t *src, size_t src_len, char *dst)
{
    for (size_t i = 0; i < src_len; i++) {
        dst[i * 2]     = nibble_to_hex((src[i] >> 4) & 0x0F);
        dst[i * 2 + 1] = nibble_to_hex(src[i] & 0x0F);
    }
}

static bool hex_decode(const char *src, size_t src_len,
                        uint8_t *dst, size_t *dst_len_out)
{
    if (src_len % 2 != 0) return false;
    *dst_len_out = src_len / 2;
    for (size_t i = 0; i < *dst_len_out; i++) {
        int hi = hex_to_nibble(src[i * 2]);
        int lo = hex_to_nibble(src[i * 2 + 1]);
        if (hi < 0 || lo < 0) return false;
        dst[i] = (uint8_t)((hi << 4) | lo);
    }
    return true;
}

/* =========================================================================
 * CRC16-XMODEM
 * =========================================================================
 * Poly: 0x1021, Init: 0x0000, MSB first, no reflection.
 * Verification (IRD §3.3.2.1):
 *   nstar_crc16_xmodem("123456789", 9) == 0x31C3
 *   nstar_crc16_xmodem("<V00::",    6) == 0x68D3
 *   nstar_crc16_xmodem("<R02:10:",  8) == 0x9EA5
 */
uint16_t nstar_crc16_xmodem(const uint8_t *data, size_t len)
{
    uint16_t crc = 0x0000U;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x8000U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

/* =========================================================================
 * Frame encoder
 * =========================================================================
 *
 * On-wire layout (CRC enabled):
 *
 *   Offset  Field       Content
 *   0       '<'         Start delimiter
 *   1       CMD_ID      1 ASCII char  e.g. 'R'
 *   2-3     DATA_SIZE   2 ASCII hex chars = count of ASCII chars in DATA
 *   4       ':'         Sep1
 *   5..N    DATA        data_len*2 ASCII hex chars
 *   N+1     ':'         Sep2  -- INCLUDED in CRC input
 *   N+2..N+5 CRC        4 ASCII hex chars
 *   N+6     '>'         End delimiter
 *
 * CRC is computed over buf[0..N+1] inclusive (includes Sep2).
 *
 * Examples:
 *   V cmd (no data):  <V00::68D3>
 *     DATA_SIZE='00', DATA='', CRC input=b"<V00::"
 *
 *   R cmd (addr=0x10):  <R02:10:9EA5>
 *     DATA_SIZE='02' (2 ASCII chars), DATA='10', CRC input=b"<R02:10:"
 *
 *   W cmd (addr=0x40, val=0x01):  <W04:4001:CCCC>
 *     DATA_SIZE='04' (4 ASCII chars), DATA='4001', CRC input=b"<W04:4001:"
 */
nstar_result_t nstar_frame_encode(char cmd_id,
                                   const uint8_t *data_in, size_t data_len,
                                   uint8_t *buf_out, size_t *len_out)
{
    if (!buf_out || !len_out) return NSTAR_ERR_PARAM;
    if (data_len > 0 && !data_in) return NSTAR_ERR_PARAM;
    /* DATA_SIZE field is 2 hex chars encoding ascii char count (max 0xFF = 255) */
    if (data_len > 127) return NSTAR_ERR_PARAM; /* 127 raw = 254 ASCII chars, fits in 2 hex */

    uint8_t *p = buf_out;

    /* Start delimiter */
    *p++ = (uint8_t)NSTAR_FRAME_START;

    /* CMD_ID */
    *p++ = (uint8_t)cmd_id;

    /* DATA_SIZE = number of ASCII chars in DATA field = data_len * 2 */
    size_t ascii_data_len = data_len * 2;
    *p++ = (uint8_t)nibble_to_hex((uint8_t)((ascii_data_len >> 4) & 0x0F));
    *p++ = (uint8_t)nibble_to_hex((uint8_t)(ascii_data_len & 0x0F));

    /* Sep1 */
    *p++ = (uint8_t)NSTAR_FRAME_SEP;

    /* DATA: each raw byte -> 2 uppercase ASCII hex chars */
    if (data_len > 0) {
        hex_encode(data_in, data_len, (char *)p);
        p += ascii_data_len;
    }

    /* Sep2 — written BEFORE CRC computation because it is part of CRC input */
    *p++ = (uint8_t)NSTAR_FRAME_SEP;

#ifdef NSTAR_CRC_ENABLED
    /* CRC over everything written so far: '<' CMD SIZE ':' DATA ':' */
    size_t crc_input_len = (size_t)(p - buf_out);
    uint16_t crc = nstar_crc16_xmodem(buf_out, crc_input_len);

    /* CRC as 4 uppercase ASCII hex chars */
    *p++ = (uint8_t)nibble_to_hex((crc >> 12) & 0x0F);
    *p++ = (uint8_t)nibble_to_hex((crc >>  8) & 0x0F);
    *p++ = (uint8_t)nibble_to_hex((crc >>  4) & 0x0F);
    *p++ = (uint8_t)nibble_to_hex(crc & 0x0F);
#endif

    /* End delimiter */
    *p++ = (uint8_t)NSTAR_FRAME_END;

    *len_out = (size_t)(p - buf_out);
    return NSTAR_OK;
}

/* =========================================================================
 * Frame decoder
 * =========================================================================
 *
 * Accepts the raw on-wire bytes including '<' and '>'.
 *
 * With CRC enabled frame structure:
 *   < CMD(1) SIZE(2) : DATA(SIZE chars) : CRC(4) >
 *   Total = 1+1+2+1+SIZE+1+4+1 = 11 + SIZE bytes
 *
 * DATA_SIZE encodes the count of ASCII chars in DATA.
 * Decode converts those ASCII pairs back to raw bytes.
 *
 * CRC verification:
 *   Input = buf[0 .. sep2_pos] inclusive
 *   sep2_pos = 5 + SIZE  (the Sep2 immediately after DATA)
 */
nstar_result_t nstar_frame_decode(const uint8_t *buf, size_t len,
                                   char *cmd_id_out,
                                   uint8_t *data_out, size_t *data_len_out)
{
    if (!buf || !cmd_id_out || !data_out || !data_len_out) {
        return NSTAR_ERR_PARAM;
    }

#ifdef NSTAR_CRC_ENABLED
    if (len < (size_t)NSTAR_FRAME_MIN_LEN_CRC) return NSTAR_ERR_BAD_FRAME;
#else
    if (len < (size_t)NSTAR_FRAME_MIN_LEN_NOCRC) return NSTAR_ERR_BAD_FRAME;
#endif

    /* Delimiters */
    if (buf[0] != (uint8_t)NSTAR_FRAME_START) return NSTAR_ERR_BAD_FRAME;
    if (buf[len - 1] != (uint8_t)NSTAR_FRAME_END) return NSTAR_ERR_BAD_FRAME;

    /* CMD_ID at index 1 */
    *cmd_id_out = (char)buf[1];

    /* DATA_SIZE at index 2-3: ASCII hex count of chars in DATA */
    int size_hi = hex_to_nibble((char)buf[2]);
    int size_lo = hex_to_nibble((char)buf[3]);
    if (size_hi < 0 || size_lo < 0) return NSTAR_ERR_BAD_FRAME;
    size_t data_ascii_len = (size_t)((size_hi << 4) | size_lo);

    /* Sep1 at index 4 */
    if (buf[4] != (uint8_t)NSTAR_FRAME_SEP) return NSTAR_ERR_BAD_FRAME;

    /* DATA at index 5, length = data_ascii_len ASCII chars */
    size_t data_start = 5;
    size_t sep2_pos   = data_start + data_ascii_len;  /* index of Sep2 */

    /* Sep2 must be present */
    if (sep2_pos >= len) return NSTAR_ERR_BAD_FRAME;
    if (buf[sep2_pos] != (uint8_t)NSTAR_FRAME_SEP) return NSTAR_ERR_BAD_FRAME;

#ifdef NSTAR_CRC_ENABLED
    /*
     * With CRC: after Sep2 we need 4 CRC chars then '>'.
     * sep2_pos + 1 + 4 + 1 = sep2_pos + 6 = len
     */
    if (sep2_pos + 6 != len) return NSTAR_ERR_BAD_FRAME;

    /* Extract declared CRC from frame (4 chars at sep2_pos+1) */
    int c0 = hex_to_nibble((char)buf[sep2_pos + 1]);
    int c1 = hex_to_nibble((char)buf[sep2_pos + 2]);
    int c2 = hex_to_nibble((char)buf[sep2_pos + 3]);
    int c3 = hex_to_nibble((char)buf[sep2_pos + 4]);
    if (c0 < 0 || c1 < 0 || c2 < 0 || c3 < 0) return NSTAR_ERR_BAD_FRAME;
    uint16_t received_crc = (uint16_t)(((uint16_t)c0 << 12) |
                                        ((uint16_t)c1 <<  8) |
                                        ((uint16_t)c2 <<  4) |
                                         (uint16_t)c3);

    /*
     * CRC input = buf[0 .. sep2_pos] inclusive (includes '<' and Sep2).
     */
    uint16_t computed_crc = nstar_crc16_xmodem(buf, sep2_pos + 1);
    if (computed_crc != received_crc) return NSTAR_ERR_CRC;

#else /* CRC disabled */
    /*
     * Without CRC: after Sep2 we expect only '>'.
     * sep2_pos + 1 + 1 = sep2_pos + 2 = len
     */
    if (sep2_pos + 2 != len) return NSTAR_ERR_BAD_FRAME;
#endif

    /* Hex-decode the DATA field */
    if (data_ascii_len == 0) {
        *data_len_out = 0;
    } else {
        if (!hex_decode((const char *)(buf + data_start),
                        data_ascii_len, data_out, data_len_out)) {
            return NSTAR_ERR_BAD_FRAME;
        }
    }

    return NSTAR_OK;
}
