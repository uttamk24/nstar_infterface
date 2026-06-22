/**
 * @file    nstar.h
 * @brief   N-STAR S/S-Band Transceiver — OBC Interface Module
 *          Single source of truth: all types, constants, HAL prototypes,
 *          and public API prototypes.
 *
 * Project:  Lumos / Dhruva Space
 * Unit:     Engineering Model (EM)
 * Refs:     EICD v1.4, User Manual v1.3, IRD v1.5, Config Sheet Rev H
 *
 * Build flags (pass via -D to compiler or CMake):
 *   NSTAR_CRC_DISABLED  — disable CRC on all UART frames.
 *                         Default (flag absent) = CRC enabled.
 */

#ifndef NSTAR_H
#define NSTAR_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * COMPILE-TIME OPTIONS
 * =========================================================================
 */
#ifndef NSTAR_CRC_DISABLED
#  define NSTAR_CRC_ENABLED
#endif

/* =========================================================================
 * CONSTANTS
 * =========================================================================
 */

/** Maximum frame size handled by the interface. Change here only. */
#define NSTAR_FRAME_SIZE_BYTES          2048U

/** UART baud rate (IRD §3.3.1). */
#define NSTAR_UART_BAUD                 38400U

/** Mandatory wait after power-on before any UART command (User Manual §3.1). */
#define NSTAR_POWERUP_WAIT_MS           3000U

/** Maximum time N-STAR must respond to any command (IRD §3.3.2.4). */
#define NSTAR_CMD_TIMEOUT_MS            500U

/** Maximum time to receive a complete command frame (IRD §3.3.2.4). */
#define NSTAR_CMD_RECV_TIMEOUT_MS       40U

/** Retries on timeout or CRC error before returning failure. */
#define NSTAR_CMD_MAX_RETRIES           1U

/** TX clock stable duration before Modulation command (User Manual §3.1). */
#define NSTAR_TX_CLOCK_PRESTABLE_MS     100U

/** Max TX clock gap before N-STAR auto-enters standby (EICD §5.11.1). */
#define NSTAR_TX_CLOCK_GAP_MAX_MS       1U

/** Soft PA temperature warning — OBC stops TX proactively. */
#define NSTAR_PA_TEMP_WARN_CELSIUS      85.0f

/** Hard PA temperature limit — N-STAR auto-stops (User Manual §2.2). */
#define NSTAR_PA_TEMP_LIMIT_CELSIUS     90.0f

/** Health monitor poll interval (ms). */
#define NSTAR_HEALTH_POLL_INTERVAL_MS   30000U

/* --- Frame delimiters (IRD §3.3.2.1) --- */
#define NSTAR_FRAME_START               '<'
#define NSTAR_FRAME_END                 '>'
#define NSTAR_FRAME_SEP                 ':'

/** Minimum on-wire frame length with CRC enabled:  '<' CMD SIZE ':' ':' CRC '>' = 11 bytes */
#define NSTAR_FRAME_MIN_LEN_CRC         11U

/** Minimum on-wire frame length with CRC disabled:  '<' CMD SIZE ':' '>' = 6 bytes */
#define NSTAR_FRAME_MIN_LEN_NOCRC        6U

/**
 * Maximum on-wire frame buffer.
 * < (1) CMD(1) SIZE(2) :(1) DATA(2*255=510) :(1) CRC(4) >(1) = 521 bytes.
 * Rounded up.
 */
#define NSTAR_FRAME_BUF_MAX             560U

/* --- Register addresses (IRD Annex A/B) --- */
#define NSTAR_REG_FPGA_VERSION          0x00U
#define NSTAR_REG_FPGA_BUILD            0x01U
#define NSTAR_REG_HW_ID_YEAR            0x02U
#define NSTAR_REG_HW_ID_WEEK            0x03U
#define NSTAR_REG_HW_ID_ORDER_H         0x04U
#define NSTAR_REG_HW_ID_ORDER_L         0x05U
#define NSTAR_REG_FPGA_TYPE             0x06U
#define NSTAR_REG_LCL_STATUS            0x07U
#define NSTAR_REG_FPGA_OPTION_H         0x08U
#define NSTAR_REG_FPGA_OPTION_L         0x09U
#define NSTAR_REG_RX_NB_SWEEP           0x10U  /* write: sweep config    */
#define NSTAR_REG_RX_STATUS             0x10U  /* read:  RX state        */
#define NSTAR_REG_RX_SENSITIVITY_MSB    0x11U
#define NSTAR_REG_RX_SENSITIVITY_MID    0x12U
#define NSTAR_REG_RX_SENSITIVITY_LSB    0x13U
#define NSTAR_REG_RX_FREQ_SHIFT_MSB     0x14U
#define NSTAR_REG_RX_FREQ_SHIFT_MID     0x15U
#define NSTAR_REG_RX_FREQ_SHIFT_LSB     0x16U
#define NSTAR_REG_RX_IQ_POWER_MSB       0x17U
#define NSTAR_REG_RX_IQ_POWER_LSB       0x18U
#define NSTAR_REG_RX_AGC_MSB            0x19U
#define NSTAR_REG_RX_AGC_LSB            0x1AU
#define NSTAR_REG_RX_DEMOD_EB_MSB       0x1CU
#define NSTAR_REG_RX_DEMOD_EB_MID       0x1DU
#define NSTAR_REG_RX_DEMOD_EB_LSB       0x1EU
#define NSTAR_REG_RX_DEMOD_N0_MSB       0x1FU
#define NSTAR_REG_RX_DEMOD_N0_MID       0x20U
#define NSTAR_REG_RX_DEMOD_N0_LSB       0x21U
#define NSTAR_REG_RX_DATA_RATE          0x22U
#define NSTAR_REG_TX_MODE               0x40U
#define NSTAR_REG_TX_CONV_DIFF          0x41U
#define NSTAR_REG_TX_CONF_FILTER_MSB    0x42U
#define NSTAR_REG_TX_CONF_FILTER_LSB    0x43U
#define NSTAR_REG_TX_WAVEFORM           0x44U
#define NSTAR_REG_TX_PCM_INDEX          0x45U
#define NSTAR_REG_TX_AGC_MSB            0x46U
#define NSTAR_REG_TX_AGC_LSB            0x47U
#define NSTAR_REG_ADC_BB_TEMP_MSB       0xC0U
#define NSTAR_REG_ADC_BB_TEMP_LSB       0xC1U
#define NSTAR_REG_ADC_PA_TEMP_MSB       0xC8U
#define NSTAR_REG_ADC_PA_TEMP_LSB       0xC9U

/* --- Register values --- */
#define NSTAR_FPGA_TYPE_EXPECTED        0x62U  /* N-STAR PCM/PM, RX+TX    */
#define NSTAR_TX_MODE_STANDBY           0x00U
#define NSTAR_TX_MODE_MODULATION        0x01U
#define NSTAR_TX_MODE_CW                0x02U
#define NSTAR_RX_SWEEP_2PASS            0x02U  /* 2-pass OBS sweep        */
#define NSTAR_RESET_MAGIC               0x5A5AU

/* --- RX_STATUS bit masks --- */
#define NSTAR_RX_STATUS_CARRIER_DETECT  (1u << 0)
#define NSTAR_RX_STATUS_CARRIER_LOCK    (1u << 1)
#define NSTAR_RX_STATUS_BIT_LOCK        (1u << 2)
#define NSTAR_RX_STATUS_DATA_VALID      (1u << 3)

/* --- TX_STATUS bit masks --- */
#define NSTAR_TX_STATUS_MODE_MASK       0x03U
#define NSTAR_TX_STATUS_CLOCK_DETECTED  (1u << 4)

/** Temperature formula: T(degC) = 0.06105 * raw - 50  (IRD Annex C). */
#define NSTAR_TEMP_FROM_ADC(raw)        ((float)(raw) * 0.06105f - 50.0f)

/* =========================================================================
 * RETURN CODES
 * =========================================================================
 */
typedef enum {
    NSTAR_OK                =  0,
    NSTAR_ERR_PARAM         = -1,
    NSTAR_ERR_TIMEOUT       = -2,
    NSTAR_ERR_CRC           = -3,
    NSTAR_ERR_BAD_FRAME     = -4,
    NSTAR_ERR_BAD_ACK       = -5,
    NSTAR_ERR_NOT_INIT      = -6,
    NSTAR_ERR_NO_CLOCK      = -7,
    NSTAR_ERR_FPGA_TYPE     = -8,
    NSTAR_ERR_HAL           = -9,
    NSTAR_ERR_THERMAL       = -10,
    NSTAR_ERR_BUSY          = -11,
    NSTAR_ERR_NOT_READY     = -12,  /**< Module not in READY state.            */
} nstar_result_t;

/* =========================================================================
 * ENUMERATIONS
 * =========================================================================
 */

typedef enum {
    NSTAR_TX_STANDBY    = 0,
    NSTAR_TX_MODULATION = 1,
    NSTAR_TX_CW         = 2,
} nstar_tx_mode_t;

typedef enum {
    NSTAR_RX_IDLE      = 0,
    NSTAR_RX_ACQUIRING = 1,
    NSTAR_RX_LOCKED    = 2,
    NSTAR_RX_LOCK_LOST = 3,
} nstar_rx_state_t;

/**
 * Module-level FSM state — tracks the overall lifecycle of the interface,
 * independent of the RX sub-state-machine (nstar_rx_state_t) and TX
 * activity flag.
 *
 * Legal transitions:
 *   UNINIT        -> INITIALISING   (nstar_init() called)
 *   INITIALISING  -> STARTING       (threads spawned, startup about to run)
 *   STARTING      -> READY          (startup_sequence succeeds)
 *   STARTING      -> FAULT          (startup_sequence fails)
 *   READY         -> FAULT          (FAULT_N asserted)
 *   FAULT         -> STARTING       (recovery: re-running startup_sequence)
 *   READY|FAULT   -> SHUTTING_DOWN  (nstar_deinit() called)
 *
 * TX, RX configuration, and register read/write entry points require
 * NSTAR_MODULE_READY. Calls made during FAULT or STARTING are rejected
 * with NSTAR_ERR_NOT_READY so the application does not race recovery.
 */
typedef enum {
    NSTAR_MODULE_UNINIT        = 0,
    NSTAR_MODULE_INITIALISING  = 1,
    NSTAR_MODULE_STARTING      = 2,
    NSTAR_MODULE_READY         = 3,
    NSTAR_MODULE_FAULT         = 4,
    NSTAR_MODULE_SHUTTING_DOWN = 5,
} nstar_module_state_t;

typedef enum {
    NSTAR_FAULT_SEL         = 0,
    NSTAR_FAULT_OVERCURRENT = 1,
    NSTAR_FAULT_TEMPERATURE = 2,
} nstar_fault_source_t;

typedef enum {
    NSTAR_GPIO_EDGE_RISING  = 0,
    NSTAR_GPIO_EDGE_FALLING = 1,
} nstar_gpio_edge_t;

/** RX data rate codes — PCM/PM demodulation (IRD Table 10). */
typedef enum {
    NSTAR_RX_RATE_256K = 0x01,
    NSTAR_RX_RATE_128K = 0x03,
    NSTAR_RX_RATE_64K  = 0x07,
    NSTAR_RX_RATE_32K  = 0x0F,
    NSTAR_RX_RATE_16K  = 0x1F,  /* Lumos EM default */
    NSTAR_RX_RATE_8K   = 0x3F,
} nstar_rx_rate_code_t;

/** TX data rate codes (OPT-C31-TDF active on Lumos EM). */
typedef enum {
    NSTAR_TX_RATE_256K = 0x01,
    NSTAR_TX_RATE_128K = 0x03,
    NSTAR_TX_RATE_64K  = 0x07,
    NSTAR_TX_RATE_32K  = 0x0F,  /* Lumos EM default */
    NSTAR_TX_RATE_16K  = 0x1F,
    NSTAR_TX_RATE_8K   = 0x3F,
} nstar_tx_rate_code_t;

/* =========================================================================
 * STRUCTS
 * =========================================================================
 */

typedef struct {
    int uart_fd;
    int gpio_lock_detect;
    int gpio_data_valid;
    int gpio_fault_n;
    int gpio_reset_n;
    int data_fd;           /* OPEN POINT: physical interface TBD */
} nstar_config_t;

typedef struct {
    bool    carrier_detect;
    bool    carrier_lock;
    bool    bit_lock;
    bool    data_valid;    /* Only consume RX data when this is true */
    uint8_t sweep_state;
    uint8_t raw;
} nstar_rx_status_t;

typedef struct {
    nstar_tx_mode_t current_mode;
    bool            clock_detected;
    uint8_t         config_set;
    uint8_t         raw;
} nstar_tx_status_t;

typedef struct {
    float    pa_temp_celsius;
    float    bb_temp_celsius;
    bool     fault_active;
    uint16_t pa_adc_raw;
    uint16_t bb_adc_raw;
} nstar_health_t;

typedef struct {
    uint8_t  fpga_version;
    uint8_t  fpga_build;
    uint8_t  hw_year;
    uint8_t  hw_week;
    uint16_t hw_order;
    uint8_t  fpga_type;
    uint16_t fpga_options;
} nstar_identity_t;

typedef struct {
    float   rssi_dbm;
    float   eb_no_db;
    int32_t freq_shift_hz;
} nstar_link_quality_t;

/**
 * Application callbacks registered at nstar_init().
 * Called from internal threads — do NOT call nstar_* functions inside them.
 */
typedef struct {
    void (*on_frame_received)(const uint8_t *buf, size_t len);
    void (*on_tx_complete)(size_t bytes_sent);
    void (*on_fault)(nstar_fault_source_t source);
    void (*on_lock_acquired)(void);
    void (*on_lock_lost)(void);
} nstar_callbacks_t;

/** Opaque context handle returned by nstar_init(). */
typedef struct nstar_ctx nstar_ctx_t;

/* =========================================================================
 * HAL PROTOTYPES  (implemented in hal/ or src/nstar_hal_mock.c)
 * =========================================================================
 */

ssize_t        nstar_hal_uart_write(int fd, const uint8_t *buf, size_t len);
ssize_t        nstar_hal_uart_read(int fd, uint8_t *buf, size_t len,
                                    uint32_t timeout_ms);
nstar_result_t nstar_hal_gpio_wait_edge(int fd, nstar_gpio_edge_t edge,
                                         uint32_t timeout_ms);
int            nstar_hal_gpio_read(int fd);
nstar_result_t nstar_hal_gpio_write(int fd, int value);
ssize_t        nstar_hal_data_write(int fd, const uint8_t *buf, size_t len);
ssize_t        nstar_hal_data_read(int fd, uint8_t *buf, size_t len);

/**
 * Assert the TX clock on CLK_TX pins.
 * Called before nstar_hal_data_write() and before issuing TX_MODE=Modulation.
 * OPEN POINT: implementation depends on data interface (SPI / other).
 * @return NSTAR_OK or NSTAR_ERR_HAL.
 */
nstar_result_t nstar_hal_data_clock_start(int fd);

/**
 * De-assert the TX clock on CLK_TX pins.
 * Called after TX_MODE=Standby.
 * OPEN POINT: implementation depends on data interface.
 * @return NSTAR_OK or NSTAR_ERR_HAL.
 */
nstar_result_t nstar_hal_data_clock_stop(int fd);

void           nstar_hal_sleep_ms(uint32_t ms);
uint64_t       nstar_hal_timestamp_ms(void);

/* =========================================================================
 * FRAME CODEC  (src/nstar_frame.c)
 * =========================================================================
 */

/**
 * Compute CRC16-XMODEM.
 * CRC input for a UART frame includes '<' (IRD §3.3.2.1).
 * Verification: nstar_crc16_xmodem("<V00:", 5) == 0x68D3
 */
uint16_t nstar_crc16_xmodem(const uint8_t *data, size_t len);

/**
 * Encode a UART command frame.
 *
 * CRC input = '<' + CMD_ID + DATA_SIZE + ':' + DATA
 * Sep2 ':', CRC field, and '>' are excluded from CRC input.
 *
 * @param cmd_id    Uppercase command letter ('R','W','V','E','C').
 * @param data_in   Raw payload bytes. NULL when data_len == 0.
 * @param data_len  Number of raw bytes in data_in.
 * @param buf_out   Output buffer, must be >= NSTAR_FRAME_BUF_MAX bytes.
 * @param len_out   Written with the number of bytes placed in buf_out.
 * @return          NSTAR_OK or NSTAR_ERR_PARAM.
 */
nstar_result_t nstar_frame_encode(char cmd_id,
                                   const uint8_t *data_in, size_t data_len,
                                   uint8_t *buf_out, size_t *len_out);

/**
 * Decode a received UART response frame.
 *
 * @param buf           Raw bytes including '<' and '>'.
 * @param len           Number of bytes in buf.
 * @param cmd_id_out    Set to the CMD_ID character.
 * @param data_out      Decoded payload (must be >= NSTAR_FRAME_BUF_MAX/2 bytes).
 * @param data_len_out  Set to number of decoded bytes.
 * @return  NSTAR_OK, NSTAR_ERR_BAD_FRAME, or NSTAR_ERR_CRC.
 */
nstar_result_t nstar_frame_decode(const uint8_t *buf, size_t len,
                                   char *cmd_id_out,
                                   uint8_t *data_out, size_t *data_len_out);

/* =========================================================================
 * PUBLIC API — LIFECYCLE  (src/nstar_core.c)
 * =========================================================================
 */

nstar_result_t nstar_init(const nstar_config_t *config,
                           const nstar_callbacks_t *callbacks,
                           nstar_ctx_t **ctx_out);

void           nstar_deinit(nstar_ctx_t *ctx);

nstar_result_t nstar_startup_sequence(nstar_ctx_t *ctx);

/**
 * Return the current module-level FSM state.
 * Thread-safe — may be called from any thread including callbacks.
 */
nstar_module_state_t nstar_get_module_state(nstar_ctx_t *ctx);

/* =========================================================================
 * PUBLIC API — REGISTER ACCESS  (src/nstar_core.c)
 * =========================================================================
 */

nstar_result_t nstar_reg_read(nstar_ctx_t *ctx, uint8_t addr,
                               uint8_t *val_out);
nstar_result_t nstar_reg_write(nstar_ctx_t *ctx, uint8_t addr, uint8_t val);
nstar_result_t nstar_reg_read_multi(nstar_ctx_t *ctx, uint8_t start_addr,
                                     uint8_t n, uint8_t *buf_out);

/* =========================================================================
 * PUBLIC API — NAMED COMMANDS  (src/nstar_core.c)
 * =========================================================================
 */

nstar_result_t nstar_cmd_read_identity(nstar_ctx_t *ctx,
                                        nstar_identity_t *out);
nstar_result_t nstar_cmd_read_all_rx_status(nstar_ctx_t *ctx,
                                             uint8_t *raw_out,
                                             size_t *len_out);
nstar_result_t nstar_cmd_reset(nstar_ctx_t *ctx);

/* =========================================================================
 * PUBLIC API — TX  (src/nstar_core.c)
 * =========================================================================
 */

nstar_result_t nstar_tx_start(nstar_ctx_t *ctx,
                               nstar_tx_rate_code_t rate_code);
nstar_result_t nstar_tx_write(nstar_ctx_t *ctx,
                               const uint8_t *buf, size_t len);
nstar_result_t nstar_tx_stop(nstar_ctx_t *ctx);
nstar_result_t nstar_tx_get_status(nstar_ctx_t *ctx,
                                    nstar_tx_status_t *out);

/* =========================================================================
 * PUBLIC API — RX  (src/nstar_core.c)
 * =========================================================================
 */

nstar_result_t nstar_rx_configure(nstar_ctx_t *ctx,
                                   nstar_rx_rate_code_t rate_code);
nstar_result_t nstar_rx_get_status(nstar_ctx_t *ctx,
                                    nstar_rx_status_t *out);
nstar_result_t nstar_rx_get_link_quality(nstar_ctx_t *ctx,
                                          nstar_link_quality_t *out);

/**
 * Return the current RX sub-state-machine state, as tracked by the
 * RX thread (IDLE / ACQUIRING / LOCKED / LOCK_LOST).
 * Thread-safe — may be called from any thread.
 */
nstar_rx_state_t nstar_rx_get_state(nstar_ctx_t *ctx);

/**
 * Retrieve the FPGA identity cached during the most recent successful
 * nstar_startup_sequence() call. Returns NSTAR_ERR_NOT_READY if startup
 * has never completed successfully.
 */
nstar_result_t nstar_get_identity(nstar_ctx_t *ctx, nstar_identity_t *out);

/* =========================================================================
 * PUBLIC API — HEALTH  (src/nstar_core.c)
 * =========================================================================
 */

nstar_result_t nstar_health_read(nstar_ctx_t *ctx, nstar_health_t *out);

#ifdef __cplusplus
}
#endif

#endif /* NSTAR_H */
