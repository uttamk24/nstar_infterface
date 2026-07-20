/**
 * @file    ttc_nstar.h
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

#ifndef INC_TTC_NSTAR_H
#define INC_TTC_NSTAR_H

#ifdef TTC_NSTAR_DEBUG_L1
#define TTC_NSTAR_Debug_L1(message)         logWithTimestamp(message)
#define TTC_NSTAR_Debug_L1_D(message, data) logWithTimestamp_Data(message, data)
#else
#define TTC_NSTAR_Debug_L1(message)
#define TTC_NSTAR_Debug_L1_D(message, data)
#endif

#ifdef TTC_NSTAR_DEBUG_L2
#define TTC_NSTAR_Debug_L2(message)         logWithTimestamp(message)
#define TTC_NSTAR_Debug_L2_D(message, data) logWithTimestamp_Data(message, data)
#define TTC_NSTAR_Debug_L2_G(message, size) PRINT_ARRAY(message, size)
#define TTC_NSTAR_Debug_L2_B(message, size) PRINT_ERROR_ARRAY(message, size)
#else
#define TTC_NSTAR_Debug_L2(message)
#define TTC_NSTAR_Debug_L2_D(message, data)
#define TTC_NSTAR_Debug_L2_G(message, size)
#define TTC_NSTAR_Debug_L2_B(message, size)
#endif


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
} NSTAR_Result_t;

/* =========================================================================
 * ENUMERATIONS
 * =========================================================================
 */

typedef enum {
    NSTAR_TX_STANDBY    = 0,
    NSTAR_TX_MODULATION = 1,
    NSTAR_TX_CW         = 2,
} NSTAR_TXMode_t;

typedef enum {
    NSTAR_RX_IDLE      = 0,
    NSTAR_RX_ACQUIRING = 1,
    NSTAR_RX_LOCKED    = 2,
    NSTAR_RX_LOCK_LOST = 3,
} NSTAR_RXState_t;

/**
 * Module-level FSM state — tracks the overall lifecycle of the interface,
 * independent of the RX sub-state-machine (NSTAR_RXState_t) and TX
 * activity flag.
 *
 * Legal transitions:
 *   UNINIT        -> INITIALISING   (NSTAR_Init() called)
 *   INITIALISING  -> STARTING       (threads spawned, startup about to run)
 *   STARTING      -> READY          (startup_sequence succeeds)
 *   STARTING      -> FAULT          (startup_sequence fails)
 *   READY         -> FAULT          (FAULT_N asserted)
 *   FAULT         -> STARTING       (recovery: re-running startup_sequence)
 *   READY|FAULT   -> SHUTTING_DOWN  (NSTAR_Deinit() called)
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
} NSTAR_ModuleState_t;

typedef enum {
    NSTAR_FAULT_SEL         = 0,
    NSTAR_FAULT_OVERCURRENT = 1,
    NSTAR_FAULT_TEMPERATURE = 2,
} NSTAR_FaultSource_t;

typedef enum {
    NSTAR_GPIO_EDGE_RISING  = 0,
    NSTAR_GPIO_EDGE_FALLING = 1,
} NSTAR_GPIOEdge_t;

/** RX data rate codes — PCM/PM demodulation (IRD Table 10). */
typedef enum {
    NSTAR_RX_RATE_256K = 0x01,
    NSTAR_RX_RATE_128K = 0x03,
    NSTAR_RX_RATE_64K  = 0x07,
    NSTAR_RX_RATE_32K  = 0x0F,
    NSTAR_RX_RATE_16K  = 0x1F,  /* Lumos EM default */
    NSTAR_RX_RATE_8K   = 0x3F,
} NSTAR_RXRateCode_t;

/** TX data rate codes (OPT-C31-TDF active on Lumos EM). */
typedef enum {
    NSTAR_TX_RATE_256K = 0x01,
    NSTAR_TX_RATE_128K = 0x03,
    NSTAR_TX_RATE_64K  = 0x07,
    NSTAR_TX_RATE_32K  = 0x0F,  /* Lumos EM default */
    NSTAR_TX_RATE_16K  = 0x1F,
    NSTAR_TX_RATE_8K   = 0x3F,
} NSTAR_TXRateCode_t;

/* =========================================================================
 * STRUCTS
 * =========================================================================
 */

typedef struct {
    int uartFd;
    int gpioLockDetect;
    int gpioDataValid;
    int gpioFaultN;
    int gpioResetN;
    int dataFd;           /* OPEN POINT: physical interface TBD */
} NSTAR_Config_t;

typedef struct {
    bool    carrierDetect;
    bool    carrierLock;
    bool    bitLock;
    bool    dataValid;    /* Only consume RX data when this is true */
    uint8_t sweepState;
    uint8_t raw;
} NSTAR_RXStatus_t;

typedef struct {
    NSTAR_TXMode_t currentMode;
    bool            clockDetected;
    uint8_t         configSet;
    uint8_t         raw;
} NSTAR_TXStatus_t;

typedef struct {
    float    paTempCelsius;
    float    bbTempCelsius;
    bool     faultActive;
    uint16_t paAdcRaw;
    uint16_t bbAdcRaw;
} NSTAR_Health_t;

typedef struct {
    uint8_t  fpgaVersion;
    uint8_t  fpgaBuild;
    uint8_t  hwYear;
    uint8_t  hwWeek;
    uint16_t hwOrder;
    uint8_t  fpgaType;
    uint16_t fpgaOptions;
} NSTAR_Identity_t;

typedef struct {
    float   rssiDBM;
    float   ebNoDB;
    int32_t freqShiftHz;
} NSTAR_LinkQuality_t;

/**
 * Application callbacks registered at NSTAR_Init().
 * Called from internal threads — do NOT call nstar_* functions inside them.
 */
typedef struct {
    void (*onFrameReceived)(const uint8_t *buf, size_t len);
    void (*onTXComplete)(size_t bytesSent);
    void (*onFault)(NSTAR_FaultSource_t source);
    void (*onLockAcquired)(void);
    void (*onLockLost)(void);
} NSTAR_Callbacks_t;

/** Opaque context handle returned by NSTAR_Init(). */
typedef struct NSTAR_Ctx NSTAR_Ctx_t;

/* =========================================================================
 * HAL PROTOTYPES  (implemented in hal/ or src/nstar_hal_mock.c)
 * =========================================================================
 */

ssize_t        nstarUARTWrite(int fd, const uint8_t *buf, size_t len);
ssize_t        nstarUARTRead(int fd, uint8_t *buf, size_t len,
                                    uint32_t timeoutMs);
NSTAR_Result_t nstarGPIOWaitEdge(int fd, NSTAR_GPIOEdge_t edge,
                                         uint32_t timeoutMs);
int            nstarGPIORead(int fd);
NSTAR_Result_t nstarGPIOWrite(int fd, int value);
ssize_t        nstarDataWrite(int fd, const uint8_t *buf, size_t len);
ssize_t        nstarDataRead(int fd, uint8_t *buf, size_t len);

/**
 * Assert the TX clock on CLK_TX pins.
 * Called before nstarDataWrite() and before issuing TX_MODE=Modulation.
 * OPEN POINT: implementation depends on data interface (SPI / other).
 * @return NSTAR_OK or NSTAR_ERR_HAL.
 */
NSTAR_Result_t nstarDataClockStart(int fd);

/**
 * De-assert the TX clock on CLK_TX pins.
 * Called after TX_MODE=Standby.
 * OPEN POINT: implementation depends on data interface.
 * @return NSTAR_OK or NSTAR_ERR_HAL.
 */
NSTAR_Result_t nstarDataClockStop(int fd);

void           nstarSleepMS(uint32_t ms);
uint64_t       nstarTimestampMS(void);

/* =========================================================================
 * FRAME CODEC  (src/nstar_frame.c)
 * =========================================================================
 */

/**
 * Compute CRC16-XMODEM.
 * CRC input for a UART frame includes '<' (IRD §3.3.2.1).
 * Verification: nstarCRC16Xmodem("<V00:", 5) == 0x68D3
 */
uint16_t nstarCRC16Xmodem(const uint8_t *data, size_t len);

/**
 * Encode a UART command frame.
 *
 * CRC input = '<' + CMD_ID + DATA_SIZE + ':' + DATA
 * Sep2 ':', CRC field, and '>' are excluded from CRC input.
 *
 * @param cmdId    Uppercase command letter ('R','W','V','E','C').
 * @param dataIn   Raw payload bytes. NULL when dataLen == 0.
 * @param dataLen  Number of raw bytes in dataIn.
 * @param bufOut   Output buffer, must be >= NSTAR_FRAME_BUF_MAX bytes.
 * @param lenOut   Written with the number of bytes placed in bufOut.
 * @return          NSTAR_OK or NSTAR_ERR_PARAM.
 */
NSTAR_Result_t nstarFrameEncode(char cmdId,
                                   const uint8_t *dataIn, size_t dataLen,
                                   uint8_t *bufOut, size_t *lenOut);

/**
 * Decode a received UART response frame.
 *
 * @param buf           Raw bytes including '<' and '>'.
 * @param len           Number of bytes in buf.
 * @param cmdIdOut    Set to the CMD_ID character.
 * @param dataOut      Decoded payload (must be >= NSTAR_FRAME_BUF_MAX/2 bytes).
 * @param dataLenOut  Set to number of decoded bytes.
 * @return  NSTAR_OK, NSTAR_ERR_BAD_FRAME, or NSTAR_ERR_CRC.
 */
NSTAR_Result_t nstarFrameDecode(const uint8_t *buf, size_t len,
                                   char *cmdIdOut,
                                   uint8_t *dataOut, size_t *dataLenOut);

/* =========================================================================
 * PUBLIC API — LIFECYCLE  (src/nstar_core.c)
 * =========================================================================
 */

NSTAR_Result_t NSTAR_Init(const NSTAR_Config_t *config,
                           const NSTAR_Callbacks_t *callbacks,
                           NSTAR_Ctx_t **ctxOut);

void           NSTAR_Deinit(NSTAR_Ctx_t *ctx);

NSTAR_Result_t NSTAR_StartupSequence(NSTAR_Ctx_t *ctx);

/**
 * Return the current module-level FSM state.
 * Thread-safe — may be called from any thread including callbacks.
 */
NSTAR_ModuleState_t NSTAR_GetModuleState(NSTAR_Ctx_t *ctx);

/* =========================================================================
 * PUBLIC API — REGISTER ACCESS  (src/nstar_core.c)
 * =========================================================================
 */

NSTAR_Result_t NSTAR_RegRead(NSTAR_Ctx_t *ctx, uint8_t addr,
                               uint8_t *valOut);
NSTAR_Result_t NSTAR_RegWrite(NSTAR_Ctx_t *ctx, uint8_t addr, uint8_t val);
NSTAR_Result_t NSTAR_RegReadMulti(NSTAR_Ctx_t *ctx, uint8_t startAddr,
                                     uint8_t n, uint8_t *bufOut);

/* =========================================================================
 * PUBLIC API — NAMED COMMANDS  (src/nstar_core.c)
 * =========================================================================
 */

NSTAR_Result_t NSTAR_CMDReadIdentity(NSTAR_Ctx_t *ctx,
                                        NSTAR_Identity_t *out);
NSTAR_Result_t NSTAR_CMDReadAllRXStatus(NSTAR_Ctx_t *ctx,
                                             uint8_t *rawOut,
                                             size_t *lenOut);
NSTAR_Result_t NSTAR_CMDReset(NSTAR_Ctx_t *ctx);

/* =========================================================================
 * PUBLIC API — TX  (src/nstar_core.c)
 * =========================================================================
 */

NSTAR_Result_t NSTAR_TXStart(NSTAR_Ctx_t *ctx,
                               NSTAR_TXRateCode_t rateCode);
NSTAR_Result_t NSTAR_TXWrite(NSTAR_Ctx_t *ctx,
                               const uint8_t *buf, size_t len);
NSTAR_Result_t NSTAR_TXStop(NSTAR_Ctx_t *ctx);
NSTAR_Result_t NSTAR_TXGetStatus(NSTAR_Ctx_t *ctx,
                                    NSTAR_TXStatus_t *out);

/* =========================================================================
 * PUBLIC API — RX  (src/nstar_core.c)
 * =========================================================================
 */

NSTAR_Result_t NSTAR_RXConfigure(NSTAR_Ctx_t *ctx,
                                   NSTAR_RXRateCode_t rateCode);
NSTAR_Result_t NSTAR_RXGetStatus(NSTAR_Ctx_t *ctx,
                                    NSTAR_RXStatus_t *out);
NSTAR_Result_t NSTAR_RXGetLinkQuality(NSTAR_Ctx_t *ctx,
                                          NSTAR_LinkQuality_t *out);

/**
 * Return the current RX sub-state-machine state, as tracked by the
 * RX thread (IDLE / ACQUIRING / LOCKED / LOCK_LOST).
 * Thread-safe — may be called from any thread.
 */
NSTAR_RXState_t NSTAR_RXGetState(NSTAR_Ctx_t *ctx);

/**
 * Retrieve the FPGA identity cached during the most recent successful
 * NSTAR_StartupSequence() call. Returns NSTAR_ERR_NOT_READY if startup
 * has never completed successfully.
 */
NSTAR_Result_t NSTAR_GetIdentity(NSTAR_Ctx_t *ctx, NSTAR_Identity_t *out);

/* =========================================================================
 * PUBLIC API — HEALTH  (src/nstar_core.c)
 * =========================================================================
 */

NSTAR_Result_t NSTAR_HealthRead(NSTAR_Ctx_t *ctx, NSTAR_Health_t *out);

#ifdef __cplusplus
}
#endif

#endif /* INC_TTC_NSTAR_H */
