/**
 * @file  nstar_sim.c
 * @brief Standalone N-STAR simulator process for SIL testing.
 *
 * Opens a PTY master/slave pair, writes the slave device path to
 * SIL_PTY_LINK_FILE so nstar_app_sil can open the same line as its UART,
 * then runs a loop: read a frame, decode it with the REAL
 * nstarFrameDecode(), look up the command, mutate/report register
 * state, encode a response with the REAL nstarFrameEncode(), write it
 * back.
 *
 * Using the production frame codec (not a re-implementation) means any
 * bug fixed in nstar_frame.c is automatically reflected on both sides of
 * the simulated link — the simulator can never silently diverge from
 * what the real N-STAR's framing actually requires, because it's running
 * the identical code the application uses.
 *
 * Register model:
 *   A small in-memory register file covers every address nstar_core.c
 *   currently reads or writes (see the REG_* table below). Defaults are
 *   chosen to make a normal startup_sequence() succeed out of the box:
 *   FPGA_TYPE = 0x62 (matches NSTAR_FPGA_TYPE_EXPECTED).
 *
 * Control file (SIL_SIM_CONTROL_FILE):
 *   Polled once per loop iteration (i.e. between frames, never mid-frame).
 *   Each line is a KEY=VALUE command, consumed and the file truncated
 *   after processing so commands are one-shot unless re-written.
 *   Supported commands:
 *     FPGA_TYPE=<hex>      e.g. FPGA_TYPE=FF forces a startup_sequence
 *                          failure (NSTAR_ERR_FPGA_TYPE)
 *     PA_TEMP_RAW=<dec>    sets register 0xC8/0xC9 (PA temp ADC)
 *     BB_TEMP_RAW=<dec>    sets register 0xC0/0xC1 (BB temp ADC)
 *     FORCE_TIMEOUT=1|0    when 1, the simulator stops responding
 *                          entirely (used to test NSTAR_ERR_TIMEOUT paths)
 *     RESET                restores every register to its default value
 *
 * The simulator does NOT drive GPIO lines — those are plain files the
 * SIL test suite writes directly (see sil_common.h), independent of the
 * UART/register simulation here. This mirrors reality: FAULT_N, RESET_N,
 * LOCK_DETECT, and DATA_VALID are separate physical pins from the UART,
 * so a SIL test orchestrates them independently rather than expecting
 * the protocol simulator to infer GPIO state from register writes.
 */

#define _DEFAULT_SOURCE
#include "ttc_nstar.h"
#include "sil_common.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <termios.h>
#include <sys/select.h>
#include <time.h>
#include <errno.h>

#if defined(__APPLE__)
#include <util.h>
#else
#include <pty.h>
#endif

/* =========================================================================
 * Register model
 * =========================================================================
 */

#define REG_COUNT 256
static uint8_t gRegs[REG_COUNT];
static int     gForceTimeout = 0;
static int     gTXClockDetected = 1;   /* default: clock present */
static volatile int gRunning  = 1;

static void setRegisterDefaults(void)
{
    memset(gRegs, 0, sizeof(gRegs));
    gRegs[NSTAR_REG_FPGA_VERSION]   = 0x01;
    gRegs[NSTAR_REG_FPGA_BUILD]     = 0x00;
    gRegs[NSTAR_REG_HW_ID_YEAR]     = 0x18;
    gRegs[NSTAR_REG_HW_ID_WEEK]     = 0x23;
    gRegs[NSTAR_REG_HW_ID_ORDER_H]  = 0x00;
    gRegs[NSTAR_REG_HW_ID_ORDER_L]  = 0x42;
    gRegs[NSTAR_REG_FPGA_TYPE]      = NSTAR_FPGA_TYPE_EXPECTED;  /* 0x62 */
    gRegs[NSTAR_REG_FPGA_OPTION_H]  = 0x03;
    gRegs[NSTAR_REG_FPGA_OPTION_L]  = 0x07;
    /* PA temp default ~36C: raw = (36+50)/0.06105 ~= 1409 = 0x0581 */
    gRegs[NSTAR_REG_ADC_PA_TEMP_MSB] = 0x05;
    gRegs[NSTAR_REG_ADC_PA_TEMP_LSB] = 0x81;
    /* BB temp default ~23C: raw = (23+50)/0.06105 ~= 1196 = 0x04AC */
    gRegs[NSTAR_REG_ADC_BB_TEMP_MSB] = 0x04;
    gRegs[NSTAR_REG_ADC_BB_TEMP_LSB] = 0xAC;
}

static void sighandler(int sig)
{
    (void)sig;
    gRunning = 0;
}

/* =========================================================================
 * Control file polling
 * =========================================================================
 */

static void applyControlCommands(void)
{
    char path[512];
    silPath(path, sizeof(path), SIL_SIM_CONTROL_FILE);

    FILE *f = fopen(path, "r");
    if (!f) return;

    char line[128];
    int any = 0;
    while (fgets(line, sizeof(line), f)) {
        char *nl = strchr(line, '\n');
        if (nl) *nl = '\0';
        if (line[0] == '\0') continue;
        any = 1;

        if (strncmp(line, "FPGA_TYPE=", 10) == 0) {
            gRegs[NSTAR_REG_FPGA_TYPE] = (uint8_t)strtol(line + 10, NULL, 16);
            fprintf(stderr, "[nstar_sim] FPGA_TYPE -> 0x%02X\n",
                    gRegs[NSTAR_REG_FPGA_TYPE]);
        } else if (strncmp(line, "PA_TEMP_RAW=", 12) == 0) {
            long raw = strtol(line + 12, NULL, 10);
            gRegs[NSTAR_REG_ADC_PA_TEMP_MSB] = (uint8_t)((raw >> 8) & 0xFF);
            gRegs[NSTAR_REG_ADC_PA_TEMP_LSB] = (uint8_t)(raw & 0xFF);
            fprintf(stderr, "[nstar_sim] PA_TEMP_RAW -> %ld\n", raw);
        } else if (strncmp(line, "BB_TEMP_RAW=", 12) == 0) {
            long raw = strtol(line + 12, NULL, 10);
            gRegs[NSTAR_REG_ADC_BB_TEMP_MSB] = (uint8_t)((raw >> 8) & 0xFF);
            gRegs[NSTAR_REG_ADC_BB_TEMP_LSB] = (uint8_t)(raw & 0xFF);
            fprintf(stderr, "[nstar_sim] BB_TEMP_RAW -> %ld\n", raw);
        } else if (strncmp(line, "FORCE_TIMEOUT=", 14) == 0) {
            gForceTimeout = (int)strtol(line + 14, NULL, 10);
            fprintf(stderr, "[nstar_sim] FORCE_TIMEOUT -> %d\n", gForceTimeout);
        } else if (strncmp(line, "TX_CLOCK_DETECTED=", 18) == 0) {
            gTXClockDetected = (int)strtol(line + 18, NULL, 10);
            fprintf(stderr, "[nstar_sim] TX_CLOCK_DETECTED -> %d\n",
                    gTXClockDetected);
        } else if (strncmp(line, "RESET", 5) == 0) {
            setRegisterDefaults();
            gForceTimeout = 0;
            gTXClockDetected = 1;
            fprintf(stderr, "[nstar_sim] RESET -> defaults restored\n");
        }
    }
    fclose(f);

    if (any) {
        /* Truncate so each command is consumed once */
        FILE *tf = fopen(path, "w");
        if (tf) fclose(tf);
    }
}

/* =========================================================================
 * Command handling — builds the response DATA payload for each command
 * =========================================================================
 *
 * Returns the response CMD_ID character and fills respData/respLen.
 * For W commands, also performs the register write as a side effect.
 */

static char handleCommand(char cmdId, const uint8_t *data, size_t dataLen,
                            uint8_t *respData, size_t *respLen)
{
    switch (cmdId) {

    case 'V': {
        /* Identity: FPGA_VERSION, FPGA_BUILD, HW_YEAR, HW_WEEK,
         * HW_ORDER(2), FPGA_TYPE, FPGA_OPTION(2) = 9 bytes */
        respData[0] = gRegs[NSTAR_REG_FPGA_VERSION];
        respData[1] = gRegs[NSTAR_REG_FPGA_BUILD];
        respData[2] = gRegs[NSTAR_REG_HW_ID_YEAR];
        respData[3] = gRegs[NSTAR_REG_HW_ID_WEEK];
        respData[4] = gRegs[NSTAR_REG_HW_ID_ORDER_H];
        respData[5] = gRegs[NSTAR_REG_HW_ID_ORDER_L];
        respData[6] = gRegs[NSTAR_REG_FPGA_TYPE];
        respData[7] = gRegs[NSTAR_REG_FPGA_OPTION_H];
        respData[8] = gRegs[NSTAR_REG_FPGA_OPTION_L];
        *respLen = 9;
        return 'V';
    }

    case 'R': {
        if (dataLen < 1) { *respLen = 0; return 'A'; }
        uint8_t addr = data[0];
        uint8_t val  = gRegs[addr];

        /*
         * Register 0x40 doubles as TX_MODE (write) and TX_STATUS (read)
         * per the IRD. Bit 4 (NSTAR_TX_STATUS_CLOCK_DETECTED) reports
         * whether the TX clock is present on CLK_TX — this is a real
         * hardware signal nstar_core.c's NSTAR_TXStart() checks before
         * issuing the Modulation command. The simulator has no actual
         * clock line to sense, so gTXClockDetected stands in for it,
         * settable via the TX_CLOCK_DETECTED control command so SIL
         * scenarios can exercise both the success and NSTAR_ERR_NO_CLOCK
         * paths without needing a real clock signal.
         */
        if (addr == NSTAR_REG_TX_MODE && gTXClockDetected) {
            val |= NSTAR_TX_STATUS_CLOCK_DETECTED;
        }

        respData[0] = val;
        *respLen = 1;
        return 'R';
    }

    case 'W': {
        if (dataLen < 2) { respData[0] = 0x01; *respLen = 1; return 'A'; }
        uint8_t addr = data[0];
        uint8_t val  = data[1];
        gRegs[addr] = val;
        respData[0] = 0x00;   /* ACK success */
        *respLen = 1;
        return 'A';
    }

    case 'E': {
        /* All RX status registers 0x10-0x22 inclusive = 19 bytes */
        for (int i = 0; i < 19; i++) {
            respData[i] = gRegs[0x10 + i];
        }
        *respLen = 19;
        return 'E';
    }

    case 'C': {
        /* Reset command — magic word check skipped for simplicity in sim;
         * any 2-byte payload triggers a register reset, matching the
         * documented behaviour (N-STAR reverts registers to defaults). */
        setRegisterDefaults();
        respData[0] = 0x00;
        *respLen = 1;
        return 'A';
    }

    default:
        respData[0] = 0xFF;
        *respLen = 1;
        return 'A';
    }
}

/* =========================================================================
 * Main loop
 * =========================================================================
 */

int main(void)
{
    signal(SIGINT,  sighandler);
    signal(SIGTERM, sighandler);

    setRegisterDefaults();

    int masterFd, slaveFd;
    char slaveName[256];

    if (openpty(&masterFd, &slaveFd, slaveName, NULL, NULL) != 0) {
        fprintf(stderr, "[nstar_sim] openpty failed: %s\n", strerror(errno));
        return 1;
    }

    /* Close our copy of the slave fd — nstar_app_sil opens it independently
     * by path. Keeping a second open slave fd around is harmless but
     * unnecessary; closing it makes lsof/debugging cleaner. */
    close(slaveFd);

    fprintf(stderr, "[nstar_sim] PTY slave: %s\n", slaveName);

    char linkPath[512];
    silPath(linkPath, sizeof(linkPath), SIL_PTY_LINK_FILE);
    FILE *lf = fopen(linkPath, "w");
    if (!lf) {
        fprintf(stderr, "[nstar_sim] cannot write %s: %s\n",
                linkPath, strerror(errno));
        return 1;
    }
    fprintf(lf, "%s", slaveName);
    fclose(lf);

    char readyPath[512];
    silPath(readyPath, sizeof(readyPath), SIL_SIM_READY_FILE);
    FILE *rf = fopen(readyPath, "w");
    if (rf) fclose(rf);

    fprintf(stderr, "[nstar_sim] ready, waiting for frames\n");

    uint8_t rxBuf[NSTAR_FRAME_BUF_MAX];

    while (gRunning) {
        applyControlCommands();

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(masterFd, &rfds);
        struct timeval tv = { 0, 100 * 1000 };  /* 100ms poll for control file */

        int sel = select(masterFd + 1, &rfds, NULL, NULL, &tv);
        if (sel < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (sel == 0) continue;   /* no frame yet — loop to re-poll control */

        ssize_t n = read(masterFd, rxBuf, sizeof(rxBuf));
        if (n <= 0) continue;

        if (gForceTimeout) {
            fprintf(stderr, "[nstar_sim] FORCE_TIMEOUT active — dropping frame\n");
            continue;
        }

        char    cmdId   = 0;
        uint8_t data[NSTAR_FRAME_BUF_MAX / 2];
        size_t  dataLen = 0;

        NSTAR_Result_t rc = nstarFrameDecode(rxBuf, (size_t)n,
                                                &cmdId, data, &dataLen);
        if (rc != NSTAR_OK) {
            fprintf(stderr, "[nstar_sim] frame decode error %d, ignoring\n", rc);
            continue;
        }

        uint8_t respData[32];
        size_t  respLen = 0;
        char respCmd = handleCommand(cmdId, data, dataLen,
                                        respData, &respLen);

        uint8_t txFrame[NSTAR_FRAME_BUF_MAX];
        size_t  txLen = 0;
        nstarFrameEncode(respCmd, respData, respLen, txFrame, &txLen);

        ssize_t written = 0;
        while (written < (ssize_t)txLen) {
            ssize_t w = write(masterFd, txFrame + written,
                               txLen - (size_t)written);
            if (w < 0) {
                if (errno == EINTR) continue;
                break;
            }
            written += w;
        }

        fprintf(stderr, "[nstar_sim] %c (%zu bytes) -> %c (%zu bytes)\n",
                cmdId, dataLen, respCmd, respLen);
    }

    fprintf(stderr, "[nstar_sim] shutting down\n");
    close(masterFd);
    unlink(linkPath);
    unlink(readyPath);
    return 0;
}
