/**
 * @file  nstar_sim.c
 * @brief Standalone N-STAR simulator process for SIL testing.
 *
 * Opens a PTY master/slave pair, writes the slave device path to
 * SIL_PTY_LINK_FILE so nstar_app_sil can open the same line as its UART,
 * then runs a loop: read a frame, decode it with the REAL
 * nstar_frame_decode(), look up the command, mutate/report register
 * state, encode a response with the REAL nstar_frame_encode(), write it
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
#include "nstar.h"
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
static uint8_t g_regs[REG_COUNT];
static int     g_force_timeout = 0;
static int     g_tx_clock_detected = 1;   /* default: clock present */
static volatile int g_running  = 1;

static void set_register_defaults(void)
{
    memset(g_regs, 0, sizeof(g_regs));
    g_regs[NSTAR_REG_FPGA_VERSION]   = 0x01;
    g_regs[NSTAR_REG_FPGA_BUILD]     = 0x00;
    g_regs[NSTAR_REG_HW_ID_YEAR]     = 0x18;
    g_regs[NSTAR_REG_HW_ID_WEEK]     = 0x23;
    g_regs[NSTAR_REG_HW_ID_ORDER_H]  = 0x00;
    g_regs[NSTAR_REG_HW_ID_ORDER_L]  = 0x42;
    g_regs[NSTAR_REG_FPGA_TYPE]      = NSTAR_FPGA_TYPE_EXPECTED;  /* 0x62 */
    g_regs[NSTAR_REG_FPGA_OPTION_H]  = 0x03;
    g_regs[NSTAR_REG_FPGA_OPTION_L]  = 0x07;
    /* PA temp default ~36C: raw = (36+50)/0.06105 ~= 1409 = 0x0581 */
    g_regs[NSTAR_REG_ADC_PA_TEMP_MSB] = 0x05;
    g_regs[NSTAR_REG_ADC_PA_TEMP_LSB] = 0x81;
    /* BB temp default ~23C: raw = (23+50)/0.06105 ~= 1196 = 0x04AC */
    g_regs[NSTAR_REG_ADC_BB_TEMP_MSB] = 0x04;
    g_regs[NSTAR_REG_ADC_BB_TEMP_LSB] = 0xAC;
}

static void sighandler(int sig)
{
    (void)sig;
    g_running = 0;
}

/* =========================================================================
 * Control file polling
 * =========================================================================
 */

static void apply_control_commands(void)
{
    char path[512];
    sil_path(path, sizeof(path), SIL_SIM_CONTROL_FILE);

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
            g_regs[NSTAR_REG_FPGA_TYPE] = (uint8_t)strtol(line + 10, NULL, 16);
            fprintf(stderr, "[nstar_sim] FPGA_TYPE -> 0x%02X\n",
                    g_regs[NSTAR_REG_FPGA_TYPE]);
        } else if (strncmp(line, "PA_TEMP_RAW=", 12) == 0) {
            long raw = strtol(line + 12, NULL, 10);
            g_regs[NSTAR_REG_ADC_PA_TEMP_MSB] = (uint8_t)((raw >> 8) & 0xFF);
            g_regs[NSTAR_REG_ADC_PA_TEMP_LSB] = (uint8_t)(raw & 0xFF);
            fprintf(stderr, "[nstar_sim] PA_TEMP_RAW -> %ld\n", raw);
        } else if (strncmp(line, "BB_TEMP_RAW=", 12) == 0) {
            long raw = strtol(line + 12, NULL, 10);
            g_regs[NSTAR_REG_ADC_BB_TEMP_MSB] = (uint8_t)((raw >> 8) & 0xFF);
            g_regs[NSTAR_REG_ADC_BB_TEMP_LSB] = (uint8_t)(raw & 0xFF);
            fprintf(stderr, "[nstar_sim] BB_TEMP_RAW -> %ld\n", raw);
        } else if (strncmp(line, "FORCE_TIMEOUT=", 14) == 0) {
            g_force_timeout = (int)strtol(line + 14, NULL, 10);
            fprintf(stderr, "[nstar_sim] FORCE_TIMEOUT -> %d\n", g_force_timeout);
        } else if (strncmp(line, "TX_CLOCK_DETECTED=", 18) == 0) {
            g_tx_clock_detected = (int)strtol(line + 18, NULL, 10);
            fprintf(stderr, "[nstar_sim] TX_CLOCK_DETECTED -> %d\n",
                    g_tx_clock_detected);
        } else if (strncmp(line, "RESET", 5) == 0) {
            set_register_defaults();
            g_force_timeout = 0;
            g_tx_clock_detected = 1;
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
 * Returns the response CMD_ID character and fills resp_data/resp_len.
 * For W commands, also performs the register write as a side effect.
 */

static char handle_command(char cmd_id, const uint8_t *data, size_t data_len,
                            uint8_t *resp_data, size_t *resp_len)
{
    switch (cmd_id) {

    case 'V': {
        /* Identity: FPGA_VERSION, FPGA_BUILD, HW_YEAR, HW_WEEK,
         * HW_ORDER(2), FPGA_TYPE, FPGA_OPTION(2) = 9 bytes */
        resp_data[0] = g_regs[NSTAR_REG_FPGA_VERSION];
        resp_data[1] = g_regs[NSTAR_REG_FPGA_BUILD];
        resp_data[2] = g_regs[NSTAR_REG_HW_ID_YEAR];
        resp_data[3] = g_regs[NSTAR_REG_HW_ID_WEEK];
        resp_data[4] = g_regs[NSTAR_REG_HW_ID_ORDER_H];
        resp_data[5] = g_regs[NSTAR_REG_HW_ID_ORDER_L];
        resp_data[6] = g_regs[NSTAR_REG_FPGA_TYPE];
        resp_data[7] = g_regs[NSTAR_REG_FPGA_OPTION_H];
        resp_data[8] = g_regs[NSTAR_REG_FPGA_OPTION_L];
        *resp_len = 9;
        return 'V';
    }

    case 'R': {
        if (data_len < 1) { *resp_len = 0; return 'A'; }
        uint8_t addr = data[0];
        uint8_t val  = g_regs[addr];

        /*
         * Register 0x40 doubles as TX_MODE (write) and TX_STATUS (read)
         * per the IRD. Bit 4 (NSTAR_TX_STATUS_CLOCK_DETECTED) reports
         * whether the TX clock is present on CLK_TX — this is a real
         * hardware signal nstar_core.c's nstar_tx_start() checks before
         * issuing the Modulation command. The simulator has no actual
         * clock line to sense, so g_tx_clock_detected stands in for it,
         * settable via the TX_CLOCK_DETECTED control command so SIL
         * scenarios can exercise both the success and NSTAR_ERR_NO_CLOCK
         * paths without needing a real clock signal.
         */
        if (addr == NSTAR_REG_TX_MODE && g_tx_clock_detected) {
            val |= NSTAR_TX_STATUS_CLOCK_DETECTED;
        }

        resp_data[0] = val;
        *resp_len = 1;
        return 'R';
    }

    case 'W': {
        if (data_len < 2) { resp_data[0] = 0x01; *resp_len = 1; return 'A'; }
        uint8_t addr = data[0];
        uint8_t val  = data[1];
        g_regs[addr] = val;
        resp_data[0] = 0x00;   /* ACK success */
        *resp_len = 1;
        return 'A';
    }

    case 'E': {
        /* All RX status registers 0x10-0x22 inclusive = 19 bytes */
        for (int i = 0; i < 19; i++) {
            resp_data[i] = g_regs[0x10 + i];
        }
        *resp_len = 19;
        return 'E';
    }

    case 'C': {
        /* Reset command — magic word check skipped for simplicity in sim;
         * any 2-byte payload triggers a register reset, matching the
         * documented behaviour (N-STAR reverts registers to defaults). */
        set_register_defaults();
        resp_data[0] = 0x00;
        *resp_len = 1;
        return 'A';
    }

    default:
        resp_data[0] = 0xFF;
        *resp_len = 1;
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

    set_register_defaults();

    int master_fd, slave_fd;
    char slave_name[256];

    if (openpty(&master_fd, &slave_fd, slave_name, NULL, NULL) != 0) {
        fprintf(stderr, "[nstar_sim] openpty failed: %s\n", strerror(errno));
        return 1;
    }

    /* Close our copy of the slave fd — nstar_app_sil opens it independently
     * by path. Keeping a second open slave fd around is harmless but
     * unnecessary; closing it makes lsof/debugging cleaner. */
    close(slave_fd);

    fprintf(stderr, "[nstar_sim] PTY slave: %s\n", slave_name);

    char link_path[512];
    sil_path(link_path, sizeof(link_path), SIL_PTY_LINK_FILE);
    FILE *lf = fopen(link_path, "w");
    if (!lf) {
        fprintf(stderr, "[nstar_sim] cannot write %s: %s\n",
                link_path, strerror(errno));
        return 1;
    }
    fprintf(lf, "%s", slave_name);
    fclose(lf);

    char ready_path[512];
    sil_path(ready_path, sizeof(ready_path), SIL_SIM_READY_FILE);
    FILE *rf = fopen(ready_path, "w");
    if (rf) fclose(rf);

    fprintf(stderr, "[nstar_sim] ready, waiting for frames\n");

    uint8_t rx_buf[NSTAR_FRAME_BUF_MAX];

    while (g_running) {
        apply_control_commands();

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(master_fd, &rfds);
        struct timeval tv = { 0, 100 * 1000 };  /* 100ms poll for control file */

        int sel = select(master_fd + 1, &rfds, NULL, NULL, &tv);
        if (sel < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (sel == 0) continue;   /* no frame yet — loop to re-poll control */

        ssize_t n = read(master_fd, rx_buf, sizeof(rx_buf));
        if (n <= 0) continue;

        if (g_force_timeout) {
            fprintf(stderr, "[nstar_sim] FORCE_TIMEOUT active — dropping frame\n");
            continue;
        }

        char    cmd_id   = 0;
        uint8_t data[NSTAR_FRAME_BUF_MAX / 2];
        size_t  data_len = 0;

        nstar_result_t rc = nstar_frame_decode(rx_buf, (size_t)n,
                                                &cmd_id, data, &data_len);
        if (rc != NSTAR_OK) {
            fprintf(stderr, "[nstar_sim] frame decode error %d, ignoring\n", rc);
            continue;
        }

        uint8_t resp_data[32];
        size_t  resp_len = 0;
        char resp_cmd = handle_command(cmd_id, data, data_len,
                                        resp_data, &resp_len);

        uint8_t tx_frame[NSTAR_FRAME_BUF_MAX];
        size_t  tx_len = 0;
        nstar_frame_encode(resp_cmd, resp_data, resp_len, tx_frame, &tx_len);

        ssize_t written = 0;
        while (written < (ssize_t)tx_len) {
            ssize_t w = write(master_fd, tx_frame + written,
                               tx_len - (size_t)written);
            if (w < 0) {
                if (errno == EINTR) continue;
                break;
            }
            written += w;
        }

        fprintf(stderr, "[nstar_sim] %c (%zu bytes) -> %c (%zu bytes)\n",
                cmd_id, data_len, resp_cmd, resp_len);
    }

    fprintf(stderr, "[nstar_sim] shutting down\n");
    close(master_fd);
    unlink(link_path);
    unlink(ready_path);
    return 0;
}
