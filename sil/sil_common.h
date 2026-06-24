/**
 * @file  sil_common.h
 * @brief Shared paths and constants for SIL (Software-In-the-Loop) testing.
 *
 * Three independent processes agree on this file:
 *   - nstar_sim       (sil/nstar_sim.c)      — N-STAR simulator
 *   - nstar_app_sil   (sil/nstar_app_sil.c)  — application under test
 *   - testSil        (sil/testSil.c)       — SIL test suite (orchestrator)
 *
 * All paths are under a single SIL_ROOT directory so a test run is fully
 * self-contained and easy to inspect or clean up.
 *
 * SIL_ROOT defaults to /tmp/nstar_sil but can be overridden via the
 * NSTAR_SIL_ROOT environment variable so multiple SIL runs (e.g. in CI)
 * don't collide.
 */

#ifndef SIL_COMMON_H
#define SIL_COMMON_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SIL_DEFAULT_ROOT   "/tmp/nstar_sil"

/** Resolve the SIL root directory (env override or default). Caller owns no memory — returns a static buffer. */
static inline const char *silRoot(void)
{
    const char *env = getenv("NSTAR_SIL_ROOT");
    return (env && env[0] != '\0') ? env : SIL_DEFAULT_ROOT;
}

/** Build a path under the SIL root into the caller-supplied buffer. */
static inline void silPath(char *out, size_t outLen, const char *leaf)
{
    snprintf(out, outLen, "%s/%s", silRoot(), leaf);
}

/* -------------------------------------------------------------------------
 * File leaf names
 * -------------------------------------------------------------------------
 *
 * PTY link path: written by nstar_sim once it has opened the PTY master
 * and obtained the slave device path (e.g. /dev/pts/4). nstar_app_sil
 * reads this file to know which device to open as its UART.
 */
#define SIL_PTY_LINK_FILE        "uart_pty_path.txt"

/* GPIO lines: plain text files containing "0" or "1".
 * The dummy HAL polls these for nstarHALGPIORead() and
 * nstarHALGPIOWaitEdge(); the SIL test suite writes them directly.
 * nstarHALGPIOWrite() (RESET_N is the only output line) writes here too,
 * and the simulator polls reset_n to detect a forced reset. */
#define SIL_GPIO_LOCK_DETECT_FILE "gpioLockDetect.txt"
#define SIL_GPIO_DATA_VALID_FILE  "gpioDataValid.txt"
#define SIL_GPIO_FAULT_N_FILE     "gpioFaultN.txt"
#define SIL_GPIO_RESET_N_FILE     "gpioResetN.txt"

/* Data interface: TX writes appended here, RX reads consumed from here.
 * Both append-only from the producer's perspective; the dummy HAL tracks
 * its own read offset across calls within one process lifetime. */
#define SIL_DATA_TX_FILE         "data_tx.bin"
#define SIL_DATA_RX_FILE         "data_rx.bin"

/* Data clock state: "1" while nstarHALDataClockStart() has been called
 * and not yet stopped. The simulator does not need this directly, but the
 * SIL test suite reads it to confirm TX clock control behaviour. */
#define SIL_DATA_CLOCK_FILE      "data_clock_state.txt"

/* Application status file: nstar_app_sil rewrites this atomically on every
 * moduleState transition, RX state transition, fault, and TX completion.
 * Format: one KEY=VALUE pair per line, always rewritten in full (never
 * appended), so a reader never sees a half-written line if it re-reads
 * after noticing the mtime changed. See sil_status.h for the writer and
 * silStatusRead() for the reader used by the test suite. */
#define SIL_STATUS_FILE          "app_status.txt"

/* Simulator control file: the SIL test suite writes commands here that
 * nstar_sim polls and applies before processing the next UART frame.
 * One command per line, consumed and truncated by the simulator.
 * Supported commands (see nstar_sim.c for the authoritative list):
 *   FPGA_TYPE=<hex>      override the FPGA_TYPE register reported by V/R
 *   PA_TEMP_RAW=<dec>    override the PA temperature ADC raw value
 *   BB_TEMP_RAW=<dec>    override the BB temperature ADC raw value
 *   FORCE_TIMEOUT=1|0    when 1, sim stops responding to any command
 *   RESET                reset all simulator register state to defaults
 */
#define SIL_SIM_CONTROL_FILE     "simControl.txt"

/* Simulator ready marker: nstar_sim creates this file (empty, just its
 * existence matters) once it has opened the PTY and is ready to receive
 * frames. The SIL test suite waits for this before starting nstar_app_sil. */
#define SIL_SIM_READY_FILE       "sim_ready.txt"

/* Application control file: the SIL test suite writes commands here that
 * nstar_app_sil polls in its monitor loop and acts on by calling the real
 * nstar_core.c public API (nstarTXStart, nstarTXWrite, nstarTXStop).
 * One command per line, consumed and truncated after processing.
 * Supported commands (see nstar_app_sil.c for the authoritative list):
 *   TX_START=<rate_hex>   calls nstarTXStart() with the given rate code
 *   TX_WRITE=<nbytes>     calls nstarTXWrite() with <nbytes> of a fixed
 *                         deterministic pattern (i & 0xFF per byte)
 *   TX_STOP               calls nstarTXStop()
 */
#define SIL_APP_CONTROL_FILE     "appControl.txt"

#endif /* SIL_COMMON_H */
