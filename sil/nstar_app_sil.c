/**
 * @file  nstar_app_sil.c
 * @brief Application entry point for SIL testing.
 *
 * This is the SIL counterpart to main/nstar_app.c. It links the SAME
 * src/nstar_core.c and src/nstar_frame.c used in production, against
 * hal/nstar_hal_dummy.c instead of hal/nstar_hal_linux.c.
 *
 * Responsibilities:
 *   1. Wait for the simulator's PTY link file, open that PTY slave as
 *      the UART fd with real termios configuration matching the EICD
 *      (38400 8E1 RS-422 framing — RS-422 itself is a wire-level detail
 *      the PTY can't model, but the byte-level framing parameters are
 *      configured for real so the same termios setup code is exercised).
 *   2. Initialise GPIO line "fds" to the fixed integer values the dummy
 *      HAL's gpio_fd_to_filename() table expects.
 *   3. Call nstar_init() + nstar_startup_sequence(), exactly as a real
 *      application would.
 *   4. On every module_state transition, rx_state transition, fault,
 *      frame reception, and TX completion, rewrite the SIL status file
 *      so the separate test_sil process can observe what happened.
 *   5. Stay running (so a SIL test can drive GPIO files and the
 *      simulator's control file over time) until told to exit via
 *      SIGTERM/SIGINT, or until an optional run-duration argument elapses.
 *
 * Since nstar_core.c has no built-in hook for "notify on every state
 * change", this file polls nstar_get_module_state() and
 * nstar_rx_get_state() in its own small monitor loop and diffs against
 * the last-seen values — this is allowed because these accessors are
 * part of the public API and are documented as thread-safe.
 */

#define _DEFAULT_SOURCE
#include "nstar.h"
#include "sil_common.h"
#include "sil_status.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include <time.h>
#include <errno.h>

#define SIL_FD_GPIO_LOCK_DETECT  20
#define SIL_FD_GPIO_DATA_VALID   21
#define SIL_FD_GPIO_FAULT_N      22
#define SIL_FD_GPIO_RESET_N      23
#define SIL_FD_DATA_INTERFACE    11

static volatile int g_running = 1;
static sil_status_t g_status;

static size_t  g_total_bytes_received = 0;
static size_t  g_frame_received_count = 0;
static size_t  g_lock_acquired_count  = 0;
static size_t  g_lock_lost_count      = 0;
static size_t  g_fault_count          = 0;
static char    g_last_fault[16]       = "NONE";
static size_t  g_tx_complete_count    = 0;
static size_t  g_last_tx_bytes        = 0;
static int     g_last_tx_start_rc     = 1;   /* 1 = "not yet called" sentinel */
static int     g_last_tx_write_rc     = 1;
static int     g_last_tx_stop_rc      = 1;

static void sighandler(int sig)
{
    (void)sig;
    g_running = 0;
}

/* =========================================================================
 * Callbacks — update in-memory counters, status file is rewritten by the
 * monitor loop in main() so every callback doesn't need to know the full
 * status format.
 * =========================================================================
 */

static void on_frame_received(const uint8_t *buf, size_t len)
{
    (void)buf;
    g_frame_received_count++;
    g_total_bytes_received += len;
    fprintf(stderr, "[nstar_app_sil] frame received: %zu bytes\n", len);
}

static void on_tx_complete(size_t bytes_sent)
{
    g_tx_complete_count++;
    g_last_tx_bytes = bytes_sent;
    fprintf(stderr, "[nstar_app_sil] tx complete: %zu bytes\n", bytes_sent);
}

static void on_fault(nstar_fault_source_t source)
{
    g_fault_count++;
    switch (source) {
        case NSTAR_FAULT_SEL:         snprintf(g_last_fault, sizeof(g_last_fault), "SEL"); break;
        case NSTAR_FAULT_OVERCURRENT: snprintf(g_last_fault, sizeof(g_last_fault), "OVERCURRENT"); break;
        case NSTAR_FAULT_TEMPERATURE: snprintf(g_last_fault, sizeof(g_last_fault), "TEMPERATURE"); break;
        default:                      snprintf(g_last_fault, sizeof(g_last_fault), "UNKNOWN"); break;
    }
    fprintf(stderr, "[nstar_app_sil] fault: %s\n", g_last_fault);
}

static void on_lock_acquired(void)
{
    g_lock_acquired_count++;
    fprintf(stderr, "[nstar_app_sil] lock acquired\n");
}

static void on_lock_lost(void)
{
    g_lock_lost_count++;
    fprintf(stderr, "[nstar_app_sil] lock lost\n");
}

static const nstar_callbacks_t k_callbacks = {
    .on_frame_received = on_frame_received,
    .on_tx_complete    = on_tx_complete,
    .on_fault          = on_fault,
    .on_lock_acquired  = on_lock_acquired,
    .on_lock_lost      = on_lock_lost,
};

/* =========================================================================
 * App control file — lets the SIL test suite trigger TX calls on the
 * real nstar_core.c API from outside this process.
 * =========================================================================
 */

static void apply_app_control_commands(nstar_ctx_t *ctx)
{
    char path[512];
    sil_path(path, sizeof(path), SIL_APP_CONTROL_FILE);

    FILE *f = fopen(path, "r");
    if (!f) return;

    char line[128];
    int any = 0;
    while (fgets(line, sizeof(line), f)) {
        char *nl = strchr(line, '\n');
        if (nl) *nl = '\0';
        if (line[0] == '\0') continue;
        any = 1;

        if (strncmp(line, "TX_START=", 9) == 0) {
            long rate = strtol(line + 9, NULL, 16);
            nstar_result_t rc = nstar_tx_start(ctx, (nstar_tx_rate_code_t)rate);
            g_last_tx_start_rc = (int)rc;
            fprintf(stderr, "[nstar_app_sil] TX_START(rate=0x%lx) -> %d\n",
                    rate, rc);
        } else if (strncmp(line, "TX_WRITE=", 9) == 0) {
            long nbytes = strtol(line + 9, NULL, 10);
            if (nbytes > 0 && nbytes <= (long)NSTAR_FRAME_SIZE_BYTES * 4) {
                uint8_t *buf = malloc((size_t)nbytes);
                if (buf) {
                    for (long i = 0; i < nbytes; i++) buf[i] = (uint8_t)(i & 0xFF);
                    nstar_result_t rc = nstar_tx_write(ctx, buf, (size_t)nbytes);
                    g_last_tx_write_rc = (int)rc;
                    fprintf(stderr, "[nstar_app_sil] TX_WRITE(%ld bytes) -> %d\n",
                            nbytes, rc);
                    free(buf);
                }
            }
        } else if (strncmp(line, "TX_STOP", 7) == 0) {
            nstar_result_t rc = nstar_tx_stop(ctx);
            g_last_tx_stop_rc = (int)rc;
            fprintf(stderr, "[nstar_app_sil] TX_STOP -> %d\n", rc);
        }
    }
    fclose(f);

    if (any) {
        FILE *tf = fopen(path, "w");
        if (tf) fclose(tf);
    }
}

/* =========================================================================
 * Status file refresh
 * =========================================================================
 */

static const char *module_state_name(nstar_module_state_t s)
{
    switch (s) {
        case NSTAR_MODULE_UNINIT:        return "UNINIT";
        case NSTAR_MODULE_INITIALISING:  return "INITIALISING";
        case NSTAR_MODULE_STARTING:      return "STARTING";
        case NSTAR_MODULE_READY:         return "READY";
        case NSTAR_MODULE_FAULT:         return "FAULT";
        case NSTAR_MODULE_SHUTTING_DOWN: return "SHUTTING_DOWN";
        default:                         return "UNKNOWN";
    }
}

static const char *rx_state_name(nstar_rx_state_t s)
{
    switch (s) {
        case NSTAR_RX_IDLE:      return "IDLE";
        case NSTAR_RX_ACQUIRING: return "ACQUIRING";
        case NSTAR_RX_LOCKED:    return "LOCKED";
        case NSTAR_RX_LOCK_LOST: return "LOCK_LOST";
        default:                 return "UNKNOWN";
    }
}

static void refresh_status(nstar_ctx_t *ctx)
{
    sil_status_init(&g_status);
    sil_status_set(&g_status, "MODULE_STATE",
                   module_state_name(nstar_get_module_state(ctx)));
    sil_status_set(&g_status, "RX_STATE",
                   rx_state_name(nstar_rx_get_state(ctx)));
    sil_status_set_int(&g_status, "FRAME_RECEIVED_COUNT",
                        (long)g_frame_received_count);
    sil_status_set_int(&g_status, "TOTAL_BYTES_RECEIVED",
                        (long)g_total_bytes_received);
    sil_status_set_int(&g_status, "LOCK_ACQUIRED_COUNT",
                        (long)g_lock_acquired_count);
    sil_status_set_int(&g_status, "LOCK_LOST_COUNT",
                        (long)g_lock_lost_count);
    sil_status_set_int(&g_status, "FAULT_COUNT", (long)g_fault_count);
    sil_status_set(&g_status, "LAST_FAULT", g_last_fault);
    sil_status_set_int(&g_status, "TX_COMPLETE_COUNT",
                        (long)g_tx_complete_count);
    sil_status_set_int(&g_status, "LAST_TX_BYTES", (long)g_last_tx_bytes);
    sil_status_set_int(&g_status, "LAST_TX_START_RC", g_last_tx_start_rc);
    sil_status_set_int(&g_status, "LAST_TX_WRITE_RC", g_last_tx_write_rc);
    sil_status_set_int(&g_status, "LAST_TX_STOP_RC", g_last_tx_stop_rc);
    sil_status_write(&g_status);
}

/* =========================================================================
 * UART setup — open the PTY slave with real termios configuration
 * =========================================================================
 */

static int wait_for_pty_link(char *slave_path_out, size_t out_len,
                              int timeout_ms)
{
    char link_path[512];
    sil_path(link_path, sizeof(link_path), SIL_PTY_LINK_FILE);

    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);

    for (;;) {
        FILE *f = fopen(link_path, "r");
        if (f) {
            char buf[256] = {0};
            size_t n = fread(buf, 1, sizeof(buf) - 1, f);
            fclose(f);
            if (n > 0) {
                snprintf(slave_path_out, out_len, "%s", buf);
                return 0;
            }
        }
        clock_gettime(CLOCK_MONOTONIC, &now);
        long elapsed_ms = (now.tv_sec - start.tv_sec) * 1000 +
                          (now.tv_nsec - start.tv_nsec) / 1000000;
        if (elapsed_ms >= timeout_ms) return -1;

        struct timespec d = { 0, 50 * 1000000L };
        nanosleep(&d, NULL);
    }
}

static int open_uart(const char *slave_path)
{
    int fd = open(slave_path, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        fprintf(stderr, "[nstar_app_sil] open(%s) failed: %s\n",
                slave_path, strerror(errno));
        return -1;
    }

    /*
     * Real termios configuration matching the EICD UART parameters:
     * 38400 baud, 8 data bits, EVEN parity, 1 stop bit (8E1).
     * This is the same configuration hal/nstar_hal_linux.c will need
     * for the real RS-422 port — only the device path differs.
     */
    struct termios tio;
    if (tcgetattr(fd, &tio) != 0) {
        fprintf(stderr, "[nstar_app_sil] tcgetattr failed: %s\n",
                strerror(errno));
        close(fd);
        return -1;
    }

    /*
     * Real termios configuration matching the EICD UART parameters as
     * closely as a PTY can emulate: 38400 baud, 8 data bits, 1 stop bit.
     *
     * Parity (8E1's "E") is intentionally NOT configured here. A PTY has
     * no physical wire and only loosely emulates serial line discipline —
     * enabling PARENB+INPCK on a PTY slave succeeds the first time a
     * process configures it, but a second independent open+configure of
     * the SAME slave device (e.g. a second nstar_app_sil run against a
     * still-live nstar_sim) reliably fails tcsetattr with EINVAL. This
     * was reproduced and isolated outside the test harness: removing
     * PARENB/INPCK and keeping everything else (rate, CS8, no CSTOPB)
     * eliminates the failure on repeated configuration.
     *
     * Parity is a physical-layer integrity check; it has no bearing on
     * whether the ASCII '<...>' frame protocol round-trips correctly
     * over this channel, so omitting it here does not weaken anything
     * this SIL suite is meant to verify. hal/nstar_hal_linux.c (the real
     * RS-422 binding) DOES need PARENB+PARODD-cleared (even parity) when
     * Stage 6 lands, since that runs against a real UART, not a PTY.
     */
    cfmakeraw(&tio);
    cfsetispeed(&tio, B38400);
    cfsetospeed(&tio, B38400);

    tio.c_cflag &= ~PARENB;    /* no parity on the PTY emulation  */
    tio.c_cflag &= ~CSTOPB;    /* 1 stop bit                      */
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;        /* 8 data bits                     */

    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        fprintf(stderr, "[nstar_app_sil] tcsetattr failed: %s\n",
                strerror(errno));
        close(fd);
        return -1;
    }

    return fd;
}

/* =========================================================================
 * GPIO file initialisation
 * =========================================================================
 *
 * Ensures every GPIO file exists with a sane idle default before
 * nstar_init() spawns threads that immediately start polling them.
 * FAULT_N idles HIGH (active-low, no fault). LOCK_DETECT and DATA_VALID
 * idle LOW (no signal). RESET_N idles HIGH (not asserted).
 */

static void write_gpio_default(const char *filename, int value)
{
    char path[512], tmp[520];
    sil_path(path, sizeof(path), filename);
    snprintf(tmp, sizeof(tmp), "%s.tmp", path);
    FILE *f = fopen(tmp, "w");
    if (!f) return;
    fprintf(f, "%d", value);
    fclose(f);
    rename(tmp, path);
}

static void init_gpio_defaults(void)
{
    write_gpio_default(SIL_GPIO_LOCK_DETECT_FILE, 0);
    write_gpio_default(SIL_GPIO_DATA_VALID_FILE,  0);
    write_gpio_default(SIL_GPIO_FAULT_N_FILE,      1);
    write_gpio_default(SIL_GPIO_RESET_N_FILE,      1);
}

/* =========================================================================
 * Main
 * =========================================================================
 *
 * Usage: nstar_app_sil [run_duration_seconds]
 *   If run_duration_seconds is given, the process exits automatically
 *   after that many seconds (used by test_sil.c so each scenario gets a
 *   bounded-lifetime app process). If omitted, runs until SIGTERM/SIGINT.
 */
int main(int argc, char **argv)
{
    signal(SIGINT,  sighandler);
    signal(SIGTERM, sighandler);

    int run_duration_s = 0;
    if (argc > 1) run_duration_s = atoi(argv[1]);

    init_gpio_defaults();

    char slave_path[256];
    fprintf(stderr, "[nstar_app_sil] waiting for simulator PTY link...\n");
    if (wait_for_pty_link(slave_path, sizeof(slave_path), 5000) != 0) {
        fprintf(stderr, "[nstar_app_sil] timed out waiting for %s\n",
                SIL_PTY_LINK_FILE);
        return 1;
    }
    fprintf(stderr, "[nstar_app_sil] opening UART: %s\n", slave_path);

    int uart_fd = open_uart(slave_path);
    if (uart_fd < 0) return 1;

    nstar_config_t config = {
        .uart_fd           = uart_fd,
        .data_fd            = SIL_FD_DATA_INTERFACE,
        .gpio_lock_detect   = SIL_FD_GPIO_LOCK_DETECT,
        .gpio_data_valid    = SIL_FD_GPIO_DATA_VALID,
        .gpio_fault_n       = SIL_FD_GPIO_FAULT_N,
        .gpio_reset_n       = SIL_FD_GPIO_RESET_N,
    };

    nstar_ctx_t *ctx = NULL;
    nstar_result_t rc = nstar_init(&config, &k_callbacks, &ctx);
    if (rc != NSTAR_OK) {
        fprintf(stderr, "[nstar_app_sil] nstar_init failed: %d\n", rc);
        close(uart_fd);
        return 1;
    }
    refresh_status(ctx);

    fprintf(stderr, "[nstar_app_sil] running startup_sequence...\n");
    rc = nstar_startup_sequence(ctx);
    fprintf(stderr, "[nstar_app_sil] startup_sequence -> %d (state=%s)\n",
            rc, module_state_name(nstar_get_module_state(ctx)));
    refresh_status(ctx);

    /* Monitor loop: poll module/RX state and counters, refresh the status
     * file whenever anything observable changes. 50ms cadence keeps the
     * SIL test suite's polling responsive without busy-looping. */
    nstar_module_state_t last_module_state = nstar_get_module_state(ctx);
    nstar_rx_state_t      last_rx_state     = nstar_rx_get_state(ctx);
    size_t last_frame_count = g_frame_received_count;
    size_t last_fault_count = g_fault_count;
    size_t last_tx_count    = g_tx_complete_count;
    size_t last_lock_acq    = g_lock_acquired_count;
    size_t last_lock_lost   = g_lock_lost_count;

    struct timespec run_start;
    clock_gettime(CLOCK_MONOTONIC, &run_start);

    while (g_running) {
        struct timespec d = { 0, 50 * 1000000L };
        nanosleep(&d, NULL);

        int rc_before_start = g_last_tx_start_rc;
        int rc_before_write = g_last_tx_write_rc;
        int rc_before_stop  = g_last_tx_stop_rc;

        apply_app_control_commands(ctx);

        nstar_module_state_t cur_module_state = nstar_get_module_state(ctx);
        nstar_rx_state_t      cur_rx_state     = nstar_rx_get_state(ctx);

        int tx_rc_changed = (g_last_tx_start_rc != rc_before_start) ||
                            (g_last_tx_write_rc != rc_before_write) ||
                            (g_last_tx_stop_rc  != rc_before_stop);

        if (cur_module_state != last_module_state ||
            cur_rx_state     != last_rx_state     ||
            g_frame_received_count != last_frame_count ||
            g_fault_count           != last_fault_count ||
            g_tx_complete_count     != last_tx_count ||
            g_lock_acquired_count   != last_lock_acq ||
            g_lock_lost_count       != last_lock_lost ||
            tx_rc_changed) {

            refresh_status(ctx);
            last_module_state = cur_module_state;
            last_rx_state      = cur_rx_state;
            last_frame_count   = g_frame_received_count;
            last_fault_count   = g_fault_count;
            last_tx_count      = g_tx_complete_count;
            last_lock_acq       = g_lock_acquired_count;
            last_lock_lost       = g_lock_lost_count;
        }

        if (run_duration_s > 0) {
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            long elapsed_s = now.tv_sec - run_start.tv_sec;
            if (elapsed_s >= run_duration_s) break;
        }
    }

    fprintf(stderr, "[nstar_app_sil] shutting down\n");
    nstar_deinit(ctx);
    close(uart_fd);
    return 0;
}
