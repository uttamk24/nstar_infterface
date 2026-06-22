/**
 * @file  nstar_hal_dummy.c
 * @brief Dummy HAL for SIL (Software-In-the-Loop) testing.
 *
 * Unlike nstar_hal_mock.c (in-process, scripted, single-threaded, used by
 * the Stage 1-5 unit tests), this HAL does REAL I/O:
 *
 *   - UART:  real termios over a PTY slave file descriptor, real
 *            select()-based blocking read with timeout. The PTY master
 *            side is held by a separate process (sil/nstar_sim.c), so
 *            frames genuinely travel across a kernel-mediated channel —
 *            the same code path nstar_hal_linux.c will use for the real
 *            RS-422 port, just pointed at a PTY instead of a UART chip.
 *
 *   - GPIO:  plain text files ("0" or "1") polled with a short sleep
 *            interval. The SIL test suite (or a human) writes these files
 *            directly to simulate LOCK_DETECT, DATA_VALID, FAULT_N edges.
 *            RESET_N is the one GPIO OUTPUT — nstar_hal_gpio_write()
 *            writes its value to a file the simulator can poll.
 *
 *   - Data:  real file I/O. TX writes append to a file the SIL test suite
 *            can inspect after the fact. RX reads consume from a file the
 *            SIL test suite (or a future RX data generator) populates.
 *            This is the SIL stand-in for the still-open data interface
 *            (SPI or other — TBD per the IRD); the function signatures
 *            are identical to nstar_hal_linux.c's eventual real binding,
 *            so swapping back to hardware later only means changing this
 *            file, never nstar_core.c.
 *
 * File descriptors passed in by the caller (the `fd` parameter on every
 * HAL function) are NOT used to select which file/GPIO line to touch —
 * instead, each function call resolves its target path from sil_common.h
 * leaf names. This means the `nstar_config_t` fd fields are still opened
 * by the application (nstar_app_sil.c) for symmetry with the real HAL's
 * calling convention, but the dummy HAL itself works off fixed SIL paths.
 * This keeps nstar_core.c's calling code completely unmodified — it still
 * just passes ctx->config.uart_fd etc. around as opaque integers.
 */

#define _DEFAULT_SOURCE
#include "nstar.h"
#include "sil_common.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#include <time.h>
#include <errno.h>

/* =========================================================================
 * UART — real PTY I/O
 * =========================================================================
 */

ssize_t nstar_hal_uart_write(int fd, const uint8_t *buf, size_t len)
{
    size_t written = 0;
    while (written < len) {
        ssize_t n = write(fd, buf + written, len - written);
        if (n < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        written += (size_t)n;
    }
    return (ssize_t)written;
}

ssize_t nstar_hal_uart_read(int fd, uint8_t *buf, size_t len,
                             uint32_t timeout_ms)
{
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);

    struct timeval tv;
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int sel = select(fd + 1, &rfds, NULL, NULL, &tv);
    if (sel < 0) {
        if (errno == EINTR) return 0;
        return -1;
    }
    if (sel == 0) return 0;   /* timeout, no data */

    ssize_t n = read(fd, buf, len);
    if (n < 0) {
        if (errno == EINTR) return 0;
        return -1;
    }
    return n;
}

/* =========================================================================
 * GPIO — file-polled lines
 * =========================================================================
 *
 * Each GPIO "line" in the real config struct is just an integer fd field
 * (gpio_lock_detect, gpio_data_valid, gpio_fault_n, gpio_reset_n) carried
 * over unchanged from the real HAL's calling convention. The dummy HAL
 * maps each fd value to a fixed SIL file path via a small lookup table,
 * since the SIL harness needs stable, human-writable file names rather
 * than opaque fds.
 *
 * nstar_app_sil.c assigns these same fd values (see sil_common.h-adjacent
 * constants below) when it builds the nstar_config_t it passes to
 * nstar_init(), so the mapping here and the fd values there must agree.
 */

#define SIL_FD_GPIO_LOCK_DETECT  20
#define SIL_FD_GPIO_DATA_VALID   21
#define SIL_FD_GPIO_FAULT_N      22
#define SIL_FD_GPIO_RESET_N      23

static const char *gpio_fd_to_filename(int fd)
{
    switch (fd) {
        case SIL_FD_GPIO_LOCK_DETECT: return SIL_GPIO_LOCK_DETECT_FILE;
        case SIL_FD_GPIO_DATA_VALID:  return SIL_GPIO_DATA_VALID_FILE;
        case SIL_FD_GPIO_FAULT_N:     return SIL_GPIO_FAULT_N_FILE;
        case SIL_FD_GPIO_RESET_N:     return SIL_GPIO_RESET_N_FILE;
        default:                      return NULL;
    }
}

/** Read a GPIO file's current value (0 or 1). Returns -1 on error. */
static int gpio_file_read_value(const char *filename)
{
    char path[512];
    sil_path(path, sizeof(path), filename);

    FILE *f = fopen(path, "r");
    if (!f) return -1;
    char buf[8] = {0};
    size_t n = fread(buf, 1, sizeof(buf) - 1, f);
    fclose(f);
    if (n == 0) return -1;

    if (buf[0] == '1') return 1;
    if (buf[0] == '0') return 0;
    return -1;
}

/** Write a GPIO file's value (0 or 1), atomically via temp+rename. */
static int gpio_file_write_value(const char *filename, int value)
{
    char path[512], tmp[520];
    sil_path(path, sizeof(path), filename);
    snprintf(tmp, sizeof(tmp), "%s.tmp", path);

    FILE *f = fopen(tmp, "w");
    if (!f) return -1;
    fprintf(f, "%d", value ? 1 : 0);
    fclose(f);
    return rename(tmp, path);
}

int nstar_hal_gpio_read(int fd)
{
    const char *filename = gpio_fd_to_filename(fd);
    if (!filename) return -1;
    return gpio_file_read_value(filename);
}

nstar_result_t nstar_hal_gpio_write(int fd, int value)
{
    const char *filename = gpio_fd_to_filename(fd);
    if (!filename) return NSTAR_ERR_HAL;
    if (gpio_file_write_value(filename, value) != 0) return NSTAR_ERR_HAL;
    return NSTAR_OK;
}

/**
 * Poll the GPIO file every 20ms looking for the requested edge, up to
 * timeout_ms. An edge is detected by comparing the value seen on this
 * call to the value seen on the previous call for the SAME fd — exactly
 * like the mock, each fd tracks its own last-known value across calls.
 */
#define SIL_GPIO_MAX_LINES   4
static struct {
    int fd;
    int last_value;
    int has_last;
} g_gpio_last[SIL_GPIO_MAX_LINES];

static int *gpio_last_value_slot(int fd, int *has_last_out)
{
    for (int i = 0; i < SIL_GPIO_MAX_LINES; i++) {
        if (g_gpio_last[i].has_last && g_gpio_last[i].fd == fd) {
            *has_last_out = 1;
            return &g_gpio_last[i].last_value;
        }
    }
    for (int i = 0; i < SIL_GPIO_MAX_LINES; i++) {
        if (!g_gpio_last[i].has_last) {
            g_gpio_last[i].fd = fd;
            g_gpio_last[i].has_last = 1;
            g_gpio_last[i].last_value = -1;
            *has_last_out = 0;
            return &g_gpio_last[i].last_value;
        }
    }
    /* Table full — fall back to no history (treat every read as a change) */
    *has_last_out = 0;
    return NULL;
}

nstar_result_t nstar_hal_gpio_wait_edge(int fd, nstar_gpio_edge_t edge,
                                         uint32_t timeout_ms)
{
    const char *filename = gpio_fd_to_filename(fd);
    if (!filename) return NSTAR_ERR_HAL;

    int has_last = 0;
    int *last_slot = gpio_last_value_slot(fd, &has_last);

    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);

    int prev = has_last && last_slot ? *last_slot : gpio_file_read_value(filename);

    for (;;) {
        int cur = gpio_file_read_value(filename);
        if (cur >= 0) {
            int rising  = (prev == 0 && cur == 1);
            int falling = (prev == 1 && cur == 0);
            if ((edge == NSTAR_GPIO_EDGE_RISING  && rising) ||
                (edge == NSTAR_GPIO_EDGE_FALLING && falling)) {
                if (last_slot) *last_slot = cur;
                return NSTAR_OK;
            }
            prev = cur;
        }

        clock_gettime(CLOCK_MONOTONIC, &now);
        long elapsed_ms = (now.tv_sec - start.tv_sec) * 1000 +
                          (now.tv_nsec - start.tv_nsec) / 1000000;
        if (elapsed_ms >= (long)timeout_ms) {
            if (last_slot && cur >= 0) *last_slot = cur;
            return NSTAR_ERR_TIMEOUT;
        }

        struct timespec poll_delay = { 0, 20 * 1000000L };  /* 20 ms */
        nanosleep(&poll_delay, NULL);
    }
}

/* =========================================================================
 * Data interface — file-based TX/RX (open point stand-in)
 * =========================================================================
 *
 * TX: every call appends to SIL_DATA_TX_FILE. The SIL test suite reads
 * this file after a TX session to verify exactly what nstar_tx_write()
 * sent, the same way nstar_mock_data_get_written() works for unit tests
 * but observable from a separate process.
 *
 * RX: every call reads the next unread bytes from SIL_DATA_RX_FILE,
 * tracking its own read offset for the lifetime of the process (mirrors
 * a real streaming data interface — bytes are consumed once, not re-read).
 * The SIL test suite populates this file before triggering an RX lock
 * sequence via the GPIO files.
 */

ssize_t nstar_hal_data_write(int fd, const uint8_t *buf, size_t len)
{
    (void)fd;
    char path[512];
    sil_path(path, sizeof(path), SIL_DATA_TX_FILE);

    FILE *f = fopen(path, "ab");
    if (!f) return -1;
    size_t n = fwrite(buf, 1, len, f);
    fclose(f);
    return (ssize_t)n;
}

static long g_data_rx_offset = 0;

ssize_t nstar_hal_data_read(int fd, uint8_t *buf, size_t len)
{
    (void)fd;
    char path[512];
    sil_path(path, sizeof(path), SIL_DATA_RX_FILE);

    FILE *f = fopen(path, "rb");
    if (!f) return 0;   /* file doesn't exist yet — treat as no data */

    if (fseek(f, g_data_rx_offset, SEEK_SET) != 0) {
        fclose(f);
        return 0;
    }
    size_t n = fread(buf, 1, len, f);
    fclose(f);

    g_data_rx_offset += (long)n;
    return (ssize_t)n;
}

nstar_result_t nstar_hal_data_clock_start(int fd)
{
    (void)fd;
    char path[512];
    sil_path(path, sizeof(path), SIL_DATA_CLOCK_FILE);
    FILE *f = fopen(path, "w");
    if (!f) return NSTAR_ERR_HAL;
    fprintf(f, "1");
    fclose(f);
    return NSTAR_OK;
}

nstar_result_t nstar_hal_data_clock_stop(int fd)
{
    (void)fd;
    char path[512];
    sil_path(path, sizeof(path), SIL_DATA_CLOCK_FILE);
    FILE *f = fopen(path, "w");
    if (!f) return NSTAR_ERR_HAL;
    fprintf(f, "0");
    fclose(f);
    return NSTAR_OK;
}

/* =========================================================================
 * Timing
 * =========================================================================
 */

void nstar_hal_sleep_ms(uint32_t ms)
{
    struct timespec ts;
    ts.tv_sec  = ms / 1000;
    ts.tv_nsec = (long)(ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

uint64_t nstar_hal_timestamp_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000U + (uint64_t)(ts.tv_nsec / 1000000);
}
