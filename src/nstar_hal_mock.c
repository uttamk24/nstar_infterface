/**
 * @file  nstar_hal_mock.c
 * @brief Mock HAL implementation for unit testing (Stages 1-5).
 *
 * Replaces hal/nstar_hal_linux.c at link time during test builds.
 * The mock records calls and returns scripted responses.
 *
 * Usage: tests configure the mock via nstar_mock_* functions before
 * calling the module under test.
 */

#include "nstar.h"
#include "nstar_hal_mock.h"
#include <string.h>
#include <time.h>
#include <unistd.h>

/* =========================================================================
 * Mock state
 * =========================================================================
 */

/* UART: scripted response queue */
#define MOCK_UART_RESP_MAX      64
#define MOCK_UART_RESP_BUF_MAX  NSTAR_FRAME_BUF_MAX

static struct {
    uint8_t  buf[MOCK_UART_RESP_BUF_MAX];
    size_t   len;
} g_uart_responses[MOCK_UART_RESP_MAX];

static int  g_uart_resp_count  = 0;
static int  g_uart_resp_idx    = 0;
static int  g_uart_force_timeout = 0;    /* if 1, next read returns timeout */

/* UART write capture */
#define MOCK_UART_WRITE_BUF_MAX  (NSTAR_FRAME_BUF_MAX * 8)
static uint8_t g_uart_written[MOCK_UART_WRITE_BUF_MAX];
static size_t  g_uart_written_len = 0;

/* GPIO: scripted edge sequence */
#define MOCK_GPIO_EVENTS_MAX    64
static struct {
    int fd;
    nstar_gpio_edge_t edge;
    uint32_t delay_ms;   /* simulated delay before edge fires */
} g_gpio_events[MOCK_GPIO_EVENTS_MAX];
static int  g_gpio_event_count = 0;
static int  g_gpio_event_idx   = 0;
static int  g_gpio_values[32]  = {0};   /* indexed by fd */

/* Data interface write capture */
#define MOCK_DATA_WRITE_MAX  (NSTAR_FRAME_SIZE_BYTES * 8)
static uint8_t  g_data_written[MOCK_DATA_WRITE_MAX];
static size_t   g_data_written_len = 0;

/* Data interface read supply */
static uint8_t  g_data_read_buf[MOCK_DATA_WRITE_MAX];
static size_t   g_data_read_len   = 0;
static size_t   g_data_read_pos   = 0;

/* Clock state */
static int      g_clock_running   = 0;

/* =========================================================================
 * Mock control API (used by test code)
 * =========================================================================
 */

void nstar_mock_reset(void)
{
    memset(g_uart_responses, 0, sizeof(g_uart_responses));
    g_uart_resp_count       = 0;
    g_uart_resp_idx         = 0;
    g_uart_force_timeout    = 0;
    g_uart_written_len      = 0;

    memset(g_gpio_events, 0, sizeof(g_gpio_events));
    g_gpio_event_count      = 0;
    g_gpio_event_idx        = 0;
    memset(g_gpio_values, 0, sizeof(g_gpio_values));

    g_data_written_len      = 0;
    g_data_read_len         = 0;
    g_data_read_pos         = 0;
    g_clock_running         = 0;
}

void nstar_mock_uart_queue_response(const uint8_t *buf, size_t len)
{
    if (g_uart_resp_count >= MOCK_UART_RESP_MAX) return;
    if (len > MOCK_UART_RESP_BUF_MAX) return;
    memcpy(g_uart_responses[g_uart_resp_count].buf, buf, len);
    g_uart_responses[g_uart_resp_count].len = len;
    g_uart_resp_count++;
}

void nstar_mock_uart_force_timeout(int enable)
{
    g_uart_force_timeout = enable;
}

const uint8_t *nstar_mock_uart_get_written(size_t *len_out)
{
    *len_out = g_uart_written_len;
    return g_uart_written;
}

void nstar_mock_gpio_queue_edge(int fd, nstar_gpio_edge_t edge,
                                 uint32_t delay_ms)
{
    if (g_gpio_event_count >= MOCK_GPIO_EVENTS_MAX) return;
    g_gpio_events[g_gpio_event_count].fd       = fd;
    g_gpio_events[g_gpio_event_count].edge     = edge;
    g_gpio_events[g_gpio_event_count].delay_ms = delay_ms;
    g_gpio_event_count++;
    /* Update simulated GPIO value */
    if (fd < 32) {
        g_gpio_values[fd] = (edge == NSTAR_GPIO_EDGE_RISING) ? 1 : 0;
    }
}

void nstar_mock_gpio_set_value(int fd, int value)
{
    if (fd < 32) g_gpio_values[fd] = value;
}

void nstar_mock_data_supply_read(const uint8_t *buf, size_t len)
{
    if (len > MOCK_DATA_WRITE_MAX) return;
    memcpy(g_data_read_buf, buf, len);
    g_data_read_len = len;
    g_data_read_pos = 0;
}

const uint8_t *nstar_mock_data_get_written(size_t *len_out)
{
    *len_out = g_data_written_len;
    return g_data_written;
}

int nstar_mock_data_clock_is_running(void)
{
    return g_clock_running;
}

/* =========================================================================
 * HAL implementations
 * =========================================================================
 */

ssize_t nstar_hal_uart_write(int fd, const uint8_t *buf, size_t len)
{
    (void)fd;
    if (g_uart_written_len + len <= MOCK_UART_WRITE_BUF_MAX) {
        memcpy(g_uart_written + g_uart_written_len, buf, len);
        g_uart_written_len += len;
    }
    return (ssize_t)len;
}

ssize_t nstar_hal_uart_read(int fd, uint8_t *buf, size_t len,
                             uint32_t timeout_ms)
{
    (void)fd;
    (void)timeout_ms;

    if (g_uart_force_timeout) return 0;

    if (g_uart_resp_idx >= g_uart_resp_count) return 0;   /* no more responses */

    size_t avail = g_uart_responses[g_uart_resp_idx].len;
    size_t copy  = (avail < len) ? avail : len;
    memcpy(buf, g_uart_responses[g_uart_resp_idx].buf, copy);
    g_uart_resp_idx++;
    return (ssize_t)copy;
}

nstar_result_t nstar_hal_gpio_wait_edge(int fd, nstar_gpio_edge_t edge,
                                         uint32_t timeout_ms)
{
    (void)timeout_ms;

    if (g_gpio_event_idx >= g_gpio_event_count) {
        return NSTAR_ERR_TIMEOUT;  /* no more scripted events */
    }

    /* Find next event matching this fd and edge */
    for (int i = g_gpio_event_idx; i < g_gpio_event_count; i++) {
        if (g_gpio_events[i].fd == fd &&
            g_gpio_events[i].edge == edge) {
            g_gpio_event_idx = i + 1;
            if (fd < 32) {
                g_gpio_values[fd] = (edge == NSTAR_GPIO_EDGE_RISING) ? 1 : 0;
            }
            return NSTAR_OK;
        }
    }
    return NSTAR_ERR_TIMEOUT;
}

int nstar_hal_gpio_read(int fd)
{
    if (fd < 0 || fd >= 32) return -1;
    return g_gpio_values[fd];
}

nstar_result_t nstar_hal_gpio_write(int fd, int value)
{
    if (fd < 0 || fd >= 32) return NSTAR_ERR_HAL;
    g_gpio_values[fd] = value ? 1 : 0;
    return NSTAR_OK;
}

ssize_t nstar_hal_data_write(int fd, const uint8_t *buf, size_t len)
{
    (void)fd;
    if (g_data_written_len + len <= MOCK_DATA_WRITE_MAX) {
        memcpy(g_data_written + g_data_written_len, buf, len);
        g_data_written_len += len;
    }
    return (ssize_t)len;
}

ssize_t nstar_hal_data_read(int fd, uint8_t *buf, size_t len)
{
    (void)fd;
    size_t avail = g_data_read_len - g_data_read_pos;
    if (avail == 0) return 0;
    size_t copy = (avail < len) ? avail : len;
    memcpy(buf, g_data_read_buf + g_data_read_pos, copy);
    g_data_read_pos += copy;
    return (ssize_t)copy;
}

nstar_result_t nstar_hal_data_clock_start(int fd)
{
    (void)fd;
    g_clock_running = 1;
    return NSTAR_OK;
}

nstar_result_t nstar_hal_data_clock_stop(int fd)
{
    (void)fd;
    g_clock_running = 0;
    return NSTAR_OK;
}

void nstar_hal_sleep_ms(uint32_t ms)
{
    /*
     * Tests sleep a scaled-down real duration so the suite stays fast,
     * while preserving relative ordering between short sleeps (power-up
     * wait, clock pre-stable, retry backoff — tens to low thousands of ms
     * in production) and the health thread's 30000 ms poll interval.
     *
     * Without this scaling, a fixed-duration sleep made every production
     * delay resolve at the same real time, letting the health thread's
     * first poll race a test's own synchronous multi-step UART sequence
     * (e.g. nstar_tx_start()) for the same mocked response queue, even
     * though nstar_reg_read_multi() is not atomic across the uart_mutex
     * (it acquires/releases per register, same as production).
     *
     * Scaling by /100 keeps the 30000 ms health interval at 300 ms real
     * time — long enough that any single synchronous test call (which
     * completes in well under 300 ms) finishes first — while a 100 ms
     * production sleep (e.g. clock pre-stable) becomes 1 ms, keeping
     * the suite fast.
     */
    uint32_t real_ms = ms / 100;
    if (real_ms == 0) real_ms = 1;

    struct timespec ts = {
        .tv_sec  = real_ms / 1000,
        .tv_nsec = (long)(real_ms % 1000) * 1000000L
    };
    nanosleep(&ts, NULL);
}

uint64_t nstar_hal_timestamp_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000U + (uint64_t)(ts.tv_nsec / 1000000);
}
