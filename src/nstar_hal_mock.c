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
} gUARTResponses[MOCK_UART_RESP_MAX];

static int  gUARTRespCount  = 0;
static int  gUARTRespIdx    = 0;
static int  gUARTForceTimeout = 0;    /* if 1, next read returns timeout */

/* UART write capture */
#define MOCK_UART_WRITE_BUF_MAX  (NSTAR_FRAME_BUF_MAX * 8)
static uint8_t gUARTWritten[MOCK_UART_WRITE_BUF_MAX];
static size_t  gUARTWrittenLen = 0;

/* GPIO: scripted edge sequence */
#define MOCK_GPIO_EVENTS_MAX    64
static struct {
    int fd;
    nstarGPIOEdge_t edge;
    uint32_t delayMS;   /* simulated delay before edge fires */
} gGPIOEvents[MOCK_GPIO_EVENTS_MAX];
static int  gGPIOEventCount = 0;
static int  gGPIOEventIdx   = 0;
static int  gGPIOValues[32]  = {0};   /* indexed by fd */

/* Data interface write capture */
#define MOCK_DATA_WRITE_MAX  (NSTAR_FRAME_SIZE_BYTES * 8)
static uint8_t  gDataWritten[MOCK_DATA_WRITE_MAX];
static size_t   gDataWrittenLen = 0;

/* Data interface read supply */
static uint8_t  gDataReadBuf[MOCK_DATA_WRITE_MAX];
static size_t   gDataReadLen   = 0;
static size_t   gDataReadPos   = 0;

/* Clock state */
static int      gClockRunning   = 0;

/* =========================================================================
 * Mock control API (used by test code)
 * =========================================================================
 */

void nstarMockReset(void)
{
    memset(gUARTResponses, 0, sizeof(gUARTResponses));
    gUARTRespCount       = 0;
    gUARTRespIdx         = 0;
    gUARTForceTimeout    = 0;
    gUARTWrittenLen      = 0;

    memset(gGPIOEvents, 0, sizeof(gGPIOEvents));
    gGPIOEventCount      = 0;
    gGPIOEventIdx        = 0;
    memset(gGPIOValues, 0, sizeof(gGPIOValues));

    gDataWrittenLen      = 0;
    gDataReadLen         = 0;
    gDataReadPos         = 0;
    gClockRunning         = 0;
}

void nstarMockUARTQueueResponse(const uint8_t *buf, size_t len)
{
    if (gUARTRespCount >= MOCK_UART_RESP_MAX) return;
    if (len > MOCK_UART_RESP_BUF_MAX) return;
    memcpy(gUARTResponses[gUARTRespCount].buf, buf, len);
    gUARTResponses[gUARTRespCount].len = len;
    gUARTRespCount++;
}

void nstarMockUARTForceTimeout(int enable)
{
    gUARTForceTimeout = enable;
}

const uint8_t *nstarMockUARTGetWritten(size_t *lenOut)
{
    *lenOut = gUARTWrittenLen;
    return gUARTWritten;
}

void nstarMockGPIOQueueEdge(int fd, nstarGPIOEdge_t edge,
                                 uint32_t delayMS)
{
    if (gGPIOEventCount >= MOCK_GPIO_EVENTS_MAX) return;
    gGPIOEvents[gGPIOEventCount].fd       = fd;
    gGPIOEvents[gGPIOEventCount].edge     = edge;
    gGPIOEvents[gGPIOEventCount].delayMS = delayMS;
    gGPIOEventCount++;
    /* Update simulated GPIO value */
    if (fd < 32) {
        gGPIOValues[fd] = (edge == NSTAR_GPIO_EDGE_RISING) ? 1 : 0;
    }
}

void nstarMockGPIOSetValue(int fd, int value)
{
    if (fd < 32) gGPIOValues[fd] = value;
}

void nstarMockDataSupplyRead(const uint8_t *buf, size_t len)
{
    if (len > MOCK_DATA_WRITE_MAX) return;
    memcpy(gDataReadBuf, buf, len);
    gDataReadLen = len;
    gDataReadPos = 0;
}

const uint8_t *nstarMockDataGetWritten(size_t *lenOut)
{
    *lenOut = gDataWrittenLen;
    return gDataWritten;
}

int nstarMockDataClockIsRunning(void)
{
    return gClockRunning;
}

/* =========================================================================
 * HAL implementations
 * =========================================================================
 */

ssize_t nstarHALUARTWrite(int fd, const uint8_t *buf, size_t len)
{
    (void)fd;
    if (gUARTWrittenLen + len <= MOCK_UART_WRITE_BUF_MAX) {
        memcpy(gUARTWritten + gUARTWrittenLen, buf, len);
        gUARTWrittenLen += len;
    }
    return (ssize_t)len;
}

ssize_t nstarHALUARTRead(int fd, uint8_t *buf, size_t len,
                             uint32_t timeoutMs)
{
    (void)fd;
    (void)timeoutMs;

    if (gUARTForceTimeout) return 0;

    if (gUARTRespIdx >= gUARTRespCount) return 0;   /* no more responses */

    size_t avail = gUARTResponses[gUARTRespIdx].len;
    size_t copy  = (avail < len) ? avail : len;
    memcpy(buf, gUARTResponses[gUARTRespIdx].buf, copy);
    gUARTRespIdx++;
    return (ssize_t)copy;
}

nstarResult_t nstarHALGPIOWaitEdge(int fd, nstarGPIOEdge_t edge,
                                         uint32_t timeoutMs)
{
    (void)timeoutMs;

    if (gGPIOEventIdx >= gGPIOEventCount) {
        return NSTAR_ERR_TIMEOUT;  /* no more scripted events */
    }

    /* Find next event matching this fd and edge */
    for (int i = gGPIOEventIdx; i < gGPIOEventCount; i++) {
        if (gGPIOEvents[i].fd == fd &&
            gGPIOEvents[i].edge == edge) {
            gGPIOEventIdx = i + 1;
            if (fd < 32) {
                gGPIOValues[fd] = (edge == NSTAR_GPIO_EDGE_RISING) ? 1 : 0;
            }
            return NSTAR_OK;
        }
    }
    return NSTAR_ERR_TIMEOUT;
}

int nstarHALGPIORead(int fd)
{
    if (fd < 0 || fd >= 32) return -1;
    return gGPIOValues[fd];
}

nstarResult_t nstarHALGPIOWrite(int fd, int value)
{
    if (fd < 0 || fd >= 32) return NSTAR_ERR_HAL;
    gGPIOValues[fd] = value ? 1 : 0;
    return NSTAR_OK;
}

ssize_t nstarHALDataWrite(int fd, const uint8_t *buf, size_t len)
{
    (void)fd;
    if (gDataWrittenLen + len <= MOCK_DATA_WRITE_MAX) {
        memcpy(gDataWritten + gDataWrittenLen, buf, len);
        gDataWrittenLen += len;
    }
    return (ssize_t)len;
}

ssize_t nstarHALDataRead(int fd, uint8_t *buf, size_t len)
{
    (void)fd;
    size_t avail = gDataReadLen - gDataReadPos;
    if (avail == 0) return 0;
    size_t copy = (avail < len) ? avail : len;
    memcpy(buf, gDataReadBuf + gDataReadPos, copy);
    gDataReadPos += copy;
    return (ssize_t)copy;
}

nstarResult_t nstarHALDataClockStart(int fd)
{
    (void)fd;
    gClockRunning = 1;
    return NSTAR_OK;
}

nstarResult_t nstarHALDataClockStop(int fd)
{
    (void)fd;
    gClockRunning = 0;
    return NSTAR_OK;
}

void nstarHALSleepMS(uint32_t ms)
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
     * (e.g. nstarTXStart()) for the same mocked response queue, even
     * though nstarRegReadMulti() is not atomic across the uartMutex
     * (it acquires/releases per register, same as production).
     *
     * Scaling by /100 keeps the 30000 ms health interval at 300 ms real
     * time — long enough that any single synchronous test call (which
     * completes in well under 300 ms) finishes first — while a 100 ms
     * production sleep (e.g. clock pre-stable) becomes 1 ms, keeping
     * the suite fast.
     */
    uint32_t realMS = ms / 100;
    if (realMS == 0) realMS = 1;

    struct timespec ts = {
        .tv_sec  = realMS / 1000,
        .tv_nsec = (long)(realMS % 1000) * 1000000L
    };
    nanosleep(&ts, NULL);
}

uint64_t nstarHALTimestampMS(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000U + (uint64_t)(ts.tv_nsec / 1000000);
}
