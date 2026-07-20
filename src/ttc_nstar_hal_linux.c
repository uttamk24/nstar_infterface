/**
 * @file  ttc_nstar_hal_linux.c
 * @brief Real HAL implementation — Stage 6 (hardware available).
 *        All functions are stubs returning errors until Stage 6.
 */
#include "ttc_nstar.h"

ssize_t nstarUARTWrite(int fd, const uint8_t *buf, size_t len)
{
    (void)fd; (void)buf; (void)len;
    return -1;  /* Stage 6 */
}

ssize_t nstarUARTRead(int fd, uint8_t *buf, size_t len,
                             uint32_t timeoutMs)
{
    (void)fd; (void)buf; (void)len; (void)timeoutMs;
    return -1;  /* Stage 6 */
}

NSTAR_Result_t nstarGPIOWaitEdge(int fd, NSTAR_GPIOEdge_t edge,
                                         uint32_t timeoutMs)
{
    (void)fd; (void)edge; (void)timeoutMs;
    return NSTAR_ERR_HAL;  /* Stage 6 */
}

int nstarGPIORead(int fd)
{
    (void)fd;
    return -1;  /* Stage 6 */
}

NSTAR_Result_t nstarGPIOWrite(int fd, int value)
{
    (void)fd; (void)value;
    return NSTAR_ERR_HAL;  /* Stage 6 */
}

ssize_t nstarDataWrite(int fd, const uint8_t *buf, size_t len)
{
    (void)fd; (void)buf; (void)len;
    return -1;  /* Stage 6 — open point */
}

ssize_t nstarDataRead(int fd, uint8_t *buf, size_t len)
{
    (void)fd; (void)buf; (void)len;
    return -1;  /* Stage 6 — open point */
}

void nstarSleepMS(uint32_t ms)
{
    (void)ms;  /* Stage 6 */
}

uint64_t nstarTimestampMS(void)
{
    return 0;  /* Stage 6 */
}

NSTAR_Result_t nstarDataClockStart(int fd)
{
    (void)fd;
    return NSTAR_ERR_HAL;  /* Stage 6 — open point */
}

NSTAR_Result_t nstarDataClockStop(int fd)
{
    (void)fd;
    return NSTAR_ERR_HAL;  /* Stage 6 — open point */
}
