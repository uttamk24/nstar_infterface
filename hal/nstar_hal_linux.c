/**
 * @file  nstar_hal_linux.c
 * @brief Real HAL implementation — Stage 6 (hardware available).
 *        All functions are stubs returning errors until Stage 6.
 */
#include "nstar.h"

ssize_t nstarHALUARTWrite(int fd, const uint8_t *buf, size_t len)
{
    (void)fd; (void)buf; (void)len;
    return -1;  /* Stage 6 */
}

ssize_t nstarHALUARTRead(int fd, uint8_t *buf, size_t len,
                             uint32_t timeoutMs)
{
    (void)fd; (void)buf; (void)len; (void)timeoutMs;
    return -1;  /* Stage 6 */
}

nstarResult_t nstarHALGPIOWaitEdge(int fd, nstarGPIOEdge_t edge,
                                         uint32_t timeoutMs)
{
    (void)fd; (void)edge; (void)timeoutMs;
    return NSTAR_ERR_HAL;  /* Stage 6 */
}

int nstarHALGPIORead(int fd)
{
    (void)fd;
    return -1;  /* Stage 6 */
}

nstarResult_t nstarHALGPIOWrite(int fd, int value)
{
    (void)fd; (void)value;
    return NSTAR_ERR_HAL;  /* Stage 6 */
}

ssize_t nstarHALDataWrite(int fd, const uint8_t *buf, size_t len)
{
    (void)fd; (void)buf; (void)len;
    return -1;  /* Stage 6 — open point */
}

ssize_t nstarHALDataRead(int fd, uint8_t *buf, size_t len)
{
    (void)fd; (void)buf; (void)len;
    return -1;  /* Stage 6 — open point */
}

void nstarHALSleepMS(uint32_t ms)
{
    (void)ms;  /* Stage 6 */
}

uint64_t nstarHALTimestampMS(void)
{
    return 0;  /* Stage 6 */
}
