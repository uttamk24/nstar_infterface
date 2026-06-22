/**
 * @file  nstar_hal_linux.c
 * @brief Real HAL implementation — Stage 6 (hardware available).
 *        All functions are stubs returning errors until Stage 6.
 */
#include "nstar.h"

ssize_t nstar_hal_uart_write(int fd, const uint8_t *buf, size_t len)
{
    (void)fd; (void)buf; (void)len;
    return -1;  /* Stage 6 */
}

ssize_t nstar_hal_uart_read(int fd, uint8_t *buf, size_t len,
                             uint32_t timeout_ms)
{
    (void)fd; (void)buf; (void)len; (void)timeout_ms;
    return -1;  /* Stage 6 */
}

nstar_result_t nstar_hal_gpio_wait_edge(int fd, nstar_gpio_edge_t edge,
                                         uint32_t timeout_ms)
{
    (void)fd; (void)edge; (void)timeout_ms;
    return NSTAR_ERR_HAL;  /* Stage 6 */
}

int nstar_hal_gpio_read(int fd)
{
    (void)fd;
    return -1;  /* Stage 6 */
}

nstar_result_t nstar_hal_gpio_write(int fd, int value)
{
    (void)fd; (void)value;
    return NSTAR_ERR_HAL;  /* Stage 6 */
}

ssize_t nstar_hal_data_write(int fd, const uint8_t *buf, size_t len)
{
    (void)fd; (void)buf; (void)len;
    return -1;  /* Stage 6 — open point */
}

ssize_t nstar_hal_data_read(int fd, uint8_t *buf, size_t len)
{
    (void)fd; (void)buf; (void)len;
    return -1;  /* Stage 6 — open point */
}

void nstar_hal_sleep_ms(uint32_t ms)
{
    (void)ms;  /* Stage 6 */
}

uint64_t nstar_hal_timestamp_ms(void)
{
    return 0;  /* Stage 6 */
}
