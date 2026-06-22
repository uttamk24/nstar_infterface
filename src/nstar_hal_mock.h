/**
 * @file  nstar_hal_mock.h
 * @brief Mock HAL control API — used only in test code.
 */

#ifndef NSTAR_HAL_MOCK_H
#define NSTAR_HAL_MOCK_H

#include "nstar.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Reset all mock state. Call at the start of each test. */
void nstar_mock_reset(void);

/** Queue a scripted UART response. Responses are consumed in order. */
void nstar_mock_uart_queue_response(const uint8_t *buf, size_t len);

/** Force next UART read to return timeout (0 bytes). */
void nstar_mock_uart_force_timeout(int enable);

/** Return a pointer to all bytes written via nstar_hal_uart_write(). */
const uint8_t *nstar_mock_uart_get_written(size_t *len_out);

/** Queue a GPIO edge event on fd. Consumed in order per (fd, edge) pair. */
void nstar_mock_gpio_queue_edge(int fd, nstar_gpio_edge_t edge,
                                 uint32_t delay_ms);

/** Directly set a GPIO value (for nstar_hal_gpio_read). */
void nstar_mock_gpio_set_value(int fd, int value);

/** Supply bytes to be returned by nstar_hal_data_read(). */
void nstar_mock_data_supply_read(const uint8_t *buf, size_t len);

/** Return a pointer to all bytes written via nstar_hal_data_write(). */
const uint8_t *nstar_mock_data_get_written(size_t *len_out);

/** Return 1 if clock is currently running (started and not stopped). */
int nstar_mock_data_clock_is_running(void);

#ifdef __cplusplus
}
#endif

#endif /* NSTAR_HAL_MOCK_H */
