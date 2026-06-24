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
void nstarMockReset(void);

/** Queue a scripted UART response. Responses are consumed in order. */
void nstarMockUARTQueueResponse(const uint8_t *buf, size_t len);

/** Force next UART read to return timeout (0 bytes). */
void nstarMockUARTForceTimeout(int enable);

/** Return a pointer to all bytes written via nstarHALUARTWrite(). */
const uint8_t *nstarMockUARTGetWritten(size_t *lenOut);

/** Queue a GPIO edge event on fd. Consumed in order per (fd, edge) pair. */
void nstarMockGPIOQueueEdge(int fd, nstarGPIOEdge_t edge,
                                 uint32_t delayMS);

/** Directly set a GPIO value (for nstarHALGPIORead). */
void nstarMockGPIOSetValue(int fd, int value);

/** Supply bytes to be returned by nstarHALDataRead(). */
void nstarMockDataSupplyRead(const uint8_t *buf, size_t len);

/** Return a pointer to all bytes written via nstarHALDataWrite(). */
const uint8_t *nstarMockDataGetWritten(size_t *lenOut);

/** Return 1 if clock is currently running (started and not stopped). */
int nstarMockDataClockIsRunning(void);

#ifdef __cplusplus
}
#endif

#endif /* NSTAR_HAL_MOCK_H */
