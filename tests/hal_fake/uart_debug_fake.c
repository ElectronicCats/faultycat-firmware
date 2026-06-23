#include "hal_fake_uart_debug.h"

#include <string.h>

hal_fake_uart_debug_state_t hal_fake_uart_debug_state;

void hal_fake_uart_debug_reset(void) {
    memset(&hal_fake_uart_debug_state, 0, sizeof(hal_fake_uart_debug_state));
}

void hal_uart_debug_init(uint32_t baudrate) {
    hal_fake_uart_debug_state.initialized = true;
    hal_fake_uart_debug_state.baudrate    = baudrate;
    hal_fake_uart_debug_state.init_calls++;
}

void hal_uart_debug_deinit(void) {
    hal_fake_uart_debug_state.initialized = false;
    hal_fake_uart_debug_state.deinit_calls++;
}

void hal_uart_debug_set_baud(uint32_t baudrate) {
    hal_fake_uart_debug_state.baudrate = baudrate;
    hal_fake_uart_debug_state.set_baud_calls++;
}

size_t hal_uart_debug_write(const uint8_t* data, size_t len) {
    size_t space = HAL_FAKE_UART_DEBUG_TX_CAP - hal_fake_uart_debug_state.tx_len;
    size_t n     = len > space ? space : len;
    memcpy(&hal_fake_uart_debug_state.tx_buf[hal_fake_uart_debug_state.tx_len], data, n);
    hal_fake_uart_debug_state.tx_len += n;
    return n;
}
