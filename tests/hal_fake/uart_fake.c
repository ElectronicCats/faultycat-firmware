#include "hal_fake_uart.h"

#include <string.h>

hal_fake_uart_state_t hal_fake_uart_state;

void hal_fake_uart_reset(void) {
    memset(&hal_fake_uart_state, 0, sizeof(hal_fake_uart_state));
}

void hal_fake_uart_inject_rx(const uint8_t* data, size_t len) {
    if (len > HAL_FAKE_UART_RX_CAP - hal_fake_uart_state.rx_len)
        len = HAL_FAKE_UART_RX_CAP - hal_fake_uart_state.rx_len;
    memcpy(&hal_fake_uart_state.rx_buf[hal_fake_uart_state.rx_len], data, len);
    hal_fake_uart_state.rx_len += len;
}

void hal_uart_init(hal_uart_config_t cfg) {
    hal_fake_uart_state.initialized   = true;
    hal_fake_uart_state.last_config   = cfg;
    hal_fake_uart_state.init_calls++;
}

void hal_uart_deinit(void) {
    hal_fake_uart_state.initialized = false;
    hal_fake_uart_state.deinit_calls++;
}

void hal_uart_set_config(hal_uart_config_t cfg) {
    hal_fake_uart_state.last_config = cfg;
    hal_fake_uart_state.set_config_calls++;
}

size_t hal_uart_write(const uint8_t* data, size_t len) {
    size_t space = HAL_FAKE_UART_TX_CAP - hal_fake_uart_state.tx_len;
    size_t n     = len > space ? space : len;
    memcpy(&hal_fake_uart_state.tx_buf[hal_fake_uart_state.tx_len], data, n);
    hal_fake_uart_state.tx_len += n;
    return n;
}

size_t hal_uart_read(uint8_t* data, size_t cap) {
    size_t avail = hal_fake_uart_state.rx_len - hal_fake_uart_state.rx_pos;
    size_t n     = cap > avail ? avail : cap;
    memcpy(data, &hal_fake_uart_state.rx_buf[hal_fake_uart_state.rx_pos], n);
    hal_fake_uart_state.rx_pos += n;
    return n;
}

bool hal_uart_rx_available(void) {
    return hal_fake_uart_state.rx_pos < hal_fake_uart_state.rx_len;
}
