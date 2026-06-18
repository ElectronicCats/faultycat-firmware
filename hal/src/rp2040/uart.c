#include "hal/uart.h"

#include "hardware/gpio.h"
#include "hardware/uart.h"

#define UART_PASSTHRU_TX_PIN 0u // scanner CH0 — UART0's native TX
#define UART_PASSTHRU_RX_PIN 1u // scanner CH1 — UART0's native RX

static inline uart_parity_t as_pico_parity(hal_uart_parity_t p) {
    switch (p) {
        case HAL_UART_PARITY_EVEN:
            return UART_PARITY_EVEN;
        case HAL_UART_PARITY_ODD:
            return UART_PARITY_ODD;
        default:
            return UART_PARITY_NONE;
    }
}

void hal_uart_init(hal_uart_config_t cfg) {
    gpio_set_function(UART_PASSTHRU_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_PASSTHRU_RX_PIN, GPIO_FUNC_UART);
    uart_init(uart0, cfg.baudrate);
    uart_set_format(uart0, cfg.data_bits, cfg.stop_bits, as_pico_parity(cfg.parity));
    uart_set_fifo_enabled(uart0, true);
}

void hal_uart_deinit(void) {
    uart_deinit(uart0);
}

void hal_uart_set_config(hal_uart_config_t cfg) {
    uart_set_baudrate(uart0, cfg.baudrate);
    uart_set_format(uart0, cfg.data_bits, cfg.stop_bits, as_pico_parity(cfg.parity));
}

size_t hal_uart_write(const uint8_t* data, size_t len) {
    size_t n = 0;
    while (n < len && uart_is_writable(uart0)) {
        uart_get_hw(uart0)->dr = data[n];
        n++;
    }
    return n;
}

size_t hal_uart_read(uint8_t* data, size_t cap) {
    size_t n = 0;
    while (n < cap && uart_is_readable(uart0)) {
        data[n] = (uint8_t)uart_get_hw(uart0)->dr;
        n++;
    }
    return n;
}

bool hal_uart_rx_available(void) {
    return uart_is_readable(uart0);
}
