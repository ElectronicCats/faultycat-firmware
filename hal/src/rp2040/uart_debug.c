#include "hal/uart_debug.h"

#include "hardware/gpio.h"
#include "hardware/uart.h"

#define UART_DEBUG_TX_PIN 4u // scanner CH4 — UART1's native TX, shared via swd_bus_lock

void hal_uart_debug_init(uint32_t baudrate) {
    gpio_set_function(UART_DEBUG_TX_PIN, GPIO_FUNC_UART);
    uart_init(uart1, baudrate);
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart1, true);
}

void hal_uart_debug_deinit(void) {
    uart_deinit(uart1);
}

void hal_uart_debug_set_baud(uint32_t baudrate) {
    uart_set_baudrate(uart1, baudrate);
}

size_t hal_uart_debug_write(const uint8_t* data, size_t len) {
    size_t n = 0;
    while (n < len && uart_is_writable(uart1)) {
        uart_get_hw(uart1)->dr = data[n];
        n++;
    }
    return n;
}
