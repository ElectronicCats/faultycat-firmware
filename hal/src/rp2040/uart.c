#include "hal/uart.h"

#include "hardware/gpio.h"
#include "hardware/uart.h"

#define UART_PASSTHRU_TX_PIN 0u // scanner CH0 — UART0's native TX
#define UART_PASSTHRU_RX_PIN 1u // scanner CH1 — UART0's native RX

// --- RX diagnostics (temporary — remove once the RX path is proven) --------
// The PL011 packs error flags into DR[11:8] on every read; the normal byte
// path throws them away. Accumulate them plus a raw read counter.
static volatile uint32_t s_dbg_rd; // bytes pulled from the RX FIFO
static volatile uint32_t s_dbg_fe; // framing errors
static volatile uint32_t s_dbg_pe; // parity errors
static volatile uint32_t s_dbg_be; // break errors
static volatile uint32_t s_dbg_oe; // overrun errors

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
        uint32_t dr = uart_get_hw(uart0)->dr; // full 12-bit: data + error flags
        if (dr & UART_UARTDR_FE_BITS)
            s_dbg_fe++;
        if (dr & UART_UARTDR_PE_BITS)
            s_dbg_pe++;
        if (dr & UART_UARTDR_BE_BITS)
            s_dbg_be++;
        if (dr & UART_UARTDR_OE_BITS)
            s_dbg_oe++;
        data[n] = (uint8_t)(dr & UART_UARTDR_DATA_BITS);
        s_dbg_rd++;
        n++;
    }
    return n;
}

bool hal_uart_rx_available(void) {
    return uart_is_readable(uart0);
}

void hal_uart_get_rx_diag(hal_uart_rx_diag_t* out) {
    if (!out)
        return;
    out->func_tx     = (uint32_t)gpio_get_function(UART_PASSTHRU_TX_PIN);
    out->func_rx     = (uint32_t)gpio_get_function(UART_PASSTHRU_RX_PIN);
    out->rx_level    = gpio_get(UART_PASSTHRU_RX_PIN) ? 1u : 0u;
    out->tx_level    = gpio_get(UART_PASSTHRU_TX_PIN) ? 1u : 0u;
    out->cr          = uart_get_hw(uart0)->cr;
    out->lcr_h       = uart_get_hw(uart0)->lcr_h;
    out->fr          = uart_get_hw(uart0)->fr;
    out->bytes_read  = s_dbg_rd;
    out->err_framing = s_dbg_fe;
    out->err_parity  = s_dbg_pe;
    out->err_break   = s_dbg_be;
    out->err_overrun = s_dbg_oe;
}
