#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hal/uart_debug.h"

#define HAL_FAKE_UART_DEBUG_TX_CAP 256

typedef struct {
    bool initialized;
    uint32_t baudrate;
    uint32_t init_calls;
    uint32_t deinit_calls;
    uint32_t set_baud_calls;

    uint8_t tx_buf[HAL_FAKE_UART_DEBUG_TX_CAP]; // bytes mirrored out
    size_t tx_len;
} hal_fake_uart_debug_state_t;

extern hal_fake_uart_debug_state_t hal_fake_uart_debug_state;

void hal_fake_uart_debug_reset(void);
