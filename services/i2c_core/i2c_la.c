/*
 * services/i2c_core/i2c_la.c — passive I2C logic analyzer (CPU sampler).
 *
 * See i2c_la.h for the design rationale (no PIO, no async timer, blocking
 * by design). Structurally mirrors i2c_core.c: module-static singleton
 * state, everything routes through hal/gpio + hal/time so it links
 * against tests/hal_fake/gpio_fake.c on the host.
 */

#include "i2c_la.h"

#include "hal/gpio.h"
#include "hal/time.h"

static uint8_t s_sda;
static uint8_t s_scl;
static bool s_inited = false;

static uint8_t s_buffer[I2C_LA_CAPTURE_BUFFER_BYTES];
static uint32_t s_count = 0u;

bool i2c_la_init(uint8_t sda, uint8_t scl) {
    if (s_inited)
        return false;
    if (sda >= 30u || scl >= 30u || sda == scl)
        return false;

    s_sda = sda;
    s_scl = scl;

    hal_gpio_init(s_sda, HAL_GPIO_DIR_IN);
    hal_gpio_init(s_scl, HAL_GPIO_DIR_IN);

    s_count  = 0u;
    s_inited = true;
    return true;
}

void i2c_la_deinit(void) {
    if (!s_inited)
        return;
    hal_gpio_init(s_sda, HAL_GPIO_DIR_IN);
    hal_gpio_init(s_scl, HAL_GPIO_DIR_IN);
    hal_gpio_set_pulls(s_sda, false, false);
    hal_gpio_set_pulls(s_scl, false, false);
    s_inited = false;
}

bool i2c_la_is_inited(void) {
    return s_inited;
}

uint32_t i2c_la_capture(uint32_t sample_interval_us, uint32_t max_samples, uint32_t max_ms) {
    s_count = 0u;
    if (!s_inited || max_samples == 0u || max_ms == 0u)
        return 0u;

    if (max_samples > I2C_LA_CAPTURE_BUFFER_BYTES)
        max_samples = I2C_LA_CAPTURE_BUFFER_BYTES;
    if (sample_interval_us == 0u)
        sample_interval_us = 1u;

    uint32_t start = hal_now_ms();
    while (s_count < max_samples) {
        if ((uint32_t)(hal_now_ms() - start) >= max_ms)
            break;

        uint8_t sample = 0u;
        if (hal_gpio_get(s_sda))
            sample |= I2C_LA_SAMPLE_SDA_BIT;
        if (hal_gpio_get(s_scl))
            sample |= I2C_LA_SAMPLE_SCL_BIT;
        s_buffer[s_count++] = sample;

        hal_busy_wait_us(sample_interval_us);
    }
    return s_count;
}

const uint8_t* i2c_la_buffer(void) {
    return s_buffer;
}

uint32_t i2c_la_count(void) {
    return s_count;
}
