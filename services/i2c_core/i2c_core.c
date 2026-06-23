/*
 * services/i2c_core/i2c_core.c — I2C bit-bang master.
 *
 * See i2c_core.h for the protocol decisions (timing, clock stretching,
 * NACK handling) this implementation follows. Structurally mirrors
 * services/jtag_core/jtag_core.c: module-static singleton state, no
 * PIO/hardware peripheral, everything routes through hal/gpio + hal/time
 * so it links against tests/hal_fake/gpio_fake.c on the host.
 */

#include "i2c_core.h"

#include "hal/gpio.h"
#include "hal/time.h"

static uint8_t s_sda;
static uint8_t s_scl;
static uint32_t s_half_period_us;
static bool s_inited = false;

// -----------------------------------------------------------------------------
// Open-drain emulation primitives
// -----------------------------------------------------------------------------

static inline void line_release(uint8_t pin) {
    hal_gpio_init(pin, HAL_GPIO_DIR_IN);
    hal_gpio_set_pulls(pin, true, false);
}

static inline void line_drive_low(uint8_t pin) {
    hal_gpio_init(pin, HAL_GPIO_DIR_OUT);
    hal_gpio_put(pin, false);
}

static inline void half_delay(void) {
    hal_busy_wait_us(s_half_period_us);
}

// Release SCL and wait for it to actually read high, bounded by
// I2C_CLOCK_STRETCH_TIMEOUT_US. A slave holding SCL low (clock
// stretching) keeps us here until it lets go or we time out.
static void scl_release_and_wait(void) {
    line_release(s_scl);
    uint32_t waited = 0u;
    while (!hal_gpio_get(s_scl) && waited < I2C_CLOCK_STRETCH_TIMEOUT_US) {
        hal_busy_wait_us(1u);
        waited++;
    }
}

static inline void set_sda(bool high) {
    if (high)
        line_release(s_sda);
    else
        line_drive_low(s_sda);
}

// -----------------------------------------------------------------------------
// Bit-level primitives
// -----------------------------------------------------------------------------

static void write_bit(bool bit) {
    set_sda(bit);
    half_delay();
    scl_release_and_wait();
    half_delay();
    line_drive_low(s_scl);
}

static bool read_bit(void) {
    set_sda(true); // release so the slave can drive it
    half_delay();
    scl_release_and_wait();
    half_delay();
    bool bit = hal_gpio_get(s_sda);
    line_drive_low(s_scl);
    return bit;
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------

bool i2c_init(uint8_t sda, uint8_t scl, uint32_t freq_khz) {
    if (s_inited)
        return false;
    if (sda >= 30u || scl >= 30u || sda == scl)
        return false;

    if (freq_khz == 0u)
        freq_khz = 100u;
    // period_us = 1000 / freq_khz; half of that, minimum 1us so a
    // very high freq_khz doesn't collapse the delay to zero.
    uint32_t half = 500u / freq_khz;
    s_half_period_us = (half == 0u) ? 1u : half;

    s_sda = sda;
    s_scl = scl;

    line_release(s_sda);
    line_release(s_scl);

    s_inited = true;
    return true;
}

void i2c_deinit(void) {
    if (!s_inited)
        return;
    hal_gpio_init(s_sda, HAL_GPIO_DIR_IN);
    hal_gpio_init(s_scl, HAL_GPIO_DIR_IN);
    hal_gpio_set_pulls(s_sda, false, false);
    hal_gpio_set_pulls(s_scl, false, false);
    s_inited = false;
}

bool i2c_is_inited(void) {
    return s_inited;
}

void i2c_start(void) {
    if (!s_inited)
        return;
    set_sda(true);
    scl_release_and_wait();
    half_delay();
    set_sda(false); // SDA falls while SCL is high — START condition
    half_delay();
    line_drive_low(s_scl);
}

void i2c_stop(void) {
    if (!s_inited)
        return;
    set_sda(false);
    half_delay();
    scl_release_and_wait();
    half_delay();
    set_sda(true); // SDA rises while SCL is high — STOP condition
    half_delay();
}

bool i2c_write_byte(uint8_t byte) {
    if (!s_inited)
        return false;
    for (uint8_t i = 0; i < 8u; i++) {
        write_bit((byte & 0x80u) != 0u);
        byte <<= 1;
    }
    return read_bit() == false; // ACK = slave drives SDA low
}

uint8_t i2c_read_byte(bool ack) {
    if (!s_inited)
        return 0u;
    uint8_t byte = 0u;
    for (uint8_t i = 0; i < 8u; i++) {
        byte = (uint8_t)((byte << 1) | (read_bit() ? 1u : 0u));
    }
    write_bit(!ack); // ACK -> drive SDA low; NACK -> release high
    return byte;
}

bool i2c_probe_address(uint8_t addr) {
    if (!s_inited)
        return false;
    i2c_start();
    bool acked = i2c_write_byte((uint8_t)(addr << 1)); // write direction
    i2c_stop();
    return acked;
}

size_t i2c_bus_scan(uint8_t* found, size_t max) {
    if (!s_inited || found == NULL || max == 0u)
        return 0u;
    size_t n = 0u;
    for (uint16_t addr = I2C_SCAN_ADDR_MIN; addr <= I2C_SCAN_ADDR_MAX && n < max; addr++) {
        if (i2c_probe_address((uint8_t)addr)) {
            found[n++] = (uint8_t)addr;
        }
    }
    return n;
}
