/*

Copyright (c) 2018 Mika Tuupola <tuupola@appelsiini.net>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>

static bool g_i2c_started;
static uint8_t g_i2c_sda;
static uint8_t g_i2c_scl;

#define LOW   0x00
#define HIGH  0x01

#define ACK   0x00
#define NAK   0x01

#define CLOCK_STRETCH_TIMEOUT   1000

static const char* TAG = "software_i2c";

/* https://esp-idf.readthedocs.io/en/latest/api-reference/peripherals/i2c.html#_CPPv211i2c_set_pin10i2c_port_tii13gpio_pullup_t13gpio_pullup_t10i2c_mode_t */
esp_err_t sw_i2c_init(uint8_t sda, uint8_t scl)
{
    ESP_LOGD(TAG, "Initializing software i2c with data pin %d.", sda);
    gpio_set_direction(sda, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_pull_mode(sda, GPIO_FLOATING);

    ESP_LOGD(TAG, "Initializing software i2c with clock pin %d.", scl);
    gpio_set_direction(scl, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_pull_mode(scl, GPIO_FLOATING);

    /* Save the pins in static global variables. */
    g_i2c_sda = sda;
    g_i2c_scl = scl;

    return ESP_OK;
}

esp_err_t sw_i2c_master_start()
{
    uint32_t stretch = CLOCK_STRETCH_TIMEOUT;

     /* If already started, do a restart condition. */
    if (g_i2c_started) {
        gpio_set_level(g_i2c_sda, HIGH);
        ets_delay_us(10);
        gpio_set_level(g_i2c_scl, HIGH);
        while (gpio_get_level(g_i2c_scl) == LOW && stretch--) {
            ets_delay_us(1);
        };
        ets_delay_us(10);
    }

    if (LOW == gpio_get_level(g_i2c_sda)) {
        ESP_LOGD(TAG, "Arbitration lost in sw_i2c_master_start()");
    }

    /* Start bit is indicated by a high-to-low transition of SDA with SCL high. */
    gpio_set_level(g_i2c_sda, LOW);
    ets_delay_us(10);
    gpio_set_level(g_i2c_scl, LOW);

    g_i2c_started = true;

    return ESP_OK;
}

esp_err_t sw_i2c_master_stop()
{
    uint32_t stretch = CLOCK_STRETCH_TIMEOUT;

    /* The stop bit is indicated by a low-to-high transition of SDA with SCL high. */
    gpio_set_level(g_i2c_sda, LOW);
    ets_delay_us(10);
    gpio_set_level(g_i2c_scl, HIGH);

    while (gpio_get_level(g_i2c_scl) == LOW && stretch--) {
        ets_delay_us(1);
    };

    ets_delay_us(10);
    gpio_set_level(g_i2c_sda, HIGH);
    ets_delay_us(10);

    if (gpio_get_level(g_i2c_sda) == LOW) {
        ESP_LOGD(TAG, "Arbitration lost in sw_i2c_master_stop()");
    }

    g_i2c_started = false;

    return ESP_OK;
}

static void sw_i2c_write_bit(bool bit)
{
    uint32_t stretch = CLOCK_STRETCH_TIMEOUT;

    gpio_set_level(g_i2c_sda, bit);
    ets_delay_us(10); /* SDA change propagation delay */
    gpio_set_level(g_i2c_scl, HIGH); /* New valid SDA value is available. */

    while (gpio_get_level(g_i2c_scl) == LOW && stretch--) {
        ets_delay_us(1);
    };

    ets_delay_us(10); /* Wait for SDA value to be read by slave. */

    if (bit && (LOW == gpio_get_level(g_i2c_sda))) {
        ESP_LOGD(TAG, "Arbitration lost in sw_i2c_write_bit()");
    }

    gpio_set_level(g_i2c_scl, LOW); /* Prepare for next bit. */
}

static bool sw_i2c_read_bit()
{
    uint32_t stretch = CLOCK_STRETCH_TIMEOUT;
    bool bit;

    gpio_set_level(g_i2c_sda, HIGH); /* Let the slave drive data. */
    ets_delay_us(10); /* Wait for slave to write. */
    gpio_set_level(g_i2c_scl, HIGH); /* New valid SDA value is available. */

    while (gpio_get_level(g_i2c_scl) == LOW && stretch--) {
        ets_delay_us(1);
    };

    ets_delay_us(10); /* Wait for slave to write. */
    bit = gpio_get_level(g_i2c_sda); /* SCL is high, read a bit. */
    gpio_set_level(g_i2c_scl, LOW); /* Prepare for next bit. */

    return bit;
}

static uint8_t sw_i2c_read_byte(bool ack)
{
    uint8_t byte = 0;
    uint8_t bit;

    for (bit = 0; bit < 8; ++bit) {
        byte = (byte << 1) | sw_i2c_read_bit();
    }
    sw_i2c_write_bit(ack);

    return byte;
}

// Write a byte to I2C bus. Return 0 if ack by the slave.
/* https://github.com/espressif/esp-idf/blob/master/components/driver/i2c.c#L944-L956

cmd.ack_en = ack_en; enable
cmd.ack_exp = 0; excpect
cmd.ack_val = 0; send

*/

static bool sw_i2c_write_byte(uint8_t byte) //, bool ack_enable)
{
    uint8_t bit;
    bool ack;

    for (bit = 0; bit < 8; ++bit) {
        sw_i2c_write_bit((byte & 0x80) != 0);
        byte <<= 1;
    }
    ack = sw_i2c_read_bit();
    return ack;
}

esp_err_t sw_i2c_master_write_byte(uint8_t buffer) // ack_enable??
{
    return sw_i2c_write_byte(buffer);
    //return ESP_OK;
}

esp_err_t sw_i2c_master_write(uint8_t *buffer, uint8_t length) // ack_enable??
{
    while (length--) {
        sw_i2c_write_byte(*buffer++);
    }

    return ESP_OK;
}

esp_err_t sw_i2c_master_read(uint8_t *buffer, uint16_t length, bool ack) //, bool expect_ack)
{
    while(length) {
        *buffer = sw_i2c_read_byte(ack);
        buffer++;
        length--;
    }

    return ESP_OK;
}
