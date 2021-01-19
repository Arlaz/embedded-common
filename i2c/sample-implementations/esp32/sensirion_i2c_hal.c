/*
 * Copyright (c) 2018, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "sensirion_i2c_hal.h"
#include "sensirion_common.h"
#include "sensirion_config.h"

#include <driver/i2c.h>
#include <esp_log.h>

#define SENSIRION_SDA_PIN 21
#define SENSIRION_SCL_PIN 22
#define I2C_FREQUENCY 100000  // esp-idf doesn't support frequencies above 1Mhz
#define I2C_PORT I2C_NUM_1    // can be free

#define ACK_CHECK_EN 0x1 /*!< I2C master will check ack from slave*/
#define ESP_INTR_FLG_DEFAULT (0)
#define ESP_I2C_MASTER_BUF_LEN (0)

static const char* I2C_BUS_TAG = "Sensirion i2c";

#define I2C_BUS_CHECK(a, str, ret)                                 \
    if (!(a)) {                                                    \
        ESP_LOGE(I2C_BUS_TAG, "%s:%d (%s):%s", __FILE__, __LINE__, \
                 __FUNCTION__, str);                               \
        return (ret);                                              \
    }

typedef struct {
    i2c_config_t i2c_conf; /*!<I2C bus parameters*/
    i2c_port_t i2c_port;   /*!<I2C port number */
} i2c_bus_t;

static i2c_bus_t* bus = NULL;

/**
 * Select the current i2c bus by index.
 * All following i2c operations will be directed at that bus.
 *
 * THE IMPLEMENTATION IS OPTIONAL ON SINGLE-BUS SETUPS (all sensors on the same
 * bus)
 *
 * @param bus_idx   Bus index to select
 * @returns         0 on success, an error code otherwise
 */
int16_t sensirion_i2c_select_bus(uint8_t bus_idx) {
    return NOT_IMPLEMENTED_ERROR;
}

/**
 * Initialize all hard- and software components that are needed for the I2C
 * communication.
 */
void sensirion_i2c_init(void) {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = SENSIRION_SDA_PIN;
    conf.scl_io_num = SENSIRION_SCL_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQUENCY;

    bus = (i2c_bus_t*)calloc(1, sizeof(i2c_bus_t));
    bus->i2c_conf = conf;
    bus->i2c_port = I2C_PORT;
    esp_err_t ret = i2c_param_config(bus->i2c_port, &bus->i2c_conf);
    if (ret != ESP_OK) {
        goto error;
    }
    ret = i2c_driver_install(bus->i2c_port, conf.mode, ESP_I2C_MASTER_BUF_LEN,
                             ESP_I2C_MASTER_BUF_LEN, ESP_INTR_FLG_DEFAULT);
    if (ret != ESP_OK) {
        goto error;
    }

error:
    if (bus) {
        free(bus);
    }
}

/**
 * Release all resources initialized by sensirion_i2c_init().
 */
void sensirion_i2c_release(void) {
    i2c_bus_t* i2c_bus = (i2c_bus_t*)bus;
    i2c_driver_delete(i2c_bus->i2c_port);
    free(bus);
}

/**
 * Execute one read transaction on the I2C bus, reading a given number of bytes.
 * If the device does not acknowledge the read command, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to read from
 * @param data    pointer to the buffer where the data is to be stored
 * @param count   number of bytes to read from I2C and store in the buffer
 * @returns 0 on success, error code otherwise
 */
int8_t sensirion_i2c_read(uint8_t address, uint8_t* data, uint16_t count) {
    I2C_BUS_CHECK(bus != NULL, "Bus not initialized", ESP_FAIL);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read(cmd, data, count, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    i2c_bus_t* i2c_bus = (i2c_bus_t*)bus;
    esp_err_t ret =
        i2c_master_cmd_begin(i2c_bus->i2c_port, cmd, 1000 / portTICK_RATE_MS);

    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * Execute one write transaction on the I2C bus, sending a given number of
 * bytes. The bytes in the supplied buffer must be sent to the given address. If
 * the slave device does not acknowledge any of the bytes, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to write to
 * @param data    pointer to the buffer containing the data to write
 * @param count   number of bytes to read from the buffer and send over I2C
 * @returns 0 on success, error code otherwise
 */
int8_t sensirion_i2c_write(uint8_t address, const uint8_t* data,
                           uint16_t count) {
    I2C_BUS_CHECK(bus != NULL, "Bus not initialized", ESP_FAIL);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, address << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write(cmd, data, count, true);
    i2c_master_stop(cmd);

    i2c_bus_t* i2c_bus = (i2c_bus_t*)bus;
    esp_err_t ret =
        i2c_master_cmd_begin(i2c_bus->i2c_port, cmd, 1000 / portTICK_RATE_MS);

    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * Sleep for a given number of microseconds. The function should delay the
 * execution for at least the given time, but may also sleep longer.
 *
 * Despite the unit, a <10 millisecond precision is sufficient.
 *
 * @param useconds the sleep time in microseconds
 */
void sensirion_sleep_usec(uint32_t useconds) {
    const int us_per_tick = portTICK_PERIOD_MS * 1000;
    if (useconds < us_per_tick) {
        ets_delay_us(useconds);
    } else {
        vTaskDelay((useconds + us_per_tick - 1) / us_per_tick);
    }
}
