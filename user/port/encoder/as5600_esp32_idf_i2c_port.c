/**
******************************************************************************
* @file           : as5600_esp32_idf_i2c_port.c
* @author         : Hotakus (hotakus@foxmail.com)
* @brief          : None
* @date           : 2025/2/9
******************************************************************************
*/

#include "as5600_esp32_idf_i2c_port.h"

#include <esp_attr.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>

#if AS5600_ESP32_IDF_I2C_PORT_ENABLE == 1

static gpio_num_t i2c0_gpio_sda = GPIO_NUM_19; // I2C0 SDA
static gpio_num_t i2c0_gpio_scl = GPIO_NUM_18; // I2C0 SCL
static i2c_port_num_t i2c0_port_num = I2C_NUM_0; // I2C0 port
static i2c_master_bus_config_t i2c0_bus_config; // I2C0 config
static i2c_master_bus_handle_t i2c0_bus_handle; // I2C0 handle


static i2c_master_dev_handle_t motor1_as5600_i2c_dev;
static i2c_device_config_t motor1_as5600_i2c_dev_conf = {
    .scl_speed_hz = 400 * 1000, // 400khz, can up to 1000khz
    .device_address = 0x36, // 0x36
};

uint8_t motor1_as5600_iic_init(void) {
    /* I2C0 init  */
    i2c0_bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c0_bus_config.i2c_port = i2c0_port_num;
    i2c0_bus_config.scl_io_num = i2c0_gpio_scl;
    i2c0_bus_config.sda_io_num = i2c0_gpio_sda;
    i2c0_bus_config.glitch_ignore_cnt = 7;
    i2c0_bus_config.flags.enable_internal_pullup = 1;
    i2c_new_master_bus(&i2c0_bus_config, &i2c0_bus_handle);
    i2c_master_bus_add_device(i2c0_bus_handle, &motor1_as5600_i2c_dev_conf, &motor1_as5600_i2c_dev);
    return 0;
}

uint8_t motor1_as5600_iic_deinit(void) {
    return 0;
}

uint8_t IRAM_ATTR motor1_as5600_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len) {
    esp_err_t res = i2c_master_transmit_receive(motor1_as5600_i2c_dev, &reg, 1, buf, len, 100);
    if (res != ESP_OK) {
        return 1;
    }
    return 0;
}

uint8_t motor1_as5600_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len) {
    return 0;
}

/* --------------------------- AS5600 I2C Port end --------------------------- */
#endif


as5600_handle_t *as5600_stm32_i2c_port_init(void) {
#if AS5600_ESP32_IDF_I2C_PORT_ENABLE == 1
    /* AS5600 init */
    as5600_handle_t *as5600 = malloc(sizeof(as5600_handle_t));
    DRIVER_AS5600_LINK_INIT(as5600, as5600_handle_t);
    DRIVER_AS5600_LINK_IIC_INIT(as5600, motor1_as5600_iic_init);
    DRIVER_AS5600_LINK_IIC_DEINIT(as5600, motor1_as5600_iic_deinit);
    DRIVER_AS5600_LINK_IIC_READ(as5600, motor1_as5600_iic_read);
    DRIVER_AS5600_LINK_IIC_WRITE(as5600, motor1_as5600_iic_write);
    DRIVER_AS5600_LINK_DELAY_MS(as5600, AS5600_ESP32_IDF_I2C_PORT_DELAY);
    DRIVER_AS5600_LINK_DEBUG_PRINT(as5600, AS5600_ESP32_IDF_I2C_PORT_DEBUG);
    uint8_t as5600_res = as5600_init(as5600);
    return as5600;
#else
  return NULL;
#endif
}
