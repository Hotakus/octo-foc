/**
  ******************************************************************************
  * @file           : as5600_esp32_idf_i2c_port.h
  * @author         : Hotakus (hotakus@foxmail.com)
  * @brief          : None
  * @date           : 2025/2/9
  ******************************************************************************
  */

#ifndef AS5600_ESP32_IDF_I2C_PORT_H
#define AS5600_ESP32_IDF_I2C_PORT_H

#define AS5600_ESP32_IDF_I2C_PORT_ENABLE  (0u)
#define AS5600_ESP32_IDF_I2C_PORT_DEBUG   printf
#define AS5600_ESP32_IDF_I2C_PORT_DELAY   vTaskDelay

#ifdef __cplusplus
extern "C" {
#endif

#include "as5600.h"
as5600_handle_t *as5600_esp_idf_i2c_port_init(void);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif  //AS5600_ESP32_IDF_I2C_PORT_H
