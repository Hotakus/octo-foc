/*******************************************************************************
* @file           : as5600_esp32_idf_i2c_port.h
* @author         : Hotakus (hotakus@foxmail.com)
* @brief          : FOC main header file
* @date           : 2025/2/10
*
* SPDX-License-Identifier: MPL-2.0
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this file,
* You can obtain one at https://mozilla.org/MPL/2.0/.
* Copyright (c) 2025 Hotakus. All rights reserved.
*****************************************************************************/

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
