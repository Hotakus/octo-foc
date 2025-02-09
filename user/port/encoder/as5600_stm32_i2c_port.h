/**
  ******************************************************************************
  * @file           : as5600_stm32_port.h
  * @author         : Hotakus (hotakus@foxmail.com)
  * @brief          : None
  * @date           : 2025/2/9
  ******************************************************************************
  */

#ifndef AS5600_STM32_PORT_H
#define AS5600_STM32_PORT_H

#define AS5600_STM32_SPI_PORT_ENABLE  (0u)

#define AS5600_PORT_DEBUG   printf
#define AS5600_PORT_DELAY   HAL_Delay

#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "as5600.h"

as5600_handle_t *as5600_stm32_i2c_port_init(void);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif  //AS5600_STM32_PORT_H
