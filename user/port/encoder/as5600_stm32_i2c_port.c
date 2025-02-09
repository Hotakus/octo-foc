/**
  ******************************************************************************
  * @file           : as5600_stm32_port.c
  * @author         : Hotakus (hotakus@foxmail.com)
  * @brief          : None
  * @date           : 2025/2/9
  ******************************************************************************
  */

#include "as5600_stm32_i2c_port.h"

#include <stdlib.h>


#if AS5600_STM32_I2C_PORT_ENABLE == 1

#include "i2c.h"

static uint8_t motor1_as5600_iic_init(void) {
    return 0;
}

static uint8_t motor1_as5600_iic_deinit(void) {
    return 0;
}

HAL_StatusTypeDef I2C_WaitUntilDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Timeout) {
    uint32_t tickstart = HAL_GetTick();
    while (HAL_GetTick() - tickstart <= Timeout) {
        if (HAL_I2C_IsDeviceReady(hi2c, DevAddress, 1, Timeout) == HAL_OK) {
            return HAL_OK;
        }
    }
    return HAL_TIMEOUT;
}

static uint8_t motor1_as5600_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len) {
    HAL_StatusTypeDef res = HAL_OK;
    res = HAL_I2C_Mem_Read(&hi2c2, addr, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
    if (res != HAL_OK) {
        AS5600_PORT_DEBUG("read failed\n\r");
        return 1;
    }
    return 0;
}

static uint8_t motor1_as5600_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len) {
    return 0;
}
#endif


as5600_handle_t *as5600_stm32_i2c_port_init(void) {
#if AS5600_STM32_I2C_PORT_ENABLE == 1
    /* AS5600 init */
    as5600_handle_t *as5600 = malloc(sizeof(as5600_handle_t));
    DRIVER_AS5600_LINK_INIT(as5600, as5600_handle_t);
    DRIVER_AS5600_LINK_IIC_INIT(as5600, motor1_as5600_iic_init);
    DRIVER_AS5600_LINK_IIC_DEINIT(as5600, motor1_as5600_iic_deinit);
    DRIVER_AS5600_LINK_IIC_READ(as5600, motor1_as5600_iic_read);
    DRIVER_AS5600_LINK_IIC_WRITE(as5600, motor1_as5600_iic_write);
    DRIVER_AS5600_LINK_DELAY_MS(as5600, AS5600_PORT_DELAY);
    DRIVER_AS5600_LINK_DEBUG_PRINT(as5600, AS5600_PORT_DEBUG);
    uint8_t as5600_res = as5600_init(as5600);
    return as5600;
#else
    return NULL;
#endif
}
