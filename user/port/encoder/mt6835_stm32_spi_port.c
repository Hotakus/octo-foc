/**
******************************************************************************
* @file           : mt6835_stm32_spi_port.c
* @author         : Hotakus (hotakus@foxmail.com)
* @brief          : None
* @date           : 2025/2/9
******************************************************************************
*/

#include "mt6835_stm32_spi_port.h"

#define MT6835_STM32_SPI_PORT_ENABLE  (1u)
#if MT6835_STM32_SPI_PORT_ENABLE == 1

#include "spi.h"
#define SPI_INSTANCE    hspi1
#define SPI_CS          SPI1_CS_Pin
#define SPI_CS_PORT     SPI1_CS_GPIO_Port

static void mt6835_cs_control(mt6835_cs_state_enum_t state) {
    if (state == MT6835_CS_HIGH) {
        HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS, GPIO_PIN_RESET);
    }
}

static void mt6835_spi_send(uint8_t *tx_buf, uint8_t len) {
    HAL_StatusTypeDef status = HAL_OK;
    status = HAL_SPI_Transmit(&SPI_INSTANCE, tx_buf, len, 10);
    if (status != HAL_OK) {
        printf("spi send failed %d\n\r", status);
        return;
    }
}

static void mt6835_spi_recv(uint8_t *rx_buf, uint8_t len) {
    HAL_StatusTypeDef status = HAL_OK;
    status = HAL_SPI_Receive(&SPI_INSTANCE, rx_buf, len, 10);
    if (status != HAL_OK) {
        printf("spi send failed %d\n\r", status);
        return;
    }
}

static void mt6835_spi_send_recv(uint8_t *tx_buf, uint8_t *rx_buf, uint8_t len) {
    HAL_StatusTypeDef status = HAL_OK;
    status = HAL_SPI_TransmitReceive_IT(&SPI_INSTANCE, tx_buf, rx_buf, len);
    if (status != HAL_OK) {
        printf("spi send_recv failed %d\n\r", status);
        return;
    }
    // wait IT
    uint32_t tickstart = HAL_GetTick();
    while (HAL_SPI_GetState(&SPI_INSTANCE) != HAL_SPI_STATE_READY) {
        if (HAL_GetTick() - tickstart > 1) {
            printf("spi send_recv timeout\n\r");
            return;
        }
    }
}

/**
 * @brief mt6835 stm32 spi port init
 * @return mt6835 object
 */
mt6835_t * mt6835_stm32_spi_port_init(void) {
    mt6835_t *mt6835 = mt6835_create();
    mt6835_link_spi_cs_control(mt6835, mt6835_cs_control);
    mt6835_link_spi_send_recv(mt6835, mt6835_spi_send_recv);
    mt6835_link_spi_send(mt6835, mt6835_spi_send);
    mt6835_link_spi_recv(mt6835, mt6835_spi_recv);
    return mt6835;
}
#endif
