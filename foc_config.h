/*******************************************************************************
* @file           : foc_config.h
* @author         : Hotakus (hotakus@foxmail.com)
* @brief          : FOC global config file
* @date           : 2025/2/10
*
* SPDX-License-Identifier: MPL-2.0
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this file,
* You can obtain one at https://mozilla.org/MPL/2.0/.
* Copyright (c) 2025 Hotakus. All rights reserved.
*****************************************************************************/

#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H


/**
 * calculation method
 * 0: default math lib      (math.h)
 * 1: ARM math lib          (unrealized)
 * 2: STM32 CORDIC          (such as STM32G4 or some STM32H7 series)
 * 3: custom fast math lib  (fast trigonometric functions)
 */
#define FOC_MATH_CAL_METHOD         (3u)

#define FOC_USE_FULL_ASSERT         (1u)    // 1: enable ; 0: disable
#define FOC_DEBUG                   (1u)    // 1: enable ; 0: disable
#define FOC_USE_FREERTOS            (0u)    // 1: use FreeRTOS ; 0: not use
#define FOC_USE_ESP_IDF             (0u)    // 1: use ESP-IDF ; 0: not use

/* FOC_DATA_PERSISTENCE use to save important data to storage device
 * such as the "init_angle" item in "foc_t",
 * it use to calibrate the zero point of electrical angle of FOC,
 * the zero point always is fixed for UVW motor, dont need to get per time
 *
 * If you enable this feature, you must implement some functions
 */
#define FOC_DATA_PERSISTENCE    (0u)

#if FOC_DEBUG == 1
#include <stdio.h>
#define FOC_PRINTF(fmt,...)     printf(fmt, ##__VA_ARGS__)
#else
#define FOC_PRINTF(fmt,...)
#endif

/* --------------------- Include --------------------- */
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

/* --------------------- FOC_USE_FREERTOS --------------------- */
#if FOC_USE_FREERTOS == 1
#if FOC_USE_ESP_IDF == 1
#include "freertos/FreeRTOS.h"
#define FOC_LOCK_ACQUIRE(handle, delay)   (spinlock_acquire(&handle, delay))
#define FOC_LOCK_RELEASE(handle)          (spinlock_release(&handle))
#else
#include "cmsis_os.h"
#define FOC_LOCK_ACQUIRE(handle, delay)   (osMutexAcquire(handle, delay))
#define FOC_LOCK_RELEASE(handle)          (osMutexRelease(handle))
#endif
#define FOC_DELAY(ms)                     (vTaskDelay(pdMS_TO_TICKS(ms)))
#define FOC_MALLOC(size)                  (pvPortMalloc(size))
#define FOC_FREE(ptr)                     (vPortFree(ptr))
#elif FOC_USE_FREERTOS == 0
#define FOC_DELAY(x)                      (HAL_Delay(x))
#define FOC_MALLOC(size)                  (malloc(size))
#define FOC_FREE(ptr)                     (free(ptr))
#define FOC_LOCK_ACQUIRE(handle, delay)
#define FOC_LOCK_RELEASE(handle)
#endif

#define FOC_ATTR


#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif // __cplusplus

#endif  //FOC_CONFIG_H
