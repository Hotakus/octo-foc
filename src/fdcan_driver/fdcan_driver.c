/**
******************************************************************************
* @file           : fdcan_driver.c
* @author         : Hotakus (hotakus@foxmail.com)
* @brief          : None
* @date           : 2025/2/8
******************************************************************************
*/

#include "fdcan_driver.h"
#include <stdlib.h>

// 初始化驱动
bool fdcan_init(fdcan_driver_t *driver, uint32_t arbitration_rate, uint32_t data_rate) {
    if (!driver || !driver->ops.init) return false;
    driver->is_initialized = driver->ops.init(arbitration_rate, data_rate);
    return driver->is_initialized;
}

// 发送帧（带超时逻辑，需用户实现平台特定的等待机制）
bool fdcan_send(fdcan_driver_t *driver, const fdcan_frame_t *frame, uint32_t timeout_ms) {
    if (!driver->is_initialized || !driver->ops.send) {
        return false;
    }

    if (driver->ops.send(frame)) {
        return true;
    }
    return false;
}

// 接收帧（非阻塞）
bool fdcan_receive(fdcan_driver_t *driver, fdcan_frame_t *frame) {
    if (!driver->is_initialized || !driver->ops.receive) return false;
    return driver->ops.receive(frame);
}

// 设置接收过滤器
void fdcan_set_filter(fdcan_driver_t *driver, uint8_t index, uint32_t id, uint32_t mask) {
    if (driver->is_initialized && driver->ops.set_filter) {
        driver->ops.set_filter(index, id, mask);
    }
}
