/**
******************************************************************************
* @file           : fdcan_driver.h
* @author         : Hotakus (hotakus@foxmail.com)
* @brief          : None
* @date           : 2025/2/8
******************************************************************************
*/

#ifndef FDCAN_DRIVER_H
#define FDCAN_DRIVER_H


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef struct fdcan_frame_t {
    uint32_t id;
    uint8_t data[64];
    uint8_t dlc;
    bool is_extended;
    bool is_fd;
} fdcan_frame_t;

// 硬件抽象接口（需用户根据平台实现）
typedef struct {
    bool (*init)(uint32_t arbitration_rate, uint32_t data_rate);

    bool (*send)(const fdcan_frame_t *frame);
    bool (*receive)(fdcan_frame_t *frame);

    void (*set_filter)(uint8_t index, uint32_t id, uint32_t mask);
    void (*set_exfilter)(uint8_t index, uint32_t id, uint32_t mask);
} fdcan_hardware_ops_t;

typedef struct fdcan_driver_t {
    fdcan_hardware_ops_t ops; // 硬件操作接口
    bool is_initialized;
} fdcan_driver_t;

bool fdcan_init(fdcan_driver_t *driver, uint32_t arbitration_rate, uint32_t data_rate);
bool fdcan_send(fdcan_driver_t *driver, const fdcan_frame_t *frame, uint32_t timeout_ms);
bool fdcan_receive(fdcan_driver_t *driver, fdcan_frame_t *frame);
void fdcan_set_filter(fdcan_driver_t *driver, uint8_t index, uint32_t id, uint32_t mask);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif  //FDCAN_DRIVER_H
