/**
******************************************************************************
* @file           : soft_control.h
* @author         : Hotakus (hotakus@foxmail.com)
* @brief          : None
* @date           : 2025/1/28
******************************************************************************
*/

#ifndef SOFT_CONTROL_H
#define SOFT_CONTROL_H

#include <stdint.h>

#define SOFT_CONTROL_MIN_DURATION 0.0001f

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SOFT_CONTROL_RAMP_IDLE, // idle
    SOFT_CONTROL_RAMP_ACCEL, // accel
    SOFT_CONTROL_RAMP_CRUISE, // cruise
    SOFT_CONTROL_RAMP_DECEL // decel
} soft_ctrl_ramp_state_t;

typedef void (*state_change_cb)(soft_ctrl_ramp_state_t old_state, soft_ctrl_ramp_state_t new_state);

typedef struct {
    float tgt_vel; // target velocity
    float curr_vel; // current velocity

    float t_accel; // acceleration time
    float t_decel; // deceleration time

    soft_ctrl_ramp_state_t state; // current state
    float start_vel; // start velocity
    float elapsed_t; // elapsed time
    uint8_t emergency; // emergency stop flag

    // callback
    state_change_cb on_state_change; // state change callback
} soft_ctrl_ramp_t;

const char *soft_ctrl_ramp_get_state_str(const soft_ctrl_ramp_t *ctrl);
void soft_ctrl_ramp_reset(soft_ctrl_ramp_t *ctrl, float init_vel);
float soft_ctrl_ramp_update(soft_ctrl_ramp_t *ctrl, float dt);
void soft_ctrl_ramp_start_decel(soft_ctrl_ramp_t *ctrl, float duration);
void soft_ctrl_ramp_start_accel(soft_ctrl_ramp_t *ctrl, float target_vel, float duration);
void soft_ctrl_ramp_emergency_stop(soft_ctrl_ramp_t *ctrl);
void soft_ctrl_ramp_init(soft_ctrl_ramp_t *ctrl, float init_vel);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif  //SOFT_CONTROL_H
