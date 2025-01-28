/**
******************************************************************************
* @file           : soft_control.c
* @author         : Hotakus (hotakus@foxmail.com)
* @brief          : None
* @date           : 2025/1/28
******************************************************************************
*/

#include "soft_control.h"
#include <math.h>


/**
 * @brief Cubic easing function (Ease In/Out)
 * @param t [0, 1] normalized time
 * @return eased value
 */
static float cubic_easing(float t) {
    const float t3 = t * t * t;
    return 10.0f * t3 - 15.0f * t3 * t + 6.0f * t3 * t * t;
}

/* 初始化控制器 */
void soft_ctrl_ramp_init(soft_ctrl_ramp_t *ctrl, float init_vel) {
    init_vel = fmaxf(init_vel, 0.0f);

    *ctrl = (soft_ctrl_ramp_t){
        .tgt_vel = init_vel,
        .curr_vel = init_vel,
        .t_accel = 0.5f,
        .t_decel = 0.5f,
        .state = SOFT_CONTROL_RAMP_IDLE,
        .emergency = 0,
        .on_state_change = NULL
    };
}


/**
 * @brief Emergency stop the ramp
 * @param ctrl ramp controller
 */
void soft_ctrl_ramp_emergency_stop(soft_ctrl_ramp_t *ctrl) {
    const soft_ctrl_ramp_state_t prev_state = ctrl->state;
    ctrl->state = SOFT_CONTROL_RAMP_IDLE;
    ctrl->curr_vel = 0.0f;
    ctrl->tgt_vel = 0.0f;
    ctrl->emergency = 1;
    ctrl->elapsed_t = 0.0f;

    if (ctrl->on_state_change && prev_state != SOFT_CONTROL_RAMP_IDLE) {
        ctrl->on_state_change(prev_state, SOFT_CONTROL_RAMP_IDLE);
    }
}


/**
 * @brief Start acceleration process
 * @param ctrl ramp controller
 * @param target_vel target velocity (>= 0)
 * @param duration duration of acceleration process (>= 0)
 */
void soft_ctrl_ramp_start_accel(soft_ctrl_ramp_t *ctrl, float target_vel, float duration) {
    if (ctrl->emergency ||
        fabsf(target_vel - ctrl->tgt_vel) < 1e-6f) {
        return;
    }

    const soft_ctrl_ramp_state_t prev_state = ctrl->state;
    ctrl->tgt_vel = fmaxf(target_vel, 0.0f);
    ctrl->t_accel = fmaxf(duration, SOFT_CONTROL_MIN_DURATION);
    ctrl->start_vel = ctrl->curr_vel;
    ctrl->elapsed_t = 0.0f;
    ctrl->state = SOFT_CONTROL_RAMP_ACCEL;

    if (ctrl->on_state_change && prev_state != SOFT_CONTROL_RAMP_ACCEL) {
        ctrl->on_state_change(prev_state, SOFT_CONTROL_RAMP_ACCEL);
    }
}


/**
 * @brief Start deceleration process
 * @param ctrl ramp controller
 * @param duration duration of deceleration process (>= 0)
 */
void soft_ctrl_ramp_start_decel(soft_ctrl_ramp_t *ctrl, float duration) {
    if (ctrl->emergency) return;

    const soft_ctrl_ramp_state_t prev_state = ctrl->state;
    ctrl->tgt_vel = 0.0f;
    ctrl->t_decel = fmaxf(duration, SOFT_CONTROL_MIN_DURATION);
    ctrl->start_vel = ctrl->curr_vel;
    ctrl->elapsed_t = 0.0f;
    ctrl->state = SOFT_CONTROL_RAMP_DECEL;

    if (ctrl->on_state_change && prev_state != SOFT_CONTROL_RAMP_DECEL) {
        ctrl->on_state_change(prev_state, SOFT_CONTROL_RAMP_DECEL);
    }
}


/**
 * @brief Update the ramp controller
 * @param ctrl ramp controller
 * @param dt delta time (>= 0)
 * @return current velocity
 */
float soft_ctrl_ramp_update(soft_ctrl_ramp_t *ctrl, float dt) {
    dt = fmaxf(dt, 0.0f); // 确保时间正向

    switch (ctrl->state) {
        case SOFT_CONTROL_RAMP_IDLE:
            ctrl->curr_vel = ctrl->tgt_vel; // 速度同步
            break;

        case SOFT_CONTROL_RAMP_ACCEL: {
            ctrl->elapsed_t += dt;
            const float t = fminf(ctrl->elapsed_t / ctrl->t_accel, 1.0f);
            const float ease = cubic_easing(t);

            ctrl->curr_vel = ctrl->start_vel +
                             (ctrl->tgt_vel - ctrl->start_vel) * ease;

            if (t >= 1.0f) {
                ctrl->state = SOFT_CONTROL_RAMP_CRUISE;
                ctrl->curr_vel = ctrl->tgt_vel;
            }
            break;
        }

        case SOFT_CONTROL_RAMP_DECEL: {
            ctrl->elapsed_t += dt;
            const float t = fminf(ctrl->elapsed_t / ctrl->t_decel, 1.0f);
            const float ease = cubic_easing(t);

            ctrl->curr_vel = ctrl->start_vel * (1.0f - ease);

            if (t >= 1.0f) {
                ctrl->state = SOFT_CONTROL_RAMP_IDLE;
                ctrl->curr_vel = 0.0f;
                ctrl->tgt_vel = 0.0f;
            }
            break;
        }

        case SOFT_CONTROL_RAMP_CRUISE:
            // 消除浮点误差
            if (fabsf(ctrl->curr_vel - ctrl->tgt_vel) > 1e-6f) {
                ctrl->curr_vel = ctrl->tgt_vel;
            }
            break;

        default:
            soft_ctrl_ramp_emergency_stop(ctrl);
            break;
    }

    return ctrl->curr_vel;
}


/**
 * @brief Reset the ramp controller to its initial state
 * @param ctrl ramp controller
 * @param init_vel initial velocity (>= 0)
 */
void soft_ctrl_ramp_reset(soft_ctrl_ramp_t *ctrl, float init_vel) {
    soft_ctrl_ramp_init(ctrl, init_vel);
    ctrl->emergency = 0;
}


/**
 * @brief Get the string representation of the current state
 * @param ctrl ramp controller
 * @return string representation of the current state
 */
const char *soft_ctrl_ramp_get_state_str(const soft_ctrl_ramp_t *ctrl) {
    static const char *states[] = {
        "IDLE", "ACCEL", "CRUISE", "DECEL"
    };
    return (ctrl->state < sizeof(states) / sizeof(states[0])) ? states[ctrl->state] : "UNKNOWN";
}
