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

/**
 * @brief Initialize the ramp controller
 * @param ctrl ramp controller
 * @param init_vel initial velocity
 */
void soft_ctrl_ramp_init(soft_ctrl_ramp_t *ctrl, float init_vel) {
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


static void start_ramp(soft_ctrl_ramp_t *ctrl, soft_ctrl_ramp_state_t new_state, float target_vel, float duration) {
    const soft_ctrl_ramp_state_t prev_state = ctrl->state;
    ctrl->tgt_vel = target_vel;
    if (new_state == SOFT_CONTROL_RAMP_ACCEL) {
        ctrl->t_accel = fmaxf(duration, SOFT_CONTROL_MIN_DURATION);
    } else {
        ctrl->t_decel = fmaxf(duration, SOFT_CONTROL_MIN_DURATION);
    }
    ctrl->start_vel = ctrl->curr_vel;
    ctrl->elapsed_t = 0.0f;
    ctrl->state = new_state;

    if (ctrl->on_state_change && prev_state != new_state) {
        ctrl->on_state_change(prev_state, new_state);
    }
}


/**
 * @brief Start acceleration process
 * @param ctrl ramp controller
 * @param target_vel target velocity
 * @param duration duration of acceleration process (>= 0)
 */
void soft_ctrl_ramp_start_accel(soft_ctrl_ramp_t *ctrl, float target_vel, float duration) {
    if (ctrl->emergency) return;
    start_ramp(ctrl, SOFT_CONTROL_RAMP_ACCEL, target_vel, duration);
}


/**
 * @brief Start deceleration process
 * @param ctrl ramp controller
 * @param duration duration of deceleration process (>= 0)
 */
void soft_ctrl_ramp_start_decel(soft_ctrl_ramp_t *ctrl, float target_vel, float duration) {
    if (ctrl->emergency) return;
    start_ramp(ctrl, SOFT_CONTROL_RAMP_DECEL, target_vel, duration);
}


/**
 * @brief Update the ramp controller
 * @param ctrl ramp controller
 * @param dt delta time (>= 0)
 * @return current velocity
 */
float soft_ctrl_ramp_update(soft_ctrl_ramp_t *ctrl, float dt) {
    dt = fmaxf(dt, 0.0f);

    switch (ctrl->state) {
        case SOFT_CONTROL_RAMP_IDLE:
            ctrl->curr_vel = ctrl->tgt_vel;
            break;

        case SOFT_CONTROL_RAMP_ACCEL:
        case SOFT_CONTROL_RAMP_DECEL: {
            /* 统一处理加速/减速插值 */
            const float total_t = (ctrl->state == SOFT_CONTROL_RAMP_ACCEL)
                                      ? ctrl->t_accel
                                      : ctrl->t_decel;

            ctrl->elapsed_t += dt;
            const float t = fminf(ctrl->elapsed_t / total_t, 1.0f);
            const float ease = cubic_easing(t);

            /* 通用线性插值公式 */
            ctrl->curr_vel = ctrl->start_vel + (ctrl->tgt_vel - ctrl->start_vel) * ease;

            /* 状态转移判断 */
            if (t >= 1.0f) {
                const int is_zero = fabsf(ctrl->tgt_vel) < 1e-6f;
                ctrl->state = is_zero ? SOFT_CONTROL_RAMP_IDLE : SOFT_CONTROL_RAMP_CRUISE;
                ctrl->curr_vel = ctrl->tgt_vel; // 消除插值误差
            }
            break;
        }

        case SOFT_CONTROL_RAMP_CRUISE:
            /* 巡航状态维持目标速度 */
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
 * @param init_vel initial velocity
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
