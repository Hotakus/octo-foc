/**
  ******************************************************************************
  * @file           : pid.c
  * @author         : Hotakus (hotakus@foxmail.com)
  * @brief          : None
  * @date           : 2024/10/10
  ******************************************************************************
  */

#include "pid.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

/**
 * @brief      创建PID控制器
 * @return     PID控制器指针
 */
pid_t *pid_create(pid_part_enable_enum_t enable_part) {
    pid_t *pid = (pid_t *)PID_MALLOC(sizeof(pid_t));
    if (pid == NULL) {
        FOC_PRINTF("pid malloc error\n\r");
        return NULL;
    }
    memset(pid, 0, sizeof(pid_t));
    pid->enable_part = enable_part;

    pid->i_hysteresis = PID_INTEGRAL_HYSTERESIS;
    pid->enable_integral_separation = false;

    return pid;
}

void pid_destroy(pid_t *pid) {
    if (pid == NULL) {
        return;
    }
    PID_FREE(pid);
}

/**
 * @brief      设置PID控制器的部分使能
 * @param[in]  pid        PID控制器指针
 * @param[in]  enable_part 使能的PID部分
 */
void pid_set_part(pid_t *pid, pid_part_enable_enum_t enable_part) {
    if (pid == NULL) {
        return;
    }
    pid->enable_part = enable_part;
}

void pid_set_feedforward_mode(pid_t *pid, feedforward_mode_enum_t mode, float kff) {
    if (pid == NULL) {
        return;
    }
    pid->feedforward_mode = mode;
    pid->kff = kff;
}

void pid_set_i_sep_mode(pid_t *pid, bool enable) {
    if (pid == NULL) {
        return;
    }
    pid->enable_integral_separation = enable;
}

/**
 * @brief      设置PID控制器的系数
 * @param[in]  pid        PID控制器指针
 * @param[in]  kp         P系数
 * @param[in]  ki         I系数
 * @param[in]  kd         D系数
 */
void pid_set_coefficient(pid_t *pid, float kp, float ki, float kd) {
    if (pid == NULL) {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void pid_set_hysteresis(pid_t *pid, float h) {
    if (pid == NULL) {
        return;
    }
    pid->i_hysteresis = h;
}

void pid_set_output_limit(pid_t *pid, float out_max, float out_min) {
    if (pid == NULL) {
        return;
    }
    pid->out_max = out_max;
    pid->out_min = out_min;
    pid->i_max = out_max * PID_INTEGRAL_LIMIT_FACTOR;
    pid->i_min = out_min * PID_INTEGRAL_LIMIT_FACTOR;
}

void pid_set_ramp_limit(pid_t *pid, float ramp) {
    if (pid == NULL) {
        return;
    }
    pid->out_ramp = ramp;
}


void pid_set_integral_separation(pid_t *pid, float i_sep) {
    if (pid == NULL) {
        return;
    }
    pid->i_sep = i_sep;
}

// #define HYSTERESIS 0.2f  // 滞后系数

/**
 * @brief      update the pid controller
 * @param[in]  pid        PID controller pointer
 * @param[in]  input      input
 * @param[in]  setpoint   setpoint
 */
void pid_positional_update(pid_t *pid, float input, float setpoint) {
    pid->input = input;
    pid->setpoint = setpoint;
    pid->current_err = pid->setpoint - pid->input;

    // 比例项计算
    pid->proportional = pid->kp * pid->current_err;

    if (pid->enable_part & PID_PART_INTEGRAL) {
        float delta_i = pid->ki * pid->dt * (pid->current_err + pid->last_err) * 0.5f;

        if (pid->enable_integral_separation) {
            // 分离条件判断
            int in_separation_zone = 0;
            float abs_err = fabsf(pid->current_err);

            // 正向误差且积分正向累积时冻结
            if ((pid->current_err > 0 && pid->integral > 0) ||
                (pid->current_err < 0 && pid->integral < 0)) {
                float hysteresis = pid->i_hysteresis;
                float upper_threshold = pid->i_sep * (1 + hysteresis);
                float lower_threshold = pid->i_sep * (1 - hysteresis);

                // 状态迁移逻辑
                if (abs_err > upper_threshold) {
                    in_separation_zone = 1;
                } else if (abs_err < lower_threshold) {
                    in_separation_zone = 0;
                } else {
                    // 保持之前状态
                    in_separation_zone = pid->last_separation_state;
                }
            }

            if (in_separation_zone) {
                delta_i = 0;
                // 添加积分衰减（关键！）
                pid->integral *= 0.50f; // 衰减系数可调
            }

            // 保存分离状态供下次使用
            pid->last_separation_state = in_separation_zone;
        }

        // 抗饱和条件判断
        int saturation_flag = 0;
        if ((pid->output_prev >= pid->out_max && pid->current_err > 0) ||
            (pid->output_prev <= pid->out_min && pid->current_err < 0)) {
            saturation_flag = 1;
        }

        // 仅在非饱和或误差减小时允许积分
        if (!saturation_flag) {
            pid->integral += delta_i;
            // 应用独立积分限幅
            pid->integral = fmaxf(fminf(pid->integral, pid->i_max), pid->i_min);
        }
    } else {
        pid->integral = 0;
    }

    // ==== 改进的微分处理 ====
    if (pid->enable_part & PID_PART_DERIVATIVE) {
        // 原始微分计算
        pid->derivative = pid->kd / pid->dt * (pid->current_err - pid->last_err);
    } else {
        pid->derivative = 0;
    }

    // output
    pid->output = pid->proportional + pid->integral + pid->derivative;
    // feedforward
    if (pid->feedforward_mode != PID_FEEDFORWARD_DISABLE) {
        pid->output += pid->kff * pid->setpoint;
    }

    if (pid->out_ramp > 0 && pid->dt > 1e-6) {
        float max_delta = pid->out_ramp * pid->dt;
        pid->output = fmaxf(fminf(pid->output, pid->output_prev + max_delta),
                            pid->output_prev - max_delta);
    }

    float safe_max = pid->out_max * (1.0f - PID_OUTPUT_MARGINS);
    float safe_min = pid->out_min * (1.0f - PID_OUTPUT_MARGINS);
    pid->output = fmaxf(fminf(pid->output, safe_max), safe_min);

    pid->last_err = pid->current_err;
    pid->output_prev = pid->output;
}


/**
 * @brief      update the pid controller incremental
 * @param[in]  pid        PID controller pointer
 * @param[in]  input      input
 * @param[in]  setpoint   setpoint
 */
void pid_incremental_update(pid_t *pid, float input, float setpoint) {
    pid->input = input;
    pid->setpoint = setpoint;
    pid->current_err = pid->setpoint - pid->input;
    pid->proportional = pid->kp * pid->current_err;

    if (pid->enable_part & PID_PART_INTEGRAL) {
        pid->integral += pid->ki * (pid->current_err + pid->last_err) * 0.5f;

        if (pid->integral > pid->out_max) {
            pid->integral = pid->out_max;
        } else if (pid->integral < pid->out_min) {
            pid->integral = pid->out_min;
        }
    }

    if (pid->enable_part & PID_PART_DERIVATIVE) {
        pid->derivative = pid->kd * (pid->current_err - pid->last_err);
    }

    pid->output = pid->proportional + pid->integral + pid->derivative;
    if (pid->output > pid->out_max) {
        pid->output = pid->out_max;
    } else if (pid->output < pid->out_min) {
        pid->output = pid->out_min;
    }

    pid->last_err = pid->current_err;
    pid->output_prev = pid->output;
}

void pid_set_dt(pid_t *pid, float dt) {
    if (pid == NULL) {
        return;
    }
    pid->dt = dt;
}
