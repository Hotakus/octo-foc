/**
  ******************************************************************************
  * @file           : pid.c
  * @author         : Hotakus (hotakus@foxmail.com)
  * @brief          : None
  * @date           : 2024/10/10
  ******************************************************************************
  */

#include "pid.h"
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

void pid_set_output_limit(pid_t *pid, float out_max, float out_min) {
    if (pid == NULL) {
        return;
    }
    pid->out_max = out_max;
    pid->out_min = out_min;
}

void pid_set_ramp_limit(pid_t *pid, float ramp) {
    if (pid == NULL) {
        return;
    }
    pid->out_ramp = ramp;
}

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
    pid->proportional = pid->kp * pid->current_err;

    if (pid->enable_part & PID_PART_INTEGRAL) {
        pid->integral += (pid->ki * pid->dt) * (pid->current_err + pid->last_err) * 0.5f;

        if (pid->integral > pid->out_max) {
            pid->integral = pid->out_max;
        } else if (pid->integral < pid->out_min) {
            pid->integral = pid->out_min;
        }
    }

    if (pid->enable_part & PID_PART_DERIVATIVE) {
        pid->derivative = pid->kd / pid->dt * (pid->current_err - pid->last_err);
    }

    pid->output = pid->proportional + pid->integral + pid->derivative;

    if(pid->out_ramp > 0){
        pid->output_rate = (pid->output - pid->output_prev)/pid->dt;
        if (pid->output_rate > pid->out_ramp)
            pid->output = pid->output_prev + pid->out_ramp*pid->dt;
        else if (pid->output_rate < -pid->out_ramp)
            pid->output = pid->output_prev - pid->out_ramp*pid->dt;
    }

    if (pid->output > pid->out_max) {
        pid->output = pid->out_max;
    } else if (pid->output < pid->out_min) {
        pid->output = pid->out_min;
    }

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
        pid->integral += pid->ki * pid->current_err;

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
