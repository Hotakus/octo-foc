/*******************************************************************************
* @file           : pid.h
* @author         : Hotakus (hotakus@foxmail.com)
* @brief          : FOC main header file
* @date           : 2025/2/10
*
* SPDX-License-Identifier: MPL-2.0
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this file,
* You can obtain one at https://mozilla.org/MPL/2.0/.
* Copyright (c) 2025 Hotakus. All rights reserved.
*****************************************************************************/

#ifndef PID_H
#define PID_H

#include "foc_config.h"

#define PID_MALLOC(size)      (FOC_MALLOC(size))
#define PID_FREE(ptr)         (FOC_FREE(ptr))

#define PID_INTEGRAL_LIMIT_FACTOR           (0.7f)
#define PID_INTEGRAL_SEPARATION_THRESHOLD   (0.5f)

#define PID_OUTPUT_MARGINS                  (0.5f)  // 10% margin
#define PID_INTEGRAL_HYSTERESIS             (0.2f)

#ifdef __cplusplus
extern "C" {
#endif

typedef enum pid_part_enable_enum_t pid_part_enable_enum_t;

typedef enum pid_part_enable_enum_t {
    PID_PART_PROPORTIONAL = 0x01,
    PID_PART_INTEGRAL = 0x02,
    PID_PART_DERIVATIVE = 0x04,
    PID_PART_ALL = PID_PART_PROPORTIONAL | PID_PART_INTEGRAL | PID_PART_DERIVATIVE
} pid_part_enable_enum_t;

typedef enum feedforward_mode_enum_t {
    PID_FEEDFORWARD_DISABLE = 0,
    PID_FEEDFORWARD_NORMAL = 1,
    PID_FEEDFORWARD_BLEND = 2
} feedforward_mode_enum_t;

typedef struct pid_t {
    // PID Parameters
    float kp; // Proportional gain (Kp)
    float ki; // Integral gain (Ki)
    float kd; // Derivative gain (Kd)
    float dt; // Sampling time interval (seconds)

    float i_max; // integral limit
    float i_min; // integral limit

    // Integral Separation
    float i_sep; // integral separation
    float i_hysteresis;
    int last_separation_state;

    float kff; // Feedforward gain

    // Input/Output
    float setpoint; // Target setpoint
    float input; // Input value (measured)
    float output; // Output value (controlled)

    // PID Computation Terms
    float proportional; // Proportional term
    float integral; // Integral term
    float derivative; // Derivative term

    // Error Tracking
    float current_err; // Current error
    float last_err; // Previous error

    // Output Constraints
    float out_max; // Upper output limit
    float out_min; // Lower output limit
    float out_ramp; // Maximum output ramp rate

    // Output State
    float output_prev; // Previous output value
    float output_rate; // Output rate of change

    // Feature Configuration
    pid_part_enable_enum_t enable_part; // PID component enable flags
    feedforward_mode_enum_t feedforward_mode; // Feedforward mode flag
    bool enable_integral_separation; // Enable integral separation
} pid_t;


pid_t *pid_create(pid_part_enable_enum_t enable_part);
void pid_destroy(pid_t *pid);
void pid_set_part(pid_t *pid, pid_part_enable_enum_t enable_part);
void pid_set_feedforward_mode(pid_t *pid, feedforward_mode_enum_t mode, float kff);
void pid_set_i_sep_mode(pid_t *pid, bool enable);

void pid_positional_update(pid_t *pid, float input, float setpoint);
void pid_incremental_update(pid_t *pid, float input, float setpoint);

void pid_set_dt(pid_t *pid, float dt);
void pid_set_coefficient(pid_t *pid, float kp, float ki, float kd);
void pid_set_output_limit(pid_t *pid, float out_max, float out_min);
void pid_set_ramp_limit(pid_t *pid, float output_ramp);
void pid_set_integral_separation(pid_t *pid, float i_sep);
void pid_set_hysteresis(pid_t *pid, float h);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif  //PID_H
