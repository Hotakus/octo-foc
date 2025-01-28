/**
  ******************************************************************************
  * @file           : pid.h
  * @author         : Hotakus (hotakus@foxmail.com)
  * @brief          : None
  * @date           : 2024/10/10
  ******************************************************************************
  */

#ifndef PID_H
#define PID_H

#include "foc_config.h"

#define PID_MALLOC(size)      (FOC_MALLOC(size))
#define PID_FREE(ptr)         (FOC_FREE(ptr))

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

typedef struct pid_t {
    // PID Parameters
    float kp; // Proportional gain (Kp)
    float ki; // Integral gain (Ki)
    float kd; // Derivative gain (Kd)
    float dt; // Sampling time interval (seconds)

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
} pid_t;


pid_t *pid_create(pid_part_enable_enum_t enable_part);
void pid_destroy(pid_t *pid);
void pid_set_part(pid_t *pid, pid_part_enable_enum_t enable_part);

void pid_positional_update(pid_t *pid, float input, float setpoint);
void pid_incremental_update(pid_t *pid, float input, float setpoint);

void pid_set_dt(pid_t *pid, float dt);
void pid_set_coefficient(pid_t *pid, float kp, float ki, float kd);
void pid_set_output_limit(pid_t *pid, float out_max, float out_min);
void pid_set_ramp_limit(pid_t *pid, float output_ramp);


#ifdef __cplusplus
}
#endif // __cplusplus

#endif  //PID_H
