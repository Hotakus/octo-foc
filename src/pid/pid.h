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

typedef enum pid_part_enable_enum_t pid_part_enable_enum_t;

typedef enum pid_part_enable_enum_t {
    PID_PART_PROPORTIONAL = 0x01,
    PID_PART_INTEGRAL = 0x02,
    PID_PART_DERIVATIVE = 0x04,
    PID_PART_ALL = PID_PART_PROPORTIONAL | PID_PART_INTEGRAL | PID_PART_DERIVATIVE
} pid_part_enable_enum_t;

typedef struct pid_t{
    // PID参数
    float kp;      // 比例系数
    float ki;      // 积分系数
    float kd;      // 微分系数
    float dt;      // 采样间隔时间 (单位: 秒)

    // 输出限制
    float out_max;          // 输出最大值
    float out_min;          // 输出最小值
    float out_ramp;      // 输出变化率最大值

    // PID计算部分
    float proportional;   // 比例项
    float integral;       // 积分项
    float derivative;     // 微分项

    // 误差信息
    float current_err;    // 当前误差
    float last_err;       // 上一次误差

    float setpoint;       // 目标设定值
    float input;          // 实际输入值
    float output;         // 控制输出值
    float output_prev;    // 上一次输出
    float output_rate;    // 输出变化率

    // 部分功能使能标志
    pid_part_enable_enum_t enable_part; // PID功能模块使能标志
} pid_t;


pid_t * pid_create(pid_part_enable_enum_t enable_part);
void pid_destroy(pid_t *pid);
void pid_set_part(pid_t *pid, pid_part_enable_enum_t enable_part);

void pid_positional_update(pid_t *pid, float input, float setpoint);
void pid_incremental_update(pid_t *pid, float input, float setpoint);

void pid_set_dt(pid_t *pid, float dt);
void pid_set_coefficient(pid_t *pid, float kp, float ki, float kd);
void pid_set_output_limit(pid_t *pid, float out_max, float out_min);
void pid_set_ramp_limit(pid_t *pid, float output_ramp);

#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif // __cplusplus

#endif  //PID_H
