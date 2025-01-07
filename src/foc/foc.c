/**
  ******************************************************************************
  * @file           : foc.c
  * @author         : Hotakus (hotakus@foxmail.com)
  * @brief          : None
  * @date           : 2024/10/7
  ******************************************************************************
  */

#include <math.h>
#include <memory.h>
#include <stdlib.h>
#include "foc.h"

#include "foc_fs_port.h"

#define FOC_TAG                     "FOC"
#define FOC_UNKNOWN                 "Unknown FOC object"
#define FOC_CHECK_NAME(name)        (name ? name : FOC_UNKNOWN)

#define FOC_NULL_ASSERT(x, return_value) \
do { \
    if (x == NULL) { \
        FOC_PRINTF("[%s] %s is null in func: \"%s()\".\r\n", FOC_TAG, #x, __func__); \
        return return_value; \
    } \
} while (0)

/**
 * @brief           :  FOC Clarke and Park transform for U
 * @param[in] foc   :  object
 */
void foc_park_u(foc_t *foc) {
    if (foc->trigo_calc_done == false) {
        foc_trigonometry(foc);
    }

    foc->u_d = foc->u_alpha * foc->cosine + foc->u_beta * foc->sine;
    foc->u_q = foc->u_beta * foc->cosine - foc->u_alpha * foc->sine;
}


/**
 * @brief           : FOC calibrate
 * @param[in] foc   : FOC object
 * @param[in] theta_elec : electrical angle：0
 * @param[in] ms : time for calibration
 * @param[in] div : voltage divider, default 3.0f，if 1.0f, use max smooth voltage
 * @param[in] retry : retry times, default 10
 */
foc_err_enum_t foc_zero_angle_calibration(foc_t *foc, float theta_elec, size_t ms, float div, unsigned char retry) {
#if FOC_USE_FULL_ASSERT == 1
    FOC_NULL_ASSERT(foc, FOC_ERR_NULL_PTR);
#endif

    FOC_PRINTF("[%s] ---------------- calibration start. ----------------\r\n", FOC_CHECK_NAME(foc->name));

    FOC_LOCK_ACQUIRE(foc->lock, portMAX_DELAY);

    /* some calculation */
    foc->max_voltage = foc->src_voltage / SQRT3; // calculate max voltage of ensuring smooth control
    foc->max_rpm = foc->max_voltage * foc->kv_value; // calculate max rpm in theory、

    float calibration_voltage = foc->max_voltage / (div >= 1.0f ? div : 3.0f);
    FOC_PRINTF("[%s] max smooth voltage: %.2f V\r\n", FOC_CHECK_NAME(foc->name), foc->max_voltage);
    FOC_PRINTF("[%s] max smooth RPM: %.2f RPM\r\n", FOC_CHECK_NAME(foc->name), foc->max_rpm);
    FOC_PRINTF("[%s] align zero angel after %d ms.\r\n", FOC_CHECK_NAME(foc->name), ms);
    FOC_PRINTF("[%s] calibration voltage: %.2f V\r\n", FOC_CHECK_NAME(foc->name), calibration_voltage);

    /* calibrating electrical angle */
    foc->theta_elec = theta_elec;
    foc->u_q = 0;
    foc->u_d = calibration_voltage;
    foc_inv_park(foc);
    foc_svpwm(foc);
    FOC_DELAY(ms ? ms : 1500);

    do {
        foc_get_angle(foc);
        if (foc->theta != 0) break;
    } while (--retry);
    if (retry == 0) {
        foc_pwm_pause(foc);
        FOC_PRINTF("[%s] ---------------- calibration failed (%d) ----------------\r\n", FOC_CHECK_NAME(foc->name), retry);
        while (1) {
        }
    }
    foc->init_angle = foc->theta;
    foc->zero_angle_calibrated = true;
    foc_pwm_pause(foc);

    foc->theta = 0;
    foc->theta_elec = 0;
    foc->full_rotation = 0;
    foc->velocity = 0;

    FOC_PRINTF("[%s] init angle: %f rad\r\n", FOC_CHECK_NAME(foc->name), foc->init_angle);
    FOC_PRINTF("[%s] ----------------- calibrated done. (%d)-----------------\r\n", FOC_CHECK_NAME(foc->name), retry);

    FOC_LOCK_RELEASE(foc->lock);

    return FOC_ERR_OK;
}

/**
 * @brief Current calibration
 * @param foc FOC object
 * @param calibration_times How many times to sample current
 * @return FOC_ERR_OK on success
 */
foc_err_enum_t foc_current_calibration(foc_t *foc, size_t calibration_times) {
#if FOC_USE_FULL_ASSERT == 1
    FOC_NULL_ASSERT(foc, FOC_ERR_NULL_PTR);
    FOC_NULL_ASSERT(foc->func.current_sample_get, FOC_ERR_NULL_PTR);
#endif

    FOC_PRINTF("[%s] ------------ current calibration start. ------------\r\n", FOC_CHECK_NAME(foc->name));

    float avg_a = 0.0f;
    float avg_b = 0.0f;
    float avg_c = 0.0f;
    int a, b, c;
    for (size_t i = 0; i < calibration_times; i++) {
        foc->func.current_sample_get(&a, &b, &c);
        avg_a += (float)a;
        avg_b += (float)b;
        avg_c += (float)c;
    }

    avg_a /= (float)calibration_times;
    avg_b /= (float)calibration_times;
    avg_c /= (float)calibration_times;
    float factor = foc->current_param.u_ref / (float)foc->current_param.resolution;
    foc->current_param.sensor_offset_a = avg_a * factor;
    foc->current_param.sensor_offset_b = avg_b * factor;
    foc->current_param.sensor_offset_c = avg_c * factor;

    FOC_PRINTF("[%s] A phase sampling voltage offset: %f (%d)\r\n", FOC_CHECK_NAME(foc->name), foc->current_param.sensor_offset_a, (int)avg_a);
    FOC_PRINTF("[%s] B phase sampling voltage offset: %f (%d)\r\n", FOC_CHECK_NAME(foc->name), foc->current_param.sensor_offset_b, (int)avg_b);
    FOC_PRINTF("[%s] C phase sampling voltage offset: %f (%d)\r\n", FOC_CHECK_NAME(foc->name), foc->current_param.sensor_offset_c, (int)avg_c);
    FOC_PRINTF("[%s] ------------ current calibration done. ------------\r\n", FOC_CHECK_NAME(foc->name));

    foc->current_calibrated = true;
    return FOC_ERR_OK;
}

/**
 * @brief      : Link angle sensor to FOC
 * @param[in] foc: The FOC object
 * @param[in] get_angle: The function to get the angle data from sensor
 * @param[in] resolution: The resolution of sensor
 * @return     : FOC_ERR_OK on success, FOC_ERR_NULL_PTR or FOC_ERR_INVALID_PARAM on error
 */
foc_err_enum_t foc_link_angle_sensor(foc_t *foc, void (*get_angle)(uint32_t *raw_data), uint32_t resolution) {
#if FOC_USE_FULL_ASSERT == 1
    FOC_NULL_ASSERT(foc, FOC_ERR_NULL_PTR);
    FOC_NULL_ASSERT(get_angle, FOC_ERR_NULL_PTR);
    if (resolution == 0) {
        return FOC_ERR_INVALID_PARAM;
    }
#endif
    foc->func.get_angle = get_angle;
    foc->angle_sensor_resolution = resolution;
    foc->theta_factor = TWOPI / (float)resolution;
    return FOC_ERR_OK;
}

/**
 * @brief      : Link the pwm start function to FOC
 * @param[in] foc: The FOC object
 * @param[in] pwm_start: The function to start the pwm
 * @return     : FOC_ERR_OK on success, FOC_ERR_NULL_PTR on error
 */
foc_err_enum_t foc_link_pwm_start(foc_t *foc, foc_pwm_start_t pwm_start) {
#if FOC_USE_FULL_ASSERT == 1
    FOC_NULL_ASSERT(foc, FOC_ERR_NULL_PTR);
    FOC_NULL_ASSERT(pwm_start, FOC_ERR_NULL_PTR);
#endif
    foc->func.pwm_start = pwm_start;
    return FOC_ERR_OK;
}

/**
 * @brief      : Link the pwm pause function to FOC
 * @param[in] foc: The FOC object
 * @param[in] pwm_pause: The function to pause the pwm
 * @return     : FOC_ERR_OK on success, FOC_ERR_NULL_PTR on error
 */
foc_err_enum_t foc_link_pwm_pause(foc_t *foc, foc_pwm_start_t pwm_pause) {
#if FOC_USE_FULL_ASSERT == 1
    FOC_NULL_ASSERT(foc, FOC_ERR_NULL_PTR);
    FOC_NULL_ASSERT(pwm_pause, FOC_ERR_NULL_PTR);
#endif
    foc->func.pwm_pause = pwm_pause;
    return FOC_ERR_OK;
}


/**
 * @brief      : Create a new FOC object
 * @param[in]  name  : Name of the FOC instance
 * @return     : Pointer to the created FOC object, or NULL on failure
 */
foc_t *foc_create(char *name) {
    foc_t *foc = (foc_t *)FOC_MALLOC(sizeof(foc_t));
#if FOC_USE_FULL_ASSERT == 1
    if (foc == NULL) {
        FOC_PRINTF("[%s] foc object created failed\r\n", FOC_TAG);
        return NULL;
    }
#endif

    // Use memset to initialize all float variables to 0.0f
    memset(foc, 0, sizeof(foc_t));

    // Initialize to false where applicable
    foc->trigo_calc_done = false;
    foc->pwm_is_running = false;

    foc->inited = true;
    foc->src_voltage = 0.0f;
    foc->velocity = 0.0f;

    foc->name = name;

    foc->phase_seq[0] = 0;
    foc->phase_seq[1] = 1;
    foc->phase_seq[2] = 2;

    foc->loop_mode = FOC_LOOP_SPEED;

#if FOC_USE_FREERTOS == 1
#if FOC_USE_ESP_IDF == 1
    spinlock_initialize(&foc->lock);
#else
    foc->lock_attr.name = name;
    foc->lock = osMutexNew(&foc->lock_attr);
#endif
#endif
    return foc;
}


/**
 * @brief      : Destroy a FOC object
 * @param[in] foc: the FOC object to destroy
 */
foc_err_enum_t foc_destroy(foc_t *foc) {
#if FOC_USE_FULL_ASSERT == 1
    FOC_NULL_ASSERT(foc, FOC_ERR_NULL_PTR);
#endif

#if FOC_DATA_PERSISTENCE == 1
    if (foc->snapshot.data != NULL) {
        FOC_FREE(foc->snapshot.data);
    }
#endif

    foc_lpf_deinit(foc, FOC_LPF_ALL_ENABLE);
    foc_pid_deinit(foc, FOC_PID_ALL_ENABLE);

#if FOC_USE_FREERTOS == 1
#if FOC_USE_ESP_IDF == 1
    // TODO：Implement spinlock for FreeRTOS on other platforms
#else
    osMutexDelete(foc->lock);
#endif
#endif

    FOC_FREE(foc);
    return FOC_ERR_OK;
}

void foc_set_current(foc_t *foc, float i_u, float i_v, float i_w) {
    foc->i_u = i_u;
    foc->i_v = i_v;
    foc->i_w = i_w;
}

foc_err_enum_t foc_set_pole_pairs(foc_t *foc, unsigned char pole_pairs) {
#if FOC_USE_FULL_ASSERT == 1
    FOC_NULL_ASSERT(foc, FOC_ERR_NULL_PTR);
#endif
    foc->pole_pairs = pole_pairs;
    return FOC_ERR_OK;
}

foc_err_enum_t foc_set_current_param(foc_t *foc, float u_ref, uint32_t resolution, float r_cs, uint8_t sensor_gain) {
#if FOC_USE_FULL_ASSERT == 1
    FOC_NULL_ASSERT(foc, FOC_ERR_NULL_PTR);
#endif

    foc->current_param.u_ref = u_ref ? u_ref : 3.3f;
    foc->current_param.resolution = resolution ? resolution : 4096;
    foc->current_param.r_cs = r_cs ? r_cs : 0.01f;
    foc->current_param.sensor_gain = sensor_gain ? sensor_gain : 50;

    foc->current_param.coeff = 1.0f / foc->current_param.r_cs / (float)foc->current_param.sensor_gain;
    return FOC_ERR_OK;
}

foc_err_enum_t foc_set_dt(foc_t *foc, float dt) {
#if FOC_USE_FULL_ASSERT == 1
    FOC_NULL_ASSERT(foc, FOC_ERR_NULL_PTR);
#endif
    foc->dt = dt >= 0.0f ? dt : 0.001f;
    return FOC_ERR_OK;
}


foc_err_enum_t foc_current_get(foc_t *foc) {
#if FOC_USE_FULL_ASSERT == 1
    FOC_NULL_ASSERT(foc, FOC_ERR_NULL_PTR);
    FOC_NULL_ASSERT(foc->func.current_sample_get, FOC_ERR_NULL_PTR);
#endif

    int a, b, c;
    float factor = foc->current_param.u_ref / (float)foc->current_param.resolution;
    foc->func.current_sample_get(&a, &b, &c);

    if (c == 0) {
        foc->i_u = ((float)a * factor - foc->current_param.sensor_offset_a) * foc->current_param.coeff;
        foc->i_v = ((float)b * factor - foc->current_param.sensor_offset_b) * foc->current_param.coeff;
        foc->i_w = 0;
    } else if (a == 0) {
        foc->i_v = ((float)b * factor - foc->current_param.sensor_offset_b) * foc->current_param.coeff;
        foc->i_w = ((float)c * factor - foc->current_param.sensor_offset_c) * foc->current_param.coeff;
        foc->i_u = 0;
    } else if (b == 0) {
        foc->i_u = ((float)a * factor - foc->current_param.sensor_offset_a) * foc->current_param.coeff;
        foc->i_w = ((float)c * factor - foc->current_param.sensor_offset_c) * foc->current_param.coeff;
        foc->i_v = 0;
    } else {
        foc->i_u = ((float)a * factor - foc->current_param.sensor_offset_a) * foc->current_param.coeff;
        foc->i_v = ((float)b * factor - foc->current_param.sensor_offset_b) * foc->current_param.coeff;
        foc->i_w = ((float)c * factor - foc->current_param.sensor_offset_c) * foc->current_param.coeff;
    }

    return FOC_ERR_OK;
}


void foc_set_udq(foc_t *foc, float u_d, float u_q) {
    foc->u_d = u_d;
    foc->u_q = u_q;
}


foc_err_enum_t foc_link_duty(foc_t *foc, unsigned short pwm_period, foc_duty_set_t duty_set) {
#if FOC_USE_FULL_ASSERT == 1
    FOC_NULL_ASSERT(foc, FOC_ERR_NULL_PTR);
    FOC_NULL_ASSERT(duty_set, FOC_ERR_NULL_PTR);
    if (pwm_period == 0) {
        FOC_PRINTF("[%s] pwm period is 0.\r\n", FOC_CHECK_NAME(foc->name));
        return FOC_ERR_INVALID_PARAM;
    }
#endif
    foc->pwm_period = pwm_period;
    foc->func.duty_set = duty_set;
    return FOC_ERR_OK;
}

foc_err_enum_t foc_link_current_sample(foc_t *foc, foc_current_sample_get_t current_sample_get) {
#if FOC_USE_FULL_ASSERT == 1
    FOC_NULL_ASSERT(foc, FOC_ERR_NULL_PTR);
    FOC_NULL_ASSERT(current_sample_get, FOC_ERR_NULL_PTR);
#endif
    foc->func.current_sample_get = current_sample_get;
    return FOC_ERR_OK;
}


foc_err_enum_t foc_set_src_voltage(foc_t *foc, float src_voltage) {
#if FOC_USE_FULL_ASSERT == 1
    FOC_NULL_ASSERT(foc, FOC_ERR_NULL_PTR);
#endif
    foc->src_voltage = src_voltage;
    return FOC_ERR_OK;
}

foc_err_enum_t foc_set_kv(foc_t *foc, float kv) {
#if FOC_USE_FULL_ASSERT == 1
    FOC_NULL_ASSERT(foc, FOC_ERR_NULL_PTR);
#endif
    foc->kv_value = kv;
    return FOC_ERR_OK;
}

void foc_pwm_start(foc_t *foc) {
#if FOC_USE_FULL_ASSERT == 1
    if (foc->pwm_is_running == true) {
        FOC_PRINTF("[%s] pwm already is running.\r\n", FOC_CHECK_NAME(foc->name));
        return;
    }
#endif
    foc->func.pwm_start();
    foc->pwm_is_running = true;
}

void foc_pwm_pause(foc_t *foc) {
#if FOC_USE_FULL_ASSERT == 1
    if (foc->pwm_is_running == false) {
        FOC_PRINTF("[%s] pwm already is paused.\r\n", FOC_CHECK_NAME(foc->name));
        return;
    }
#endif
    foc->func.pwm_pause();
    foc->pwm_is_running = false;
}

void FOC_ATTR foc_trigonometry(foc_t *foc) {
#if FOC_MATH_CAL_METHOD == 0 || FOC_MATH_CAL_METHOD == 1
    foc->sine = FOC_SINE(foc->theta_elec);
    foc->cosine = FOC_COSINE(foc->theta_elec);
#elif FOC_MATH_CAL_METHOD == 2 || FOC_MATH_CAL_METHOD == 3
    FOC_CALC_SINE_COSINE(foc->theta_elec, &foc->sine, &foc->cosine);
#endif
    foc->trigo_calc_done = true;
}

/**
 * @brief Inverse Park transform
 * @param foc FOC struct
 */
void FOC_ATTR foc_inv_park(foc_t *foc) {
    if (foc->trigo_calc_done == false) {
        foc_trigonometry(foc);
    }
    foc->u_alpha = foc->u_d * foc->cosine - foc->u_q * foc->sine;
    foc->u_beta = foc->u_d * foc->sine + foc->u_q * foc->cosine;
}

void FOC_ATTR foc_park(foc_t *foc) {
    if (foc->trigo_calc_done == false) {
        foc_trigonometry(foc);
    }
    foc->i_d = foc->i_alpha * foc->cosine + foc->i_beta * foc->sine;
    foc->i_q = foc->i_beta * foc->cosine - foc->i_alpha * foc->sine;
}

void FOC_ATTR foc_clarke(foc_t *foc) {
    if (!foc->i_w) {
        foc->i_alpha = foc->i_u;
        foc->i_beta = _1_SQRT3 * foc->i_u + _2_SQRT3 * foc->i_v;
    } else if (!foc->i_u) {
        float a = -foc->i_w - foc->i_v;
        foc->i_alpha = a;
        foc->i_beta = _1_SQRT3 * a + _2_SQRT3 * foc->i_v;
    } else if (!foc->i_v) {
        float b = -foc->i_u - foc->i_w;
        foc->i_alpha = foc->i_u;
        foc->i_beta = _1_SQRT3 * foc->i_u + _2_SQRT3 * b;
    } else {
        float mid = (1.f / 3) * (foc->i_u + foc->i_v + foc->i_w);
        float a = foc->i_u - mid;
        float b = foc->i_v - mid;
        foc->i_alpha = a;
        foc->i_beta = _1_SQRT3 * a + _2_SQRT3 * b;
    }
}


/**
 * @brief      : Space vector PWM
 * @param[in] foc: The FOC object
 */
void FOC_ATTR foc_svpwm(foc_t *foc) {
    const float ts = 1.0f;
    const float k = SQRT3 * ts / foc->src_voltage;
    const float u1 = foc->u_beta;
    const float u2 = -SQRT3_2 * foc->u_alpha - foc->u_beta * 0.5;
    const float u3 = SQRT3_2 * foc->u_alpha - foc->u_beta * 0.5;
    const unsigned char sector = ((u1 > 0) << 0) | ((u2 > 0) << 1) | ((u3 > 0) << 2);

    switch (sector) {
        case 5: {
            // 101: sector 1
            float t4 = u3 * k;
            float t6 = u1 * k;
            float sum = t4 + t6;
            if (sum > ts) {
                float k_svpwm = ts / sum;
                t4 = t4 * k_svpwm;
                t6 = t6 * k_svpwm;
            }
            float t7 = (ts - t4 - t6) * 0.5f;
            foc->u_phase[0] = t4 + t6 + t7;
            foc->u_phase[1] = t6 + t7;
            foc->u_phase[2] = t7;
            break;
        }
        case 1: {
            // 001: sector 2
            float t2 = -u3 * k;
            float t6 = -u2 * k;
            float sum = t2 + t6;
            if (sum > ts) {
                float k_svpwm = ts / sum;
                t2 = t2 * k_svpwm;
                t6 = t6 * k_svpwm;
            }
            float t7 = (ts - t2 - t6) * 0.5f;
            foc->u_phase[0] = t6 + t7;
            foc->u_phase[1] = t2 + t6 + t7;
            foc->u_phase[2] = t7;
            break;
        }
        case 3: {
            // 011: sector 3
            float t2 = u1 * k;
            float t3 = u2 * k;
            float sum = t2 + t3;
            if (sum > ts) {
                float k_svpwm = ts / sum;
                t2 = t2 * k_svpwm;
                t3 = t3 * k_svpwm;
            }
            float t7 = (ts - t2 - t3) * 0.5f;
            foc->u_phase[0] = t7;
            foc->u_phase[1] = t2 + t3 + t7;
            foc->u_phase[2] = t3 + t7;
            break;
        }
        case 2: {
            // 010: sector 4
            float t1 = -u1 * k;
            float t3 = -u3 * k;
            float sum = t1 + t3;
            if (sum > ts) {
                float k_svpwm = ts / sum;
                t1 = t1 * k_svpwm;
                t3 = t3 * k_svpwm;
            }
            float t7 = (ts - t1 - t3) * 0.5f;
            foc->u_phase[0] = t7;
            foc->u_phase[1] = t3 + t7;
            foc->u_phase[2] = t1 + t3 + t7;
            break;
        }
        case 6: {
            // 110: sector 5
            float t1 = u2 * k;
            float t5 = u3 * k;
            float sum = t1 + t5;
            if (sum > ts) {
                float k_svpwm = ts / sum;
                t1 = t1 * k_svpwm;
                t5 = t5 * k_svpwm;
            }
            float t7 = (ts - t1 - t5) * 0.5f;
            foc->u_phase[0] = t5 + t7;
            foc->u_phase[1] = t7;
            foc->u_phase[2] = t1 + t5 + t7;
            break;
        }
        case 4: {
            // 100: sector 6
            float t4 = -u2 * k;
            float t5 = -u1 * k;
            float sum = t4 + t5;
            if (sum > ts) {
                float k_svpwm = ts / sum;
                t4 = t4 * k_svpwm;
                t5 = t5 * k_svpwm;
            }
            float t7 = (ts - t4 - t5) * 0.5f;
            foc->u_phase[0] = t4 + t5 + t7;
            foc->u_phase[1] = t7;
            foc->u_phase[2] = t5 + t7;
            break;
        }
        default: {
            break;
        }
    }

    if (foc->pwm_is_running == false) {
        foc_pwm_start(foc);
    }

    foc->func.duty_set((uint16_t)(foc->u_phase[foc->phase_seq[0]] * (float)foc->pwm_period),
                       (uint16_t)(foc->u_phase[foc->phase_seq[1]] * (float)foc->pwm_period),
                       (uint16_t)(foc->u_phase[foc->phase_seq[2]] * (float)foc->pwm_period));

    foc->trigo_calc_done = false; // recalculate trigonometry
}

/**
 * @brief Set the PWM duty cycles for the FOC object
 * @param foc The FOC object
 * @param t_u Duty cycle for phase U
 * @param t_v Duty cycle for phase V
 * @param t_w Duty cycle for phase W
 */
void FOC_ATTR foc_pwm_set_duty(foc_t *foc, uint16_t t_u, uint16_t t_v, uint16_t t_w) {
    foc->func.duty_set(t_u, t_v, t_w);
}

/**
 * @brief      : Normalize electrical angle to [-PI, PI]
 * @param[in] rad: electrical angle
 * @return     : normalized electrical angle
 */
float FOC_ATTR foc_normalize_angle(float rad) {
    rad = rad - TWOPI * floorf(rad / TWOPI);
    if (rad < -PI) {
        rad += TWOPI;
    } else if (rad > PI) {
        rad -= TWOPI;
    }
    return rad;
}

/**
 * @brief      : Calculate the motor torque, Torque = (8.27 * Iq) / Kv
 * @param[in] foc: The FOC object
 * @return     : Motor torque （unit: N.M）
 */
foc_err_enum_t foc_torque_calc(foc_t *foc) {
#if FOC_USE_FULL_ASSERT == 1
    FOC_NULL_ASSERT(foc, FOC_ERR_NULL_PTR);
    if (foc->kv_value <= 0) {
        FOC_PRINTF("[%s] Kv value: %f is invalid.\r\n", FOC_CHECK_NAME(foc->name), foc->kv_value);
        return FOC_ERR_INVALID_PARAM;
    }
#endif
    foc->torque = (TORQUE_CONSTANT * foc->i_q) / foc->kv_value;
    return FOC_ERR_OK;
}

/**
 * @brief      : Initialize the FOC object's low-pass filters
 * @param[in] foc: The FOC object
 * @param[in] enable: The FOC object's low-pass filter enable mask
 * @return     : FOC_ERR_OK on success, FOC_ERR_NULL_PTR on error
 */
foc_err_enum_t foc_lpf_init(foc_t *foc, foc_lpf_enable_t enable) {
#if FOC_USE_FULL_ASSERT == 1
    FOC_NULL_ASSERT(foc, FOC_ERR_NULL_PTR);
#endif
    for (uint8_t i = 0x01, k = 0; i < (FOC_LPF_ALL_ENABLE & 0x0F); i <<= 1, k++) {
        if (enable & i) {
            if (foc->lpf_all[k] != NULL) {
                continue;
            }
            foc->lpf_all[k] = foc_lpf_create(FOC_LPF_DEFAULT_K);
        }
    }
    return FOC_ERR_OK;
}

/**
 * @brief      : Deinitialize the FOC object's low-pass filters
 * @param[in] foc: The FOC object
 * @param[in] enable: The FOC object's low-pass filter enable mask
 * @return     : FOC_ERR_OK on success, FOC_ERR_NULL_PTR on error
 */
foc_err_enum_t foc_lpf_deinit(foc_t *foc, foc_lpf_enable_t enable) {
#if FOC_USE_FULL_ASSERT == 1
    FOC_NULL_ASSERT(foc, FOC_ERR_NULL_PTR);
#endif
    for (uint8_t i = 0x01, k = 0; i < (FOC_LPF_ALL_ENABLE & 0x0F); i <<= 1, k++) {
        if (enable & i) {
            if (foc->lpf_all[k] == NULL) {
                continue;
            }
            foc_lpf_destroy(foc->lpf_all[k]);
            foc->lpf_all[k] = NULL;
        }
    }
    return FOC_ERR_OK;
}

/**
 * @brief      : Set the low-pass filter coefficient for velocity
 * @param[in] foc: The FOC object
 * @param[in] k: The low-pass filter coefficient
 * @return     : FOC_ERR_OK on success, FOC_ERR_NULL_PTR on error
 */
foc_err_enum_t foc_lpf_vel_set_k(foc_t *foc, float k) {
#if FOC_USE_FULL_ASSERT == 1
    FOC_NULL_ASSERT(foc, FOC_ERR_NULL_PTR);
    FOC_NULL_ASSERT(foc->lpf.vel, FOC_ERR_NULL_PTR);
#endif
    foc->lpf.vel->k = k;
    return FOC_ERR_OK;
}

/**
 * @brief      : Set the low-pass filter coefficient for q axis current
 * @param[in] foc: The FOC object
 * @param[in] k: The low-pass filter coefficient
 * @return     : FOC_ERR_OK on success, FOC_ERR_NULL_PTR on error
 */
foc_err_enum_t foc_lpf_cur_q_set_k(foc_t *foc, float k) {
#if FOC_USE_FULL_ASSERT == 1
    FOC_NULL_ASSERT(foc, FOC_ERR_NULL_PTR);
    FOC_NULL_ASSERT(foc->lpf.cur_q, FOC_ERR_NULL_PTR);
#endif
    foc->lpf.cur_q->k = k;
    return FOC_ERR_OK;
}

/**
 * @brief      : Set the low-pass filter coefficient for d axis current
 * @param[in] foc: The FOC object
 * @param[in] k: The low-pass filter coefficient
 * @return     : FOC_ERR_OK on success, FOC_ERR_NULL_PTR on error
 */
foc_err_enum_t foc_lpf_cur_d_set_k(foc_t *foc, float k) {
#if FOC_USE_FULL_ASSERT == 1
    FOC_NULL_ASSERT(foc, FOC_ERR_NULL_PTR);
    FOC_NULL_ASSERT(foc->lpf.cur_d, FOC_ERR_NULL_PTR);
#endif
    foc->lpf.cur_d->k = k;
    return FOC_ERR_OK;
}

foc_lpf_t *foc_lpf_vel(const foc_t *foc) {
    return foc->lpf.vel;
}

foc_lpf_t *foc_lpf_cur_q(const foc_t *foc) {
    return foc->lpf.cur_q;
}

foc_lpf_t *foc_lpf_cur_d(const foc_t *foc) {
    return foc->lpf.cur_d;
}

/**
 * @brief      : Create a new low-pass filter object
 * @param[in] k: The low-pass filter coefficient
 * @return     : Pointer to the created low-pass filter object, or NULL on failure
 */
foc_lpf_t *foc_lpf_create(float k) {
    foc_lpf_t *lpf = (foc_lpf_t *)FOC_MALLOC(sizeof(foc_lpf_t));
#if FOC_USE_FULL_ASSERT == 1
    if (lpf == NULL) {
        FOC_PRINTF("[%s] lpf malloc failed.\r\n", FOC_TAG);
        return NULL;
    }
#endif
    lpf->k = k;
    lpf->y = 0.0f;
    lpf->y_prev = 0.0f;
    return lpf;
}

/**
 * @brief      : Destroy a low-pass filter object
 * @param[in] lpf: The low-pass filter object to destroy
 * @return     : FOC_ERR_OK on success, FOC_ERR_NULL_PTR on error
 */
foc_err_enum_t foc_lpf_destroy(foc_lpf_t *lpf) {
#if FOC_USE_FULL_ASSERT == 1
    if (lpf == NULL) {
        FOC_PRINTF("[%s] lpf is null.\r\n", FOC_UNKNOWN);
        return FOC_ERR_NULL_PTR;
    }
#endif
    FOC_FREE(lpf);
    return FOC_ERR_OK;
}

/**
 * @brief      : Calculate the low-pass filter output
 * @param[in] lpf: The low-pass filter object
 * @param[in] x: The input value
 * @return     : The low-pass filter output value
 */
float FOC_ATTR foc_lpf_calc(foc_lpf_t *lpf, float x) {
    lpf->y = lpf->k * x + (float)(1.0f - lpf->k) * lpf->y_prev;
    lpf->y_prev = lpf->y;
    return lpf->y;
}

/**
 * @brief      : Calculate the velocity low-pass filter output
 * @param[in] foc: The FOC object
 */
void foc_lpf_vel_calc(foc_t *foc) {
    foc->velocity = foc_lpf_calc(foc->lpf.vel, foc->velocity);
}

/**
 * @brief      : Calculate the low-pass filter output for q axis current
 * @param[in] foc: The FOC object
 */
void foc_lpf_cur_q_calc(foc_t *foc) {
    foc->i_q = foc_lpf_calc(foc->lpf.cur_q, foc->i_q);
}

/**
 * @brief      : Calculate the low-pass filter output for d axis current
 * @param[in] foc: The FOC object
 */
void foc_lpf_cur_d_calc(foc_t *foc) {
    foc->i_d = foc_lpf_calc(foc->lpf.cur_d, foc->i_d);
}

/**
 * @brief      : Initialize the FOC object's PID controllers
 * @param[in] foc: The FOC object
 * @param[in] enable: The FOC object's PID controller enable mask
 * @return     : FOC_ERR_OK on success, FOC_ERR_NULL_PTR on error
 */
foc_err_enum_t foc_pid_init(foc_t *foc, foc_pid_enable_t enable) {
#if FOC_USE_FULL_ASSERT == 1
    FOC_NULL_ASSERT(foc, FOC_ERR_NULL_PTR);
#endif
    for (uint8_t i = 0x01, k = 0; i < (FOC_PID_ALL_ENABLE & 0x0F); i <<= 1, k++) {
        if (enable & i) {
            if (foc->pid_all[k] != NULL) {
                continue;
            }
            foc->pid_all[k] = pid_create(PID_PART_ALL);
        }
    }
    return FOC_ERR_OK;
}

/**
 * @brief      : Deinitialize the FOC object's PID controllers
 * @param[in] foc: The FOC object
 * @param[in] enable: The FOC object's PID controller enable mask
 * @return     : FOC_ERR_OK on success, FOC_ERR_NULL_PTR on error
 */
foc_err_enum_t foc_pid_deinit(foc_t *foc, foc_pid_enable_t enable) {
#if FOC_USE_FULL_ASSERT == 1
    FOC_NULL_ASSERT(foc, FOC_ERR_NULL_PTR);
#endif
    for (uint8_t i = 0x01, k = 0; i < (FOC_PID_ALL_ENABLE & 0x0F); i <<= 1, k++) {
        if (enable & i) {
            if (foc->pid_all[k] == NULL) {
                continue;
            }
            pid_destroy(foc->pid_all[k]);
            foc->pid_all[k] = NULL;
        }
    }
    return FOC_ERR_OK;
}

/**
 * @brief      : Set the PID coefficients for velocity control
 * @param[in] foc: The FOC object
 * @param[in] kp: Proportional coefficient
 * @param[in] ki: Integral coefficient
 * @param[in] kd: Derivative coefficient
 * @return     : FOC_ERR_OK on success, FOC_ERR_NULL_PTR on error
 */
foc_err_enum_t foc_pid_vel_set_k(foc_t *foc, float kp, float ki, float kd) {
#if FOC_USE_FULL_ASSERT == 1
    FOC_NULL_ASSERT(foc, FOC_ERR_NULL_PTR);
    FOC_NULL_ASSERT(foc->pid.vel, FOC_ERR_NULL_PTR);
#endif
    foc->pid.vel->kp = kp;
    foc->pid.vel->ki = ki;
    foc->pid.vel->kd = kd;
    return FOC_ERR_OK;
}

/**
 * @brief      : Set the PID coefficients for current control for q axis
 * @param[in] foc: The FOC object
 * @param[in] kp: Proportional coefficient
 * @param[in] ki: Integral coefficient
 * @param[in] kd: Derivative coefficient
 * @return     : FOC_ERR_OK on success, FOC_ERR_NULL_PTR on error
 */
foc_err_enum_t foc_pid_cur_q_set_k(foc_t *foc, float kp, float ki, float kd) {
#if FOC_USE_FULL_ASSERT == 1
    FOC_NULL_ASSERT(foc, FOC_ERR_NULL_PTR);
    FOC_NULL_ASSERT(foc->pid.cur_q, FOC_ERR_NULL_PTR);
#endif
    foc->pid.cur_q->kp = kp;
    foc->pid.cur_q->ki = ki;
    foc->pid.cur_q->kd = kd;
    return FOC_ERR_OK;
}

/**
 * @brief      : Set the PID coefficients for current control for d axis
 * @param[in] foc: The FOC object
 * @param[in] kp: Proportional coefficient
 * @param[in] ki: Integral coefficient
 * @param[in] kd: Derivative coefficient
 * @return     : FOC_ERR_OK on success, FOC_ERR_NULL_PTR on error
 */
foc_err_enum_t foc_pid_cur_d_set_k(foc_t *foc, float kp, float ki, float kd) {
#if FOC_USE_FULL_ASSERT == 1
    FOC_NULL_ASSERT(foc, FOC_ERR_NULL_PTR);
    FOC_NULL_ASSERT(foc->pid.cur_d, FOC_ERR_NULL_PTR);
#endif
    foc->pid.cur_d->kp = kp;
    foc->pid.cur_d->ki = ki;
    foc->pid.cur_d->kd = kd;
    return FOC_ERR_OK;
}

/**
 * @brief      : Get the velocity PID controller
 * @param[in] foc: The FOC object
 * @return     : Pointer to the velocity PID controller
 */
pid_t *foc_pid_vel(const foc_t *foc) {
    return foc->pid.vel;
}

/**
 * @brief      : Get the current PI controller for q axis
 * @param[in] foc: The FOC object
 * @return     : Pointer to the current PI controller for q axis
 */
pid_t *foc_pid_cur_q(const foc_t *foc) {
    return foc->pid.cur_q;
}

/**
 * @brief      : Get the current PI controller for d axis
 * @param[in] foc: The FOC object
 * @return     : Pointer to the current PI controller for d axis
 */
pid_t *foc_pid_cur_d(const foc_t *foc) {
    return foc->pid.cur_d;
}

/**
 * @brief      : Get the current angle from angle sensor to FOC object
 * @param[in] foc: The FOC object
 */
void FOC_ATTR foc_get_angle(foc_t *foc) {
    uint32_t raw_data = 0;
    foc->func.get_angle(&raw_data);
    foc->theta = (float)raw_data * foc->theta_factor;

    // calculate full rotation
    float d_angle = foc->theta - foc->theta_prev;
    if (fabsf(d_angle) > (0.8 * TWOPI)) {
        foc->full_rotation += d_angle > 0 ? -TWOPI : TWOPI;
    }

    // calculate electrical angle
    foc->theta_elec = foc_normalize_angle((foc->theta - foc->init_angle) * (float)foc->pole_pairs);
}

/**
 * @brief      : Get the motor velocity
 * @param[in] foc: The FOC object
 * @param[in] dt: The sampling time in seconds
 * @return     : The motor velocity in rad/s
 */
float FOC_ATTR foc_get_velocity(foc_t *foc) {
    foc->velocity = ((foc->full_rotation - foc->full_rotation_prev) + (foc->theta - foc->theta_prev)) / foc->dt;
    foc->full_rotation_prev = foc->full_rotation;
    foc->theta_prev = foc->theta;
    return foc->velocity;
}


/**
 * @brief      : Open loop test
 * @param[in] foc: The FOC object
 * @param[in] u_q: The q axis voltage
 * @param[in] u_d: The d axis voltage
 * @param[in] sustain_ms: The testing time in milliseconds
 * @return     : The error code
 */
foc_err_enum_t foc_openloop_test(foc_t *foc, float u_q, float u_d, size_t sustain_ms) {
#if FOC_USE_FULL_ASSERT == 1
    FOC_NULL_ASSERT(foc, FOC_ERR_NULL_PTR);
    if (u_q == 0.0f && u_d == 0.0f) {
        return FOC_ERR_INVALID_PARAM;
    }
    if (sustain_ms == 0) {
        return FOC_ERR_OK;
    }
#endif

    for (size_t i = 0; i < sustain_ms; i++) {
        foc_get_angle(foc);
        foc_get_velocity(foc);
        foc_set_udq(foc, u_d, u_q);
        foc_inv_park(foc);
        foc_svpwm(foc);
        FOC_DELAY(1);
    }
    foc_pwm_pause(foc);

    return FOC_ERR_OK;
}

/**
 * @brief      : Check the motor direction
 * @param[in] foc: The FOC object
 * @return     : The error code
 */
foc_err_enum_t foc_check_dir(foc_t *foc) {
#if FOC_USE_FULL_ASSERT == 1
    FOC_NULL_ASSERT(foc, FOC_ERR_NULL_PTR);
    if (foc->zero_angle_calibrated == false) {
        FOC_PRINTF("[%s] foc is not calibrated.\r\n", FOC_CHECK_NAME(foc->name));
    }
#endif

    if ((int)foc->velocity > 0) {
        foc->direction = 1;
    } else {
        foc->direction = 0;
    }

    return FOC_ERR_OK;
}

static void swap(uint8_t *x, uint8_t *y) {
    uint8_t temp = *x;
    *x = *y;
    *y = temp;
};

/**
 * @brief      : Check the phase sequence of the motor
 * @param[in] foc: The FOC object
 * @param[in] u_q: The q axis voltage
 * @param[in] full_rotation_threshold: The full rotation threshold in Mechanical angle
 * @param[in] sustain_ms: The testing time in milliseconds
 * @return     : The error code
 */
foc_err_enum_t foc_check_phase_seq(foc_t *foc, float u_q, float full_rotation_threshold, size_t sustain_ms) {
#if FOC_USE_FULL_ASSERT == 1
    FOC_NULL_ASSERT(foc, FOC_ERR_NULL_PTR);
    if (u_q == 0.0f) {
        return FOC_ERR_INVALID_PARAM;
    }
    if (sustain_ms == 0) {
        return FOC_ERR_OK;
    }
#endif

    if (foc->zero_angle_calibrated == false) {
        foc_err_enum_t err = foc_zero_angle_calibration(foc, 0, sustain_ms, 3.5f, 10);
        if (err != FOC_ERR_OK) {
            FOC_PRINTF("[%s] zero angle calibration error.\r\n", FOC_CHECK_NAME(foc->name));
            FOC_PRINTF("[%s] phase sequence error.\r\n", FOC_CHECK_NAME(foc->name));
            return err;
        }
    }

    uint8_t retry = 2;
    do {
        foc_openloop_test(foc, u_q, 0, sustain_ms);
        foc_check_dir(foc);
        if (foc->direction != 1 || foc->full_rotation < (full_rotation_threshold)) {
            FOC_PRINTF("[%s] phase sequence (UVW: %d%d%d) error.\r\n", FOC_CHECK_NAME(foc->name),
                       foc->phase_seq[0], foc->phase_seq[1], foc->phase_seq[2]);
            swap(&foc->phase_seq[0], &foc->phase_seq[1]);
        } else {
            break;
        }
    } while (--retry);
    if (retry == 0) {
        FOC_PRINTF("[%s] phase sequence error, please adjust the phase sequence(%d).\r\n", FOC_CHECK_NAME(foc->name), retry);
        foc->phase_calibrated = false;
        foc->zero_angle_calibrated = false;
        return FOC_ERR_FAILED;
    }
    FOC_PRINTF("[%s] phase sequence (UVW: %d%d%d) is ok.\r\n", FOC_CHECK_NAME(foc->name),
               foc->phase_seq[0], foc->phase_seq[1], foc->phase_seq[2]);
    foc->phase_calibrated = true;
    return FOC_ERR_OK;
}

/**
 * @brief      : Set the loop mode of the FOC object
 * @param[in] foc: The FOC object
 * @param[in] mode: The loop mode
 * @return     : The error code
 */
foc_err_enum_t foc_set_loop_mode(foc_t *foc, foc_loop_enum_t mode) {
#if FOC_USE_FULL_ASSERT == 1
    FOC_NULL_ASSERT(foc, FOC_ERR_NULL_PTR);
    if (mode > FOC_LOOP_ALL) {
        return FOC_ERR_INVALID_PARAM;
    }
#endif
    foc->loop_mode = mode;
    return FOC_ERR_OK;
}

/**
 * @brief      : The main task of the FOC object
 * @param[in] foc: The FOC object
 */
void foc_task(foc_t *foc) {
    // angle and velocity
    if (foc->func.get_angle == NULL) {
        foc->theta += 1 * DEG2RAD; // simulating angle auto increment
        foc->theta_elec = foc_normalize_angle((foc->theta - foc->init_angle) * (float)foc->pole_pairs);
    } else {
        foc_get_angle(foc);
    }
    foc_get_velocity(foc);
    if (foc->lpf.vel != NULL) {
        foc_lpf_vel_calc(foc);
    }

    // current sampling and calculation
    if ((foc->func.current_sample_get != NULL) && (foc->loop_mode & FOC_LOOP_CURRENT)) {
        foc_current_get(foc);
        foc_clarke(foc);
        foc_park(foc);
        if (foc->lpf.cur_q != NULL) {
            foc_lpf_cur_q_calc(foc);
        }
    }

    // PID control
    float output = 0;
    if (foc->loop_mode & FOC_LOOP_POSITION) {
        pid_positional_update(foc->pid.pos, foc->theta + foc->full_rotation, foc->pos_setpoint);
        foc->vel_setpoint = foc->pid.pos->output;
        output = foc->pid.pos->output;
    }
    if (foc->loop_mode & FOC_LOOP_SPEED) {
        pid_positional_update(foc->pid.vel, foc->velocity, foc->vel_setpoint);
        foc->cur_q_setpoint = foc->pid.vel->output;
        output = foc->pid.vel->output;
    }
    if (foc->loop_mode & FOC_LOOP_CURRENT) {
        pid_positional_update(foc->pid.cur_q, foc->i_q, foc->cur_q_setpoint);
        output = foc->pid.cur_q->output;
    }

    // motor control
    foc_set_udq(foc, 0, output);
    foc_inv_park(foc);
    foc_svpwm(foc);
}

#if FOC_DATA_PERSISTENCE == 1
/**
 * @brief      : Create a snapshot of the current FOC
 * @param[in] foc: The FOC object
 * @return     : The error code
 */
foc_err_enum_t foc_snapshot_create(foc_t *foc) {
#if FOC_USE_FULL_ASSERT == 1
    FOC_NULL_ASSERT(foc, FOC_ERR_NULL_PTR);
#endif

    FOC_LOCK_ACQUIRE(foc->lock, portMAX_DELAY);

    if (foc->snapshot.data == NULL) {
        foc->snapshot.data = (uint8_t *)FOC_MALLOC(sizeof(foc_t));
        FOC_NULL_ASSERT(foc->snapshot.data, FOC_ERR_NULL_PTR);
    }
    memset(foc->snapshot.data, 0, sizeof(foc_t));
    memcpy(foc->snapshot.data, foc, sizeof(foc_t));

    foc_t *snapshot = (foc_t *)foc->snapshot.data;
    memset(snapshot->pid_all, 0, sizeof(snapshot->pid_all));
    memset(snapshot->lpf_all, 0, sizeof(snapshot->lpf_all));
    memset(&snapshot->func, 0, sizeof(snapshot->func));
    snapshot->snapshot.data = NULL;

    foc->snapshot.version += 1;

    FOC_LOCK_RELEASE(foc->lock);

    return FOC_ERR_OK;
}

/**
 * @brief      : Save the current FOC snapshot to a file
 * @param[in] foc_src: The source FOC object
 * @param[in] save_path: The path to save the snapshot
 * @return     : The error code
 */
foc_err_enum_t foc_save_snapshot(const foc_t *foc_src, const char *save_path) {
#if FOC_USE_FULL_ASSERT == 1
    FOC_NULL_ASSERT(foc_src, FOC_ERR_NULL_PTR);
    FOC_NULL_ASSERT(save_path, FOC_ERR_NULL_PTR);
    FOC_NULL_ASSERT(foc_src->snapshot.data, FOC_ERR_NULL_PTR);
#endif

    FOC_LOCK_ACQUIRE(foc_src->lock, portMAX_DELAY);

    void *file = foc_fs_port_open(save_path, FOC_FS_MODE_READ);
    FOC_NULL_ASSERT(file, FOC_ERR_NULL_PTR);

    foc_err_enum_t err = FOC_ERR_OK;
    uint8_t retry = 3;
    do {
        err = foc_fs_port_write(file, (uint8_t *)foc_src->snapshot.data, sizeof(foc_t));
        if (err == FOC_ERR_OK) {
            break;
        }
    } while (--retry);

    foc_fs_port_close(file);
    FOC_LOCK_RELEASE(foc_src->lock);
    return err;
}

/**
 * @brief      : Load a saved snapshot of the FOC object
 * @param[in]  foc_dst: The destination FOC object
 * @param[in]  load_path: The path to load the snapshot
 * @return     : The error code
 */
foc_err_enum_t foc_load_snapshot(foc_t *foc_dst, const char *load_path) {
#if FOC_USE_FULL_ASSERT == 1
    FOC_NULL_ASSERT(foc_dst, FOC_ERR_NULL_PTR);
    FOC_NULL_ASSERT(load_path, FOC_ERR_NULL_PTR);
#endif

    FOC_LOCK_ACQUIRE( foc_dst->lock, portMAX_DELAY);

    void *file = foc_fs_port_open(load_path, FOC_FS_MODE_READ);
    FOC_NULL_ASSERT(file, FOC_ERR_NULL_PTR);

    foc_err_enum_t err = FOC_ERR_OK;
    uint8_t retry = 3;
    do {
        err = foc_fs_port_read(file, (uint8_t *)foc_dst->snapshot.data, sizeof(foc_t));
        if (err == FOC_ERR_OK) {
            break;
        }
    } while (--retry);

    foc_fs_port_close(file);
    FOC_LOCK_RELEASE(foc_dst->lock);
    return err;
}

void foc_fs_init(void) {
    foc_fs_port_init();
}

void foc_fs_deinit(void) {
    foc_fs_port_deinit();
}

#endif

#if FOC_MATH_CAL_METHOD == 2
/* Define Q31 */
#define Q31 0x80000000
/* Define Q31 to float unit in RADIAN = Q31/PI */
#define RADIAN_Q31_f 683565275.6f

void cordic_calc_sin_cos_f32(float theta, float *sin_out, float *cos_out) {
    /* Q31,two write, two read, sine calculate, 6 precision */
    CORDIC->CSR = 0x00180061; // 0000 0000 0001 1000 0000 0000 0110 0001
    /* Write data into WDATA */
    CORDIC->WDATA = (int32_t)(theta * RADIAN_Q31_f);
    /* Modulus is m=1 */
    CORDIC->WDATA = 0x7FFFFFFF;

    /* Get sin value in float */
    *sin_out = ((int32_t)CORDIC->RDATA) * 1.0f / Q31;
    /* Get cos value in float */
    *cos_out = ((int32_t)CORDIC->RDATA) * 1.0f / Q31;
}

void cordic_calc_sin_cos_f16(float theta, float *sin_out, float *cos_out) {
    // TODO：Config CORDIC

    unsigned int cordicin = 0x7fff0000; //  mag = 1
    short thetashort = theta * 10430; // wrap it
    cordicin += thetashort;
    CORDIC->WDATA = cordicin;

    unsigned int out0 = CORDIC->RDATA;
    short out2 = (out0 & 0xffff0000) >> 16;
    short out1 = out0 & 0xffff; //
    *cos_out = (float)out1 / 32768.0f;
    *sin_out = (float)out2 / 32768.0f;
}
#endif
