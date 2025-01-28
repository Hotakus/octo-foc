/**
  ******************************************************************************
  * @file           : foc.h
  * @author         : Hotakus (hotakus@foxmail.com)
  * @brief          : None
  * @date           : 2024/10/7
  ******************************************************************************
  */

#ifndef FOC_H
#define FOC_H

#include "foc_config.h"
#include "pid.h"


#ifdef __cplusplus
extern "C" {
#endif

/* --------------------- Some Constants --------------------- */
#define _1_SQRT3    (0.57735026918962576450914878050196f)
#define _2_SQRT3    (1.1547005383792515290182975610039f)
#define SQRT3       (1.7320508075688772935274463415059f)
#define SQRT3_3     (1.7320508075688772935274463415059f / 3.0f)
#define SQRT3_2     (1.7320508075688772935274463415059f / 2.0f)
#define TWOPI       6.2831853071795864769252867665590f
#define PI          3.1415926535897932384626433832795f

#define TORQUE_CONSTANT     (15 * SQRT3 / PI)   // Torque constant

#define DEG2RAD 0.01745329252f
#define RAD2DEG 57.295779513f

#define FOC_LPF_DEFAULT_K           (0.1f)
#define FOC_DEFAULT_VEL_SETPOINT    (TWOPI * 3)         // 3 revolutions per second
#define FOC_DEFAULT_POS_SETPOINT    (120.0f * DEG2RAD)  // 120 degree convert to radian
#define FOC_DEFAULT_CUR_Q_SETPOINT  (0.1f)              // 0.1A
#define FOC_DEFAULT_PHASE_SEQ       (0u)                // default phase sequence

/* --------------------- FOC_MATH_CAL_METHOD --------------------- */
#if FOC_MATH_CAL_METHOD == 0
#include <math.h>
#define FOC_SINE(x)     ((sinf(x)))
#define FOC_COSINE(x)   ((cosf(x)))
#elif FOC_MATH_CAL_METHOD == 1
#include <arm_math.h>
#define FOC_SINE(x)     (arm_sin_f32(x))
#define FOC_COSINE(x)   (arm_cos_f32(x))
#elif FOC_MATH_CAL_METHOD == 2
#include "cordic.h"
void cordic_calc_sin_cos_f32(float theta, float *sin_out, float *cos_out);
void cordic_calc_sin_cos_f16(float theta, float *sin_out, float *cos_out);
#define FOC_CALC_SINE_COSINE(theta, sine_out, cosine_out)  (cordic_calc_sin_cos_f32(theta, sine_out, cosine_out))
#elif FOC_MATH_CAL_METHOD == 3
#include "qtrigf.h"
#define FOC_SINE(x)     (qfsind(x * RAD2DEG))
#define FOC_COSINE(x)   (qfcosd(x * RAD2DEG))
#define FOC_CALC_SINE_COSINE(theta, sine_out, cosine_out)  (fast_sin_cos_q15(theta, sine_out, cosine_out))
#endif


/* --------------------- Type Definition --------------------- */
typedef void (*foc_duty_set_t)(unsigned short u_duty, unsigned short v_duty, unsigned short w_duty);
typedef void (*foc_pwm_start_t)(void);
typedef void (*foc_pwm_pause_t)(void);
typedef void (*foc_current_sample_get_t)(int *raw_adc_u, int *raw_adc_v, int *raw_adc_w);

typedef enum foc_fs_mode_enum_t {
    FOC_FS_MODE_READ = 0x01,
    FOC_FS_MODE_WRITE = 0x02
} foc_fs_mode_enum_t;

// Snapshot struct
typedef struct foc_snapshot_t {
    uint8_t *data;
    uint8_t version;
} foc_snapshot_t;

// Error enum
typedef enum foc_err_enum_t{
    FOC_ERR_OK = 0,
    FOC_ERR_FAILED,
    FOC_ERR_NULL_PTR,
    FOC_ERR_INVALID_PARAM,
    FOC_ERR_NOT_CALIBRATED,
    FOC_ERR_MISSING_SOURCE,
} foc_err_enum_t;

// LPF struct
typedef struct foc_lpf_t {
    float y_prev;
    float k;
    float y;
} foc_lpf_t;

// LPF enum
typedef enum foc_lpf_enable_t {
    FOC_LPF_DISABLE = 0,            // Disable all LPF
    FOC_LPF_VEL_ENABLE = 0x01,      // Enable velocity LPF
    FOC_LPF_CUR_D_ENABLE = 0x02,    // Enable current a LPF
    FOC_LPF_CUR_Q_ENABLE = 0x04,    // Enable current b LPF
    FOC_LPF_RESERVE = 0x08,         // Reserve
    FOC_LPF_ALL_ENABLE = 0x0F,      // Enable all LPF
} foc_lpf_enable_t;

// PID enum
typedef enum foc_pid_enable_t {
    FOC_PID_DISABLE = 0,            // Disable all PID
    FOC_PID_VEL_ENABLE = 0x01,      // Enable velocity PID
    FOC_PID_POS_ENABLE = 0x02,      // Enable position PID
    FOC_PID_CUR_D_ENABLE = 0x04,    // Enable current d PID
    FOC_PID_CUR_Q_ENABLE = 0x08,    // Enable current q PID
    FOC_PID_ALL_ENABLE = 0x0F,      // Enable all PID
} foc_pid_enable_t;

// Current sensor parameters
typedef struct foc_current_param_t {
    float u_ref;            // Reference voltage. eg. 3.3V
    float r_cs;             // Resistance of current sample. eg. 0.01 ohm
    float coeff;            // reserve
    struct {
        float sensor_offset_a;    // Sensor offset. eg. 1.65V
        float sensor_offset_b;    // Sensor offset. eg. 1.65V
        float sensor_offset_c;    // Sensor offset. eg. 1.65V
    };
    uint32_t resolution;    // ADC resolution. eg. 4096
    uint8_t sensor_gain;    // Sensor gain. eg. 50V/V
} foc_current_param_t;

// FOC state
typedef enum  foc_state_enum_t {
    FOC_STATE_PHASE_CALIB = 0,
    FOC_STATE_CUR_PHASE_CALIB,
    FOC_STATE_CUR_CALIB,
    FOC_STATE_ZERO_ANGLE_CALIB,
    FOC_STATE_TRIGO_CALCULATED,
    FOC_STATE_PWM_RUNNING,
    FOC_STATE_DIRECTION,
    FOC_STATE_INITIALIZED
} foc_state_enum_t;

// FOC loop method
typedef enum foc_loop_enum_t {
    FOC_LOOP_OPEN = 0,
    FOC_LOOP_POSITION = 0x01,
    FOC_LOOP_SPEED = 0x02,
    FOC_LOOP_CURRENT = 0x04,
    FOC_LOOP_ALL = FOC_LOOP_POSITION | FOC_LOOP_SPEED | FOC_LOOP_CURRENT
} foc_loop_enum_t;

typedef enum foc_pid_method_t {
    FOC_PID_METHOD_POSITIONAL = 0,
    FOC_PID_METHOD_INCREMENTAL = 1,
    FOC_PID_METHOD_DEFAULT = FOC_PID_METHOD_POSITIONAL,
} foc_pid_method_t;

/* ------------------------ FOC Struct ------------------------ */
typedef struct foc_t {
    /*--------------------- Core State Variables ---------------------*/
    // Angle related
    float theta;                   // Mechanical angle (rad)
    float theta_prev;              // Previous mechanical angle
    float theta_elec;              // Electrical angle (rad)
    float theta_factor;            // Pole pairs multiplier (pole_pairs * 2π)
    float full_rotation;           // Full rotation count
    float full_rotation_prev;      // Previous full rotation count

    // Velocity related
    float velocity;                // Angular velocity (rad/s)
    float velocity_prev;           // Previous angular velocity

    // Current measurements
    float i_d;                     // Direct-axis current
    float i_q;                     // Quadrature-axis current
    float i_alpha;                 // Alpha-axis current (Clark transform)
    float i_beta;                  // Beta-axis current (Clark transform)
    union {
        float i_phase[3];          // Phase currents (array format)
        struct {
            float i_u;             // Phase U current
            float i_v;             // Phase V current
            float i_w;             // Phase W current
        };
    };

    // Voltage outputs
    float u_d;                     // Direct-axis voltage
    float u_q;                     // Quadrature-axis voltage
    float u_alpha;                 // Alpha-axis voltage (Clark transform)
    float u_beta;                  // Beta-axis voltage (Clark transform)
    union {
        float u_phase[3];          // Phase voltages (array format)
        struct {
            float t_u;             // PWM duty cycle timing for phase U
            float t_v;             // PWM duty cycle timing for phase V
            float t_w;             // PWM duty cycle timing for phase W
        };
    };

    /*--------------------- Control Variables ---------------------*/
    // Setpoints
    union {
        float setpoint[3];         // Control setpoint array
        struct {
            float vel_setpoint;    // Velocity target (rad/s)
            float pos_setpoint;    // Position target (rad)
            float cur_q_setpoint;  // Q-axis current target (A)
        };
    };

    // Filters & Controllers
    union {
        foc_lpf_t *(lpf_all[3]);   // LPF array [velocity, d-axis, q-axis]
        struct {
            foc_lpf_t *vel;        // Velocity low-pass filter
            foc_lpf_t *cur_d;      // D-axis current filter
            foc_lpf_t *cur_q;      // Q-axis current filter
        } lpf;
    };

    union {
        pid_t *(pid_all[4]);       // PID array [velocity, position, d/q-axis]
        struct {
            pid_t *vel;            // Velocity PID controller
            pid_t *pos;            // Position PID controller
            pid_t *cur_d;          // D-axis current PID
            pid_t *cur_q;          // Q-axis current PID
        } pid;
    };

    /*--------------------- System Parameters ---------------------*/
    // Motor characteristics
    float kv_value;                // KV rating (RPM/Volt)
    float max_rpm;                 // Maximum operating RPM
    float max_voltage;             // Voltage limit for SVM
    float src_voltage;             // DC bus voltage
    uint8_t pole_pairs;            // Number of motor pole pairs
    foc_current_param_t current_param;  // Current sensing configuration

    // System configuration
    float dt;                      // Control loop period (seconds)
    uint16_t pwm_period;           // PWM timer period (counter ticks)
    uint32_t angle_sensor_resolution; // Encoder CPR (counts per revolution)
    foc_loop_enum_t loop_mode;     // Active control loop mode

    // Phase configuration
    union {
        uint8_t phase_seq[3];      // Phase mapping [U, V, W]
        struct {
            uint8_t phase_u_index; // Hardware phase U index
            uint8_t phase_v_index; // Hardware phase V index
            uint8_t phase_w_index; // Hardware phase W index
        };
    };

    /*--------------------- Hardware Interfaces ---------------------*/
    // Function pointers
    struct {
        foc_duty_set_t duty_set;   // PWM duty set function
        foc_pwm_start_t pwm_start; // PWM enable/disable
        foc_pwm_pause_t pwm_pause;
        void (*get_angle)(uint32_t *raw_data); // Angle sensor read callback
        foc_current_sample_get_t current_sample_get; // Current sampling callback
    } func;

    /*--------------------- Debug/System Control ---------------------*/
    char *name;                    // Instance identifier
    float init_angle;              // Initial mechanical offset (rad)
    float torque;                  // Estimated torque (Nm)

    // Status flags
    union {
        uint16_t state;
        struct {
            bool phase_calibrated : 1;     // Phase calibration status
            bool current_phase_aligned : 1;// Current-phase alignment status
            bool current_calibrated : 1;   // Current sensor calibration
            bool zero_angle_calibrated : 1;// Zero angle calibration
            bool trigo_calc_done : 1;      // Trigonometry calculation done
            bool pwm_is_running : 1;       // PWM output status
            bool inited : 1;               // Initialization status
            bool dir : 1;                  // Rotation direction (1=CW, 0=CCW)
            int8_t dir_flag;               // Direction multiplier (-1/1)
        };
    };

#if FOC_DATA_PERSISTENCE == 1
    /*--------------------- Data Persistence ---------------------*/
    foc_snapshot_t snapshot;       // State snapshot for crash recovery
#endif

#if FOC_USE_FREERTOS == 1
    /*--------------------- RTOS Integration ---------------------*/
    #if FOC_USE_ESP_IDF == 1
    spinlock_t lock;               // ESP32 atomic spinlock
    #else
    osMutexAttr_t lock_attr;       // CMSIS-RTOS mutex attributes
    osMutexId_t lock;              // Mutex handle
    #endif
#endif

    /*--------------------- Computation Cache ---------------------*/
    // High-frequency variables at end for cache optimization
    float sine;                    // sin(theta_elec)
    float cosine;                  // cos(theta_elec)
} __attribute__((aligned(4))) foc_t;


/* ------------------------ Function prototypes ------------------------ */
/* FOC Initialization and Destruction */

foc_t *foc_create(char *name);
foc_err_enum_t foc_destroy(foc_t *foc);

/* FOC Calibration */
foc_err_enum_t foc_zero_angle_calibration(foc_t *foc, float theta_elec, size_t ms, float voltage_divider, uint8_t max_retry);
foc_err_enum_t foc_current_calibration(foc_t *foc, size_t calibration_times);

/* FOC Link */
foc_err_enum_t foc_link_angle_sensor(foc_t *foc, void (*get_angle)(uint32_t *raw_data), uint32_t resolution);
foc_err_enum_t foc_link_zero_angle_set(foc_t *foc, void (*zero_angle_set)(float th));

foc_err_enum_t foc_link_pwm_start(foc_t *foc, foc_pwm_start_t pwm_start);
foc_err_enum_t foc_link_pwm_pause(foc_t *foc, foc_pwm_start_t pwm_pause);
foc_err_enum_t foc_link_duty(foc_t *foc, unsigned short pwm_period, foc_duty_set_t duty_set);
foc_err_enum_t foc_link_current_sample(foc_t *foc, foc_current_sample_get_t current_sample_get);

/**
 * @brief Angle sensor type enum
 */
typedef enum foc_angle_sensor_enum_t {
    FOC_ANGLE_SENSOR_NONE = 0,
    FOC_ANGLE_SENSOR_AS5600,
    FOC_ANGLE_SENSOR_AS5047A,
    FOC_ANGLE_SENSOR_AS5047B,
    FOC_ANGLE_SENSOR_AS5048A,
    FOC_ANGLE_SENSOR_AS5048B,
    FOC_ANGLE_SENSOR_MT6701 ,
    FOC_ANGLE_SENSOR_MT6835 ,
} foc_angle_sensor_enum_t;

/* FOC Parameters Set */
foc_err_enum_t foc_enable_angle_sensor(foc_t *foc, foc_angle_sensor_enum_t angle_sensor_type);
foc_err_enum_t foc_set_src_voltage(foc_t *foc, float src_voltage);
foc_err_enum_t foc_set_kv(foc_t *foc, float kv);
foc_err_enum_t foc_set_pole_pairs(foc_t *foc, unsigned char pole_pairs);
foc_err_enum_t foc_set_current_param(foc_t *foc, float u_ref, uint32_t resolution, float r_cs, uint8_t sensor_gain);
foc_err_enum_t foc_set_dt(foc_t *foc, float dt);
foc_err_enum_t foc_current_get(foc_t *foc);
void foc_set_udq(foc_t *foc, float u_d, float u_q);

/* FOC PWM Control */
void foc_pwm_start(foc_t *foc);
void foc_pwm_pause(foc_t *foc);
void foc_pwm_set_duty(foc_t *foc, uint16_t t_u, uint16_t t_v, uint16_t t_w);

/* FOC Mathematical Operations */
void foc_trigonometry(foc_t *foc);
void foc_inv_park(foc_t *foc);
void foc_park(foc_t *foc);
void foc_park_u(foc_t *foc);
void foc_clarke(foc_t *foc);
void foc_svpwm(foc_t *foc);
float foc_get_velocity(foc_t *foc);
float foc_normalize_angle(float rad);
foc_err_enum_t foc_torque_calc(foc_t *foc);

/* FOC Low Pass Filter */
foc_err_enum_t foc_lpf_init(foc_t *foc, foc_lpf_enable_t enable);
foc_err_enum_t foc_lpf_deinit(foc_t *foc, foc_lpf_enable_t enable);
foc_err_enum_t foc_lpf_vel_set_k(foc_t *foc, float k);
foc_err_enum_t foc_lpf_cur_q_set_k(foc_t *foc, float k);
foc_err_enum_t foc_lpf_cur_d_set_k(foc_t *foc, float k);
foc_lpf_t *foc_lpf_vel(const foc_t *foc);
foc_lpf_t *foc_lpf_cur_q(const foc_t *foc);
foc_lpf_t *foc_lpf_cur_d(const foc_t *foc);
foc_lpf_t *foc_lpf_create(float k);
foc_err_enum_t foc_lpf_destroy(foc_lpf_t *lpf);
float foc_lpf_calc(foc_lpf_t *lpf, float x);
void foc_lpf_vel_calc(foc_t *foc);
void foc_lpf_cur_q_calc(foc_t *foc);
void foc_lpf_cur_d_calc(foc_t *foc);

/* FOC PID Control */
foc_err_enum_t foc_pid_init(foc_t *foc, foc_pid_enable_t enable);
foc_err_enum_t foc_pid_deinit(foc_t *foc, foc_pid_enable_t enable);
foc_err_enum_t foc_pid_vel_set_k(foc_t *foc, float kp, float ki, float kd);
foc_err_enum_t foc_pid_cur_q_set_k(foc_t *foc, float kp, float ki, float kd);
foc_err_enum_t foc_pid_cur_d_set_k(foc_t *foc, float kp, float ki, float kd);
pid_t *foc_pid_vel(const foc_t *foc);
pid_t *foc_pid_cur_q(const foc_t *foc);
pid_t *foc_pid_cur_d(const foc_t *foc);

/* FOC Angle Calculation */
void foc_get_angle(foc_t *foc);

/* FOC Test */
foc_err_enum_t foc_openloop_test(foc_t *foc, float u_q, float u_d, float angle_step, size_t sustain_ms);
foc_err_enum_t foc_check_dir(foc_t *foc);
foc_err_enum_t foc_check_phase_seq(foc_t *foc, float u_q, size_t sustain_ms);
foc_err_enum_t foc_check_current_phase_seq(); // TODO: check current phase sequence

/* FOC Task */
foc_err_enum_t foc_set_loop_mode(foc_t *foc, foc_loop_enum_t mode);
void foc_task(foc_t *foc);

void foc_position_task(foc_t *foc);
void foc_speed_task(foc_t *foc);
void foc_cur_sample_task(foc_t *foc);
void foc_control_task(foc_t *foc);



/* FOC Data Persistence */
#if FOC_DATA_PERSISTENCE == 1
foc_err_enum_t foc_snapshot_create(foc_t *foc);
foc_err_enum_t foc_save_snapshot(const foc_t *foc_src, const char *save_path);
foc_err_enum_t foc_load_snapshot(foc_t *foc_dst, const char *load_path);
void foc_fs_init(void);
void foc_fs_deinit(void);
#endif



#ifdef __cplusplus
}
#endif // __cplusplus

#endif  //FOC_H
