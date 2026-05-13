#ifndef __CONTROL_H
#define __CONTROL_H

#include <stdint.h>

#define CONTROL_LOOP_DT_S                  0.01f

#define CONTROL_MOTOR_MIN_PWM              1000.0f
#define CONTROL_MOTOR_IDLE_PWM             1050.0f
#define CONTROL_MOTOR_CRUISE_PWM           1100.0f
#define CONTROL_MOTOR_MAX_PWM              1100.0f
#define CONTROL_MAX_YAW_DIFF_PWM           120.0f
#define CONTROL_STABILIZE_MIN_THROTTLE_PWM CONTROL_MOTOR_IDLE_PWM

#define CONTROL_REMOTE_THROTTLE_MIN_PWM    1000.0f
#define CONTROL_REMOTE_THROTTLE_MAX_PWM    2000.0f

#define CONTROL_SERVO1_CENTER_US           1300.0f
#define CONTROL_SERVO2_CENTER_US           1500.0f
#define CONTROL_SERVO_CENTER_US            CONTROL_SERVO1_CENTER_US
#define CONTROL_SERVO_MIN_US               1150.0f
#define CONTROL_SERVO_MAX_US               2150.0f
#define CONTROL_SERVO_US_PER_PID           10.0f
#define CONTROL_STICK_SERVO_FEEDFORWARD_US 250.0f
#define CONTROL_COMMAND_ANGLE_DEADBAND_DEG 1.5f
#define CONTROL_COMMAND_YAW_DEADBAND_DPS   8.0f

#define CONTROL_ANGLE_PITCH_KP             6.0f
#define CONTROL_ANGLE_PITCH_KI             0.00f
#define CONTROL_ANGLE_PITCH_KD             0.00f
#define CONTROL_ANGLE_PITCH_I_LIMIT        10.0f
#define CONTROL_ANGLE_PITCH_OUT_LIMIT      120.0f

#define CONTROL_ANGLE_ROLL_KP              6.0f
#define CONTROL_ANGLE_ROLL_KI              0.00f
#define CONTROL_ANGLE_ROLL_KD              0.00f
#define CONTROL_ANGLE_ROLL_I_LIMIT         10.0f
#define CONTROL_ANGLE_ROLL_OUT_LIMIT       120.0f

#define CONTROL_RATE_PITCH_KP              0.12f
#define CONTROL_RATE_PITCH_KI              0.02f
#define CONTROL_RATE_PITCH_KD              0.002f
#define CONTROL_RATE_PITCH_I_LIMIT         80.0f
#define CONTROL_RATE_PITCH_OUT_LIMIT       35.0f

#define CONTROL_RATE_ROLL_KP               0.12f
#define CONTROL_RATE_ROLL_KI               0.02f
#define CONTROL_RATE_ROLL_KD               0.002f
#define CONTROL_RATE_ROLL_I_LIMIT          80.0f
#define CONTROL_RATE_ROLL_OUT_LIMIT        35.0f

#define CONTROL_RATE_YAW_KP                1.20f
#define CONTROL_RATE_YAW_KI                0.10f
#define CONTROL_RATE_YAW_KD                0.00f
#define CONTROL_RATE_YAW_I_LIMIT           80.0f
#define CONTROL_RATE_YAW_OUT_LIMIT         CONTROL_MAX_YAW_DIFF_PWM

#define CONTROL_PID_DEADBAND_DEG           0.2f
#define CONTROL_PID_D_LPF_ALPHA            0.70f
#define CONTROL_INTEGRAL_SEPARATION_DEG    10.0f
#define CONTROL_RATE_D_LPF_ALPHA           0.65f
#define CONTROL_RATE_DEADBAND_DPS          0.5f
#define CONTROL_RATE_I_SEPARATION_DPS      80.0f

#define CONTROL_REMOTE_MAX_PITCH_DEG       20.0f
#define CONTROL_REMOTE_MAX_ROLL_DEG        20.0f
#define CONTROL_REMOTE_MAX_YAW_RATE_DPS    120.0f
#define CONTROL_REMOTE_CENTER_DEADBAND_US  25.0f
#define CONTROL_REMOTE_LINK_TIMEOUT_MS     500u
#define CONTROL_REMOTE_FAILSAFE_LEVEL_MS   1000u
#define CONTROL_REMOTE_FAILSAFE_STEP_MS    100u
#define CONTROL_REMOTE_FAILSAFE_STEP_PWM   5.0f
#define CONTROL_REMOTE_FAILSAFE_MIN_PWM    1060.0f
#define CONTROL_REMOTE_FAILSAFE_DISARM_MS  4000u

#define CONTROL_SERVO1_PITCH_SIGN          (-1.0f)
#define CONTROL_SERVO1_ROLL_SIGN           (-1.0f)
#define CONTROL_SERVO2_PITCH_SIGN          (-1.0f)
#define CONTROL_SERVO2_ROLL_SIGN           (1.0f)

#define CONTROL_MOTOR1_YAW_SIGN            (1.0f)
#define CONTROL_MOTOR2_YAW_SIGN            (-1.0f)

typedef struct
{
    float throttle_pwm;
    float pitch_deg;
    float roll_deg;
    float yaw_rate_dps;
} Control_Command_t;

typedef struct
{
    uint16_t motor1_pwm;
    uint16_t motor2_pwm;
    uint16_t servo1_us;
    uint16_t servo2_us;
    float pitch_angle_out;
    float roll_angle_out;
    float pitch_rate_target;
    float roll_rate_target;
    float pitch_servo_out;
    float roll_servo_out;
    float yaw_out;
} Control_Output_t;

void Control_Init(void);
void Control_Reset(void);
void Control_SetArmed(uint8_t armed);
uint8_t Control_GetArmed(void);
void Control_SetCommand(const Control_Command_t *cmd);
void Control_Update(float pitch_deg, float roll_deg,
                    float pitch_rate_dps, float roll_rate_dps, float yaw_rate_dps);
void Control_GetOutput(Control_Output_t *out);

#endif
