#include "Control.h"
#include "PID.h"
#include "Motor.h"
#include "Servo.h"

static PID_t pitch_angle_pid;
static PID_t roll_angle_pid;
static PID_t pitch_rate_pid;
static PID_t roll_rate_pid;
static PID_t yaw_rate_pid;
static Control_Command_t control_cmd;
static Control_Output_t control_out;
static uint8_t control_armed = 0u;
static uint8_t attitude_reference_valid = 0u;
static float pitch_reference_deg = 0.0f;
static float roll_reference_deg = 0.0f;

static void Control_ApplyOutput(void);

static float Control_ClampFloat(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }
    if (value < min_value)
    {
        return min_value;
    }
    return value;
}

static float Control_ApplyDeadband(float value, float deadband)
{
    if ((value > -deadband) && (value < deadband))
    {
        return 0.0f;
    }
    return value;
}

static uint16_t Control_FloatToUint16(float value)
{
    if (value <= 0.0f)
    {
        return 0u;
    }
    return (uint16_t)(value + 0.5f);
}

static float Control_MapThrottleToMotorBase(float throttle_pwm)
{
    float normalized;

    if (throttle_pwm <= CONTROL_MOTOR_IDLE_PWM)
    {
        return CONTROL_MOTOR_MIN_PWM;
    }

    normalized = (throttle_pwm - CONTROL_MOTOR_IDLE_PWM) /
                 (CONTROL_REMOTE_THROTTLE_MAX_PWM - CONTROL_MOTOR_IDLE_PWM);
    normalized = Control_ClampFloat(normalized, 0.0f, 1.0f);

    return CONTROL_MOTOR_IDLE_PWM +
           normalized * (CONTROL_MOTOR_MAX_PWM - CONTROL_MOTOR_IDLE_PWM);
}

static void Control_MixServoUs(float pitch_servo_us, float roll_servo_us)
{
    float servo1_cmd;
    float servo2_cmd;

    servo1_cmd = CONTROL_SERVO1_CENTER_US +
                 CONTROL_SERVO1_PITCH_SIGN * pitch_servo_us +
                 CONTROL_SERVO1_ROLL_SIGN * roll_servo_us;
    servo2_cmd = CONTROL_SERVO2_CENTER_US +
                 CONTROL_SERVO2_PITCH_SIGN * pitch_servo_us +
                 CONTROL_SERVO2_ROLL_SIGN * roll_servo_us;

    servo1_cmd = Control_ClampFloat(servo1_cmd, CONTROL_SERVO_MIN_US, CONTROL_SERVO_MAX_US);
    servo2_cmd = Control_ClampFloat(servo2_cmd, CONTROL_SERVO_MIN_US, CONTROL_SERVO_MAX_US);

    control_out.servo1_us = Control_FloatToUint16(servo1_cmd);
    control_out.servo2_us = Control_FloatToUint16(servo2_cmd);
}

static void Control_UpdateStickServoControl(void)
{
    float motor_base_pwm;
    float yaw_manual_pwm;
    float pitch_servo_us;
    float roll_servo_us;

    Control_Reset();

    motor_base_pwm = Control_MapThrottleToMotorBase(control_cmd.throttle_pwm);
    if (control_cmd.throttle_pwm > CONTROL_MOTOR_IDLE_PWM)
    {
        yaw_manual_pwm = (control_cmd.yaw_rate_dps / CONTROL_REMOTE_MAX_YAW_RATE_DPS) *
                         CONTROL_MAX_YAW_DIFF_PWM;
    }
    else
    {
        yaw_manual_pwm = 0.0f;
    }

    pitch_servo_us = (control_cmd.pitch_deg / CONTROL_REMOTE_MAX_PITCH_DEG) *
                     CONTROL_STICK_SERVO_FEEDFORWARD_US;
    roll_servo_us = (control_cmd.roll_deg / CONTROL_REMOTE_MAX_ROLL_DEG) *
                    CONTROL_STICK_SERVO_FEEDFORWARD_US;

    control_out.yaw_out = yaw_manual_pwm;
    control_out.motor1_pwm = Control_FloatToUint16(
        Control_ClampFloat(motor_base_pwm + CONTROL_MOTOR1_YAW_SIGN * yaw_manual_pwm,
                           CONTROL_MOTOR_MIN_PWM, CONTROL_MOTOR_MAX_PWM));
    control_out.motor2_pwm = Control_FloatToUint16(
        Control_ClampFloat(motor_base_pwm + CONTROL_MOTOR2_YAW_SIGN * yaw_manual_pwm,
                           CONTROL_MOTOR_MIN_PWM, CONTROL_MOTOR_MAX_PWM));

    Control_MixServoUs(pitch_servo_us, roll_servo_us);
    Control_ApplyOutput();
}

static void Control_ApplyOutput(void)
{
    SetMotor1(control_out.motor1_pwm);
    SetMotor2(control_out.motor2_pwm);
    Servo1_SetPulse(control_out.servo1_us);
    Servo2_SetPulse(control_out.servo2_us);
}

static void Control_LoadPidParams(void)
{
    pitch_angle_pid.Kp = CONTROL_ANGLE_PITCH_KP;
    pitch_angle_pid.Ki = CONTROL_ANGLE_PITCH_KI;
    pitch_angle_pid.Kd = CONTROL_ANGLE_PITCH_KD;
    pitch_angle_pid.Integral_Max = CONTROL_ANGLE_PITCH_I_LIMIT;
    pitch_angle_pid.Output_Max = CONTROL_ANGLE_PITCH_OUT_LIMIT;
    pitch_angle_pid.DeadBand = CONTROL_PID_DEADBAND_DEG;
    pitch_angle_pid.Derivative_LPF_Alpha = CONTROL_PID_D_LPF_ALPHA;
    pitch_angle_pid.Enable_Integral_Separation = 1u;
    pitch_angle_pid.Integral_Separation_Threshold = CONTROL_INTEGRAL_SEPARATION_DEG;

    roll_angle_pid.Kp = CONTROL_ANGLE_ROLL_KP;
    roll_angle_pid.Ki = CONTROL_ANGLE_ROLL_KI;
    roll_angle_pid.Kd = CONTROL_ANGLE_ROLL_KD;
    roll_angle_pid.Integral_Max = CONTROL_ANGLE_ROLL_I_LIMIT;
    roll_angle_pid.Output_Max = CONTROL_ANGLE_ROLL_OUT_LIMIT;
    roll_angle_pid.DeadBand = CONTROL_PID_DEADBAND_DEG;
    roll_angle_pid.Derivative_LPF_Alpha = CONTROL_PID_D_LPF_ALPHA;
    roll_angle_pid.Enable_Integral_Separation = 1u;
    roll_angle_pid.Integral_Separation_Threshold = CONTROL_INTEGRAL_SEPARATION_DEG;

    pitch_rate_pid.Kp = CONTROL_RATE_PITCH_KP;
    pitch_rate_pid.Ki = CONTROL_RATE_PITCH_KI;
    pitch_rate_pid.Kd = CONTROL_RATE_PITCH_KD;
    pitch_rate_pid.Integral_Max = CONTROL_RATE_PITCH_I_LIMIT;
    pitch_rate_pid.Output_Max = CONTROL_RATE_PITCH_OUT_LIMIT;
    pitch_rate_pid.DeadBand = CONTROL_RATE_DEADBAND_DPS;
    pitch_rate_pid.Derivative_LPF_Alpha = CONTROL_RATE_D_LPF_ALPHA;
    pitch_rate_pid.Enable_Integral_Separation = 1u;
    pitch_rate_pid.Integral_Separation_Threshold = CONTROL_RATE_I_SEPARATION_DPS;

    roll_rate_pid.Kp = CONTROL_RATE_ROLL_KP;
    roll_rate_pid.Ki = CONTROL_RATE_ROLL_KI;
    roll_rate_pid.Kd = CONTROL_RATE_ROLL_KD;
    roll_rate_pid.Integral_Max = CONTROL_RATE_ROLL_I_LIMIT;
    roll_rate_pid.Output_Max = CONTROL_RATE_ROLL_OUT_LIMIT;
    roll_rate_pid.DeadBand = CONTROL_RATE_DEADBAND_DPS;
    roll_rate_pid.Derivative_LPF_Alpha = CONTROL_RATE_D_LPF_ALPHA;
    roll_rate_pid.Enable_Integral_Separation = 1u;
    roll_rate_pid.Integral_Separation_Threshold = CONTROL_RATE_I_SEPARATION_DPS;

    yaw_rate_pid.Kp = CONTROL_RATE_YAW_KP;
    yaw_rate_pid.Ki = CONTROL_RATE_YAW_KI;
    yaw_rate_pid.Kd = CONTROL_RATE_YAW_KD;
    yaw_rate_pid.Integral_Max = CONTROL_RATE_YAW_I_LIMIT;
    yaw_rate_pid.Output_Max = CONTROL_RATE_YAW_OUT_LIMIT;
    yaw_rate_pid.DeadBand = CONTROL_RATE_DEADBAND_DPS;
    yaw_rate_pid.Derivative_LPF_Alpha = CONTROL_RATE_D_LPF_ALPHA;
    yaw_rate_pid.Enable_Integral_Separation = 1u;
    yaw_rate_pid.Integral_Separation_Threshold = CONTROL_RATE_I_SEPARATION_DPS;

    PID_Reset(&pitch_angle_pid);
    PID_Reset(&roll_angle_pid);
    PID_Reset(&pitch_rate_pid);
    PID_Reset(&roll_rate_pid);
    PID_Reset(&yaw_rate_pid);
}

void Control_Init(void)
{
    control_cmd.throttle_pwm = CONTROL_MOTOR_MIN_PWM;
    control_cmd.pitch_deg = 0.0f;
    control_cmd.roll_deg = 0.0f;
    control_cmd.yaw_rate_dps = 0.0f;

    control_out.motor1_pwm = (uint16_t)CONTROL_MOTOR_MIN_PWM;
    control_out.motor2_pwm = (uint16_t)CONTROL_MOTOR_MIN_PWM;
    control_out.servo1_us = (uint16_t)CONTROL_SERVO1_CENTER_US;
    control_out.servo2_us = (uint16_t)CONTROL_SERVO2_CENTER_US;
    control_out.pitch_angle_out = 0.0f;
    control_out.roll_angle_out = 0.0f;
    control_out.pitch_rate_target = 0.0f;
    control_out.roll_rate_target = 0.0f;
    control_out.pitch_servo_out = 0.0f;
    control_out.roll_servo_out = 0.0f;
    control_out.yaw_out = 0.0f;

    control_armed = 0u;

    Control_LoadPidParams();
    Control_ApplyOutput();
}

void Control_Reset(void)
{
    PID_Reset(&pitch_angle_pid);
    PID_Reset(&roll_angle_pid);
    PID_Reset(&pitch_rate_pid);
    PID_Reset(&roll_rate_pid);
    PID_Reset(&yaw_rate_pid);

    control_out.pitch_angle_out = 0.0f;
    control_out.roll_angle_out = 0.0f;
    control_out.pitch_rate_target = 0.0f;
    control_out.roll_rate_target = 0.0f;
    control_out.pitch_servo_out = 0.0f;
    control_out.roll_servo_out = 0.0f;
    control_out.yaw_out = 0.0f;
    attitude_reference_valid = 0u;
    pitch_reference_deg = 0.0f;
    roll_reference_deg = 0.0f;
}

void Control_SetArmed(uint8_t armed)
{
    control_armed = armed ? 1u : 0u;
    if (control_armed == 0u)
    {
        Control_Reset();
    }
}

uint8_t Control_GetArmed(void)
{
    return control_armed;
}

void Control_SetCommand(const Control_Command_t *cmd)
{
    if (cmd == 0)
    {
        return;
    }

    control_cmd.throttle_pwm = Control_ClampFloat(cmd->throttle_pwm,
                                                  CONTROL_REMOTE_THROTTLE_MIN_PWM,
                                                  CONTROL_REMOTE_THROTTLE_MAX_PWM);
    control_cmd.pitch_deg = Control_ApplyDeadband(cmd->pitch_deg,
                                                  CONTROL_COMMAND_ANGLE_DEADBAND_DEG);
    control_cmd.roll_deg = Control_ApplyDeadband(cmd->roll_deg,
                                                 CONTROL_COMMAND_ANGLE_DEADBAND_DEG);
    control_cmd.yaw_rate_dps = Control_ClampFloat(cmd->yaw_rate_dps,
                                                  -CONTROL_REMOTE_MAX_YAW_RATE_DPS,
                                                  CONTROL_REMOTE_MAX_YAW_RATE_DPS);
    control_cmd.yaw_rate_dps = Control_ApplyDeadband(control_cmd.yaw_rate_dps,
                                                     CONTROL_COMMAND_YAW_DEADBAND_DPS);
}

void Control_Update(float pitch_deg, float roll_deg,
                    float pitch_rate_dps, float roll_rate_dps, float yaw_rate_dps)
{
    float motor_base_pwm;
    float pitch_target_deg;
    float roll_target_deg;
    float pitch_servo_us;
    float roll_servo_us;

    if (control_armed == 0u)
    {
        Control_Reset();
        control_out.motor1_pwm = (uint16_t)CONTROL_MOTOR_MIN_PWM;
        control_out.motor2_pwm = (uint16_t)CONTROL_MOTOR_MIN_PWM;
        control_out.servo1_us = (uint16_t)CONTROL_SERVO1_CENTER_US;
        control_out.servo2_us = (uint16_t)CONTROL_SERVO2_CENTER_US;
        Control_ApplyOutput();
        return;
    }

    if (control_cmd.throttle_pwm <= CONTROL_STABILIZE_MIN_THROTTLE_PWM)
    {
        Control_UpdateStickServoControl();
        return;
    }

    if (attitude_reference_valid == 0u)
    {
        attitude_reference_valid = 1u;
        pitch_reference_deg = pitch_deg;
        roll_reference_deg = roll_deg;
        PID_Reset(&pitch_angle_pid);
        PID_Reset(&roll_angle_pid);
        PID_Reset(&pitch_rate_pid);
        PID_Reset(&roll_rate_pid);
        PID_Reset(&yaw_rate_pid);
    }

    pitch_target_deg = pitch_reference_deg + control_cmd.pitch_deg;
    roll_target_deg = roll_reference_deg + control_cmd.roll_deg;

    control_out.pitch_rate_target = PID_Calc(&pitch_angle_pid, pitch_target_deg, pitch_deg, CONTROL_LOOP_DT_S);
    control_out.roll_rate_target = PID_Calc(&roll_angle_pid, roll_target_deg, roll_deg, CONTROL_LOOP_DT_S);
    control_out.pitch_angle_out = control_out.pitch_rate_target;
    control_out.roll_angle_out = control_out.roll_rate_target;

    control_out.pitch_servo_out = PID_Calc(&pitch_rate_pid,
                                           control_out.pitch_rate_target,
                                           pitch_rate_dps,
                                           CONTROL_LOOP_DT_S);
    control_out.roll_servo_out = PID_Calc(&roll_rate_pid,
                                          control_out.roll_rate_target,
                                          roll_rate_dps,
                                          CONTROL_LOOP_DT_S);
    control_out.yaw_out = PID_Calc(&yaw_rate_pid,
                                   control_cmd.yaw_rate_dps,
                                   yaw_rate_dps,
                                   CONTROL_LOOP_DT_S);

    pitch_servo_us = CONTROL_SERVO_US_PER_PID * control_out.pitch_servo_out +
                     (control_cmd.pitch_deg / CONTROL_REMOTE_MAX_PITCH_DEG) *
                     CONTROL_STICK_SERVO_FEEDFORWARD_US;
    roll_servo_us = CONTROL_SERVO_US_PER_PID * control_out.roll_servo_out +
                    (control_cmd.roll_deg / CONTROL_REMOTE_MAX_ROLL_DEG) *
                    CONTROL_STICK_SERVO_FEEDFORWARD_US;

    Control_MixServoUs(pitch_servo_us, roll_servo_us);

    motor_base_pwm = Control_MapThrottleToMotorBase(control_cmd.throttle_pwm);

    control_out.motor1_pwm = Control_FloatToUint16(
        Control_ClampFloat(motor_base_pwm + CONTROL_MOTOR1_YAW_SIGN * control_out.yaw_out,
                           CONTROL_MOTOR_MIN_PWM, CONTROL_MOTOR_MAX_PWM));
    control_out.motor2_pwm = Control_FloatToUint16(
        Control_ClampFloat(motor_base_pwm + CONTROL_MOTOR2_YAW_SIGN * control_out.yaw_out,
                           CONTROL_MOTOR_MIN_PWM, CONTROL_MOTOR_MAX_PWM));

    Control_ApplyOutput();
}

void Control_GetOutput(Control_Output_t *out)
{
    if (out == 0)
    {
        return;
    }
    *out = control_out;
}
