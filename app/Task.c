#include "USART.h"
#include "Timer.h"
#include "MY_NVIC.h"
#include "Delay.h"
#include "MPU6000.h"
#include "Servo.h"
#include "Motor.h"
#include "Event.h"
#include "Attitude.h"
#include "Control.h"
#include "NRF24L01.h"
#include <stdio.h>

static float pitch = 0.0f;
static float roll = 0.0f;
static float yaw = 0.0f;
static float pitch_rate_dps = 0.0f;
static float roll_rate_dps = 0.0f;
static float yaw_rate_dps = 0.0f;
static uint8_t nrf_ready = 0u;
static uint16_t remote_timeout_ms = CONTROL_REMOTE_LINK_TIMEOUT_MS;
static Control_Command_t remote_cmd;
static float failsafe_entry_throttle_pwm = CONTROL_MOTOR_MIN_PWM;
static uint8_t failsafe_active = 0u;

#define TASK_NRF_RECHECK_PERIOD_MS        100u
#define TASK_NRF_RECHECK_STEP_MS          10u
#define TASK_ESC_CALIBRATION_MODE         0u

static uint16_t nrf_recheck_ms = 0u;

#if TASK_ESC_CALIBRATION_MODE
static void Task_RunEscCalibrationMode(void)
{
    printf("esc calibration: high throttle\r\n");
    SetMotor1(2000u);
    SetMotor2(2000u);
    Delay_s(6);

    printf("esc calibration: low throttle\r\n");
    SetMotor1(1000u);
    SetMotor2(1000u);
    Delay_s(8);

    printf("esc calibration: done, holding low throttle\r\n");
    while (1)
    {
        SetMotor1(1000u);
        SetMotor2(1000u);
        Delay_s(1);
    }
}
#endif

static void Task_WarmupMahony(void)
{
    uint16_t i;

    for (i = 0; i < 200; i++)
    {
        MPU6000_DataChange();
        Delay_ms(5);
    }
}

static float Task_ClampFloat(float value, float min_value, float max_value)
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

static float Task_MapCenteredChannel(uint16_t value, float limit)
{
    float centered;
    float normalized;

    centered = (float)value - 1500.0f;
    if ((centered > -CONTROL_REMOTE_CENTER_DEADBAND_US) &&
        (centered < CONTROL_REMOTE_CENTER_DEADBAND_US))
    {
        return 0.0f;
    }

    if (centered > 0.0f)
    {
        centered -= CONTROL_REMOTE_CENTER_DEADBAND_US;
    }
    else
    {
        centered += CONTROL_REMOTE_CENTER_DEADBAND_US;
    }

    normalized = centered / (500.0f - CONTROL_REMOTE_CENTER_DEADBAND_US);
    normalized = Task_ClampFloat(normalized, -1.0f, 1.0f);

    return normalized * limit;
}

static void Task_SetSafeRemoteCommand(void)
{
    remote_cmd.throttle_pwm = CONTROL_MOTOR_MIN_PWM;
    remote_cmd.pitch_deg = 0.0f;
    remote_cmd.roll_deg = 0.0f;
    remote_cmd.yaw_rate_dps = 0.0f;
}

static void Task_ResetFailsafe(void)
{
    failsafe_active = 0u;
    failsafe_entry_throttle_pwm = CONTROL_MOTOR_MIN_PWM;
}

static void Task_ApplyRemoteLossPolicy(void)
{
    uint16_t failsafe_elapsed_ms;
    uint16_t descend_steps;
    float failsafe_throttle_pwm;

    if (remote_timeout_ms < 0xFFFFu - 10u)
    {
        remote_timeout_ms = (uint16_t)(remote_timeout_ms + 10u);
    }

    if (Control_GetArmed() == 0u)
    {
        if (remote_timeout_ms >= CONTROL_REMOTE_LINK_TIMEOUT_MS)
        {
            Task_SetSafeRemoteCommand();
            Control_SetCommand(&remote_cmd);
            Control_SetArmed(0u);
            Task_ResetFailsafe();
        }
        return;
    }

    if (remote_timeout_ms < CONTROL_REMOTE_LINK_TIMEOUT_MS)
    {
        return;
    }

    if (failsafe_active == 0u)
    {
        failsafe_active = 1u;
        failsafe_entry_throttle_pwm = Task_ClampFloat(remote_cmd.throttle_pwm,
                                                      CONTROL_MOTOR_IDLE_PWM,
                                                      CONTROL_REMOTE_THROTTLE_MAX_PWM);
    }

    failsafe_elapsed_ms = (uint16_t)(remote_timeout_ms - CONTROL_REMOTE_LINK_TIMEOUT_MS);
    failsafe_throttle_pwm = failsafe_entry_throttle_pwm;

    if (failsafe_elapsed_ms > CONTROL_REMOTE_FAILSAFE_LEVEL_MS)
    {
        descend_steps = (uint16_t)((failsafe_elapsed_ms - CONTROL_REMOTE_FAILSAFE_LEVEL_MS) /
                                   CONTROL_REMOTE_FAILSAFE_STEP_MS);
        failsafe_throttle_pwm = failsafe_entry_throttle_pwm -
                                ((float)descend_steps * CONTROL_REMOTE_FAILSAFE_STEP_PWM);
    }

    if (failsafe_throttle_pwm < CONTROL_REMOTE_FAILSAFE_MIN_PWM)
    {
        failsafe_throttle_pwm = CONTROL_REMOTE_FAILSAFE_MIN_PWM;
    }

    remote_cmd.throttle_pwm = failsafe_throttle_pwm;
    remote_cmd.pitch_deg = 0.0f;
    remote_cmd.roll_deg = 0.0f;
    remote_cmd.yaw_rate_dps = 0.0f;
    Control_SetCommand(&remote_cmd);
    Control_SetArmed(1u);

    if (failsafe_elapsed_ms >= CONTROL_REMOTE_FAILSAFE_DISARM_MS)
    {
        Task_SetSafeRemoteCommand();
        Control_SetCommand(&remote_cmd);
        Control_SetArmed(0u);
        Task_ResetFailsafe();
    }
}

static void Task_TryRecoverRadio(void)
{
    if ((nrf_ready != 0u) && (NRF24L01_IsConfigured() != 0u))
    {
        nrf_recheck_ms = 0u;
        return;
    }

    if (nrf_recheck_ms < TASK_NRF_RECHECK_PERIOD_MS)
    {
        nrf_recheck_ms = (uint16_t)(nrf_recheck_ms + TASK_NRF_RECHECK_STEP_MS);
    }

    if (nrf_recheck_ms >= TASK_NRF_RECHECK_PERIOD_MS)
    {
        nrf_recheck_ms = 0u;
        NRF24L01_Init();
        nrf_ready = (uint8_t)((NRF24L01_Check() != 0u) && (NRF24L01_IsConfigured() != 0u));
    }
}

static void Task_UpdateCommandFromRemote(void)
{
    NRF24L01_RemoteFrame_t frame;
    uint8_t updated = 0u;

    Task_TryRecoverRadio();

    if (nrf_ready == 0u)
    {
        Task_ApplyRemoteLossPolicy();
        return;
    }

    while (NRF24L01_ReadRemoteFrame(&frame) != 0u)
    {
        updated = 1u;

        remote_cmd.throttle_pwm = Task_ClampFloat((float)frame.throttle,
                                                  CONTROL_REMOTE_THROTTLE_MIN_PWM,
                                                  CONTROL_REMOTE_THROTTLE_MAX_PWM);
        remote_cmd.yaw_rate_dps = Task_MapCenteredChannel(frame.yaw, CONTROL_REMOTE_MAX_YAW_RATE_DPS);
        remote_cmd.pitch_deg = Task_MapCenteredChannel(frame.pitch, CONTROL_REMOTE_MAX_PITCH_DEG);
        remote_cmd.roll_deg = Task_MapCenteredChannel(frame.roll, CONTROL_REMOTE_MAX_ROLL_DEG);

        Control_SetCommand(&remote_cmd);
        Control_SetArmed((frame.flags & NRF24L01_FLAG_ARMED) ? 1u : 0u);
        remote_timeout_ms = 0u;
        Task_ResetFailsafe();
    }

    if (updated == 0u)
    {
        Task_ApplyRemoteLossPolicy();
    }
}

void System_Init(void)
{
    My_NVIC_Init();
    My_UART_Init();
    Motor_Init();
    Servo_Init();

#if TASK_ESC_CALIBRATION_MODE
    Task_RunEscCalibrationMode();
#endif

    SetMotor1((uint16_t)CONTROL_MOTOR_MIN_PWM);
    SetMotor2((uint16_t)CONTROL_MOTOR_MIN_PWM);
    Servo1_SetPulse((uint16_t)CONTROL_SERVO1_CENTER_US);
    Servo2_SetPulse((uint16_t)CONTROL_SERVO2_CENTER_US);
    Delay_ms(100);
    Timer_Init();
    MPU6000_Init();
    NRF24L01_Init();

    Delay_s(2);
    Gyro_Calibrate();
    Angle_Calibrate();
    Task_WarmupMahony();
    Mahony_AngleCalibrate();

    Control_Init();
    Task_SetSafeRemoteCommand();
    Control_SetCommand(&remote_cmd);
    Control_SetArmed(0u);
    Task_ResetFailsafe();

    nrf_ready = (uint8_t)((NRF24L01_Check() != 0u) && (NRF24L01_IsConfigured() != 0u));
    nrf_recheck_ms = 0u;

    printf("system ready, nrf:%d\r\n", nrf_ready);
}

void Task_Run(void)
{
    Control_Output_t out;

    if (MPU_Flag != 0u)
    {
        MPU_Flag = 0u;

        MPU6000_DataChange();
        MPU6000_GetEuler_Mahony(&pitch, &roll, &yaw);
        MPU6000_GetBodyRate(&pitch_rate_dps, &roll_rate_dps, &yaw_rate_dps);
    }

    if (Control_Flag != 0u)
    {
        Control_Flag = 0u;
        Task_UpdateCommandFromRemote();
        Control_Update(pitch, roll, pitch_rate_dps, roll_rate_dps, yaw_rate_dps);
    }

    if (Usart_Flag != 0u)
    {
        Usart_Flag = 0u;
        Control_GetOutput(&out);

        printf("pit:%.2f rol:%.2f yaw:%.2f arm:%d nrf:%d link:%d m1:%u m2:%u s1:%u s2:%u\r\n",
               pitch, roll, yaw,
               Control_GetArmed(), nrf_ready,
               (remote_timeout_ms < CONTROL_REMOTE_LINK_TIMEOUT_MS) ? 1 : 0,
               out.motor1_pwm, out.motor2_pwm,
               out.servo1_us, out.servo2_us);
    }
}
