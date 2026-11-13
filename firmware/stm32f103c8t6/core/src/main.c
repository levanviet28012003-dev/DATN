#include <stdint.h>
#include "RCC.h"
#include "servo.h"
#include "motor.h"
#include "encoder.h"
#include "systick.h"

// Chỉ thử servo khi đặt 1.
#ifndef APP_SERVO_TEST_ONLY
#define APP_SERVO_TEST_ONLY 1U
#endif

// Đặt cả hai chế độ riêng bằng 0 để thử servo và hai motor cùng lúc.
#ifndef APP_REAR_MOTOR_TEST_ONLY
#define APP_REAR_MOTOR_TEST_ONLY 0U
#endif

#if APP_SERVO_TEST_ONLY && APP_REAR_MOTOR_TEST_ONLY
#error "Chi chon mot che do thu rieng servo hoac motor"
#endif

#if !APP_REAR_MOTOR_TEST_ONLY
// Watch: góc servo 50–130 độ.
volatile uint16_t servo_test_angle = SERVO_CENTER_ANGLE;

static void App_UpdateServo(void)
{
    uint16_t angle = servo_test_angle;
    if (angle < SERVO_STEERING_MIN_ANGLE)
    {
        angle = SERVO_STEERING_MIN_ANGLE;
    }
    else if (angle > SERVO_STEERING_MAX_ANGLE)
    {
        angle = SERVO_STEERING_MAX_ANGLE;
    }
    servo_test_angle = angle;
    Servo_SetAngle(angle);
}
#endif

#define APP_CONTROL_PERIOD_MS   ENCODER_SAMPLE_TIME_MS
#define APP_VOLTAGE_PERIOD_MS 1000U
#define APP_STOP_RETRY_MS      100U
#define APP_TEST_SPEED          10

_Static_assert(APP_CONTROL_PERIOD_MS > 0U, "Chu ky cap nhat phai lon hon 0");

#if !APP_SERVO_TEST_ONLY
// Watch: đặt tốc độ, enable = 1 để chạy, 0 để dừng.
volatile int16_t motor_test_left_speed = 0;
volatile int16_t motor_test_right_speed = 0;
volatile uint8_t motor_test_enable = 0U;

_Static_assert(APP_TEST_SPEED > 0 && APP_TEST_SPEED <= MOTOR_MAX_SPEED &&
               -APP_TEST_SPEED >= MOTOR_MIN_SPEED, "Toc do thu khong hop le");

static int8_t App_ReadTestSpeed(volatile int16_t *command)
{
    int16_t speed = *command;
    if (speed > APP_TEST_SPEED)
    {
        speed = APP_TEST_SPEED;
    }
    else if (speed < -APP_TEST_SPEED)
    {
        speed = -APP_TEST_SPEED;
    }
    *command = speed;
    return (int8_t)speed;
}
#endif

typedef enum
{
    APP_STARTING = 0,
    APP_READY,
    APP_FAULT
} App_State;

typedef enum
{
    APP_ERROR_NONE = 0,
    APP_ERROR_CLOCK,
    APP_ERROR_MOTOR_INIT,
    APP_ERROR_COMMAND,
    APP_ERROR_ENCODER,
    APP_ERROR_VOLTAGE
} App_Error;

// Trạng thái hệ thống trong Watch.
volatile struct
{
    App_State state;
    App_Error error;
    uint32_t i2c_error_count;
    uint8_t stop_command_ok;
    uint8_t encoder_valid;
    uint8_t voltage_valid;
    int32_t encoder_left;
    int32_t encoder_right;
    uint16_t voltage_mv;
    uint32_t encoder_updated_ms;
    uint32_t voltage_updated_ms;
    uint32_t last_stop_attempt_ms;
} app_status;

#if !APP_SERVO_TEST_ONLY
// Watch: dữ liệu từng bánh, vận tốc xe và cờ hợp lệ.
volatile struct
{
    Encoder_Data_t left;
    Encoder_Data_t right;
    float vehicle_velocity;
    uint32_t updated_ms;
    uint8_t valid;
} encoder_status;

static void App_CopyEncoder(volatile Encoder_Data_t *destination,
                            const Encoder_Data_t *source)
{
    destination->count = source->count;
    destination->previous_count = source->previous_count;
    destination->delta_count = source->delta_count;
    destination->rpm = source->rpm;
    destination->velocity = source->velocity;
    destination->distance = source->distance;
}

static uint8_t App_UpdateEncoder(void)
{
    uint32_t now = SysTick_Millis();
    if (!Encoder_Update(now))
    {
        return 0U;
    }

    App_CopyEncoder(&encoder_status.left, Encoder_GetLeftData());
    App_CopyEncoder(&encoder_status.right, Encoder_GetRightData());
    encoder_status.vehicle_velocity = Encoder_GetVehicleVelocity();
    encoder_status.updated_ms = now;
    encoder_status.valid = Encoder_IsValid();
    app_status.encoder_left = encoder_status.left.count;
    app_status.encoder_right = encoder_status.right.count;
    app_status.encoder_updated_ms = now;
    app_status.encoder_valid = encoder_status.valid;
    return 1U;
}

static void App_SendStop(void)
{
    app_status.stop_command_ok = Motor_Stop();
    if (!app_status.stop_command_ok)
    {
        ++app_status.i2c_error_count;
    }
    app_status.last_stop_attempt_ms = SysTick_Millis();
}

static void App_LatchFault(App_Error error)
{
    app_status.state = APP_FAULT;
    app_status.error = error;
    ++app_status.i2c_error_count;
    encoder_status.valid = 0U;
    app_status.encoder_valid = 0U;
    app_status.voltage_valid = 0U;
    motor_test_enable = 0U;
    motor_test_left_speed = 0;
    motor_test_right_speed = 0;
#if !APP_REAR_MOTOR_TEST_ONLY
    servo_test_angle = SERVO_CENTER_ANGLE;
    Servo_Center();
#endif
    App_SendStop();
}
#endif

int main(void)
{
#if !APP_SERVO_TEST_ONLY
    uint32_t control_tick;
    uint32_t voltage_tick;
    int8_t target_left = 0;
    int8_t target_right = 0;
#endif

    RCC_Config_72Mhz();
    if (!SysTick_Init())
    {
        app_status.state = APP_FAULT;
        app_status.error = APP_ERROR_CLOCK;
        while (1) {}
    }
#if !APP_REAR_MOTOR_TEST_ONLY
    Servo_Init();
#endif

#if APP_SERVO_TEST_ONLY
    uint32_t servo_tick = SysTick_Millis();
    app_status.state = APP_READY;

    // Cập nhật servo mỗi 20 ms.
    while (1)
    {
        uint32_t now = SysTick_Millis();
        if ((uint32_t)(now - servo_tick) >= APP_CONTROL_PERIOD_MS)
        {
            servo_tick = now;
            App_UpdateServo();
        }
    }
#else
    Encoder_Init();
    if (!Motor_Init())
    {
        App_LatchFault(APP_ERROR_MOTOR_INIT);
    }
    else if (!App_UpdateEncoder())
    {
        App_LatchFault(APP_ERROR_ENCODER);
    }
    else
    {
        app_status.state = APP_READY;
        app_status.stop_command_ok = 1U;
    }

    control_tick = SysTick_Millis();
    voltage_tick = control_tick;

    while (1)
    {
        uint32_t now = SysTick_Millis();

        // Giữ lỗi đến khi reset, thử gửi dừng mỗi 100 ms.
        if (app_status.state == APP_FAULT)
        {
            if ((uint32_t)(now - app_status.last_stop_attempt_ms) >= APP_STOP_RETRY_MS)
            {
                App_SendStop();
            }
            continue;
        }

        if ((uint32_t)(now - control_tick) >= APP_CONTROL_PERIOD_MS)
        {
            control_tick = now;
#if !APP_REAR_MOTOR_TEST_ONLY
            App_UpdateServo();
#endif
            if (motor_test_enable)
            {
                target_left = App_ReadTestSpeed(&motor_test_left_speed);
                target_right = App_ReadTestSpeed(&motor_test_right_speed);
            }
            else
            {
                target_left = 0;
                target_right = 0;
            }
            if (!Motor_SetSpeed(target_left, target_right))
            {
                App_LatchFault(APP_ERROR_COMMAND);
                continue;
            }
            app_status.stop_command_ok = (uint8_t)(target_left == 0 && target_right == 0);
            if (!App_UpdateEncoder())
            {
                App_LatchFault(APP_ERROR_ENCODER);
                continue;
            }
        }

        now = SysTick_Millis();
        if ((uint32_t)(now - voltage_tick) >= APP_VOLTAGE_PERIOD_MS)
        {
            uint16_t voltage;
            voltage_tick = now;
            if (!Motor_ReadVoltage(&voltage))
            {
                App_LatchFault(APP_ERROR_VOLTAGE);
                continue;
            }
            app_status.voltage_mv = voltage;
            app_status.voltage_updated_ms = SysTick_Millis();
            app_status.voltage_valid = 1U;
        }
    }
#endif
}
