#include <math.h>
#include <stdint.h>
#include "RCC.h"
#include "servo.h"
#include "motor.h"
#include "encoder.h"
#include "ackerman.h"
#include "communication.h"
#include "systick.h"

/*
 * Chọn chế độ bằng macro APP_MANUAL_CONTROL (có thể truyền qua -D khi build):
 *  - 1: điều khiển tay cả servo và hai motor qua biến Watch manual_control (thử trên giá kê).
 *  - 0: chạy chính, nhận lệnh từ Pi4 qua UART (watchdog 300 ms, ESTOP), gửi STATE 50 Hz.
 */
#ifndef APP_MANUAL_CONTROL
#define APP_MANUAL_CONTROL 1U
#endif

#define APP_PI_CONTROL (!APP_MANUAL_CONTROL)

#define APP_CONTROL_PERIOD_MS   ENCODER_SAMPLE_TIME_MS
#define APP_VOLTAGE_PERIOD_MS 1000U
#define APP_STOP_RETRY_MS      100U
#define APP_STATE_PERIOD_MS     20U

_Static_assert(APP_CONTROL_PERIOD_MS > 0U, "Chu ky cap nhat phai lon hon 0");

#if APP_MANUAL_CONTROL
// Giới hạn lệnh motor khi đặt riêng từng bánh: bằng giới hạn mà động học Ackermann dùng.
#define APP_MANUAL_MAX_SPEED   ((int16_t)ACKERMAN_MOTOR_CMD_LIMIT)
#define APP_MANUAL_MAX_STEER_DEG (ACKERMAN_MAX_STEER_RAD * 180.0f / ACKERMAN_PI)

_Static_assert(APP_MANUAL_MAX_SPEED > 0 && APP_MANUAL_MAX_SPEED <= MOTOR_MAX_SPEED,
               "Gioi han toc do dieu khien tay khong hop le");

/*
 * Watch: điều khiển tay trên giá kê. Giá trị ngoài giới hạn được kẹp lại và ghi ngược vào biến.
 *  - use_ackermann = 1: lái như xe thật bằng velocity (m/s) và steering_deg (độ, dương = trái);
 *    firmware tính tốc độ hai bánh sau và xung servo qua Ackermann_SetCommand().
 *  - use_ackermann = 0: đặt riêng servo_angle (góc trục servo 50–130 độ) và lệnh từng motor
 *    left_speed/right_speed (xung/10 ms, ±APP_MANUAL_MAX_SPEED); dùng để thử riêng servo hoặc motor.
 *  - enable = 0: motor dừng, servo vẫn theo lệnh lái để thử riêng phần lái.
 */
volatile struct
{
    uint8_t enable;
    uint8_t use_ackermann;
    float velocity;
    float steering_deg;
    uint16_t servo_angle;
    int16_t left_speed;
    int16_t right_speed;
} manual_control = {0U, 1U, 0.0f, 0.0f, SERVO_CENTER_ANGLE, 0, 0};

static float App_ClampFloat(float value, float limit)
{
    // Giá trị NaN/vô cùng gõ nhầm trong Watch được coi là 0.
    if (!isfinite(value))
    {
        return 0.0f;
    }
    return (value > limit) ? limit : ((value < -limit) ? -limit : value);
}

static int8_t App_ClampManualSpeed(volatile int16_t *command)
{
    int16_t speed = *command;
    if (speed > APP_MANUAL_MAX_SPEED)
    {
        speed = APP_MANUAL_MAX_SPEED;
    }
    else if (speed < -APP_MANUAL_MAX_SPEED)
    {
        speed = -APP_MANUAL_MAX_SPEED;
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
    uint8_t driving;
    int32_t encoder_left;
    int32_t encoder_right;
    uint16_t voltage_mv;
    uint32_t encoder_updated_ms;
    uint32_t voltage_updated_ms;
    uint32_t last_stop_attempt_ms;
} app_status;

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
    app_status.driving = 0U;
#if APP_MANUAL_CONTROL
    // Sau khi reset lỗi, xe không tự chạy lại lệnh cũ.
    manual_control.enable = 0U;
    manual_control.velocity = 0.0f;
    manual_control.left_speed = 0;
    manual_control.right_speed = 0;
#endif
    Servo_Center();
    App_SendStop();
}

// Gửi lệnh tới servo và motor; trả về 0 nếu giao dịch I2C lỗi.
static uint8_t App_ApplyCommand(uint32_t now)
{
#if APP_PI_CONTROL
    Comm_Command_t command;
    const Ackermann_Data_t *applied;
    uint8_t ok;

    if (Comm_GetCommand(now, &command))
    {
        app_status.driving = 1U;
        ok = Ackermann_SetCommand(command.velocity, command.steering_angle);
    }
    else
    {
        // Mất lệnh, enable = 0 hoặc ESTOP: dừng motor và đưa lái về giữa.
        app_status.driving = 0U;
        ok = Ackermann_Stop();
    }
    applied = Ackermann_GetData();
    app_status.stop_command_ok = (uint8_t)(ok && applied->left_command == 0 &&
                                           applied->right_command == 0);
    return ok;
#else
    (void)now;
    if (manual_control.use_ackermann)
    {
        const Ackermann_Data_t *applied;
        float velocity = App_ClampFloat(manual_control.velocity, ACKERMAN_MAX_VELOCITY);
        float steering_deg = App_ClampFloat(manual_control.steering_deg, APP_MANUAL_MAX_STEER_DEG);
        uint8_t ok;

        manual_control.velocity = velocity;
        manual_control.steering_deg = steering_deg;
        if (!manual_control.enable)
        {
            velocity = 0.0f;
        }
        ok = Ackermann_SetCommand(velocity, steering_deg * (ACKERMAN_PI / 180.0f));
        applied = Ackermann_GetData();
        app_status.driving = (uint8_t)(applied->left_command != 0 || applied->right_command != 0);
        app_status.stop_command_ok = (uint8_t)(ok && !app_status.driving);
        return ok;
    }
    else
    {
        uint16_t angle = manual_control.servo_angle;
        int8_t target_left = App_ClampManualSpeed(&manual_control.left_speed);
        int8_t target_right = App_ClampManualSpeed(&manual_control.right_speed);

        if (angle < SERVO_STEERING_MIN_ANGLE)
        {
            angle = SERVO_STEERING_MIN_ANGLE;
        }
        else if (angle > SERVO_STEERING_MAX_ANGLE)
        {
            angle = SERVO_STEERING_MAX_ANGLE;
        }
        manual_control.servo_angle = angle;
        Servo_SetAngle(angle);

        if (!manual_control.enable)
        {
            target_left = 0;
            target_right = 0;
        }
        if (!Motor_SetSpeed(target_left, target_right))
        {
            return 0U;
        }
        app_status.driving = (uint8_t)(target_left != 0 || target_right != 0);
        app_status.stop_command_ok = (uint8_t)!app_status.driving;
        return 1U;
    }
#endif
}

#if APP_PI_CONTROL
static void App_SendState(uint32_t now)
{
    Protocol_State_t state;
    uint8_t status = 0U;

    if (app_status.state == APP_READY)
    {
        status |= PROTOCOL_STATUS_READY;
    }
    if (app_status.state == APP_FAULT)
    {
        status |= PROTOCOL_STATUS_FAULT;
    }
    if (Comm_IsEstopActive())
    {
        status |= PROTOCOL_STATUS_ESTOP;
    }
    if (Comm_IsCommandTimedOut(now))
    {
        status |= PROTOCOL_STATUS_CMD_TIMEOUT;
    }
    if (app_status.driving)
    {
        status |= PROTOCOL_STATUS_DRIVING;
    }
    if (app_status.encoder_valid)
    {
        status |= PROTOCOL_STATUS_ENCODER_VALID;
    }
    if (app_status.voltage_valid)
    {
        status |= PROTOCOL_STATUS_VOLTAGE_VALID;
    }

    state.encoder_left = app_status.encoder_left;
    state.encoder_right = app_status.encoder_right;
    state.voltage_mv = app_status.voltage_mv;
    state.status = status;
    (void)Comm_SendState(&state);
}
#endif

int main(void)
{
    uint32_t control_tick;
    uint32_t voltage_tick;
#if APP_PI_CONTROL
    uint32_t state_tick;
#endif

    RCC_Config_72Mhz();
    if (!SysTick_Init())
    {
        app_status.state = APP_FAULT;
        app_status.error = APP_ERROR_CLOCK;
        while (1) {}
    }
    Servo_Init();
    Ackermann_Init();
#if APP_PI_CONTROL
    // Bật UART trước khi khởi tạo motor (mất khoảng 200 ms) để Pi thấy STATE sớm.
    Comm_Init();
#endif
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
#if APP_PI_CONTROL
    state_tick = control_tick;
#endif

    while (1)
    {
        uint32_t now = SysTick_Millis();

#if APP_PI_CONTROL
        Comm_Poll(now);
#endif

        if (app_status.state == APP_FAULT)
        {
            // Giữ lỗi đến khi reset, thử gửi dừng mỗi 100 ms.
            if ((uint32_t)(now - app_status.last_stop_attempt_ms) >= APP_STOP_RETRY_MS)
            {
                App_SendStop();
            }
        }
        else
        {
            if ((uint32_t)(now - control_tick) >= APP_CONTROL_PERIOD_MS)
            {
                control_tick = now;
#if APP_PI_CONTROL
                // Lấy cả byte đến trong lúc chờ để không dùng lệnh đã cũ.
                Comm_Poll(now);
#endif
                if (!App_ApplyCommand(now))
                {
                    App_LatchFault(APP_ERROR_COMMAND);
                }
                else if (!App_UpdateEncoder())
                {
                    App_LatchFault(APP_ERROR_ENCODER);
                }
            }

            now = SysTick_Millis();
            if (app_status.state != APP_FAULT &&
                (uint32_t)(now - voltage_tick) >= APP_VOLTAGE_PERIOD_MS)
            {
                uint16_t voltage;
                voltage_tick = now;
                if (!Motor_ReadVoltage(&voltage))
                {
                    App_LatchFault(APP_ERROR_VOLTAGE);
                }
                else
                {
                    app_status.voltage_mv = voltage;
                    app_status.voltage_updated_ms = SysTick_Millis();
                    app_status.voltage_valid = 1U;
                }
            }
        }

#if APP_PI_CONTROL
        // Gửi STATE 50 Hz, kể cả khi đang lỗi, để Pi biết lý do xe dừng.
        now = SysTick_Millis();
        if ((uint32_t)(now - state_tick) >= APP_STATE_PERIOD_MS)
        {
            state_tick = now;
            App_SendState(now);
        }
#endif
    }
}
