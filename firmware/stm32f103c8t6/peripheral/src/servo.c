#include "servo.h"
#include "TIM.h"

_Static_assert(SERVO_MIN_ANGLE < SERVO_CENTER_ANGLE &&
               SERVO_CENTER_ANGLE < SERVO_MAX_ANGLE &&
               SERVO_CENTER_ANGLE - SERVO_MIN_ANGLE >= SERVO_STEERING_LIMIT &&
               SERVO_MAX_ANGLE - SERVO_CENTER_ANGLE >= SERVO_STEERING_LIMIT,
               "Gioi han goc servo khong hop le");
_Static_assert(SERVO_MIN_PULSE < SERVO_CENTER_PULSE &&
               SERVO_CENTER_PULSE < SERVO_MAX_PULSE &&
               SERVO_MAX_PULSE < 20000U && SERVO_PWM_FREQUENCY == 50U,
               "Thong so PWM servo khong hop le");

static uint8_t servo_ready;

static uint16_t Servo_AngleToPulse(uint16_t angle)
{
    uint32_t pulse;

    // Nội suy quanh xung giữa, làm tròn về phía giữa để giữ giới hạn góc.
    if (angle <= SERVO_CENTER_ANGLE)
    {
        pulse = SERVO_CENTER_PULSE -
            (uint32_t)(SERVO_CENTER_PULSE - SERVO_MIN_PULSE) *
            (SERVO_CENTER_ANGLE - angle) / (SERVO_CENTER_ANGLE - SERVO_MIN_ANGLE);
    }
    else
    {
        pulse = SERVO_CENTER_PULSE +
            (uint32_t)(SERVO_MAX_PULSE - SERVO_CENTER_PULSE) *
            (angle - SERVO_CENTER_ANGLE) / (SERVO_MAX_ANGLE - SERVO_CENTER_ANGLE);
    }
    return (uint16_t)pulse;
}

void Servo_Init(void)
{
    servo_ready = 0U;
    TIM2_PWM_Init();
    servo_ready = 1U;
    Servo_Center();
}

/**
 * @brief Dieu khien Servo bang do rong xung.
 */
void Servo_SetPulse(uint16_t pulse)
{
    uint16_t min_pulse = Servo_AngleToPulse(SERVO_STEERING_MIN_ANGLE);
    uint16_t max_pulse = Servo_AngleToPulse(SERVO_STEERING_MAX_ANGLE);

    if (!servo_ready)
    {
        return;
    }

    // Cả lệnh đặt xung trực tiếp cũng giữ biên đánh lái ±40 độ.
    if (pulse < min_pulse)
    {
        pulse = min_pulse;
    }
    else if (pulse > max_pulse)
    {
        pulse = max_pulse;
    }

    TIM2_PWM_SetPulse(pulse);
}

/**
 * @brief Dieu khien goc quay Servo.
 */
void Servo_SetAngle(uint16_t angle)
{
    if (angle < SERVO_STEERING_MIN_ANGLE)
    {
        angle = SERVO_STEERING_MIN_ANGLE;
    }
    else if (angle > SERVO_STEERING_MAX_ANGLE)
    {
        angle = SERVO_STEERING_MAX_ANGLE;
    }
    Servo_SetPulse(Servo_AngleToPulse(angle));
}

/**
 * @brief Dua Servo ve vi tri trung tam.
 */
void Servo_Center(void)
{
    Servo_SetPulse(SERVO_CENTER_PULSE);
}
