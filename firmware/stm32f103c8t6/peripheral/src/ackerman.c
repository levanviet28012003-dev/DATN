#include "ackerman.h"
#include "motor.h"
#include "servo.h"

#include <math.h>

static Ackermann_Data_t ackermann_data;
static float Ackermann_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

/**
 * @brief Chuyen van toc banh xe sang RPM.
 */
static float Ackermann_VelocityToRPM(float velocity)
{
    float circumference;
    circumference = ACKERMAN_PI * ACKERMAN_WHEEL_DIAMETER;
    return (velocity * 60.0f) / circumference;
}

/**
 * @brief Chuyen van toc banh xe sang lenh driver.
 */
static int8_t Ackermann_VelocityToCommand(float velocity)
{
    float command;

    command = (velocity / ACKERMAN_CALIB_VELOCITY) * ACKERMAN_CALIB_COMMAND;

    command = Ackermann_Clamp( command,-ACKERMAN_MOTOR_CMD_LIMIT, ACKERMAN_MOTOR_CMD_LIMIT
    );

    /* Lam tron ve so nguyen */
    if (command >= 0.0f)
    {
        return (int8_t)(command + 0.5f);
    }

    return (int8_t)(command - 0.5f);
}

/**
 * @brief Chuyen goc lai sang do rong xung servo.
 */
static uint16_t Ackermann_AngleToPulse(float angle)
{
    float pulse;
    float ratio;
    angle = Ackermann_Clamp( angle,-ACKERMAN_MAX_STEER_RAD, ACKERMAN_MAX_STEER_RAD );
    if (angle >= 0.0f)
    {
        ratio = angle / ACKERMAN_MAX_STEER_RAD;
        pulse = (float)ACKERMAN_SERVO_CENTER_US + ratio * ((float)ACKERMAN_SERVO_LEFT_US -
             (float)ACKERMAN_SERVO_CENTER_US);
    }
    else
    {
        ratio = (-angle) / ACKERMAN_MAX_STEER_RAD;
        pulse = (float)ACKERMAN_SERVO_CENTER_US + ratio * ((float)ACKERMAN_SERVO_RIGHT_US -
             (float)ACKERMAN_SERVO_CENTER_US);
    }
    return (uint16_t)(pulse + 0.5f);
}

/* =========================================================
 *                     PUBLIC FUNCTIONS
 * ========================================================= */

/**
 * @brief Khoi tao module dong hoc Ackermann.
 */
void Ackermann_Init(void)
{
    ackermann_data = (Ackermann_Data_t){0};
    ackermann_data.servo_pulse = ACKERMAN_SERVO_CENTER_US;
}

/**
 * @brief Tinh van toc hai banh sau theo dong hoc Ackermann.
 */
uint8_t Ackermann_Calculate(float velocity,
                            float steering_angle,
                            Ackermann_Data_t *result)
{
    float omega;
    float left_velocity;
    float right_velocity;
    float max_wheel_velocity;
    float scale;

    /* Kiem tra con tro */
    if (result == 0)
    {
        return 0U;
    }

    /* Khong chap nhan NaN hoac Infinity */
    if (!isfinite(velocity) || !isfinite(steering_angle))
    {
        return 0U;
    }

    /* Kiem tra thong so cau hinh */
    if ((ACKERMAN_WHEEL_BASE <= 0.0f) ||
        (ACKERMAN_TRACK_WIDTH <= 0.0f) ||
        (ACKERMAN_WHEEL_DIAMETER <= 0.0f) ||
        (ACKERMAN_MAX_STEER_RAD <= 0.0f) ||
        (ACKERMAN_CALIB_VELOCITY <= 0.0f) ||
        (ACKERMAN_CALIB_COMMAND <= 0.0f) ||
        (ACKERMAN_MOTOR_CMD_LIMIT <= 0.0f) ||
        (ACKERMAN_MOTOR_CMD_LIMIT > 100.0f))
    {
        return 0U;
    }

    /* Gioi han van toc xe */
    velocity = Ackermann_Clamp( velocity, -ACKERMAN_MAX_VELOCITY, ACKERMAN_MAX_VELOCITY );

    /* Gioi han goc lai */
    steering_angle = Ackermann_Clamp( steering_angle, -ACKERMAN_MAX_STEER_RAD,
        ACKERMAN_MAX_STEER_RAD );

    /*
     * Buoc 1: Tinh van toc goc.
     *
     * omega = v * tan(delta) / L
     */
    omega = velocity * tanf(steering_angle) / ACKERMAN_WHEEL_BASE;

    /*
     * Buoc 2: Tinh van toc muc tieu cua hai banh sau.
     *
     * vL = v - omega * W/2
     * vR = v + omega * W/2
     */
    left_velocity = velocity - omega * ACKERMAN_TRACK_WIDTH * 0.5f;
    right_velocity = velocity + omega * ACKERMAN_TRACK_WIDTH * 0.5f;

    /*
     * Buoc 3: Gioi han van toc hai banh.
     */
    max_wheel_velocity = ACKERMAN_CALIB_VELOCITY * ACKERMAN_MOTOR_CMD_LIMIT /
        ACKERMAN_CALIB_COMMAND;

    if (fmaxf(fabsf(left_velocity), fabsf(right_velocity)) > max_wheel_velocity)
    {
        scale = max_wheel_velocity / fmaxf(fabsf(left_velocity), fabsf(right_velocity));

        left_velocity *= scale;
        right_velocity *= scale;

        velocity *= scale;
        omega *= scale;
    }

    /*
     * Buoc 4: Luu van toc va goc lai.
     */
    result->velocity = velocity;
    result->steering_angle = steering_angle;
    result->angular_velocity = omega;

    /*
     * Buoc 5: Tinh ban kinh quay.
     */
    if (fabsf(steering_angle) < 0.00001f)
    {
        result->turning_radius = INFINITY;
    }
    else
    {
        result->turning_radius = ACKERMAN_WHEEL_BASE / tanf(steering_angle);
    }

    /*
     * Buoc 6: Luu van toc tung banh.
     */
    result->left_velocity = left_velocity;
    result->right_velocity = right_velocity;

    /*
     * Buoc 7: Chuyen van toc thanh RPM.
     */
    result->left_rpm = Ackermann_VelocityToRPM(left_velocity);
    result->right_rpm = Ackermann_VelocityToRPM(right_velocity);

    /*
     * Buoc 8: Chuyen van toc thanh lenh motor.
     */
    result->left_command = Ackermann_VelocityToCommand(left_velocity);
    result->right_command = Ackermann_VelocityToCommand(right_velocity);

    /*
     * Buoc 9: Chuyen goc lai thanh xung servo.
     */
    result->servo_pulse = Ackermann_AngleToPulse(steering_angle);

    return 1U;
}

/**
 * @brief Dieu khien xe theo mo hinh Ackermann.
 */
uint8_t Ackermann_SetCommand(float velocity, float steering_angle)
{
    Ackermann_Data_t result;
    int8_t left_command;
    int8_t right_command;

    /* Tinh dong hoc */
    if (Ackermann_Calculate( velocity, steering_angle, &result) == 0U)
    {
        (void)Ackermann_Stop();
        return 0U;
    }

    /*
     * Xu ly chieu quay cua hai dong co.
     */
    left_command = (int8_t)(result.left_command * ACKERMAN_LEFT_MOTOR_SIGN);

    right_command = (int8_t)(result.right_command * ACKERMAN_RIGHT_MOTOR_SIGN);

    /*
     * Dieu khien goc lai servo.
     */
    Servo_SetPulse(result.servo_pulse);

    /*
     * Gui lenh toc do den driver Hiwonder.
     */
    if (Motor_SetSpeed(left_command, right_command) == 0U)
    {
        (void)Ackermann_Stop();
        return 0U;
    }

    /* Luu du lieu lenh moi nhat */
    ackermann_data = result;

    return 1U;
}

/**
 * @brief Dung hai dong co va dua servo ve giua.
 */
uint8_t Ackermann_Stop(void)
{
    uint8_t status;
    status = Motor_Stop();
    Servo_SetPulse(ACKERMAN_SERVO_CENTER_US);
    ackermann_data = (Ackermann_Data_t){0};
    ackermann_data.servo_pulse = ACKERMAN_SERVO_CENTER_US;

    return status;
}

/**
 * @brief Lay thong tin dong hoc gan nhat.
 */
const Ackermann_Data_t *Ackermann_GetData(void)
{
    return &ackermann_data;
}