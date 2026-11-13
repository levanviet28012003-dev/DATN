#include "motor.h"
#include "I2C.h"
#include "systick.h"

/* ================= BIEN NOI BO ================= */

static int8_t motor_left_speed = 0;
static int8_t motor_right_speed = 0;
static uint8_t motor_ready;

_Static_assert(MOTOR_LEFT_INDEX < 4U && MOTOR_RIGHT_INDEX < 4U &&
               MOTOR_LEFT_INDEX != MOTOR_RIGHT_INDEX, "Cong motor khong hop le");
_Static_assert((MOTOR_LEFT_DIRECTION == 1 || MOTOR_LEFT_DIRECTION == -1) &&
               (MOTOR_RIGHT_DIRECTION == 1 || MOTOR_RIGHT_DIRECTION == -1),
               "Chieu motor phai bang 1 hoac -1");
_Static_assert(MOTOR_MIN_SPEED >= -127 && MOTOR_MIN_SPEED <= 0 &&
               MOTOR_MAX_SPEED <= 127 && MOTOR_MAX_SPEED >= 0 &&
               MOTOR_INIT_RETRIES > 0U && MOTOR_INIT_RETRIES <= 255U,
               "Cau hinh motor khong hop le");

static void Motor_DelayMs(uint32_t delay_ms)
{
    uint32_t start = SysTick_Millis();
    while ((uint32_t)(SysTick_Millis() - start) < delay_ms) {}
}

/**
 * @brief Gioi han lenh toc do dong co.
 */
static int8_t Motor_LimitSpeed(int8_t speed)
{
    if (speed > MOTOR_MAX_SPEED)
    {
        return MOTOR_MAX_SPEED;
    }

    if (speed < MOTOR_MIN_SPEED)
    {
        return MOTOR_MIN_SPEED;
    }

    return speed;
}

/**
 * @brief Gui lenh toc do cho 4 cong dong co.
 */
static uint8_t Motor_WriteSpeed(int8_t left, int8_t right)
{
    int8_t speed[4] = {0, 0, 0, 0};

    if (!motor_ready)
    {
        return 0U;
    }

    /* Gioi han toc do */
    left = Motor_LimitSpeed(left);
    right = Motor_LimitSpeed(right);

    /* Gan toc do cho tung cong */
    speed[MOTOR_LEFT_INDEX] = (int8_t)(left * MOTOR_LEFT_DIRECTION);
    speed[MOTOR_RIGHT_INDEX] = (int8_t)(right * MOTOR_RIGHT_DIRECTION);

    /* Gui lenh dieu khien vong kin */
    if (!I2C_WriteReg(
            MOTOR_I2C_ADDRESS,
            MOTOR_FIXED_SPEED_ADDR,
            (const uint8_t *)speed,
            4U))
    {
        return 0U;
    }

    motor_left_speed = left;
    motor_right_speed = right;

    return 1U;
}

/**
 * @brief Khoi tao module dieu khien dong co.
 *
 * @details
 * - Khoi tao I2C1.
 * - Cau hinh dong co JGB.
 * - Cau hinh chieu encoder.
 * - Dat toc do ban dau bang 0.
 *
 * @return 1 thanh cong, 0 that bai.
 */
uint8_t Motor_Init(void)
{
    uint8_t motor_type = MOTOR_TYPE_JGB;
    uint8_t polarity = MOTOR_ENCODER_POLARITY;
    uint8_t attempt;

    motor_ready = 0U;
    motor_left_speed = 0;
    motor_right_speed = 0;

    if (!SysTick_IsReady())
    {
        return 0U;
    }

    I2C_Init();
    Motor_DelayMs(MOTOR_POWER_ON_DELAY_MS);

    for (attempt = 0U; attempt < MOTOR_INIT_RETRIES; ++attempt)
    {
        // Dừng lệnh cũ trước khi cấu hình lại module.
        if (Motor_Stop() && I2C_WriteReg(MOTOR_I2C_ADDRESS, MOTOR_TYPE_ADDR,
                                       &motor_type, 1U))
        {
            Motor_DelayMs(MOTOR_CONFIG_DELAY_MS);
            if (I2C_WriteReg(MOTOR_I2C_ADDRESS, MOTOR_ENCODER_POLARITY_ADDR,
                            &polarity, 1U))
            {
                Motor_DelayMs(MOTOR_CONFIG_DELAY_MS);
                if (Motor_Stop())
                {
                    motor_ready = 1U;
                    return 1U;
                }
            }
        }
        if (attempt + 1U < MOTOR_INIT_RETRIES)
        {
            Motor_DelayMs(MOTOR_RETRY_DELAY_MS);
            I2C_Init();
        }
    }
    (void)Motor_Stop();
    return 0U;
}

/**
 * @brief Dieu khien toc do vong kin hai dong co.
 */
uint8_t Motor_SetSpeed(int8_t left, int8_t right)
{
    return Motor_WriteSpeed(left, right);
}

/**
 * @brief Dieu khien rieng dong co trai.
 */
uint8_t Motor_SetLeftSpeed(int8_t speed)
{
    return Motor_WriteSpeed(speed, motor_right_speed);
}

/**
 * @brief Dieu khien rieng dong co phai.
 */
uint8_t Motor_SetRightSpeed(int8_t speed)
{
    return Motor_WriteSpeed(motor_left_speed, speed);
}

/**
 * @brief Chuyen 4 byte little-endian thanh int32_t.
 */
static int32_t Motor_ConvertEncoder(const uint8_t *data, int8_t direction)
{
    uint32_t value;

    value = ((uint32_t)data[0])       |
            ((uint32_t)data[1] << 8)  |
            ((uint32_t)data[2] << 16) |
            ((uint32_t)data[3] << 24);

    // Phép trừ không dấu giữ đúng quy tắc tràn bộ đếm, kể cả INT32_MIN.
    if (direction < 0)
    {
        value = 0U - value;
    }

    if (value <= 0x7FFFFFFFUL)
    {
        return (int32_t)value;
    }

    return (int32_t)((int64_t)value - 4294967296LL);
}

/**
 * @brief Doc xung encoder hai dong co.
 */
uint8_t Motor_ReadEncoder(int32_t *left, int32_t *right)
{
    uint8_t data[16];

    if (!motor_ready || left == 0 || right == 0)
    {
        return 0U;
    }

    if (MOTOR_LEFT_INDEX >= 4U ||
        MOTOR_RIGHT_INDEX >= 4U)
    {
        return 0U;
    }

    /* Doc 16 byte encoder cua 4 cong */
    if (!I2C_ReadReg(
            MOTOR_I2C_ADDRESS,
            MOTOR_ENCODER_TOTAL_ADDR,
            data,
            16U))
    {
        return 0U;
    }

    /* Lay encoder banh trai */
    *left = Motor_ConvertEncoder(
        &data[MOTOR_LEFT_INDEX * 4U], MOTOR_LEFT_DIRECTION
    );

    /* Lay encoder banh phai */
    *right = Motor_ConvertEncoder(
        &data[MOTOR_RIGHT_INDEX * 4U], MOTOR_RIGHT_DIRECTION
    );

    return 1U;
}

/**
 * @brief Doc dien ap nguon module driver.
 */
uint8_t Motor_ReadVoltage(uint16_t *voltage_mv)
{
    uint8_t data[2];

    if (!motor_ready || voltage_mv == 0)
    {
        return 0U;
    }

    if (!I2C_ReadReg(
            MOTOR_I2C_ADDRESS,
            MOTOR_ADC_BAT_ADDR,
            data,
            2U))
    {
        return 0U;
    }

    *voltage_mv = (uint16_t)data[0] |
                  ((uint16_t)data[1] << 8);

    return 1U;
}

/**
 * @brief Dung tat ca dong co.
 */
uint8_t Motor_Stop(void)
{
    uint8_t speed[4] = {0, 0, 0, 0};

    if (!I2C_WriteReg(
            MOTOR_I2C_ADDRESS,
            MOTOR_FIXED_SPEED_ADDR,
            speed,
            4U))
    {
        return 0U;
    }

    motor_left_speed = 0;
    motor_right_speed = 0;

    return 1U;
}
