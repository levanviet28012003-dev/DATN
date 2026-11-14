#include "encoder.h"
#include "motor.h"

/* ================= PRIVATE VARIABLES ================= */

static Encoder_Data_t encoder_left;
static Encoder_Data_t encoder_right;
static uint32_t encoder_previous_time = 0U;
static uint8_t encoder_initialized = 0U;
static uint8_t encoder_valid = 0U;

/* ================= PRIVATE FUNCTIONS ================= */

/**
 * @brief Tinh chenh lech bo dem 32-bit.
 */
static int32_t Encoder_CalculateDelta(int32_t current, int32_t previous)
{
    uint32_t delta;

    delta = (uint32_t)current - (uint32_t)previous;

    if (delta <= 0x7FFFFFFFUL)
    {
        return (int32_t)delta;
    }

    return (int32_t)((int64_t)delta - 4294967296LL);
}

/**
 * @brief Cap nhat toc do va quang duong mot banh xe.
 */
static void Encoder_Calculate(Encoder_Data_t *encoder, int32_t new_count, uint32_t dt_ms)
{
    float delta_time;
    float circumference;

    /* Luu gia tri encoder cu */
    encoder->previous_count = encoder->count;

    /* Cap nhat gia tri encoder moi */
    encoder->count = new_count;

    /* Tinh so xung thay doi */
    encoder->delta_count = Encoder_CalculateDelta( encoder->count, encoder->previous_count
    );

    /* Chuyen ms sang giay */
    delta_time = (float)dt_ms / 1000.0f;

    /* Tinh chu vi banh xe */
    circumference = ENCODER_PI * ENCODER_WHEEL_DIAMETER;

    /*
     * RPM = DeltaPulse * 60 / (PPR * DeltaTime)
     */
    encoder->rpm = ((float)encoder->delta_count * 60.0f) / (ENCODER_PPR * delta_time);

    /*
     * Velocity = RPM * PI * D / 60
     */
    encoder->velocity = (encoder->rpm * circumference) / 60.0f;

    /*
     * Distance = DeltaPulse / PPR * PI * D
     */
    encoder->distance += ((float)encoder->delta_count / ENCODER_PPR) * circumference;
}

/* ================= PUBLIC FUNCTIONS ================= */

/**
 * @brief Khoi tao bo xu ly Encoder.
 */
void Encoder_Init(void)
{
    encoder_left = (Encoder_Data_t){0};
    encoder_right = (Encoder_Data_t){0};
    encoder_previous_time = 0U;
    encoder_initialized = 0U;
    encoder_valid = 0U;
}

/**
 * @brief Doc encoder va cap nhat RPM hai banh.
 */
uint8_t Encoder_Update(uint32_t now_ms)
{
    int32_t current_left;
    int32_t current_right;

    uint32_t dt_ms;

    /* Doc encoder tu driver */
    if (!Motor_ReadEncoder(&current_left, &current_right))
    {
        encoder_valid = 0U;
        return 0U;
    }

    /*
     * Lan doc dau tien chi lay gia tri goc.
     */
    if (encoder_initialized == 0U)
    {
        encoder_left.count = current_left;
        encoder_left.previous_count = current_left;
        encoder_right.count = current_right;
        encoder_right.previous_count = current_right;
        encoder_previous_time = now_ms;
        encoder_initialized = 1U;
        encoder_valid = 0U;

        return 1U;
    }

    /*
     * Tinh thoi gian thuc te giua hai lan doc.
     */
    dt_ms = now_ms - encoder_previous_time;

    if (dt_ms == 0U)
    {
        return 0U;
    }

    /*
     * Cap nhat encoder trai.
     */
    Encoder_Calculate( &encoder_left, current_left, dt_ms
    );

    /*
     * Cap nhat encoder phai.
     */
    Encoder_Calculate( &encoder_right, current_right, dt_ms
    );

    /* Luu thoi gian lay mau */
    encoder_previous_time = now_ms;
    encoder_valid = 1U;
    return 1U;
}

/**
 * @brief Lay du lieu encoder 2 banh.
 */
const Encoder_Data_t *Encoder_GetLeftData(void)
{
    return &encoder_left;
}

const Encoder_Data_t *Encoder_GetRightData(void)
{
    return &encoder_right;
}

/**
 * @brief Lay RPM 2 banh.
 
 */
float Encoder_GetLeftRPM(void)
{
    return encoder_left.rpm;
}
float Encoder_GetRightRPM(void)
{
    return encoder_right.rpm;
}

/**
 * @brief Lay van toc 2 banh
 */
float Encoder_GetLeftVelocity(void)
{
    return encoder_left.velocity;
}
float Encoder_GetRightVelocity(void)
{
    return encoder_right.velocity;
}

/**
 * @brief Lay van toc trung binh hai banh sau.
 */
float Encoder_GetVehicleVelocity(void)
{
    return (encoder_left.velocity + encoder_right.velocity) * 0.5f;
}

/**
 * @brief Lay quang duong 2 banh.
 */
float Encoder_GetLeftDistance(void)
{
    return encoder_left.distance;
}
float Encoder_GetRightDistance(void)
{
    return encoder_right.distance;
}

/**
 * @brief Reset quang duong tich luy.
 */
void Encoder_ResetDistance(void)
{
    encoder_left.distance = 0.0f;
    encoder_right.distance = 0.0f;
}

/**
 * @brief Kiem tra tinh hop le cua toc do.
 */
uint8_t Encoder_IsValid(void)
{
    return encoder_valid;
}