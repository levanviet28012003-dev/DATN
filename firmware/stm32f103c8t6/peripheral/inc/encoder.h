#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/* ================= ENCODER CONFIG ================= */

/**
 * @brief So xung encoder tren mot vong quay banh xe.
 */
#define ENCODER_PPR             3960.0f

/**
 * @brief Duong kinh banh xe (met).
 */
#define ENCODER_WHEEL_DIAMETER  0.07f

/**
 * @brief Chu ky cap nhat encoder (ms).
 */
#define ENCODER_SAMPLE_TIME_MS  20U

/**
 * @brief Hang so PI.
 */
#define ENCODER_PI              3.14159265358979323846f

/* ================= DATA STRUCTURE ================= */

/**
 * @brief Cau truc luu thong tin mot encoder.
 */
typedef struct
{
    int32_t count;          /**< So xung tich luy hien tai */
    int32_t previous_count; /**< So xung tai lan doc truoc */
    int32_t delta_count;    /**< So xung thay doi */

    float rpm;              /**< Toc do quay (vong/phut) */
    float velocity;         /**< Van toc banh xe (m/s) */
    float distance;         /**< Quang duong co dau (m) */

} Encoder_Data_t;

/* ================= FUNCTION PROTOTYPES ================= */

/**
 * @brief Khoi tao bo xu ly encoder.
 */
void Encoder_Init(void);

/**
 * @brief Doc encoder va cap nhat toc do hai banh.
 */
uint8_t Encoder_Update(uint32_t now_ms);

/**
 * @brief Lay du lieu encoder 2 banh.
 */
const Encoder_Data_t *Encoder_GetLeftData(void);
const Encoder_Data_t *Encoder_GetRightData(void);

/**
 * @brief Lay toc do quay 2 banh.
 */
float Encoder_GetLeftRPM(void);
float Encoder_GetRightRPM(void);

/**
 * @brief Lay van toc 2 banh.
 */
float Encoder_GetLeftVelocity(void);
float Encoder_GetRightVelocity(void);

/**
 * @brief Lay van toc trung binh hai banh sau.
 */
float Encoder_GetVehicleVelocity(void);

/**
 * @brief Lay quang duong tich luy 2 banh.
 */
float Encoder_GetLeftDistance(void);
float Encoder_GetRightDistance(void);

/**
 * @brief Dat lai quang duong tich luy ve 0.
 */
void Encoder_ResetDistance(void);

/**
 * @brief Kiem tra da co du lieu toc do hop le.
 */
uint8_t Encoder_IsValid(void);

#endif