#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/* ================= I2C ================= */

#define MOTOR_I2C_ADDRESS             0x34U

/* ================= REGISTER ================= */

#define MOTOR_TYPE_ADDR               20U
#define MOTOR_ENCODER_POLARITY_ADDR   21U

#define MOTOR_FIXED_PWM_ADDR          31U
#define MOTOR_FIXED_SPEED_ADDR        51U

#define MOTOR_ENCODER_TOTAL_ADDR      60U
#define MOTOR_ADC_BAT_ADDR             0U

/* ================= MOTOR TYPE ================= */

#define MOTOR_TYPE_WITHOUT_ENCODER     0U
#define MOTOR_TYPE_TT                  1U
#define MOTOR_TYPE_N20                 2U
#define MOTOR_TYPE_JGB                 3U

/* ================= MOTOR CONFIG ================= */

/* Gia su banh trai noi M1, banh phai noi M2 */
#define MOTOR_LEFT_INDEX              0U
#define MOTOR_RIGHT_INDEX             1U

// Đặt -1 cho bánh bị đảo chiều khi lắp; áp dụng cùng dấu cho lệnh và encoder.
#ifndef MOTOR_LEFT_DIRECTION
#define MOTOR_LEFT_DIRECTION          -1
#endif
#ifndef MOTOR_RIGHT_DIRECTION
#define MOTOR_RIGHT_DIRECTION         -1
#endif

#define MOTOR_ENCODER_POLARITY        0U

// Đơn vị xung encoder/10 ms, cần hiệu chỉnh theo motor thực tế; không phải % PWM.
#define MOTOR_MAX_SPEED               100
#define MOTOR_MIN_SPEED              -100

#define MOTOR_POWER_ON_DELAY_MS       200U
#define MOTOR_CONFIG_DELAY_MS           5U
#define MOTOR_INIT_RETRIES              3U
#define MOTOR_RETRY_DELAY_MS           20U

/**
 * @brief Khoi tao module dieu khien dong co Hiwonder.
 *
 * @return 1 thanh cong, 0 that bai.
 */
// Gọi từ main sau SysTick_Init, khi ngắt đang bật; chỉ chờ ở giai đoạn khởi tạo.
uint8_t Motor_Init(void);

/**
 * @brief Dieu khien toc do vong kin hai dong co.
 *
 * @param left Toc do muc tieu banh trai.
 * @param right Toc do muc tieu banh phai.
 *
 * @return 1 thanh cong, 0 that bai.
 *
 * @note Don vi xung encoder/10 ms; dau duong theo chieu tien da hieu chinh.
 */
uint8_t Motor_SetSpeed(int8_t left, int8_t right);

/**
 * @brief Dieu khien toc do dong co trai.
 *
 * @param speed Gia tri toc do.
 * @return 1 thanh cong, 0 that bai.
 */
uint8_t Motor_SetLeftSpeed(int8_t speed);

/**
 * @brief Dieu khien toc do dong co phai.
 *
 * @param speed Gia tri toc do.
 * @return 1 thanh cong, 0 that bai.
 */
uint8_t Motor_SetRightSpeed(int8_t speed);

/**
 * @brief Doc gia tri encoder hai dong co.
 *
 * @param left Con tro luu encoder trai.
 * @param right Con tro luu encoder phai.
 *
 * @return 1 thanh cong, 0 that bai.
 */
uint8_t Motor_ReadEncoder(int32_t *left, int32_t *right);

/**
 * @brief Doc dien ap module dong co.
 *
 * @param voltage_mv Con tro luu dien ap mV.
 * @return 1 thanh cong, 0 that bai.
 */
uint8_t Motor_ReadVoltage(uint16_t *voltage_mv);

/**
 * @brief Dung tat ca dong co.
 *
 * @return 1 thanh cong, 0 that bai.
 */
uint8_t Motor_Stop(void);

#endif
