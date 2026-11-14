#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

/**
 * @brief Tan so PWM dieu khien Servo.
 */
#define SERVO_PWM_FREQUENCY    50U

/**
 * @brief Do rong xung PWM (micro giay).
 */
#define SERVO_MIN_PULSE        500U
#define SERVO_MAX_PULSE        2500U
#define SERVO_CENTER_PULSE     1500U

/**
 * @brief Gioi han goc quay Servo.
 */
#define SERVO_MIN_ANGLE        0U
#define SERVO_MAX_ANGLE        180U
#define SERVO_CENTER_ANGLE     90U

// Góc trục servo giới hạn ±40 độ quanh giữa,
#define SERVO_STEERING_LIMIT       40U
#define SERVO_STEERING_MIN_ANGLE   (SERVO_CENTER_ANGLE - SERVO_STEERING_LIMIT)
#define SERVO_STEERING_MAX_ANGLE   (SERVO_CENTER_ANGLE + SERVO_STEERING_LIMIT)

/**
 * @brief Khoi tao Servo su dung TIM2 Channel 1.
 */
void Servo_Init(void);

/**
 * @brief Dieu khien Servo theo do rong xung PWM.
 */
void Servo_SetPulse(uint16_t pulse);

/**
 * @brief Dieu khien Servo theo goc quay.
 */
void Servo_SetAngle(uint16_t angle);

/**
 * @brief Dua Servo ve vi tri trung tam.
 */
void Servo_Center(void);

#endif
