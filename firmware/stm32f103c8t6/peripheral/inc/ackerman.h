#ifndef ACKERMAN_H
#define ACKERMAN_H

#include <stdint.h>

#define ACKERMAN_PI                  3.14159265358979323846f
#define ACKERMAN_WHEEL_BASE          0.170f
#define ACKERMAN_TRACK_WIDTH         0.180f
#define ACKERMAN_WHEEL_DIAMETER      0.075f
#define ACKERMAN_MAX_STEER_RAD       (10.0f * ACKERMAN_PI / 180.0f)
#define ACKERMAN_MAX_VELOCITY        0.20f

#define ACKERMAN_SERVO_CENTER_US     1500U
#define ACKERMAN_SERVO_LEFT_US       1600U
#define ACKERMAN_SERVO_RIGHT_US      1400U

#define ACKERMAN_CALIB_COMMAND       20.0f
#define ACKERMAN_CALIB_VELOCITY      0.10f
#define ACKERMAN_MOTOR_CMD_LIMIT     30.0f

#define ACKERMAN_LEFT_MOTOR_SIGN     1
#define ACKERMAN_RIGHT_MOTOR_SIGN    1

typedef struct
{
    float velocity;             /**< Van toc xe (m/s) */

    float steering_angle;       /**< Goc lai tuong duong (rad) */

    float angular_velocity;     /**< Van toc goc (rad/s) */

    float turning_radius;       /**< Ban kinh quay co dau (m) */

    float left_velocity;        /**< Van toc banh trai (m/s) */

    float right_velocity;       /**< Van toc banh phai (m/s) */

    float left_rpm;             /**< RPM muc tieu banh trai */

    float right_rpm;            /**< RPM muc tieu banh phai */

    int8_t left_command;        /**< Lenh motor trai */

    int8_t right_command;       /**< Lenh motor phai */

    uint16_t servo_pulse;       /**< Xung dieu khien servo (us) */

} Ackermann_Data_t;

void Ackermann_Init(void);

uint8_t Ackermann_Calculate(float velocity, float steering_angle, Ackermann_Data_t *result);
uint8_t Ackermann_SetCommand(float velocity, float steering_angle);

uint8_t Ackermann_Stop(void);
const Ackermann_Data_t *Ackermann_GetData(void);

#endif