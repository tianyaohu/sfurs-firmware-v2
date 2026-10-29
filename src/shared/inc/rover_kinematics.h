#ifndef ROVER_KINEMATICS_H
#define ROVER_KINEMATICS_H

/** @file rover_kinematics.h
 *  @brief Omni inverse kinematics: body twist to four wheel motor RPM.
 *
 *  Body frame: +x right, +y forward, yaw CCW positive from above.
 *  Wheel order is FR, BR, BL, FL.
 */

enum {
    ROVER_WHEEL_FR = 0,
    ROVER_WHEEL_BR,
    ROVER_WHEEL_BL,
    ROVER_WHEEL_FL,
    ROVER_WHEEL_COUNT
};

/** @brief Wheel radius, m (65.5 mm diameter, direct drive). */
#define ROVER_WHEEL_RADIUS_M 0.03275f

/**
 * @brief Positive magnitude cap on raw motor RPM.
 *
 * A twist that would exceed this is scaled on every wheel together.
 */
#define ROVER_MOTOR_RPM_LIMIT 900.0f

/**
 * @brief Contact point and rolling direction of one wheel.
 *
 * dir_x and dir_y are a unit vector for positive wheel travel, in the body
 * frame, before motor_sign. motor_sign is +1 or -1 and converts that travel
 * into raw motor RPM. Left wheels are mirrored, so their sign is -1.
 */
struct rover_wheel {
    float x_m;
    float y_m;
    float dir_x;
    float dir_y;
    float motor_sign;
};

/** @brief Provisional wheel map the drive will use. */
extern const struct rover_wheel rover_wheels[ROVER_WHEEL_COUNT];

/**
 * @brief Inverse kinematics for one body twist.
 *
 * @param vx_m_s        lateral velocity, +x right, m/s
 * @param vy_m_s        forward velocity, +y, m/s
 * @param omega_rad_s   yaw rate, CCW positive, rad/s
 * @param rpm           out: motor RPM, FR/BR/BL/FL; must hold four floats
 * @param applied_scale optional out: common scale in (0, 1]; 0 on failure
 * @return 0 on success, -1 if @p rpm is NULL or an input is non-finite
 *
 * Outputs are raw signed motor RPM. A transport must not flip signs again.
 * When any wheel would pass ::ROVER_MOTOR_RPM_LIMIT, every wheel is multiplied
 * by the same factor so the twist direction is unchanged and the fastest
 * wheel lands on the limit. Zero stays zero: there is no speed floor.
 */
int rover_twist_to_rpm(float vx_m_s, float vy_m_s, float omega_rad_s,
                       float rpm[ROVER_WHEEL_COUNT], float *applied_scale);

#endif
