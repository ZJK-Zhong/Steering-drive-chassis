#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include <stdint.h>

typedef enum
{
    ROBOT_SAFE = 0,
    ROBOT_ZERO_FORCE,
    ROBOT_FRICTION_ONLY,
    ROBOT_FIRE,
    ROBOT_COMPETITION,
    ROBOT_MOVE,
    ROBOT_SPIN,
    ROBOT_GIMBAL_FIRE,
    ROBOT_RC_DEBUG
} robot_mode_e;

typedef enum
{
    CHASSIS_MODE_ZERO_FORCE = 0,
    CHASSIS_RC_DEBUG,
    CHASSIS_FOLLOW_GIMBAL,
    CHASSIS_SPIN
} chassis_mode_e;

/* Internal contracts, NOT packed CAN wire formats. Angles rad, rates deg/s. */
typedef struct
{
    robot_mode_e robot_mode;
    uint8_t rc_online;
    float vx; /* m/s, forward positive; entry frame in spin, body frame otherwise */
    float vy; /* m/s, left positive */
    float wz; /* deg/s, counterclockwise positive */
} Chassis_Ctrl_Cmd_s;

typedef struct
{
    robot_mode_e robot_mode;
    chassis_mode_e chassis_mode;
    uint8_t rc_online, ins_online, gimbal_online;
    float yaw, gyro_z_dps;
    float vx_set, vy_set, wz_set;
    float wheel_rpm[4], steering_angle_deg[4];
} Chassis_Upload_Data_s;

#endif
