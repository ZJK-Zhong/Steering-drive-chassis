/* (C) COPYRIGHT 2019 DJI. Control-state definitions adapted from chassis_task.h. */
#ifndef CHASSIS_H
#define CHASSIS_H

#include "robot_def.h"
#include "robot_config.h"
#include "CAN_receive.h"
#include "remote_control.h"
#include "pid.h"
#include "user_lib.h"

typedef struct
{
    const motor_measure_t *steering_motor_measure;
    pid_type_def steering_motor_relative_angle_pid;
    int16_t steering_motor_circle;
    uint16_t steering_zero_ecd;
    float steering_last_feedback_deg;
    float steering_ecd_angle, steering_ecd_angle_set;
    uint8_t steering_turn_flag;
    float steering_current_set;
    int16_t steering_given_current;
} steering_motor_t;

typedef struct
{
    const motor_measure_t *chassis_motor_measure;
    float accel, speed, speed_set;
    int16_t give_current;
    float give_current_set;
} chassis_motor_t;

typedef struct
{
    const RC_ctrl_t *chassis_RC; /* Legacy diagnostics only; CMD owns RC input. */
    RC_ctrl_t rc_snapshot;      /* Legacy callers may use as snapshot storage. */
    robot_mode_e robot_mode;
    chassis_mode_e mode;
    uint8_t rc_online, gimbal_online, ins_online;
    float ins_yaw, ins_wz_dps;
    float spin_reference_yaw, spin_relative_yaw, spin_rate_set_dps;
    pid_type_def spin_rate_pid, chassis_angle_pid;
    chassis_motor_t motor_chassis[4];
    pid_type_def motor_speed_pid[4];
    steering_motor_t steering_motor[4];
    float vx, vy, wz, vx_set, vy_set, wz_set;
    float chassis_relative_angle, chassis_relative_angle_set, chassis_yaw_set;
} chassis_move_t;

extern chassis_move_t chassis_move;
void ChassisInit(void); /* Once, after the original startup settling delay. */
void ChassisTask(void); /* One nonblocking control cycle. Sole motor-output owner. */

#endif
