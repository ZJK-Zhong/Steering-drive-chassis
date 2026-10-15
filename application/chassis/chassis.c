/* Steering-drive chassis application, derived from DJI's chassis_task.c. */
#include "chassis.h"
#include "message_center.h"
#include "swerve_module.h"
#include "swerve_kinematics.h"
#include "INS_task.h"
#include "arm_math.h"
#include <string.h>

chassis_move_t chassis_move;

static uint8_t command_valid(const Chassis_Ctrl_Cmd_s *command)
{
    /* Ordered comparisons also reject NaN and infinity before PID/casts. */
    return command->rc_online == 1 &&
        command->vx >= -NORMAL_MAX_CHASSIS_SPEED_X && command->vx <= NORMAL_MAX_CHASSIS_SPEED_X &&
        command->vy >= -NORMAL_MAX_CHASSIS_SPEED_Y && command->vy <= NORMAL_MAX_CHASSIS_SPEED_Y &&
        command->wz >= -CHASSIS_SPIN_MAX_COMMAND_DPS && command->wz <= CHASSIS_SPIN_MAX_COMMAND_DPS;
}

static void chassis_set_control(chassis_move_t *control)
{
    Chassis_Ctrl_Cmd_s command;
    gimbal_chassis_command_t gimbal;
    ins_motion_t ins;
    chassis_mode_e last_mode = control->mode;
    float angle;
    uint8_t fresh = ChassisCommandRead(&command);
    control->mode = CHASSIS_MODE_ZERO_FORCE;
    control->vx_set = control->vy_set = control->wz_set = 0;
    control->rc_online = fresh && command_valid(&command);
    control->robot_mode = control->rc_online ? command.robot_mode : ROBOT_SAFE;
    control->gimbal_online = gimbal_chassis_snapshot(&gimbal);
    control->ins_online = INS_motion_snapshot(&ins);
    control->ins_yaw = control->ins_online ? CHASSIS_INS_YAW_SIGN * ins.yaw : 0;
    control->ins_wz_dps = control->ins_online ? CHASSIS_INS_YAW_SIGN * ins.gyro_z * 180.0f / PI : 0;
    control->chassis_relative_angle = control->gimbal_online ?
        SwerveNormalizeRad(gimbal.relative_angle * PI / 180.0f) : 0;
    switch (control->robot_mode)
    {
        case ROBOT_RC_DEBUG: control->mode = CHASSIS_RC_DEBUG; break;
        case ROBOT_MOVE:
            if (control->gimbal_online) control->mode = CHASSIS_FOLLOW_GIMBAL;
            break;
        case ROBOT_SPIN:
            if (control->ins_online) control->mode = CHASSIS_SPIN;
            break;
        default: break; /* Unsupported shooting/vision modes stay unpowered. */
    }
    if (control->mode != last_mode || control->mode == CHASSIS_MODE_ZERO_FORCE)
        PID_clear(&control->chassis_angle_pid);
    if (control->mode != last_mode || control->mode != CHASSIS_SPIN)
    {
        PID_clear(&control->spin_rate_pid);
        control->spin_rate_set_dps = control->spin_relative_yaw = 0;
        control->spin_reference_yaw = control->ins_yaw;
    }
    if (control->mode == CHASSIS_MODE_ZERO_FORCE) return;
    if (control->mode == CHASSIS_RC_DEBUG)
    {
        control->vx_set = command.vx;
        control->vy_set = command.vy;
        control->wz_set = command.wz;
        return;
    }
    if (control->mode == CHASSIS_SPIN)
    {
        control->spin_relative_yaw = SwerveNormalizeRad(control->ins_yaw - control->spin_reference_yaw);
        angle = control->spin_relative_yaw;
        control->spin_rate_set_dps += fp32_constrain(CHASSIS_SPIN_SPEED_DPS - control->spin_rate_set_dps,
            -CHASSIS_SPIN_ACCEL_DPS2 * CHASSIS_CONTROL_TIME, CHASSIS_SPIN_ACCEL_DPS2 * CHASSIS_CONTROL_TIME);
        control->wz_set = fp32_constrain(control->spin_rate_set_dps +
            PID_calc(&control->spin_rate_pid, control->ins_wz_dps, control->spin_rate_set_dps),
            -CHASSIS_SPIN_MAX_COMMAND_DPS, CHASSIS_SPIN_MAX_COMMAND_DPS);
    }
    else
    {
        angle = control->chassis_relative_angle;
        control->wz_set = -PID_calc(&control->chassis_angle_pid, angle, 0) * 180.0f / PI;
    }
    control->vx_set = arm_cos_f32(angle) * command.vx + arm_sin_f32(angle) * command.vy;
    control->vy_set = -arm_sin_f32(angle) * command.vx + arm_cos_f32(angle) * command.vy;
}

static void chassis_motor_moving_solution(float vx, float vy, float wz, chassis_move_t *state)
{
    unsigned i;
    float actual[4];
    SwerveTarget target[4];
    for (i = 0; i < 4; i++)
    {
        actual[i] = state->steering_motor[i].steering_ecd_angle;
        target[i].angle_deg = state->steering_motor[i].steering_ecd_angle_set;
        target[i].reversed = state->steering_motor[i].steering_turn_flag;
    }
    SwerveSolve(vx, vy, wz, actual, target);
    for (i = 0; i < 4; i++)
    {
        state->motor_chassis[i].speed_set = target[i].speed_rpm;
        state->steering_motor[i].steering_ecd_angle_set = target[i].angle_deg;
        state->steering_motor[i].steering_turn_flag = target[i].reversed;
    }
}

void ChassisInit(void)
{
    static const float follow_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP,
        CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
    static const float spin_pid[3] = {CHASSIS_SPIN_RATE_PID_KP,
        CHASSIS_SPIN_RATE_PID_KI, CHASSIS_SPIN_RATE_PID_KD};
    memset(&chassis_move, 0, sizeof(chassis_move));
    chassis_move.chassis_RC = get_remote_control_point();
    chassis_move.robot_mode = ROBOT_ZERO_FORCE;
    WheelInit(&chassis_move);
    PID_init(&chassis_move.chassis_angle_pid, PID_POSITION, follow_pid,
        CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
    PID_init(&chassis_move.spin_rate_pid, PID_POSITION, spin_pid,
        CHASSIS_SPIN_RATE_PID_MAX_OUT, CHASSIS_SPIN_RATE_PID_MAX_IOUT);
}

void ChassisTask(void)
{
    unsigned i;
    Chassis_Upload_Data_s feedback;
    memset(&feedback, 0, sizeof(feedback));
    WheelFeedback(&chassis_move);
    chassis_set_control(&chassis_move);
    if (chassis_move.mode != CHASSIS_MODE_ZERO_FORCE)
        chassis_motor_moving_solution(chassis_move.vx_set, chassis_move.vy_set, chassis_move.wz_set, &chassis_move);
    WheelControl(&chassis_move);
    WheelSend(&chassis_move);
    feedback.robot_mode = chassis_move.robot_mode;
    feedback.chassis_mode = chassis_move.mode;
    feedback.rc_online = chassis_move.rc_online;
    feedback.gimbal_online = chassis_move.gimbal_online;
    feedback.ins_online = chassis_move.ins_online;
    feedback.yaw = chassis_move.ins_yaw;
    feedback.gyro_z_dps = chassis_move.ins_wz_dps;
    feedback.vx_set = chassis_move.vx_set;
    feedback.vy_set = chassis_move.vy_set;
    feedback.wz_set = chassis_move.wz_set;
    for (i = 0; i < 4; i++)
    {
        feedback.wheel_rpm[i] = chassis_move.motor_chassis[i].speed;
        feedback.steering_angle_deg[i] = chassis_move.steering_motor[i].steering_ecd_angle;
    }
    ChassisFeedbackPublish(&feedback);
}
