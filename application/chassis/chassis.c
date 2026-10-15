/* (C) COPYRIGHT 2019 DJI. Derived from chassis_task.c; application split in 2026. */
#include "chassis.h"
#include "message_center.h"
#include "swerve_kinematics.h"
#include "INS_task.h"
#include "arm_math.h"
#include "bsp_runtime.h"
#include <string.h>

chassis_move_t chassis_move;

typedef struct
{
    int16_t drive_current[4];
    int16_t steering_current[4];
} ChassisWheelBus_s;

typedef struct
{
    uint8_t index;
    ChassisWheelBus_s *bus;
    SwerveWheelFeedback_s feedback;
} ChassisWheelPortContext_s;

static ChassisWheelBus_s chassis_wheel_bus;
static ChassisWheelPortContext_s chassis_wheel_ports[4];

static void chassis_wheel_snapshot(void)
{
    unsigned i;
    uint32_t irq = BspRuntimeEnterCritical();
    for (i = 0; i < 4; i++)
    {
        chassis_wheel_ports[i].feedback.encoder = get_steering_motor_measure_point(i)->ecd;
        chassis_wheel_ports[i].feedback.speed_rpm = get_chassis_motor_measure_point(i)->speed_rpm;
    }
    BspRuntimeExitCritical(irq);
}

static uint8_t chassis_wheel_read(void *context, SwerveWheelFeedback_s *feedback)
{
    ChassisWheelPortContext_s *port = (ChassisWheelPortContext_s *)context;
    if (port == 0 || feedback == 0 || port->index >= 4) return 0;
    *feedback = port->feedback;
    return 1;
}

static void chassis_wheel_write(void *context, const SwerveWheelCommand_s *command,
    int16_t drive_current, int16_t steering_current)
{
    ChassisWheelPortContext_s *port = (ChassisWheelPortContext_s *)context;
    (void)command;
    if (port == 0 || port->bus == 0 || port->index >= 4) return;
    port->bus->drive_current[port->index] = drive_current;
    port->bus->steering_current[port->index] = steering_current;
}

static void chassis_sync_legacy_from_modules(chassis_move_t *state)
{
    unsigned i;
    for (i = 0; i < 4; i++)
    {
        SwerveWheelInstance_s *wheel = &state->swerve_wheels[i];
        state->motor_chassis[i].speed = wheel->feedback.speed_rpm;
        state->motor_chassis[i].speed_set = wheel->command.speed_rpm;
        state->motor_chassis[i].give_current = wheel->drive_current;
        state->motor_chassis[i].give_current_set = wheel->drive_pid.out;
        state->motor_speed_pid[i] = wheel->drive_pid;
        state->steering_motor[i].steering_motor_circle = wheel->steering_circle;
        state->steering_motor[i].steering_last_feedback_deg = wheel->steering_last_angle_deg;
        state->steering_motor[i].steering_ecd_angle = wheel->steering_angle_deg;
        state->steering_motor[i].steering_ecd_angle_set = wheel->command.angle_deg;
        state->steering_motor[i].steering_turn_flag = wheel->command.reversed;
        state->steering_motor[i].steering_current_set = -wheel->steering_pid.out;
        state->steering_motor[i].steering_given_current = wheel->steering_current;
        state->steering_motor[i].steering_motor_relative_angle_pid = wheel->steering_pid;
    }
}

static void chassis_sync_modules_from_legacy(chassis_move_t *state)
{
    unsigned i;
    for (i = 0; i < 4; i++)
    {
        SwerveWheelInstance_s *wheel = &state->swerve_wheels[i];
        SwerveWheelCommand_s command;
        command.speed_rpm = state->motor_chassis[i].speed_set;
        command.angle_deg = state->steering_motor[i].steering_ecd_angle_set;
        command.reversed = state->steering_motor[i].steering_turn_flag;
        SwerveWheelSetTarget(wheel, &command);
    }
}

void WheelInit(chassis_move_t *state)
{
    unsigned i;
    static const uint16_t zeros[4] = SWERVE_ZERO_ENCODERS;
    static const float drive_pid[3] = {M3505_MOTOR_SPEED_PID_KP,
        M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
    static const float steering_pid[3] = {STEERING_ENCODE_RELATIVE_PID_KP,
        STEERING_ENCODE_RELATIVE_PID_KI, STEERING_ENCODE_RELATIVE_PID_KD};
    SwerveWheelConfig_s config;
    memset(&chassis_wheel_bus, 0, sizeof(chassis_wheel_bus));
    memset(chassis_wheel_ports, 0, sizeof(chassis_wheel_ports));
    chassis_wheel_snapshot();
    for (i = 0; i < 4; i++)
    {
        SwerveWheelPort_s port;
        state->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        state->steering_motor[i].steering_motor_measure = get_steering_motor_measure_point(i);
        state->steering_motor[i].steering_zero_ecd = zeros[i];
        memset(&config, 0, sizeof(config));
        config.encoder_zero = zeros[i];
        memcpy(config.drive_pid, drive_pid, sizeof(drive_pid));
        config.drive_max_out = M3505_MOTOR_SPEED_PID_MAX_OUT;
        config.drive_max_iout = M3505_MOTOR_SPEED_PID_MAX_IOUT;
        memcpy(config.steering_pid, steering_pid, sizeof(steering_pid));
        config.steering_max_out = STEERING_ENCODE_RELATIVE_PID_MAX_OUT;
        config.steering_max_iout = STEERING_ENCODE_RELATIVE_PID_MAX_IOUT;
        chassis_wheel_ports[i].index = (uint8_t)i;
        chassis_wheel_ports[i].bus = &chassis_wheel_bus;
        port.read_feedback = chassis_wheel_read;
        port.write_command = chassis_wheel_write;
        port.context = &chassis_wheel_ports[i];
        SwerveWheelRegister(&state->swerve_wheels[i], &config, &port);
    }
    WheelFeedback(state);
    for (i = 0; i < 4; i++)
        state->steering_motor[i].steering_ecd_angle_set =
            state->steering_motor[i].steering_ecd_angle;
    chassis_sync_modules_from_legacy(state);
}

void WheelFeedback(chassis_move_t *state)
{
    unsigned i;
    chassis_wheel_snapshot();
    for (i = 0; i < 4; i++) SwerveWheelUpdate(&state->swerve_wheels[i]);
    chassis_sync_legacy_from_modules(state);
}

void WheelControl(chassis_move_t *state)
{
    unsigned i;
    chassis_sync_modules_from_legacy(state);
    for (i = 0; i < 4; i++)
        SwerveWheelControl(&state->swerve_wheels[i],
            state->mode != CHASSIS_MODE_ZERO_FORCE);
    chassis_sync_legacy_from_modules(state);
}

void WheelSend(chassis_move_t *state)
{
    unsigned i;
    memset(chassis_wheel_bus.drive_current, 0, sizeof(chassis_wheel_bus.drive_current));
    memset(chassis_wheel_bus.steering_current, 0, sizeof(chassis_wheel_bus.steering_current));
    for (i = 0; i < 4; i++) SwerveWheelWrite(&state->swerve_wheels[i]);
    CAN_cmd_steering_chassis(chassis_wheel_bus.steering_current[0],
        chassis_wheel_bus.steering_current[1], chassis_wheel_bus.steering_current[2],
        chassis_wheel_bus.steering_current[3]);
    CAN_cmd_chassis(chassis_wheel_bus.drive_current[0], chassis_wheel_bus.drive_current[1],
        chassis_wheel_bus.drive_current[2], chassis_wheel_bus.drive_current[3]);
}

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
    if (control->control_enabled)
    {
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
    chassis_move.control_enabled = 0;
    WheelInit(&chassis_move);
    PID_init(&chassis_move.chassis_angle_pid, PID_POSITION, follow_pid,
        CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
    PID_init(&chassis_move.spin_rate_pid, PID_POSITION, spin_pid,
        CHASSIS_SPIN_RATE_PID_MAX_OUT, CHASSIS_SPIN_RATE_PID_MAX_IOUT);
}

void ChassisEnable(void)
{
    unsigned i;
    if (chassis_move.control_enabled) return;
    WheelFeedback(&chassis_move);
    for (i = 0; i < 4; i++) SwerveWheelStop(&chassis_move.swerve_wheels[i]);
    chassis_sync_legacy_from_modules(&chassis_move);
    chassis_move.control_enabled = 1;
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
