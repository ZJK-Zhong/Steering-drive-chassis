/* Motor feedback/PID/CAN adapter; kinematics stays hardware-independent. */
#include "swerve_module.h"
#include "main.h"

void WheelInit(chassis_move_t *state)
{
    unsigned i;
    static const uint16_t zeros[4] = SWERVE_ZERO_ENCODERS;
    static const float drive_pid[3] = {M3505_MOTOR_SPEED_PID_KP,
        M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
    static const float steer_pid[3] = {STEERING_ENCODE_RELATIVE_PID_KP,
        STEERING_ENCODE_RELATIVE_PID_KI, STEERING_ENCODE_RELATIVE_PID_KD};
    for (i = 0; i < 4; i++)
    {
        state->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        state->steering_motor[i].steering_motor_measure = get_steering_motor_measure_point(i);
        state->steering_motor[i].steering_zero_ecd = zeros[i];
        state->steering_motor[i].steering_motor_circle = 0;
        state->steering_motor[i].steering_last_feedback_deg = 0;
        PID_init(&state->motor_speed_pid[i], PID_POSITION, drive_pid,
                 M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
        PID_init(&state->steering_motor[i].steering_motor_relative_angle_pid,
                 PID_POSITION, steer_pid, STEERING_ENCODE_RELATIVE_PID_MAX_OUT,
                 STEERING_ENCODE_RELATIVE_PID_MAX_IOUT);
    }
    WheelFeedback(state);
    for (i = 0; i < 4; i++) state->steering_motor[i].steering_ecd_angle_set =
        state->steering_motor[i].steering_ecd_angle;
}

void WheelFeedback(chassis_move_t *state)
{
    unsigned i;
    motor_measure_t drive[4], steer[4];
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    for (i = 0; i < 4; i++)
    {
        drive[i] = *state->motor_chassis[i].chassis_motor_measure;
        steer[i] = *state->steering_motor[i].steering_motor_measure;
    }
    __set_PRIMASK(irq);
    for (i = 0; i < 4; i++)
    {
        steering_motor_t *wheel = &state->steering_motor[i];
        float angle = -(float)((int)steer[i].ecd - wheel->steering_zero_ecd) / 8192 * 360;
        float delta = angle - wheel->steering_last_feedback_deg;
        if (delta > 180) wheel->steering_motor_circle--;
        if (delta < -180) wheel->steering_motor_circle++;
        wheel->steering_last_feedback_deg = angle;
        wheel->steering_ecd_angle = angle + wheel->steering_motor_circle * 360;
        state->motor_chassis[i].speed = drive[i].speed_rpm;
    }
}

void WheelControl(chassis_move_t *state)
{
    unsigned i;
    for (i = 0; i < 4; i++)
    {
        chassis_motor_t *drive = &state->motor_chassis[i];
        steering_motor_t *steer = &state->steering_motor[i];
        if (state->mode == CHASSIS_MODE_ZERO_FORCE)
        {
            drive->speed_set = drive->give_current_set = 0;
            drive->give_current = 0;
            steer->steering_current_set = 0;
            steer->steering_given_current = 0;
            steer->steering_ecd_angle_set = steer->steering_ecd_angle;
            steer->steering_turn_flag = 0;
            PID_clear(&state->motor_speed_pid[i]);
            PID_clear(&steer->steering_motor_relative_angle_pid);
            continue;
        }
        drive->give_current_set = PID_calc(&state->motor_speed_pid[i], drive->speed, drive->speed_set);
        drive->give_current = (int16_t)drive->give_current_set;
        steer->steering_current_set = -PID_calc(&steer->steering_motor_relative_angle_pid,
                                              steer->steering_ecd_angle, steer->steering_ecd_angle_set);
        steer->steering_given_current = (int16_t)steer->steering_current_set;
    }
}

void WheelSend(const chassis_move_t *state)
{
    CAN_cmd_steering_chassis(state->steering_motor[0].steering_given_current,
        state->steering_motor[1].steering_given_current, state->steering_motor[2].steering_given_current,
        state->steering_motor[3].steering_given_current);
    CAN_cmd_chassis(state->motor_chassis[0].give_current, state->motor_chassis[1].give_current,
        state->motor_chassis[2].give_current, state->motor_chassis[3].give_current);
}
