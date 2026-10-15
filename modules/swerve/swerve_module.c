/* Hardware-independent steering-drive wheel controller. */
#include "swerve_module.h"
#include <float.h>
#include <string.h>

static uint8_t finite_value(float value)
{
    return value >= -FLT_MAX && value <= FLT_MAX;
}

static uint8_t valid_pid(const float gains[3], float limit, float integral_limit)
{
    return finite_value(gains[0]) && finite_value(gains[1]) && finite_value(gains[2]) &&
        limit >= 0 && limit <= 32767.0f && integral_limit >= 0 && integral_limit <= limit;
}

static float encoder_to_angle(uint16_t encoder, uint16_t zero)
{
    return -(float)((int32_t)encoder - (int32_t)zero) * 360.0f / 8192.0f;
}

uint8_t SwerveWheelRegister(SwerveWheelInstance_s *instance,
    const SwerveWheelConfig_s *config, const SwerveWheelPort_s *port)
{
    if (instance == 0) return 0;
    if (config == 0 || port == 0 || port->read_feedback == 0 || port->write_command == 0 ||
        config->encoder_zero >= 8192 ||
        !valid_pid(config->drive_pid, config->drive_max_out, config->drive_max_iout) ||
        !valid_pid(config->steering_pid, config->steering_max_out, config->steering_max_iout))
    {
        memset(instance, 0, sizeof(*instance));
        return 0;
    }
    instance->config = *config;
    instance->port = *port;
    instance->feedback.encoder = config->encoder_zero;
    instance->feedback.speed_rpm = 0;
    instance->command.speed_rpm = 0;
    instance->command.angle_deg = 0;
    instance->command.reversed = 0;
    instance->steering_angle_deg = 0;
    instance->steering_last_angle_deg = 0;
    instance->steering_circle = 0;
    instance->drive_current = 0;
    instance->steering_current = 0;
    PID_init(&instance->drive_pid, PID_POSITION, config->drive_pid,
        config->drive_max_out, config->drive_max_iout);
    PID_init(&instance->steering_pid, PID_POSITION, config->steering_pid,
        config->steering_max_out, config->steering_max_iout);
    PID_clear(&instance->drive_pid);
    PID_clear(&instance->steering_pid);
    instance->registered = 1;
    instance->feedback_valid = 0;
    if (!SwerveWheelUpdate(instance))
    {
        instance->steering_angle_deg = 0;
        instance->steering_last_angle_deg = 0;
    }
    instance->command.angle_deg = instance->steering_angle_deg;
    return 1;
}

uint8_t SwerveWheelUpdate(SwerveWheelInstance_s *instance)
{
    SwerveWheelFeedback_s feedback;
    float angle, delta;
    if (instance == 0 || !instance->registered || instance->port.read_feedback == 0)
        return 0;
    instance->feedback_valid = 0;
    if (!instance->port.read_feedback(instance->port.context, &feedback) ||
        feedback.encoder >= 8192 || !finite_value(feedback.speed_rpm))
    {
        SwerveWheelStop(instance);
        return 0;
    }
    instance->feedback = feedback;
    angle = encoder_to_angle(feedback.encoder, instance->config.encoder_zero);
    delta = angle - instance->steering_last_angle_deg;
    if (delta > 180.0f) instance->steering_circle--;
    if (delta < -180.0f) instance->steering_circle++;
    instance->steering_last_angle_deg = angle;
    instance->steering_angle_deg = angle + instance->steering_circle * 360.0f;
    instance->feedback_valid = 1;
    return 1;
}

void SwerveWheelSetTarget(SwerveWheelInstance_s *instance,
    const SwerveWheelCommand_s *command)
{
    if (instance == 0 || command == 0 || !instance->registered) return;
    if (!finite_value(command->speed_rpm) || !finite_value(command->angle_deg))
    {
        instance->feedback_valid = 0;
        SwerveWheelStop(instance);
        return;
    }
    instance->command = *command;
}

void SwerveWheelStop(SwerveWheelInstance_s *instance)
{
    if (instance == 0 || !instance->registered) return;
    instance->command.speed_rpm = 0;
    instance->command.angle_deg = instance->steering_angle_deg;
    instance->command.reversed = 0;
    instance->drive_current = 0;
    instance->steering_current = 0;
    PID_clear(&instance->drive_pid);
    PID_clear(&instance->steering_pid);
}

void SwerveWheelControl(SwerveWheelInstance_s *instance, uint8_t enabled)
{
    float drive_output, steering_output;
    if (instance == 0 || !instance->registered) return;
    if (!enabled || !instance->feedback_valid)
    {
        SwerveWheelStop(instance);
        return;
    }
    /* Kinematics already folds the reverse flag into the signed speed target. */
    drive_output = PID_calc(&instance->drive_pid, instance->feedback.speed_rpm,
        instance->command.speed_rpm);
    steering_output = -PID_calc(&instance->steering_pid, instance->steering_angle_deg,
        instance->command.angle_deg);
    if (!(drive_output >= -32767.0f && drive_output <= 32767.0f &&
          steering_output >= -32767.0f && steering_output <= 32767.0f))
    {
        SwerveWheelStop(instance);
        return;
    }
    instance->drive_current = (int16_t)drive_output;
    instance->steering_current = (int16_t)steering_output;
}

void SwerveWheelWrite(SwerveWheelInstance_s *instance)
{
    if (instance == 0 || !instance->registered || instance->port.write_command == 0)
        return;
    instance->port.write_command(instance->port.context, &instance->command,
        instance->drive_current, instance->steering_current);
}
