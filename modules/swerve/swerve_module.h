#ifndef SWERVE_MODULE_H
#define SWERVE_MODULE_H

#include <stdint.h>
#include "pid.h"

typedef struct
{
    uint16_t encoder; /* Raw steering encoder, 0..8191. */
    float speed_rpm; /* Measured signed drive motor RPM. */
} SwerveWheelFeedback_s;

typedef struct
{
    float speed_rpm; /* Signed motor RPM, reverse decision already applied. */
    float angle_deg; /* Continuous steering angle in degrees. */
    uint8_t reversed; /* Diagnostic flag only; does not negate speed again. */
} SwerveWheelCommand_s;

typedef uint8_t (*SwerveWheelReadFeedback_f)(void *context,
    SwerveWheelFeedback_s *feedback);
typedef void (*SwerveWheelWriteCommand_f)(void *context,
    const SwerveWheelCommand_s *command, int16_t drive_current,
    int16_t steering_current);

typedef struct
{
    uint16_t encoder_zero;
    float drive_pid[3];
    float drive_max_out;
    float drive_max_iout;
    float steering_pid[3];
    float steering_max_out;
    float steering_max_iout;
} SwerveWheelConfig_s;

typedef struct
{
    SwerveWheelReadFeedback_f read_feedback;
    SwerveWheelWriteCommand_f write_command;
    void *context;
} SwerveWheelPort_s;

typedef struct
{
    SwerveWheelConfig_s config;
    SwerveWheelPort_s port;
    SwerveWheelFeedback_s feedback;
    SwerveWheelCommand_s command;
    pid_type_def drive_pid;
    pid_type_def steering_pid;
    float steering_angle_deg;
    float steering_last_angle_deg;
    int32_t steering_circle;
    int16_t drive_current;
    int16_t steering_current;
    uint8_t registered;
    uint8_t feedback_valid;
} SwerveWheelInstance_s;

/* Register once before scheduling. Returns 1 on valid configuration, even if
 * feedback has not arrived; output stays zero until a valid update. Configuration
 * is copied; caller retains instance/context storage and must serialize access. */
uint8_t SwerveWheelRegister(SwerveWheelInstance_s *instance,
    const SwerveWheelConfig_s *config, const SwerveWheelPort_s *port);
/* Read one callback sample. Failure clears currents/history; returns 0. */
uint8_t SwerveWheelUpdate(SwerveWheelInstance_s *instance);
/* Copy signed targets. Nonfinite values invalidate feedback and clear output. */
void SwerveWheelSetTarget(SwerveWheelInstance_s *instance,
    const SwerveWheelCommand_s *command);
/* Run both PID loops only while enabled and feedback_valid. Does not transmit. */
void SwerveWheelControl(SwerveWheelInstance_s *instance, uint8_t enabled);
/* Zero currents/speed, hold measured angle, and reset both PID histories. */
void SwerveWheelStop(SwerveWheelInstance_s *instance);
/* Stage one output via callback; grouping and physical transmission are external. */
void SwerveWheelWrite(SwerveWheelInstance_s *instance);

#endif
