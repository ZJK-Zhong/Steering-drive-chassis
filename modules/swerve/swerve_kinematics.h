#ifndef SWERVE_KINEMATICS_H
#define SWERVE_KINEMATICS_H

#include <stdint.h>

typedef struct
{
    float angle_deg;
    float speed_rpm;
    uint8_t reversed;
} SwerveTarget;

float SwerveNormalizeRad(float angle);
/* Zero velocity holds each target angle. No motor, remote or RTOS dependency. */
void SwerveSolve(float vx, float vy, float wz_dps, const float actual_deg[4],
                 SwerveTarget target[4]);

#endif
