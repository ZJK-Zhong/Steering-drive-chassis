#include "swerve_kinematics.h"
#include "robot_config.h"
#include "arm_math.h"
#include <math.h>

float SwerveNormalizeRad(float angle)
{
    while (angle > PI) angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
}

void SwerveSolve(float vx, float vy, float wz_dps, const float actual_deg[4],
                 SwerveTarget target[4])
{
    unsigned i;
    const float theta = atan(1.0 / 1.0);
    /* Preserve the commissioned conversion, including the legacy 3.14 factor. */
    const float vw = wz_dps * 3.14f / 180.0f * CHASSIS_CENTER_TO_WHEEL;
    const float ratio = 60.0f / (CHASSIS_MOTOT_PER * SWERVE_DRIVE_REDUCTION);
    const float x[4] = {vx - vw * arm_sin_f32(theta), vx + vw * arm_sin_f32(theta),
                        vx + vw * arm_sin_f32(theta), vx - vw * arm_sin_f32(theta)};
    const float y[4] = {vy + vw * arm_cos_f32(theta), vy + vw * arm_cos_f32(theta),
                        vy - vw * arm_cos_f32(theta), vy - vw * arm_cos_f32(theta)};
    if (vx == 0 && vy == 0 && wz_dps == 0)
    {
        for (i = 0; i < 4; i++) target[i].speed_rpm = 0;
        return;
    }
    for (i = 0; i < 4; i++)
    {
        float current = actual_deg[i] * PI / 180.0f;
        float diff = SwerveNormalizeRad(atan2(y[i], x[i]) - current);
        target[i].reversed = 0;
        if (diff > PI / 2.0f) { diff -= PI; target[i].reversed = 1; }
        else if (diff < -PI / 2.0f) { diff += PI; target[i].reversed = 1; }
        target[i].angle_deg = (current + diff) * 180.0f / PI;
        target[i].speed_rpm = sqrt(pow(y[i], 2) + pow(x[i], 2)) * ratio;
        if (target[i].reversed) target[i].speed_rpm = -target[i].speed_rpm;
    }
}
