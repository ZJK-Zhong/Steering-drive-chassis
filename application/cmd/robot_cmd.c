#include "robot_cmd.h"
#include "robot_config.h"
#include "message_center.h"
#include "user_lib.h"
#include <string.h>

RC_ctrl_t robot_cmd_rc;

static int16_t deadband(int16_t value)
{
    return (value > CHASSIS_RC_DEADLINE || value < -CHASSIS_RC_DEADLINE) ? value : 0;
}

robot_mode_e robot_mode_choose(const RC_ctrl_t *rc)
{
    if (rc == NULL) return ROBOT_SAFE;
    switch (rc->rc.s[CHASSIS_RIGHT_SWITCH])
    {
        case RC_SW_DOWN: return ROBOT_ZERO_FORCE;
        case RC_SW_MID: return ROBOT_RC_DEBUG;
        case RC_SW_UP: return ROBOT_SPIN;
        default: return ROBOT_SAFE;
    }
}

void RobotCMDInit(void)
{
    const Chassis_Ctrl_Cmd_s safe = {ROBOT_SAFE, 0, 0, 0, 0};
    memset(&robot_cmd_rc, 0, sizeof(robot_cmd_rc));
    ChassisCommandPublish(&safe);
}

void RobotCMDTask(void)
{
    Chassis_Ctrl_Cmd_s command = {ROBOT_SAFE, 0, 0, 0, 0};
    command.rc_online = remote_control_snapshot(&robot_cmd_rc);
    if (command.rc_online)
    {
        command.robot_mode = robot_mode_choose(&robot_cmd_rc);
        command.vx = fp32_constrain(deadband(robot_cmd_rc.rc.ch[CHASSIS_X_CHANNEL]) * CHASSIS_VX_RC_SEN,
                                   -NORMAL_MAX_CHASSIS_SPEED_X, NORMAL_MAX_CHASSIS_SPEED_X);
        command.vy = fp32_constrain(-deadband(robot_cmd_rc.rc.ch[CHASSIS_Y_CHANNEL]) * CHASSIS_VY_RC_SEN,
                                   -NORMAL_MAX_CHASSIS_SPEED_Y, NORMAL_MAX_CHASSIS_SPEED_Y);
        command.wz = -deadband(robot_cmd_rc.rc.ch[CHASSIS_WZ_CHANNEL]) * CHASSIS_WZ_RC_SEN;
    }
    ChassisCommandPublish(&command);
}
