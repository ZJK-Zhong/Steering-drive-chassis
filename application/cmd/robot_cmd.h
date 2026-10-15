#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H

#include "robot_def.h"
#include "remote_control.h"

void RobotCMDInit(void);
void RobotCMDTask(void);
robot_mode_e robot_mode_choose(const RC_ctrl_t *rc);
extern RC_ctrl_t robot_cmd_rc; /* Snapshot for Keil Watch, not a control input. */

#endif
