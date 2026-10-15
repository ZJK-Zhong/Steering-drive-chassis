#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

/* Legacy include retained for inactive DJI behaviour/power modules. */
#include "chassis.h"
#include "robot_cmd.h"

#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D
#define SWING_NO_MOVE_ANGLE 0.7f
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f

void chassis_task(void const *argument);

#endif
