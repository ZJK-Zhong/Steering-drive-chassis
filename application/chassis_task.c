/* Compatibility entry for existing RTOS projects. Control lives in chassis/. */
#include "chassis_task.h"
#include "robot_task.h"

void chassis_task(void const *argument)
{
    StartRobotTask(argument);
}
