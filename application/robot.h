#ifndef ROBOT_H
#define ROBOT_H

void RobotInit(void); /* Peripheral/application setup, before scheduler. */
void RobotControlEnable(void); /* Enable outputs after startup settling delay. */
void RobotTask(void); /* One command -> chassis -> motor cycle. */

#endif
