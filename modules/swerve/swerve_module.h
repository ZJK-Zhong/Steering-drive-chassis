#ifndef SWERVE_MODULE_H
#define SWERVE_MODULE_H

#include "chassis.h"

void WheelInit(chassis_move_t *state);
void WheelFeedback(chassis_move_t *state);
void WheelControl(chassis_move_t *state);
void WheelSend(const chassis_move_t *state);

#endif
