#ifndef MESSAGE_CENTER_H
#define MESSAGE_CENTER_H

#include "robot_def.h"

/* Single latest-value channel per contract; copies are interrupt-safe. */
void MessageCenterInit(void);
void ChassisCommandPublish(const Chassis_Ctrl_Cmd_s *command);
uint8_t ChassisCommandRead(Chassis_Ctrl_Cmd_s *command);
void ChassisFeedbackPublish(const Chassis_Upload_Data_s *feedback);
uint8_t ChassisFeedbackRead(Chassis_Upload_Data_s *feedback);

#endif
