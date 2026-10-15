#include "message_center.h"
#include "robot_config.h"
#include "bsp_runtime.h"
#include <string.h>

static Chassis_Ctrl_Cmd_s chassis_command;
static Chassis_Upload_Data_s chassis_feedback;
static uint32_t command_ms, feedback_ms;
static uint8_t command_received, feedback_received;

void MessageCenterInit(void)
{
    uint32_t irq = BspRuntimeEnterCritical();
    memset(&chassis_command, 0, sizeof(chassis_command));
    memset(&chassis_feedback, 0, sizeof(chassis_feedback));
    command_received = feedback_received = 0;
    command_ms = feedback_ms = 0;
    BspRuntimeExitCritical(irq);
}

void ChassisCommandPublish(const Chassis_Ctrl_Cmd_s *command)
{
    uint32_t irq;
    if (command == NULL) return;
    irq = BspRuntimeEnterCritical();
    chassis_command = *command;
    command_ms = BspRuntimeTick();
    command_received = 1;
    BspRuntimeExitCritical(irq);
}

uint8_t ChassisCommandRead(Chassis_Ctrl_Cmd_s *command)
{
    uint32_t irq;
    uint8_t valid;
    if (command == NULL) return 0;
    irq = BspRuntimeEnterCritical();
    valid = command_received &&
        (uint32_t)(BspRuntimeTick() - command_ms) < CHASSIS_COMMAND_TIMEOUT_MS;
    if (valid) *command = chassis_command;
    else memset(command, 0, sizeof(*command));
    BspRuntimeExitCritical(irq);
    return valid;
}

void ChassisFeedbackPublish(const Chassis_Upload_Data_s *feedback)
{
    uint32_t irq;
    if (feedback == NULL) return;
    irq = BspRuntimeEnterCritical();
    chassis_feedback = *feedback;
    feedback_ms = BspRuntimeTick();
    feedback_received = 1;
    BspRuntimeExitCritical(irq);
}

uint8_t ChassisFeedbackRead(Chassis_Upload_Data_s *feedback)
{
    uint32_t irq;
    uint8_t valid;
    if (feedback == NULL) return 0;
    irq = BspRuntimeEnterCritical();
    valid = feedback_received &&
        (uint32_t)(BspRuntimeTick() - feedback_ms) < CHASSIS_FEEDBACK_TIMEOUT_MS;
    if (valid) *feedback = chassis_feedback;
    else memset(feedback, 0, sizeof(*feedback));
    BspRuntimeExitCritical(irq);
    return valid;
}
