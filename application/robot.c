#include "robot.h"
#include "robot_cmd.h"
#include "chassis.h"
#include "message_center.h"
#include "bsp_can.h"
#include "bsp_delay.h"
#include "bsp_usart.h"
#include "calibrate_task.h"

void RobotInit(void)
{
    /* Preserve board startup order; calibration/delays require interrupts. */
    can_filter_init();
    delay_init();
    cali_param_init();
    remote_control_init();
    usart1_tx_dma_init();
    MessageCenterInit();
    RobotCMDInit();
}

void RobotControlInit(void)
{
    ChassisInit();
}

void RobotTask(void)
{
    RobotCMDTask();
    ChassisTask();
}
