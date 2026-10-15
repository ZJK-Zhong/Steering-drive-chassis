#include "robot_task.h"
#include "robot.h"
#include "robot_config.h"
#include "cmsis_os.h"
#include "calibrate_task.h"
#include "INS_task.h"
#include "led_flow_task.h"
#include "oled_task.h"
#include "voltage_task.h"

osThreadId calibrate_tast_handle, chassisTaskHandle, imuTaskHandle;
osThreadId led_RGB_flow_handle, oled_handle, battery_voltage_handle;
#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

void OSTaskInit(void)
{
    osThreadDef(cali, calibrate_task, osPriorityNormal, 0, 512);
    calibrate_tast_handle = osThreadCreate(osThread(cali), NULL);
    osThreadDef(Robot, StartRobotTask, osPriorityAboveNormal, 0, 512);
    chassisTaskHandle = osThreadCreate(osThread(Robot), NULL);
    osThreadDef(imuTask, INS_task, osPriorityRealtime, 0, 1024);
    imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);
    osThreadDef(led, led_RGB_flow_task, osPriorityNormal, 0, 256);
    led_RGB_flow_handle = osThreadCreate(osThread(led), NULL);
    osThreadDef(OLED, oled_task, osPriorityLow, 0, 256);
    oled_handle = osThreadCreate(osThread(OLED), NULL);
    osThreadDef(BATTERY_VOLTAGE, battery_voltage_task, osPriorityNormal, 0, 128);
    battery_voltage_handle = osThreadCreate(osThread(BATTERY_VOLTAGE), NULL);
}

void StartRobotTask(void const *argument)
{
    TickType_t wake;
    (void)argument;
    vTaskDelay(pdMS_TO_TICKS(CHASSIS_TASK_INIT_TIME));
    RobotControlEnable();
    wake = xTaskGetTickCount();
    for (;;)
    {
        RobotTask();
#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(CHASSIS_CONTROL_TIME_MS));
    }
}
