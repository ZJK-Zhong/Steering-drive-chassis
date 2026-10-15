#include "bsp_runtime.h"
#include "main.h"

uint32_t BspRuntimeTick(void)
{
    return HAL_GetTick();
}

uint32_t BspRuntimeEnterCritical(void)
{
    uint32_t state = __get_PRIMASK();
    __disable_irq();
    return state;
}

void BspRuntimeExitCritical(uint32_t state)
{
    __set_PRIMASK(state);
}
