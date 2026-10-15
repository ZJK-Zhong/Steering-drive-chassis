#ifndef BSP_RUNTIME_H
#define BSP_RUNTIME_H

#include <stdint.h>

/* Small runtime services shared by modules without exposing STM32 HAL. */
uint32_t BspRuntimeTick(void);
uint32_t BspRuntimeEnterCritical(void);
void BspRuntimeExitCritical(uint32_t state);

#endif
