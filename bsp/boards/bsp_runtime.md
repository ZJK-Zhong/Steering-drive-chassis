# Runtime services

`bsp_runtime.c` isolates the STM32/CMSIS tick and interrupt-mask operations used
by the new control path. It does not initialize any peripheral or RTOS object.

- `BspRuntimeTick()` returns the wrapping HAL millisecond counter.
- `BspRuntimeEnterCritical()` saves PRIMASK and disables interrupts.
- `BspRuntimeExitCritical(state)` restores the exact saved PRIMASK.

Critical sections must be short and balanced. Do not wait, run PID/kinematics, or
transmit CAN while interrupts are masked. Preserve the returned state so nested
calls do not accidentally re-enable an already masked interrupt context.

```c
uint32_t state = BspRuntimeEnterCritical();
/* Copy a shared feedback sample here. */
BspRuntimeExitCritical(state);
```

The ARM regression uses the production wrapper with a simulated HAL tick and
checks nested mask restoration and message age arithmetic across tick wrap.
