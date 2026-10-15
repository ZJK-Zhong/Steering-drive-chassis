# Message center

The message center exposes typed latest-value channels between applications.
Each channel stores a value and publication tick in a fixed static buffer.
Reads copy the value inside a short BSP critical section and return false when
the channel is empty or older than its configured timeout.

The module depends on `bsp_runtime`, not STM32 HAL directly. Applications do
not include one another to exchange chassis commands or feedback.
