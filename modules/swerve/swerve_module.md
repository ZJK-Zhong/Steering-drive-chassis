# Swerve wheel module

This module is hardware-independent. A `SwerveWheelInstance_s` receives a
`SwerveWheelPort_s` containing callbacks for one feedback sample and one
command sink. The module never includes STM32 HAL, CAN, `main.h`, or an
application header.

The caller owns instance and callback-context storage and serializes access.
Configuration and port definitions are copied during registration; no allocation
or peripheral initialization occurs inside this module.

## API

- `SwerveWheelRegister`: copy configuration and initialize both PID controllers.
- `SwerveWheelUpdate`: read one encoder/speed sample and unwrap steering angle.
- `SwerveWheelSetTarget`: set signed wheel speed, steering angle, and reverse flag.
- `SwerveWheelControl`: calculate drive and steering currents, or stop safely.
- `SwerveWheelStop`: clear targets, currents, and PID history.
- `SwerveWheelWrite`: pass the calculated currents to the application adapter.

The signed speed target already includes the kinematics reverse decision. The
application adapter owns CAN grouping and hardware-specific motor mapping.

## Example

```c
static SwerveWheelInstance_s wheel;
SwerveWheelConfig_s config = {4976, {5, 0, 0}, 16000, 0,
    {360, 0, 0}, 30000, 0};
SwerveWheelPort_s port = {ReadWheelFeedback, StageWheelOutput, &wheel_context};
SwerveWheelCommand_s target = {100, 0, 0};
/* Register before scheduling; callbacks and context are supplied by the owner. */
SwerveWheelRegister(&wheel, &config, &port);
/* Each periodic cycle: update, target, calculate, then stage output. */
SwerveWheelUpdate(&wheel);
SwerveWheelSetTarget(&wheel, &target);
SwerveWheelControl(&wheel, enabled);
SwerveWheelWrite(&wheel);
```

Registration rejects invalid encoder zeros, nonfinite gains, and output limits
outside the signed CAN-current range. A failed callback, invalid raw encoder, or
nonfinite speed disables output until a valid update. Invalid numeric targets
also invalidate the current sample. Disabling control clears PID history and
stages zero current; no command is physically transmitted by this module.

Callback success means that a sample is structurally valid, not necessarily fresh.
The legacy CAN adapter has no motor timestamp, so this change does **not** add
motor-disconnection detection. A finite stale sample is still accepted.
