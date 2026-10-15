# Chassis application

`chassis.c` owns the robot's steering-drive instances and the chassis control
state. It is the only application that converts typed chassis commands into
wheel targets and owns the motor output cycle.

## Cycle

`ChassisInit()` registers four `SwerveWheelInstance_s` objects before the RTOS
starts. `ChassisTask()` then performs one nonblocking cycle:

1. Read the four CAN feedback values through the local adapter.
2. Read the latest command and validate its freshness and physical limits.
3. Apply zero-force, manual, gimbal-follow, or INS-spin safety gates.
4. Solve wheel targets in `m/s`, `deg/s`, and wheel angle degrees.
5. Run wheel PID control and publish two grouped CAN frames.
6. Publish typed feedback through the message center.

The right RC switch selects zero force (down), manual chassis control (middle),
or INS spin (up). The left switch is intentionally unused on this chassis board.
The 357 ms startup wait only enables outputs; it is not module initialization.
`RobotControlEnable()` calls `ChassisEnable()` once after this wait. The chassis
refreshes feedback and latches measured steering angles before enabling control.
Repeated enable calls do not reinitialize PID history or change wheel targets.

Feedback is copied for all eight motors in one short BSP critical section. PID,
kinematics, callback dispatch and CAN transmission execute outside that section.

The legacy `Wheel*` functions are application adapters retained for watch-window
diagnostics and existing tests. Hardware-independent wheel behavior belongs to
`modules/swerve`.

Keil Watch: `robot_cmd_rc` is the RC snapshot; `chassis_move.mode` is the actual
mode after safety checks, and `chassis_move.control_enabled` is the startup gate.
`chassis_move.swerve_wheels[0..3]` owns PID history, feedback and current commands.
Legacy motor/PID fields are diagnostic mirrors, not writable tuning inputs.
Wheel order remains front-left, front-right, rear-right, rear-left. Steering and
drive frames remain on their original buses with the original identifiers.
