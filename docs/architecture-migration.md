# Steering-drive chassis architecture migration

This branch moves the chassis-board firmware toward the application layering used by
`E:\RM\rm-sentry`, while keeping the commissioned steering-drive behavior and CAN
wire protocol unchanged.

## Runtime flow

`RobotInit` performs the existing board startup sequence and initializes the
chassis application and four wheel instances before the scheduler starts.
`OSTaskInit` creates the existing INS, calibration, UI and battery tasks plus one
robot task scheduled at 2 ms using `vTaskDelayUntil`. Priorities, stacks and the
357 ms startup settling delay are preserved. This is a nominal period, not a
guarantee against CPU overruns.
After settling, `RobotControlEnable()` calls `ChassisEnable()` to refresh measured
angles and enable the already registered wheels. Before this point the control
loop's startup gate remains zero force, including any attempted spin command.
The robot task executes this fixed order:

1. `RobotCMDTask` snapshots the local DBUS receiver, validates its timeout and
   publishes a typed chassis command.
2. `ChassisTask` snapshots the four drive/steer feedback values through the motor
   adapter, consumes the command and checks INS and gimbal-CAN freshness.
3. The chassis application handles mode transitions, spin rate control and frame
   transforms. The pure `SwerveSolve` module calculates wheel targets.
4. The wheel module runs the existing PID gains. The chassis-owned adapter
   collects four outputs and sends the existing grouped CAN frames.
5. The chassis application publishes typed feedback.

`modules/message_center` deliberately uses static latest-value channels with bounded
age checks. It does not use the reference repository's dynamic queue implementation,
which has no timestamp contract and an unsafe post-increment queue index.

## Preserved behavior

- Physical switch mapping: right down = zero force, right middle = remote body-frame
  control, right up = INS small-spin. The left switch remains unused.
- Existing RC sign conventions, INS sign, spin entry-yaw reference and rate ramp.
- Four steering encoder zeros, wheel order, kinematic geometry, speed conversion,
  PID gains and CAN IDs/current limits.
- Zero-force output clears all eight actuator commands and all motor PID integrators.
- Invalid/stale RC or a missing/stale internal command forces all eight outputs to
  zero. Spin additionally requires fresh INS; follow requires fresh gimbal-CAN data.
  Manual mode intentionally works without either sensor source.

## Ownership and configuration

| Path | Responsibility |
| --- | --- |
| `application/robot_config.h` | Chassis parameters, gains, switch mapping and timing |
| `application/robot_def.h` | Typed application commands/feedback, explicit units |
| `application/robot.c` | Board startup and command-to-chassis cycle |
| `application/robot_task.c` | RTOS task registration and periodic scheduling |
| `application/cmd/robot_cmd.c` | Local remote snapshot, mode request and stick scaling |
| `application/chassis/chassis.c` | Safety gates, spin/follow state, wheel targets |
| `modules/swerve/swerve_kinematics.c` | Hardware-independent four-wheel kinematics |
| `modules/swerve/swerve_module.c` | Hardware-independent feedback conversion and PID |
| `bsp/boards/bsp_runtime.c` | HAL-backed tick and critical-section wrappers |
| `modules/message_center` | Static typed latest-value channels, 20 ms expiry |

Keep `chassis_move` in Keil Watch for modes, sensor flags, spin references, wheel
targets and currents. Raw remote diagnostics now live at `robot_cmd_rc`; the
legacy `chassis_move.rc_snapshot` is no longer the active command source.
The authoritative wheel PID state now lives in `chassis_move.swerve_wheels[i]`;
the old PID fields are read-only diagnostic mirrors, not duplicate controllers.
The feedback contract reports wheel RPM and steering degrees, not estimated body
velocity. Old DJI `chassis_behaviour` and power-control files remain inactive;
`chassis_task.c/h` are compatibility wrappers, not a second control loop.

## Deliberate limits

The gimbal board firmware and its CAN protocol are not migrated here. Vision, firing
and referee behavior remain unsupported on this chassis board and fall back to zero
force. The previously observed long-duration spin drift is not silently tuned in this
architectural change; it still needs hardware logging and calibration.
Motor feedback timeout detection, CAN transmit retry/watchdog, a gimbal follow
remote entry, referee power limiting and odometry are not newly implemented.
Internal message expiry protects against a missing publisher only while the
consumer is still running; it is not a watchdog for a stalled robot task.
This is an incremental migration, not a claim that every legacy file now follows
the new layering. The existing RC, INS, CAN receive, calibration and UI drivers
still retain legacy HAL dependencies. The new wheel module and message runtime
boundary are separated without rewriting those commissioned drivers.

## Verification

Run the production-source ARM regression with:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_remote_modes.ps1 -Simulator Unicorn -PythonExecutable D:/anaconda3/python.exe
```

Build `standard_robot` in Keil after opening
`MDK-ARM/standard_robot.uvprojx`. The branch has been checked with both the ARM
regression and the Keil command-line build; hardware flashing and motion testing
remain required before merging.

The ARM tests exercise the real modules with simulated HAL I/O: all nine switch
combinations, stick signs, RC/INS/CAN expiry, spin ramp/entry/wrap, zero force on all
eight CAN outputs, typed-channel expiry across tick wrap, invalid numeric commands,
legacy-formula kinematics parity and the complete `RobotTask` cycle. Additional
tests cover independent callback-based wheel instances, encoder wrap, signed RPM,
bad configuration/feedback, startup output gating, distinct four-wheel feedback
and output mapping, and nested critical sections. A static check rejects direct
HAL/application includes in the new modules and checks Keil source registration.

First hardware check: raise the wheels, start with right switch down, confirm zero
currents, verify manual wheel order/signs at low stick input, then verify spin and
remote-loss stop before any ground test. No hardware validation is claimed here.
