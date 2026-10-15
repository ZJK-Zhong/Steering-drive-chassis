# Steering-drive chassis architecture migration

This branch moves the chassis-board firmware toward the application layering used by
`E:\RM\rm-sentry`, while keeping the commissioned steering-drive behavior and CAN
wire protocol unchanged.

## Runtime flow

`RobotInit` performs the existing board startup sequence. `OSTaskInit` creates the
existing INS, calibration, UI and battery tasks plus one deterministic 2 ms robot
task. The robot task executes this fixed order:

1. `RobotCMDTask` snapshots the local DBUS receiver, validates its timeout and
   publishes a typed chassis command.
2. `ChassisTask` consumes that command, checks INS and gimbal-CAN freshness, applies
   the right-switch mode policy, and publishes typed feedback.
3. The swerve motor adapter reads all four drive/steer feedback values, runs the
   existing PID gains and sends the existing CAN frames.

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
- Stale or invalid RC, INS or gimbal data cannot produce a non-zero command.

## Deliberate limits

The gimbal board firmware and its CAN protocol are not migrated here. Vision, firing
and referee behavior remain unsupported on this chassis board and fall back to zero
force. The previously observed long-duration spin drift is not silently tuned in this
architectural change; it still needs hardware logging and calibration.

## Verification

Run the production-source ARM regression with:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_remote_modes.ps1 -Simulator Unicorn -PythonExecutable D:/anaconda3/python.exe
```

Build `standard_robot` in Keil after opening
`MDK-ARM/standard_robot.uvprojx`. The branch has been checked with both the ARM
regression and the Keil command-line build; hardware flashing and motion testing
remain required before merging.
