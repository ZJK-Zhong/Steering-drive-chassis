# Chassis command application

`robot_cmd.c` is the chassis-board command publisher. It snapshots the local
DBUS receiver, rejects stale frames, applies stick deadbands/scales, and writes
one `Chassis_Ctrl_Cmd_s` value to the message center each control cycle.

The command contract uses forward/left `m/s` and counterclockwise `deg/s`.
It is a latest-value channel with a bounded age, so the chassis falls back to
zero force when the publisher or receiver stops updating.

`robot_mode_choose()` intentionally maps only the right switch. Unsupported
shooting, vision, and competition modes remain safe on this board because the
gimbal and firing hardware are owned by another controller.
