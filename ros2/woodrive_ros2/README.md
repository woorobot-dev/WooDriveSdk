# WooDrive ROS 2 driver

This package has two things:

- **`woodrive_node`** (below) -- the persistent, full-featured driver. It maps
  the five Arduino/Raspberry Pi SDK examples onto ROS 2 topics, services,
  messages, and actions, with a 200ms watchdog that auto-stops the motor on a
  command timeout or a fault. Use this to actually run a robot.
- **`examples/`** ([README](examples/README.md)) -- five small, standalone
  nodes with no topics/services/actions, one per raspberrypi/arduino example
  (BasicCheck, ReadStatus, AutoMotorSetup, Speed, Position). Use these to learn
  the SDK or bring up a new controller without the driver's complexity.

Both target Jetson/Raspberry Pi Linux and use the SDK's `PosixSerialTransport`.

## Build on Jetson (ROS 2 Humble)

```bash
mkdir -p ~/woodrive_ws/src
cp -r WooDriveSdk ~/woodrive_ws/src/
cd ~/woodrive_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select woodrive_ros2
source install/setup.bash
```

## Run

```bash
ros2 launch woodrive_ros2 woodrive.launch.py
```

## Example 01: basic check

```bash
ros2 service call /woodrive/check woodrive_ros2/srv/BasicCheck '{}'
```

## Example 02: read status

The combined status contains position, velocity, currents, voltages,
temperature, pulse count, fault, and enable state.

```bash
ros2 topic echo /woodrive/status
```

Convenience topics remain available:

- `/woodrive/velocity_rpm`
- `/woodrive/bus_voltage`
- `/woodrive/temperature`
- `/woodrive/fault`

## Example 03: auto motor setup

Warning: this operation changes motor parameters and may move the motor. Secure
the mechanism before sending the goal. The default pole-pair count is 4 and the
default timeout is 240 seconds. Set `pole_pairs` explicitly for a different motor;
this session's motor uses 5.

```bash
ros2 action send_goal --feedback \
  /woodrive/auto_setup woodrive_ros2/action/AutoMotorSetup \
  '{pole_pairs: 4, timeout_sec: 240.0}'
```

## Example 04: speed

Send positive or negative RPM. Commands stop automatically if they are not
refreshed before `command_timeout_sec`.

```bash
ros2 topic pub -r 10 /woodrive/target_rpm std_msgs/msg/Float32 '{data: 20.0}'
ros2 topic pub --once /woodrive/target_rpm std_msgs/msg/Float32 '{data: 0.0}'
```

## Example 05: position

Mode 245 is used by the relative action; mode 244 is used by the absolute
action. Position actions disable/brake the motor and wait 1s to settle into a
known, stopped state before starting the requested move (direction is only
ever 1=CCW or 2=CW here -- never 0/DIR_ZERO, which the protocol defines as "go
to absolute position 0", not "no motion").

```bash
ros2 action send_goal --feedback \
  /woodrive/move_relative woodrive_ros2/action/MovePosition \
  '{target_deg: 360.0, max_rpm: 20.0, accel_ms: 1000, decel_ms: 1000, timeout_sec: 15.0}'
```

```bash
ros2 action send_goal --feedback \
  /woodrive/move_absolute woodrive_ros2/action/MovePosition \
  '{target_deg: 0.0, max_rpm: 20.0, accel_ms: 1000, decel_ms: 1000, timeout_sec: 30.0}'
```

## Generic motion command

`~/target_rpm`, `~/move_relative`, and `~/move_absolute` cover the common
cases (Vel Curr Abs for speed, Pos Vel Abs/Inc for position). For any other
motion mode from the protocol's motion mode table (p.134-136) -- Voltage,
Current, Velocity, or Position family, Target/Snap/Time/Time-Left variants,
Abs or Inc -- call `~/motion_command` directly with the raw
`setMotorMotionAll()` parameters. Like `~/target_rpm`, this is a "live"
command: it auto-stops if not repeated within `command_timeout_sec`, so for
anything that takes longer than that, re-send the same request periodically
(see examples/speed_node.cpp / examples/position_node.cpp for the polling
pattern).

All 32 defined modes are accepted, including Voltage-direct-drive
(0x10-0x1D) -- unlike the other three families, Voltage bypasses the
current loop entirely, so nothing regulates the resulting current except
V/R and back-EMF. Use it deliberately, with a conservative `main_target`
and `accel_ms`, and watch `~/status`/`~/fault` while doing it.

```bash
# Vel Time Abs (0x78 / 120): reach 50 rpm within 3000ms, current-limited to 10A
ros2 service call /woodrive/motion_command woodrive_ros2/srv/MotionCommand \
  '{accel_ms: 1000, decel_ms: 1000, motion_mode: 120, sub_target: 10.0, main_target: 50.0, direction: 1}'

# Curr Volt Abs (0x34 / 52): reach 2A current, voltage-limited to 24V
ros2 service call /woodrive/motion_command woodrive_ros2/srv/MotionCommand \
  '{accel_ms: 500, decel_ms: 500, motion_mode: 52, sub_target: 24.0, main_target: 2.0, direction: 1}'
```

## Gain / limit tuning

`~/get_gain_limit` (high-level) reads all 20 gain+limit fields in two
round-trips (`getMotorGainAll()` + `getMotorLimitAll()`, each already a
single contiguous register-block read). `~/set_gain_limit_field`
(low-level) writes exactly one field at a time via the matching individual
core setter -- see `command_utils.hpp`'s `isKnownGainLimitField()` for the
full list of accepted field names. No range clamping is applied; this is a
tuning interface for a human in the loop, not something to script blindly.

```bash
ros2 service call /woodrive/get_gain_limit woodrive_ros2/srv/GetGainLimit '{}'

ros2 service call /woodrive/set_gain_limit_field woodrive_ros2/srv/SetGainLimitField \
  '{field: velocity_p_gain, value: 500}'
```

## Safety services

```bash
ros2 service call /woodrive/stop std_srvs/srv/Trigger '{}'
ros2 service call /woodrive/enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /woodrive/brake std_srvs/srv/SetBool '{data: true}'
```
