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

## Safety services

```bash
ros2 service call /woodrive/stop std_srvs/srv/Trigger '{}'
ros2 service call /woodrive/enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /woodrive/brake std_srvs/srv/SetBool '{data: true}'
```
