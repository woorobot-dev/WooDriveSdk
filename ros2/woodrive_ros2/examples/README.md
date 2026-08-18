# WooDrive ROS 2 examples

Five small, standalone nodes -- no topics, services, or actions. Each one
connects, runs through one feature, prints the result via `RCLCPP_INFO`, and
exits. They are the ROS 2 counterpart of `linux/examples/Example01..05_*.cpp` /
`arduino/examples/Example01..05_*`, built directly against `core/WooDriveSdk.h`.

For a persistent driver you can actually run a robot with (topics, services,
actions, a 200ms safety watchdog), use `woodrive_node` -- see the
[package README](../README.md) -- instead.

All five accept the same parameters:

| Parameter | Default | Meaning |
|---|---|---|
| `port` | `/dev/ttyUSB0` | Serial port |
| `baudrate` | `9600` | Serial baud rate |
| `target_id` | `1` | WooDrive controller ID |

`auto_motor_setup_node` additionally accepts `pole_pairs` (default `4`).

## basic_check_node

Connects, reads ID and fault, prints pass/fail.

```bash
ros2 run woodrive_ros2 basic_check_node --ros-args -p port:=/dev/ttyUSB0
```

## read_status_node

Dumps every config/status block once (board, motor, FOC, param, gain, limit,
control, status).

```bash
ros2 run woodrive_ros2 read_status_node --ros-args -p port:=/dev/ttyUSB0
```

## auto_motor_setup_node

Runs the controller's auto motor setup procedure and polls for completion
(up to 240s). **Warning: this changes motor parameters and may move the
motor.** Secure the mechanism first.

```bash
ros2 run woodrive_ros2 auto_motor_setup_node --ros-args -p port:=/dev/ttyUSB0 -p pole_pairs:=4
```

## speed_node

Runs +100 rpm, stops, -100 rpm, stops, then demonstrates DIR_ZERO
(velocity mode: decelerate to a stop), stops.

```bash
ros2 run woodrive_ros2 speed_node --ros-args -p port:=/dev/ttyUSB0
```

## position_node

Homes to position 0 (DIR_ZERO in position mode -- see the note in the source),
moves +360°, moves -360°, homes again.

```bash
ros2 run woodrive_ros2 position_node --ros-args -p port:=/dev/ttyUSB0
```
