# amr_2wheel_robot

Robot-specific package for a 2-wheel AMR (Autonomous Mobile Robot) built on
two WooDrive controllers (one per wheel, distinct protocol IDs on one
RS-485 bus). Lives under `ros2/projects/` -- a grouping folder for
robot-level packages, as opposed to `ros2/woodrive_ros2` (the generic,
robot-agnostic WooDrive SDK package). Kept **separate** from
`woodrive_ros2` on purpose -- this package hardcodes robot-specific
concepts (left/right wheel) that don't belong inside a generic SDK. It
links directly against `../../../core/` (the same SDK `woodrive_ros2`
uses); no WooDrive protocol logic is reimplemented here.

## Stage 1 purpose

Build and verify a solid **left/right dual-axis control foundation**
before any driving logic exists: bring up two WooDrive controllers on one
bus, confirm both are individually reachable, and be able to enable,
disable, and command a speed on each wheel independently -- with safety
behavior (command timeout, fault handling, clean shutdown) already in
place. Nothing here computes how those two numbers relate to the robot
actually moving in a line or turning; that's Stage 2 (see below).

## Required hardware

- 2x WooDrive controller, each wired to its own motor
- Both controllers on **one shared RS-485 bus**, each with a **distinct
  protocol ID** (this package's convention: left = 1, right = 2)
- USB-RS485 adapter from the host (Jetson/PC/Raspberry Pi) to that bus

### WooDrive ID example

Both controllers ship at the same default ID, so before wiring both onto
one bus, set them one at a time (only one controller connected while doing
this -- two controllers sharing an ID at once can both answer the same
request and corrupt each other's responses):

```bash
# with only the future left-wheel controller connected, confirm/leave it at ID 1
ros2 run woodrive_ros2 set_id_node --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=1000000 -p current_id:=1

# with only the future right-wheel controller connected, move it to ID 2
ros2 run woodrive_ros2 set_id_node --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=1000000 -p current_id:=1 -p new_id:=2
```

Also set the bus to **1,000,000 bps** on both controllers (not the SDK's
9600 default) -- see `woodrive_ros2/tools/README.md` for why and how.

## Configuration

`config/amr_2wheel.yaml`:

| Parameter | Default | Meaning |
|---|---|---|
| `port` | `/dev/ttyUSB0` | Serial port |
| `baudrate` | `1000000` | Must match both controllers' actual setting |
| `left_id` / `right_id` | `1` / `2` | Protocol ID wired as left / right |
| `left_invert` / `right_invert` | `false` | Flip if that wheel is mechanically mounted mirrored (keeps positive = forward consistent) |
| `motion_mode` | `116` (Vel Curr Abs) | Default control mode -- current-limited velocity target |
| `sub_target` | `5.0` | Current limit (A) for that mode |
| `accel_ms` / `decel_ms` | `500` / `500` | Ramp time |
| `max_rpm` | `100.0` | Hard clamp -- any commanded rpm outside +/-this is clipped |
| `command_timeout_ms` | `500` | No `target_rpm` refresh within this window -> that wheel goes to 0 rpm |
| `test_rpm` | `20.0` | Suggested value for the first manual test below |

`motion_mode`/`sub_target`/`accel_ms`/`decel_ms` aren't in most "minimum
config" checklists for this kind of node, but `setMotorMotionAllSigned()`
(the `core/` call used to send a speed command) structurally requires all
of them -- they're exposed as parameters rather than hardcoded so they can
be tuned without a rebuild.

No `wheel_radius` / `wheel_separation` yet -- not needed until Stage 2's
kinematics math exists, and guessing them now would just be wrong later.

## Build

```bash
colcon build --packages-select amr_2wheel_robot
source install/setup.bash
```

## Launch

```bash
ros2 launch amr_2wheel_robot amr_2wheel.launch.py
# or with a different config file:
ros2 launch amr_2wheel_robot amr_2wheel.launch.py config:=/path/to/other.yaml
```

## Topics / services (Stage 1 test interface)

All plain `std_msgs`/`std_srvs` -- no custom message types.

| Name | Type | Direction | Purpose |
|---|---|---|---|
| `~/left/target_rpm` | `std_msgs/Float32` | sub | Set left wheel's signed target rpm |
| `~/right/target_rpm` | `std_msgs/Float32` | sub | Set right wheel's signed target rpm |
| `~/left/enable` | `std_srvs/SetBool` | service | Enable/disable the left wheel |
| `~/right/enable` | `std_srvs/SetBool` | service | Enable/disable the right wheel |
| `~/stop` | `std_srvs/Trigger` | service | 0 rpm + disable, both wheels |
| `~/left/velocity_rpm`, `~/right/velocity_rpm` | `std_msgs/Float32` | pub | Current measured velocity |
| `~/left/fault`, `~/right/fault` | `std_msgs/UInt8` | pub | Current fault code |

## Testing left/right motors manually

```bash
# 1) enable both wheels
ros2 service call /dual_motor_node/left/enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /dual_motor_node/right/enable std_srvs/srv/SetBool '{data: true}'

# 2) command the left wheel only, at the suggested test_rpm
ros2 topic pub -r 10 /dual_motor_node/left/target_rpm std_msgs/msg/Float32 '{data: 20.0}'
# Ctrl+C to stop publishing -- command_timeout_ms will bring it back to 0 rpm on its own

# 3) watch it
ros2 topic echo /dual_motor_node/left/velocity_rpm
ros2 topic echo /dual_motor_node/left/fault

# 4) repeat for the right wheel with /dual_motor_node/right/target_rpm
```

`target_rpm` is a "live" command like `woodrive_ros2`'s `~/target_rpm`:
publish once and it will be zeroed automatically after `command_timeout_ms`
if not refreshed. Use `-r 10` (10 Hz) to keep a wheel running continuously.

## Emergency stop / shutdown

- **From another terminal, any time:** `ros2 service call /dual_motor_node/stop std_srvs/srv/Trigger '{}'`
- **`Ctrl+C` the node (or `ros2 launch` process):** the node's destructor
  always sends 0 rpm, then disables, then brakes both wheels before the
  process exits -- this runs even if you interrupt mid-command.
- **If either wheel reports a fault:** both wheels are automatically
  stopped (not just the faulted one) -- see "Safety behavior" below.

## Safety behavior

- **Command timeout:** if a wheel's last `target_rpm` (or `enable`) is
  older than `command_timeout_ms`, that wheel is driven to 0 rpm every
  watchdog tick until a fresh command arrives.
- **Shutdown:** node destructor sends 0 rpm, then `setMotorEnable(0)`,
  then `setMotorBrake(1)`, to both wheels, after the comm thread has
  stopped (no other code touches the bus by then).
- **Fault on either wheel stops both:** a 2-wheel robot isn't safely
  controllable on only one working wheel, so any nonzero fault code from
  either controller queues a stop for both.
- **No busy-wait on serial errors:** the comm thread's poll loop sleeps
  every iteration regardless of whether the last read/write succeeded --
  a disconnected bus does not spin a CPU core.
- **`max_rpm` clamp:** every incoming `target_rpm` is clamped to
  +/-`max_rpm` before it reaches the motor, regardless of what value was
  published.

## Not implemented yet

This is Stage 1 only. None of the following exist in this package yet:

- `/cmd_vel` input
- Differential-drive kinematics (left/right rpm <-> linear/angular velocity)
- Odometry (`/odom`) or `odom` -> `base_link` TF
- `ros2_control` integration
- Nav2
- LiDAR / camera / any sensor integration

## Testing this package

**Without hardware:**
```bash
colcon build --packages-select amr_2wheel_robot woodrive_ros2  # build
colcon test --packages-select amr_2wheel_robot                # unit tests (motor_command_utils)
ros2 launch amr_2wheel_robot amr_2wheel.launch.py --show-args  # launch file syntax
```
Running the node itself against a nonexistent port also verifies the
startup-failure path (task requirement: name which device failed) without
needing real hardware -- see "Development verification" in this repo's
session notes, or just try `-p port:=/dev/ttyUSB9`.

**With hardware (2 real WooDrive controllers):** see the top-level
project's test procedure -- reachability check, enable/disable each side,
low-rpm move on each side individually, command-timeout auto-stop,
`~/stop`, and Ctrl+C shutdown.
