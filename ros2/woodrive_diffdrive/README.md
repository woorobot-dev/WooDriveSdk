# woodrive_diffdrive

Robot-specific package for a 2-wheel differential-drive robot built on two
WooDrive controllers (one per wheel, distinct protocol IDs on one RS-485
bus). Kept **separate** from `woodrive_ros2` (the generic, robot-agnostic
WooDrive SDK package) on purpose -- this package hardcodes robot-specific
concepts (left/right wheel, eventually wheel radius/separation for
kinematics and odometry) that don't belong inside a generic SDK. It links
directly against `../../core/` (the same SDK `woodrive_ros2` uses).

## Current state

- **`dual_motor_node`** -- dual-motor comm/control groundwork: a dedicated
  comm thread polls both wheel IDs into a shared cache; a control loop
  reads the cache without touching the bus. See the file's own header
  comment for the full architecture writeup.

## Planned next

- `/cmd_vel` (`geometry_msgs/Twist`) in -> differential-drive kinematics ->
  per-wheel RPM targets
- Wheel encoder feedback -> `/odom` (`nav_msgs/Odometry`) + `odom` ->
  `base_link` TF out
- Physical parameters (wheel radius, wheel separation) once the chassis is
  built and measured

## Prerequisites

- Both controllers set to distinct protocol IDs (see `woodrive_ros2`'s
  `dev/set_id_node` -- do this with only one controller on the bus at a
  time)
- RS-485 bus at 1,000,000 bps, not the SDK's 9600 default (required for the
  1-10ms comm/control cycle to be physically possible)

## Run

```bash
ros2 run woodrive_diffdrive dual_motor_node --ros-args \
  -p port:=/dev/ttyUSB0 -p baudrate:=1000000 -p left_id:=1 -p right_id:=2
```
