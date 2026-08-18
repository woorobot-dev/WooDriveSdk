# playground/

Personal practice / experimentation space. **Not part of the WooDriveSDK
itself** -- nothing here is a released SDK component, an example someone
else should learn from, or a ROS2 package meant to be built as part of
this repo's real deliverables.

Use it for:
- Practicing the WooDrive SDK's API before writing real example/tool code
- ROS2 experiments (node/topic/service/action/parameter practice)
- Communication tests (serial, RS-485 timing, etc.)
- Personal temporary/throwaway code

## Rules

1. **Never gets discovered as a ROS2 package.** The `COLCON_IGNORE` marker
   file in this directory makes `colcon` skip it entirely during package
   discovery, even if a subfolder here ends up with its own `package.xml`
   while experimenting. Don't remove that marker.
2. **No dependency on this repo's real code.** Don't `#include` or link
   against `core/`, `linux/`, `ros2/woodrive_ros2/`, or
   `ros2/projects/amr_2wheel_robot/` from here, and don't have real SDK
   code depend on anything in `playground/`. If something built here turns
   out useful, promote it into the real SDK structure (`examples/`,
   `tools/`, or a proper `projects/` package) instead of linking back to
   this folder.
3. **Nothing in here is guaranteed to build or to stay.** Nest folders
   however you like (e.g. `cpp/`, `ros2/`, `linux/`, `lidar/`, `camera/`,
   `temp/`) -- there's no fixed structure. Failed/abandoned experiments are
   fine to leave or delete freely.
