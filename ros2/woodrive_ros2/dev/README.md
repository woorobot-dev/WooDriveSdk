# dev/

Growing playground nodes -- **not** the stable Example01-05 parity set (see
`../examples/README.md` for those). Everything here is expected to keep
changing shape as it grows toward a 2-wheel differential-drive demo robot
built on WooDrive.

Current state / direction:

- **`dual_motor_node.cpp`** -- talks to two WooDrive controllers (different
  protocol IDs) on one RS-485 bus. Currently a one-shot read loop over both
  IDs; next planned step is folding in the comm/control-thread pattern
  (dedicated thread polls the bus continuously into a shared/cached struct,
  a separate thread/callback reads the cache and decides -- see the
  session's `comm_control_demo.cpp` prototype) so the wheel-communication
  loop can run at 1-10ms instead of blocking per call.

Prerequisites this work assumes:

- RS-485 bus speed set to **1,000,000 bps** (not the SDK's 9600 default) --
  required for a 1-10ms cycle to be physically possible on the wire. Verify
  with `getBps()` after setting; the controller does not ack `setBps()`
  itself (fire-and-forget), so reconnect at the new baud and confirm with a
  GET.
- Second WooDrive controller present on the bus with a distinct ID (ID 2 by
  convention here) before `dual_motor_node`'s right-wheel path can be
  tested against real hardware rather than looped back onto ID 1.

Eventual direction (not yet started): once this node is doing robot-level
things -- `/cmd_vel` in, differential-drive kinematics, `/odom` + TF out --
it's meant to be promoted out of `woodrive_ros2` into its own package
(`woodrive_diffdrive`), keeping `woodrive_ros2` itself scoped to
"one generic WooDrive controller."
