# dev/

Growing playground for **generic, robot-agnostic** WooDrive SDK utilities --
not the stable Example01-05 parity set (see `../examples/README.md` for
those), and not robot-specific code either (that lives in its own package
under `../../Robot/`, e.g. `../../Robot/amr_2wheel_robot` for the 2-wheel
AMR -- robot-specific code doesn't belong inside the SDK's package, so if
a tool here starts assuming a particular robot's layout, move it out).

Current contents:

- **`set_id_node.cpp`** -- checks (and optionally sets) a WooDrive
  controller's protocol ID. Needed whenever a freshly-added controller
  needs to move off its default ID before sharing a bus with another one.
  Do this with only the target controller on the bus -- two controllers
  sharing an ID at once can both answer the same request and corrupt each
  other's responses.

Prerequisites work in this area tends to assume:

- RS-485 bus speed set to **1,000,000 bps** (not the SDK's 9600 default) --
  required for a 1-10ms comm/control cycle to be physically possible on the
  wire (~0.1ms/frame at 1Mbps vs ~10.4ms/frame at 9600). The controller
  does not ack `setBps()` itself (fire-and-forget), so reconnect at the new
  baud and confirm with a `getBps()` GET afterward.
