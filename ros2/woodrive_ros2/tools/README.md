# tools/

Diagnostic/setup utilities for the WooDrive SDK -- **generic and
robot-agnostic**. Not the stable Example01-05 parity set (see
`../examples/README.md` for those), and not robot-specific code either
(that lives in its own package under `../../projects/`, e.g.
`../../projects/amr_2wheel_robot` for the 2-wheel AMR -- robot-specific
code doesn't belong inside the SDK's package, so if a tool here starts
assuming a particular robot's layout, move it out).

Unlike `examples/`, these aren't meant to teach the SDK's API step by step
-- they're what you actually reach for while bringing up hardware.

Current contents:

- **`set_id_node.cpp`** -- checks (and optionally sets) a WooDrive
  controller's protocol ID. Needed whenever a freshly-added controller
  needs to move off its default ID before sharing a bus with another one.
  Do this with only the target controller on the bus -- two controllers
  sharing an ID at once can both answer the same request and corrupt each
  other's responses.
- **`raw_io_check_node.cpp`** -- shows the raw RS-485 bytes (TX/RX hex),
  not just parsed values. Runs the same getId()/getFault() sequence as
  `examples/basic_check_node.cpp` with the wire traffic visible -- useful
  for cross-checking against the protocol doc's SET/GET/RSP hex examples.

Prerequisites work in this area tends to assume:

- RS-485 bus speed set to **1,000,000 bps** (not the SDK's 9600 default) --
  required for a 1-10ms comm/control cycle to be physically possible on the
  wire (~0.1ms/frame at 1Mbps vs ~10.4ms/frame at 9600). The controller
  does not ack `setBps()` itself (fire-and-forget), so reconnect at the new
  baud and confirm with a `getBps()` GET afterward.
