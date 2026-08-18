# check_id

Tiny playground communication test: is a WooDrive controller present at a
given protocol ID? Read-only -- sends `WooDrive::getId()` (a GET request,
protocol address `0x01`) and nothing else. No SET requests, no motor
enable, no speed/position commands.

This is **not** part of the SDK (`core/`, `linux/`, `ros2/`) -- it lives
entirely under `playground/` (see `../COLCON_IGNORE` / `../README.md`) and
is not built or discovered by anything else in the repo.

## What it reuses from the SDK

- `PosixSerialTransport`, `StdClock`, `WooDrive` -- exactly as every other
  example in this repo constructs them (`linux/examples/Example01_BasicCheck.cpp`
  is the closest analog).
- `WooDrive::getId(uint8_t id, uint8_t& outValue)` -- the same GET request
  every existence check in this SDK already uses (e.g.
  `linux/examples/Example01_BasicCheck.cpp`, `woodrive_ros2/examples/basic_check_node.cpp`).
  No new protocol logic; no framing/CRC code was written for this tool.

## Known limitation (not fixed here -- core/ is out of scope this round)

`core/WooDriveSdk.h`'s `readFrame()`/`sendAndReceive()`/`checkRsp()` --
the pieces that would know *why* a request failed -- are `private`. The
only public surface (`getId()`) returns a plain `bool`, so this tool
**cannot tell a real timeout (no response at all) apart from a response
that arrived but failed CRC/length/address validation.** Both print as
`No response from ID=N`. Distinguishing them would need either
reimplementing frame parsing here (not allowed -- reuse only) or a new
public accessor added to `core/` (out of scope for a playground tool this
round).

## Build

```bash
cd playground/check_id
make
```

## Run

```bash
# single ID
./check_id /dev/ttyUSB0 1

# scan a range (50ms gap between requests, does not flood the bus)
./check_id /dev/ttyUSB0 --scan 1 10

# override baud rate (default 9600, matching core/'s own PosixSerialTransport default --
# if your controller was reconfigured to 1,000,000 like this session's hardware, pass it explicitly)
./check_id /dev/ttyUSB0 1 --baud 1000000
```

## Output

Single ID:
```
check_id -- port=/dev/ttyUSB0 baud=9600
WooDrive detected: ID=1
```
or
```
check_id -- port=/dev/ttyUSB0 baud=9600
No response from ID=1
```

Scan:
```
check_id -- port=/dev/ttyUSB0 baud=9600
Scanning ID 1..10
ID 1 : FOUND
ID 2 : -
ID 3 : FOUND
...
Scan done -- 2 of 10 IDs responded
```

Exit code is `0` if at least one ID responded, `1` otherwise (including
serial-open failure).
