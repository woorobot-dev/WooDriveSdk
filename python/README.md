# woodrive (Python bindings)

pybind11 bindings over [`core/WooDriveSdk.h`](../core/WooDriveSdk.h) — the
same protocol engine used by the Arduino, Raspberry Pi, Windows and ROS 2
targets. Ships a native serial transport (no `pyserial` dependency): it
compiles `PosixSerialTransport` on Linux/macOS and `WindowsSerialTransport`
on Windows, both from `core/`.

> **Status: scaffold.** `src/bindings.cpp` exposes a useful but partial
> subset of `core/WooDriveSdk.h` (id/fault, direction/phase, motor
> enable/brake, `setMotorMotionAll`, position/velocity, full status). Extend
> `src/bindings.cpp` with more `.def(...)` calls as you need more of the
> ~100 registers the C++ SDK exposes.

## Build

Requires a C++17 compiler and CMake — this repo could not verify the build
in the sandbox it was written in (no compiler available there), so build and
sanity-check it once locally before relying on it.

```bash
cd python
pip install -e .
```

This uses [scikit-build-core](https://scikit-build-core.readthedocs.io/) to
drive the CMake build in `CMakeLists.txt`, which compiles
`src/bindings.cpp` together with `../core/WooDriveSdk.cpp` (and
`../core/WindowsSerialTransport.cpp` on Windows) into the `_woodrive`
extension module.

## Usage

```python
import woodrive

transport = woodrive.SerialTransport("/dev/ttyUSB0", 9600)  # "COM5" on Windows
clock = woodrive.Clock()
drive = woodrive.WooDrive(transport, clock)
drive.set_timeout(500)

ok, device_id = drive.get_id(1)
ok, status = drive.get_motor_status_all(1)
print(status.position_deg, status.velocity_rpm)
```

See `examples/basic_check.py` for a complete runnable example.
