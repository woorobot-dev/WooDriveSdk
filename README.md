# WooDrive SDK

Control BLDC motors with simple commands using the WooDrive SDK.
Supports FOC-based control over RS-485 — one C++ core, used from Arduino,
Raspberry Pi / Linux, Windows, ROS 2, and Python.

---

## What you can do

- BLDC / PMSM motor control
- Speed / Position / Current control
- High-performance FOC-based control
- Multi-controller daisy-chain via RS-485

No complex control implementation needed — just send a command and it works.

---

## System Overview

```
Arduino / Raspberry Pi / Windows / ROS 2 / Python  →  RS-485  →  WooDrive  →  Motor
```

---

## Project Structure

`core/` is the single platform-agnostic protocol implementation (packet
framing, CRC, every register read/write). Everything else is a thin,
platform-specific transport plus examples built on top of it.

```
WooDriveSdk/
├─ core/                          # platform-agnostic SDK — edit here first
│   ├─ WooDriveSdk.h / .cpp        # WooDrive protocol engine + ITransport/IClock
│   │                              #  (embeds ArduinoTransport and the POSIX
│   │                              #   transport behind #if defined(...))
│   └─ WindowsSerialTransport.h / .cpp
│
├─ arduino/                       # Arduino Library Manager package.
│   ├─ WooDriveSdk.h / .cpp        # generated copy of core/ — do not hand-edit,
│   │                              #  run scripts/sync-arduino.sh instead
│   └─ examples/
│       ├─ Example01_BasicCheck/
│       ├─ Example02_ReadStatus/
│       ├─ Example03_AutoMotorSetup/
│       ├─ Example04_Speed/
│       └─ Example05_Position/
│
├─ raspberrypi/                   # Linux examples, builds against ../core
│   ├─ Makefile
│   ├─ Example01_BasicCheck.cpp
│   ├─ Example02_ReadStatus.cpp
│   ├─ Example03_AutoMotorSetup.cpp
│   ├─ Example04_Speed.cpp
│   └─ Example05_Position.cpp
│
├─ windows/                       # Windows examples, builds against ../core
│   ├─ CMakeLists.txt
│   ├─ BasicCheck.cpp / LowSpeedRun.cpp / Continuous20Rpm.cpp / Position360.cpp
│
├─ ros2/woodrive_ros2/            # ROS 2 package, builds against ../../core
│   ├─ src/woodrive_node.cpp       # persistent driver: topics/services/actions + watchdog
│   └─ examples/                   # 5 standalone nodes, no topics/services (see its README)
│
├─ python/                        # pybind11 bindings over core/ (in progress)
│
└─ scripts/                       # sync-arduino / check-arduino-sync (.sh + .ps1)
```

---

## Arduino Usage

Open the following file in Arduino IDE and upload:

```
arduino/examples/Example01_BasicCheck/Example01_BasicCheck.ino
```

---

## Raspberry Pi Usage

**Build**
```bash
cd raspberrypi
make
```

**Run**
```bash
./Example01_BasicCheck
```

**Check USB port**
```bash
ls /dev/ttyUSB*
ls /dev/ttyACM*
```

**Set permissions**
```bash
sudo chmod 777 /dev/ttyUSB0
# or
sudo chmod 777 /dev/ttyACM0
```

**Troubleshooting**
```bash
lsusb
dmesg | tail -n 30
```

---

## Example Overview

| Example | Description |
|---|---|
| Example01_BasicCheck | Check connection |
| Example02_ReadStatus | Read motor status |
| Example03_AutoMotorSetup | Auto motor setup (no manual tuning) |
| Example04_Speed | Speed control |
| Example05_Position | Position control |

---

## Quick Start

```bash
git clone https://github.com/woorobot-dev/WooDriveSdk.git
cd WooDriveSdk/raspberrypi
make
./Example01_BasicCheck
```

## Windows / ROS 2 / Python

- **Windows**: `cd windows && cmake -B build && cmake --build build` (see `windows/CMakeLists.txt`).
- **ROS 2**: `colcon build --packages-select woodrive_ros2` from a workspace containing `ros2/woodrive_ros2`.
- **Python**: see `python/README.md` (pybind11 bindings over `core/`, work in progress).

## Contributing to core/

`core/WooDriveSdk.{h,cpp}` is the single source of truth for the protocol.
`raspberrypi/`, `windows/` and `ros2/` all build it directly by relative path,
so changes there apply everywhere automatically. `arduino/` is the one
exception — the Arduino Library Manager needs a self-contained folder, so
`arduino/WooDriveSdk.{h,cpp}` is a **generated copy** of `core/`:

```bash
# after editing core/WooDriveSdk.h or core/WooDriveSdk.cpp
scripts/sync-arduino.sh        # (or scripts\sync-arduino.ps1 on Windows)
scripts/check-arduino-sync.sh  # verifies arduino/ matches core/, e.g. in CI
```

## Protocol compatibility

The SDK supports WooDrive Protocol V1.1.0, where a fault response uses
direction value `0x00`. Fault responses using the legacy `0x04` direction are
also accepted for compatibility with previously shipped WooDrive devices.
Address `0x13` uses the
`DirectionPhase` values below:

| Value | Meaning |
|---|---|
| `DirectionPhase::Normal` | Normal direction and phase |
| `DirectionPhase::DirectionInverted` | Invert the command direction |
| `DirectionPhase::PhaseSwapped` | Swap the U/V phases |
| `DirectionPhase::DirectionInvertedAndPhaseSwapped` | Invert direction and swap U/V phases |

```cpp
drive.setDirectionPhase(1, DirectionPhase::PhaseSwapped);
```

Direction/phase values are passed through as an unsigned byte so newer
firmware values are not rejected by the SDK.

Changing the phase invalidates the existing controller setup. Run the motor
setup procedure again before operating the motor. The legacy
`setDirectionInvert` and `getDirectionInvert` APIs remain available for source
compatibility with Protocol V1.0.3 applications.
