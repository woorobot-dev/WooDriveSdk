# WooDrive SDK (Arduino / Raspberry Pi)

Control BLDC motors with simple commands using the WooDrive SDK.
Supports FOC-based control over RS-485 — works with Arduino and Raspberry Pi.

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
Arduino / Raspberry Pi  →  RS-485  →  WooDrive  →  Motor
```

---

## Project Structure

```
WooDriveSdk/
├─ arduino/
│   ├─ WooDriveSdk.h
│   ├─ WooDriveSdk.cpp
│   └─ examples/
│       ├─ Example01_BasicCheck/
│       ├─ Example02_ReadStatus/
│       ├─ Example03_AutoMotorSetup/
│       ├─ Example04_Speed/
│       └─ Example05_Position/
│
├─ raspberrypi/
│   ├─ Makefile
│   ├─ WooDriveSdk.h
│   ├─ WooDriveSdk.cpp
│   ├─ Example01_BasicCheck.cpp
│   ├─ Example02_ReadStatus.cpp
│   ├─ Example03_AutoMotorSetup.cpp
│   ├─ Example04_Speed.cpp
│   └─ Example05_Position.cpp
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
