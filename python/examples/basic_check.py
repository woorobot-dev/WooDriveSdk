"""Equivalent of raspberrypi/Example01_BasicCheck.cpp using the Python bindings.

Usage:
    python basic_check.py /dev/ttyUSB0      # Linux / macOS
    python basic_check.py COM5              # Windows
"""

import sys

import woodrive


def main() -> int:
    if len(sys.argv) < 2:
        print(f"usage: {sys.argv[0]} <serial-port> [baudrate]")
        return 1

    port = sys.argv[1]
    baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 9600
    target_id = 1

    transport = woodrive.SerialTransport(port, baudrate)
    if not transport.is_open():
        print(f"Failed to open {port}")
        return 1

    clock = woodrive.Clock()
    drive = woodrive.WooDrive(transport, clock)
    drive.set_timeout(500)

    ok, actual_id = drive.get_id(target_id)
    if not ok:
        print("WooDrive did not respond")
        return 1
    print(f"Connected. Device ID = {actual_id}")

    ok, fault = drive.get_fault(target_id)
    print(f"Fault = 0x{fault:02X}" if ok else "Failed to read fault")

    ok, status = drive.get_motor_status_all(target_id)
    if ok:
        print(f"position={status.position_deg:.2f} deg  velocity={status.velocity_rpm:.2f} rpm")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
