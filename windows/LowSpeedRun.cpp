#include "../core/WindowsSerialTransport.h"

#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <thread>

namespace {
constexpr uint8_t kTargetId = 1;
constexpr uint16_t kAccelMs = 1000;
constexpr uint16_t kDecelMs = 1000;
constexpr uint8_t kMotionMode = 116;
constexpr float kSubTarget = 10.0f;
constexpr float kTargetRpm = 20.0f;
constexpr uint8_t kStopDirection = 0;
constexpr uint8_t kPositiveDirection = 1;

void sleepMs(unsigned ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

bool command(WooDrive& drive, float rpm, uint8_t direction)
{
    return drive.setMotorMotionAll(kTargetId, kAccelMs, kDecelMs,
                                   kMotionMode, kSubTarget, rpm, direction);
}

void safeStop(WooDrive& drive)
{
    command(drive, 0.0f, kStopDirection);
    sleepMs(300);
    drive.setMotorEnable(kTargetId, 0);
    sleepMs(100);
    drive.setMotorBrake(kTargetId, 1);
}
}

int main()
{
    WindowsSerialTransport serial("COM7", 9600);
    if (!serial.isOpen()) {
        std::cerr << "FAIL: cannot open COM7\n";
        return 1;
    }

    WindowsClock clock;
    WooDrive drive(serial, clock);
    drive.setTimeout(100);

    uint8_t id = 0;
    uint8_t fault = 0;
    if (!drive.getId(kTargetId, id) || !drive.getFault(kTargetId, fault)) {
        std::cerr << "FAIL: communication check\n";
        return 2;
    }
    if (fault != 0) {
        std::cerr << "ABORT: pre-run fault 0x" << std::hex
                  << static_cast<unsigned>(fault) << std::dec << "\n";
        return 3;
    }

    if (!drive.setMotorBrake(kTargetId, 1) ||
        !drive.setMotorEnable(kTargetId, 1)) {
        std::cerr << "FAIL: motor enable\n";
        safeStop(drive);
        return 4;
    }

    if (!command(drive, kTargetRpm, kPositiveDirection)) {
        std::cerr << "FAIL: speed command\n";
        safeStop(drive);
        return 5;
    }

    bool healthy = true;
    for (int i = 0; i < 10; ++i) {
        WooDrive::MotorStatus status{};
        if (!drive.getFault(kTargetId, fault) ||
            !drive.getMotorStatusAll(kTargetId, status)) {
            std::cerr << "FAIL: status read\n";
            healthy = false;
            break;
        }
        std::cout << "t=" << std::fixed << std::setprecision(1) << i * 0.2
                  << "s velocity=" << status.velocity
                  << " fault=0x" << std::hex << static_cast<unsigned>(fault)
                  << std::dec << "\n";
        if (fault != 0) {
            healthy = false;
            break;
        }
        sleepMs(200);
    }

    safeStop(drive);
    std::cout << "STOP: speed zero, motor disabled\n";
    return healthy ? 0 : 6;
}
