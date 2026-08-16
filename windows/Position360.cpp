#include "../core/WindowsSerialTransport.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <thread>

namespace {
constexpr uint8_t kId = 1;

void sleepMs(unsigned ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void safeStop(WooDrive& drive)
{
    drive.setMotorEnable(kId, 0);
    sleepMs(100);
    drive.setMotorBrake(kId, 1);
}
}

int main()
{
    WindowsSerialTransport serial("COM7", 9600);
    if (!serial.isOpen()) return 1;

    WindowsClock clock;
    WooDrive drive(serial, clock);
    drive.setTimeout(100);

    uint8_t fault = 0;
    WooDrive::MotorStatus before{};
    if (!drive.getFault(kId, fault) || fault != 0 ||
        !drive.getMotorStatusAll(kId, before)) {
        std::cerr << "ABORT: pre-run check failed, fault="
                  << static_cast<unsigned>(fault) << "\n";
        return 2;
    }

    std::cout << "START position=" << before.position << " deg\n";
    if (!drive.setMotorBrake(kId, 1) || !drive.setMotorEnable(kId, 1)) {
        safeStop(drive);
        return 3;
    }

    // Mode 245 is relative position control. Arm the mode with DIR_ZERO,
    // then request one +360 degree relative move and let WooDrive settle.
    if (!drive.setMotorMotionAll(kId, 1000, 1000, 245, 20.0f, 360.0f, 0)) {
        safeStop(drive);
        return 4;
    }
    // The official example completes the zero-direction run, disables the
    // drive, and enables it again before issuing the relative move.
    sleepMs(1000);
    safeStop(drive);
    sleepMs(1000);
    if (!drive.setMotorBrake(kId, 1) || !drive.setMotorEnable(kId, 1)) {
        safeStop(drive);
        return 4;
    }
    sleepMs(100);
    WooDrive::MotorStatus motionStart{};
    if (!drive.getMotorStatusAll(kId, motionStart)) {
        safeStop(drive);
        return 4;
    }
    std::cout << "MOTION_START position=" << motionStart.position << " deg\n";
    if (!drive.setMotorMotionAll(kId, 1000, 1000, 245, 20.0f, 360.0f, 1)) {
        safeStop(drive);
        return 4;
    }

    bool healthy = true;
    bool settled = false;
    int settledSamples = 0;
    for (int i = 0; i < 200; ++i) {
        WooDrive::MotorStatus status{};
        if (!drive.getFault(kId, fault) ||
            !drive.getMotorStatusAll(kId, status) || fault != 0) {
            healthy = false;
            break;
        }
        std::cout << "position=" << std::fixed << std::setprecision(2) << status.position
                  << " velocity=" << status.velocity
                  << " fault=" << static_cast<unsigned>(fault) << "\n";

        const float delta = status.position - motionStart.position;
        if (std::abs(delta) > 540.0f) {
            std::cerr << "ABORT: relative move exceeded 540 degrees\n";
            healthy = false;
            break;
        }
        if (std::abs(delta) > 300.0f && std::abs(status.velocity) < 1.0f) {
            if (++settledSamples >= 5) {
                settled = true;
                break;
            }
        } else {
            settledSamples = 0;
        }
        sleepMs(50);
    }

    WooDrive::MotorStatus after{};
    const bool finalRead = drive.getMotorStatusAll(kId, after);
    safeStop(drive);
    if (finalRead) {
        std::cout << "FINAL position=" << after.position
                  << " delta=" << (after.position - motionStart.position) << " deg\n";
    }
    std::cout << "STOPPED AND DISABLED\n";
    return healthy && finalRead && settled ? 0 : 5;
}
