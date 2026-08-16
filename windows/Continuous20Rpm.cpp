#include "../core/WindowsSerialTransport.h"

#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <thread>
#include <windows.h>

namespace {
constexpr const char* kStopEventName = "Local\\WooDriveContinuous20RpmStop";
constexpr uint8_t kTargetId = 1;

void sleepMs(unsigned ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

bool command(WooDrive& drive, float rpm, uint8_t direction)
{
    return drive.setMotorMotionAll(kTargetId, 1000, 1000, 116, 10.0f, rpm, direction);
}

void safeStop(WooDrive& drive)
{
    command(drive, 0.0f, 0);
    sleepMs(300);
    drive.setMotorEnable(kTargetId, 0);
    sleepMs(100);
    drive.setMotorBrake(kTargetId, 1);
}
}

int main(int argc, char** argv)
{
    if (argc > 1 && std::string(argv[1]) == "--stop") {
        HANDLE event = OpenEventA(EVENT_MODIFY_STATE, FALSE, kStopEventName);
        if (!event) return 1;
        const bool ok = SetEvent(event) != FALSE;
        CloseHandle(event);
        return ok ? 0 : 2;
    }

    HANDLE stopEvent = CreateEventA(nullptr, TRUE, FALSE, kStopEventName);
    if (!stopEvent) return 1;

    std::ofstream log("continuous20.status.log", std::ios::trunc);

    WindowsSerialTransport serial("COM7", 9600);
    if (!serial.isOpen()) {
        CloseHandle(stopEvent);
        return 2;
    }

    WindowsClock clock;
    WooDrive drive(serial, clock);
    drive.setTimeout(100);

    uint8_t fault = 0;
    if (!drive.getFault(kTargetId, fault) || fault != 0 ||
        !drive.setMotorBrake(kTargetId, 1) ||
        !drive.setMotorEnable(kTargetId, 1) ||
        !command(drive, 20.0f, 1)) {
        safeStop(drive);
        CloseHandle(stopEvent);
        return 3;
    }

    log << "RUNNING: +20 RPM\n" << std::flush;
    bool healthy = true;
    while (WaitForSingleObject(stopEvent, 0) != WAIT_OBJECT_0) {
        WooDrive::MotorStatus status{};
        if (!drive.getFault(kTargetId, fault) || fault != 0 ||
            !drive.getMotorStatusAll(kTargetId, status)) {
            healthy = false;
            break;
        }
        log << "velocity=" << status.velocity
            << " fault=" << static_cast<unsigned>(fault) << "\n" << std::flush;
        sleepMs(250);
    }

    safeStop(drive);
    log << "STOPPED\n" << std::flush;
    CloseHandle(stopEvent);
    return healthy ? 0 : 4;
}
