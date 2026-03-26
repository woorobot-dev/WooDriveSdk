#include "WooDriveSdk.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cstdint>

static const char* PORT_NAME = "/dev/ttyUSB0";
static const uint32_t WOODRIVE_BAUDRATE = 9600;
static const uint8_t TARGET_ID = 1;

int main()
{
    PosixSerialTransport serial(PORT_NAME, WOODRIVE_BAUDRATE);
    if (!serial.isOpen()) {
        std::cout << "serial open failed\n";
        return 1;
    }

    StdClock wooClock;
    WooDrive drive(serial, wooClock);

    drive.setTimeout(300);

    std::cout << "=== WooDrive Example01 : Basic Check ===\n";

    uint8_t id = 0;
    uint8_t fault = 0;

    if (drive.getId(TARGET_ID, id)) {
        std::cout << "ID OK : " << static_cast<unsigned long>(id) << "\n";
    } else {
        std::cout << "ID FAIL\n";
        return 0;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (drive.getFault(TARGET_ID, fault)) {
        std::cout << "FAULT : " << static_cast<unsigned long>(fault) << "\n";
    } else {
        std::cout << "FAULT READ FAIL\n";
        return 0;
    }

    std::cout << "=== RESULT : PASS ===\n";

    return 0;
}