#include "../core/WindowsSerialTransport.h"

#include <cstdint>
#include <iostream>

int main(int argc, char** argv)
{
    const char* port = argc > 1 ? argv[1] : "COM7";
    const int baudrate = argc > 2 ? std::stoi(argv[2]) : 9600;
    const uint8_t targetId = argc > 3 ? static_cast<uint8_t>(std::stoi(argv[3])) : 1;

    std::cout << "WooDrive read-only check: " << port << ", " << baudrate
              << " baud, target ID " << static_cast<unsigned>(targetId) << "\n";

    WindowsSerialTransport serial(port, baudrate);
    if (!serial.isOpen()) {
        std::cerr << "ERROR: cannot open " << port << " (Windows error "
                  << GetLastError() << ")\n";
        return 1;
    }

    WindowsClock clock;
    WooDrive drive(serial, clock);
    drive.setTimeout(100);

    uint8_t actualId = 0;
    if (!drive.getId(targetId, actualId)) {
        std::cerr << "FAIL: no ID response\n";
        return 2;
    }
    std::cout << "ID OK: " << static_cast<unsigned>(actualId) << "\n";

    uint8_t fault = 0;
    if (!drive.getFault(targetId, fault)) {
        std::cerr << "FAIL: no fault response\n";
        return 3;
    }
    std::cout << "FAULT: 0x" << std::hex << static_cast<unsigned>(fault) << std::dec << "\n";

    uint8_t polePairs = 0;
    if (!drive.getPolePairs(targetId, polePairs)) {
        std::cerr << "FAIL: no pole-pairs response\n";
        return 4;
    }
    std::cout << "POLE PAIRS: " << static_cast<unsigned>(polePairs) << "\n";
    std::cout << "RESULT: PASS\n";
    return fault == 0 ? 0 : 5;
}
