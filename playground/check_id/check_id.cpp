// playground/ communication test -- read-only device existence check.
//
// Sends a single safe GET request (WooDrive::getId(), protocol address
// 0x01) to one or more device IDs and reports whether a WooDrive
// controller answered. No SET requests, no motor enable/speed/position
// commands anywhere in this file -- this only ever calls getId().
//
// Reuses core/WooDriveSdk.h as-is (PosixSerialTransport, StdClock,
// WooDrive::getId()) -- no protocol framing/CRC logic is reimplemented
// here. See this folder's README.md for why getId() specifically, and for
// a known limitation (timeout vs CRC/protocol error can't be told apart
// through the SDK's public API).
//
// Usage:
//   check_id <port> <id>                       single ID check
//   check_id <port> --scan <start_id> <end_id>  scan a range of IDs
//   ... --baud <rate>                            override baud rate

#include "WooDriveSdk.h"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

namespace {

// Matches core/WooDriveSdk.h's own PosixSerialTransport default (see its
// constructor declaration). Override with --baud if your controller(s)
// are configured for a different rate.
constexpr uint32_t kDefaultBaudrate = 9600;

// Deliberate gap between scan requests so a --scan sweep doesn't hammer
// the bus back-to-back.
constexpr auto kScanDelay = std::chrono::milliseconds(50);

void printUsage(const char* argv0)
{
    std::fprintf(stderr,
        "Usage:\n"
        "  %s <port> <id> [--baud <rate>]\n"
        "  %s <port> --scan <start_id> <end_id> [--baud <rate>]\n"
        "\n"
        "Examples:\n"
        "  %s /dev/ttyUSB0 1\n"
        "  %s /dev/ttyUSB0 --scan 1 10\n"
        "  %s /dev/ttyUSB0 1 --baud 1000000\n",
        argv0, argv0, argv0, argv0, argv0);
}

// Sends WooDrive::getId(id, ...) -- one safe GET, protocol address 0x01 --
// and reports whether that ID answered.
bool checkOneId(WooDrive& drive, uint8_t id, bool verbose)
{
    uint8_t idRead = 0;
    const bool ok = drive.getId(id, idRead);
    if (ok) {
        if (verbose) std::printf("WooDrive detected: ID=%u\n", id);
        return true;
    }
    // core/'s public API has no way to tell "no bytes came back at all"
    // (timeout) apart from "a response came back but failed CRC/length/
    // address validation" (protocol error) -- getId() collapses both into
    // a plain `false`. See README.md.
    if (verbose) std::printf("No response from ID=%u\n", id);
    return false;
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc < 3) {
        printUsage(argv[0]);
        return 1;
    }

    const std::string port = argv[1];
    uint32_t baudrate = kDefaultBaudrate;
    bool scanMode = false;
    int scanStart = 0;
    int scanEnd = 0;
    int singleId = -1;

    int i = 2;
    if (std::strcmp(argv[i], "--scan") == 0) {
        scanMode = true;
        if (argc < i + 3) {
            printUsage(argv[0]);
            return 1;
        }
        scanStart = std::atoi(argv[i + 1]);
        scanEnd = std::atoi(argv[i + 2]);
        i += 3;
    } else {
        singleId = std::atoi(argv[i]);
        i += 1;
    }
    for (; i < argc; ++i) {
        if (std::strcmp(argv[i], "--baud") == 0 && i + 1 < argc) {
            baudrate = static_cast<uint32_t>(std::atol(argv[i + 1]));
            ++i;
        }
    }

    if (scanMode && (scanStart < 1 || scanEnd < scanStart || scanEnd > 255)) {
        std::fprintf(stderr, "Invalid scan range: %d..%d (must be 1..255, start<=end)\n", scanStart, scanEnd);
        return 1;
    }
    if (!scanMode && (singleId < 1 || singleId > 255)) {
        std::fprintf(stderr, "Invalid device ID: %d (must be 1..255)\n", singleId);
        return 1;
    }

    PosixSerialTransport serial(port.c_str(), baudrate);
    if (!serial.isOpen()) {
        std::fprintf(stderr, "Failed to open serial port: %s\n", port.c_str());
        return 1;
    }
    StdClock clock;
    WooDrive drive(serial, clock);
    drive.setTimeout(200);  // ms to wait for a response before declaring "no response"

    std::printf("check_id -- port=%s baud=%u\n", port.c_str(), baudrate);

    if (!scanMode) {
        const bool found = checkOneId(drive, static_cast<uint8_t>(singleId), /*verbose=*/true);
        return found ? 0 : 1;
    }

    std::printf("Scanning ID %d..%d\n", scanStart, scanEnd);
    int foundCount = 0;
    for (int id = scanStart; id <= scanEnd; ++id) {
        const bool found = checkOneId(drive, static_cast<uint8_t>(id), /*verbose=*/false);
        std::printf("ID %d : %s\n", id, found ? "FOUND" : "-");
        if (found) ++foundCount;
        if (id != scanEnd) std::this_thread::sleep_for(kScanDelay);
    }
    std::printf("Scan done -- %d of %d IDs responded\n", foundCount, scanEnd - scanStart + 1);
    return foundCount > 0 ? 0 : 1;
}
