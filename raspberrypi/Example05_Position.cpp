#include "WooDriveSdk.h"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <cstdint>

static const char* PORT_NAME = "/dev/ttyUSB0";
static const uint32_t WOODRIVE_BAUDRATE = 9600;
static const uint8_t TARGET_ID = 1;

static const uint16_t ACCEL_TIME_MS = 1000;
static const uint16_t DECEL_TIME_MS = 1000;
// static const uint8_t MOTION_MODE = 244;
static const uint8_t MOTION_MODE = 245;

static const uint8_t DIR_ZERO = 0;
static const uint8_t DIR_POSITIVE = 1;
static const uint8_t DIR_NEGATIVE = 2;

static const float SUB_TARGET = 100.0f;
static const float MAIN_TARGET_POS = 360.0f;

static const unsigned long RUN_MONITOR_MS = 5000UL;

#define STOP_MODE_COAST          1
#define STOP_MODE_DYNAMIC_BRAKE  2
static const uint8_t STOP_MODE = STOP_MODE_COAST;

static PosixSerialTransport serial(PORT_NAME, WOODRIVE_BAUDRATE);
static StdClock wooClock;
static WooDrive drive(serial, wooClock);

static void sleepMs(unsigned long ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

static void printLine()
{
    std::cout << "==================================================\n";
}

static void printTitle(const char* title)
{
    std::cout << "\n";
    printLine();
    std::cout << title << "\n";
    printLine();
}

static void printStep(const char* name, bool ok)
{
    std::cout << name << " : " << (ok ? "OK" : "FAIL") << "\n";
}

static void printU8(const char* name, uint8_t v)
{
    std::cout << name << " : " << static_cast<unsigned long>(v) << "\n";
}

static void printF32(const char* name, float v, uint8_t digits = 2)
{
    std::cout << name << " : " << std::fixed << std::setprecision(digits) << v << "\n";
}

static void printFaultHex(const char* name, uint8_t v)
{
    std::cout << name << " : 0x"
              << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
              << static_cast<unsigned int>(v)
              << std::dec << std::nouppercase << std::setfill(' ') << "\n";
}

static void printPulseInline(int64_t v)
{
    std::cout << v;
}

static void printStatusLine(const WooDrive::MotorStatus& s, uint8_t fault)
{
    std::cout << "F:" << static_cast<unsigned long>(fault);
    std::cout << " | P:" << std::fixed << std::setprecision(2) << s.position;
    std::cout << " | V:" << std::fixed << std::setprecision(2) << s.velocity;
    std::cout << " | Iq:" << std::fixed << std::setprecision(3) << s.iqCurrent;
    std::cout << " | Id:" << std::fixed << std::setprecision(3) << s.idCurrent;
    std::cout << " | BusV:" << std::fixed << std::setprecision(2) << s.busVoltage;
    std::cout << " | Temp:" << std::fixed << std::setprecision(2) << s.temperature;
    std::cout << " | Pulse:";
    printPulseInline(s.pulseCount);
    std::cout << "\n";
}

static bool checkFaultZero()
{
    uint8_t fault = 0;
    bool ok = drive.getFault(TARGET_ID, fault);
    printStep("getFault", ok);
    if (!ok) return false;

    printFaultHex("Fault", fault);
    return (fault == 0x00);
}

static bool setRunReady()
{
    bool ok;

    ok = drive.setMotorBrake(TARGET_ID, 1);
    printStep("setMotorBrake(1)", ok);
    if (!ok) return false;
    sleepMs(100);

    ok = drive.setMotorEnable(TARGET_ID, 1);
    printStep("setMotorEnable(1)", ok);
    if (!ok) return false;
    sleepMs(100);

    return true;
}

static bool stopCoast()
{
    bool ok;

    ok = drive.setMotorEnable(TARGET_ID, 0);
    printStep("setMotorEnable(0)", ok);
    sleepMs(100);

    ok = drive.setMotorBrake(TARGET_ID, 1);
    printStep("setMotorBrake(1)", ok);
    sleepMs(100);

    return ok;
}

static bool stopDynamicBrake()
{
    bool ok;

    ok = drive.setMotorEnable(TARGET_ID, 0);
    printStep("setMotorEnable(0)", ok);
    sleepMs(100);

    ok = drive.setMotorBrake(TARGET_ID, 0);
    printStep("setMotorBrake(0)", ok);
    sleepMs(100);

    return ok;
}

static bool stopMotor()
{
    printTitle("STOP MOTOR");

    if (STOP_MODE == STOP_MODE_DYNAMIC_BRAKE) {
        std::cout << "Stop Mode : Dynamic Brake\n";
        return stopDynamicBrake();
    }

    std::cout << "Stop Mode : Coast\n";
    return stopCoast();
}

static bool monitorStatus(unsigned long durationMs)
{
    const auto tStart = std::chrono::steady_clock::now();
    const auto deadline = tStart + std::chrono::milliseconds(durationMs);

    while (std::chrono::steady_clock::now() < deadline)
    {
        WooDrive::MotorStatus status;
        uint8_t fault = 0;

        bool ok1 = drive.getFault(TARGET_ID, fault);
        bool ok2 = drive.getMotorStatusAll(TARGET_ID, status);

        if (ok1 && ok2)
        {
            printStatusLine(status, fault);

            if (fault != 0x00)
            {
                printTitle("RUN FAIL");
                std::cout << "Fault detected during position control.\n";
                return false;
            }
        }
        else
        {
            std::cout << "Status read fail\n";
        }

        sleepMs(200);
    }

    return true;
}

static bool runPosition(float positionTarget, uint8_t direction)
{
    bool ok = drive.setMotorMotionAll(TARGET_ID,
                                      ACCEL_TIME_MS,
                                      DECEL_TIME_MS,
                                      MOTION_MODE,
                                      SUB_TARGET,
                                      positionTarget,
                                      direction);

    if (direction == DIR_ZERO) {
        printStep("setMotorMotionAll(zero)", ok);
    } else if (direction == DIR_POSITIVE) {
        printStep("setMotorMotionAll(+dir)", ok);
    } else {
        printStep("setMotorMotionAll(-dir)", ok);
    }

    if (!ok) return false;

    return monitorStatus(RUN_MONITOR_MS);
}

int main()
{
    if (!serial.isOpen()) {
        std::cout << "serial open failed\n";
        return 1;
    }

    drive.setTimeout(300);

    printTitle("WooDrive Example05 Position");

    uint8_t idRead = 0;
    bool ok = drive.getId(TARGET_ID, idRead);
    printStep("getId", ok);
    if (!ok) return 0;

    printU8("ID", idRead);

    if (!checkFaultZero())
    {
        printTitle("STOP");
        std::cout << "WooDrive fault detected before run.\n";
        return 0;
    }

    printTitle("SET RUN READY");
    if (!setRunReady())
    {
        printTitle("STOP");
        std::cout << "Run ready setup failed.\n";
        return 0;
    }

    printTitle("RUN 1 : ZERO");
    printF32("Position Target", MAIN_TARGET_POS, 2);
    printU8("Direction", DIR_ZERO);
    if (!runPosition(MAIN_TARGET_POS, DIR_ZERO))
    {
        stopMotor();
        return 0;
    }
    sleepMs(500);

    printTitle("RUN 2 : + POSITION");
    printF32("Position Target", MAIN_TARGET_POS, 2);
    printU8("Direction", DIR_POSITIVE);
    if (!runPosition(MAIN_TARGET_POS, DIR_POSITIVE))
    {
        stopMotor();
        return 0;
    }
    sleepMs(500);

    printTitle("RUN 3 : - POSITION");
    printF32("Position Target", MAIN_TARGET_POS, 2);
    printU8("Direction", DIR_NEGATIVE);
    if (!runPosition(MAIN_TARGET_POS, DIR_NEGATIVE))
    {
        stopMotor();
        return 0;
    }
    sleepMs(500);

    printTitle("RUN 4 : - POSITION");
    printF32("Position Target", MAIN_TARGET_POS, 2);
    printU8("Direction", DIR_NEGATIVE);
    if (!runPosition(MAIN_TARGET_POS, DIR_NEGATIVE))
    {
        stopMotor();
        return 0;
    }
    sleepMs(500);

    printTitle("RUN 5 : + POSITION");
    printF32("Position Target", MAIN_TARGET_POS, 2);
    printU8("Direction", DIR_POSITIVE);
    if (!runPosition(MAIN_TARGET_POS, DIR_POSITIVE))
    {
        stopMotor();
        return 0;
    }
    sleepMs(500);

    printTitle("RUN 6 : ZERO");
    printF32("Position Target", MAIN_TARGET_POS, 2);
    printU8("Direction", DIR_ZERO);
    if (!runPosition(MAIN_TARGET_POS, DIR_ZERO))
    {
        stopMotor();
        return 0;
    }

    if (!stopMotor())
    {
        return 0;
    }
    sleepMs(500);

    printTitle("DONE");
    return 0;
}