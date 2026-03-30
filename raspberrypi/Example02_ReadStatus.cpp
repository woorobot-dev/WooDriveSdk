#include "WooDriveSdk.h"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <cstdint>

// =========================================================
// SERIAL
// =========================================================
static const char* PORT_NAME = "/dev/ttyUSB0";
static const int WOODRIVE_BAUDRATE = 9600;

// =========================================================
// TARGET
// =========================================================
static const uint8_t TARGET_ID = 1;

// =========================================================
// GLOBAL
// =========================================================
static PosixSerialTransport serial(PORT_NAME, WOODRIVE_BAUDRATE);
static StdClock wooClock;
static WooDrive drive(serial, wooClock);

// =========================================================
// PRINT HELPERS
// =========================================================
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

static void printU8(const char* name, uint8_t v)
{
    std::cout << name << " : " << static_cast<unsigned long>(v) << "\n";
}

static void printU16(const char* name, uint16_t v)
{
    std::cout << name << " : " << static_cast<unsigned long>(v) << "\n";
}

static void printU32(const char* name, uint32_t v)
{
    std::cout << name << " : " << static_cast<unsigned long>(v) << "\n";
}

static void printF32(const char* name, float v, uint8_t digits = 2)
{
    std::cout << name << " : "
              << std::fixed << std::setprecision(digits) << v << "\n";
}

static void printI64Raw(int64_t v)
{
    std::cout << v;
}

static void printInt64(const char* name, int64_t v)
{
    std::cout << name << " : ";
    printI64Raw(v);
    std::cout << "\n";
}

static void printStep(const char* name, bool ok)
{
    std::cout << name << " : " << (ok ? "OK" : "FAIL") << "\n";
}

// =========================================================
// PRINT CONFIG BLOCKS
// =========================================================
static void printBoardConfig(const WooDrive::BoardConfig& s)
{
    printU8("Reset", s.reset);
    printU8("ID", s.id);
    printU8("Fault", s.fault);
    printU8("Communication Mode", s.communicationMode);
    printU32("BPS", s.bps);
    printU16("Watchdog", s.watchdog);
}

static void printMotorConfig(const WooDrive::MotorConfig& s)
{
    printU8("Motor Type", s.motorType);
    printU8("Feedback Type", s.feedbackType);
    printU8("Startup Feedback Type", s.startupFeedbackType);
    printU8("Direction Invert", s.directionInvert);
    printU8("Field Weakening Enable", s.fieldWeakeningEnable);
    printU8("External Brake Present", s.externalBrakePresent);
    printU8("Pole Pairs", s.polePairs);
    printU8("Feedback Dir", s.feedbackDir);
    printU32("Feedback Resolution", s.feedbackResolution);
    printF32("Gear", s.gear, 6);
}

static void printMotorFocSetting(const WooDrive::MotorFocSetting& s)
{
    printU8("Auto Motor Setup", s.autoMotorSetup);
    printU8("Alignment Mode", s.alignmentMode);
    printU32("Phase Offset", s.phaseOffset);
    printU8("Force Angle Level", s.forceAngleLevel);
    printU8("Hall Sensor Setting", s.hallSensorSetting);
    printU8("Main Port Level Now", s.mainPortLevelNow);
    printU8("Sub Port Level Now", s.subPortLevelNow);
}

static void printMotorParam(const WooDrive::MotorParam& s)
{
    printU8("Auto Parameter", s.autoParameter);
    printF32("Rated Speed", s.ratedSpeed, 2);
    printF32("Rated Current", s.ratedCurrent, 2);
    printF32("Rated Voltage", s.ratedVoltage, 2);
    printF32("Resistance", s.resistance, 3);
    printF32("Inductance", s.inductance, 6);
    printF32("Torque Constant", s.torqueConstant, 3);
    printF32("Moment Of Inertia", s.momentOfInertia, 3);
}

static void printMotorGain(const WooDrive::MotorGain& s)
{
    printU8("Gain Mode", s.gainMode);
    printU8("Position Gain Scale", s.positionGainScale);
    printU8("Velocity Gain Scale", s.velocityGainScale);
    printU8("Current Gain Scale", s.currentGainScale);
    printU32("Position P Gain", s.positionPGain);
    printU32("Velocity P Gain", s.velocityPGain);
    printU32("Velocity I Gain", s.velocityIGain);
    printU32("Current P Gain", s.currentPGain);
    printU32("Current I Gain", s.currentIGain);
}

static void printMotorLimit(const WooDrive::MotorLimit& s)
{
    printF32("Position CCW Max", s.positionCcwMax, 2);
    printF32("Position CW Max", s.positionCwMax, 2);
    printF32("Velocity CCW Max", s.velocityCcwMax, 2);
    printF32("Velocity CW Max", s.velocityCwMax, 2);
    printF32("Iq Current CCW Max", s.iqCurrentCcwMax, 2);
    printF32("Iq Current CW Max", s.iqCurrentCwMax, 2);
    printF32("Id Current Max", s.idCurrentMax, 2);
    printF32("Iq Current Limit", s.iqCurrentLimit, 2);
    printF32("Bus Voltage Max Limit", s.busVoltageMaxLimit, 2);
    printF32("Bus Voltage Min Limit", s.busVoltageMinLimit, 2);
    printF32("Temperature Max Limit", s.temperatureMaxLimit, 2);
}

static void printMotorControl(const WooDrive::MotorControl& s)
{
    printU16("Accel Time", s.accelTime);
    printU16("Decel Time", s.decelTime);
    printU8("Motion Mode", s.motionMode);
    printF32("Sub Target", s.subTarget, 2);
    printF32("Main Target", s.mainTarget, 2);
    printU8("Direction", s.direction);
    printU8("Run Mode", s.runMode);
    printU8("Motor Enable", s.motorEnable);
    printU8("Motor Brake", s.motorBrake);
    printU8("External Brake", s.externalBrake);
}

static void printStatusBlock(const WooDrive::MotorStatus& s)
{
    printU32("Total Time", s.totalTime);
    printU32("Elapsed Time", s.elapsedTime);
    printU32("Remain Time", s.remainTime);
    printF32("Position", s.position, 2);
    printF32("Velocity", s.velocity, 2);
    printF32("Iq Current", s.iqCurrent, 3);
    printF32("Id Current", s.idCurrent, 3);
    printF32("Bus Current", s.busCurrent, 3);
    printF32("Vq Voltage", s.vqVoltage, 3);
    printF32("Vd Voltage", s.vdVoltage, 3);
    printF32("Bus Voltage", s.busVoltage, 2);
    printF32("Temperature", s.temperature, 2);
    printInt64("Pulse Count", s.pulseCount);
    printF32("U Phase Current", s.uPhaseCurrent, 3);
    printF32("W Phase Current", s.wPhaseCurrent, 3);
}

// =========================================================
// READ + PRINT HELPERS
// =========================================================
static void readAndPrintBoard()
{
    WooDrive::BoardConfig s;
    bool ok = drive.getBoardConfigAll(TARGET_ID, s);

    if (!ok) {
        printTitle("BOARD CONFIG (FAIL)");
        return;
    }

    printTitle("BOARD CONFIG (OK)");
    printBoardConfig(s);
}

static void readAndPrintMotorConfig()
{
    WooDrive::MotorConfig s;
    bool ok = drive.getMotorConfigAll(TARGET_ID, s);

    if (!ok) {
        printTitle("MOTOR CONFIG (FAIL)");
        return;
    }

    printTitle("MOTOR CONFIG (OK)");
    printMotorConfig(s);
}

static void readAndPrintMotorFoc()
{
    WooDrive::MotorFocSetting s;
    bool ok = drive.getMotorFocSettingAll(TARGET_ID, s);

    if (!ok) {
        printTitle("MOTOR FOC SETTING (FAIL)");
        return;
    }

    printTitle("MOTOR FOC SETTING (OK)");
    printMotorFocSetting(s);
}

static void readAndPrintMotorParam()
{
    WooDrive::MotorParam s;
    bool ok = drive.getMotorParamAll(TARGET_ID, s);

    if (!ok) {
        printTitle("MOTOR PARAM (FAIL)");
        return;
    }

    printTitle("MOTOR PARAM (OK)");
    printMotorParam(s);
}

static void readAndPrintMotorGain()
{
    WooDrive::MotorGain s;
    bool ok = drive.getMotorGainAll(TARGET_ID, s);

    if (!ok) {
        printTitle("MOTOR GAIN (FAIL)");
        return;
    }

    printTitle("MOTOR GAIN (OK)");
    printMotorGain(s);
}

static void readAndPrintMotorLimit()
{
    WooDrive::MotorLimit s;
    bool ok = drive.getMotorLimitAll(TARGET_ID, s);

    if (!ok) {
        printTitle("MOTOR LIMIT (FAIL)");
        return;
    }

    printTitle("MOTOR LIMIT (OK)");
    printMotorLimit(s);
}

static void readAndPrintMotorControl()
{
    WooDrive::MotorControl s;
    bool ok = drive.getMotorControlAll(TARGET_ID, s);

    if (!ok) {
        printTitle("MOTOR CONTROL (FAIL)");
        return;
    }

    printTitle("MOTOR CONTROL (OK)");
    printMotorControl(s);
}

static void readAndPrintMotorStatus()
{
    WooDrive::MotorStatus s;
    bool ok = drive.getMotorStatusAll(TARGET_ID, s);

    if (!ok) {
        printTitle("MOTOR STATUS (FAIL)");
        return;
    }

    printTitle("MOTOR STATUS (OK)");
    printStatusBlock(s);
}

int main()
{
    if (!serial.isOpen()) {
        std::cout << "serial open failed\n";
        return 1;
    }

    drive.setTimeout(300);

    printTitle("WooDrive Example02 Read Status");

    uint8_t idRead = 0;
    uint8_t fault = 0;

    bool ok = drive.getId(TARGET_ID, idRead);
    printStep("getId", ok);
    if (!ok) return 1;

    ok = drive.getFault(TARGET_ID, fault);
    printStep("getFault", ok);
    if (!ok) return 1;

    printU8("ID", idRead);
    printU8("FAULT", fault);

    printTitle("READ RESULT");
    readAndPrintBoard();
    readAndPrintMotorConfig();
    readAndPrintMotorFoc();
    readAndPrintMotorParam();
    readAndPrintMotorGain();
    readAndPrintMotorLimit();
    readAndPrintMotorControl();
    readAndPrintMotorStatus();

    printTitle("START MONITORING");

    // Arduino 원본과 동일하게 모니터링은 기본 비활성화
    /*
    while (1)
    {
        WooDrive::MotorStatus status;
        uint8_t faultNow = 0;

        bool ok1 = drive.getFault(TARGET_ID, faultNow);
        bool ok2 = drive.getMotorStatusAll(TARGET_ID, status);

        if (ok1 && ok2) {
            printStatusLine(status, faultNow);
        } else {
            std::cout << "READ FAIL\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    */

    return 0;
}