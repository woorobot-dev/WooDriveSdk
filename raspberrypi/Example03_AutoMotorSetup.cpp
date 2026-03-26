#include "WooDriveSdk.h"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <cstdint>

static const char* PORT_NAME = "/dev/ttyUSB0";
static const uint32_t WOODRIVE_BAUDRATE = 9600;
static const uint8_t TARGET_ID = 1;

// Auto setup command
static const uint8_t AUTO_SETUP_CMD = 0;

// Auto setup 최대 대기 시간: 4분
static const unsigned long AUTO_SETUP_TIMEOUT_MS = 240000UL;

// 재시도 간격: 5초
static const unsigned long RETRY_INTERVAL_MS = 5000UL;

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
    std::cout << name << " : " << std::fixed << std::setprecision(digits) << v << "\n";
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

static void printFaultHex(const char* name, uint8_t v)
{
    std::cout << name << " : 0x"
              << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
              << static_cast<unsigned int>(v)
              << std::dec << std::nouppercase << std::setfill(' ') << "\n";
}

static void printBoardConfig(const WooDrive::BoardConfig& s)
{
    printTitle("BOARD CONFIG");
    printU8("Reset", s.reset);
    printU8("ID", s.id);
    printU8("Fault", s.fault);
    printU8("Communication Mode", s.communicationMode);
    printU32("BPS", s.bps);
    printU16("Watchdog", s.watchdog);
}

static void printMotorConfig(const WooDrive::MotorConfig& s)
{
    printTitle("MOTOR CONFIG");
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
    printTitle("MOTOR FOC SETTING");
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
    printTitle("MOTOR PARAM");
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
    printTitle("MOTOR GAIN");
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
    printTitle("MOTOR LIMIT");
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
    printTitle("MOTOR CONTROL");
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

static void printMotorStatus(const WooDrive::MotorStatus& s)
{
    printTitle("MOTOR STATUS");
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

static bool waitAutoSetupComplete(uint8_t targetId, uint8_t& autoSetupState)
{
    auto tStart = std::chrono::steady_clock::now();

    printTitle("WAIT SETUP");
    std::cout << "Motor setup is running...\n";

    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - tStart).count() < AUTO_SETUP_TIMEOUT_MS)
    {
        if (drive.getAutoMotorSetup(targetId, autoSetupState))
        {
            std::cout << "Controller response restored.\n";
            return true;
        }

        std::cout << "Setup running, controller busy... wait 5 sec\n";
        sleepMs(RETRY_INTERVAL_MS);
    }

    return false;
}

static void readAndPrintBoard()
{
    WooDrive::BoardConfig s;
    bool ok = drive.getBoardConfigAll(TARGET_ID, s);
    printStep("BoardConfigAll", ok);
    if (ok) printBoardConfig(s);
}

static void readAndPrintMotorConfig()
{
    WooDrive::MotorConfig s;
    bool ok = drive.getMotorConfigAll(TARGET_ID, s);
    printStep("MotorConfigAll", ok);
    if (ok) printMotorConfig(s);
}

static void readAndPrintMotorFoc()
{
    WooDrive::MotorFocSetting s;
    bool ok = drive.getMotorFocSettingAll(TARGET_ID, s);
    printStep("MotorFocSettingAll", ok);
    if (ok) printMotorFocSetting(s);
}

static void readAndPrintMotorParam()
{
    WooDrive::MotorParam s;
    bool ok = drive.getMotorParamAll(TARGET_ID, s);
    printStep("MotorParamAll", ok);
    if (ok) printMotorParam(s);
}

static void readAndPrintMotorGain()
{
    WooDrive::MotorGain s;
    bool ok = drive.getMotorGainAll(TARGET_ID, s);
    printStep("MotorGainAll", ok);
    if (ok) printMotorGain(s);
}

static void readAndPrintMotorLimit()
{
    WooDrive::MotorLimit s;
    bool ok = drive.getMotorLimitAll(TARGET_ID, s);
    printStep("MotorLimitAll", ok);
    if (ok) printMotorLimit(s);
}

static void readAndPrintMotorControl()
{
    WooDrive::MotorControl s;
    bool ok = drive.getMotorControlAll(TARGET_ID, s);
    printStep("MotorControlAll", ok);
    if (ok) printMotorControl(s);
}

static void readAndPrintMotorStatus()
{
    WooDrive::MotorStatus s;
    bool ok = drive.getMotorStatusAll(TARGET_ID, s);
    printStep("MotorStatusAll", ok);
    if (ok) printMotorStatus(s);
}

int main()
{
    if (!serial.isOpen()) {
        std::cout << "serial open failed\n";
        return 1;
    }

    drive.setTimeout(300);

    printTitle("WooDrive Example03 Auto Motor Setup");

    uint8_t idRead = 0;
    uint8_t faultBefore = 0;
    uint8_t faultAfter = 0;
    uint8_t autoSetupState = 0xFF;

    bool ok = drive.getId(TARGET_ID, idRead);
    printStep("getId", ok);
    if (!ok) return 0;

    ok = drive.getFault(TARGET_ID, faultBefore);
    printStep("getFault(before)", ok);
    if (!ok) return 0;

    ok = drive.setPolePairs(TARGET_ID, 4);
    (void)ok;

    printU8("ID", idRead);
    printFaultHex("Fault Before", faultBefore);

    printTitle("START AUTO MOTOR SETUP");

    ok = drive.setAutoMotorSetup(TARGET_ID, AUTO_SETUP_CMD);
    printStep("setAutoMotorSetup", ok);
    if (!ok) return 0;

    ok = waitAutoSetupComplete(TARGET_ID, autoSetupState);
    printStep("waitAutoSetupComplete", ok);
    if (!ok) {
        printTitle("AUTO SETUP TIMEOUT");
        std::cout << "Controller did not respond within 240 sec.\n";
        return 0;
    }

    printU8("Auto Setup State", autoSetupState);

    ok = drive.getFault(TARGET_ID, faultAfter);
    printStep("getFault(after)", ok);
    if (ok) {
        printFaultHex("Fault After", faultAfter);
    }

    printTitle("READ RESULT");
    readAndPrintBoard();
    readAndPrintMotorConfig();
    readAndPrintMotorFoc();
    readAndPrintMotorParam();
    readAndPrintMotorGain();
    readAndPrintMotorLimit();
    readAndPrintMotorControl();
    readAndPrintMotorStatus();

    printTitle("AUTO MOTOR SETUP DONE");
    return 0;
}