#include <Arduino.h>
#include <SoftwareSerial.h>
#include "WooDriveSdk.h"

// =========================================================
// SERIAL
// =========================================================
#define RX_PIN 2
#define TX_PIN 3

static const uint32_t WOODRIVE_BAUDRATE = 9600;
SoftwareSerial WOO_SERIAL(RX_PIN, TX_PIN);
ArduinoTransport wooTransport(WOO_SERIAL, WOODRIVE_BAUDRATE);
ArduinoClock wooClock;
WooDrive drive(wooTransport, wooClock);

// =========================================================
// TARGET
// =========================================================
static const uint8_t TARGET_ID = 1;

// =========================================================
// PRINT HELPERS
// =========================================================
static void printLine()
{
    Serial.println(F("=================================================="));
}

static void printTitle(const __FlashStringHelper* title)
{
    Serial.println();
    printLine();
    Serial.println(title);
    printLine();
}

static void printU8(const __FlashStringHelper* name, uint8_t v)
{
    Serial.print(name);
    Serial.print(F(" : "));
    Serial.println((unsigned long)v);
}

static void printU16(const __FlashStringHelper* name, uint16_t v)
{
    Serial.print(name);
    Serial.print(F(" : "));
    Serial.println((unsigned long)v);
}

static void printU32(const __FlashStringHelper* name, uint32_t v)
{
    Serial.print(name);
    Serial.print(F(" : "));
    Serial.println((unsigned long)v);
}

static void printF32(const __FlashStringHelper* name, float v, uint8_t digits = 2)
{
    Serial.print(name);
    Serial.print(F(" : "));
    Serial.println(v, digits);
}

static void printI64Raw(int64_t v)
{
    if (v == 0) {
        Serial.print('0');
        return;
    }

    if (v < 0) {
        Serial.print('-');
        v = -v;
    }

    char buf[21];
    int i = 0;

    while (v > 0 && i < (int)(sizeof(buf) - 1)) {
        buf[i++] = char('0' + (v % 10));
        v /= 10;
    }

    while (i > 0) {
        Serial.print(buf[--i]);
    }
}

static void printInt64(const __FlashStringHelper* name, int64_t v)
{
    Serial.print(name);
    Serial.print(F(" : "));
    printI64Raw(v);
    Serial.println();
}

static void printStep(const __FlashStringHelper* name, bool ok)
{
    Serial.print(name);
    Serial.print(F(" : "));
    Serial.println(ok ? F("OK") : F("FAIL"));
}

// =========================================================
// PRINT CONFIG BLOCKS
// =========================================================
static void printBoardConfig(const WooDrive::BoardConfig& s)
{
    // printTitle(F("BOARD CONFIG"));
    printU8(F("Reset"), s.reset);
    printU8(F("ID"), s.id);
    printU8(F("Fault"), s.fault);
    printU8(F("Communication Mode"), s.communicationMode);
    printU32(F("BPS"), s.bps);
    printU16(F("Watchdog"), s.watchdog);
}

static void printMotorConfig(const WooDrive::MotorConfig& s)
{
    // printTitle(F("MOTOR CONFIG"));
    printU8(F("Motor Type"), s.motorType);
    printU8(F("Feedback Type"), s.feedbackType);
    printU8(F("Startup Feedback Type"), s.startupFeedbackType);
    printU8(F("Direction Invert"), s.directionInvert);
    printU8(F("Field Weakening Enable"), s.fieldWeakeningEnable);
    printU8(F("External Brake Present"), s.externalBrakePresent);
    printU8(F("Pole Pairs"), s.polePairs);
    printU8(F("Feedback Dir"), s.feedbackDir);
    printU32(F("Feedback Resolution"), s.feedbackResolution);
    printF32(F("Gear"), s.gear, 6);
}

static void printMotorFocSetting(const WooDrive::MotorFocSetting& s)
{
    // printTitle(F("MOTOR FOC SETTING"));
    printU8(F("Auto Motor Setup"), s.autoMotorSetup);
    printU8(F("Alignment Mode"), s.alignmentMode);
    printU32(F("Phase Offset"), s.phaseOffset);
    printU8(F("Force Angle Level"), s.forceAngleLevel);
    printU8(F("Hall Sensor Setting"), s.hallSensorSetting);
    printU8(F("Main Port Level Now"), s.mainPortLevelNow);
    printU8(F("Sub Port Level Now"), s.subPortLevelNow);
}

static void printMotorParam(const WooDrive::MotorParam& s)
{
    // printTitle(F("MOTOR PARAM"));
    printU8(F("Auto Parameter"), s.autoParameter);
    printF32(F("Rated Speed"), s.ratedSpeed, 2);
    printF32(F("Rated Current"), s.ratedCurrent, 2);
    printF32(F("Rated Voltage"), s.ratedVoltage, 2);
    printF32(F("Resistance"), s.resistance, 3);
    printF32(F("Inductance"), s.inductance, 6);
    printF32(F("Torque Constant"), s.torqueConstant, 3);
    printF32(F("Moment Of Inertia"), s.momentOfInertia, 3);
}

static void printMotorGain(const WooDrive::MotorGain& s)
{
    // printTitle(F("MOTOR GAIN"));
    printU8(F("Gain Mode"), s.gainMode);
    printU8(F("Position Gain Scale"), s.positionGainScale);
    printU8(F("Velocity Gain Scale"), s.velocityGainScale);
    printU8(F("Current Gain Scale"), s.currentGainScale);
    printU32(F("Position P Gain"), s.positionPGain);
    printU32(F("Velocity P Gain"), s.velocityPGain);
    printU32(F("Velocity I Gain"), s.velocityIGain);
    printU32(F("Current P Gain"), s.currentPGain);
    printU32(F("Current I Gain"), s.currentIGain);
}

static void printMotorLimit(const WooDrive::MotorLimit& s)
{
    // printTitle(F("MOTOR LIMIT"));
    printF32(F("Position CCW Max"), s.positionCcwMax, 2);
    printF32(F("Position CW Max"), s.positionCwMax, 2);
    printF32(F("Velocity CCW Max"), s.velocityCcwMax, 2);
    printF32(F("Velocity CW Max"), s.velocityCwMax, 2);
    printF32(F("Iq Current CCW Max"), s.iqCurrentCcwMax, 2);
    printF32(F("Iq Current CW Max"), s.iqCurrentCwMax, 2);
    printF32(F("Id Current Max"), s.idCurrentMax, 2);
    printF32(F("Iq Current Limit"), s.iqCurrentLimit, 2);
    printF32(F("Bus Voltage Max Limit"), s.busVoltageMaxLimit, 2);
    printF32(F("Bus Voltage Min Limit"), s.busVoltageMinLimit, 2);
    printF32(F("Temperature Max Limit"), s.temperatureMaxLimit, 2);
}

static void printMotorControl(const WooDrive::MotorControl& s)
{
    // printTitle(F("MOTOR CONTROL"));
    printU16(F("Accel Time"), s.accelTime);
    printU16(F("Decel Time"), s.decelTime);
    printU8(F("Motion Mode"), s.motionMode);
    printF32(F("Sub Target"), s.subTarget, 2);
    printF32(F("Main Target"), s.mainTarget, 2);
    printU8(F("Direction"), s.direction);
    printU8(F("Run Mode"), s.runMode);
    printU8(F("Motor Enable"), s.motorEnable);
    printU8(F("Motor Brake"), s.motorBrake);
    printU8(F("External Brake"), s.externalBrake);
}

static void printStatusBlock(const WooDrive::MotorStatus& s)
{
    // printTitle(F("MOTOR STATUS"));
    printU32(F("Total Time"), s.totalTime);
    printU32(F("Elapsed Time"), s.elapsedTime);
    printU32(F("Remain Time"), s.remainTime);
    printF32(F("Position"), s.position, 2);
    printF32(F("Velocity"), s.velocity, 2);
    printF32(F("Iq Current"), s.iqCurrent, 3);
    printF32(F("Id Current"), s.idCurrent, 3);
    printF32(F("Bus Current"), s.busCurrent, 3);
    printF32(F("Vq Voltage"), s.vqVoltage, 3);
    printF32(F("Vd Voltage"), s.vdVoltage, 3);
    printF32(F("Bus Voltage"), s.busVoltage, 2);
    printF32(F("Temperature"), s.temperature, 2);
    printInt64(F("Pulse Count"), s.pulseCount);
    printF32(F("U Phase Current"), s.uPhaseCurrent, 3);
    printF32(F("W Phase Current"), s.wPhaseCurrent, 3);
}

static void printStatusLine(const WooDrive::MotorStatus& s, uint8_t fault)
{
    Serial.print(F("F:"));
    Serial.print(fault);

    Serial.print(F(" | P:"));
    Serial.print(s.position, 2);

    Serial.print(F(" | V:"));
    Serial.print(s.velocity, 2);

    Serial.print(F(" | Iq:"));
    Serial.print(s.iqCurrent, 3);

    Serial.print(F(" | Id:"));
    Serial.print(s.idCurrent, 3);

    Serial.print(F(" | BusV:"));
    Serial.print(s.busVoltage, 2);

    Serial.print(F(" | Temp:"));
    Serial.print(s.temperature, 2);

    Serial.print(F(" | Pulse:"));
    printI64Raw(s.pulseCount);
    Serial.println();
}

// =========================================================
// READ + PRINT HELPERS
// 구조체를 한 번에 하나만 잡아서 stack 사용량 감소
// =========================================================
static void readAndPrintBoard()
{
    WooDrive::BoardConfig s;
    bool ok = drive.getBoardConfigAll(TARGET_ID, s);

    if (!ok) {
        printTitle(F("BOARD CONFIG (FAIL)"));
        return;
    }

    printTitle(F("BOARD CONFIG (OK)"));
    printBoardConfig(s);
}

static void readAndPrintMotorConfig()
{
    WooDrive::MotorConfig s;
    bool ok = drive.getMotorConfigAll(TARGET_ID, s);

    if (!ok) {
        printTitle(F("MOTOR CONFIG (FAIL)"));
        return;
    }

    printTitle(F("MOTOR CONFIG (OK)"));
    printMotorConfig(s);
}

static void readAndPrintMotorFoc()
{
    WooDrive::MotorFocSetting s;
    bool ok = drive.getMotorFocSettingAll(TARGET_ID, s);

    if (!ok) {
        printTitle(F("MOTOR FOC SETTING (FAIL)"));
        return;
    }

    printTitle(F("MOTOR FOC SETTING (OK)"));
    printMotorFocSetting(s);
}

static void readAndPrintMotorParam()
{
    WooDrive::MotorParam s;
    bool ok = drive.getMotorParamAll(TARGET_ID, s);

    if (!ok) {
        printTitle(F("MOTOR PARAM (FAIL)"));
        return;
    }

    printTitle(F("MOTOR PARAM (OK)"));
    printMotorParam(s);
}

static void readAndPrintMotorGain()
{
    WooDrive::MotorGain s;
    bool ok = drive.getMotorGainAll(TARGET_ID, s);

    if (!ok) {
        printTitle(F("MOTOR GAIN (FAIL)"));
        return;
    }

    printTitle(F("MOTOR GAIN (OK)"));
    printMotorGain(s);
}

static void readAndPrintMotorLimit()
{
    WooDrive::MotorLimit s;
    bool ok = drive.getMotorLimitAll(TARGET_ID, s);

    if (!ok) {
        printTitle(F("MOTOR LIMIT (FAIL)"));
        return;
    }

    printTitle(F("MOTOR LIMIT (OK)"));
    printMotorLimit(s);
}

static void readAndPrintMotorControl()
{
    WooDrive::MotorControl s;
    bool ok = drive.getMotorControlAll(TARGET_ID, s);

    if (!ok) {
        printTitle(F("MOTOR CONTROL (FAIL)"));
        return;
    }

    printTitle(F("MOTOR CONTROL (OK)"));
    printMotorControl(s);
}

static void readAndPrintMotorStatus()
{
    WooDrive::MotorStatus s;
    bool ok = drive.getMotorStatusAll(TARGET_ID, s);

    if (!ok) {
        printTitle(F("MOTOR STATUS (FAIL)"));
        return;
    }

    printTitle(F("MOTOR STATUS (OK)"));
    printStatusBlock(s);
}

// =========================================================
// SETUP
// =========================================================
void setup()
{
    Serial.begin(9600);
    WOO_SERIAL.begin(WOODRIVE_BAUDRATE);
    delay(3000);

    drive.setTimeout(300);

    printTitle(F("WooDrive Example02 Read Status"));

    uint8_t idRead = 0;
    uint8_t fault = 0;

    bool ok = drive.getId(TARGET_ID, idRead);
    printStep(F("getId"), ok);
    if (!ok) return;

    ok = drive.getFault(TARGET_ID, fault);
    printStep(F("getFault"), ok);
    if (!ok) return;

    printU8(F("ID"), idRead);
    printU8(F("FAULT"), fault);

    printTitle(F("READ RESULT"));
    readAndPrintBoard();
    readAndPrintMotorConfig();
    readAndPrintMotorFoc();
    readAndPrintMotorParam();
    readAndPrintMotorGain();
    readAndPrintMotorLimit();
    readAndPrintMotorControl();
    readAndPrintMotorStatus();

    printTitle(F("START MONITORING"));
}

// =========================================================
// LOOP
// =========================================================
void loop()
{
    // 꼭 필요할 때만 켜는 것을 권장
    /*
    WooDrive::MotorStatus status;
    uint8_t fault = 0;

    bool ok1 = drive.getFault(TARGET_ID, fault);
    bool ok2 = drive.getMotorStatusAll(TARGET_ID, status);

    if (ok1 && ok2) {
        printStatusLine(status, fault);
    } else {
        Serial.println(F("READ FAIL"));
    }

    delay(200);
    */
}