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
// TARGET / OPTION
// =========================================================
static const uint8_t TARGET_ID = 1;

// Auto setup command
static const uint8_t AUTO_SETUP_CMD = 0;

// Auto setup 최대 대기 시간: 4분
static const unsigned long AUTO_SETUP_TIMEOUT_MS = 240000UL;

// 재시도 간격: 5초
static const unsigned long RETRY_INTERVAL_MS = 5000UL;

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

static void printStep(const __FlashStringHelper* name, bool ok)
{
    Serial.print(name);
    Serial.print(F(" : "));
    Serial.println(ok ? F("OK") : F("FAIL"));
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

static void printFaultHex(const __FlashStringHelper* name, uint8_t v)
{
    Serial.print(name);
    Serial.print(F(" : 0x"));
    if (v < 0x10) Serial.print('0');
    Serial.println(v, HEX);
}

// =========================================================
// PRINT BLOCKS
// =========================================================
static void printBoardConfig(const WooDrive::BoardConfig& s)
{
    printTitle(F("BOARD CONFIG"));
    printU8(F("Reset"), s.reset);
    printU8(F("ID"), s.id);
    printU8(F("Fault"), s.fault);
    printU8(F("Communication Mode"), s.communicationMode);
    printU32(F("BPS"), s.bps);
    printU16(F("Watchdog"), s.watchdog);
}

static void printMotorConfig(const WooDrive::MotorConfig& s)
{
    printTitle(F("MOTOR CONFIG"));
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
    printTitle(F("MOTOR FOC SETTING"));
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
    printTitle(F("MOTOR PARAM"));
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
    printTitle(F("MOTOR GAIN"));
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
    printTitle(F("MOTOR LIMIT"));
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
    printTitle(F("MOTOR CONTROL"));
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

static void printMotorStatus(const WooDrive::MotorStatus& s)
{
    printTitle(F("MOTOR STATUS"));
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

// =========================================================
// AUTO SETUP WAIT
// Auto setup 중 UART1 응답이 끊길 수 있으므로
// 응답 복귀를 완료 조건으로 판단
// =========================================================
static bool waitAutoSetupComplete(uint8_t targetId, uint8_t& autoSetupState)
{
    uint32_t tStart = millis();

    printTitle(F("WAIT SETUP"));
    Serial.println(F("Motor setup is running..."));

    while (millis() - tStart < AUTO_SETUP_TIMEOUT_MS)
    {
        if (drive.getAutoMotorSetup(targetId, autoSetupState))
        {
            Serial.println(F("Controller response restored."));
            return true;
        }

        Serial.println(F("Setup running, controller busy... wait 5 sec"));
        delay(RETRY_INTERVAL_MS);
    }

    return false;
}

// =========================================================
// READ + PRINT HELPERS
// 구조체를 한 번에 하나만 잡아서 stack 사용량 감소
// =========================================================
static void readAndPrintBoard()
{
    WooDrive::BoardConfig s;
    bool ok = drive.getBoardConfigAll(TARGET_ID, s);
    printStep(F("BoardConfigAll"), ok);
    if (ok) printBoardConfig(s);
}

static void readAndPrintMotorConfig()
{
    WooDrive::MotorConfig s;
    bool ok = drive.getMotorConfigAll(TARGET_ID, s);
    printStep(F("MotorConfigAll"), ok);
    if (ok) printMotorConfig(s);
}

static void readAndPrintMotorFoc()
{
    WooDrive::MotorFocSetting s;
    bool ok = drive.getMotorFocSettingAll(TARGET_ID, s);
    printStep(F("MotorFocSettingAll"), ok);
    if (ok) printMotorFocSetting(s);
}

static void readAndPrintMotorParam()
{
    WooDrive::MotorParam s;
    bool ok = drive.getMotorParamAll(TARGET_ID, s);
    printStep(F("MotorParamAll"), ok);
    if (ok) printMotorParam(s);
}

static void readAndPrintMotorGain()
{
    WooDrive::MotorGain s;
    bool ok = drive.getMotorGainAll(TARGET_ID, s);
    printStep(F("MotorGainAll"), ok);
    if (ok) printMotorGain(s);
}

static void readAndPrintMotorLimit()
{
    WooDrive::MotorLimit s;
    bool ok = drive.getMotorLimitAll(TARGET_ID, s);
    printStep(F("MotorLimitAll"), ok);
    if (ok) printMotorLimit(s);
}

static void readAndPrintMotorControl()
{
    WooDrive::MotorControl s;
    bool ok = drive.getMotorControlAll(TARGET_ID, s);
    printStep(F("MotorControlAll"), ok);
    if (ok) printMotorControl(s);
}

static void readAndPrintMotorStatus()
{
    WooDrive::MotorStatus s;
    bool ok = drive.getMotorStatusAll(TARGET_ID, s);
    printStep(F("MotorStatusAll"), ok);
    if (ok) printMotorStatus(s);
}

// =========================================================
// SETUP
// =========================================================
void setup()
{
    Serial.begin(9600);
    WOO_SERIAL.begin(WOODRIVE_BAUDRATE);
    delay(5000);

    drive.setTimeout(300);

    printTitle(F("WooDrive Example03 Auto Motor Setup"));

    uint8_t idRead = 0;
    uint8_t faultBefore = 0;
    uint8_t faultAfter = 0;
    uint8_t autoSetupState = 0xFF;

    bool ok = drive.getId(TARGET_ID, idRead);
    printStep(F("getId"), ok);
    if (!ok) return;

    ok = drive.getFault(TARGET_ID, faultBefore);
    printStep(F("getFault(before)"), ok);
    if (!ok) return;


    // 홀 센서만 연결시 폴 페어 무조건 입력(모터 데이터시트 참고)
    ok = drive.setPolePairs(TARGET_ID, 4); //예시) 4폴페어(8극 수)

    printU8(F("ID"), idRead);
    printFaultHex(F("Fault Before"), faultBefore);

    printTitle(F("START AUTO MOTOR SETUP"));

    ok = drive.setAutoMotorSetup(TARGET_ID, AUTO_SETUP_CMD);
    printStep(F("setAutoMotorSetup"), ok);
    if (!ok) return;

    ok = waitAutoSetupComplete(TARGET_ID, autoSetupState);
    printStep(F("waitAutoSetupComplete"), ok);
    if (!ok) {
        printTitle(F("AUTO SETUP TIMEOUT"));
        Serial.println(F("Controller did not respond within 240 sec."));
        return;
    }

    printU8(F("Auto Setup State"), autoSetupState);

    ok = drive.getFault(TARGET_ID, faultAfter);
    printStep(F("getFault(after)"), ok);
    if (ok) {
        printFaultHex(F("Fault After"), faultAfter);
    }

    printTitle(F("READ RESULT"));
    readAndPrintBoard();
    readAndPrintMotorConfig();
    readAndPrintMotorFoc();
    readAndPrintMotorParam();
    readAndPrintMotorGain();
    readAndPrintMotorLimit();
    readAndPrintMotorControl();
    readAndPrintMotorStatus();

    printTitle(F("AUTO MOTOR SETUP DONE"));
}

// =========================================================
// LOOP
// =========================================================
void loop()
{
    // 필요하면 이후 상태 모니터링 추가
}