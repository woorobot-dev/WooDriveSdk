#include <Arduino.h>
#include <SoftwareSerial.h>
#include "WooDriveSdk.h"

// =========================================================
// SERIAL
// =========================================================
#define RX_PIN 2
#define TX_PIN 3

static const uint32_t WOODRIVE_BAUDRATE = 9600;   // 제어기 통신속도, 사용자가 바꿀 수 있음
SoftwareSerial WOO_SERIAL(RX_PIN, TX_PIN);
ArduinoTransport wooTransport(WOO_SERIAL, WOODRIVE_BAUDRATE);
ArduinoClock wooClock;
WooDrive drive(wooTransport, wooClock);

// =========================================================
// TARGET
// =========================================================
static const uint8_t TARGET_ID = 1;

// =========================================================
// RUN OPTION
// =========================================================
// 이 예제는 "속도제어" 기준 예제
// mainTarget : 목표 속도값 (항상 양수)
// subTarget  : 등가속 구간에서의 최대 전류 값 (내부적으로 전류/토크 = 가속도에 비례)
//              값이 클수록 가속이 빠르고, 작을수록 부드럽게 가속됨
// direction  : 0 = 원점, 1 = 정방향, 2 = 역방향

static const uint16_t ACCEL_TIME_MS = 1000; // 가속 시간
static const uint16_t DECEL_TIME_MS = 1000; // 감속 시간

// 제어기 속도제어 motion mode 값으로 수정
static const uint8_t MOTION_MODE = 116; // 속도-전류[절대] 제어
// static const uint8_t MOTION_MODE = 117; // 속도-전류[상대] 제어

// 목표 전류
static const float SUB_TARGET = 10.0f; //전류 10A (가속도에 비례)

// 테스트용 목표 속도 (항상 양수)
static const float MAIN_TARGET_RPM = 100.0f; //속도 100RPM

// 방향 정의
static const uint8_t DIR_ZERO = 0;      //속도 0
static const uint8_t DIR_POSITIVE = 1;  //정방향 +RPM
static const uint8_t DIR_NEGATIVE = 2;  //역방향 -RPM

// 각 구간 모니터링 시간
static const unsigned long RUN_MONITOR_MS = 5000UL;

// 정지 방식 선택
// 1 = Coast Stop      -> Enable=0, Brake=1
// 2 = Dynamic Brake   -> Enable=0, Brake=0
#define STOP_MODE_COAST          1
#define STOP_MODE_DYNAMIC_BRAKE  2

static const uint8_t STOP_MODE = STOP_MODE_COAST;

// =========================================================
// PRINT HELPERS
// =========================================================
static void printLine()
{
    Serial.println(F("=================================================="));
}

static void printTitle(const char* title)
{
    Serial.println();
    printLine();
    Serial.println(title);
    printLine();
}

static void printStep(const char* name, bool ok)
{
    Serial.print(name);
    Serial.print(F(" : "));
    Serial.println(ok ? F("OK") : F("FAIL"));
}

static void printU8(const char* name, uint8_t v)
{
    Serial.print(name);
    Serial.print(F(" : "));
    Serial.println((unsigned long)v);
}

static void printF32(const char* name, float v, uint8_t digits = 2)
{
    Serial.print(name);
    Serial.print(F(" : "));
    Serial.println(v, digits);
}

static void printFaultHex(const char* name, uint8_t v)
{
    Serial.print(name);
    Serial.print(F(" : 0x"));
    if (v < 0x10) Serial.print('0');
    Serial.println(v, HEX);
}

static void printPulseInline(int64_t v)
{
    if (v == 0) {
        Serial.print(F("0"));
        return;
    }

    if (v < 0) {
        Serial.print(F("-"));
        v = -v;
    }

    char buf[24];
    int i = 0;
    while (v > 0 && i < (int)(sizeof(buf) - 1)) {
        buf[i++] = '0' + (v % 10);
        v /= 10;
    }

    while (i--) Serial.print(buf[i]);
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
    printPulseInline(s.pulseCount);
    Serial.println();
}

// =========================================================
// HELPERS
// =========================================================
static bool checkFaultZero()
{
    uint8_t fault = 0;
    bool ok = drive.getFault(TARGET_ID, fault);
    printStep("getFault", ok);
    if (!ok) return false;

    printFaultHex("Fault", fault);
    return (fault == 0x00);
}

// 구동 준비
// Non-brake = 1
// Enable    = 1
static bool setRunReady()
{
    bool ok;

    ok = drive.setMotorBrake(TARGET_ID, 1);   // 브레이크 해제
    printStep("setMotorBrake(1)", ok);
    if (!ok) return false;
    delay(100);

    ok = drive.setMotorEnable(TARGET_ID, 1);  // enable
    printStep("setMotorEnable(1)", ok);
    if (!ok) return false;
    delay(100);

    return true;
}

// 목표 속도 0 명령
static bool commandSpeedZero()
{
    bool ok = drive.setMotorMotionAll(TARGET_ID,
                                      ACCEL_TIME_MS,
                                      DECEL_TIME_MS,
                                      MOTION_MODE,
                                      SUB_TARGET,
                                      0.0f,
                                      DIR_POSITIVE);

    printStep("setMotorMotionAll(0)", ok);
    delay(100);
    return ok;
}

// Coast Stop
// Enable=0, Brake=1
static bool stopCoast()
{
    bool ok;

    ok = drive.setMotorEnable(TARGET_ID, 0);
    printStep("setMotorEnable(0)", ok);
    delay(100);

    ok = drive.setMotorBrake(TARGET_ID, 1);
    printStep("setMotorBrake(1)", ok);
    delay(100);

    return ok;
}

// Dynamic Brake Stop
// Enable=0, Brake=0
static bool stopDynamicBrake()
{
    bool ok;

    ok = drive.setMotorEnable(TARGET_ID, 0);
    printStep("setMotorEnable(0)", ok);
    delay(100);

    ok = drive.setMotorBrake(TARGET_ID, 0);
    printStep("setMotorBrake(0)", ok);
    delay(100);

    return ok;
}

static bool stopMotor()
{
    printTitle("STOP MOTOR");

    // 먼저 목표 속도 0 명령
    if (!commandSpeedZero()) {
        return false;
    }

    delay(300);

    if (STOP_MODE == STOP_MODE_DYNAMIC_BRAKE) {
        Serial.println(F("Stop Mode : Dynamic Brake"));
        return stopDynamicBrake();
    }

    Serial.println(F("Stop Mode : Coast"));
    return stopCoast();
}

static bool monitorStatus(unsigned long durationMs)
{
    uint32_t tStart = millis();

    while (millis() - tStart < durationMs)
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
                Serial.println(F("Fault detected during motor run."));
                return false;
            }
        }
        else
        {
            Serial.println(F("Status read fail"));
        }

        delay(200);
    }

    return true;
}

static bool runSpeed(float speedTarget, uint8_t direction)
{
    bool ok = drive.setMotorMotionAll(TARGET_ID,
                                      ACCEL_TIME_MS,
                                      DECEL_TIME_MS,
                                      MOTION_MODE,
                                      SUB_TARGET,
                                      speedTarget,
                                      direction);

    if (direction == DIR_ZERO) {
        printStep("setMotorMotionAll(zero)", ok);
    } 
    else if (direction == DIR_POSITIVE) {
        printStep("setMotorMotionAll(+dir)", ok);
    } 
    else {
        printStep("setMotorMotionAll(-dir)", ok);
    }

    if (!ok) return false;

    return monitorStatus(RUN_MONITOR_MS);
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

    printTitle("WooDrive Example04 Speed");

    uint8_t idRead = 0;
    bool ok = drive.getId(TARGET_ID, idRead);
    printStep("getId", ok);
    if (!ok) return;

    printU8("ID", idRead);

    if (!checkFaultZero())
    {
        printTitle("STOP");
        Serial.println(F("WooDrive fault detected before run."));
        return;
    }

    printTitle("SET RUN READY");
    if (!setRunReady())
    {
        printTitle("STOP");
        Serial.println(F("Run ready setup failed."));
        return;
    }

    // =====================================================
    // RUN 1 : + SPEED
    // =====================================================
    printTitle("RUN 1 : + SPEED");
    printF32("Speed Target", MAIN_TARGET_RPM, 2);
    printU8("Direction", DIR_POSITIVE);

    if (!runSpeed(MAIN_TARGET_RPM, DIR_POSITIVE))
    {
        stopMotor();
        return;
    }

    if (!stopMotor())
    {
        return;
    }

    delay(1000);

    // =====================================================
    // RUN 2 : - SPEED
    // =====================================================
    printTitle("RUN 2 : - SPEED");
    printF32("Speed Target", MAIN_TARGET_RPM, 2);
    printU8("Direction", DIR_NEGATIVE);

    if (!setRunReady())
    {
        Serial.println(F("Run ready setup failed before RUN 2."));
        return;
    }

    if (!runSpeed(MAIN_TARGET_RPM, DIR_NEGATIVE))
    {
        stopMotor();
        return;
    }

    if (!stopMotor())
    {
        return;
    }

    // =====================================================
    // RUN 3 : ZERO
    // =====================================================
    printTitle("RUN 3 : ZERO");
    printF32("Speed Target", MAIN_TARGET_RPM, 2);
    printU8("Direction", DIR_ZERO);

    if (!setRunReady())
    {
        Serial.println(F("Run ready setup failed before RUN 3."));
        return;
    }

    if (!runSpeed(MAIN_TARGET_RPM, DIR_ZERO))
    {
        stopMotor();
        return;
    }

    if (!stopMotor())
    {
        return;
    }

        // =====================================================
    // RUN 4 : + SPEED -> - SPEED
    // =====================================================
    printTitle("RUN 4 : + SPEED -> - SPEED");

    if (!setRunReady())
    {
        Serial.println(F("Run ready setup failed before RUN 4."));
        return;
    }

    runSpeed(MAIN_TARGET_RPM, DIR_POSITIVE);
    delay(2000);
    runSpeed(MAIN_TARGET_RPM, DIR_NEGATIVE);
    delay(2000);

    // =====================================================
    // RUN 5 : ZERO
    // =====================================================
    printTitle("RUN 5 : ZERO");
    printF32("Speed Target", MAIN_TARGET_RPM, 2);
    printU8("Direction", DIR_ZERO);

    if (!setRunReady())
    {
        Serial.println(F("Run ready setup failed before RUN 5."));
        return;
    }

    if (!runSpeed(MAIN_TARGET_RPM, DIR_ZERO))
    {
        stopMotor();
        return;
    }

    if (!stopMotor())
    {
        return;
    }


    printTitle("DONE");
}

void loop()
{
}