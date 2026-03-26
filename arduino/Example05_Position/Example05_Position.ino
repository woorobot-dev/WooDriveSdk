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
// POSITION CONTROL OPTION
// =========================================================
// 이 예제는 "위치제어" 기준 예제
// mainTarget : 목표 위치값 (항상 양수)
// subTarget  : 속도 제한값 (속도에 비례)
// direction  : 0 = 원점, 1 = 정방향, 2 = 역방향

static const uint16_t ACCEL_TIME_MS = 1000; // 가속 시간
static const uint16_t DECEL_TIME_MS = 1000; // 감속 시간

// 제어기 위치제어 motion mode 값으로 수정
// static const uint8_t MOTION_MODE = 244; // 위치-속도[절대] 제어
static const uint8_t MOTION_MODE = 245; //  위치-속도[상대] 제어

// 방향 정의
static const uint8_t DIR_ZERO = 0;
static const uint8_t DIR_POSITIVE = 1;
static const uint8_t DIR_NEGATIVE = 2;

// 등속구간 최대 속도 제한값
static const float SUB_TARGET = 100.0f; //100rpm

// 목표 위치값 (항상 양수)
// 처음엔 작은 값으로 시작하는 걸 추천
static const float MAIN_TARGET_POS = 360.0f; // +360degree/ -360degree 예시

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
                Serial.println(F("Fault detected during position control."));
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

// =========================================================
// SETUP
// =========================================================
void setup()
{
    Serial.begin(9600);
    WOO_SERIAL.begin(WOODRIVE_BAUDRATE);  
    delay(5000);

    drive.setTimeout(300);

    printTitle("WooDrive Example05 Position");

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
    // RUN 1 : ZERO
    // =====================================================
    printTitle("RUN 1 : ZERO");
    printF32("Position Target", MAIN_TARGET_POS, 2);
    printU8("Direction", DIR_ZERO);
    if (!runPosition(MAIN_TARGET_POS, DIR_ZERO))
    {
        stopMotor();
        return;
    }
    delay(500);

    // =====================================================
    // RUN 2 : + POSITION
    // =====================================================
    printTitle("RUN 2 : + POSITION");
    printF32("Position Target", MAIN_TARGET_POS, 2);
    printU8("Direction", DIR_POSITIVE);
    if (!runPosition(MAIN_TARGET_POS, DIR_POSITIVE))
    {
        stopMotor();
        return;
    }
    delay(500);

    // =====================================================
    // RUN 3 : - POSITION
    // =====================================================
    printTitle("RUN 3 : - POSITION");
    printF32("Position Target", MAIN_TARGET_POS, 2);
    printU8("Direction", DIR_NEGATIVE);
    if (!runPosition(MAIN_TARGET_POS, DIR_NEGATIVE))
    {
        stopMotor();
        return;
    }
    delay(500);

    // =====================================================
    // RUN 4 : - POSITION
    // =====================================================
    printTitle("RUN 4 : - POSITION");
    printF32("Position Target", MAIN_TARGET_POS, 2);
    printU8("Direction", DIR_NEGATIVE);
    if (!runPosition(MAIN_TARGET_POS, DIR_NEGATIVE))
    {
        stopMotor();
        return;
    }
    delay(500);

    // =====================================================
    // RUN 5 : + POSITION
    // =====================================================
    printTitle("RUN 5 : + POSITION");
    printF32("Position Target", MAIN_TARGET_POS, 2);
    printU8("Direction", DIR_POSITIVE);
    if (!runPosition(MAIN_TARGET_POS, DIR_POSITIVE))
    {
        stopMotor();
        return;
    }
    delay(500);

    // =====================================================
    // RUN 6 : ZERO
    // =====================================================
    printTitle("RUN 6 : ZERO");
    printF32("Position Target", MAIN_TARGET_POS, 2);
    printU8("Direction", DIR_ZERO);
    if (!runPosition(MAIN_TARGET_POS, DIR_ZERO))
    {
        stopMotor();
        return;
    }

    if (!stopMotor())
    {
        return;
    }
    delay(500);
    printTitle("DONE");
}

void loop()
{
}