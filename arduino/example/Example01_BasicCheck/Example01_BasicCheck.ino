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
// SETUP
// =========================================================
void setup()
{
    Serial.begin(9600);
    WOO_SERIAL.begin(WOODRIVE_BAUDRATE); 
    delay(5000);

    drive.setTimeout(300);

    Serial.println("=== WooDrive Example01 : Basic Check ===");

    uint8_t id = 0;
    uint8_t fault = 0;

    // =========================
    // 1. ID 확인
    // =========================
    if (drive.getId(TARGET_ID, id)) {
        Serial.print("ID OK : ");
        Serial.println(id);
    } else {
        Serial.println("ID FAIL");
        return;
    }
    
    delay(100);

    // =========================
    // 2. Fault 확인
    // =========================
    if (drive.getFault(TARGET_ID, fault)) {
        Serial.print("FAULT : ");
        Serial.println(fault);
    } else {
        Serial.println("FAULT READ FAIL");
        return;
    }

    // =========================
    // RESULT
    // =========================
    Serial.println("=== RESULT : PASS ===");
}

void loop()
{
}