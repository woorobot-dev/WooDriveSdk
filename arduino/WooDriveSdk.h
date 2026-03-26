#pragma once

#if defined(__AVR__) || defined(ARDUINO_ARCH_AVR)
#include <stddef.h>
#include <stdint.h>
#else
#include <cstddef>
#include <cstdint>
#endif

#if defined(__AVR__)
#include <avr/pgmspace.h>
#define WOODRIVE_PROGMEM PROGMEM
#define WOODRIVE_READ_WORD(addr) pgm_read_word(addr)
#else
#define WOODRIVE_PROGMEM
#define WOODRIVE_READ_WORD(addr) (*(const uint16_t*)(addr))
#endif

#if defined(ARDUINO)
#include <Arduino.h>

#if defined(__has_include)
  #if __has_include(<SoftwareSerial.h>)
    #include <SoftwareSerial.h>
    #define WOODRIVE_HAS_SOFTWARESERIAL 1
  #endif
#endif

#endif
class ITransport
{
public:
    virtual ~ITransport() = default;

    virtual size_t write(const uint8_t* data, size_t len) = 0;
    virtual int available() = 0;
    virtual int read() = 0;
    virtual void flush() = 0;
    virtual void clearRx() = 0;
};

class IClock
{
public:
    virtual ~IClock() = default;
    virtual uint32_t nowMs() const = 0;
};

namespace WooProtocol {
    struct Packet;
    struct ResponseFrame;
}

enum class ResistanceUnit : uint8_t {
    uOhm,
    mOhm,
    Ohm
};

enum class InductanceUnit : uint8_t {
    nH,
    uH,
    mH,
    H
};

enum class TorqueConstantUnit : uint8_t {
    mNmPerA,
    NmPerA
};

enum class InertiaUnit : uint8_t {
    g_mm2,
    g_cm2,
    g_m2,
    kg_mm2,
    kg_cm2,
    kg_m2
};

class WooDrive
{
public:
    explicit WooDrive(ITransport& transport, IClock& clock);

    void setTimeout(uint32_t timeoutMs);
    uint32_t timeout() const;

    // BOARD CONFIG
    bool setReset(uint8_t id, uint8_t value);
    bool getReset(uint8_t id, uint8_t& outValue);
    bool setId(uint8_t id, uint8_t value);
    bool getId(uint8_t id, uint8_t& outValue);
    bool getFault(uint8_t id, uint8_t& outValue);
    bool setCommunicationMode(uint8_t id, uint8_t value);
    bool getCommunicationMode(uint8_t id, uint8_t& outValue);
    bool setCommunicationWatchdog(uint8_t id, uint16_t value);
    bool getCommunicationWatchdog(uint8_t id, uint16_t& outValue);
    bool setBps(uint8_t id, uint32_t value);
    bool getBps(uint8_t id, uint32_t& outValue);

    // MOTOR CONFIG
    bool setMotorType(uint8_t id, uint8_t value);
    bool getMotorType(uint8_t id, uint8_t& outValue);
    bool setFeedbackType(uint8_t id, uint8_t value);
    bool getFeedbackType(uint8_t id, uint8_t& outValue);
    bool setStartupFeedbackType(uint8_t id, uint8_t value);
    bool getStartupFeedbackType(uint8_t id, uint8_t& outValue);
    bool setDirectionInvert(uint8_t id, uint8_t value);
    bool getDirectionInvert(uint8_t id, uint8_t& outValue);
    bool setFieldWeakeningEnable(uint8_t id, uint8_t value);
    bool getFieldWeakeningEnable(uint8_t id, uint8_t& outValue);
    bool setExternalBrakePresent(uint8_t id, uint8_t value);
    bool getExternalBrakePresent(uint8_t id, uint8_t& outValue);
    bool setPolePairs(uint8_t id, uint8_t value);
    bool getPolePairs(uint8_t id, uint8_t& outValue);
    bool setFeedbackDir(uint8_t id, uint8_t value);
    bool getFeedbackDir(uint8_t id, uint8_t& outValue);
    bool setFeedbackResolution(uint8_t id, uint32_t value);
    bool getFeedbackResolution(uint8_t id, uint32_t& outValue);
    bool setGear(uint8_t id, float value);
    bool getGear(uint8_t id, float& outValue);

    // MOTOR FOC SETTING
    bool setAutoMotorSetup(uint8_t id, uint8_t value);
    bool getAutoMotorSetup(uint8_t id, uint8_t& outValue);
    bool setAlignmentMode(uint8_t id, uint8_t value);
    bool getAlignmentMode(uint8_t id, uint8_t& outValue);
    bool setPhaseOffset(uint8_t id, uint32_t value);
    bool getPhaseOffset(uint8_t id, uint32_t& outValue);
    bool setForceAngleLevel(uint8_t id, uint8_t value);
    bool getForceAngleLevel(uint8_t id, uint8_t& outValue);
    bool setHallSensorSetting(uint8_t id, uint8_t value);
    bool getHallSensorSetting(uint8_t id, uint8_t& outValue);
    bool getMainPortLevelNow(uint8_t id, uint8_t& outValue);
    bool getSubPortLevelNow(uint8_t id, uint8_t& outValue);

    // MOTOR PARAM
    bool setAutoParameter(uint8_t id, uint8_t value);
    bool getAutoParameter(uint8_t id, uint8_t& outValue);
    bool setRatedSpeed(uint8_t id, float value);
    bool setRatedSpeedRaw(uint8_t id, uint32_t value);
    bool getRatedSpeedRaw(uint8_t id, uint32_t& outValue);
    bool getRatedSpeed(uint8_t id, float& outValue);
    bool setRatedCurrent(uint8_t id, float value);
    bool setRatedCurrentRaw(uint8_t id, uint16_t value);
    bool getRatedCurrentRaw(uint8_t id, uint16_t& outValue);
    bool getRatedCurrent(uint8_t id, float& outValue);
    bool setRatedVoltage(uint8_t id, float value);
    bool setRatedVoltageRaw(uint8_t id, uint16_t value);
    bool getRatedVoltageRaw(uint8_t id, uint16_t& outValue);
    bool getRatedVoltage(uint8_t id, float& outValue);
    bool setResistanceConstant(uint8_t id, float value);
    bool setResistanceConstant(uint8_t id, float value, ResistanceUnit unit);
    bool getResistanceConstantRaw(uint8_t id, uint32_t& outValue);
    bool getResistanceConstant(uint8_t id, float& outValue);
    bool getResistanceConstant(uint8_t id, float& outValue, ResistanceUnit unit);
    bool setInductanceConstant(uint8_t id, float value);
    bool setInductanceConstant(uint8_t id, float value, InductanceUnit unit);
    bool getInductanceConstantRaw(uint8_t id, uint32_t& outValue);
    bool getInductanceConstant(uint8_t id, float& outValue);
    bool getInductanceConstant(uint8_t id, float& outValue, InductanceUnit unit);
    bool setTorqueConstant(uint8_t id, float value);
    bool setTorqueConstant(uint8_t id, float value, TorqueConstantUnit unit);
    bool getTorqueConstantRaw(uint8_t id, uint32_t& outValue);
    bool getTorqueConstant(uint8_t id, float& outValue);
    bool getTorqueConstant(uint8_t id, float& outValue, TorqueConstantUnit unit);
    bool setMomentOfInertia(uint8_t id, float value);
    bool setMomentOfInertia(uint8_t id, float value, InertiaUnit unit);
    bool getMomentOfInertiaRaw(uint8_t id, uint32_t& outValue);
    bool getMomentOfInertia(uint8_t id, float& outValue);
    bool getMomentOfInertia(uint8_t id, float& outValue, InertiaUnit unit);

    // MOTOR GAIN
    bool setGainMode(uint8_t id, uint8_t value);
    bool getGainMode(uint8_t id, uint8_t& outValue);
    bool setPositionGainScale(uint8_t id, uint8_t value);
    bool getPositionGainScale(uint8_t id, uint8_t& outValue);
    bool setVelocityGainScale(uint8_t id, uint8_t value);
    bool getVelocityGainScale(uint8_t id, uint8_t& outValue);
    bool setCurrentGainScale(uint8_t id, uint8_t value);
    bool getCurrentGainScale(uint8_t id, uint8_t& outValue);
    bool setPositionPGain(uint8_t id, uint32_t value);
    bool getPositionPGain(uint8_t id, uint32_t& outValue);
    bool setVelocityPGain(uint8_t id, uint32_t value);
    bool getVelocityPGain(uint8_t id, uint32_t& outValue);
    bool setVelocityIGain(uint8_t id, uint32_t value);
    bool getVelocityIGain(uint8_t id, uint32_t& outValue);
    bool setCurrentPGain(uint8_t id, uint32_t value);
    bool getCurrentPGain(uint8_t id, uint32_t& outValue);
    bool setCurrentIGain(uint8_t id, uint32_t value);
    bool getCurrentIGain(uint8_t id, uint32_t& outValue);

    // MOTOR LIMIT
    bool setPositionCcwMax(uint8_t id, float value);
    bool getPositionCcwMax(uint8_t id, float& outValue);
    bool setPositionCwMax(uint8_t id, float value);
    bool getPositionCwMax(uint8_t id, float& outValue);
    bool setVelocityCcwMax(uint8_t id, float value);
    bool getVelocityCcwMax(uint8_t id, float& outValue);
    bool setVelocityCwMax(uint8_t id, float value);
    bool getVelocityCwMax(uint8_t id, float& outValue);
    bool setIqCurrentCcwMax(uint8_t id, float value);
    bool getIqCurrentCcwMax(uint8_t id, float& outValue);
    bool setIqCurrentCwMax(uint8_t id, float value);
    bool getIqCurrentCwMax(uint8_t id, float& outValue);
    bool setIdCurrentMax(uint8_t id, float value);
    bool getIdCurrentMax(uint8_t id, float& outValue);
    bool setIqCurrentLimit(uint8_t id, float value);
    bool getIqCurrentLimit(uint8_t id, float& outValue);
    bool setBusVoltageMaxLimit(uint8_t id, float value);
    bool getBusVoltageMaxLimit(uint8_t id, float& outValue);
    bool setBusVoltageMinLimit(uint8_t id, float value);
    bool getBusVoltageMinLimit(uint8_t id, float& outValue);
    bool setTemperatureMaxLimit(uint8_t id, float value);
    bool getTemperatureMaxLimit(uint8_t id, float& outValue);

    // MOTOR CONTROL
    bool setMotorAccelTime(uint8_t targetId, uint16_t value);
    bool getMotorAccelTime(uint8_t targetId, uint16_t& outValue);
    bool setMotorDecelTime(uint8_t targetId, uint16_t value);
    bool getMotorDecelTime(uint8_t targetId, uint16_t& outValue);
    bool setMotorMotionMode(uint8_t targetId, uint8_t value);
    bool getMotorMotionMode(uint8_t targetId, uint8_t& outValue);
    bool setMotorSubTarget(uint8_t targetId, float value);
    bool getMotorSubTarget(uint8_t targetId, float& outValue);
    bool setMotorMainTarget(uint8_t targetId, float value);
    bool getMotorMainTarget(uint8_t targetId, float& outValue);
    bool setMotorDirection(uint8_t targetId, uint8_t value);
    bool getMotorDirection(uint8_t targetId, uint8_t& outValue);
    bool setRunMode(uint8_t targetId, uint8_t value);
    bool getRunMode(uint8_t targetId, uint8_t& outValue);
    bool setMotorEnable(uint8_t targetId, uint8_t value);
    bool getMotorEnable(uint8_t targetId, uint8_t& outValue);
    bool setMotorBrake(uint8_t targetId, uint8_t value);
    bool getMotorBrake(uint8_t targetId, uint8_t& outValue);
    bool setExternalBrake(uint8_t targetId, uint8_t value);
    bool getExternalBrake(uint8_t targetId, uint8_t& outValue);

    // MOTOR STATUS
    bool getTotalTimeNow(uint8_t id, uint32_t& outValue);
    bool getElapsedTimeNow(uint8_t id, uint32_t& outValue);
    bool getRemainTimeNow(uint8_t id, uint32_t& outValue);
    bool getPositionNow(uint8_t id, float& outValue);
    bool getVelocityNow(uint8_t id, float& outValue);
    bool getIqCurrentNow(uint8_t id, float& outValue);
    bool getIdCurrentNow(uint8_t id, float& outValue);
    bool getBusCurrentNow(uint8_t id, float& outValue);
    bool getVqVoltageNow(uint8_t id, float& outValue);
    bool getVdVoltageNow(uint8_t id, float& outValue);
    bool getBusVoltageNow(uint8_t id, float& outValue);
    bool getTemperatureNow(uint8_t id, float& outValue);
    bool getMotorPulseCountNow(uint8_t id, int64_t& outValue);
    bool getUPhaseCurrentNow(uint8_t id, float& outValue);
    bool getWPhaseCurrentNow(uint8_t id, float& outValue);

    // =========================================================
    // HIGH LEVEL SET
    // =========================================================
    bool setMotorMainTargetDir(uint8_t id, float mainTarget, uint8_t dir);
    bool setMotorSubMainTargetDir(uint8_t id, float subTarget, float mainTarget, uint8_t dir);
    bool setMotorMotionAll(uint8_t id,
                           uint16_t accelTime,
                           uint16_t decelTime,
                           uint8_t motionMode,
                           float subTarget,
                           float mainTarget,
                           uint8_t dir);

    bool setMotorMainTargetSigned(uint8_t id, float mainTarget);
    bool setMotorSubMainTargetSigned(uint8_t id, float subTarget, float mainTarget);
    bool setMotorMotionAllSigned(uint8_t id,
                                uint16_t accelTime,
                                uint16_t decelTime,
                                uint8_t motionMode,
                                float subTarget,
                                float mainTarget);
                                
    // =========================================================
    // HIGH LEVEL GET
    // =========================================================
    struct PositionVelocity
    {
        float position;
        float velocity;
    };

    struct PositionVelocityIq
    {
        float position;
        float velocity;
        float iqCurrent;
    };
    bool getPositionVelocity(uint8_t id, float& position, float& velocity);
    bool getPositionVelocity(uint8_t id, PositionVelocity& s);

    bool getPositionVelocityIq(uint8_t id, float& position, float& velocity, float& iqCurrent);
    bool getPositionVelocityIq(uint8_t id, PositionVelocityIq& s);
    
    struct BoardConfig
    {
        uint8_t  reset;
        uint8_t  id;
        uint8_t  fault;
        uint8_t  communicationMode;
        uint32_t bps;
        uint16_t watchdog;
    };

    struct MotorConfig
    {
        uint8_t  motorType;
        uint8_t  feedbackType;
        uint8_t  startupFeedbackType;
        uint8_t  directionInvert;
        uint8_t  fieldWeakeningEnable;
        uint8_t  externalBrakePresent;
        uint8_t  polePairs;
        uint8_t  feedbackDir;
        uint32_t feedbackResolution;
        float    gear;
    };

    struct MotorFocSetting
    {
        uint8_t  autoMotorSetup;
        uint8_t  alignmentMode;
        uint32_t phaseOffset;
        uint8_t  forceAngleLevel;
        uint8_t  hallSensorSetting;
        uint8_t  mainPortLevelNow;
        uint8_t  subPortLevelNow;
    };

    struct MotorParam
    {
        uint8_t  autoParameter;
        float    ratedSpeed;
        float    ratedCurrent;
        float    ratedVoltage;
        float    resistance;
        float    inductance;
        float    torqueConstant;
        float    momentOfInertia;
    };

    struct MotorGain
    {
        uint8_t  gainMode;
        uint8_t  positionGainScale;
        uint8_t  velocityGainScale;
        uint8_t  currentGainScale;
        uint32_t positionPGain;
        uint32_t velocityPGain;
        uint32_t velocityIGain;
        uint32_t currentPGain;
        uint32_t currentIGain;
    };

    struct MotorLimit
    {
        float positionCcwMax;
        float positionCwMax;
        float velocityCcwMax;
        float velocityCwMax;
        float iqCurrentCcwMax;
        float iqCurrentCwMax;
        float idCurrentMax;
        float iqCurrentLimit;
        float busVoltageMaxLimit;
        float busVoltageMinLimit;
        float temperatureMaxLimit;
    };

    struct MotorControl
    {
        uint16_t accelTime;
        uint16_t decelTime;
        uint8_t  motionMode;
        float    subTarget;
        float    mainTarget;
        uint8_t  direction;
        uint8_t  runMode;
        uint8_t  motorEnable;
        uint8_t  motorBrake;
        uint8_t  externalBrake;
    };

    struct MotorStatus
    {
        uint32_t totalTime;
        uint32_t elapsedTime;
        uint32_t remainTime;

        float position;
        float velocity;

        float iqCurrent;
        float idCurrent;
        float busCurrent;

        float vqVoltage;
        float vdVoltage;
        float busVoltage;

        float temperature;

        int64_t pulseCount;

        float uPhaseCurrent;
        float wPhaseCurrent;
    };

    // BLOCK READ ALL
    bool getBoardConfigAll(uint8_t id, uint8_t& reset, uint8_t& idRead, uint8_t& fault, 
                        uint8_t& communicationMode, uint32_t& bps, uint16_t& watchdog);
    bool getBoardConfigAll(uint8_t id, BoardConfig& s);

    bool getMotorConfigAll(uint8_t id, uint8_t& motorType, uint8_t& feedbackType, uint8_t& startupFeedbackType,
                        uint8_t& directionInvert, uint8_t& fieldWeakeningEnable, uint8_t& externalBrakePresent,
                        uint8_t& polePairs, uint8_t& feedbackDir, uint32_t& feedbackResolution, float& gear);
    bool getMotorConfigAll(uint8_t id, MotorConfig& s);

    bool getMotorFocSettingAll(uint8_t id, uint8_t& autoMotorSetup, uint8_t& alignmentMode, uint32_t& phaseOffset,
                            uint8_t& forceAngleLevel, uint8_t& hallSensorSetting, uint8_t& mainPortLevelNow, uint8_t& subPortLevelNow);
    bool getMotorFocSettingAll(uint8_t id, MotorFocSetting& s);

    bool getMotorParamAll(uint8_t id, uint8_t& autoParameter, float& ratedSpeed, float& ratedCurrent, float& ratedVoltage,
                        float& resistance, float& inductance, float& torqueConstant, float& momentOfInertia);
    bool getMotorParamAll(uint8_t id, MotorParam& s);

    bool getMotorGainAll(uint8_t id, uint8_t& gainMode, uint8_t& positionGainScale, uint8_t& velocityGainScale, uint8_t& currentGainScale,
                        uint32_t& positionPGain, uint32_t& velocityPGain, uint32_t& velocityIGain, uint32_t& currentPGain, uint32_t& currentIGain);
    bool getMotorGainAll(uint8_t id, MotorGain& s);

    bool getMotorLimitAll(uint8_t id, float& positionCcwMax, float& positionCwMax,
                        float& velocityCcwMax, float& velocityCwMax,
                        float& iqCurrentCcwMax, float& iqCurrentCwMax, float& idCurrentMax, float& iqCurrentLimit,
                        float& busVoltageMaxLimit, float& busVoltageMinLimit, float& temperatureMaxLimit);
    bool getMotorLimitAll(uint8_t id, MotorLimit& s);

    bool getMotorControlAll(uint8_t id, uint16_t& accelTime, uint16_t& decelTime, uint8_t& motionMode,
                            float& subTarget, float& mainTarget, uint8_t& direction,
                            uint8_t& runMode, uint8_t& motorEnable, uint8_t& motorBrake, uint8_t& externalBrake);
    bool getMotorControlAll(uint8_t id, MotorControl& s);

    bool getMotorStatusAll(uint8_t id, uint32_t& totalTime, uint32_t& elapsedTime, uint32_t& remainTime,
                           float& position, float& velocity, float& iqCurrent, float& idCurrent, float& busCurrent, 
                           float& vqVoltage, float& vdVoltage, float& busVoltage, float& temperature,
                           int64_t& pulseCount, float& uPhaseCurrent, float& wPhaseCurrent);
    bool getMotorStatusAll(uint8_t id, MotorStatus& s);

private:
    ITransport& _transport;
    IClock& _clock;
    uint32_t _timeoutMs;

    bool sendOnly(const WooProtocol::Packet& pkt);
    bool sendAndReceive(const WooProtocol::Packet& pkt, WooProtocol::ResponseFrame& rsp);
    bool readFrame(WooProtocol::ResponseFrame& rsp);
    bool checkRsp(const WooProtocol::ResponseFrame& rsp, uint8_t id, uint8_t addr, uint8_t minLen) const;

    static bool makeGetU8(WooProtocol::Packet& pkt, uint8_t id, uint8_t addr);
    static bool makeGetU16(WooProtocol::Packet& pkt, uint8_t id, uint8_t addr);
    static bool makeGetU24(WooProtocol::Packet& pkt, uint8_t id, uint8_t addr);
    static bool makeGetU32(WooProtocol::Packet& pkt, uint8_t id, uint8_t addr);

    static bool makeSetU8(WooProtocol::Packet& pkt, uint8_t id, uint8_t addr, uint8_t value);
    static bool makeSetU16(WooProtocol::Packet& pkt, uint8_t id, uint8_t addr, uint16_t value);
    static bool makeSetU24(WooProtocol::Packet& pkt, uint8_t id, uint8_t addr, uint32_t value);
    static bool makeSetU32(WooProtocol::Packet& pkt, uint8_t id, uint8_t addr, uint32_t value);
    static bool makeSetU16x100(WooProtocol::Packet& pkt, uint8_t id, uint8_t addr, float value);
    static bool makeSetU24x100(WooProtocol::Packet& pkt, uint8_t id, uint8_t addr, float value);
    static bool makeSetU32x1e6(WooProtocol::Packet& pkt, uint8_t id, uint8_t addr, float value);
    static bool makeGetS32(WooProtocol::Packet& pkt, uint8_t id, uint8_t addr);
    static bool makeSetS32(WooProtocol::Packet& pkt, uint8_t id, uint8_t addr, int32_t value);

    static uint8_t  beU8(const uint8_t* d);
    static uint16_t beU16(const uint8_t* d);
    static uint32_t beU24(const uint8_t* d);
    static uint32_t beU32(const uint8_t* d);
    static uint64_t beU64(const uint8_t* d);
    static int32_t  beS24(const uint8_t* d);
    static int32_t  beS32(const uint8_t* d);
    static float    fixed2u16(const uint8_t* d);
    static float    fixed2u24(const uint8_t* d);
    static float    fixed2s16(const uint8_t* d);
    static float    fixed2s32(const uint8_t* d);
    static float    fixed6u32(const uint8_t* d);
};




namespace WooProtocol
{
    constexpr uint8_t DIR_SET       = 0x01;
    constexpr uint8_t DIR_GET       = 0x02;
    constexpr uint8_t DIR_RSP       = 0x03;
    constexpr uint8_t DIR_FAULT_RSP = 0x04;

    static constexpr size_t MAX_PACKET_SIZE   = 128;
    static constexpr size_t MAX_RESPONSE_DATA = 64;

    struct Packet
    {
        uint8_t buf[MAX_PACKET_SIZE];
        size_t len = 0;
    };

    struct ResponseFrame
    {
        uint8_t id = 0;
        uint8_t dir = 0;
        uint8_t cmd = 0;
        uint8_t lenField = 0;
        uint8_t data[MAX_RESPONSE_DATA] = {0};
        uint8_t dataLen = 0;
        uint16_t crc = 0;
    };

    uint16_t computeCrc16(const uint8_t* data, size_t len);

    bool makeRequest(Packet& out,
                     uint8_t targetId,
                     const uint8_t* payload,
                     size_t payloadLen);

    bool makeRequestAutoLen(Packet& out,
                            uint8_t targetId,
                            uint8_t dir,
                            uint8_t cmd,
                            const uint8_t* data,
                            size_t dataLen);

    bool validateFrame(const ResponseFrame& rsp);
}


namespace Address {
constexpr uint8_t RESET                             = 0x00;
constexpr uint8_t ID                                = 0x01;
constexpr uint8_t FAULT                             = 0x02;
constexpr uint8_t COMMUNICATION_MODE                = 0x03;
constexpr uint8_t BAUDRATE                          = 0x04;
constexpr uint8_t WATCHDOG                          = 0x08;

constexpr uint8_t MOTOR_TYPE                        = 0x10;
constexpr uint8_t FEEDBACK_TYPE                     = 0x11;
constexpr uint8_t STARTUP_FEEDBACK_TYPE             = 0x12;

constexpr uint8_t DIRECTION_INVERT                  = 0x13;
constexpr uint8_t FIELD_WEAKENING_ENABLE            = 0x14;

constexpr uint8_t EXTERNAL_BRAKE_PRESENT            = 0x15;
constexpr uint8_t POLE_PAIRS                        = 0x16;
constexpr uint8_t FEEDBACK_DIR                      = 0x17;
constexpr uint8_t FEEDBACK_RESOLUTION               = 0x18;
constexpr uint8_t GEAR                              = 0x1C;

constexpr uint8_t AUTO_MOTOR_SETUP                  = 0x20;
constexpr uint8_t ALIGNMENT_MODE                    = 0x21;
constexpr uint8_t PHASE_OFFSET                      = 0x22;
constexpr uint8_t FORCE_ANGLE_LEVEL                 = 0x26;
constexpr uint8_t HALLSENSOR_SETTING                = 0x27;
constexpr uint8_t MAINPORT_LEVEL_NOW                = 0x28;
constexpr uint8_t SUBPORT_LEVEL_NOW                 = 0x29;


constexpr uint8_t MOTOR_AUTO_PARAMETER_MEASUREMENT_MODE  = 0x30;
constexpr uint8_t MOTOR_RATED_SPEED                 = 0x31;
constexpr uint8_t MOTOR_RATED_CURRENT               = 0x34;
constexpr uint8_t MOTOR_RATED_VOLTAGE               = 0x36;

constexpr uint8_t MOTOR_RESISTANCE_CONSTANT         = 0x38;
constexpr uint8_t MOTOR_INDUCTANCE_CONSTANT         = 0x3C;
constexpr uint8_t MOTOR_TORQUE_CONSTANT             = 0x40;
constexpr uint8_t MOTOR_MOMENT_OF_INERTIA_CONSTANT  = 0x44;
constexpr uint8_t MOTOR_INDUCTANCE_LQ_CONSTANT      = 0x48;


constexpr uint8_t MOTOR_GAIN_MODE                   = 0x50;
constexpr uint8_t POSITION_GAIN_SCALE               = 0x51;
constexpr uint8_t VELOCITY_GAIN_SCALE               = 0x52;
constexpr uint8_t CURRENT_GAIN_SCALE                = 0x53;

constexpr uint8_t POSITION_P_GAIN                   = 0x54;
constexpr uint8_t VELOCITY_P_GAIN                   = 0x58;
constexpr uint8_t VELOCITY_I_GAIN                   = 0x5C;
constexpr uint8_t CURRENT_P_GAIN                    = 0x60;
constexpr uint8_t CURRENT_I_GAIN                    = 0x64;

constexpr uint8_t POSITION_CCW_MAX                  = 0x70;
constexpr uint8_t POSITION_CW_MAX                   = 0x74;
constexpr uint8_t VELOCITY_CCW_MAX                  = 0x78;
constexpr uint8_t VELOCITY_CW_MAX                   = 0x7B;
constexpr uint8_t IQ_CURRENT_CCW_MAX                = 0x7E;
constexpr uint8_t IQ_CURRENT_CW_MAX                 = 0x80;
constexpr uint8_t ID_CURRENT_MAX                    = 0x82;
constexpr uint8_t IQ_CURRENT_LIMIT                  = 0x84;
constexpr uint8_t BUS_VOLTAGE_MAX_LIMIT          = 0x86;
constexpr uint8_t BUS_VOLTAGE_MIN_LIMIT          = 0x88;
constexpr uint8_t TEMPERATURE_MAX_LIMIT             = 0x8A;

constexpr uint8_t MOTOR_ACCTIME                     = 0x90;
constexpr uint8_t MOTOR_DECTIME                     = 0x92;
constexpr uint8_t MOTOR_MOTION_MODE                 = 0x94;
constexpr uint8_t MOTOR_SUB_TARGET                  = 0x95;
constexpr uint8_t MOTOR_MAIN_TARGET                 = 0x98;
constexpr uint8_t MOTOR_DIRECTION                   = 0x9B;
constexpr uint8_t MOTOR_MOTION_PROFILE_RUNMODE      = 0x9C;
constexpr uint8_t MOTOR_ENABLE                      = 0x9D;
constexpr uint8_t MOTOR_BRAKE                       = 0x9E;
constexpr uint8_t MOTOR_EXTERNAL_BRAKE              = 0x9F;
constexpr uint8_t BRAKE_AUTO_VOLTAGE_MODE           = 0xA0;
constexpr uint8_t MOTOR_POSITION_ZERO_SET           = 0xA1;


constexpr uint8_t MOTION_PROFILE_STATE              = 0xA7;
constexpr uint8_t MOTION_MAIN_TARGET                = 0xA8;
constexpr uint8_t MOTION_SUB_TARGET                 = 0xAC;

constexpr uint8_t TOTAL_TIME_NOW                    = 0xB0;
constexpr uint8_t ELAPSED_TIME_LEFT_NOW             = 0xB4;
constexpr uint8_t REMAIN_TIME_NOW                   = 0xB8;
constexpr uint8_t POSITION_NOW                      = 0xBC;
constexpr uint8_t POSITION_DIR                      = 0xC0;
constexpr uint8_t VELOCITY_NOW                      = 0xC1;
constexpr uint8_t VELOCITY_DIR                      = 0xC4;
constexpr uint8_t IQ_CURRENT_NOW                    = 0xC5;
constexpr uint8_t IQ_CURRENT_DIR                    = 0xC7;
constexpr uint8_t ID_CURRENT_NOW                    = 0xC8;
constexpr uint8_t ID_CURRENT_DIR                    = 0xCA;
constexpr uint8_t BUS_CURRENT_NOW                   = 0xCB;
constexpr uint8_t VQ_VOLTAGE_NOW                    = 0xCD;
constexpr uint8_t VQ_VOLTAGE_DIR                    = 0xCF;
constexpr uint8_t VD_VOLTAGE_NOW                    = 0xD0;
constexpr uint8_t VD_VOLTAGE_DIR                    = 0xD2;
constexpr uint8_t BUS_VOLTAGE_NOW                   = 0xD3;
constexpr uint8_t TEMPERATURE_NOW                   = 0xD5;

constexpr uint8_t MOTOR_PULSECOUNT_NOW              = 0xD7;
constexpr uint8_t MOTOR_PULSECOUNT_NOW_DIR          = 0xDF;

constexpr uint8_t U_PHASE_CURRENT_NOW               = 0xE0;
constexpr uint8_t U_PHASE_CURRENT_DIR               = 0xE2;
constexpr uint8_t W_PHASE_CURRENT_NOW               = 0xE3;
constexpr uint8_t W_PHASE_CURRENT_DIR               = 0xE5;

constexpr uint8_t MOTOR_U_PHASE_VOLTAGE_NOW         = 0xE6;
constexpr uint8_t MOTOR_V_PHASE_VOLTAGE_NOW         = 0xE8;
constexpr uint8_t MOTOR_W_PHASE_VOLTAGE_NOW         = 0xEA;
constexpr uint8_t MOTOR_X_PHASE_VOLTAGE_NOW         = 0xEC;

constexpr uint8_t SENSOR1_SETTING                   = 0xAA;
constexpr uint8_t SENSOR2_SETTING                   = 0xAB;

constexpr uint8_t FIRMWARE_VERSION                  = 0xF7;
constexpr uint8_t PROTOCOL_VERSION                  = 0xFA;
}


// =========================================================
// INTEGRATED PLATFORM ADAPTERS
// 기본 baudrate는 9600이며, 사용자는 숫자 값만 넣으면 됩니다.
// =========================================================

#if defined(ARDUINO)

class ArduinoTransport : public ITransport
{
public:
    explicit ArduinoTransport(HardwareSerial& serial, uint32_t baudrate = 9600);
#if defined(WOODRIVE_HAS_SOFTWARESERIAL)
    explicit ArduinoTransport(SoftwareSerial& serial, uint32_t baudrate = 9600);
#endif

    void begin();
    uint32_t baudrate() const;
    void setBaudrate(uint32_t baudrate);

    size_t write(const uint8_t* data, size_t len) override;
    int available() override;
    int read() override;
    void flush() override;
    void clearRx() override;

private:
    Stream* _serial;
    void (*_beginFunc)(Stream*, uint32_t);
    uint32_t _baudrate;
};

class ArduinoClock : public IClock
{
public:
    uint32_t nowMs() const override;
};

#endif // ARDUINO

#if defined(__linux__) || defined(__APPLE__)

class PosixSerialTransport : public ITransport
{
public:
    explicit PosixSerialTransport(const char* portName, int baudrate = 9600);
    ~PosixSerialTransport();

    bool isOpen() const;
    int fd() const;
    int baudrate() const;

    size_t write(const uint8_t* data, size_t len) override;
    int available() override;
    int read() override;
    void flush() override;
    void clearRx() override;

private:
    int _fd;
    int _baudrate;
};

class StdClock : public IClock
{
public:
    StdClock();
    uint32_t nowMs() const override;

private:
    uint64_t _startMs;
};

#endif // __linux__ || __APPLE__

