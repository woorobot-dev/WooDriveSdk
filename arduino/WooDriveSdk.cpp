#include "WooDriveSdk.h"

using WooProtocol::Packet;
using WooProtocol::ResponseFrame;
using namespace WooProtocol;


WooDrive::WooDrive(ITransport& transport, IClock& clock)
    : _transport(transport), _clock(clock), _timeoutMs(1000)
{
}

void WooDrive::setTimeout(uint32_t timeoutMs) { _timeoutMs = timeoutMs; }
uint32_t WooDrive::timeout() const { return _timeoutMs; }

uint8_t WooDrive::beU8(const uint8_t* d)  { return d[0]; }
uint16_t WooDrive::beU16(const uint8_t* d){ return ((uint16_t)d[0] << 8) | d[1]; }
uint32_t WooDrive::beU24(const uint8_t* d){ return ((uint32_t)d[0] << 16) | ((uint32_t)d[1] << 8) | d[2]; }
uint32_t WooDrive::beU32(const uint8_t* d){ return ((uint32_t)d[0] << 24) | ((uint32_t)d[1] << 16) | ((uint32_t)d[2] << 8) | d[3]; }
uint64_t WooDrive::beU64(const uint8_t* d){ return ((uint64_t)d[0] << 56) | ((uint64_t)d[1] << 48) | ((uint64_t)d[2] << 40) | ((uint64_t)d[3] << 32) | ((uint64_t)d[4] << 24) | ((uint64_t)d[5] << 16) | ((uint64_t)d[6] << 8) | (uint64_t)d[7]; }
int32_t WooDrive::beS24(const uint8_t* d) { uint32_t u = beU24(d); if (u & 0x00800000UL) u |= 0xFF000000UL; return (int32_t)u; }
int32_t WooDrive::beS32(const uint8_t* d) { return (int32_t)beU32(d); }
float WooDrive::fixed2u16(const uint8_t* d){ return (float)beU16(d) / 100.0f; }
float WooDrive::fixed2u24(const uint8_t* d){ return (float)beU24(d) / 100.0f; }
float WooDrive::fixed2s16(const uint8_t* d){ return (float)(int16_t)beU16(d) / 100.0f; }
float WooDrive::fixed2s32(const uint8_t* d){ return (float)beS32(d) / 100.0f; }
float WooDrive::fixed6u32(const uint8_t* d){ return (float)beU32(d) / 1000000.0f; }

static uint32_t resistanceToRaw(float value, ResistanceUnit unit)
{
    switch (unit) {
        case ResistanceUnit::uOhm: return (uint32_t)(value + 0.5f);
        case ResistanceUnit::mOhm: return (uint32_t)(value * 1000.0f + 0.5f);
        case ResistanceUnit::Ohm:  return (uint32_t)(value * 1000000.0f + 0.5f);
        default: return 0;
    }
}

static float rawToResistance(uint32_t raw, ResistanceUnit unit)
{
    switch (unit) {
        case ResistanceUnit::uOhm: return (float)raw;
        case ResistanceUnit::mOhm: return (float)raw / 1000.0f;
        case ResistanceUnit::Ohm:  return (float)raw / 1000000.0f;
        default: return 0.0f;
    }
}

static uint32_t inductanceToRaw(float value, InductanceUnit unit)
{
    switch (unit) {
        case InductanceUnit::nH: return (uint32_t)(value + 0.5f);
        case InductanceUnit::uH: return (uint32_t)(value * 1000.0f + 0.5f);
        case InductanceUnit::mH: return (uint32_t)(value * 1000000.0f + 0.5f);
        case InductanceUnit::H:  return (uint32_t)(value * 1000000000.0f + 0.5f);
        default: return 0;
    }
}

static float rawToInductance(uint32_t raw, InductanceUnit unit)
{
    switch (unit) {
        case InductanceUnit::nH: return (float)raw;
        case InductanceUnit::uH: return (float)raw / 1000.0f;
        case InductanceUnit::mH: return (float)raw / 1000000.0f;
        case InductanceUnit::H:  return (float)raw / 1000000000.0f;
        default: return 0.0f;
    }
}

static uint32_t torqueConstantToRaw(float value, TorqueConstantUnit unit)
{
    switch (unit) {
        case TorqueConstantUnit::mNmPerA: return (uint32_t)(value + 0.5f);
        case TorqueConstantUnit::NmPerA:  return (uint32_t)(value * 1000.0f + 0.5f);
        default: return 0;
    }
}

static float rawToTorqueConstant(uint32_t raw, TorqueConstantUnit unit)
{
    switch (unit) {
        case TorqueConstantUnit::mNmPerA: return (float)raw;
        case TorqueConstantUnit::NmPerA:  return (float)raw / 1000.0f;
        default: return 0.0f;
    }
}

static float inertiaUnitToGcm2(float value, InertiaUnit unit)
{
    switch (unit) {
        case InertiaUnit::g_mm2:  return value / 100.0f;
        case InertiaUnit::g_cm2:  return value;
        case InertiaUnit::g_m2:   return value * 10000.0f;
        case InertiaUnit::kg_mm2: return value * 10.0f;
        case InertiaUnit::kg_cm2: return value * 1000.0f;
        case InertiaUnit::kg_m2:  return value * 10000000.0f;
        default: return 0.0f;
    }
}

static float gcm2ToInertiaUnit(float gcm2, InertiaUnit unit)
{
    switch (unit) {
        case InertiaUnit::g_mm2:  return gcm2 * 100.0f;
        case InertiaUnit::g_cm2:  return gcm2;
        case InertiaUnit::g_m2:   return gcm2 / 10000.0f;
        case InertiaUnit::kg_mm2: return gcm2 / 10.0f;
        case InertiaUnit::kg_cm2: return gcm2 / 1000.0f;
        case InertiaUnit::kg_m2:  return gcm2 / 10000000.0f;
        default: return 0.0f;
    }
}

static uint32_t inertiaToRaw(float value, InertiaUnit unit)
{
    float gcm2 = inertiaUnitToGcm2(value, unit);
    return (uint32_t)(gcm2 * 100.0f + 0.5f);
}

static float rawToInertia(uint32_t raw, InertiaUnit unit)
{
    float gcm2 = (float)raw / 100.0f;
    return gcm2ToInertiaUnit(gcm2, unit);
}
bool WooDrive::checkRsp(const ResponseFrame& rsp, uint8_t id, uint8_t addr, uint8_t minLen) const
{
    if (rsp.id != id) return false;
    if (rsp.dir != DIR_RSP && rsp.dir != DIR_FAULT_RSP) return false;
    if (rsp.cmd != addr) return false;
    if (rsp.dataLen < minLen) return false;
    return true;
}

bool WooDrive::makeGetU8(Packet& pkt, uint8_t id, uint8_t addr)
{ const uint8_t d[] = {1}; return makeRequestAutoLen(pkt, id, DIR_GET, addr, d, 1); }
bool WooDrive::makeGetU16(Packet& pkt, uint8_t id, uint8_t addr)
{ const uint8_t d[] = {2}; return makeRequestAutoLen(pkt, id, DIR_GET, addr, d, 1); }
bool WooDrive::makeGetU24(Packet& pkt, uint8_t id, uint8_t addr)
{ const uint8_t d[] = {3}; return makeRequestAutoLen(pkt, id, DIR_GET, addr, d, 1); }
bool WooDrive::makeGetU32(Packet& pkt, uint8_t id, uint8_t addr)
{ const uint8_t d[] = {4}; return makeRequestAutoLen(pkt, id, DIR_GET, addr, d, 1); }

bool WooDrive::makeSetU8(Packet& pkt, uint8_t id, uint8_t addr, uint8_t value)
{ const uint8_t d[] = {value}; return makeRequestAutoLen(pkt, id, DIR_SET, addr, d, 1); }
bool WooDrive::makeSetU16(Packet& pkt, uint8_t id, uint8_t addr, uint16_t value)
{ const uint8_t d[] = {(uint8_t)(value >> 8), (uint8_t)value}; return makeRequestAutoLen(pkt, id, DIR_SET, addr, d, 2); }
bool WooDrive::makeSetU24(Packet& pkt, uint8_t id, uint8_t addr, uint32_t value)
{ const uint8_t d[] = {(uint8_t)(value >> 16), (uint8_t)(value >> 8), (uint8_t)value}; return makeRequestAutoLen(pkt, id, DIR_SET, addr, d, 3); }
bool WooDrive::makeSetU32(Packet& pkt, uint8_t id, uint8_t addr, uint32_t value)
{ const uint8_t d[] = {(uint8_t)(value >> 24), (uint8_t)(value >> 16), (uint8_t)(value >> 8), (uint8_t)value}; return makeRequestAutoLen(pkt, id, DIR_SET, addr, d, 4); }
bool WooDrive::makeSetU16x100(Packet& pkt, uint8_t id, uint8_t addr, float value)
{ return makeSetU16(pkt, id, addr, (uint16_t)(value * 100.0f)); }
bool WooDrive::makeSetU24x100(Packet& pkt, uint8_t id, uint8_t addr, float value)
{ return makeSetU24(pkt, id, addr, (uint32_t)(value * 100.0f)); }
bool WooDrive::makeSetU32x1e6(Packet& pkt, uint8_t id, uint8_t addr, float value)
{ return makeSetU32(pkt, id, addr, (uint32_t)(value * 1000000.0f)); }

bool WooDrive::makeSetS32(Packet& pkt, uint8_t id, uint8_t addr, int32_t value)
{
    uint32_t raw = (uint32_t)value;
    const uint8_t d[] = {
        (uint8_t)(raw >> 24),
        (uint8_t)(raw >> 16),
        (uint8_t)(raw >> 8),
        (uint8_t)(raw)
    };
    return makeRequestAutoLen(pkt, id, DIR_SET, addr, d, 4);
}

bool WooDrive::makeGetS32(Packet& pkt, uint8_t id, uint8_t addr)
{
    const uint8_t d[] = {4};
    return makeRequestAutoLen(pkt, id, DIR_GET, addr, d, 1);
}

bool WooDrive::sendOnly(const Packet& pkt)
{
    size_t n = _transport.write(pkt.buf, pkt.len);
    _transport.flush();   // RS485면 유지
    return (n == pkt.len);
}

bool WooDrive::sendAndReceive(const Packet& pkt, ResponseFrame& rsp)
{
    _transport.clearRx();

    size_t n = _transport.write(pkt.buf, pkt.len);
    _transport.flush();
    if (n != pkt.len)
        return false;

    uint32_t start = _clock.nowMs();

    while ((_clock.nowMs() - start) < _timeoutMs) {
        ResponseFrame tmp;

        if (!readFrame(tmp)) {
            continue;
        }

        if (tmp.dir == DIR_GET || tmp.dir == DIR_SET) {
            continue;
        }

        if (tmp.dir == DIR_RSP || tmp.dir == DIR_FAULT_RSP) {
            rsp = tmp;
            return true;
        }
    }

    return false;
}

bool WooDrive::readFrame(ResponseFrame& rsp)
{
    enum State { READ_H1, READ_H2, READ_H3, READ_ID, READ_DIR, READ_CMD, READ_LEN, READ_PAYLOAD };
    State state = READ_H1;
    uint8_t raw[64];
    uint8_t idx = 0;
    uint8_t lenField = 0;
    uint8_t payloadRead = 0;
    uint32_t start = _clock.nowMs();

    while ((_clock.nowMs() - start) < _timeoutMs) {
        while (_transport.available() > 0) {
            int rv = _transport.read();
            if (rv < 0) {
                continue;
            }
            uint8_t b = (uint8_t)rv;
            switch (state) {
                case READ_H1: if (b == 0xFF) { raw[0] = b; idx = 1; state = READ_H2; } break;
                case READ_H2: if (b == 0xFF) { raw[idx++] = b; state = READ_H3; } else { idx = 0; state = READ_H1; } break;
                case READ_H3: if (b == 0xFE) { raw[idx++] = b; state = READ_ID; } else { idx = 0; state = READ_H1; } break;
                case READ_ID: raw[idx++] = b; state = READ_DIR; break;
                case READ_DIR: raw[idx++] = b; state = READ_CMD; break;
                case READ_CMD: raw[idx++] = b; state = READ_LEN; break;
                case READ_LEN:
                    raw[idx++] = b; lenField = b; payloadRead = 0;
                    if ((uint16_t)idx + lenField > sizeof(raw) || lenField < 2) { idx = 0; state = READ_H1; break; }
                    state = READ_PAYLOAD; break;
                case READ_PAYLOAD:
                    raw[idx++] = b; payloadRead++;
                    if (payloadRead >= lenField) {
                        rsp.id = raw[3]; rsp.dir = raw[4]; rsp.cmd = raw[5]; rsp.lenField = raw[6]; rsp.dataLen = rsp.lenField - 2;
                        for (uint8_t i = 0; i < rsp.dataLen && i < sizeof(rsp.data); i++) rsp.data[i] = raw[7 + i];
                        rsp.crc = (uint16_t)raw[7 + rsp.dataLen] | ((uint16_t)raw[8 + rsp.dataLen] << 8);
                        return validateFrame(rsp);
                    }
                    break;
            }
        }
    }
    return false;
}


// BOARD CONFIG
bool WooDrive::setReset(uint8_t id, uint8_t value)
{
    Packet pkt;
    return makeSetU8(pkt, id, Address::RESET, value) && sendOnly(pkt);
}

bool WooDrive::getReset(uint8_t id, uint8_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU8(pkt, id, Address::RESET) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::RESET, 1))
        return false;

    outValue = beU8(rsp.data);
    return true;
}

bool WooDrive::setId(uint8_t id, uint8_t value)
{
    Packet pkt;
    return makeSetU8(pkt, id, Address::ID, value) && sendOnly(pkt);
}

bool WooDrive::getId(uint8_t id, uint8_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU8(pkt, id, Address::ID) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::ID, 1))
        return false;

    outValue = beU8(rsp.data);
    return true;
}

bool WooDrive::getFault(uint8_t id, uint8_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU8(pkt, id, Address::FAULT) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::FAULT, 1))
        return false;

    outValue = beU8(rsp.data);
    return true;
}

bool WooDrive::setCommunicationMode(uint8_t id, uint8_t value)
{
    Packet pkt;
    return makeSetU8(pkt, id, Address::COMMUNICATION_MODE, value) && sendOnly(pkt);
}

bool WooDrive::getCommunicationMode(uint8_t id, uint8_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU8(pkt, id, Address::COMMUNICATION_MODE) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::COMMUNICATION_MODE, 1))
        return false;

    outValue = beU8(rsp.data);
    return true;
}

bool WooDrive::setBps(uint8_t id, uint32_t value)
{
    Packet pkt;
    return makeSetU32(pkt, id, Address::BAUDRATE, value) && sendOnly(pkt);
}

bool WooDrive::getBps(uint8_t id, uint32_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU32(pkt, id, Address::BAUDRATE) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::BAUDRATE, 4))
        return false;

    outValue = beU32(rsp.data);
    return true;
}

bool WooDrive::setCommunicationWatchdog(uint8_t id, uint16_t value)
{
    Packet pkt;
    return makeSetU16(pkt, id, Address::WATCHDOG, value) && sendOnly(pkt);
}

bool WooDrive::getCommunicationWatchdog(uint8_t id, uint16_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU16(pkt, id, Address::WATCHDOG) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::WATCHDOG, 2))
        return false;

    outValue = beU16(rsp.data);
    return true;
}

bool WooDrive::setMotorType(uint8_t id, uint8_t value)
{
    Packet pkt;
    return makeSetU8(pkt, id, Address::MOTOR_TYPE, value) && sendOnly(pkt);
}

bool WooDrive::getMotorType(uint8_t id, uint8_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU8(pkt, id, Address::MOTOR_TYPE) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::MOTOR_TYPE, 1))
        return false;

    outValue = beU8(rsp.data);
    return true;
}

bool WooDrive::setFeedbackType(uint8_t id, uint8_t value)
{
    Packet pkt;
    return makeSetU8(pkt, id, Address::FEEDBACK_TYPE, value) && sendOnly(pkt);
}

bool WooDrive::getFeedbackType(uint8_t id, uint8_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU8(pkt, id, Address::FEEDBACK_TYPE) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::FEEDBACK_TYPE, 1))
        return false;

    outValue = beU8(rsp.data);
    return true;
}

bool WooDrive::setStartupFeedbackType(uint8_t id, uint8_t value)
{
    Packet pkt;
    return makeSetU8(pkt, id, Address::STARTUP_FEEDBACK_TYPE, value) && sendOnly(pkt);
}

bool WooDrive::getStartupFeedbackType(uint8_t id, uint8_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU8(pkt, id, Address::STARTUP_FEEDBACK_TYPE) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::STARTUP_FEEDBACK_TYPE, 1))
        return false;

    outValue = beU8(rsp.data);
    return true;
}

bool WooDrive::setDirectionInvert(uint8_t id, uint8_t value)
{
    Packet pkt;
    return makeSetU8(pkt, id, Address::DIRECTION_INVERT, value) && sendOnly(pkt);
}

bool WooDrive::getDirectionInvert(uint8_t id, uint8_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU8(pkt, id, Address::DIRECTION_INVERT) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::DIRECTION_INVERT, 1))
        return false;

    outValue = beU8(rsp.data);
    return true;
}

bool WooDrive::setFieldWeakeningEnable(uint8_t id, uint8_t value)
{
    Packet pkt;
    return makeSetU8(pkt, id, Address::FIELD_WEAKENING_ENABLE, value) && sendOnly(pkt);
}

bool WooDrive::getFieldWeakeningEnable(uint8_t id, uint8_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU8(pkt, id, Address::FIELD_WEAKENING_ENABLE) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::FIELD_WEAKENING_ENABLE, 1))
        return false;

    outValue = beU8(rsp.data);
    return true;
}

bool WooDrive::setExternalBrakePresent(uint8_t id, uint8_t value)
{
    Packet pkt;
    return makeSetU8(pkt, id, Address::EXTERNAL_BRAKE_PRESENT, value) && sendOnly(pkt);
}

bool WooDrive::getExternalBrakePresent(uint8_t id, uint8_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU8(pkt, id, Address::EXTERNAL_BRAKE_PRESENT) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::EXTERNAL_BRAKE_PRESENT, 1))
        return false;

    outValue = beU8(rsp.data);
    return true;
}

bool WooDrive::setPolePairs(uint8_t id, uint8_t value)
{
    Packet pkt;
    return makeSetU8(pkt, id, Address::POLE_PAIRS, value) && sendOnly(pkt);
}

bool WooDrive::getPolePairs(uint8_t id, uint8_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU8(pkt, id, Address::POLE_PAIRS) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::POLE_PAIRS, 1))
        return false;

    outValue = beU8(rsp.data);
    return true;
}

bool WooDrive::setFeedbackDir(uint8_t id, uint8_t value)
{
    Packet pkt;
    return makeSetU8(pkt, id, Address::FEEDBACK_DIR, value) && sendOnly(pkt);
}

bool WooDrive::getFeedbackDir(uint8_t id, uint8_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU8(pkt, id, Address::FEEDBACK_DIR) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::FEEDBACK_DIR, 1))
        return false;

    outValue = beU8(rsp.data);
    return true;
}

bool WooDrive::setFeedbackResolution(uint8_t id, uint32_t value)
{
    Packet pkt;
    return makeSetU32(pkt, id, Address::FEEDBACK_RESOLUTION, value) && sendOnly(pkt);
}

bool WooDrive::getFeedbackResolution(uint8_t id, uint32_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU32(pkt, id, Address::FEEDBACK_RESOLUTION) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::FEEDBACK_RESOLUTION, 4))
        return false;

    outValue = beU32(rsp.data);
    return true;
}

bool WooDrive::setGear(uint8_t id, float value)
{
    Packet pkt;
    uint32_t raw = (uint32_t)(value * 1000000.0f + 0.5f);
    return makeSetU32(pkt, id, Address::GEAR, raw) && sendOnly(pkt);
}

bool WooDrive::getGear(uint8_t id, float& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU32(pkt, id, Address::GEAR) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::GEAR, 4))
        return false;

    outValue = (float)beU32(rsp.data) / 1000000.0f;
    return true;
}

bool WooDrive::setAutoMotorSetup(uint8_t id, uint8_t value)
{
    Packet pkt;
    return makeSetU8(pkt, id, Address::AUTO_MOTOR_SETUP, value) && sendOnly(pkt);
}

bool WooDrive::setGainMode(uint8_t id, uint8_t value)
{
    Packet pkt;
    return makeSetU8(pkt, id, Address::MOTOR_GAIN_MODE, value) && sendOnly(pkt);
}
bool WooDrive::getAutoMotorSetup(uint8_t id, uint8_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU8(pkt, id, Address::AUTO_MOTOR_SETUP) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::AUTO_MOTOR_SETUP, 1))
        return false;

    outValue = beU8(rsp.data);
    return true;
}

bool WooDrive::setAlignmentMode(uint8_t id, uint8_t value)
{
    Packet pkt;
    return makeSetU8(pkt, id, Address::ALIGNMENT_MODE, value) && sendOnly(pkt);
}

bool WooDrive::getAlignmentMode(uint8_t id, uint8_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU8(pkt, id, Address::ALIGNMENT_MODE) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::ALIGNMENT_MODE, 1))
        return false;

    outValue = beU8(rsp.data);
    return true;
}

bool WooDrive::setPhaseOffset(uint8_t id, uint32_t value)
{
    Packet pkt;
    return makeSetU32(pkt, id, Address::PHASE_OFFSET, value) && sendOnly(pkt);
}

bool WooDrive::getPhaseOffset(uint8_t id, uint32_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU32(pkt, id, Address::PHASE_OFFSET) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::PHASE_OFFSET, 4))
        return false;

    outValue = beU32(rsp.data);
    return true;
}

bool WooDrive::setForceAngleLevel(uint8_t id, uint8_t value)
{
    Packet pkt;
    return makeSetU8(pkt, id, Address::FORCE_ANGLE_LEVEL, value) && sendOnly(pkt);
}

bool WooDrive::getForceAngleLevel(uint8_t id, uint8_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU8(pkt, id, Address::FORCE_ANGLE_LEVEL) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::FORCE_ANGLE_LEVEL, 1))
        return false;

    outValue = beU8(rsp.data);
    return true;
}

bool WooDrive::setHallSensorSetting(uint8_t id, uint8_t value)
{
    Packet pkt;
    return makeSetU8(pkt, id, Address::HALLSENSOR_SETTING, value) && sendOnly(pkt);
}

bool WooDrive::getHallSensorSetting(uint8_t id, uint8_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU8(pkt, id, Address::HALLSENSOR_SETTING) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::HALLSENSOR_SETTING, 1))
        return false;

    outValue = beU8(rsp.data);
    return true;
}

bool WooDrive::getMainPortLevelNow(uint8_t id, uint8_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU8(pkt, id, Address::MAINPORT_LEVEL_NOW) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::MAINPORT_LEVEL_NOW, 1))
        return false;

    outValue = beU8(rsp.data);
    return true;
}

bool WooDrive::getSubPortLevelNow(uint8_t id, uint8_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU8(pkt, id, Address::SUBPORT_LEVEL_NOW) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::SUBPORT_LEVEL_NOW, 1))
        return false;

    outValue = beU8(rsp.data);
    return true;
}

// MOTOR PARAM
bool WooDrive::setAutoParameter(uint8_t id, uint8_t value)
{
    Packet pkt;
    return makeSetU8(pkt, id, Address::MOTOR_AUTO_PARAMETER_MEASUREMENT_MODE, value) && sendOnly(pkt);
}

bool WooDrive::getAutoParameter(uint8_t id, uint8_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU8(pkt, id, Address::MOTOR_AUTO_PARAMETER_MEASUREMENT_MODE) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::MOTOR_AUTO_PARAMETER_MEASUREMENT_MODE, 1))
        return false;

    outValue = beU8(rsp.data);
    return true;
}

bool WooDrive::setRatedSpeed(uint8_t id, float value)
{
    Packet pkt;
    uint32_t raw = (uint32_t)(value * 100.0f + 0.5f);
    return makeSetU24(pkt, id, Address::MOTOR_RATED_SPEED, raw) && sendOnly(pkt);
}

bool WooDrive::setRatedSpeedRaw(uint8_t id, uint32_t value)
{
    Packet pkt;
    return makeSetU24(pkt, id, Address::MOTOR_RATED_SPEED, value) && sendOnly(pkt);
}

bool WooDrive::getRatedSpeedRaw(uint8_t id, uint32_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU24(pkt, id, Address::MOTOR_RATED_SPEED) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::MOTOR_RATED_SPEED, 3))
        return false;

    outValue = beU24(rsp.data);
    return true;
}

bool WooDrive::getRatedSpeed(uint8_t id, float& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU24(pkt, id, Address::MOTOR_RATED_SPEED) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::MOTOR_RATED_SPEED, 3))
        return false;

    outValue = fixed2u24(rsp.data);
    return true;
}

bool WooDrive::setRatedCurrent(uint8_t id, float value)
{
    Packet pkt;
    uint16_t raw = (uint16_t)(value * 100.0f + 0.5f);
    return makeSetU16(pkt, id, Address::MOTOR_RATED_CURRENT, raw) && sendOnly(pkt);
}

bool WooDrive::setRatedCurrentRaw(uint8_t id, uint16_t value)
{
    Packet pkt;
    return makeSetU16(pkt, id, Address::MOTOR_RATED_CURRENT, value) && sendOnly(pkt);
}

bool WooDrive::getRatedCurrentRaw(uint8_t id, uint16_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU16(pkt, id, Address::MOTOR_RATED_CURRENT) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::MOTOR_RATED_CURRENT, 2))
        return false;

    outValue = beU16(rsp.data);
    return true;
}

bool WooDrive::getRatedCurrent(uint8_t id, float& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU16(pkt, id, Address::MOTOR_RATED_CURRENT) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::MOTOR_RATED_CURRENT, 2))
        return false;

    outValue = fixed2u16(rsp.data);
    return true;
}

bool WooDrive::setRatedVoltage(uint8_t id, float value)
{
    Packet pkt;
    uint16_t raw = (uint16_t)(value * 100.0f + 0.5f);
    return makeSetU16(pkt, id, Address::MOTOR_RATED_VOLTAGE, raw) && sendOnly(pkt);
}

bool WooDrive::setRatedVoltageRaw(uint8_t id, uint16_t value)
{
    Packet pkt;
    return makeSetU16(pkt, id, Address::MOTOR_RATED_VOLTAGE, value) && sendOnly(pkt);
}

bool WooDrive::getRatedVoltageRaw(uint8_t id, uint16_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU16(pkt, id, Address::MOTOR_RATED_VOLTAGE) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::MOTOR_RATED_VOLTAGE, 2))
        return false;

    outValue = beU16(rsp.data);
    return true;
}

bool WooDrive::getRatedVoltage(uint8_t id, float& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU16(pkt, id, Address::MOTOR_RATED_VOLTAGE) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::MOTOR_RATED_VOLTAGE, 2))
        return false;

    outValue = fixed2u16(rsp.data);
    return true;
}

bool WooDrive::setResistanceConstant(uint8_t id, float value)
{
    return setResistanceConstant(id, value, ResistanceUnit::mOhm);
}

bool WooDrive::setResistanceConstant(uint8_t id, float value, ResistanceUnit unit)
{
    Packet pkt;
    uint32_t raw = resistanceToRaw(value, unit);
    return makeSetU32(pkt, id, Address::MOTOR_RESISTANCE_CONSTANT, raw) && sendOnly(pkt);
}

bool WooDrive::getResistanceConstantRaw(uint8_t id, uint32_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU32(pkt, id, Address::MOTOR_RESISTANCE_CONSTANT) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::MOTOR_RESISTANCE_CONSTANT, 4))
        return false;

    outValue = beU32(rsp.data);
    return true;
}

bool WooDrive::getResistanceConstant(uint8_t id, float& outValue)
{
    return getResistanceConstant(id, outValue, ResistanceUnit::mOhm);
}

bool WooDrive::getResistanceConstant(uint8_t id, float& outValue, ResistanceUnit unit)
{
    uint32_t raw;
    if (!getResistanceConstantRaw(id, raw))
        return false;

    outValue = rawToResistance(raw, unit);
    return true;
}

bool WooDrive::setInductanceConstant(uint8_t id, float value)
{
    return setInductanceConstant(id, value, InductanceUnit::mH);
}

bool WooDrive::setInductanceConstant(uint8_t id, float value, InductanceUnit unit)
{
    Packet pkt;
    uint32_t raw = inductanceToRaw(value, unit);
    return makeSetU32(pkt, id, Address::MOTOR_INDUCTANCE_CONSTANT, raw) && sendOnly(pkt);
}

bool WooDrive::getInductanceConstantRaw(uint8_t id, uint32_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU32(pkt, id, Address::MOTOR_INDUCTANCE_CONSTANT) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::MOTOR_INDUCTANCE_CONSTANT, 4))
        return false;

    outValue = beU32(rsp.data);
    return true;
}

bool WooDrive::getInductanceConstant(uint8_t id, float& outValue)
{
    return getInductanceConstant(id, outValue, InductanceUnit::mH);
}

bool WooDrive::getInductanceConstant(uint8_t id, float& outValue, InductanceUnit unit)
{
    uint32_t raw;
    if (!getInductanceConstantRaw(id, raw))
        return false;

    outValue = rawToInductance(raw, unit);
    return true;
}

bool WooDrive::setTorqueConstant(uint8_t id, float value)
{
    return setTorqueConstant(id, value, TorqueConstantUnit::mNmPerA);
}

bool WooDrive::setTorqueConstant(uint8_t id, float value, TorqueConstantUnit unit)
{
    Packet pkt;
    uint32_t raw = torqueConstantToRaw(value, unit);
    return makeSetU32(pkt, id, Address::MOTOR_TORQUE_CONSTANT, raw) && sendOnly(pkt);
}

bool WooDrive::getTorqueConstantRaw(uint8_t id, uint32_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU32(pkt, id, Address::MOTOR_TORQUE_CONSTANT) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::MOTOR_TORQUE_CONSTANT, 4))
        return false;

    outValue = beU32(rsp.data);
    return true;
}

bool WooDrive::getTorqueConstant(uint8_t id, float& outValue)
{
    return getTorqueConstant(id, outValue, TorqueConstantUnit::mNmPerA);
}

bool WooDrive::getTorqueConstant(uint8_t id, float& outValue, TorqueConstantUnit unit)
{
    uint32_t raw;
    if (!getTorqueConstantRaw(id, raw))
        return false;

    outValue = rawToTorqueConstant(raw, unit);
    return true;
}

bool WooDrive::setMomentOfInertia(uint8_t id, float value)
{
    return setMomentOfInertia(id, value, InertiaUnit::g_cm2);
}

bool WooDrive::setMomentOfInertia(uint8_t id, float value, InertiaUnit unit)
{
    Packet pkt;
    uint32_t raw = inertiaToRaw(value, unit);
    return makeSetU32(pkt, id, Address::MOTOR_MOMENT_OF_INERTIA_CONSTANT, raw) && sendOnly(pkt);
}

bool WooDrive::getMomentOfInertiaRaw(uint8_t id, uint32_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU32(pkt, id, Address::MOTOR_MOMENT_OF_INERTIA_CONSTANT) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::MOTOR_MOMENT_OF_INERTIA_CONSTANT, 4))
        return false;

    outValue = beU32(rsp.data);
    return true;
}

bool WooDrive::getMomentOfInertia(uint8_t id, float& outValue)
{
    return getMomentOfInertia(id, outValue, InertiaUnit::g_cm2);
}

bool WooDrive::getMomentOfInertia(uint8_t id, float& outValue, InertiaUnit unit)
{
    uint32_t raw;
    if (!getMomentOfInertiaRaw(id, raw))
        return false;

    outValue = rawToInertia(raw, unit);
    return true;
}

bool WooDrive::getGainMode(uint8_t id, uint8_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU8(pkt, id, Address::MOTOR_GAIN_MODE) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::MOTOR_GAIN_MODE, 1))
        return false;

    outValue = beU8(rsp.data);
    return true;
}

bool WooDrive::setPositionGainScale(uint8_t id, uint8_t value)
{
    Packet pkt;
    return makeSetU8(pkt, id, Address::POSITION_GAIN_SCALE, value) && sendOnly(pkt);
}

bool WooDrive::getPositionGainScale(uint8_t id, uint8_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU8(pkt, id, Address::POSITION_GAIN_SCALE) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::POSITION_GAIN_SCALE, 1))
        return false;

    outValue = beU8(rsp.data);
    return true;
}

bool WooDrive::setVelocityGainScale(uint8_t id, uint8_t value)
{
    Packet pkt;
    return makeSetU8(pkt, id, Address::VELOCITY_GAIN_SCALE, value) && sendOnly(pkt);
}

bool WooDrive::getVelocityGainScale(uint8_t id, uint8_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU8(pkt, id, Address::VELOCITY_GAIN_SCALE) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::VELOCITY_GAIN_SCALE, 1))
        return false;

    outValue = beU8(rsp.data);
    return true;
}

bool WooDrive::setCurrentGainScale(uint8_t id, uint8_t value)
{
    Packet pkt;
    return makeSetU8(pkt, id, Address::CURRENT_GAIN_SCALE, value) && sendOnly(pkt);
}

bool WooDrive::getCurrentGainScale(uint8_t id, uint8_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU8(pkt, id, Address::CURRENT_GAIN_SCALE) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::CURRENT_GAIN_SCALE, 1))
        return false;

    outValue = beU8(rsp.data);
    return true;
}

bool WooDrive::setPositionPGain(uint8_t id, uint32_t value)
{
    Packet pkt;
    return makeSetU32(pkt, id, Address::POSITION_P_GAIN, value) && sendOnly(pkt);
}

bool WooDrive::getPositionPGain(uint8_t id, uint32_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU32(pkt, id, Address::POSITION_P_GAIN) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::POSITION_P_GAIN, 4))
        return false;

    outValue = beU32(rsp.data);
    return true;
}

bool WooDrive::setVelocityPGain(uint8_t id, uint32_t value)
{
    Packet pkt;
    return makeSetU32(pkt, id, Address::VELOCITY_P_GAIN, value) && sendOnly(pkt);
}

bool WooDrive::getVelocityPGain(uint8_t id, uint32_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU32(pkt, id, Address::VELOCITY_P_GAIN) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::VELOCITY_P_GAIN, 4))
        return false;

    outValue = beU32(rsp.data);
    return true;
}

bool WooDrive::setVelocityIGain(uint8_t id, uint32_t value)
{
    Packet pkt;
    return makeSetU32(pkt, id, Address::VELOCITY_I_GAIN, value) && sendOnly(pkt);
}

bool WooDrive::getVelocityIGain(uint8_t id, uint32_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU32(pkt, id, Address::VELOCITY_I_GAIN) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::VELOCITY_I_GAIN, 4))
        return false;

    outValue = beU32(rsp.data);
    return true;
}

bool WooDrive::setCurrentPGain(uint8_t id, uint32_t value)
{
    Packet pkt;
    return makeSetU32(pkt, id, Address::CURRENT_P_GAIN, value) && sendOnly(pkt);
}

bool WooDrive::getCurrentPGain(uint8_t id, uint32_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU32(pkt, id, Address::CURRENT_P_GAIN) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::CURRENT_P_GAIN, 4))
        return false;

    outValue = beU32(rsp.data);
    return true;
}

bool WooDrive::setCurrentIGain(uint8_t id, uint32_t value)
{
    Packet pkt;
    return makeSetU32(pkt, id, Address::CURRENT_I_GAIN, value) && sendOnly(pkt);
}

bool WooDrive::getCurrentIGain(uint8_t id, uint32_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU32(pkt, id, Address::CURRENT_I_GAIN) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::CURRENT_I_GAIN, 4))
        return false;

    outValue = beU32(rsp.data);
    return true;
}

// =========================================================
// MOTOR LIMIT
// =========================================================
bool WooDrive::setPositionCcwMax(uint8_t id, float value)
{
    Packet pkt;
    uint32_t raw = (uint32_t)(value * 100.0f + 0.5f);
    return makeSetU32(pkt, id, Address::POSITION_CCW_MAX, raw) && sendOnly(pkt);
}

bool WooDrive::getPositionCcwMax(uint8_t id, float& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU32(pkt, id, Address::POSITION_CCW_MAX) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::POSITION_CCW_MAX, 4))
        return false;

    outValue = (float)beU32(rsp.data) / 100.0f;
    return true;
}

bool WooDrive::setPositionCwMax(uint8_t id, float value)
{
    Packet pkt;
    uint32_t raw = (uint32_t)(value * 100.0f + 0.5f);
    return makeSetU32(pkt, id, Address::POSITION_CW_MAX, raw) && sendOnly(pkt);
}

bool WooDrive::getPositionCwMax(uint8_t id, float& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU32(pkt, id, Address::POSITION_CW_MAX) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::POSITION_CW_MAX, 4))
        return false;

    outValue = (float)beU32(rsp.data) / 100.0f;
    return true;
}

bool WooDrive::setVelocityCcwMax(uint8_t id, float value)
{
    Packet pkt;
    return makeSetU24x100(pkt, id, Address::VELOCITY_CCW_MAX, value) && sendOnly(pkt);
}

bool WooDrive::getVelocityCcwMax(uint8_t id, float& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU24(pkt, id, Address::VELOCITY_CCW_MAX) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::VELOCITY_CCW_MAX, 3))
        return false;

    outValue = fixed2u24(rsp.data);
    return true;
}

bool WooDrive::setVelocityCwMax(uint8_t id, float value)
{
    Packet pkt;
    return makeSetU24x100(pkt, id, Address::VELOCITY_CW_MAX, value) && sendOnly(pkt);
}

bool WooDrive::getVelocityCwMax(uint8_t id, float& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU24(pkt, id, Address::VELOCITY_CW_MAX) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::VELOCITY_CW_MAX, 3))
        return false;

    outValue = fixed2u24(rsp.data);
    return true;
}

bool WooDrive::setIqCurrentCcwMax(uint8_t id, float value)
{
    Packet pkt;
    return makeSetU16x100(pkt, id, Address::IQ_CURRENT_CCW_MAX, value) && sendOnly(pkt);
}

bool WooDrive::getIqCurrentCcwMax(uint8_t id, float& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU16(pkt, id, Address::IQ_CURRENT_CCW_MAX) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::IQ_CURRENT_CCW_MAX, 2))
        return false;

    outValue = fixed2u16(rsp.data);
    return true;
}

bool WooDrive::setIqCurrentCwMax(uint8_t id, float value)
{
    Packet pkt;
    return makeSetU16x100(pkt, id, Address::IQ_CURRENT_CW_MAX, value) && sendOnly(pkt);
}

bool WooDrive::getIqCurrentCwMax(uint8_t id, float& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU16(pkt, id, Address::IQ_CURRENT_CW_MAX) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::IQ_CURRENT_CW_MAX, 2))
        return false;

    outValue = fixed2u16(rsp.data);
    return true;
}

bool WooDrive::setIdCurrentMax(uint8_t id, float value)
{
    Packet pkt;
    return makeSetU16x100(pkt, id, Address::ID_CURRENT_MAX, value) && sendOnly(pkt);
}

bool WooDrive::getIdCurrentMax(uint8_t id, float& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU16(pkt, id, Address::ID_CURRENT_MAX) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::ID_CURRENT_MAX, 2))
        return false;

    outValue = fixed2u16(rsp.data);
    return true;
}

bool WooDrive::setIqCurrentLimit(uint8_t id, float value)
{
    Packet pkt;
    return makeSetU16x100(pkt, id, Address::IQ_CURRENT_LIMIT, value) && sendOnly(pkt);
}

bool WooDrive::getIqCurrentLimit(uint8_t id, float& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU16(pkt, id, Address::IQ_CURRENT_LIMIT) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::IQ_CURRENT_LIMIT, 2))
        return false;

    outValue = fixed2u16(rsp.data);
    return true;
}

bool WooDrive::setBusVoltageMaxLimit(uint8_t id, float value)
{
    Packet pkt;
    return makeSetU16x100(pkt, id, Address::BUS_VOLTAGE_MAX_LIMIT, value) && sendOnly(pkt);
}

bool WooDrive::getBusVoltageMaxLimit(uint8_t id, float& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU16(pkt, id, Address::BUS_VOLTAGE_MAX_LIMIT) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::BUS_VOLTAGE_MAX_LIMIT, 2))
        return false;

    outValue = fixed2u16(rsp.data);
    return true;
}

bool WooDrive::setBusVoltageMinLimit(uint8_t id, float value)
{
    Packet pkt;
    return makeSetU16x100(pkt, id, Address::BUS_VOLTAGE_MIN_LIMIT, value) && sendOnly(pkt);
}

bool WooDrive::getBusVoltageMinLimit(uint8_t id, float& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU16(pkt, id, Address::BUS_VOLTAGE_MIN_LIMIT) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::BUS_VOLTAGE_MIN_LIMIT, 2))
        return false;

    outValue = fixed2u16(rsp.data);
    return true;
}

bool WooDrive::setTemperatureMaxLimit(uint8_t id, float value)
{
    Packet pkt;
    return makeSetU16x100(pkt, id, Address::TEMPERATURE_MAX_LIMIT, value) && sendOnly(pkt);
}

bool WooDrive::getTemperatureMaxLimit(uint8_t id, float& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU16(pkt, id, Address::TEMPERATURE_MAX_LIMIT) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::TEMPERATURE_MAX_LIMIT, 2))
        return false;

    outValue = fixed2u16(rsp.data);
    return true;
}

// MOTOR CONTROL
bool WooDrive::setMotorAccelTime(uint8_t targetId, uint16_t value)
{
    Packet pkt;
    return makeSetU16(pkt, targetId, Address::MOTOR_ACCTIME, value) && sendOnly(pkt);
}

bool WooDrive::getMotorAccelTime(uint8_t targetId, uint16_t& outValue)
{
    WooProtocol::Packet pkt;
    WooProtocol::ResponseFrame rsp;

    if (!makeGetU16(pkt, targetId, Address::MOTOR_ACCTIME) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, targetId, Address::MOTOR_ACCTIME, 2))
        return false;

    outValue = beU16(rsp.data);
    return true;
}

bool WooDrive::setMotorDecelTime(uint8_t targetId, uint16_t value)
{
    Packet pkt;
    return makeSetU16(pkt, targetId, Address::MOTOR_DECTIME, value) && sendOnly(pkt);
}

bool WooDrive::getMotorDecelTime(uint8_t targetId, uint16_t& outValue)
{
    WooProtocol::Packet pkt;
    WooProtocol::ResponseFrame rsp;

    if (!makeGetU16(pkt, targetId, Address::MOTOR_DECTIME) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, targetId, Address::MOTOR_DECTIME, 2))
        return false;

    outValue = beU16(rsp.data);
    return true;
}

bool WooDrive::setMotorMotionMode(uint8_t targetId, uint8_t value)
{
    Packet pkt;
    return makeSetU8(pkt, targetId, Address::MOTOR_MOTION_MODE, value) && sendOnly(pkt);
}

bool WooDrive::getMotorMotionMode(uint8_t targetId, uint8_t& outValue)
{
    WooProtocol::Packet pkt;
    WooProtocol::ResponseFrame rsp;

    if (!makeGetU8(pkt, targetId, Address::MOTOR_MOTION_MODE) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, targetId, Address::MOTOR_MOTION_MODE, 1))
        return false;

    outValue = rsp.data[0];
    return true;
}

bool WooDrive::setMotorSubTarget(uint8_t targetId, float value)
{
    Packet pkt;
    if (value < 0.0f) value = 0.0f;
    uint32_t raw = (uint32_t)(value * 100.0f + 0.5f);
    return makeSetU24(pkt, targetId, Address::MOTOR_SUB_TARGET, raw) && sendOnly(pkt);
}

bool WooDrive::getMotorSubTarget(uint8_t targetId, float& outValue)
{
    WooProtocol::Packet pkt;
    WooProtocol::ResponseFrame rsp;

    if (!makeGetU24(pkt, targetId, Address::MOTOR_SUB_TARGET) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, targetId, Address::MOTOR_SUB_TARGET, 3))
        return false;

    outValue = fixed2u24(rsp.data);
    return true;
}

bool WooDrive::setMotorMainTarget(uint8_t targetId, float value)
{
    Packet pkt;
    if (value < 0.0f) value = 0.0f;
    uint32_t raw = (uint32_t)(value * 100.0f + 0.5f);
    return makeSetU24(pkt, targetId, Address::MOTOR_MAIN_TARGET, raw) && sendOnly(pkt);
}

bool WooDrive::getMotorMainTarget(uint8_t targetId, float& outValue)
{
    WooProtocol::Packet pkt;
    WooProtocol::ResponseFrame rsp;

    if (!makeGetU24(pkt, targetId, Address::MOTOR_MAIN_TARGET) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, targetId, Address::MOTOR_MAIN_TARGET, 3))
        return false;

    outValue = fixed2u24(rsp.data);
    return true;
}

bool WooDrive::setMotorDirection(uint8_t targetId, uint8_t value)
{
    Packet pkt;
    return makeSetU8(pkt, targetId, Address::MOTOR_DIRECTION, value) && sendOnly(pkt);
}

bool WooDrive::getMotorDirection(uint8_t targetId, uint8_t& outValue)
{
    WooProtocol::Packet pkt;
    WooProtocol::ResponseFrame rsp;

    if (!makeGetU8(pkt, targetId, Address::MOTOR_DIRECTION) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, targetId, Address::MOTOR_DIRECTION, 1))
        return false;

    outValue = rsp.data[0];
    return true;
}

bool WooDrive::setRunMode(uint8_t targetId, uint8_t value)
{
    Packet pkt;
    return makeSetU8(pkt, targetId, Address::MOTOR_MOTION_PROFILE_RUNMODE, value) && sendOnly(pkt);
}

bool WooDrive::getRunMode(uint8_t targetId, uint8_t& outValue)
{
    WooProtocol::Packet pkt;
    WooProtocol::ResponseFrame rsp;

    if (!makeGetU8(pkt, targetId, Address::MOTOR_MOTION_PROFILE_RUNMODE) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, targetId, Address::MOTOR_MOTION_PROFILE_RUNMODE, 1))
        return false;

    outValue = rsp.data[0];
    return true;
}

bool WooDrive::setMotorEnable(uint8_t targetId, uint8_t value)
{
    Packet pkt;
    return makeSetU8(pkt, targetId, Address::MOTOR_ENABLE, value) && sendOnly(pkt);
}

bool WooDrive::getMotorEnable(uint8_t targetId, uint8_t& outValue)
{
    WooProtocol::Packet pkt;
    WooProtocol::ResponseFrame rsp;

    if (!makeGetU8(pkt, targetId, Address::MOTOR_ENABLE) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, targetId, Address::MOTOR_ENABLE, 1))
        return false;

    outValue = rsp.data[0];
    return true;
}

bool WooDrive::setMotorBrake(uint8_t targetId, uint8_t value)
{
    Packet pkt;
    return makeSetU8(pkt, targetId, Address::MOTOR_BRAKE, value) && sendOnly(pkt);
}

bool WooDrive::getMotorBrake(uint8_t targetId, uint8_t& outValue)
{
    WooProtocol::Packet pkt;
    WooProtocol::ResponseFrame rsp;

    if (!makeGetU8(pkt, targetId, Address::MOTOR_BRAKE) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, targetId, Address::MOTOR_BRAKE, 1))
        return false;

    outValue = rsp.data[0];
    return true;
}

bool WooDrive::setExternalBrake(uint8_t targetId, uint8_t value)
{
    Packet pkt;
    return makeSetU8(pkt, targetId, Address::MOTOR_EXTERNAL_BRAKE, value) && sendOnly(pkt);
}

bool WooDrive::getExternalBrake(uint8_t targetId, uint8_t& outValue)
{
    WooProtocol::Packet pkt;
    WooProtocol::ResponseFrame rsp;

    if (!makeGetU8(pkt, targetId, Address::MOTOR_EXTERNAL_BRAKE) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, targetId, Address::MOTOR_EXTERNAL_BRAKE, 1))
        return false;

    outValue = rsp.data[0];
    return true;
}


// MOTOR STATUS
// MOTOR STATUS
bool WooDrive::getTotalTimeNow(uint8_t id, uint32_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU32(pkt, id, Address::TOTAL_TIME_NOW) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::TOTAL_TIME_NOW, 4))
        return false;

    outValue = beU32(rsp.data);
    return true;
}

bool WooDrive::getElapsedTimeNow(uint8_t id, uint32_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU32(pkt, id, Address::ELAPSED_TIME_LEFT_NOW) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::ELAPSED_TIME_LEFT_NOW, 4))
        return false;

    outValue = beU32(rsp.data);
    return true;
}

bool WooDrive::getRemainTimeNow(uint8_t id, uint32_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU32(pkt, id, Address::REMAIN_TIME_NOW) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::REMAIN_TIME_NOW, 4))
        return false;

    outValue = beU32(rsp.data);
    return true;
}

bool WooDrive::getPositionNow(uint8_t id, float& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    const uint8_t d[] = {5};

    if (!makeRequestAutoLen(pkt, id, DIR_GET, Address::POSITION_NOW, d, 1) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::POSITION_NOW, 5))
        return false;

    float value = (float)beU32(rsp.data) / 100.0f;
    uint8_t dir = rsp.data[4];

    outValue = (dir == 2) ? -value : value;
    return true;
}

bool WooDrive::getVelocityNow(uint8_t id, float& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    const uint8_t d[] = {4};

    if (!makeRequestAutoLen(pkt, id, DIR_GET, Address::VELOCITY_NOW, d, 1) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::VELOCITY_NOW, 4))
        return false;

    float value = (float)beU24(rsp.data) / 100.0f;
    uint8_t dir = rsp.data[3];

    outValue = (dir == 2) ? -value : value;
    return true;
}

bool WooDrive::getIqCurrentNow(uint8_t id, float& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    const uint8_t d[] = {3};
    if (!makeRequestAutoLen(pkt, id, DIR_GET, Address::IQ_CURRENT_NOW, d, 1) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::IQ_CURRENT_NOW, 3))
        return false;

    outValue = fixed2u16(rsp.data);
    if (rsp.data[2] == 2) outValue = -outValue;
    return true;
}

bool WooDrive::getIdCurrentNow(uint8_t id, float& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    const uint8_t d[] = {3};
    if (!makeRequestAutoLen(pkt, id, DIR_GET, Address::ID_CURRENT_NOW, d, 1) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::ID_CURRENT_NOW, 3))
        return false;

    outValue = fixed2u16(rsp.data);
    if (rsp.data[2] == 2) outValue = -outValue;
    return true;
}

bool WooDrive::getBusCurrentNow(uint8_t id, float& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU16(pkt, id, Address::BUS_CURRENT_NOW) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::BUS_CURRENT_NOW, 2))
        return false;

    outValue = fixed2u16(rsp.data);
    return true;
}

bool WooDrive::getVqVoltageNow(uint8_t id, float& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    const uint8_t d[] = {3};
    if (!makeRequestAutoLen(pkt, id, DIR_GET, Address::VQ_VOLTAGE_NOW, d, 1) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::VQ_VOLTAGE_NOW, 3))
        return false;

    outValue = fixed2u16(rsp.data);
    if (rsp.data[2] == 2) outValue = -outValue;
    return true;
}

bool WooDrive::getVdVoltageNow(uint8_t id, float& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    const uint8_t d[] = {3};
    if (!makeRequestAutoLen(pkt, id, DIR_GET, Address::VD_VOLTAGE_NOW, d, 1) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::VD_VOLTAGE_NOW, 3))
        return false;

    outValue = fixed2u16(rsp.data);
    if (rsp.data[2] == 2) outValue = -outValue;
    return true;
}

bool WooDrive::getBusVoltageNow(uint8_t id, float& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU16(pkt, id, Address::BUS_VOLTAGE_NOW) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::BUS_VOLTAGE_NOW, 2))
        return false;

    outValue = fixed2u16(rsp.data);
    return true;
}

bool WooDrive::getTemperatureNow(uint8_t id, float& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    if (!makeGetU16(pkt, id, Address::TEMPERATURE_NOW) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::TEMPERATURE_NOW, 2))
        return false;

    outValue = fixed2u16(rsp.data);
    return true;
}

bool WooDrive::getMotorPulseCountNow(uint8_t id, int64_t& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    const uint8_t d[] = {9};
    if (!makeRequestAutoLen(pkt, id, DIR_GET, Address::MOTOR_PULSECOUNT_NOW, d, 1) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::MOTOR_PULSECOUNT_NOW, 9))
        return false;

    uint64_t mag = beU64(rsp.data);
    uint8_t dir = rsp.data[8];
    outValue = (dir == 2) ? -(int64_t)mag : (int64_t)mag;
    return true;
}

bool WooDrive::getUPhaseCurrentNow(uint8_t id, float& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    const uint8_t d[] = {3};
    if (!makeRequestAutoLen(pkt, id, DIR_GET, Address::U_PHASE_CURRENT_NOW, d, 1) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::U_PHASE_CURRENT_NOW, 3))
        return false;

    outValue = fixed2u16(rsp.data);
    if (rsp.data[2] == 2) outValue = -outValue;
    return true;
}

bool WooDrive::getWPhaseCurrentNow(uint8_t id, float& outValue)
{
    Packet pkt;
    ResponseFrame rsp;
    const uint8_t d[] = {3};
    if (!makeRequestAutoLen(pkt, id, DIR_GET, Address::W_PHASE_CURRENT_NOW, d, 1) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::W_PHASE_CURRENT_NOW, 3))
        return false;

    outValue = fixed2u16(rsp.data);
    if (rsp.data[2] == 2) outValue = -outValue;
    return true;
}


// =========================================================
// HIGH LEVEL SET
// =========================================================
bool WooDrive::setMotorMainTargetDir(uint8_t id, float mainTarget, uint8_t dir)
{
    Packet pkt;
    if (mainTarget < 0.0f) mainTarget = 0.0f;
    uint32_t raw = (uint32_t)(mainTarget * 100.0f + 0.5f);
    uint8_t data[4];
    // MOTOR_MAIN_TARGET : 3 bytes
    data[0] = (raw >> 16) & 0xFF;
    data[1] = (raw >> 8)  & 0xFF;
    data[2] = raw & 0xFF;
    // MOTOR_DIRECTION : 1 byte
    data[3] = dir;
    return makeRequestAutoLen(pkt, id, DIR_SET, Address::MOTOR_MAIN_TARGET, data, 4) && sendOnly(pkt);
}

bool WooDrive::setMotorSubMainTargetDir(uint8_t id, float subTarget, float mainTarget, uint8_t dir)
{
    Packet pkt;
    if (subTarget < 0.0f)  subTarget = 0.0f;
    if (mainTarget < 0.0f) mainTarget = 0.0f;
    uint32_t rawSub  = (uint32_t)(subTarget  * 100.0f + 0.5f);
    uint32_t rawMain = (uint32_t)(mainTarget * 100.0f + 0.5f);
    uint8_t data[7];
    // MOTOR_SUB_TARGET : 3 bytes
    data[0] = (rawSub  >> 16) & 0xFF;
    data[1] = (rawSub  >> 8)  & 0xFF;
    data[2] = (rawSub)        & 0xFF;

    // MOTOR_MAIN_TARGET : 3 bytes
    data[3] = (rawMain >> 16) & 0xFF;
    data[4] = (rawMain >> 8)  & 0xFF;
    data[5] = (rawMain)       & 0xFF;

    // MOTOR_DIRECTION : 1 byte
    data[6] = dir;
    return makeRequestAutoLen(pkt, id, DIR_SET, Address::MOTOR_SUB_TARGET, data, 7) && sendOnly(pkt);
}

bool WooDrive::setMotorMotionAll(uint8_t id, uint16_t accelTime, uint16_t decelTime, uint8_t motionMode, float subTarget, float mainTarget, uint8_t dir)
{
    Packet pkt;
    if (subTarget < 0.0f)  subTarget = 0.0f;
    if (mainTarget < 0.0f) mainTarget = 0.0f;
    uint32_t rawSub  = (uint32_t)(subTarget  * 100.0f + 0.5f);
    uint32_t rawMain = (uint32_t)(mainTarget * 100.0f + 0.5f);
    uint8_t data[12];

    // MOTOR_ACCTIME : 2 bytes
    data[0] = (accelTime >> 8) & 0xFF;
    data[1] = accelTime & 0xFF;

    // MOTOR_DECTIME : 2 bytes
    data[2] = (decelTime >> 8) & 0xFF;
    data[3] = decelTime & 0xFF;

    // MOTOR_MOTION_MODE : 1 byte
    data[4] = motionMode;

    // MOTOR_SUB_TARGET : 3 bytes
    data[5] = (rawSub >> 16) & 0xFF;
    data[6] = (rawSub >> 8)  & 0xFF;
    data[7] = rawSub & 0xFF;

    // MOTOR_MAIN_TARGET : 3 bytes
    data[8]  = (rawMain >> 16) & 0xFF;
    data[9]  = (rawMain >> 8)  & 0xFF;
    data[10] = rawMain & 0xFF;

    // MOTOR_DIRECTION : 1 byte
    data[11] = dir;

    return makeRequestAutoLen(pkt, id, DIR_SET, Address::MOTOR_ACCTIME, data, 12) && sendOnly(pkt);
}


bool WooDrive::setMotorMainTargetSigned(uint8_t id, float mainTarget)
{
    uint8_t dir = (mainTarget >= 0.0f) ? 1 : 2;
    float absTarget = (mainTarget >= 0.0f) ? mainTarget : -mainTarget;
    return setMotorMainTargetDir(id, absTarget, dir);
}

bool WooDrive::setMotorSubMainTargetSigned(uint8_t id, float subTarget, float mainTarget)
{
    uint8_t dir = (mainTarget >= 0.0f) ? 1 : 2;
    float absMainTarget = (mainTarget >= 0.0f) ? mainTarget : -mainTarget;
    return setMotorSubMainTargetDir(id, subTarget, absMainTarget, dir);
}

bool WooDrive::setMotorMotionAllSigned(uint8_t id,
                                       uint16_t accelTime,
                                       uint16_t decelTime,
                                       uint8_t motionMode,
                                       float subTarget,
                                       float mainTarget)
{
    uint8_t dir;
    float magnitude;
    if (mainTarget >= 0.0f) {
        dir = 1;
        magnitude = mainTarget;
    } 
    else {
        dir = 2;
        magnitude = -mainTarget;
    }
    return setMotorMotionAll(id, accelTime, decelTime, motionMode,
                             subTarget, magnitude, dir);
}
// =========================================================
// HIGH LEVEL GET
// =========================================================
bool WooDrive::getPositionVelocity(uint8_t id, float& position, float& velocity)
{
    Packet pkt;
    ResponseFrame rsp;

    // 0xBC ~ 0xC4 = 9 bytes
    const uint8_t req[] = { 0x09 };

    if (!makeRequestAutoLen(pkt, id, DIR_GET, Address::POSITION_NOW, req, 1) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::POSITION_NOW, 0x09))
        return false;

    const uint8_t* d = rsp.data;
    uint8_t idx = 0;

    // POSITION_NOW (4) + DIR (1)
    {
        float mag = (float)beU32(&d[idx]) / 100.0f;
        idx += 4;
        uint8_t dir = d[idx++];
        position = (dir == 2) ? -mag : mag;
    }

    // VELOCITY_NOW (3) + DIR (1)
    {
        float mag = (float)beU24(&d[idx]) / 100.0f;
        idx += 3;
        uint8_t dir = d[idx++];
        velocity = (dir == 2) ? -mag : mag;
    }

    return (idx == 0x09);
}
bool WooDrive::getPositionVelocity(uint8_t id, PositionVelocity& s)
{
    return getPositionVelocity(id, s.position, s.velocity);
}

bool WooDrive::getPositionVelocityIq(uint8_t id, float& position, float& velocity, float& iqCurrent)
{
    Packet pkt;
    ResponseFrame rsp;

    // 0xBC ~ 0xC7 = 12 bytes
    const uint8_t req[] = { 0x0C };

    if (!makeRequestAutoLen(pkt, id, DIR_GET, Address::POSITION_NOW, req, 1) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::POSITION_NOW, 0x0C))
        return false;

    const uint8_t* d = rsp.data;
    uint8_t idx = 0;

    // POSITION_NOW (4) + DIR (1)
    {
        float mag = (float)beU32(&d[idx]) / 100.0f;
        idx += 4;
        uint8_t dir = d[idx++];
        position = (dir == 2) ? -mag : mag;
    }

    // VELOCITY_NOW (3) + DIR (1)
    {
        float mag = (float)beU24(&d[idx]) / 100.0f;
        idx += 3;
        uint8_t dir = d[idx++];
        velocity = (dir == 2) ? -mag : mag;
    }

    // IQ_CURRENT_NOW (2) + DIR (1)
    {
        float mag = fixed2u16(&d[idx]);
        idx += 2;
        uint8_t dir = d[idx++];
        iqCurrent = (dir == 2) ? -mag : mag;
    }

    return (idx == 0x0C);
}
bool WooDrive::getPositionVelocityIq(uint8_t id, PositionVelocityIq& s)
{
    return getPositionVelocityIq(id, s.position, s.velocity, s.iqCurrent);
}


bool WooDrive::getBoardConfigAll(uint8_t id, uint8_t& reset, uint8_t& idRead, uint8_t& fault,
                                 uint8_t& communicationMode, uint32_t& bps, uint16_t& watchdog)
{
    Packet pkt;
    ResponseFrame rsp;

    const uint8_t req[] = { 0x0A };   // 0x00 ~ 0x09

    if (!makeRequestAutoLen(pkt, id, DIR_GET, Address::RESET, req, 1) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::RESET, 0x0A))
        return false;

    const uint8_t* d = rsp.data;
    uint8_t idx = 0;

    reset             = beU8(&d[idx]);  idx += 1; // 0x00
    idRead            = beU8(&d[idx]);  idx += 1; // 0x01
    fault             = beU8(&d[idx]);  idx += 1; // 0x02
    communicationMode = beU8(&d[idx]);  idx += 1; // 0x03
    bps               = beU32(&d[idx]); idx += 4; // 0x04~0x07
    watchdog          = beU16(&d[idx]); idx += 2; // 0x08~0x09

    return (idx == 0x0A);
}

bool WooDrive::getBoardConfigAll(uint8_t id, BoardConfig& s)
{
    return getBoardConfigAll(id, s.reset, s.id, s.fault, s.communicationMode, s.bps, s.watchdog);
}

bool WooDrive::getMotorConfigAll(uint8_t id, uint8_t& motorType, uint8_t& feedbackType, uint8_t& startupFeedbackType,
                                 uint8_t& directionInvert, uint8_t& fieldWeakeningEnable, uint8_t& externalBrakePresent, 
                                 uint8_t& polePairs, uint8_t& feedbackDir, uint32_t& feedbackResolution, float& gear)
{
    Packet pkt;
    ResponseFrame rsp;

    const uint8_t req[] = { 0x10 };   // 0x10 ~ 0x1F

    if (!makeRequestAutoLen(pkt, id, DIR_GET, Address::MOTOR_TYPE, req, 1) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::MOTOR_TYPE, 0x10))
        return false;

    const uint8_t* d = rsp.data;
    uint8_t idx = 0;

    motorType            = beU8(&d[idx]);  idx += 1; // 0x10
    feedbackType         = beU8(&d[idx]);  idx += 1; // 0x11
    startupFeedbackType  = beU8(&d[idx]);  idx += 1; // 0x12
    directionInvert      = beU8(&d[idx]);  idx += 1; // 0x13
    fieldWeakeningEnable = beU8(&d[idx]);  idx += 1; // 0x14
    externalBrakePresent = beU8(&d[idx]);  idx += 1; // 0x15
    polePairs            = beU8(&d[idx]);  idx += 1; // 0x16
    feedbackDir          = beU8(&d[idx]);  idx += 1; // 0x17
    feedbackResolution   = beU32(&d[idx]); idx += 4; // 0x18~0x1B
    gear                 = fixed6u32(&d[idx]); idx += 4; // 0x1C~0x1F

    return (idx == 0x10);
}

bool WooDrive::getMotorConfigAll(uint8_t id, MotorConfig& s)
{
    return getMotorConfigAll(id, s.motorType, s.feedbackType, s.startupFeedbackType, s.directionInvert, s.fieldWeakeningEnable, s.externalBrakePresent,
                             s.polePairs, s.feedbackDir, s.feedbackResolution, s.gear);
}

bool WooDrive::getMotorFocSettingAll(uint8_t id, uint8_t& autoMotorSetup, uint8_t& alignmentMode, uint32_t& phaseOffset,
                                     uint8_t& forceAngleLevel, uint8_t& hallSensorSetting, uint8_t& mainPortLevelNow, uint8_t& subPortLevelNow)
{
    Packet pkt;
    ResponseFrame rsp;

    const uint8_t req[] = { 0x0A };   // 0x20 ~ 0x29

    if (!makeRequestAutoLen(pkt, id, DIR_GET, Address::AUTO_MOTOR_SETUP, req, 1) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::AUTO_MOTOR_SETUP, 0x0A))
        return false;

    const uint8_t* d = rsp.data;
    uint8_t idx = 0;

    autoMotorSetup  = beU8(&d[idx]);  idx += 1; // 0x20
    alignmentMode   = beU8(&d[idx]);  idx += 1; // 0x21
    phaseOffset     = beU32(&d[idx]); idx += 4; // 0x22~0x25
    forceAngleLevel = beU8(&d[idx]);  idx += 1; // 0x26
    hallSensorSetting = beU8(&d[idx]); idx += 1; // 0x27
    mainPortLevelNow  = beU8(&d[idx]); idx += 1; // 0x28
    subPortLevelNow   = beU8(&d[idx]); idx += 1; // 0x29

    return (idx == 0x0A);
}

bool WooDrive::getMotorFocSettingAll(uint8_t id, MotorFocSetting& s)
{
    return getMotorFocSettingAll(id, s.autoMotorSetup, s.alignmentMode, s.phaseOffset,
                                 s.forceAngleLevel, s.hallSensorSetting, s.mainPortLevelNow, s.subPortLevelNow);
}

bool WooDrive::getMotorParamAll(uint8_t id, uint8_t& autoParameter, float& ratedSpeed, float& ratedCurrent, float& ratedVoltage,
                                float& resistance, float& inductance, float& torqueConstant, float& momentOfInertia)
{
    Packet pkt;
    ResponseFrame rsp;

    const uint8_t req[] = { 0x18 };   // 0x30 ~ 0x47

    if (!makeRequestAutoLen(pkt, id, DIR_GET, Address::MOTOR_AUTO_PARAMETER_MEASUREMENT_MODE, req, 1) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::MOTOR_AUTO_PARAMETER_MEASUREMENT_MODE, 0x18))
        return false;

    const uint8_t* d = rsp.data;
    uint8_t idx = 0;

    autoParameter   = beU8(&d[idx]); idx += 1;                          // 0x30
    ratedSpeed      = fixed2u24(&d[idx]); idx += 3;                     // 0x31~0x33
    ratedCurrent    = fixed2u16(&d[idx]); idx += 2;                     // 0x34~0x35
    ratedVoltage    = fixed2u16(&d[idx]); idx += 2;                     // 0x36~0x37

    resistance      = rawToResistance(beU32(&d[idx]), ResistanceUnit::mOhm); idx += 4;       // 0x38~0x3B
    inductance      = rawToInductance(beU32(&d[idx]), InductanceUnit::mH); idx += 4;         // 0x3C~0x3F
    torqueConstant  = rawToTorqueConstant(beU32(&d[idx]), TorqueConstantUnit::mNmPerA); idx += 4; // 0x40~0x43
    momentOfInertia = rawToInertia(beU32(&d[idx]), InertiaUnit::g_cm2); idx += 4;            // 0x44~0x47

    return (idx == 0x18);
}

bool WooDrive::getMotorParamAll(uint8_t id, MotorParam& s)
{
    return getMotorParamAll(id, s.autoParameter, s.ratedSpeed, s.ratedCurrent, s.ratedVoltage,
                            s.resistance, s.inductance, s.torqueConstant, s.momentOfInertia);
}


bool WooDrive::getMotorGainAll(uint8_t id, uint8_t& gainMode, uint8_t& positionGainScale, uint8_t& velocityGainScale, uint8_t& currentGainScale,
                               uint32_t& positionPGain, uint32_t& velocityPGain, uint32_t& velocityIGain, uint32_t& currentPGain, uint32_t& currentIGain)
{
    Packet pkt;
    ResponseFrame rsp;

    const uint8_t req[] = { 0x18 };  // 0x50 ~ 0x67

    if (!makeRequestAutoLen(pkt, id, DIR_GET, Address::MOTOR_GAIN_MODE, req, 1) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::MOTOR_GAIN_MODE, 0x18))
        return false;

    const uint8_t* d = rsp.data;
    uint8_t idx = 0;

    gainMode          = beU8(&d[idx]); idx += 1;
    positionGainScale = beU8(&d[idx]); idx += 1;
    velocityGainScale = beU8(&d[idx]); idx += 1;
    currentGainScale  = beU8(&d[idx]); idx += 1;

    positionPGain = beU32(&d[idx]); idx += 4;
    velocityPGain = beU32(&d[idx]); idx += 4;
    velocityIGain = beU32(&d[idx]); idx += 4;
    currentPGain  = beU32(&d[idx]); idx += 4;
    currentIGain  = beU32(&d[idx]); idx += 4;

    return (idx == 0x18);
}

bool WooDrive::getMotorGainAll(uint8_t id, MotorGain& s)
{
    return getMotorGainAll(id, s.gainMode, s.positionGainScale, s.velocityGainScale, s.currentGainScale,
                           s.positionPGain, s.velocityPGain, s.velocityIGain, s.currentPGain, s.currentIGain);
}



bool WooDrive::getMotorLimitAll(uint8_t id, float& positionCcwMax, float& positionCwMax, float& velocityCcwMax, float& velocityCwMax,
                                float& iqCurrentCcwMax, float& iqCurrentCwMax, float& idCurrentMax, float& iqCurrentLimit,
                                float& busVoltageMaxLimit, float& busVoltageMinLimit, float& temperatureMaxLimit)
{
    Packet pkt;
    ResponseFrame rsp;

    const uint8_t req[] = { 0x1C };   // 0x70 ~ 0x8B

    if (!makeRequestAutoLen(pkt, id, DIR_GET, Address::POSITION_CCW_MAX, req, 1) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::POSITION_CCW_MAX, 0x1C))
        return false;

    const uint8_t* d = rsp.data;
    uint8_t idx = 0;

    positionCcwMax    = (float)beU32(&d[idx]) / 100.0f; idx += 4; // 0x70~0x73
    positionCwMax     = (float)beU32(&d[idx]) / 100.0f; idx += 4; // 0x74~0x77
    velocityCcwMax    = fixed2u24(&d[idx]); idx += 3;             // 0x78~0x7A
    velocityCwMax     = fixed2u24(&d[idx]); idx += 3;             // 0x7B~0x7D
    iqCurrentCcwMax   = fixed2u16(&d[idx]); idx += 2;             // 0x7E~0x7F
    iqCurrentCwMax    = fixed2u16(&d[idx]); idx += 2;             // 0x80~0x81
    idCurrentMax      = fixed2u16(&d[idx]); idx += 2;             // 0x82~0x83
    iqCurrentLimit    = fixed2u16(&d[idx]); idx += 2;             // 0x84~0x85
    busVoltageMaxLimit = fixed2u16(&d[idx]); idx += 2;            // 0x86~0x87
    busVoltageMinLimit = fixed2u16(&d[idx]); idx += 2;            // 0x88~0x89
    temperatureMaxLimit = fixed2u16(&d[idx]); idx += 2;           // 0x8A~0x8B

    return (idx == 0x1C);
}

bool WooDrive::getMotorLimitAll(uint8_t id, MotorLimit& s)
{
    return getMotorLimitAll(id, s.positionCcwMax, s.positionCwMax, s.velocityCcwMax, s.velocityCwMax, s.iqCurrentCcwMax, s.iqCurrentCwMax, s.idCurrentMax, s.iqCurrentLimit, s.busVoltageMaxLimit, s.busVoltageMinLimit, s.temperatureMaxLimit);
}

bool WooDrive::getMotorControlAll(uint8_t id, uint16_t& accelTime, uint16_t& decelTime, uint8_t& motionMode,
                                  float& subTarget, float& mainTarget, uint8_t& direction,
                                  uint8_t& runMode, uint8_t& motorEnable, uint8_t& motorBrake, uint8_t& externalBrake)
{
    Packet pkt;
    ResponseFrame rsp;

    const uint8_t req[] = { 0x10 };  // 0x90 ~ 0x9F

    if (!makeRequestAutoLen(pkt, id, DIR_GET, Address::MOTOR_ACCTIME, req, 1) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::MOTOR_ACCTIME, 0x10))
        return false;

    const uint8_t* d = rsp.data;
    uint8_t idx = 0;

    accelTime = beU16(&d[idx]); idx += 2;
    decelTime = beU16(&d[idx]); idx += 2;

    motionMode = beU8(&d[idx]); idx += 1;

    subTarget = fixed2u24(&d[idx]); idx += 3;
    mainTarget = fixed2u24(&d[idx]); idx += 3;

    direction = beU8(&d[idx]); idx += 1;
    runMode = beU8(&d[idx]); idx += 1;
    motorEnable = beU8(&d[idx]); idx += 1;
    motorBrake = beU8(&d[idx]); idx += 1;
    externalBrake = beU8(&d[idx]); idx += 1;

    return (idx == 0x10);
}

bool WooDrive::getMotorControlAll(uint8_t id, MotorControl& s)
{
    return getMotorControlAll(id, s.accelTime, s.decelTime, s.motionMode,
                              s.subTarget, s.mainTarget, s.direction,
                              s.runMode, s.motorEnable, s.motorBrake, s.externalBrake);
}

bool WooDrive::getMotorStatusAll(uint8_t id, uint32_t& totalTime, uint32_t& elapsedTime, uint32_t& remainTime,
                                 float& position, float& velocity, float& iqCurrent, float& idCurrent, float& busCurrent, 
                                 float& vqVoltage, float& vdVoltage, float& busVoltage, float& temperature,
                                 int64_t& pulseCount, float& uPhaseCurrent, float& wPhaseCurrent)
{
    Packet pkt;
    ResponseFrame rsp;

    const uint8_t req[] = { 0x36 };  // 🔥 54 bytes

    if (!makeRequestAutoLen(pkt, id, DIR_GET, Address::TOTAL_TIME_NOW, req, 1) ||
        !sendAndReceive(pkt, rsp) ||
        !checkRsp(rsp, id, Address::TOTAL_TIME_NOW, 0x36))
        return false;

    const uint8_t* d = rsp.data;
    uint8_t idx = 0;

    totalTime   = beU32(&d[idx]); idx += 4;
    elapsedTime = beU32(&d[idx]); idx += 4;
    remainTime  = beU32(&d[idx]); idx += 4;

    {
        float mag = (float)beU32(&d[idx]) / 100.0f;
        idx += 4;
        uint8_t dir = d[idx++];
        position = (dir == 2) ? -mag : mag;
    }

    {
        float mag = (float)beU24(&d[idx]) / 100.0f;
        idx += 3;
        uint8_t dir = d[idx++];
        velocity = (dir == 2) ? -mag : mag;
    }

    {
        float mag = fixed2u16(&d[idx]);
        idx += 2;
        uint8_t dir = d[idx++];
        iqCurrent = (dir == 2) ? -mag : mag;
    }

    {
        float mag = fixed2u16(&d[idx]);
        idx += 2;
        uint8_t dir = d[idx++];
        idCurrent = (dir == 2) ? -mag : mag;
    }

    busCurrent = fixed2u16(&d[idx]);
    idx += 2;

    {
        float mag = fixed2u16(&d[idx]);
        idx += 2;
        uint8_t dir = d[idx++];
        vqVoltage = (dir == 2) ? -mag : mag;
    }

    {
        float mag = fixed2u16(&d[idx]);
        idx += 2;
        uint8_t dir = d[idx++];
        vdVoltage = (dir == 2) ? -mag : mag;
    }

    busVoltage = fixed2u16(&d[idx]);
    idx += 2;

    temperature = fixed2u16(&d[idx]);
    idx += 2;

    {
        uint64_t mag = beU64(&d[idx]);
        idx += 8;
        uint8_t dir = d[idx++];
        pulseCount = (dir == 2) ? -(int64_t)mag : (int64_t)mag;
    }

    {
        float mag = fixed2u16(&d[idx]);
        idx += 2;
        uint8_t dir = d[idx++];
        uPhaseCurrent = (dir == 2) ? -mag : mag;
    }

    {
        float mag = fixed2u16(&d[idx]);
        idx += 2;
        uint8_t dir = d[idx++];
        wPhaseCurrent = (dir == 2) ? -mag : mag;
    }

    return (idx == 0x36);  
}
bool WooDrive::getMotorStatusAll(uint8_t id, MotorStatus& s)
{
    return getMotorStatusAll(id, s.totalTime, s.elapsedTime, s.remainTime,
                             s.position, s.velocity,
                             s.iqCurrent, s.idCurrent, s.busCurrent,
                             s.vqVoltage, s.vdVoltage, s.busVoltage,
                             s.temperature, s.pulseCount, s.uPhaseCurrent, s.wPhaseCurrent);
}














namespace WooProtocol
{
    static const uint16_t CrcTable[256] WOODRIVE_PROGMEM = {
        0x0000, 0xc0c1, 0xc181, 0x0140, 0xc301, 0x03c0, 0x0280, 0xc241,
        0xc601, 0x06c0, 0x0780, 0xc741, 0x0500, 0xc5c1, 0xc481, 0x0440,
        0xcc01, 0x0cc0, 0x0d80, 0xcd41, 0x0f00, 0xcfc1, 0xce81, 0x0e40,
        0x0a00, 0xcac1, 0xcb81, 0x0b40, 0xc901, 0x09c0, 0x0880, 0xc841,
        0xd801, 0x18c0, 0x1980, 0xd941, 0x1b00, 0xdbc1, 0xda81, 0x1a40,
        0x1e00, 0xdec1, 0xdf81, 0x1f40, 0xdd01, 0x1dc0, 0x1c80, 0xdc41,
        0x1400, 0xd4c1, 0xd581, 0x1540, 0xd701, 0x17c0, 0x1680, 0xd641,
        0xd201, 0x12c0, 0x1380, 0xd341, 0x1100, 0xd1c1, 0xd081, 0x1040,
        0xf001, 0x30c0, 0x3180, 0xf141, 0x3300, 0xf3c1, 0xf281, 0x3240,
        0x3600, 0xf6c1, 0xf781, 0x3740, 0xf501, 0x35c0, 0x3480, 0xf441,
        0x3c00, 0xfcc1, 0xfd81, 0x3d40, 0xff01, 0x3fc0, 0x3e80, 0xfe41,
        0xfa01, 0x3ac0, 0x3b80, 0xfb41, 0x3900, 0xf9c1, 0xf881, 0x3840,
        0x2800, 0xe8c1, 0xe981, 0x2940, 0xeb01, 0x2bc0, 0x2a80, 0xea41,
        0xee01, 0x2ec0, 0x2f80, 0xef41, 0x2d00, 0xedc1, 0xec81, 0x2c40,
        0xe401, 0x24c0, 0x2580, 0xe541, 0x2700, 0xe7c1, 0xe681, 0x2640,
        0x2200, 0xe2c1, 0xe381, 0x2340, 0xe101, 0x21c0, 0x2080, 0xe041,
        0xa001, 0x60c0, 0x6180, 0xa141, 0x6300, 0xa3c1, 0xa281, 0x6240,
        0x6600, 0xa6c1, 0xa781, 0x6740, 0xa501, 0x65c0, 0x6480, 0xa441,
        0x6c00, 0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0, 0x6e80, 0xae41,
        0xaa01, 0x6ac0, 0x6b80, 0xab41, 0x6900, 0xa9c1, 0xa881, 0x6840,
        0x7800, 0xb8c1, 0xb981, 0x7940, 0xbb01, 0x7bc0, 0x7a80, 0xba41,
        0xbe01, 0x7ec0, 0x7f80, 0xbf41, 0x7d00, 0xbdc1, 0xbc81, 0x7c40,
        0xb401, 0x74c0, 0x7580, 0xb541, 0x7700, 0xb7c1, 0xb681, 0x7640,
        0x7200, 0xb2c1, 0xb381, 0x7340, 0xb101, 0x71c0, 0x7080, 0xb041,
        0x5000, 0x90c1, 0x9181, 0x5140, 0x9301, 0x53c0, 0x5280, 0x9241,
        0x9601, 0x56c0, 0x5780, 0x9741, 0x5500, 0x95c1, 0x9481, 0x5440,
        0x9c01, 0x5cc0, 0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81, 0x5e40,
        0x5a00, 0x9ac1, 0x9b81, 0x5b40, 0x9901, 0x59c0, 0x5880, 0x9841,
        0x8801, 0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1, 0x8a81, 0x4a40,
        0x4e00, 0x8ec1, 0x8f81, 0x4f40, 0x8d01, 0x4dc0, 0x4c80, 0x8c41,
        0x4400, 0x84c1, 0x8581, 0x4540, 0x8701, 0x47c0, 0x4680, 0x8641,
        0x8201, 0x42c0, 0x4380, 0x8341, 0x4100, 0x81c1, 0x8081, 0x4040
    };

    uint16_t computeCrc16(const uint8_t* data, size_t len)
    {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < len; ++i) {
            crc = (crc >> 8) ^ WOODRIVE_READ_WORD(&CrcTable[(crc ^ data[i]) & 0xFF]);
        }
        return crc;
    }

    bool makeRequest(Packet& out,
                     uint8_t targetId,
                     const uint8_t* payload,
                     size_t payloadLen)
    {
        const size_t need = 3 + 1 + payloadLen + 2;
        if (need > MAX_PACKET_SIZE) return false;

        size_t idx = 0;
        out.buf[idx++] = 0xFF;
        out.buf[idx++] = 0xFF;
        out.buf[idx++] = 0xFE;
        out.buf[idx++] = targetId;

        for (size_t i = 0; i < payloadLen; ++i) {
            out.buf[idx++] = payload[i];
        }

        const uint16_t crc = computeCrc16(out.buf, idx);
        out.buf[idx++] = static_cast<uint8_t>(crc & 0xFF);
        out.buf[idx++] = static_cast<uint8_t>((crc >> 8) & 0xFF);

        out.len = idx;
        return true;
    }

    bool makeRequestAutoLen(Packet& out,
                            uint8_t targetId,
                            uint8_t dir,
                            uint8_t cmd,
                            const uint8_t* data,
                            size_t dataLen)
    {
        const size_t need = 3 + 1 + 1 + 1 + 1 + dataLen + 2;
        if (need > MAX_PACKET_SIZE) return false;

        size_t idx = 0;
        out.buf[idx++] = 0xFF;
        out.buf[idx++] = 0xFF;
        out.buf[idx++] = 0xFE;
        out.buf[idx++] = targetId;
        out.buf[idx++] = dir;
        out.buf[idx++] = cmd;
        out.buf[idx++] = static_cast<uint8_t>(dataLen + 2);

        for (size_t i = 0; i < dataLen; ++i) {
            out.buf[idx++] = data[i];
        }

        const uint16_t crc = computeCrc16(out.buf, idx);
        out.buf[idx++] = static_cast<uint8_t>(crc & 0xFF);
        out.buf[idx++] = static_cast<uint8_t>((crc >> 8) & 0xFF);

        out.len = idx;
        return true;
    }

    bool validateFrame(const ResponseFrame& rsp)
    {
        uint8_t raw[3 + 1 + 1 + 1 + 1 + MAX_RESPONSE_DATA] = {0};

        size_t idx = 0;
        raw[idx++] = 0xFF;
        raw[idx++] = 0xFF;
        raw[idx++] = 0xFE;
        raw[idx++] = rsp.id;
        raw[idx++] = rsp.dir;
        raw[idx++] = rsp.cmd;
        raw[idx++] = rsp.lenField;

        for (uint8_t i = 0; i < rsp.dataLen; ++i) {
            raw[idx++] = rsp.data[i];
        }

        return computeCrc16(raw, idx) == rsp.crc;
    }
}

// =========================================================
// INTEGRATED PLATFORM ADAPTERS IMPLEMENTATION
// =========================================================

#if defined(ARDUINO)

static void woo_begin_hardware_serial(Stream* s, uint32_t baudrate)
{
    ((HardwareSerial*)s)->begin(baudrate);
}

ArduinoTransport::ArduinoTransport(HardwareSerial& serial, uint32_t baudrate)
    : _serial(&serial), _beginFunc(&woo_begin_hardware_serial), _baudrate(baudrate)
{
}

#if defined(WOODRIVE_HAS_SOFTWARESERIAL)
static void woo_begin_software_serial(Stream* s, uint32_t baudrate)
{
    ((SoftwareSerial*)s)->begin(baudrate);
}

ArduinoTransport::ArduinoTransport(SoftwareSerial& serial, uint32_t baudrate)
    : _serial(&serial), _beginFunc(&woo_begin_software_serial), _baudrate(baudrate)
{
}
#endif

void ArduinoTransport::begin()
{
    if (_serial && _beginFunc) {
        _beginFunc(_serial, _baudrate);
    }
}

uint32_t ArduinoTransport::baudrate() const
{
    return _baudrate;
}

void ArduinoTransport::setBaudrate(uint32_t baudrate)
{
    _baudrate = baudrate;
}

size_t ArduinoTransport::write(const uint8_t* data, size_t len)
{
    return _serial ? _serial->write(data, len) : 0;
}

int ArduinoTransport::available()
{
    return _serial ? _serial->available() : 0;
}

int ArduinoTransport::read()
{
    return _serial ? _serial->read() : -1;
}

void ArduinoTransport::flush()
{
    if (_serial) _serial->flush();
}

void ArduinoTransport::clearRx()
{
    if (!_serial) return;
    while (_serial->available() > 0) {
        _serial->read();
    }
}

uint32_t ArduinoClock::nowMs() const
{
    return millis();
}

#endif // ARDUINO

#if defined(__linux__) || defined(__APPLE__)

#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#if defined(__linux__)
#include <asm/termbits.h>
#endif

static uint64_t woo_std_now_ms()
{
    using namespace std::chrono;
    return (uint64_t)duration_cast<milliseconds>(
        steady_clock::now().time_since_epoch()).count();
}

StdClock::StdClock()
    : _startMs(woo_std_now_ms())
{
}

uint32_t StdClock::nowMs() const
{
    return (uint32_t)(woo_std_now_ms() - _startMs);
}

static speed_t woo_posix_baud_to_speed(int baudrate)
{
    switch (baudrate) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
#ifdef B230400
        case 230400: return B230400;
#endif
#ifdef B460800
        case 460800: return B460800;
#endif
#ifdef B500000
        case 500000: return B500000;
#endif
#ifdef B576000
        case 576000: return B576000;
#endif
#ifdef B921600
        case 921600: return B921600;
#endif
#ifdef B1000000
        case 1000000: return B1000000;
#endif
#ifdef B1152000
        case 1152000: return B1152000;
#endif
#ifdef B1500000
        case 1500000: return B1500000;
#endif
#ifdef B2000000
        case 2000000: return B2000000;
#endif
        default: return (speed_t)0;
    }
}

#if defined(__linux__)
static bool woo_posix_set_custom_baud_linux(int fd, int baudrate)
{
    struct termios2 tio;
    if (ioctl(fd, TCGETS2, &tio) != 0) {
        return false;
    }

    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= BOTHER;
    tio.c_ispeed = baudrate;
    tio.c_ospeed = baudrate;

    if (ioctl(fd, TCSETS2, &tio) != 0) {
        return false;
    }

    return true;
}
#endif

static bool woo_posix_setup_serial(int fd, int baudrate)
{
    struct termios tty;
    std::memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd, &tty) != 0) {
        return false;
    }

    cfmakeraw(&tty);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    speed_t spd = woo_posix_baud_to_speed(baudrate);

    if (spd != 0) {
        if (cfsetispeed(&tty, spd) != 0) return false;
        if (cfsetospeed(&tty, spd) != 0) return false;
        if (tcsetattr(fd, TCSANOW, &tty) != 0) return false;
        tcflush(fd, TCIOFLUSH);
        return true;
    }

#if defined(__linux__)
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        return false;
    }
    if (!woo_posix_set_custom_baud_linux(fd, baudrate)) {
        return false;
    }
    tcflush(fd, TCIOFLUSH);
    return true;
#else
    return false;
#endif
}

PosixSerialTransport::PosixSerialTransport(const char* portName, int baudrate)
    : _fd(-1), _baudrate(baudrate)
{
    _fd = ::open(portName, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (_fd < 0) {
        return;
    }

    if (!woo_posix_setup_serial(_fd, _baudrate)) {
        ::close(_fd);
        _fd = -1;
    }
}

PosixSerialTransport::~PosixSerialTransport()
{
    if (_fd >= 0) {
        ::close(_fd);
        _fd = -1;
    }
}

bool PosixSerialTransport::isOpen() const
{
    return (_fd >= 0);
}

int PosixSerialTransport::fd() const
{
    return _fd;
}

int PosixSerialTransport::baudrate() const
{
    return _baudrate;
}

size_t PosixSerialTransport::write(const uint8_t* data, size_t len)
{
    if (_fd < 0) return 0;

    ssize_t n = ::write(_fd, data, len);
    if (n < 0) return 0;
    return (size_t)n;
}

int PosixSerialTransport::available()
{
    if (_fd < 0) return 0;

    int bytes = 0;
    if (ioctl(_fd, FIONREAD, &bytes) != 0) {
        return 0;
    }
    return bytes;
}

int PosixSerialTransport::read()
{
    if (_fd < 0) return -1;

    uint8_t b = 0;
    ssize_t n = ::read(_fd, &b, 1);
    if (n == 1) return (int)b;
    return -1;
}

void PosixSerialTransport::flush()
{
    if (_fd >= 0) {
        tcdrain(_fd);
    }
}

void PosixSerialTransport::clearRx()
{
    if (_fd >= 0) {
        tcflush(_fd, TCIFLUSH);
    }
}

#endif // __linux__ || __APPLE__

