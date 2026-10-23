// =============================================================================
//  depth.cpp — MS5837-30BA: reset -> PROM + CRC4 -> D1 -> D2 -> расчёт.
// =============================================================================
#include "depth.h"

#include "config.h"
#include "i2c_bus.h"

namespace {

// Команды MS5837 (даташит)
constexpr uint8_t kCmdReset = 0x1E;
constexpr uint8_t kCmdAdcRead = 0x00;
constexpr uint8_t kCmdPromBase = 0xA0;     // + 2*i
constexpr uint8_t kCmdConvD1Osr8192 = 0x4A;
constexpr uint8_t kCmdConvD2Osr8192 = 0x5A;

// Тайминги — MS5837_*_MS и DEPTH_ZERO_MS из config.h; период цикла — производный
constexpr uint32_t kCycleMs = 1000UL / DEPTH_POLL_HZ;

}  // namespace

DepthSensor::DepthSensor()
    : state_(State::Init), stateMs_(0), lastCycleMs_(0), d1_(0), ok_(false),
      depthM_(0.0f), tempC_(0.0f), pressureMbar_(0.0f), zeroStarted_(false),
      zeroDone_(false), zeroStartMs_(0), zeroSum_(0.0), zeroN_(0), zeroMbar_(0.0f),
      errors_(0), lastError_("нет") {
    for (uint8_t i = 0; i < MS5837_PROM_WORDS; i++) prom_[i] = 0;
}

void DepthSensor::begin(uint32_t nowMs) {
    state_ = State::Init;
    stateMs_ = nowMs;
}

void DepthSensor::fail(uint32_t nowMs, const char *why) {
    ok_ = false;
    errors_++;
    lastError_ = why;
    state_ = State::Error;
    stateMs_ = nowMs;
}

bool DepthSensor::readAdc(uint32_t &value) {
    uint8_t b[3];
    if (!i2cbus::readRegs(MS5837_I2C_ADDR, kCmdAdcRead, b, 3)) return false;
    value = (static_cast<uint32_t>(b[0]) << 16) | (static_cast<uint32_t>(b[1]) << 8) | b[2];
    return true;
}

void DepthSensor::process(uint32_t nowMs, uint32_t d1, uint32_t d2) {
    Ms5837Result r;
    ms5837Compute(prom_, d1, d2, true, r);
    tempC_ = static_cast<float>(r.tempCenti) * 0.01f;
    pressureMbar_ = static_cast<float>(r.pressureDeci) * 0.1f;
    ok_ = true;

    if (!zeroDone_) {
        if (!zeroStarted_) {
            zeroStarted_ = true;
            zeroStartMs_ = nowMs;
        }
        zeroSum_ += static_cast<double>(pressureMbar_);
        zeroN_++;
        if (nowMs - zeroStartMs_ >= DEPTH_ZERO_MS) {
            zeroMbar_ = static_cast<float>(zeroSum_ / static_cast<double>(zeroN_));
            zeroDone_ = true;
        }
        depthM_ = 0.0f;
        return;
    }
    depthM_ = ms5837DepthM(pressureMbar_, zeroMbar_);
}

bool DepthSensor::update(uint32_t nowMs) {
    switch (state_) {
        case State::Error:
            if (nowMs - stateMs_ < MS5837_RETRY_MS) return false;
            state_ = State::Init;
            return false;

        case State::Init:
            if (!i2cbus::writeCmd(MS5837_I2C_ADDR, kCmdReset)) {
                fail(nowMs, "нет ответа на reset");
                return true;
            }
            state_ = State::WaitReset;
            stateMs_ = nowMs;
            return true;

        case State::WaitReset: {
            if (nowMs - stateMs_ < MS5837_RESET_WAIT_MS) return false;
            for (uint8_t i = 0; i < MS5837_PROM_WORDS; i++) {
                uint8_t b[2];
                if (!i2cbus::readRegs(MS5837_I2C_ADDR, static_cast<uint8_t>(kCmdPromBase + 2U * i), b, 2)) {
                    fail(nowMs, "ошибка чтения PROM");
                    return true;
                }
                prom_[i] = static_cast<uint16_t>((static_cast<uint16_t>(b[0]) << 8) | b[1]);
            }
            if (!ms5837PromValid(prom_)) {
                fail(nowMs, "CRC4 PROM не сошлась");
                return true;
            }
            state_ = State::Idle;
            lastCycleMs_ = nowMs - kCycleMs;  // первый цикл сразу
            return true;
        }

        case State::Idle:
            if (nowMs - lastCycleMs_ < kCycleMs) return false;
            lastCycleMs_ = nowMs;
            if (!i2cbus::writeCmd(MS5837_I2C_ADDR, kCmdConvD1Osr8192)) {
                fail(nowMs, "ошибка запуска D1");
                return true;
            }
            state_ = State::WaitD1;
            stateMs_ = nowMs;
            return true;

        case State::WaitD1: {
            if (nowMs - stateMs_ < MS5837_CONV_WAIT_MS) return false;
            uint32_t v = 0;
            if (!readAdc(v) || v == 0) {
                fail(nowMs, "ошибка чтения D1");
                return true;
            }
            d1_ = v;
            if (!i2cbus::writeCmd(MS5837_I2C_ADDR, kCmdConvD2Osr8192)) {
                fail(nowMs, "ошибка запуска D2");
                return true;
            }
            state_ = State::WaitD2;
            stateMs_ = nowMs;
            return true;
        }

        case State::WaitD2: {
            if (nowMs - stateMs_ < MS5837_CONV_WAIT_MS) return false;
            uint32_t v = 0;
            if (!readAdc(v) || v == 0) {
                fail(nowMs, "ошибка чтения D2");
                return true;
            }
            process(nowMs, d1_, v);
            state_ = State::Idle;
            return true;
        }
    }
    return false;
}
