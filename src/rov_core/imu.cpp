// =============================================================================
//  imu.cpp — MPU6050/MPU6500 по I2C, неблокирующий автомат:
//  Probe (WHO_AM_I + настройка) -> Calibrating (2 с, смещение гироскопа) -> Running.
//  Ошибка I2C -> Error, повторная попытка через IMU_RETRY_MS.
// =============================================================================
#include "imu.h"

#include <Arduino.h>

#include "rov_log.h"

#include "config.h"
#include "i2c_bus.h"
#include "imu_id.h"

namespace {

// Регистры MPU6050/MPU6500 (общая карта)
constexpr uint8_t kRegSmplrtDiv = 0x19;
constexpr uint8_t kRegConfig = 0x1A;
constexpr uint8_t kRegGyroConfig = 0x1B;
constexpr uint8_t kRegAccelConfig = 0x1C;
constexpr uint8_t kRegAccelXoutH = 0x3B;
constexpr uint8_t kRegPwrMgmt1 = 0x6B;
constexpr uint8_t kRegWhoAmI = 0x75;

constexpr uint8_t kPwrClockPllX = 0x01;   // выход из сна, тактирование от PLL гироскопа X
constexpr uint8_t kSmplrtDiv1kHz = 0x00;
constexpr uint8_t kDlpf44Hz = 0x03;       // DLPF ~44 Гц: меньше вибрации от моторов
constexpr uint8_t kGyroFs500 = 0x08;      // ±500 град/с
constexpr uint8_t kAccelFs2g = 0x00;      // ±2 g
constexpr float kGyroLsbPerDps = 65.5f;   // для ±500 град/с
constexpr float kAccelLsbPerG = 16384.0f; // для ±2 g

bool idWarned = false;   // предупреждение о WHO_AM_I — однократно за запуск

// Тайминги — IMU_CALIB_MS/IMU_RETRY_MS из config.h; период опроса — производный
constexpr uint32_t kPollMs = 1000UL / SENSOR_POLL_HZ;

int16_t be16(const uint8_t *p) {
    return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

}  // namespace

Mpu6050Imu::Mpu6050Imu()
    : state_(State::Probe), stateMs_(0), lastPollMs_(0), lastSampleUs_(0),
      haveSampleTime_(false), ok_(false), biasValid_(false), seedPending_(true),
      whoAmI_(0), model_("не найден"), errors_(0) {
    bias_[0] = bias_[1] = bias_[2] = 0.0f;
}

void Mpu6050Imu::begin(uint32_t nowMs) {
    state_ = State::Probe;
    stateMs_ = nowMs;
}

void Mpu6050Imu::fail(uint32_t nowMs) {
    ok_ = false;
    errors_++;
    state_ = State::Error;
    stateMs_ = nowMs;
    haveSampleTime_ = false;
}

bool Mpu6050Imu::readSample(float accG[3], float gyroDps[3]) {
    uint8_t b[14];
    if (!i2cbus::readRegs(IMU_I2C_ADDR, kRegAccelXoutH, b, sizeof(b))) return false;
    const float accRaw[3] = {be16(&b[0]) / kAccelLsbPerG, be16(&b[2]) / kAccelLsbPerG,
                             be16(&b[4]) / kAccelLsbPerG};
    const float gyroRaw[3] = {be16(&b[8]) / kGyroLsbPerDps, be16(&b[10]) / kGyroLsbPerDps,
                              be16(&b[12]) / kGyroLsbPerDps};
    remapAxes(accRaw, IMU_AXIS_MAP, IMU_AXIS_SIGN, accG);
    remapAxes(gyroRaw, IMU_AXIS_MAP, IMU_AXIS_SIGN, gyroDps);
    return true;
}

bool Mpu6050Imu::update(uint32_t nowMs) {
    switch (state_) {
        case State::Error:
            if (nowMs - stateMs_ < IMU_RETRY_MS) return false;
            state_ = State::Probe;
            return false;

        case State::Probe: {
            uint8_t id = 0;
            if (!i2cbus::readRegs(IMU_I2C_ADDR, kRegWhoAmI, &id, 1)) {
                model_ = "не найден";
                fail(nowMs);
                return true;
            }
            whoAmI_ = id;
            const ImuIdClass cls = imuClassifyWhoAmI(id);
            model_ = imuModelName(cls);
            if (!imuSupported(cls)) {
                fail(nowMs);
                return true;
            }
            if (imuNeedsWarning(cls) && !idWarned) {
                idWarned = true;
                if (cls == ImuIdClass::Mpu6050Clone) {
                    logPrintf("[imu] ВНИМАНИЕ: клон MPU-6050, WHO_AM_I=0x%02X, настройка как для 6050\n", id);
                } else {
                    logPrintf("[imu] ВНИМАНИЕ: WHO_AM_I=0x%02X, не 6050/6500, работаем как с совместимым\n", id);
                }
            }
            if (!i2cbus::writeReg(IMU_I2C_ADDR, kRegPwrMgmt1, kPwrClockPllX) ||
                !i2cbus::writeReg(IMU_I2C_ADDR, kRegSmplrtDiv, kSmplrtDiv1kHz) ||
                !i2cbus::writeReg(IMU_I2C_ADDR, kRegConfig, kDlpf44Hz) ||
                !i2cbus::writeReg(IMU_I2C_ADDR, kRegGyroConfig, kGyroFs500) ||
                !i2cbus::writeReg(IMU_I2C_ADDR, kRegAccelConfig, kAccelFs2g)) {
                fail(nowMs);
                return true;
            }
            ok_ = true;
            haveSampleTime_ = false;
            lastPollMs_ = nowMs;
            stateMs_ = nowMs;
            if (biasValid_) {
                // Переинициализация после сбоя: смещение и yaw сохраняются
                seedPending_ = true;
                state_ = State::Running;
            } else {
                biasEst_.reset();
                state_ = State::Calibrating;
            }
            return true;
        }

        case State::Calibrating:
        case State::Running: {
            if (nowMs - lastPollMs_ < kPollMs) return false;
            lastPollMs_ = nowMs;
            float acc[3];
            float gyro[3];
            if (!readSample(acc, gyro)) {
                fail(nowMs);
                return true;
            }
            const uint32_t nowUs = micros();
            const float dtS = haveSampleTime_ ? static_cast<float>(nowUs - lastSampleUs_) * 1e-6f : 0.0f;
            lastSampleUs_ = nowUs;
            haveSampleTime_ = true;

            if (state_ == State::Calibrating) {
                biasEst_.add(gyro);
                if (nowMs - stateMs_ >= IMU_CALIB_MS) {
                    biasEst_.bias(bias_);
                    biasValid_ = true;
                    filter_.reset();
                    filter_.seed(acc);
                    seedPending_ = false;
                    state_ = State::Running;
                }
                return true;
            }

            if (seedPending_) {
                filter_.seed(acc);
                seedPending_ = false;
                return true;
            }
            for (uint8_t i = 0; i < 3; i++) gyro[i] -= bias_[i];
            filter_.update(acc, gyro, dtS);
            return true;
        }
    }
    return false;
}
