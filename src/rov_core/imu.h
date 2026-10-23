// =============================================================================
//  imu.h — абстрактный интерфейс IMU и реализация для MPU6050/MPU6500.
//  Углы по ICD §6: roll ±180 (+ правый борт вниз), pitch ±90 (+ нос вверх),
//  yawRel 0..360 (растёт при повороте вправо, обнуляется после калибровки).
// =============================================================================
#pragma once

#include <stdint.h>

#include "attitude.h"

class IImu {
public:
    virtual ~IImu() = default;
    virtual void begin(uint32_t nowMs) = 0;
    // Шаг автомата; true, если в этом вызове была транзакция I2C
    virtual bool update(uint32_t nowMs) = 0;
    virtual float roll() const = 0;
    virtual float pitch() const = 0;
    virtual float yawRel() const = 0;
    virtual bool ok() const = 0;
    virtual bool calibrating() const = 0;
    virtual const char *model() const = 0;
};

class Mpu6050Imu : public IImu {
public:
    Mpu6050Imu();

    void begin(uint32_t nowMs) override;
    bool update(uint32_t nowMs) override;
    float roll() const override { return running() ? filter_.roll() : 0.0f; }
    float pitch() const override { return running() ? filter_.pitch() : 0.0f; }
    float yawRel() const override { return running() ? filter_.yaw() : 0.0f; }
    bool ok() const override { return ok_; }
    bool calibrating() const override { return state_ == State::Calibrating; }
    const char *model() const override { return model_; }

    uint8_t whoAmI() const { return whoAmI_; }
    uint32_t errors() const { return errors_; }

private:
    enum class State : uint8_t { Probe, Calibrating, Running, Error };

    bool running() const { return biasValid_; }
    void fail(uint32_t nowMs);
    bool readSample(float accG[3], float gyroDps[3]);

    State state_;
    uint32_t stateMs_;
    uint32_t lastPollMs_;
    uint32_t lastSampleUs_;
    bool haveSampleTime_;
    bool ok_;
    bool biasValid_;
    bool seedPending_;
    float bias_[3];
    uint8_t whoAmI_;
    const char *model_;
    uint32_t errors_;
    GyroBiasEstimator biasEst_;
    AttitudeFilter filter_;
};
