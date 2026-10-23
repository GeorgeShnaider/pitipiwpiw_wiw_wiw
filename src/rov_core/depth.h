// =============================================================================
//  depth.h — неблокирующий драйвер MS5837-30BA (конечный автомат на millis()).
//  OSR 8192: шум ~0.2 мбар (~2 мм воды), конверсия до 18.08 мс; цикл D1+D2
//  ~40 мс укладывается в период DEPTH_POLL_HZ = 10 Гц с запасом.
// =============================================================================
#pragma once

#include <stdint.h>

#include "ms5837_math.h"

class DepthSensor {
public:
    DepthSensor();

    void begin(uint32_t nowMs);

    // Шаг автомата. Возвращает true, если в этом вызове была транзакция I2C.
    bool update(uint32_t nowMs);

    bool ok() const { return ok_; }
    bool zeroed() const { return zeroDone_; }
    float depthM() const { return depthM_; }
    float tempC() const { return tempC_; }
    float pressureMbar() const { return pressureMbar_; }
    uint32_t errors() const { return errors_; }
    const char *lastError() const { return lastError_; }

private:
    enum class State : uint8_t { Init, WaitReset, Idle, WaitD1, WaitD2, Error };

    void fail(uint32_t nowMs, const char *why);
    bool readAdc(uint32_t &value);
    void process(uint32_t nowMs, uint32_t d1, uint32_t d2);

    State state_;
    uint32_t stateMs_;
    uint32_t lastCycleMs_;
    uint16_t prom_[MS5837_PROM_WORDS];
    uint32_t d1_;

    bool ok_;
    float depthM_;
    float tempC_;
    float pressureMbar_;

    // Обнуление глубины по среднему давлению за первые DEPTH_ZERO_MS
    bool zeroStarted_;
    bool zeroDone_;
    uint32_t zeroStartMs_;
    double zeroSum_;
    uint32_t zeroN_;
    float zeroMbar_;

    uint32_t errors_;
    const char *lastError_;
};
