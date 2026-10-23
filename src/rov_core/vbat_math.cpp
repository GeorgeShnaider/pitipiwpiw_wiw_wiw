// =============================================================================
//  vbat_math.cpp — делитель батареи и скользящее среднее.
// =============================================================================
#include "vbat_math.h"

#include "config.h"

float vbatFromPinMv(uint32_t pinMv) {
    const float k = (VBAT_R_TOP + VBAT_R_BOTTOM) / VBAT_R_BOTTOM;
    return static_cast<float>(pinMv) * 0.001f * k * VBAT_CAL_K;
}

float vbatToPinMv(float vbat) {
    const float k = (VBAT_R_TOP + VBAT_R_BOTTOM) / VBAT_R_BOTTOM;
    return vbat * 1000.0f / (k * VBAT_CAL_K);
}

MovingAverage::MovingAverage() { reset(); }

void MovingAverage::reset() {
    for (uint8_t i = 0; i < VBAT_AVG_WINDOW; i++) buf_[i] = 0.0f;
    sum_ = 0.0f;
    idx_ = 0;
    n_ = 0;
}

float MovingAverage::add(float v) {
    if (n_ < VBAT_AVG_WINDOW) n_++;
    buf_[idx_] = v;
    idx_ = static_cast<uint8_t>((idx_ + 1U) % VBAT_AVG_WINDOW);
    // Сумма пересчитывается целиком: окно маленькое, ошибка округления не копится
    float s = 0.0f;
    for (uint8_t i = 0; i < n_; i++) s += buf_[i];
    sum_ = s;
    return value();
}

float MovingAverage::value() const {
    return n_ ? sum_ / static_cast<float>(n_) : 0.0f;
}
