// =============================================================================
//  vbat_math.h — пересчёт напряжения батареи и скользящее среднее (без Arduino.h).
// =============================================================================
#pragma once

#include <stdint.h>

#include "config.h"   // VBAT_AVG_WINDOW

// мВ на пине АЦП -> В батареи (делитель VBAT_R_TOP/VBAT_R_BOTTOM, поправка VBAT_CAL_K)
float vbatFromPinMv(uint32_t pinMv);
// В батареи -> ожидаемые мВ на пине (для проверки делителя)
float vbatToPinMv(float vbat);

class MovingAverage {
public:
    MovingAverage();
    void reset();
    float add(float v);          // возвращает новое среднее
    float value() const;
    uint8_t count() const { return n_; }

private:
    float buf_[VBAT_AVG_WINDOW];
    float sum_;
    uint8_t idx_;
    uint8_t n_;
};
