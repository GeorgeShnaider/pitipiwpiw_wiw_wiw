// =============================================================================
//  attitude.h — ориентация по IMU (чистая логика, без Arduino.h).
//  Связанная СК: X — вперёд, Y — влево, Z — вверх (правая).
//  Выходные знаки по ICD §6: roll + — правый борт вниз; pitch + — нос вверх;
//  yaw растёт при повороте вправо, 0..360, относительный.
// =============================================================================
#pragma once

#include <stdint.h>

#include "config.h"   // IMU_COMPL_ALPHA

float wrap180(float deg);   // -> (-180, 180]
float wrap360(float deg);   // -> [0, 360)

// Перестановка/инверсия осей датчика в связанную СК: out[i] = sign[i] * in[map[i]]
void remapAxes(const float in[3], const uint8_t map[3], const int8_t sign[3], float out[3]);

// Накопитель смещения гироскопа
class GyroBiasEstimator {
public:
    GyroBiasEstimator();
    void reset();
    void add(const float gyroDps[3]);
    uint32_t count() const { return n_; }
    void bias(float out[3]) const;   // среднее; при n=0 — нули

private:
    double sum_[3];
    uint32_t n_;
};

class AttitudeFilter {
public:
    AttitudeFilter();

    // Нулевые углы
    void reset();

    // Начальные roll/pitch по акселерометру; yaw не трогает
    void seed(const float accG[3]);

    // accG — ускорение в g, gyroDps — угловая скорость (без смещения), град/с, dtS — шаг, с
    void update(const float accG[3], const float gyroDps[3], float dtS);

    float roll() const { return roll_; }
    float pitch() const { return pitch_; }
    float yaw() const { return yaw_; }

    // Углы по акселерометру в соглашении ICD §6
    static float accelRoll(const float accG[3]);
    static float accelPitch(const float accG[3]);

private:
    float roll_;
    float pitch_;
    float yaw_;
};
