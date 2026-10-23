// =============================================================================
//  attitude.cpp — комплементарный фильтр roll/pitch, интеграл yaw.
//
//  Вывод знаков (СК X вперёд, Y влево, Z вверх, правая):
//   - поворот +X поднимает левый борт = правый вниз -> roll_rate = +gx;
//     в крене на правый борт ось Y смотрит вверх -> ay > 0 -> roll = atan2(ay, az);
//   - поворот +Y опускает нос -> pitch_rate = -gy; при носе вверх ax > 0 ->
//     pitch = atan2(ax, sqrt(ay^2 + az^2));
//   - поворот +Z — влево (против часовой сверху) -> yaw_rate = -gz.
// =============================================================================
#include "attitude.h"

#include <math.h>

namespace {
constexpr float kRadToDeg = 57.29577951308232f;
}

float wrap180(float deg) {
    float r = fmodf(deg, 360.0f);
    if (r > 180.0f) r -= 360.0f;
    if (r <= -180.0f) r += 360.0f;
    return r;
}

float wrap360(float deg) {
    float r = fmodf(deg, 360.0f);
    if (r < 0.0f) r += 360.0f;
    if (r >= 360.0f) r -= 360.0f;
    return r;
}

void remapAxes(const float in[3], const uint8_t map[3], const int8_t sign[3], float out[3]) {
    for (uint8_t i = 0; i < 3; i++) {
        const uint8_t src = map[i] < 3 ? map[i] : i;
        out[i] = static_cast<float>(sign[i]) * in[src];
    }
}

GyroBiasEstimator::GyroBiasEstimator() { reset(); }

void GyroBiasEstimator::reset() {
    sum_[0] = sum_[1] = sum_[2] = 0.0;
    n_ = 0;
}

void GyroBiasEstimator::add(const float gyroDps[3]) {
    for (uint8_t i = 0; i < 3; i++) sum_[i] += static_cast<double>(gyroDps[i]);
    n_++;
}

void GyroBiasEstimator::bias(float out[3]) const {
    for (uint8_t i = 0; i < 3; i++) {
        out[i] = n_ ? static_cast<float>(sum_[i] / static_cast<double>(n_)) : 0.0f;
    }
}

AttitudeFilter::AttitudeFilter() { reset(); }

void AttitudeFilter::reset() {
    roll_ = 0.0f;
    pitch_ = 0.0f;
    yaw_ = 0.0f;
}

float AttitudeFilter::accelRoll(const float accG[3]) {
    return atan2f(accG[1], accG[2]) * kRadToDeg;
}

float AttitudeFilter::accelPitch(const float accG[3]) {
    return atan2f(accG[0], sqrtf(accG[1] * accG[1] + accG[2] * accG[2])) * kRadToDeg;
}

void AttitudeFilter::seed(const float accG[3]) {
    roll_ = accelRoll(accG);
    pitch_ = accelPitch(accG);
}

void AttitudeFilter::update(const float accG[3], const float gyroDps[3], float dtS) {
    if (dtS < 0.0f) dtS = 0.0f;
    const float rollRate = gyroDps[0];
    const float pitchRate = -gyroDps[1];
    const float yawRate = -gyroDps[2];

    // Прогноз по гироскопу, коррекция по акселерометру с учётом перехода ±180
    const float rollPred = roll_ + rollRate * dtS;
    roll_ = wrap180(rollPred + (1.0f - IMU_COMPL_ALPHA) * wrap180(accelRoll(accG) - rollPred));

    const float pitchPred = pitch_ + pitchRate * dtS;
    float p = pitchPred + (1.0f - IMU_COMPL_ALPHA) * (accelPitch(accG) - pitchPred);
    if (p > 90.0f) p = 90.0f;
    if (p < -90.0f) p = -90.0f;
    pitch_ = p;

    yaw_ = wrap360(yaw_ + yawRate * dtS);
}
