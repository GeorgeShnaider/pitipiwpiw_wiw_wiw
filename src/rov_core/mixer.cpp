// =============================================================================
//  mixer.cpp — реализация цепочки ICD §9 (чистая логика, без Arduino.h).
// =============================================================================
#include "mixer.h"

namespace {

float clampUnit(float v) {
    if (v > 1.0f) return 1.0f;
    if (v < -1.0f) return -1.0f;
    return v;
}

float absf(float v) { return v < 0.0f ? -v : v; }

}  // namespace

Mixer::Mixer() { reset(); }

void Mixer::reset() {
    for (uint8_t i = 0; i < ESC_COUNT; i++) slew_[i] = 0.0f;
}

float Mixer::command(uint8_t motor) const {
    return motor < ESC_COUNT ? slew_[motor] : 0.0f;
}

uint16_t Mixer::commandToUs(float u) {
    u = clampUnit(u);
    if (absf(u) < ESC_DEADBAND) return ESC_US_NEUTRAL;
    const float halfSpan = static_cast<float>(ESC_US_MAX - ESC_US_MIN) * 0.5f;
    const float us = static_cast<float>(ESC_US_NEUTRAL) + halfSpan * u;
    // Значение всегда положительное, округление прибавлением 0.5
    int32_t r = static_cast<int32_t>(us + 0.5f);
    if (r < ESC_US_MIN) r = ESC_US_MIN;
    if (r > ESC_US_MAX) r = ESC_US_MAX;
    return static_cast<uint16_t>(r);
}

void Mixer::compute(const float axes[AX_COUNT], float dtS, uint16_t outUs[ESC_COUNT]) {
    float in[AX_COUNT];
    for (uint8_t j = 0; j < AX_COUNT; j++) in[j] = clampUnit(axes[j]);
    if (dtS < 0.0f) dtS = 0.0f;

    // 1. MIX
    float m[ESC_COUNT];
    float maxAbs = 0.0f;
    for (uint8_t i = 0; i < ESC_COUNT; i++) {
        float s = 0.0f;
        for (uint8_t j = 0; j < AX_COUNT; j++) s += MIX[i][j] * in[j];
        m[i] = s;
        if (MOTOR_ENABLED[i] && absf(s) > maxAbs) maxAbs = absf(s);
    }

    // 2. Нормализация, если max|мотор| > 1
    const float norm = maxAbs > 1.0f ? 1.0f / maxAbs : 1.0f;

    const float maxStep = ESC_SLEW_PER_S * dtS;
    for (uint8_t i = 0; i < ESC_COUNT; i++) {
        if (!MOTOR_ENABLED[i]) {
            slew_[i] = 0.0f;
            outUs[i] = ESC_US_NEUTRAL;
            continue;
        }
        // 3. MOTOR_DIR, 4. THRUST_LIMIT
        const float target = clampUnit(m[i] * norm) * static_cast<float>(MOTOR_DIR[i]) * THRUST_LIMIT;

        // 5. Slew-rate
        const float diff = target - slew_[i];
        if (diff > maxStep) {
            slew_[i] += maxStep;
        } else if (diff < -maxStep) {
            slew_[i] -= maxStep;
        } else {
            slew_[i] = target;
        }

        // 6. Мёртвая зона и 7. перевод в мкс
        outUs[i] = commandToUs(slew_[i]);
    }
}
