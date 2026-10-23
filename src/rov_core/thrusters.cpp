// =============================================================================
//  thrusters.cpp — LEDC 50 Гц / 16 бит, перевод мкс -> duty от периода ШИМ.
// =============================================================================
#include "thrusters.h"

#include <Arduino.h>

#include "mixer.h"

namespace thrusters {

namespace {

constexpr uint32_t kPeriodUs = 1000000UL / ESC_PWM_FREQ_HZ;          // 20000 мкс
constexpr uint32_t kDutyMax = (1UL << ESC_PWM_RES_BITS) - 1UL;       // 65535

Mixer mixer;
uint16_t currentUs[ESC_COUNT];
uint32_t lastUpdateUs = 0;
bool timeValid = false;   // false: следующий update() стартует с dt=0

void writeUs(uint8_t ch, uint16_t us, bool force) {
    if (!force && currentUs[ch] == us) return;
    currentUs[ch] = us;
    const uint32_t duty = (static_cast<uint32_t>(us) * kDutyMax + kPeriodUs / 2U) / kPeriodUs;
    ledcWrite(ch, duty);
}

}  // namespace

void begin() {
    for (uint8_t i = 0; i < ESC_COUNT; i++) {
        ledcSetup(i, ESC_PWM_FREQ_HZ, ESC_PWM_RES_BITS);
        ledcAttachPin(ESC_PINS[i], i);
        writeUs(i, ESC_US_NEUTRAL, true);
    }
    mixer.reset();
    timeValid = false;
}

void stopAll() {
    for (uint8_t i = 0; i < ESC_COUNT; i++) writeUs(i, ESC_US_NEUTRAL, false);
    mixer.reset();
    timeValid = false;
}

void update(const float axes[AX_COUNT]) {
    const uint32_t nowUs = micros();
    float dtS = 0.0f;
    if (timeValid) dtS = static_cast<float>(nowUs - lastUpdateUs) * 1e-6f;
    lastUpdateUs = nowUs;
    timeValid = true;

    uint16_t out[ESC_COUNT];
    mixer.compute(axes, dtS, out);
    for (uint8_t i = 0; i < ESC_COUNT; i++) writeUs(i, out[i], false);
}

uint16_t pulseUs(uint8_t motor) {
    return motor < ESC_COUNT ? currentUs[motor] : ESC_US_NEUTRAL;
}

}  // namespace thrusters
