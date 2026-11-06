// =============================================================================
//  mixer.h — чистая логика микширования движителей (ICD §9), без Arduino.h.
//  Цепочка: MIX -> нормализация по max|.|>1 -> MOTOR_DIR -> THRUST_LIMIT ->
//  slew ESC_SLEW_PER_S -> ESC_DEADBAND -> ESC_US_NEUTRAL + (ESC_US_MAX - ESC_US_MIN)/2 * u мкс.
//  Собирается и на ESP32, и g++ на ПК (хост-тесты).
// =============================================================================
#pragma once

#include <stdint.h>

#include "config.h"

class Mixer {
public:
    Mixer();

    // Сброс состояния slew в 0: следующий разгон начнётся плавно с нуля
    void reset();

    // axes: {surge, yaw, heave, sway} в -1..+1; dtS — реальный шаг времени, с.
    // outUs: ширина импульса для каждого из ESC_COUNT каналов.
    void compute(const float axes[AX_COUNT], float dtS, uint16_t outUs[ESC_COUNT]);

    // Текущая команда мотора после slew (до мёртвой зоны), -1..+1
    float command(uint8_t motor) const;

    // Перевод команды -1..+1 в мкс с учётом мёртвой зоны и пределов ESC
    static uint16_t commandToUs(float u);

private:
    float slew_[ESC_COUNT];
};
