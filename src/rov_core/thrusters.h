// =============================================================================
//  thrusters.h — выход на 6 ESC (LEDC, core 2.0.x) поверх чистого Mixer.
// =============================================================================
#pragma once

#include <stdint.h>

#include "config.h"

namespace thrusters {

// Настройка LEDC и немедленная выдача ESC_US_NEUTRAL на все каналы
void begin();

// Все ESC = ESC_US_NEUTRAL немедленно, состояние slew = 0
void stopAll();

// Микширование осей {surge, yaw, heave, sway} -1..+1 с реальным dt по micros()
void update(const float axes[AX_COUNT]);

// Последняя выданная ширина импульса канала, мкс
uint16_t pulseUs(uint8_t motor);

}  // namespace thrusters
