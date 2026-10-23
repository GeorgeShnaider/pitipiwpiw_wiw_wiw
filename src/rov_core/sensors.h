// =============================================================================
//  sensors.h — опрос датчиков ROV_CORE: MS5837-30BA, IMU, VBAT. Всё неблокирующе.
//  За один вызов update() выполняется I2C-обмен не более чем одного датчика.
// =============================================================================
#pragma once

#include <stdint.h>

#include "comms.h"

namespace sensors {

void begin(uint32_t nowMs);
void update(uint32_t nowMs);

// Последние значения (при отказе датчика — последние валидные, изначально 0)
void fill(TelemetryData &t);

bool depthOk();
bool depthZeroed();
bool imuOk();
bool imuCalibrating();
const char *imuModel();
const char *depthError();

}  // namespace sensors
