// =============================================================================
//  i2c_bus.h — короткие транзакции I2C поверх Wire с ограниченным таймаутом.
// =============================================================================
#pragma once

#include <stddef.h>
#include <stdint.h>

// Таймаут транзакции — I2C_TIMEOUT_MS из config.h.

namespace i2cbus {

void begin();

// Запись одного байта-команды (MS5837: reset, конверсия, чтение АЦП)
bool writeCmd(uint8_t addr, uint8_t cmd);
// Запись регистра
bool writeReg(uint8_t addr, uint8_t reg, uint8_t value);
// Запись адреса регистра/команды и чтение n байт (повторный старт)
bool readRegs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n);

}  // namespace i2cbus
