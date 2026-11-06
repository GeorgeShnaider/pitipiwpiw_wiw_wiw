// =============================================================================
//  i2c_bus.cpp — Wire на I2C_SDA_PIN/I2C_SCL_PIN, I2C_FREQ_HZ (порт I2C_NUM_0).
// =============================================================================
#include "i2c_bus.h"

#include <Arduino.h>
#include <Wire.h>
#include <driver/i2c.h>

#include "config.h"

namespace i2cbus {

namespace {

constexpr uint32_t kApbMhz = 80;   // тактирование таймаута I2C на ESP32 — APB 80 МГц
constexpr int kHwTimeoutCycles = static_cast<int>(I2C_HW_BIT_TIMEOUT_US * kApbMhz);

}  // namespace

void begin() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, static_cast<uint32_t>(I2C_FREQ_HZ));
    Wire.setTimeOut(static_cast<uint16_t>(I2C_TIMEOUT_MS));
    // Ядро ставит максимальный таймаут бита (≈13 мс); уменьшаем. Вызывать после
    // Wire.begin() и не вызывать Wire.setClock() позже — он вернёт максимум.
    i2c_set_timeout(I2C_NUM_0, kHwTimeoutCycles);
}

bool busIdle() {
    // Пины в режиме open-drain с включённым входом: уровень читается напрямую
    return digitalRead(I2C_SDA_PIN) == HIGH && digitalRead(I2C_SCL_PIN) == HIGH;
}

bool probe(uint8_t addr) {
    if (!busIdle()) return false;
    Wire.beginTransmission(addr);
    return Wire.endTransmission(true) == 0;
}

bool writeCmd(uint8_t addr, uint8_t cmd) {
    if (!busIdle()) return false;
    Wire.beginTransmission(addr);
    Wire.write(cmd);
    return Wire.endTransmission(true) == 0;
}

bool writeReg(uint8_t addr, uint8_t reg, uint8_t value) {
    if (!busIdle()) return false;
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission(true) == 0;
}

bool readRegs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
    // Запись адреса регистра со STOP: при NACK ядро ничего не печатает
    if (!writeCmd(addr, reg)) return false;
    if (!busIdle()) return false;
    const uint8_t got = Wire.requestFrom(static_cast<uint16_t>(addr), n, true);
    if (got != n) {
        while (Wire.available() > 0) Wire.read();
        return false;
    }
    for (uint8_t i = 0; i < n; i++) buf[i] = static_cast<uint8_t>(Wire.read());
    return true;
}

}  // namespace i2cbus
