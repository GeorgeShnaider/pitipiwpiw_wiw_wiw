// =============================================================================
//  i2c_bus.cpp — Wire на I2C_SDA_PIN/I2C_SCL_PIN, I2C_FREQ_HZ.
// =============================================================================
#include "i2c_bus.h"

#include <Arduino.h>
#include <Wire.h>

#include "config.h"

namespace i2cbus {

void begin() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, static_cast<uint32_t>(I2C_FREQ_HZ));
    Wire.setTimeOut(static_cast<uint16_t>(I2C_TIMEOUT_MS));
}

bool writeCmd(uint8_t addr, uint8_t cmd) {
    Wire.beginTransmission(addr);
    Wire.write(cmd);
    return Wire.endTransmission(true) == 0;
}

bool writeReg(uint8_t addr, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission(true) == 0;
}

bool readRegs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    const uint8_t got = Wire.requestFrom(static_cast<uint16_t>(addr), n, true);
    if (got != n) {
        while (Wire.available() > 0) Wire.read();
        return false;
    }
    for (uint8_t i = 0; i < n; i++) buf[i] = static_cast<uint8_t>(Wire.read());
    return true;
}

}  // namespace i2cbus
