// =============================================================================
//  sensors.cpp — планировщик датчиков, VBAT, предупреждения в лог.
// =============================================================================
#include "sensors.h"

#include <Arduino.h>

#include "config.h"
#include "depth.h"
#include "i2c_bus.h"
#include "imu.h"
#include "rov_log.h"
#include "vbat_math.h"

namespace sensors {

namespace {

DepthSensor depth;
Mpu6050Imu mpu;
IImu &imu = mpu;
MovingAverage vbatAvg;

TelemetryData last = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
uint32_t lastVbatMs = 0;
uint32_t lastWarnMs = 0;
bool imuTurn = true;   // чередование датчиков на шине

void pollVbat(uint32_t nowMs) {
    if (nowMs - lastVbatMs < VBAT_POLL_MS) return;
    lastVbatMs = nowMs;
    const uint32_t mv = analogReadMilliVolts(VBAT_ADC_PIN);
    last.vbat = vbatAvg.add(vbatFromPinMv(mv));
}

void warnings(uint32_t nowMs) {
    if (nowMs - lastWarnMs < SENSOR_WARN_PERIOD_MS) return;
    lastWarnMs = nowMs;
    if (!depth.ok()) {
        logPrintf("[датчики] ВНИМАНИЕ: MS5837 не работает (%s), глубина/температура — последние значения\n",
                  depth.lastError());
    }
    if (!imu.ok()) {
        logPrintf("[датчики] ВНИМАНИЕ: IMU не работает (%s), углы — последние значения\n", imu.model());
    }
    if (vbatAvg.count() > 0) {
        if (last.vbat < VBAT_CRIT_V) {
            logPrintf("[датчики] КРИТИЧЕСКИЙ РАЗРЯД: VBAT=%.2f В < %.2f В, поднимайте аппарат\n",
                      static_cast<double>(last.vbat), static_cast<double>(VBAT_CRIT_V));
        } else if (last.vbat < VBAT_WARN_V) {
            logPrintf("[датчики] ВНИМАНИЕ: VBAT=%.2f В < %.2f В\n", static_cast<double>(last.vbat),
                      static_cast<double>(VBAT_WARN_V));
        }
    }
}

}  // namespace

void begin(uint32_t nowMs) {
    i2cbus::begin();
    analogSetPinAttenuation(VBAT_ADC_PIN, ADC_11db);
    depth.begin(nowMs);
    imu.begin(nowMs);
    lastVbatMs = nowMs - VBAT_POLL_MS;
    // Первое предупреждение — после первой попытки инициализации
    lastWarnMs = nowMs - SENSOR_WARN_PERIOD_MS + VBAT_POLL_MS * 10U;
}

void update(uint32_t nowMs) {
    // Не больше одного датчика на шине за проход loop(): ограничивает задержку
    // обработки пакетов при зависшей шине двумя таймаутами I2C.
    bool used;
    if (imuTurn) {
        used = imu.update(nowMs);
        if (!used) used = depth.update(nowMs);
    } else {
        used = depth.update(nowMs);
        if (!used) used = imu.update(nowMs);
    }
    if (used) imuTurn = !imuTurn;

    if (depth.ok()) {
        last.depth = depth.depthM();
        last.temp = depth.tempC();
    }
    if (imu.ok()) {
        last.roll = imu.roll();
        last.pitch = imu.pitch();
        last.yaw = imu.yawRel();
    }
    pollVbat(nowMs);
    warnings(nowMs);
}

void fill(TelemetryData &t) { t = last; }

bool depthOk() { return depth.ok(); }
bool depthZeroed() { return depth.zeroed(); }
bool imuOk() { return imu.ok(); }
bool imuCalibrating() { return imu.calibrating(); }
const char *imuModel() { return imu.model(); }
const char *depthError() { return depth.lastError(); }

}  // namespace sensors
