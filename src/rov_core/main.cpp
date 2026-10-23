// =============================================================================
//  Главный контроллер ТНПА (env:rov_core): движители, линия, fail-safe, датчики.
//  Порядок прохода loop(): приём пакетов -> fail-safe -> ESC -> датчики (не более
//  одного I2C-устройства за проход) -> телеметрия -> индикация и лог.
// =============================================================================
#include <Arduino.h>
#include <stdio.h>

#include "comms.h"
#include "config.h"
#include "link_supervisor.h"
#include "protocol.h"
#include "rov_log.h"
#include "sensors.h"
#include "thrusters.h"

static LinkSupervisor supervisor;
static uint32_t lastLogMs = 0;
static uint32_t lastLogPackets = 0;
static uint32_t lastBlinkMs = 0;
static bool blinkState = false;
static bool lastArmed = false;
static uint32_t loopMaxUs = 0;     // худший проход loop() за последний период лога
static uint32_t loopStartUs = 0;

static void updateStatusLed(uint32_t nowMs) {
    bool level;
    if (!supervisor.linkOk(nowMs)) {
        if (nowMs - lastBlinkMs >= STATUS_BLINK_SLOW_MS) {
            lastBlinkMs = nowMs;
            blinkState = !blinkState;
        }
        level = blinkState;
    } else if (supervisor.armed()) {
        if (nowMs - lastBlinkMs >= STATUS_BLINK_FAST_MS) {
            lastBlinkMs = nowMs;
            blinkState = !blinkState;
        }
        level = blinkState;
    } else {
        level = true;
    }
    digitalWrite(STATUS_LED_PIN, level ? HIGH : LOW);
}

static void logStatus(uint32_t nowMs) {
    if (nowMs - lastLogMs < ROV_LOG_PERIOD_MS) return;
    lastLogMs = nowMs;
    const uint32_t total = comms::validPackets();
    const uint32_t perSec = total - lastLogPackets;
    lastLogPackets = total;
    logPrintf("[rov_core] пакетов/с=%lu ошибок CRC=%lu вне диапазона=%lu armed=%d причина disarm: %s "
                  "мкс M0=%u M1=%u M2=%u M3=%u свет=%d loop макс=%lu мкс\n",
                  static_cast<unsigned long>(perSec),
                  static_cast<unsigned long>(comms::crcErrors()),
                  static_cast<unsigned long>(comms::rangeErrors()),
                  supervisor.armed() ? 1 : 0, disarmReasonText(supervisor.reason()),
                  thrusters::pulseUs(M_STERN_LEFT), thrusters::pulseUs(M_STERN_RIGHT),
                  thrusters::pulseUs(M_BOW_VERT), thrusters::pulseUs(M_BOW_HORIZ),
                  supervisor.light() ? 1 : 0, static_cast<unsigned long>(loopMaxUs));
    loopMaxUs = 0;

    TelemetryData t;
    sensors::fill(t);
    char vbatText[32];
#if VBAT_ENABLED
    snprintf(vbatText, sizeof(vbatText), "vbat=%.2f В", static_cast<double>(t.vbat));
#else
    // D-009: делителя нет, в телеметрии vbat = 0 означает «не измеряется»
    snprintf(vbatText, sizeof(vbatText), "vbat: не измеряется");
#endif
    logPrintf("[датчики] MS5837=%s%s IMU=%s%s depth=%.2f м temp=%.1f °C %s "
              "roll=%.1f pitch=%.1f yaw=%.1f\n",
              sensors::depthOk() ? "OK" : "НЕТ", sensors::depthZeroed() ? "" : " (обнуление)",
              sensors::imuModel(), sensors::imuCalibrating() ? " (калибровка)" : (sensors::imuOk() ? "" : " НЕТ"),
              static_cast<double>(t.depth), static_cast<double>(t.temp), vbatText,
              static_cast<double>(t.roll), static_cast<double>(t.pitch), static_cast<double>(t.yaw));
}

void setup() {
    // Первым делом — нейтраль на все ESC
    thrusters::begin();

    pinMode(LIGHT_PIN, OUTPUT);
    digitalWrite(LIGHT_PIN, LOW);
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);

    Serial.setTxBufferSize(USB_LOG_TX_BUFFER);
    Serial.begin(USB_LOG_BAUD);
    comms::begin();

    const uint32_t now = millis();
    supervisor.begin(now);
    lastLogMs = now;
    lastBlinkMs = now;
    sensors::begin(now);
    logPrintf("[rov_core] прошивка %s, протокол v%d. ESC = %u мкс, удержание %u мс\n",
              FW_VERSION, PROTOCOL_VERSION, ESC_US_NEUTRAL, ESC_ARM_TIME_MS);
    loopStartUs = micros();
}

void loop() {
    const uint32_t startUs = micros();
    const uint32_t passUs = startUs - loopStartUs;
    loopStartUs = startUs;
    if (passUs > loopMaxUs) loopMaxUs = passUs;

    ControlPacket pkt;
    if (comms::poll(pkt)) {
        supervisor.onPacket(pkt, millis());
    }
    const uint32_t now = millis();
    supervisor.tick(now);

    if (supervisor.armed()) {
        const ControlPacket &p = supervisor.lastPacket();
        const float k = 1.0f / static_cast<float>(CTRL_AXIS_MAX);
        float axes[AX_COUNT];
        axes[AX_SURGE] = static_cast<float>(p.surge) * k;
        axes[AX_YAW] = static_cast<float>(p.yaw) * k;
        axes[AX_HEAVE] = static_cast<float>(p.heave) * k;
        axes[AX_SWAY] = static_cast<float>(p.sway) * k;
        thrusters::update(axes);
    } else {
        thrusters::stopAll();
    }

    if (supervisor.armed() != lastArmed) {
        lastArmed = supervisor.armed();
        if (lastArmed) {
            logPrintf("[rov_core] ARMED\n");
        } else {
            logPrintf("[rov_core] DISARM: %s\n", disarmReasonText(supervisor.reason()));
        }
    }

    digitalWrite(LIGHT_PIN, supervisor.light() ? HIGH : LOW);
    updateStatusLed(now);

    sensors::update(now);
    TelemetryData t;
    sensors::fill(t);
    comms::setTelemetry(t);
    comms::serviceTelemetry(now);
    logStatus(now);
}
