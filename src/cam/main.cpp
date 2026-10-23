// =============================================================================
//  Курсовая камера ESP32-CAM — bring-up скелет прошивки (env:cam).
//  Владелец каталога src/cam/ — агент узла (см. docs/agents/). Файл полностью
//  заменяется реализацией по docs/ICD.md. Сейчас: баннер в Serial и сообщение
//  "alive" раз в секунду без delay(), чтобы проверить сборку и прошивку.
// =============================================================================
#include <Arduino.h>

#include "config.h"
#include "protocol.h"

static uint32_t lastBlinkMs = 0;

void setup() {
    Serial.begin(115200);
    pinMode(CAM_FLASH_LED_PIN, OUTPUT);
    digitalWrite(CAM_FLASH_LED_PIN, LOW);
    Serial.printf("[cam] fw %s, protocol v%d\n", FW_VERSION, PROTOCOL_VERSION);
}

void loop() {
    const uint32_t now = millis();
    if (now - lastBlinkMs >= 1000) {
        lastBlinkMs = now;
        // Вспышку ESP32-CAM не мигаем: она слишком яркая. Только сообщение в лог.
        Serial.printf("[cam] alive %lu ms\n", static_cast<unsigned long>(now));
    }
}
