// =============================================================================
//  Надводный буй — bring-up скелет прошивки (env:buoy).
//  Владелец каталога src/buoy/ — агент узла (см. docs/agents/). Файл полностью
//  заменяется реализацией по docs/ICD.md. Сейчас: баннер в Serial и мигание
//  индикатором раз в секунду без delay(), чтобы проверить сборку и прошивку.
// =============================================================================
#include <Arduino.h>

#include "config.h"
#include "protocol.h"

static uint32_t lastBlinkMs = 0;
static bool ledState = false;

void setup() {
    Serial.begin(115200);
    pinMode(BUOY_STATUS_LED, OUTPUT);
    digitalWrite(BUOY_STATUS_LED, LOW);
    Serial.printf("[buoy] fw %s, protocol v%d\n", FW_VERSION, PROTOCOL_VERSION);
}

void loop() {
    const uint32_t now = millis();
    if (now - lastBlinkMs >= 1000) {
        lastBlinkMs = now;
        ledState = !ledState;
        digitalWrite(BUOY_STATUS_LED, ledState ? HIGH : LOW);
    }
}
