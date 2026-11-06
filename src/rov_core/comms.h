// =============================================================================
//  comms.h — линия кабеля UART2: приём ControlPacket (контракт №2),
//  отправка JSON-телеметрии 10 Гц (контракт №3).
// =============================================================================
#pragma once

#include <stdint.h>

#include "protocol.h"

// Значения для телеметрии. Этап 4 (датчики) заполняет через comms::setTelemetry().
struct TelemetryData {
    float depth;   // м
    float temp;    // °C
    float vbat;    // В
    float roll;    // град
    float pitch;   // град
    float yaw;     // град, относительный
};

namespace comms {

void begin();

// Разбирает все доступные байты. Возвращает true, если получен хотя бы один
// валидный пакет; out — последний из них.
bool poll(ControlPacket &out);

// Отправляет телеметрию, если подошло время (TELEMETRY_RATE_HZ).
// armed — фактическое состояние взвода на момент формирования строки (контракт №3, v2).
void serviceTelemetry(uint32_t nowMs, bool armed);

void setTelemetry(const TelemetryData &t);

uint32_t validPackets();
uint32_t crcErrors();
uint32_t rangeErrors();
uint32_t telemetryDropped();   // строки, не уложившиеся в TELEMETRY_MAX_LEN

}  // namespace comms
