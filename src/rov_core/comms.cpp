// =============================================================================
//  comms.cpp — Serial2 на ROV_LINK_RX_PIN/ROV_LINK_TX_PIN, LINK_BAUD 8N1.
// =============================================================================
#include "comms.h"

#include <Arduino.h>
#include <stdio.h>

#include "config.h"
#include "packet_parser.h"

namespace comms {

namespace {

constexpr uint32_t kTelemetryPeriodMs = 1000UL / TELEMETRY_RATE_HZ;
constexpr size_t kLinkTxBuffer = 2U * TELEMETRY_MAX_LEN;   // запись телеметрии не ждёт FIFO

PacketParser parser;
TelemetryData telem = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
char txBuf[TELEMETRY_MAX_LEN];
uint32_t lastTelemetryMs = 0;
uint32_t dropped = 0;

}  // namespace

void begin() {
    Serial2.setTxBufferSize(kLinkTxBuffer);
    Serial2.begin(LINK_BAUD, SERIAL_8N1, ROV_LINK_RX_PIN, ROV_LINK_TX_PIN);
    parser.reset();
    lastTelemetryMs = millis();
}

bool poll(ControlPacket &out) {
    bool got = false;
    int avail = Serial2.available();
    while (avail-- > 0) {
        const int c = Serial2.read();
        if (c < 0) break;
        if (parser.feed(static_cast<uint8_t>(c), out)) got = true;
    }
    return got;
}

void serviceTelemetry(uint32_t nowMs, bool armed) {
    if (nowMs - lastTelemetryMs < kTelemetryPeriodMs) return;
    lastTelemetryMs = nowMs;
    const int n = snprintf(txBuf, sizeof(txBuf), TELEMETRY_JSON_FMT,
                           static_cast<double>(telem.depth), static_cast<double>(telem.temp),
                           static_cast<double>(telem.vbat), static_cast<double>(telem.roll),
                           static_cast<double>(telem.pitch), static_cast<double>(telem.yaw),
                           armed ? 1 : 0);
    if (n <= 0 || n >= static_cast<int>(sizeof(txBuf))) {
        // Строка обрезана бы без '\n' — не отправляем битый JSON
        dropped++;
        return;
    }
    Serial2.write(reinterpret_cast<const uint8_t *>(txBuf), static_cast<size_t>(n));
}

void setTelemetry(const TelemetryData &t) { telem = t; }

uint32_t validPackets() { return parser.validCount(); }
uint32_t crcErrors() { return parser.crcErrors(); }
uint32_t rangeErrors() { return parser.rangeErrors(); }
uint32_t telemetryDropped() { return dropped; }

}  // namespace comms
