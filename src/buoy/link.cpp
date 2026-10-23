// =============================================================================
//  link — реализация канала UART2 буй <-> ROV_CORE.
// =============================================================================
#include "link.h"

#include <Arduino.h>

#include "buoy_timing.h"
#include "cmd_codec.h"
#include "config.h"
#include "protocol.h"
#include "telemetry_line.h"
#include "web.h"

namespace {

TelemetryLineAssembler assembler;
uint32_t nextTxMs = 0;
LinkStats stats = {};
uint8_t rxChunk[TELEMETRY_MAX_LEN];

// Время t уже наступило относительно now (корректно при переполнении millis)
bool reached(uint32_t now, uint32_t t) { return static_cast<int32_t>(now - t) >= 0; }

void sendControl() {
    const WebCmdSnapshot snap = webGetCmd();
    ControlPacket pkt;
    buildControlPacket(snap.cmd, snap.fresh, pkt);
    if (Serial2.availableForWrite() < static_cast<int>(sizeof(pkt))) {
        stats.packetsSkipped++;
        return;
    }
    Serial2.write(reinterpret_cast<const uint8_t *>(&pkt), sizeof(pkt));
    stats.packetsSent++;
    if (pkt.flags & CTRL_FLAG_ARMED) stats.packetsArmed++;
    digitalWrite(BUOY_STATUS_LED, snap.fresh ? HIGH : LOW);
}

void pollTelemetry() {
    int avail = Serial2.available();
    while (avail > 0) {
        const size_t want = static_cast<size_t>(avail) < sizeof(rxChunk) ? static_cast<size_t>(avail) : sizeof(rxChunk);
        const size_t got = Serial2.read(rxChunk, want);
        if (got == 0) break;
        for (size_t i = 0; i < got; i++) {
            if (assembler.push(rxChunk[i])) {
                stats.linesReceived++;
                webBroadcastText(assembler.line(), assembler.length());
            }
        }
        avail -= static_cast<int>(got);
    }
    stats.linesOverflow = assembler.overflows();
}

}  // namespace

void linkBegin() {
    pinMode(BUOY_STATUS_LED, OUTPUT);
    digitalWrite(BUOY_STATUS_LED, LOW);
    Serial2.begin(LINK_BAUD, SERIAL_8N1, BUOY_LINK_RX_PIN, BUOY_LINK_TX_PIN);
    nextTxMs = millis();
}

void linkLoop(uint32_t nowMs) {
    if (reached(nowMs, nextTxMs)) {
        sendControl();
        // Шаг от расписания, а не от now: без накопления дрейфа
        nextTxMs += BUOY_CTRL_PERIOD_MS;
        // Если loop надолго задержался — не догоняем пачкой, а пересинхронизируемся
        if (reached(nowMs, nextTxMs)) nextTxMs = nowMs + BUOY_CTRL_PERIOD_MS;
    }
    pollTelemetry();
}

LinkStats linkGetStats() { return stats; }
