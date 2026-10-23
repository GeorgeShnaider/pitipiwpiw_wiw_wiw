// =============================================================================
//  link — кабельная линия UART2 (пары 2/3) к ROV_CORE.
//  Вниз: ControlPacket 20 Гц (контракт №2), всегда, даже без клиента.
//  Вверх: строки телеметрии (контракт №3) -> WebSocket.
// =============================================================================
#pragma once

#include <stdint.h>

struct LinkStats {
    uint32_t packetsSent;     // ControlPacket ушло в UART
    uint32_t packetsSkipped;  // не было места в TX-буфере UART
    uint32_t packetsArmed;    // из них с флагом armed
    uint32_t linesReceived;   // целых строк телеметрии
    uint32_t linesOverflow;   // строк, выброшенных из-за переполнения
};

// Serial2 на BUOY_LINK_*; вызывать из setup()
void linkBegin();

// Таймер пакетов вниз и приём телеметрии; вызывать из loop()
void linkLoop(uint32_t nowMs);

LinkStats linkGetStats();
