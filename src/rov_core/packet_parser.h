// =============================================================================
//  packet_parser.h — разбор ControlPacket скользящим окном (ICD §5), без Arduino.h.
//  Битый пакет (CRC, ось вне диапазона) отбрасывается целиком; поиск заголовка
//  продолжается с байта, следующего за отброшенным заголовком.
// =============================================================================
#pragma once

#include <stdint.h>

#include "protocol.h"

class PacketParser {
public:
    PacketParser();

    void reset();

    // Подаёт очередной байт. Возвращает true, если собран валидный пакет (out заполнен).
    bool feed(uint8_t b, ControlPacket &out);

    uint32_t validCount() const { return validCount_; }
    uint32_t crcErrors() const { return crcErrors_; }
    uint32_t rangeErrors() const { return rangeErrors_; }

private:
    void resync();

    uint8_t buf_[sizeof(ControlPacket)];
    uint8_t len_;
    uint32_t validCount_;
    uint32_t crcErrors_;
    uint32_t rangeErrors_;
};
