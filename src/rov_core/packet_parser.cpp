// =============================================================================
//  packet_parser.cpp — скользящее окно по 0xAA 0x55, проверка controlPacketValid().
// =============================================================================
#include "packet_parser.h"

#include <string.h>

PacketParser::PacketParser() : len_(0), validCount_(0), crcErrors_(0), rangeErrors_(0) {
    memset(buf_, 0, sizeof(buf_));
}

void PacketParser::reset() { len_ = 0; }

// Сдвигает окно к следующему возможному заголовку внутри буфера (начиная с байта 1)
void PacketParser::resync() {
    uint8_t start = 1;
    while (start < len_) {
        if (buf_[start] == CTRL_HEADER1 &&
            (start + 1 >= len_ || buf_[start + 1] == CTRL_HEADER2)) {
            break;
        }
        start++;
    }
    const uint8_t remain = static_cast<uint8_t>(len_ - start);
    if (remain > 0) memmove(buf_, buf_ + start, remain);
    len_ = remain;
}

bool PacketParser::feed(uint8_t b, ControlPacket &out) {
    buf_[len_++] = b;

    // Проверка заголовка по мере набора
    if (len_ == 1 && buf_[0] != CTRL_HEADER1) {
        len_ = 0;
        return false;
    }
    if (len_ == 2 && buf_[1] != CTRL_HEADER2) {
        resync();
        return false;
    }
    if (len_ < sizeof(ControlPacket)) return false;

    ControlPacket p;
    memcpy(&p, buf_, sizeof(p));
    if (controlPacketValid(p)) {
        out = p;
        len_ = 0;
        validCount_++;
        return true;
    }

    // Классификация ошибки для статистики; решение о валидности — только controlPacketValid()
    if (calculateCRC(buf_, sizeof(ControlPacket) - 1) != buf_[sizeof(ControlPacket) - 1]) {
        crcErrors_++;
    } else {
        rangeErrors_++;
    }
    resync();
    return false;
}
