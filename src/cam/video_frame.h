// =============================================================================
//  video_frame.h — сборка обёртки видеокадра по контракту №4 (docs/ICD.md §7).
//  [VideoFrameHeader 10 Б][JPEG length Б][CRC16-CCITT(JPEG) 2 Б, little-endian]
//
//  Чистые функции без Arduino.h: файл подключается и в прошивку, и в хост-тест.
// =============================================================================
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "protocol.h"

// Длина хвоста кадра: CRC16 от JPEG, 2 байта little-endian
static constexpr size_t VIDEO_TRAILER_LEN = 2;

// Заполняет заголовок кадра. Возвращает false, если длина JPEG вне 1..VIDEO_MAX_FRAME
// (такой кадр по контракту передавать нельзя).
inline bool videoFrameBuildHeader(uint16_t seq, uint32_t jpegLen, VideoFrameHeader &h) {
    if (jpegLen == 0 || jpegLen > VIDEO_MAX_FRAME) return false;
    h.seq = seq;
    h.length = jpegLen;
    h.flags = 0;
    videoHeaderSeal(h);
    return true;
}

// Записывает CRC16 в хвост кадра, младший байт первым
inline void videoFrameBuildTrailer(uint16_t crc, uint8_t out[VIDEO_TRAILER_LEN]) {
    out[0] = static_cast<uint8_t>(crc & 0xFFu);
    out[1] = static_cast<uint8_t>((crc >> 8) & 0xFFu);
}

// CRC16 всего JPEG за один проход (в прошивке считается потоково через crc16Update)
inline uint16_t videoFrameCrc(const uint8_t *jpeg, size_t len) {
    return crc16Update(CRC16_INIT, jpeg, len);
}
