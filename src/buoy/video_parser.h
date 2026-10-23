// =============================================================================
//  video_parser — потоковый разбор видеокадров (контракт №4), без Arduino.h.
//  [VideoFrameHeader 10 Б][JPEG length Б][CRC16-CCITT(JPEG) 2 Б, LE]
//
//  Поведение совпадает с эталонным приёмником cam_rx.py: при плохом заголовке
//  или CRC отбрасывается ОДИН байт (начало ложного/битого кадра) и поиск магии
//  продолжается внутри уже принятых байт — кадр, начавшийся внутри обрезанного,
//  не теряется. Ложная магия A5 5A внутри JPEG допустима: её отсеивают hcrc,
//  длина и CRC16.
//
//  Буфер статический (заголовок + VIDEO_MAX_FRAME + CRC); байты принимаются
//  ровно столько, сколько нужно для следующего решения, поэтому при успехе
//  сдвигов памяти нет. CRC16 считается инкрементально по мере прихода данных.
// =============================================================================
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "protocol.h"

struct VideoParserStats {
    uint32_t framesOk;    // целые кадры
    uint32_t badHeader;   // плохой hcrc или длина вне 1..VIDEO_MAX_FRAME
    uint32_t badCrc;      // CRC16 JPEG не сошёлся (битый или обрезанный кадр)
    uint32_t junkBytes;   // байт, отброшенных при поиске магии
};

class VideoFrameParser {
public:
    // Вызывается для каждого целого кадра. jpeg валиден только во время вызова.
    using FrameCallback = void (*)(void *ctx, const uint8_t *jpeg, size_t len, uint16_t seq);

    static constexpr size_t kHeaderLen = sizeof(VideoFrameHeader);
    static constexpr size_t kCrcLen = 2U;
    static constexpr size_t kCapacity = kHeaderLen + VIDEO_MAX_FRAME + kCrcLen;

    void setCallback(FrameCallback cb, void *ctx) {
        cb_ = cb;
        ctx_ = ctx;
    }

    // Принять порцию байт любого размера (в том числе 0).
    void feed(const uint8_t *data, size_t n);

    const VideoParserStats &stats() const { return stats_; }

private:
    void process();
    void drop(size_t k);
    void compact();

    uint8_t buf_[kCapacity] = {};
    size_t start_ = 0;      // начало окна кандидата в buf_
    size_t end_ = 0;        // конец принятых байт
    size_t need_ = 0;       // сколько байт нужно до следующего решения (0 — поиск магии)
    uint16_t crc_ = CRC16_INIT;
    size_t crcDone_ = 0;    // байт JPEG текущего кандидата, уже учтённых в crc_
    VideoParserStats stats_ = {};
    FrameCallback cb_ = nullptr;
    void *ctx_ = nullptr;
};
