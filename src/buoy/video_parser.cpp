// =============================================================================
//  video_parser — реализация потокового разбора видеокадров.
// =============================================================================
#include "video_parser.h"

#include <string.h>

void VideoFrameParser::drop(size_t k) {
    start_ += k;
    if (start_ >= end_) {
        start_ = 0;
        end_ = 0;
    }
    // Начало кандидата сместилось — инкрементальный CRC недействителен
    crc_ = CRC16_INIT;
    crcDone_ = 0;
}

void VideoFrameParser::compact() {
    if (start_ == 0) return;
    const size_t len = end_ - start_;
    memmove(buf_, buf_ + start_, len);
    start_ = 0;
    end_ = len;
}

void VideoFrameParser::process() {
    for (;;) {
        const size_t len = end_ - start_;
        if (len == 0) {
            need_ = 0;
            return;
        }
        uint8_t *w = buf_ + start_;

        // Поиск магии A5 5A в окне
        size_t i = 0;
        bool found = false;
        while (i < len) {
            const void *p = memchr(w + i, VIDEO_MAGIC1, len - i);
            if (p == nullptr) {
                i = len;
                break;
            }
            i = static_cast<size_t>(static_cast<const uint8_t *>(p) - w);
            if (i + 1U < len && w[i + 1U] == VIDEO_MAGIC2) {
                found = true;
                break;
            }
            if (i + 1U == len) break;  // A5 в последнем байте — возможное начало магии
            i++;
        }
        if (!found) {
            // Сохраняем последний байт, если это A5
            const size_t keep = (w[len - 1U] == VIDEO_MAGIC1) ? 1U : 0U;
            stats_.junkBytes += static_cast<uint32_t>(len - keep);
            drop(len - keep);
            need_ = keep;  // 1: ждём второй байт магии; 0: поиск
            return;
        }
        if (i > 0) {
            stats_.junkBytes += static_cast<uint32_t>(i);
            drop(i);
            continue;
        }

        if (len < kHeaderLen) {
            need_ = kHeaderLen - len;
            return;
        }
        VideoFrameHeader h;
        memcpy(&h, w, kHeaderLen);
        if (!videoHeaderValid(h)) {
            stats_.badHeader++;
            stats_.junkBytes++;
            drop(1);
            continue;
        }

        const size_t jpegLen = h.length;
        const size_t total = kHeaderLen + jpegLen + kCrcLen;
        // Инкрементальный CRC по уже принятой части JPEG
        const size_t have = (len - kHeaderLen) < jpegLen ? (len - kHeaderLen) : jpegLen;
        if (have > crcDone_) {
            crc_ = crc16Update(crc_, w + kHeaderLen + crcDone_, have - crcDone_);
            crcDone_ = have;
        }
        if (len < total) {
            need_ = total - len;
            return;
        }

        const uint16_t rxCrc = static_cast<uint16_t>(w[kHeaderLen + jpegLen]) |
                               static_cast<uint16_t>(static_cast<uint16_t>(w[kHeaderLen + jpegLen + 1U]) << 8);
        if (crc_ != rxCrc) {
            stats_.badCrc++;
            stats_.junkBytes++;
            drop(1);
            continue;
        }

        stats_.framesOk++;
        if (cb_ != nullptr) cb_(ctx_, w + kHeaderLen, jpegLen, h.seq);
        drop(total);
    }
}

void VideoFrameParser::feed(const uint8_t *data, size_t n) {
    size_t used = 0;
    while (used < n) {
        if (end_ == start_) {
            // Быстрый путь: окно пусто — пропускаем мусор до первого A5 без копирования
            const void *p = memchr(data + used, VIDEO_MAGIC1, n - used);
            if (p == nullptr) {
                stats_.junkBytes += static_cast<uint32_t>(n - used);
                return;
            }
            const size_t pos = static_cast<size_t>(static_cast<const uint8_t *>(p) - data);
            stats_.junkBytes += static_cast<uint32_t>(pos - used);
            used = pos;
            start_ = 0;
            end_ = 0;
            buf_[end_++] = data[used++];
            need_ = 1;
            continue;
        }
        // Берём ровно столько, сколько нужно до следующего решения
        size_t take = need_ != 0 ? need_ : 1U;  // need_==0 при непустом окне недостижимо
        if (take > n - used) take = n - used;
        if (end_ + take > kCapacity) compact();
        memcpy(buf_ + end_, data + used, take);
        end_ += take;
        used += take;
        process();
    }
}
