// =============================================================================
//  telemetry_line — реализация сборщика строк телеметрии.
// =============================================================================
#include "telemetry_line.h"

bool TelemetryLineAssembler::push(uint8_t c) {
    if (ready_) {
        // Предыдущая строка уже отдана — начинаем новую
        ready_ = false;
        len_ = 0;
        buf_[0] = '\0';
    }
    if (c == '\r') return false;
    if (c == '\n') {
        if (discarding_) {
            discarding_ = false;
            len_ = 0;
            return false;
        }
        if (len_ == 0) return false;
        buf_[len_] = '\0';
        ready_ = true;
        return true;
    }
    if (discarding_) return false;
    if (len_ >= kMaxContent) {
        discarding_ = true;
        overflows_++;
        len_ = 0;
        return false;
    }
    buf_[len_++] = static_cast<char>(c);
    return false;
}
