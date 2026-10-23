// =============================================================================
//  telemetry_line — сборщик строк телеметрии (контракт №3), без Arduino.h.
//  Байты из UART -> строки до '\n'. '\r' отбрасывается, пустые строки
//  пропускаются. Строка длиннее TELEMETRY_MAX_LEN-1 символов (без '\n')
//  считается переполнением и выбрасывается целиком до следующего '\n'.
// =============================================================================
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "protocol.h"

class TelemetryLineAssembler {
public:
    // Максимум полезных символов в строке (TELEMETRY_MAX_LEN включает '\n')
    static constexpr size_t kMaxContent = TELEMETRY_MAX_LEN - 1U;

    // Возвращает true, когда строка готова: line()/length() валидны до
    // следующего вызова push().
    bool push(uint8_t c);

    const char *line() const { return buf_; }
    size_t length() const { return len_; }
    uint32_t overflows() const { return overflows_; }

private:
    char buf_[TELEMETRY_MAX_LEN] = {};  // kMaxContent символов + '\0'
    size_t len_ = 0;
    bool ready_ = false;
    bool discarding_ = false;
    uint32_t overflows_ = 0;
};
