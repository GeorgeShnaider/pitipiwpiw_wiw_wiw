// =============================================================================
//  rov_log.cpp — форматирование в статический буфер и Serial.write.
// =============================================================================
#include "rov_log.h"

#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>

namespace {
char logBuf[ROV_LOG_LINE_MAX];
}

void logPrintf(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(logBuf, sizeof(logBuf), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if (n >= static_cast<int>(sizeof(logBuf))) {
        // Обрезанная строка: гарантируем перевод строки в конце
        n = static_cast<int>(sizeof(logBuf)) - 1;
        logBuf[n - 1] = '\n';
    }
    Serial.write(reinterpret_cast<const uint8_t *>(logBuf), static_cast<size_t>(n));
}
