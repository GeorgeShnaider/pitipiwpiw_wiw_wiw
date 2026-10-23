// =============================================================================
//  buoy_log — реализация.
// =============================================================================
#include "buoy_log.h"

#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>

void logLine(const char *fmt, ...) {
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    const int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    const size_t len = static_cast<size_t>(n) < sizeof(buf) ? static_cast<size_t>(n) : sizeof(buf) - 1U;
    Serial.write(reinterpret_cast<const uint8_t *>(buf), len);
    Serial.write('\n');
}
