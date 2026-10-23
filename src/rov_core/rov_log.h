// =============================================================================
//  rov_log.h — лог в USB Serial без кучи: vsnprintf в статический буфер.
//  (Print::printf ядра 2.0.x делает malloc для строк длиннее 64 байт.)
// =============================================================================
#pragma once

#include "config.h"   // ROV_LOG_LINE_MAX

void logPrintf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
