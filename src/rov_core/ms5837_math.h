// =============================================================================
//  ms5837_math.h — расчёты MS5837-30BA по даташиту (чистая логика, без Arduino.h):
//  CRC4 PROM, первый и второй порядок температурной компенсации, глубина.
// =============================================================================
#pragma once

#include <stdint.h>

static constexpr uint8_t MS5837_PROM_WORDS = 7;   // C0 (CRC+factory) .. C6

struct Ms5837Result {
    int32_t tempCenti;     // температура, 0.01 °C
    int32_t pressureDeci;  // давление, 0.1 мбар (разрешение 30BA)
};

// CRC4 по алгоритму даташита (AN520, вариант MS5837): C0[15:12] — CRC
uint8_t ms5837Crc4(const uint16_t prom[MS5837_PROM_WORDS]);
bool ms5837PromValid(const uint16_t prom[MS5837_PROM_WORDS]);

// Пересчёт D1 (давление) и D2 (температура). secondOrder=false — только первый
// порядок (для сверки с примером даташита), true — полная компенсация 30BA.
void ms5837Compute(const uint16_t prom[MS5837_PROM_WORDS], uint32_t d1, uint32_t d2,
                   bool secondOrder, Ms5837Result &out);

// Глубина, м: (P - P0) / (WATER_DENSITY * GRAVITY), давления в мбар
float ms5837DepthM(float pressureMbar, float zeroMbar);
