// =============================================================================
//  ms5837_math.cpp — формулы MS5837-30BA (даташит, раздел PRESSURE AND TEMPERATURE
//  CALCULATION и SECOND ORDER TEMPERATURE COMPENSATION).
// =============================================================================
#include "ms5837_math.h"

#include "config.h"

namespace {

// Деление на 2^n с округлением вниз (как арифметический сдвиг в даташите).
// Для отрицательного dT обычное деление C++ округляет к нулю и даёт ошибку
// на 1 LSB (пример даташита: TEMP = 1981, а не 1982).
int64_t floorDiv(int64_t a, int64_t d) {
    return a >= 0 ? a / d : -((-a + d - 1) / d);
}

}  // namespace

uint8_t ms5837Crc4(const uint16_t prom[MS5837_PROM_WORDS]) {
    // 8 слов по 16 бит: C0 без CRC-битов, C1..C6, C7 = 0
    uint16_t w[8];
    w[0] = static_cast<uint16_t>(prom[0] & 0x0FFF);
    for (uint8_t i = 1; i < MS5837_PROM_WORDS; i++) w[i] = prom[i];
    w[7] = 0;

    uint16_t rem = 0;
    for (uint8_t cnt = 0; cnt < 16; cnt++) {
        if (cnt & 1U) {
            rem ^= static_cast<uint16_t>(w[cnt >> 1] & 0x00FF);
        } else {
            rem ^= static_cast<uint16_t>(w[cnt >> 1] >> 8);
        }
        for (uint8_t bit = 8; bit > 0; bit--) {
            if (rem & 0x8000U) {
                rem = static_cast<uint16_t>((rem << 1) ^ 0x3000U);
            } else {
                rem = static_cast<uint16_t>(rem << 1);
            }
        }
    }
    return static_cast<uint8_t>((rem >> 12) & 0x000F);
}

bool ms5837PromValid(const uint16_t prom[MS5837_PROM_WORDS]) {
    const uint8_t stored = static_cast<uint8_t>(prom[0] >> 12);
    return stored == ms5837Crc4(prom);
}

void ms5837Compute(const uint16_t prom[MS5837_PROM_WORDS], uint32_t d1, uint32_t d2,
                   bool secondOrder, Ms5837Result &out) {
    const int64_t C1 = prom[1], C2 = prom[2], C3 = prom[3];
    const int64_t C4 = prom[4], C5 = prom[5], C6 = prom[6];

    // Первый порядок
    const int64_t dT = static_cast<int64_t>(d2) - C5 * 256LL;
    int64_t temp = 2000LL + floorDiv(dT * C6, 8388608LL);           // 2^23
    int64_t off = C2 * 65536LL + floorDiv(C4 * dT, 128LL);          // 2^16, 2^7
    int64_t sens = C1 * 32768LL + floorDiv(C3 * dT, 256LL);         // 2^15, 2^8

    if (secondOrder) {
        int64_t ti, offi, sensi;
        if (temp < 2000LL) {
            // Низкая температура (< 20 °C)
            const int64_t t20 = temp - 2000LL;
            ti = (3LL * dT * dT) / 8589934592LL;              // 2^33
            offi = (3LL * t20 * t20) / 2LL;
            sensi = (5LL * t20 * t20) / 8LL;
            if (temp < -1500LL) {
                // Очень низкая температура (< -15 °C)
                const int64_t t15 = temp + 1500LL;
                offi += 7LL * t15 * t15;
                sensi += 4LL * t15 * t15;
            }
        } else {
            // Высокая температура (>= 20 °C)
            const int64_t t20 = temp - 2000LL;
            ti = (2LL * dT * dT) / 137438953472LL;            // 2^37
            offi = (t20 * t20) / 16LL;
            sensi = 0;
        }
        off -= offi;
        sens -= sensi;
        temp -= ti;
    }

    const int64_t p = floorDiv(floorDiv(static_cast<int64_t>(d1) * sens, 2097152LL) - off, 8192LL);  // 2^21, 2^13
    out.tempCenti = static_cast<int32_t>(temp);
    out.pressureDeci = static_cast<int32_t>(p);
}

float ms5837DepthM(float pressureMbar, float zeroMbar) {
    // 1 мбар = 100 Па
    return (pressureMbar - zeroMbar) * 100.0f / (WATER_DENSITY * GRAVITY);
}
