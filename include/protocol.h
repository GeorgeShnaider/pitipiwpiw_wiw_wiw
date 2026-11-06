// =============================================================================
//  protocol.h — ЕДИНЫЙ источник правды для всех протоколов обмена.
//  Владелец: ЛИД (чат 0). Любое изменение — только с согласия пользователя
//  и с синхронным обновлением docs/ICD.md.
//
//  Порядок байт во всех бинарных структурах: little-endian (родной для ESP32).
//  Файл не зависит от Arduino.h, чтобы его можно было подключать в тестах на ПК.
// =============================================================================
#pragma once

#include <stddef.h>
#include <stdint.h>

#define PROTOCOL_VERSION 2   // v2: в телеметрию (контракт №3) добавлен ключ armed (D-010)

// -----------------------------------------------------------------------------
//  Скорости линий кабеля (UART поверх RS-485)
// -----------------------------------------------------------------------------
#define LINK_BAUD  115200UL   // Пары 2+3: управление вниз, телеметрия вверх
#define VIDEO_BAUD 921600UL   // Пара 4: видео камера -> буй (можно поднять до 2000000 после испытаний)

// -----------------------------------------------------------------------------
//  Контракт №1. WebApp -> Буй, WebSocket text, 20 Гц
//  {"type":"cmd","armed":bool,"light":bool,
//   "axes":{"surge":float,"yaw":float,"heave":float,"sway":float}}
//  Значения осей: -1.0 .. +1.0
// -----------------------------------------------------------------------------
#define WS_PATH          "/ws"
#define WS_CMD_TYPE      "cmd"
#define WS_CMD_RATE_HZ   20

// -----------------------------------------------------------------------------
//  Контракт №2. Буй -> ТНПА, UART binary, 20 Гц (пары 2, линия вниз)
// -----------------------------------------------------------------------------
#define CTRL_HEADER1     0xAA
#define CTRL_HEADER2     0x55
#define CTRL_FLAG_ARMED  0x01   // бит 0
#define CTRL_FLAG_LIGHT  0x02   // бит 1
#define CTRL_AXIS_MAX    100    // оси передаются как int8_t в диапазоне -100..+100
#define CTRL_RATE_HZ     20

#pragma pack(push, 1)
struct ControlPacket {
    uint8_t header1;  // 0xAA
    uint8_t header2;  // 0x55
    int8_t  surge;    // -100 .. +100, вперёд/назад
    int8_t  yaw;      // -100 .. +100, поворот (+ = вправо)
    int8_t  heave;    // -100 .. +100, вверх/вниз (+ = вверх)
    int8_t  sway;     // -100 .. +100, лаг (+ = вправо)
    uint8_t flags;    // бит 0: armed, бит 1: light
    uint8_t crc;      // XOR первых 7 байт
};
#pragma pack(pop)

static_assert(sizeof(ControlPacket) == 8, "ControlPacket must be 8 bytes");

// XOR-контрольная сумма для ControlPacket
inline uint8_t calculateCRC(const uint8_t *data, size_t length) {
    uint8_t crc = 0;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
    }
    return crc;
}

// Заполняет заголовок и CRC готового пакета
inline void controlPacketSeal(ControlPacket &p) {
    p.header1 = CTRL_HEADER1;
    p.header2 = CTRL_HEADER2;
    p.crc = calculateCRC(reinterpret_cast<const uint8_t *>(&p), sizeof(ControlPacket) - 1);
}

// Проверяет заголовок, CRC и диапазоны осей
inline bool controlPacketValid(const ControlPacket &p) {
    if (p.header1 != CTRL_HEADER1 || p.header2 != CTRL_HEADER2) return false;
    if (calculateCRC(reinterpret_cast<const uint8_t *>(&p), sizeof(ControlPacket) - 1) != p.crc) return false;
    const int8_t axes[4] = {p.surge, p.yaw, p.heave, p.sway};
    for (int i = 0; i < 4; i++) {
        if (axes[i] > CTRL_AXIS_MAX || axes[i] < -CTRL_AXIS_MAX) return false;
    }
    return true;
}

// -----------------------------------------------------------------------------
//  Контракт №3. ТНПА -> Буй -> WebApp, однострочный JSON + '\n', 5-10 Гц
//  (UART пара 3 вверх, затем WebSocket text без изменений)
// -----------------------------------------------------------------------------
#define TELEMETRY_RATE_HZ   10
#define TELEMETRY_MAX_LEN   160   // максимум байт в строке, включая '\n'
// Формат для snprintf на стороне rov_core: depth[m], temp[°C], vbat[V], roll/pitch/yaw[deg],
// armed — фактическое состояние ROV_CORE (int: 0 или 1)
#define TELEMETRY_JSON_FMT \
    "{\"depth\":%.2f,\"temp\":%.1f,\"vbat\":%.2f,\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f,\"armed\":%d}\n"

// -----------------------------------------------------------------------------
//  Контракт №4. Камера -> Буй, UART binary (пара 4); Буй -> WebApp, WS binary
//
//  По кабелю каждый JPEG-кадр передаётся так:
//    [VideoFrameHeader 10 байт][JPEG, length байт][CRC16-CCITT от JPEG, 2 байта]
//  Буй проверяет оба CRC и пересылает в WebSocket ОДНО бинарное сообщение,
//  содержащее только JPEG-байты (без заголовка). Битые кадры отбрасываются.
// -----------------------------------------------------------------------------
#define VIDEO_MAGIC1        0xA5
#define VIDEO_MAGIC2        0x5A
#define VIDEO_MAX_FRAME     32768U   // кадры больше — отбрасываются (QVGA/VGA при quality 12 укладываются)

#pragma pack(push, 1)
struct VideoFrameHeader {
    uint8_t  magic1;   // 0xA5
    uint8_t  magic2;   // 0x5A
    uint16_t seq;      // счётчик кадров, переполняется
    uint32_t length;   // длина JPEG в байтах, 1 .. VIDEO_MAX_FRAME
    uint8_t  flags;    // зарезервировано, передавать 0
    uint8_t  hcrc;     // XOR первых 9 байт заголовка
};
#pragma pack(pop)

static_assert(sizeof(VideoFrameHeader) == 10, "VideoFrameHeader must be 10 bytes");

// CRC16-CCITT (полином 0x1021, начальное значение 0xFFFF), допускает потоковый расчёт
inline uint16_t crc16Update(uint16_t crc, const uint8_t *data, size_t length) {
    for (size_t i = 0; i < length; i++) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}
#define CRC16_INIT 0xFFFF

inline void videoHeaderSeal(VideoFrameHeader &h) {
    h.magic1 = VIDEO_MAGIC1;
    h.magic2 = VIDEO_MAGIC2;
    h.hcrc = calculateCRC(reinterpret_cast<const uint8_t *>(&h), sizeof(VideoFrameHeader) - 1);
}

inline bool videoHeaderValid(const VideoFrameHeader &h) {
    if (h.magic1 != VIDEO_MAGIC1 || h.magic2 != VIDEO_MAGIC2) return false;
    if (calculateCRC(reinterpret_cast<const uint8_t *>(&h), sizeof(VideoFrameHeader) - 1) != h.hcrc) return false;
    return h.length > 0 && h.length <= VIDEO_MAX_FRAME;
}
