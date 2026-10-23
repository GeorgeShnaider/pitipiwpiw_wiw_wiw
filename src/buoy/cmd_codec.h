// =============================================================================
//  cmd_codec — чистая логика контрактов №1 и №2 (без Arduino.h).
//  JSON cmd от WebApp -> BuoyCmd -> ControlPacket.
//  Модуль собирается и на ПК (хост-тест), поэтому зависит только от
//  protocol.h, config.h и header-only ArduinoJson.
// =============================================================================
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "config.h"
#include "protocol.h"

// Последняя принятая команда WebApp. Оси уже обрезаны до [-1, +1].
struct BuoyCmd {
    bool armed;
    bool light;
    float axes[AX_COUNT];  // порядок AxisIndex: surge, yaw, heave, sway
};

enum class CmdParseResult : uint8_t {
    Ok = 0,
    BadJson,   // не JSON, не объект или не хватило памяти разбора
    BadType,   // нет "type" или он не равен WS_CMD_TYPE
    BadField,  // нет armed/light/axes, неверный тип, NaN/Inf
};

// Разбор одного целого text-сообщения WebSocket (контракт №1).
// Не реентерабельна: использует статическую арену ArduinoJson.
// Вызывать только из одной задачи (async_tcp). При ошибке out не меняется.
CmdParseResult parseCmdJson(const uint8_t *data, size_t len, BuoyCmd &out);

// Ось float -> int8 по контракту №2: round(axis * 100), ограничение ±CTRL_AXIS_MAX.
// NaN -> 0.
int8_t axisToWire(float axis);

// Сборка пакета вниз. fresh=false (таймаут ICD §8): armed=0, все оси 0,
// light сохраняется из cmd. Заголовок и CRC ставит controlPacketSeal.
void buildControlPacket(const BuoyCmd &cmd, bool fresh, ControlPacket &pkt);

// Максимум занятой памяти статической арены JSON (байт) — для диагностики.
size_t cmdArenaHighWater();
