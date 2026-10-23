// =============================================================================
//  link_supervisor.h — автомат fail-safe ROV_CORE (ICD §8), без Arduino.h.
//  Время передаётся явно (nowMs), чтобы логику можно было проверить на ПК.
//
//  Правила:
//   - первые ESC_ARM_TIME_MS после begin(): armed=false независимо от пакетов;
//   - нет валидного пакета > FAILSAFE_TIMEOUT_MS: armed=false;
//   - пакет armed=0: armed=false, оси игнорируются;
//   - вход в armed только по фронту флага armed 0->1. Состояние «0» задаётся
//     стартом, таймаутом связи и пакетом armed=0. Пакеты armed=1, пришедшие
//     в окне старта, фронт «съедают»: после окна нужен повторный 0->1.
//   - подсветка = бит light последнего валидного пакета (при fail-safe сохраняется).
// =============================================================================
#pragma once

#include <stdint.h>

#include "protocol.h"

enum class DisarmReason : uint8_t {
    None = 0,     // armed
    Boot,         // окно инициализации ESC после старта
    WaitEdge,     // окно старта прошло, ждём фронт armed 0->1
    NoLink,       // таймаут связи (или пакетов ещё не было)
    Command       // пришёл пакет armed=0
};

const char *disarmReasonText(DisarmReason r);

class LinkSupervisor {
public:
    LinkSupervisor();

    void begin(uint32_t nowMs);

    // Валидный пакет (уже прошедший controlPacketValid())
    void onPacket(const ControlPacket &p, uint32_t nowMs);

    // Периодическая проверка таймаута и окна старта; вызывать каждый проход loop()
    void tick(uint32_t nowMs);

    bool armed() const { return armed_; }
    bool light() const { return light_; }
    bool linkOk(uint32_t nowMs) const;
    bool bootHold(uint32_t nowMs) const;
    DisarmReason reason() const { return reason_; }
    const ControlPacket &lastPacket() const { return last_; }
    uint32_t lastPacketMs() const { return lastPacketMs_; }

private:
    void disarm(DisarmReason r);

    ControlPacket last_;
    uint32_t bootMs_;
    uint32_t lastPacketMs_;
    bool havePacket_;
    bool armed_;
    bool light_;
    bool prevArmFlag_;   // последний учтённый флаг armed (для детектора фронта)
    DisarmReason reason_;
};
