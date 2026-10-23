// =============================================================================
//  link_supervisor.cpp — автомат fail-safe ROV_CORE (ICD §8).
// =============================================================================
#include "link_supervisor.h"

#include <string.h>

#include "config.h"

const char *disarmReasonText(DisarmReason r) {
    switch (r) {
        case DisarmReason::None:     return "нет";
        case DisarmReason::Boot:     return "старт (инициализация ESC)";
        case DisarmReason::WaitEdge: return "ждём фронт armed 0->1";
        case DisarmReason::NoLink:   return "нет связи";
        case DisarmReason::Command:  return "команда armed=0";
    }
    return "?";
}

LinkSupervisor::LinkSupervisor()
    : bootMs_(0), lastPacketMs_(0), havePacket_(false), armed_(false), light_(false),
      prevArmFlag_(false), reason_(DisarmReason::Boot) {
    memset(&last_, 0, sizeof(last_));
}

void LinkSupervisor::begin(uint32_t nowMs) {
    memset(&last_, 0, sizeof(last_));
    bootMs_ = nowMs;
    lastPacketMs_ = nowMs;
    havePacket_ = false;
    armed_ = false;
    light_ = false;
    prevArmFlag_ = false;
    reason_ = DisarmReason::Boot;
}

bool LinkSupervisor::bootHold(uint32_t nowMs) const {
    return (nowMs - bootMs_) < ESC_ARM_TIME_MS;
}

bool LinkSupervisor::linkOk(uint32_t nowMs) const {
    return havePacket_ && (nowMs - lastPacketMs_) <= FAILSAFE_TIMEOUT_MS;
}

void LinkSupervisor::disarm(DisarmReason r) {
    armed_ = false;
    reason_ = r;
}

void LinkSupervisor::onPacket(const ControlPacket &p, uint32_t nowMs) {
    last_ = p;
    lastPacketMs_ = nowMs;
    havePacket_ = true;
    light_ = (p.flags & CTRL_FLAG_LIGHT) != 0;

    const bool armFlag = (p.flags & CTRL_FLAG_ARMED) != 0;

    if (bootHold(nowMs)) {
        // В окне старта ничего не включаем, но флаг запоминаем: фронт «съеден»
        prevArmFlag_ = armFlag;
        disarm(DisarmReason::Boot);
        return;
    }

    if (!armFlag) {
        prevArmFlag_ = false;
        disarm(DisarmReason::Command);
        return;
    }

    if (!armed_ && !prevArmFlag_) {
        armed_ = true;
        reason_ = DisarmReason::None;
    }
    prevArmFlag_ = true;
    if (!armed_) reason_ = DisarmReason::WaitEdge;
}

void LinkSupervisor::tick(uint32_t nowMs) {
    if (bootHold(nowMs)) {
        disarm(DisarmReason::Boot);
        return;
    }
    if (!linkOk(nowMs)) {
        // Таймаут (или пакетов ещё не было): disarm, детектор фронта сбрасывается в 0
        prevArmFlag_ = false;
        disarm(DisarmReason::NoLink);
        return;
    }
    if (!armed_ && reason_ == DisarmReason::Boot) {
        // Окно старта закончилось при живой связи
        reason_ = prevArmFlag_ ? DisarmReason::WaitEdge : DisarmReason::Command;
    }
}
