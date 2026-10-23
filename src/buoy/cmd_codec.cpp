// =============================================================================
//  cmd_codec — реализация. Без Arduino.h, без кучи: JsonDocument работает
//  поверх статической арены (собственный ArduinoJson::Allocator).
// =============================================================================
#include "cmd_codec.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include <ArduinoJson.h>

namespace {

// Арена-«стопка» для одного JsonDocument. Перед каждым разбором сбрасывается.
// Размер выведен из ArduinoJson: пул вариантов 128 слотов x 8 Б на 32-битной
// платформе (256 x 16 Б на 64-битной) + список пулов + строки ключей/значений.
class StaticArena : public ArduinoJson::Allocator {
public:
    static constexpr size_t kAlign = alignof(max_align_t);
    static constexpr size_t kSize = 1024U * sizeof(void *);

    void reset() {
        top_ = 0;
        lastHdr_ = kNoBlock;
    }

    size_t highWater() const { return highWater_; }

    void *allocate(size_t size) override {
        const size_t need = kAlign + roundUp(size);
        if (need > kSize - top_) return nullptr;
        writeSize(top_, size);
        lastHdr_ = top_;
        top_ += need;
        if (top_ > highWater_) highWater_ = top_;
        return buf_ + lastHdr_ + kAlign;
    }

    void deallocate(void *ptr) override {
        // Освобождается только последний блок; остальное — при reset().
        if (ptr != nullptr && lastHdr_ != kNoBlock && ptr == buf_ + lastHdr_ + kAlign) {
            top_ = lastHdr_;
            lastHdr_ = kNoBlock;
        }
    }

    void *reallocate(void *ptr, size_t newSize) override {
        if (ptr == nullptr) return allocate(newSize);
        uint8_t *p = static_cast<uint8_t *>(ptr);
        const size_t hdr = static_cast<size_t>(p - buf_) - kAlign;
        const size_t oldSize = readSize(hdr);
        if (hdr == lastHdr_) {
            // Последний блок можно растянуть или сжать на месте
            const size_t need = kAlign + roundUp(newSize);
            if (need > kSize - hdr) return nullptr;
            writeSize(hdr, newSize);
            top_ = hdr + need;
            if (top_ > highWater_) highWater_ = top_;
            return ptr;
        }
        if (newSize <= oldSize) {
            writeSize(hdr, newSize);
            return ptr;
        }
        void *fresh = allocate(newSize);
        if (fresh == nullptr) return nullptr;
        memcpy(fresh, ptr, oldSize);
        return fresh;
    }

private:
    static constexpr size_t kNoBlock = static_cast<size_t>(-1);

    static size_t roundUp(size_t n) { return (n + kAlign - 1U) & ~(kAlign - 1U); }

    void writeSize(size_t hdr, size_t size) { memcpy(buf_ + hdr, &size, sizeof(size)); }

    size_t readSize(size_t hdr) const {
        size_t size = 0;
        memcpy(&size, buf_ + hdr, sizeof(size));
        return size;
    }

    alignas(max_align_t) uint8_t buf_[kSize] = {};
    size_t top_ = 0;
    size_t lastHdr_ = kNoBlock;
    size_t highWater_ = 0;
};

StaticArena gArena;

// Имена осей в порядке AxisIndex (контракт №1)
const char *const kAxisKeys[AX_COUNT] = {"surge", "yaw", "heave", "sway"};

}  // namespace

CmdParseResult parseCmdJson(const uint8_t *data, size_t len, BuoyCmd &out) {
    if (data == nullptr || len == 0) return CmdParseResult::BadJson;

    gArena.reset();
    CmdParseResult result = CmdParseResult::Ok;
    BuoyCmd parsed = {};
    {
        JsonDocument doc(&gArena);
        const DeserializationError err = deserializeJson(doc, reinterpret_cast<const char *>(data), len);
        JsonObjectConst root = doc.as<JsonObjectConst>();
        if (err || root.isNull()) {
            result = CmdParseResult::BadJson;
        } else {
            JsonVariantConst type = root["type"];
            JsonVariantConst armed = root["armed"];
            JsonVariantConst light = root["light"];
            JsonObjectConst axes = root["axes"].as<JsonObjectConst>();
            if (!type.is<const char *>() || strcmp(type.as<const char *>(), WS_CMD_TYPE) != 0) {
                result = CmdParseResult::BadType;
            } else if (!armed.is<bool>() || !light.is<bool>() || axes.isNull()) {
                result = CmdParseResult::BadField;
            } else {
                parsed.armed = armed.as<bool>();
                parsed.light = light.as<bool>();
                for (uint8_t i = 0; i < AX_COUNT; i++) {
                    JsonVariantConst v = axes[kAxisKeys[i]];
                    if (!v.is<double>()) {
                        result = CmdParseResult::BadField;
                        break;
                    }
                    double a = v.as<double>();
                    if (!isfinite(a)) {
                        result = CmdParseResult::BadField;
                        break;
                    }
                    if (a > 1.0) a = 1.0;
                    if (a < -1.0) a = -1.0;
                    parsed.axes[i] = static_cast<float>(a);
                }
            }
        }
    }
    gArena.reset();

    if (result == CmdParseResult::Ok) out = parsed;
    return result;
}

int8_t axisToWire(float axis) {
    if (!isfinite(axis)) return 0;
    long v = lroundf(axis * static_cast<float>(CTRL_AXIS_MAX));
    if (v > CTRL_AXIS_MAX) v = CTRL_AXIS_MAX;
    if (v < -CTRL_AXIS_MAX) v = -CTRL_AXIS_MAX;
    return static_cast<int8_t>(v);
}

void buildControlPacket(const BuoyCmd &cmd, bool fresh, ControlPacket &pkt) {
    memset(&pkt, 0, sizeof(pkt));
    if (fresh) {
        pkt.surge = axisToWire(cmd.axes[AX_SURGE]);
        pkt.yaw = axisToWire(cmd.axes[AX_YAW]);
        pkt.heave = axisToWire(cmd.axes[AX_HEAVE]);
        pkt.sway = axisToWire(cmd.axes[AX_SWAY]);
        if (cmd.armed) pkt.flags |= CTRL_FLAG_ARMED;
    }
    if (cmd.light) pkt.flags |= CTRL_FLAG_LIGHT;
    controlPacketSeal(pkt);
}

size_t cmdArenaHighWater() { return gArena.highWater(); }
