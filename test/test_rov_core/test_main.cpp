// =============================================================================
//  Хост-тесты чистой логики ROV_CORE (Unity, env:native).
//  Запуск: pio test -e native -f test_rov_core
//  Эталоны MS5837 / CRC4 получены независимыми скриптами в ref/ (см. шапки).
// =============================================================================
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unity.h>

#include "config.h"
#include "protocol.h"
#include "rov_core/attitude.h"
#include "rov_core/link_supervisor.h"
#include "rov_core/mixer.h"
#include "rov_core/ms5837_math.h"
#include "rov_core/packet_parser.h"
#include "rov_core/vbat_math.h"

#include "crc_ref.inc"

// Общее число проверок CHECK во всех тестах (контроль, что ничего не потеряно)
static const int EXPECTED_CHECKS = 269;

static int g_checks = 0;
static char g_msg[256];

// Проверка с сообщением на русском; провал — через TEST_FAIL_MESSAGE.
// Unity экранирует не-ASCII байты (\xD0...), поэтому текст дублируется в stdout
// как есть, чтобы сообщение читалось в выводе pio test.
#define CHECK(c, ...)                                         \
    do {                                                      \
        g_checks++;                                           \
        if (!(c)) {                                           \
            snprintf(g_msg, sizeof(g_msg), __VA_ARGS__);      \
            printf("ПРОВАЛ: %s\n", g_msg);                  \
            TEST_FAIL_MESSAGE(g_msg);                         \
        }                                                     \
    } while (0)

#define INFO(...)                                             \
    do {                                                      \
        snprintf(g_msg, sizeof(g_msg), __VA_ARGS__);          \
        printf("%s\n", g_msg);                               \
    } while (0)

void setUp(void) {}
void tearDown(void) {}

// ---------------------------------------------------------------- микшер ----

static void run(Mixer &m, float s, float y, float h, float w, float t, uint16_t out[ESC_COUNT],
                float dt = 0.01f) {
    float ax[AX_COUNT] = {s, y, h, w};
    int n = (int)(t / dt + 0.5f);
    for (int i = 0; i < n; i++) m.compute(ax, dt, out);
}

static void test_mixer_signs(void) {
    uint16_t o[ESC_COUNT];
    Mixer m;
    run(m, 1, 0, 0, 0, 1, o);
    CHECK(o[0] > 1500 && o[1] > 1500, "surge+: ожидались M0,M1 > 1500, получено M0=%u M1=%u", o[0], o[1]);
    CHECK(o[4] == 1500 && o[5] == 1500, "резервные M4/M5 должны быть 1500, получено %u/%u", o[4], o[5]);
    m.reset(); run(m, 0, 1, 0, 0, 1, o);
    CHECK(o[0] > 1500 && o[1] < 1500, "yaw+: ожидались M0 > 1500, M1 < 1500, получено M0=%u M1=%u", o[0], o[1]);
    m.reset(); run(m, 0, 0, 1, 0, 1, o);
    CHECK(o[2] > 1500 && o[0] == 1500 && o[3] == 1500, "heave+: ожидался только M2 > 1500, получено M2=%u", o[2]);
    m.reset(); run(m, 0, 0, 0, 1, 1, o);
    CHECK(o[3] > 1500 && o[2] == 1500, "sway+: ожидался только M3 > 1500, получено M3=%u", o[3]);
    m.reset(); run(m, -1, -1, -1, -1, 1, o);
    CHECK(o[4] == 1500 && o[5] == 1500, "резервные M4/M5 при всех осях -1 должны быть 1500");
    m.reset(); run(m, -1, 0, 0, 0, 1, o);
    CHECK(o[0] == 1500 - (int)(500 * THRUST_LIMIT) && o[1] == o[0], "surge-: неверный M0=%u", o[0]);
}

static void test_mixer_normalization(void) {
    uint16_t o[ESC_COUNT];
    Mixer m;
    run(m, 1, 1, 0, 0, 2, o);
    uint16_t exp0 = (uint16_t)(1500 + 500 * THRUST_LIMIT + 0.5f);
    CHECK(o[0] == exp0 && o[1] == 1500, "нормализация surge=1,yaw=1: M0=%u (ожидалось %u), M1=%u (ожидалось 1500)",
          o[0], exp0, o[1]);
}

static void test_mixer_slew(void) {
    Mixer m;
    uint16_t o[ESC_COUNT];
    float ax[AX_COUNT] = {1, 0, 0, 0};
    const float dt = 0.001f;
    float t = 0;
    float target = THRUST_LIMIT;
    float prev = 0;
    bool ok = true;
    while (m.command(0) < target - 1e-6f && t < 5) {
        m.compute(ax, dt, o);
        t += dt;
        if (m.command(0) - prev > ESC_SLEW_PER_S * dt + 1e-6f) ok = false;
        prev = m.command(0);
    }
    float minT = target / ESC_SLEW_PER_S;
    CHECK(ok, "slew: шаг команды больше ESC_SLEW_PER_S*dt");
    CHECK(t >= minT - dt - 1e-4f, "slew: цель достигнута за %.4f с, быстрее минимума %.4f с", t, minT);
    m.reset(); m.compute(ax, 0.01f, o);
    CHECK(fabsf(m.command(0) - ESC_SLEW_PER_S * 0.01f) < 1e-6f, "slew: один шаг 0.01 с дал %.4f", m.command(0));
    m.reset(); run(m, 1, 0, 0, 0, 1, o);
    float ax2[AX_COUNT] = {-1, 0, 0, 0};
    m.compute(ax2, 0.01f, o);
    CHECK(fabsf(m.command(0) - (THRUST_LIMIT - ESC_SLEW_PER_S * 0.01f)) < 1e-5f, "slew: реверс не ограничен");
    INFO("slew: 0 -> %.2f за %.3f с (минимум %.3f с)", target, t, minT);
}

static void test_mixer_deadband(void) {
    for (float u = -0.049f; u <= 0.049f; u += 0.001f) {
        CHECK(Mixer::commandToUs(u) == 1500, "мёртвая зона: u=%f дал %u вместо 1500", u, Mixer::commandToUs(u));
    }
    CHECK(Mixer::commandToUs(0.05f) > 1500, "мёртвая зона: u=0.05 должен давать > 1500");
    CHECK(Mixer::commandToUs(1.0f) == ESC_US_MAX && Mixer::commandToUs(-1.0f) == ESC_US_MIN,
          "u=±1 должны давать ESC_US_MAX/ESC_US_MIN");
    CHECK(Mixer::commandToUs(5.0f) == ESC_US_MAX, "u=5 должен ограничиваться ESC_US_MAX");
    Mixer m;
    uint16_t o[ESC_COUNT];
    run(m, 0.08f, 0, 0, 0, 1, o);
    CHECK(o[0] == 1500, "мёртвая зона через микшер (0.08*0.5=0.04): M0=%u вместо 1500", o[0]);
}

// ---------------------------------------------------------------- парсер ----

static ControlPacket mk(int8_t s, int8_t y, int8_t h, int8_t w, uint8_t f) {
    ControlPacket p;
    p.surge = s; p.yaw = y; p.heave = h; p.sway = w; p.flags = f;
    controlPacketSeal(p);
    return p;
}

static int feedBytes(PacketParser &pp, const uint8_t *b, size_t n, ControlPacket *last) {
    int got = 0;
    ControlPacket o;
    for (size_t i = 0; i < n; i++) {
        if (pp.feed(b[i], o)) {
            got++;
            if (last) *last = o;
        }
    }
    return got;
}

static void test_packet_parser(void) {
    PacketParser pp;
    ControlPacket last;
    ControlPacket p = mk(10, -20, 30, -40, 3);
    uint8_t buf[64];
    size_t n = 0;
    const uint8_t junk[] = {0x00, 0xAA, 0x12, 0x55, 0xAA, 0xAA, 0xFF};
    memcpy(buf, junk, sizeof(junk)); n = sizeof(junk);
    memcpy(buf + n, &p, 8); n += 8;
    CHECK(feedBytes(pp, buf, n, &last) == 1 && memcmp(&last, &p, 8) == 0, "парсер: пакет после мусора не принят");
    const uint8_t *pb = (const uint8_t *)&p;
    CHECK(feedBytes(pp, pb, 3, nullptr) == 0, "парсер: часть 1 пакета не должна давать пакет");
    CHECK(feedBytes(pp, pb + 3, 2, nullptr) == 0, "парсер: часть 2 пакета не должна давать пакет");
    CHECK(feedBytes(pp, pb + 5, 3, &last) == 1, "парсер: разрезанный пакет не собран");
    ControlPacket bad = p;
    bad.crc ^= 0x01;
    uint32_t ce = pp.crcErrors();
    CHECK(feedBytes(pp, (uint8_t *)&bad, 8, nullptr) == 0 && pp.crcErrors() == ce + 1,
          "парсер: пакет с плохим CRC принят или не посчитан");
    ControlPacket r = mk(101, 0, 0, 0, 1);
    uint32_t re = pp.rangeErrors();
    CHECK(feedBytes(pp, (uint8_t *)&r, 8, nullptr) == 0 && pp.rangeErrors() == re + 1,
          "парсер: ось 101 принята или не посчитана");
    ControlPacket r2 = mk(0, 0, 0, -101, 1);
    CHECK(feedBytes(pp, (uint8_t *)&r2, 8, nullptr) == 0, "парсер: ось -101 принята");
    ControlPacket q = mk(-100, 100, 0, 0, 0);
    memcpy(buf, &p, 8); memcpy(buf + 8, &q, 8); n = 16;
    CHECK(feedBytes(pp, buf, n, &last) == 2 && memcmp(&last, &q, 8) == 0, "парсер: два пакета подряд не приняты");
    n = 0; buf[n++] = 0xAA; buf[n++] = 0x55; buf[n++] = 0x01; memcpy(buf + n, &q, 8); n += 8;
    CHECK(feedBytes(pp, buf, n, &last) == 1 && memcmp(&last, &q, 8) == 0,
          "парсер: не найден пакет внутри битого (ресинхронизация)");
    n = 0; buf[n++] = 0xAA; memcpy(buf + n, &p, 8); n += 8;
    CHECK(feedBytes(pp, buf, n, &last) == 1, "парсер: последовательность AA AA 55 не обработана");
    INFO("парсер: валидных=%u, ошибок CRC=%u, вне диапазона=%u", (unsigned)pp.validCount(),
         (unsigned)pp.crcErrors(), (unsigned)pp.rangeErrors());
}

// ----------------------------------------------------------- супервизор -----

static void test_link_supervisor(void) {
    LinkSupervisor s;
    s.begin(1000);
    ControlPacket on = mk(50, 0, 0, 0, CTRL_FLAG_ARMED | CTRL_FLAG_LIGHT);
    ControlPacket off = mk(50, 0, 0, 0, 0);
    uint32_t t = 1000;
    for (; t < 1000 + ESC_ARM_TIME_MS; t += 50) {
        s.onPacket(on, t); s.tick(t);
        CHECK(!s.armed(), "fail-safe: armed в окне старта (t=%u)", (unsigned)t);
    }
    CHECK(s.reason() == DisarmReason::Boot, "fail-safe: причина в окне старта должна быть Boot");
    CHECK(s.light(), "fail-safe: подсветка должна следовать пакетам и в окне старта");
    for (int i = 0; i < 10; i++, t += 50) {
        s.onPacket(on, t); s.tick(t);
        CHECK(!s.armed(), "fail-safe: взвод без фронта после окна старта");
    }
    CHECK(s.reason() == DisarmReason::WaitEdge, "fail-safe: причина должна быть WaitEdge");
    s.onPacket(off, t); s.tick(t); t += 50;
    CHECK(!s.armed() && s.reason() == DisarmReason::Command, "fail-safe: пакет armed=0 не дал disarm");
    s.onPacket(on, t); s.tick(t);
    CHECK(s.armed(), "fail-safe: фронт 0->1 не взвёл");
    uint32_t lastPkt = t;
    s.tick(lastPkt + FAILSAFE_TIMEOUT_MS);
    CHECK(s.armed(), "fail-safe: ровно на таймауте ещё должен быть armed");
    s.tick(lastPkt + FAILSAFE_TIMEOUT_MS + 1);
    CHECK(!s.armed() && s.reason() == DisarmReason::NoLink, "fail-safe: нет disarm после таймаута");
    CHECK(s.light(), "fail-safe: подсветка должна сохраниться при таймауте");
    t = lastPkt + 2000;
    s.onPacket(on, t); s.tick(t);
    CHECK(s.armed(), "fail-safe: после таймаута armed=1 должен взводить (детектор сброшен в 0)");
    t += 50; s.onPacket(off, t); s.tick(t);
    CHECK(!s.armed(), "fail-safe: armed=0 не дал disarm");
    t += 50; s.onPacket(on, t); s.tick(t);
    CHECK(s.armed(), "fail-safe: фронт после armed=0 не взвёл");
    t += 50; s.onPacket(on, t); s.tick(t);
    CHECK(s.armed(), "fail-safe: поток armed=1 должен удерживать armed");
    LinkSupervisor s2;
    s2.begin(0); s2.tick(100);
    CHECK(!s2.armed() && s2.reason() == DisarmReason::Boot, "fail-safe: без пакетов в окне старта — Boot");
    s2.tick(ESC_ARM_TIME_MS + 10);
    CHECK(s2.reason() == DisarmReason::NoLink, "fail-safe: после окна без пакетов — NoLink");
    s2.onPacket(on, ESC_ARM_TIME_MS + 20); s2.tick(ESC_ARM_TIME_MS + 20);
    CHECK(s2.armed(), "fail-safe: первый armed=1 после окна старта должен взводить");
    LinkSupervisor s3;
    s3.begin(0xFFFFF000u); s3.tick(0xFFFFF000u + 100);
    CHECK(!s3.armed(), "fail-safe: переполнение millis — окно старта");
    uint32_t tw = 0xFFFFF000u + ESC_ARM_TIME_MS + 10;
    s3.onPacket(on, tw); s3.tick(tw);
    CHECK(s3.armed(), "fail-safe: переполнение millis — взвод");
    s3.tick(tw + FAILSAFE_TIMEOUT_MS + 1);
    CHECK(!s3.armed(), "fail-safe: переполнение millis — таймаут");
}

// ---------------------------------------------------------------- MS5837 ----

// C1..C6 из примера даташита MS5837-30BA
static const uint16_t DS_PROM[7] = {0, 34982, 36352, 20328, 22354, 26646, 26146};

static void test_ms5837_compensation(void) {
    Ms5837Result r;
    ms5837Compute(DS_PROM, 4958179, 6815414, false, r);
    CHECK(r.tempCenti == 1981 && r.pressureDeci == 39998,
          "MS5837 пример даташита: T=%d (ожид. 1981), P=%d (ожид. 39998)", (int)r.tempCenti, (int)r.pressureDeci);
    // Эталон второго порядка — ref/ms5837_ref.py
    struct { uint32_t d1, d2; int32_t t, p; } ref[] = {
        {4958179, 6815414, 1981, 39998},   // 19.81 °C — малая поправка
        {4958179, 6400000, 624, 39339},    // ~6 °C, ветка TEMP < 20 °C
        {4958179, 5200000, -3972, 36761},  // < -15 °C, дополнительная поправка
        {4958179, 7300000, 3488, 40789},   // > 20 °C
        {5300000, 6700000, 1616, 62428},
    };
    for (auto &e : ref) {
        ms5837Compute(DS_PROM, e.d1, e.d2, true, r);
        CHECK(r.tempCenti == e.t && r.pressureDeci == e.p, "MS5837 второй порядок D2=%u: T=%d/%d, P=%d/%d",
              (unsigned)e.d2, (int)r.tempCenti, (int)e.t, (int)r.pressureDeci, (int)e.p);
    }
    CHECK(fabsf(ms5837DepthM(1013.25f + 97.7723f, 1013.25f) - 1.0f) < 1e-3f, "глубина: 97.77 мбар дали %.4f м вместо 1",
          ms5837DepthM(1013.25f + 97.7723f, 1013.25f));
    CHECK(ms5837DepthM(1000.0f, 1000.0f) == 0.0f, "глубина: P=P0 должна давать 0");
}

static void test_ms5837_crc4(void) {
    CHECK(ms5837Crc4(DS_PROM) == 2, "CRC4 для C1..C6 даташита = %u, ожидалось 2", ms5837Crc4(DS_PROM));
    for (auto &e : CRC_REF) {
        CHECK(ms5837Crc4(e.w) == e.crc, "CRC4 эталонного вектора: %u вместо %u", ms5837Crc4(e.w), e.crc);
    }
    uint16_t pr[7];
    memcpy(pr, DS_PROM, sizeof(pr));
    pr[0] = 0x0123;
    const uint8_t cr = ms5837Crc4(pr);
    pr[0] = (uint16_t)((cr << 12) | 0x0123);
    CHECK(ms5837Crc4(pr) == cr, "CRC4 не должен зависеть от собственного ниббла C0[15:12]");
    CHECK(ms5837PromValid(pr), "PROM с верным CRC4 признан невалидным");
    pr[3] ^= 0x0100;
    CHECK(!ms5837PromValid(pr), "CRC4 не обнаружил битовую ошибку в C3");
    pr[3] ^= 0x0100;
    pr[0] = (uint16_t)((((cr + 1u) & 0xFu) << 12) | 0x0123);
    CHECK(!ms5837PromValid(pr), "PROM с неверным CRC4 признан валидным");
    uint16_t zeros[7] = {0, 0, 0, 0, 0, 0, 0};
    CHECK(ms5837Crc4(zeros) == 0, "CRC4 нулевого PROM должен быть 0");
}

// -------------------------------------------------------------- ориентация ---

static void test_attitude(void) {
    CHECK(fabsf(wrap360(361.0f) - 1.0f) < 1e-4f && fabsf(wrap360(-1.0f) - 359.0f) < 1e-4f && wrap360(360.0f) == 0.0f,
          "wrap360 работает неверно");
    CHECK(fabsf(wrap180(181.0f) + 179.0f) < 1e-4f && fabsf(wrap180(-181.0f) - 179.0f) < 1e-4f && wrap180(180.0f) == 180.0f,
          "wrap180 работает неверно");
    const float d2r = 3.14159265f / 180.0f;
    float accR[3] = {0, sinf(30 * d2r), cosf(30 * d2r)};
    CHECK(fabsf(AttitudeFilter::accelRoll(accR) - 30.0f) < 0.01f, "крен на правый борт 30°: roll=%.2f вместо +30",
          AttitudeFilter::accelRoll(accR));
    float accP[3] = {sinf(20 * d2r), 0, cosf(20 * d2r)};
    CHECK(fabsf(AttitudeFilter::accelPitch(accP) - 20.0f) < 0.01f, "нос вверх 20°: pitch=%.2f вместо +20",
          AttitudeFilter::accelPitch(accP));
    float level[3] = {0, 0, 1}, zero[3] = {0, 0, 0};
    AttitudeFilter f;
    f.seed(level);
    float gx[3] = {10, 0, 0};
    f.update(level, gx, 0.1f);
    CHECK(f.roll() > 0.9f, "гироскоп +X (правый борт вниз) должен увеличивать roll: %.3f", f.roll());
    f.reset();
    float gy[3] = {0, 10, 0};
    f.update(level, gy, 0.1f);
    CHECK(f.pitch() < -0.9f, "гироскоп +Y (нос вниз) должен уменьшать pitch: %.3f", f.pitch());
    f.reset();
    float gzR[3] = {0, 0, -90};
    for (int i = 0; i < 50; i++) f.update(level, gzR, 0.02f);
    CHECK(fabsf(f.yaw() - 90.0f) < 0.01f, "поворот вправо на 90°: yaw=%.3f", f.yaw());
    f.reset();
    float gzL[3] = {0, 0, 50};
    f.update(level, gzL, 0.02f);
    CHECK(fabsf(f.yaw() - 359.0f) < 0.01f, "поворот влево с 0: yaw=%.3f вместо 359", f.yaw());
    f.reset();
    float gz1[3] = {0, 0, -359};
    f.update(level, gz1, 1.0f);
    CHECK(fabsf(f.yaw() - 359.0f) < 0.01f, "yaw после 359° вправо: %.3f", f.yaw());
    float gz2[3] = {0, 0, -2};
    f.update(level, gz2, 1.0f);
    CHECK(fabsf(f.yaw() - 1.0f) < 0.01f, "переход yaw 359 -> 1: %.3f", f.yaw());
    bool inRange = true;
    f.reset();
    for (int i = 0; i < 2000; i++) {
        f.update(level, gzR, 0.02f);
        if (f.yaw() < 0 || f.yaw() >= 360) inRange = false;
    }
    CHECK(inRange, "yaw вышел за диапазон [0, 360)");
    f.reset();
    for (int i = 0; i < 50; i++) f.update(accR, zero, 0.02f);
    float r1 = f.roll();
    for (int i = 0; i < 500; i++) f.update(accR, zero, 0.02f);
    CHECK(r1 > 15.0f && r1 < 25.0f && fabsf(f.roll() - 30.0f) < 0.1f,
          "комплементарный фильтр: за 1 с %.2f (ожид. 15..25), за 11 с %.2f (ожид. 30)", r1, f.roll());
    f.reset();
    f.seed(level);
    float gfast[3] = {100, 0, 0};
    f.update(level, gfast, 0.02f);
    CHECK(f.roll() > 1.9f, "на коротком интервале должен доминировать гироскоп: %.3f", f.roll());
    f.reset();
    float accUp[3] = {0, sinf(179 * d2r), cosf(179 * d2r)};
    f.seed(accUp);
    float accUp2[3] = {0, sinf(-179 * d2r), cosf(-179 * d2r)};
    for (int i = 0; i < 300; i++) f.update(accUp2, zero, 0.02f);
    CHECK(fabsf(fabsf(f.roll()) - 179.0f) < 0.5f, "коррекция roll около ±180 ушла через 0: %.2f", f.roll());
    const uint8_t map[3] = {1, 0, 2};
    const int8_t sg[3] = {-1, 1, 1};
    float in[3] = {1, 2, 3}, out[3];
    remapAxes(in, map, sg, out);
    CHECK(out[0] == -2 && out[1] == 1 && out[2] == 3, "перестановка осей IMU работает неверно");
    GyroBiasEstimator b;
    float s1[3] = {1, -2, 0.5f}, s2[3] = {3, -4, 1.5f}, bo[3];
    b.add(s1); b.add(s2); b.bias(bo);
    CHECK(bo[0] == 2 && bo[1] == -3 && bo[2] == 1, "среднее смещения гироскопа посчитано неверно");
}

// ------------------------------------------------------------------ VBAT ----

static void test_vbat(void) {
    float mv = vbatToPinMv(16.8f);
    CHECK(fabsf(mv - 3029.5f) < 0.5f, "VBAT 16.8 В -> %.1f мВ на пине, ожидалось 3029.5", mv);
    CHECK(fabsf(vbatFromPinMv(3030) - 16.8027f) < 0.01f, "VBAT 3030 мВ -> %.3f В, ожидалось 16.803", vbatFromPinMv(3030));
    CHECK(fabsf(vbatToPinMv(13.2f) - 2380.3f) < 0.5f, "VBAT 13.2 В -> %.1f мВ, ожидалось 2380.3", vbatToPinMv(13.2f));
    MovingAverage a;
    CHECK(a.value() == 0.0f, "пустое скользящее среднее должно быть 0");
    a.add(10); a.add(20);
    CHECK(a.value() == 15.0f, "среднее 10 и 20 должно быть 15, получено %.3f", a.value());
    for (int i = 0; i < 100; i++) a.add(16.0f);
    CHECK(a.value() == 16.0f && a.count() == VBAT_AVG_WINDOW, "окно скользящего среднего работает неверно");
}

// ------------------------------------------------------------- контроль -----

static void test_all_checks_ran(void) {
    snprintf(g_msg, sizeof(g_msg), "выполнено проверок %d, ожидалось %d", g_checks, EXPECTED_CHECKS);
    TEST_ASSERT_EQUAL_INT_MESSAGE(EXPECTED_CHECKS, g_checks, g_msg);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_mixer_signs);
    RUN_TEST(test_mixer_normalization);
    RUN_TEST(test_mixer_slew);
    RUN_TEST(test_mixer_deadband);
    RUN_TEST(test_packet_parser);
    RUN_TEST(test_link_supervisor);
    RUN_TEST(test_ms5837_compensation);
    RUN_TEST(test_ms5837_crc4);
    RUN_TEST(test_attitude);
    RUN_TEST(test_vbat);
    RUN_TEST(test_all_checks_ran);
    return UNITY_END();
}
