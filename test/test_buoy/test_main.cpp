// =============================================================================
//  Хост-тесты чистых модулей буя (env:native, Unity):
//    cmd_codec      — контракт №1 -> ControlPacket (контракт №2);
//    telemetry_line — сборщик строк телеметрии (контракт №3);
//    video_parser   — потоковый разбор видеокадров (контракт №4) против
//                     эталона tools/cam_rx.py (фикстуры в fixtures/).
//  Запуск: pio test -e native -f test_buoy
// =============================================================================
#include <unity.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <memory>
#include <string>
#include <vector>

#include "buoy/cmd_codec.h"
#include "buoy/telemetry_line.h"
#include "buoy/video_parser.h"

void setUp() {}
void tearDown() {}

// -----------------------------------------------------------------------------
//  Вспомогательное
// -----------------------------------------------------------------------------
static CmdParseResult parse(const char *s, BuoyCmd &c) {
    return parseCmdJson(reinterpret_cast<const uint8_t *>(s), strlen(s), c);
}

static void feedLines(TelemetryLineAssembler &a, const std::string &s, std::vector<std::string> &out) {
    for (unsigned char ch : s) {
        if (a.push(ch)) out.emplace_back(a.line(), a.length());
    }
}

// Каталог фикстур. __FILE__ в env:native относительный (от корня проекта, где
// pio запускает компилятор и тест), поэтому пробуем: путь из __FILE__ как есть,
// затем от корня проекта, вычисленного по argv[0] (.pio/build/native/program).
static std::string gProjectRoot;

static std::string dirOf(const std::string &path) {
    const size_t slash = path.find_last_of("/\\");
    return (slash == std::string::npos) ? std::string(".") : path.substr(0, slash);
}

static std::string fixturePath(const char *name) {
    const std::string rel = dirOf(__FILE__) + "/fixtures/" + name;
    FILE *f = fopen(rel.c_str(), "rb");
    if (f != nullptr) {
        fclose(f);
        return rel;
    }
    const bool absolute = !rel.empty() && (rel[0] == '/' || rel[0] == '\\');
    if (!absolute && !gProjectRoot.empty()) return gProjectRoot + "/" + rel;
    return rel;
}

static bool readFile(const std::string &path, std::string &out) {
    FILE *f = fopen(path.c_str(), "rb");
    if (f == nullptr) return false;
    char buf[4096];
    size_t n;
    out.clear();
    while ((n = fread(buf, 1, sizeof buf, f)) > 0) out.append(buf, n);
    fclose(f);
    return true;
}

static uint32_t crc32(const uint8_t *d, size_t n) {
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) {
        c ^= d[i];
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
    }
    return ~c;
}

// =============================================================================
//  cmd_codec
// =============================================================================
static void test_cmd_valid_packet_bytes_and_crc() {
    BuoyCmd c{};
    ControlPacket p{};
    TEST_ASSERT_TRUE_MESSAGE(parse("{\"type\":\"cmd\",\"armed\":true,\"light\":true,"
                                   "\"axes\":{\"surge\":0.5,\"yaw\":-0.25,\"heave\":1,\"sway\":-1}}", c) == CmdParseResult::Ok,
                             "валидный cmd должен разбираться");
    buildControlPacket(c, true, p);
    const uint8_t exp[8] = {0xAA, 0x55, 50, static_cast<uint8_t>(-25), 100, static_cast<uint8_t>(-100), 0x03, 0xD1};
    TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE(exp, reinterpret_cast<const uint8_t *>(&p), 7, "байты 0..6 пакета");
    uint8_t x = 0;
    for (int i = 0; i < 7; i++) x ^= exp[i];
    TEST_ASSERT_EQUAL_HEX8_MESSAGE(x, p.crc, "CRC = XOR байт 0..6");
    TEST_ASSERT_EQUAL_HEX8_MESSAGE(exp[7], p.crc, "CRC эталонного пакета D1");
    TEST_ASSERT_TRUE_MESSAGE(controlPacketValid(p), "пакет проходит controlPacketValid");
}

static void test_cmd_timeout_keeps_light() {
    BuoyCmd c{};
    ControlPacket p{};
    TEST_ASSERT_TRUE(parse("{\"type\":\"cmd\",\"armed\":true,\"light\":true,"
                           "\"axes\":{\"surge\":0.5,\"yaw\":-0.25,\"heave\":1,\"sway\":-1}}", c) == CmdParseResult::Ok);
    buildControlPacket(c, false, p);
    TEST_ASSERT_TRUE_MESSAGE(p.surge == 0 && p.yaw == 0 && p.heave == 0 && p.sway == 0 && p.flags == CTRL_FLAG_LIGHT &&
                                 controlPacketValid(p),
                             "таймаут: armed=0, оси 0, light сохраняется");
}

static void test_cmd_rounding_and_clamp() {
    BuoyCmd c{};
    ControlPacket p{};
    TEST_ASSERT_TRUE(parse("{\"type\":\"cmd\",\"armed\":false,\"light\":false,"
                           "\"axes\":{\"surge\":1.5,\"yaw\":-0.004,\"heave\":0.555,\"sway\":-7}}", c) == CmdParseResult::Ok);
    buildControlPacket(c, true, p);
    TEST_ASSERT_EQUAL_INT8_MESSAGE(100, p.surge, "1.5 -> 100");
    TEST_ASSERT_EQUAL_INT8_MESSAGE(0, p.yaw, "-0.004 -> 0");
    TEST_ASSERT_EQUAL_INT8_MESSAGE(56, p.heave, "0.555 -> 56");
    TEST_ASSERT_EQUAL_INT8_MESSAGE(-100, p.sway, "-7 -> -100");
    TEST_ASSERT_EQUAL_HEX8_MESSAGE(0, p.flags, "флаги сброшены");
    TEST_ASSERT_EQUAL_INT8(100, axisToWire(1.5f));
    TEST_ASSERT_EQUAL_INT8(0, axisToWire(-0.004f));
    TEST_ASSERT_EQUAL_INT8(56, axisToWire(0.555f));
    TEST_ASSERT_EQUAL_INT8(-56, axisToWire(-0.555f));
    TEST_ASSERT_EQUAL_INT8(1, axisToWire(0.005f));
    TEST_ASSERT_EQUAL_INT8(0, axisToWire(-0.004f));
    TEST_ASSERT_EQUAL_INT8_MESSAGE(0, axisToWire(NAN), "NaN -> 0");
    TEST_ASSERT_EQUAL_INT8(100, axisToWire(1e30f));
    TEST_ASSERT_EQUAL_INT8(-100, axisToWire(-1e30f));
    TEST_ASSERT_TRUE(parse("{\"type\":\"cmd\",\"armed\":true,\"light\":false,"
                           "\"axes\":{\"surge\":1e30,\"yaw\":0,\"heave\":0,\"sway\":0}}", c) == CmdParseResult::Ok);
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(1.0f, c.axes[AX_SURGE], "огромное число в JSON обрезается до 1");
}

static void test_cmd_rejects_keep_previous() {
    BuoyCmd c{};
    TEST_ASSERT_TRUE(parse("{\"type\":\"cmd\",\"armed\":true,\"light\":false,"
                           "\"axes\":{\"surge\":1,\"yaw\":0,\"heave\":0,\"sway\":0}}", c) == CmdParseResult::Ok);
    const BuoyCmd before = c;
    struct Case {
        const char *json;
        CmdParseResult want;
        const char *msg;
    };
    const Case cases[] = {
        {"{\"type\":\"ping\",\"armed\":true,\"light\":false,\"axes\":{\"surge\":0,\"yaw\":0,\"heave\":0,\"sway\":0}}",
         CmdParseResult::BadType, "чужой type"},
        {"{\"armed\":true,\"light\":false,\"axes\":{\"surge\":0,\"yaw\":0,\"heave\":0,\"sway\":0}}", CmdParseResult::BadType,
         "нет type"},
        {"{\"type\":5,\"armed\":true,\"light\":false,\"axes\":{\"surge\":0,\"yaw\":0,\"heave\":0,\"sway\":0}}",
         CmdParseResult::BadType, "type-число"},
        {"{\"type\":\"cmd\",\"armed\":true,\"light\":fal", CmdParseResult::BadJson, "обрезанный JSON"},
        {"not json", CmdParseResult::BadJson, "не JSON"},
        {"[1,2,3]", CmdParseResult::BadJson, "массив вместо объекта"},
        {"", CmdParseResult::BadJson, "пустая строка"},
        {"{\"type\":\"cmd\",\"armed\":true,\"light\":false}", CmdParseResult::BadField, "нет axes"},
        {"{\"type\":\"cmd\",\"armed\":true,\"light\":false,\"axes\":{\"surge\":0,\"yaw\":0,\"heave\":0}}",
         CmdParseResult::BadField, "нет одной оси"},
        {"{\"type\":\"cmd\",\"armed\":true,\"light\":false,\"axes\":{\"surge\":\"0\",\"yaw\":0,\"heave\":0,\"sway\":0}}",
         CmdParseResult::BadField, "ось-строка"},
        {"{\"type\":\"cmd\",\"armed\":true,\"light\":false,\"axes\":{\"surge\":null,\"yaw\":0,\"heave\":0,\"sway\":0}}",
         CmdParseResult::BadField, "ось null"},
        {"{\"type\":\"cmd\",\"armed\":1,\"light\":false,\"axes\":{\"surge\":0,\"yaw\":0,\"heave\":0,\"sway\":0}}",
         CmdParseResult::BadField, "armed не bool"},
        {"{\"type\":\"cmd\",\"light\":false,\"axes\":{\"surge\":0,\"yaw\":0,\"heave\":0,\"sway\":0}}", CmdParseResult::BadField,
         "нет armed"},
        {"{\"type\":\"cmd\",\"armed\":true,\"light\":false,\"axes\":[0,0,0,0]}", CmdParseResult::BadField, "axes-массив"},
    };
    for (const Case &k : cases) {
        TEST_ASSERT_TRUE_MESSAGE(parse(k.json, c) == k.want, k.msg);
    }
    TEST_ASSERT_TRUE_MESSAGE(
        parse("{\"type\":\"cmd\",\"armed\":true,\"light\":false,\"axes\":{\"surge\":NaN,\"yaw\":0,\"heave\":0,\"sway\":0}}", c) !=
            CmdParseResult::Ok,
        "NaN в оси отклоняется");
    TEST_ASSERT_EQUAL_MEMORY_MESSAGE(&before, &c, sizeof c, "при отказе последняя команда не меняется");
}

static void test_cmd_arena_big_message_and_repeat() {
    BuoyCmd c{};
    const std::string big = "{\"type\":\"cmd\",\"pad\":\"" + std::string(20000, 'x') +
                            "\",\"armed\":true,\"light\":false,\"axes\":{\"surge\":0,\"yaw\":0,\"heave\":0,\"sway\":0}}";
    TEST_ASSERT_TRUE_MESSAGE(parseCmdJson(reinterpret_cast<const uint8_t *>(big.data()), big.size(), c) == CmdParseResult::BadJson,
                             "сообщение 20 КБ не помещается в арену -> отказ");
    TEST_ASSERT_TRUE_MESSAGE(parse("{\"type\":\"cmd\",\"armed\":true,\"light\":false,"
                                   "\"axes\":{\"surge\":0.1,\"yaw\":0,\"heave\":0,\"sway\":0}}", c) == CmdParseResult::Ok,
                             "после большого сообщения разбор снова работает");
    bool allOk = true;
    for (int i = 0; i < 100000 && allOk; i++) {
        allOk = parse("{\"type\":\"cmd\",\"armed\":true,\"light\":true,"
                      "\"axes\":{\"surge\":0.3,\"yaw\":0.2,\"heave\":-0.1,\"sway\":0}}", c) == CmdParseResult::Ok;
    }
    TEST_ASSERT_TRUE_MESSAGE(allOk, "100000 разборов подряд: арена сбрасывается");
    TEST_ASSERT_TRUE_MESSAGE(cmdArenaHighWater() > 0, "арена использовалась");
}

// =============================================================================
//  telemetry_line
// =============================================================================
static void test_telemetry_chunks_and_crlf() {
    TelemetryLineAssembler a;
    std::vector<std::string> lines;
    feedLines(a, "{\"depth\":1.2", lines);
    TEST_ASSERT_TRUE_MESSAGE(lines.empty(), "кусок 1 без \\n — строки нет");
    feedLines(a, "3,\"temp\":18.4}", lines);
    TEST_ASSERT_TRUE_MESSAGE(lines.empty(), "кусок 2 без \\n — строки нет");
    feedLines(a, "\r\n", lines);
    TEST_ASSERT_TRUE_MESSAGE(lines.size() == 1 && lines[0] == "{\"depth\":1.23,\"temp\":18.4}", "строка из трёх кусков, \\r отброшен");
    lines.clear();
    feedLines(a, "A\nB\r\n\n\r\nC\n", lines);
    TEST_ASSERT_TRUE_MESSAGE(lines.size() == 3 && lines[0] == "A" && lines[1] == "B" && lines[2] == "C",
                             "несколько строк в куске, пустые пропущены");
}

static void test_telemetry_overflow() {
    TelemetryLineAssembler a;
    std::vector<std::string> lines;
    feedLines(a, std::string(200, 'z'), lines);
    TEST_ASSERT_TRUE_MESSAGE(lines.empty(), "200 байт без \\n — строки нет");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(1, a.overflows(), "одно переполнение");
    feedLines(a, "zzzz\n", lines);
    TEST_ASSERT_TRUE_MESSAGE(lines.empty(), "хвост переполненной строки выброшен");
    feedLines(a, "{\"ok\":1}\n", lines);
    TEST_ASSERT_TRUE_MESSAGE(lines.size() == 1 && lines[0] == "{\"ok\":1}", "следующая строка проходит");
}

static void test_telemetry_length_boundary() {
    TelemetryLineAssembler a;
    std::vector<std::string> lines;
    feedLines(a, std::string(TELEMETRY_MAX_LEN - 1, 'q') + "\n", lines);
    TEST_ASSERT_TRUE_MESSAGE(lines.size() == 1 && lines[0].size() == TELEMETRY_MAX_LEN - 1, "159 символов + \\n проходят");
    lines.clear();
    feedLines(a, std::string(TELEMETRY_MAX_LEN, 'q') + "\nX\n", lines);
    TEST_ASSERT_TRUE_MESSAGE(lines.size() == 1 && lines[0] == "X", "160 символов — переполнение, следующая строка проходит");
    TEST_ASSERT_EQUAL_UINT32(1, a.overflows());
}

static void test_telemetry_real_format() {
    TelemetryLineAssembler a;
    std::vector<std::string> lines;
    char buf[TELEMETRY_MAX_LEN + 1];
    const int n = snprintf(buf, sizeof buf, TELEMETRY_JSON_FMT, 1.23, 18.4, 15.62, -2.1, 0.8, 134.5, 1);
    TEST_ASSERT_TRUE(n > 1);
    feedLines(a, std::string(buf, static_cast<size_t>(n)), lines);
    TEST_ASSERT_TRUE_MESSAGE(lines.size() == 1 && lines[0] == std::string(buf, static_cast<size_t>(n) - 1U),
                             "строка TELEMETRY_JSON_FMT v2 (7 ключей) пересылается без \\n");
    TEST_ASSERT_TRUE_MESSAGE(lines[0].find("\"armed\":1}") != std::string::npos, "ключ armed v2 сохраняется как есть");
}

static void test_telemetry_v2_worst_length() {
    // Худшая правдоподобная строка v2: все поля максимальной ширины со знаком
    TelemetryLineAssembler a;
    std::vector<std::string> lines;
    char buf[2 * TELEMETRY_MAX_LEN];
    const int n = snprintf(buf, sizeof buf, TELEMETRY_JSON_FMT, -999.99, -999.9, 999.99, -180.0, -90.0, 359.9, 1);
    TEST_ASSERT_TRUE_MESSAGE(n > 1 && static_cast<size_t>(n) <= TELEMETRY_MAX_LEN,
                             "худшая строка v2 укладывается в TELEMETRY_MAX_LEN вместе с \\n");
    feedLines(a, std::string(buf, static_cast<size_t>(n)), lines);
    TEST_ASSERT_TRUE_MESSAGE(lines.size() == 1 && lines[0].size() == static_cast<size_t>(n) - 1U,
                             "худшая строка v2 проходит сборщик целиком");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(0, a.overflows(), "без переполнения");

    // Мусор датчика (огромные числа) даёт строку длиннее лимита: она выбрасывается
    // целиком, а не обрезается, и следующая нормальная строка проходит
    const int big = snprintf(buf, sizeof buf, TELEMETRY_JSON_FMT, 1e30, 1e30, 1e30, 1e30, 1e30, 1e30, 0);
    TEST_ASSERT_TRUE(big > static_cast<int>(TELEMETRY_MAX_LEN));
    lines.clear();
    feedLines(a, std::string(buf, strnlen(buf, sizeof buf)) + "\n", lines);
    feedLines(a, "{\"ok\":2}\n", lines);
    TEST_ASSERT_TRUE_MESSAGE(lines.size() == 1 && lines[0] == "{\"ok\":2}", "слишком длинная строка выброшена целиком");
    TEST_ASSERT_EQUAL_UINT32(1, a.overflows());
}

// =============================================================================
//  video_parser против эталона cam_rx.py
// =============================================================================
static void onFrame(void *ctx, const uint8_t *jpeg, size_t len, uint16_t seq) {
    char line[48];
    snprintf(line, sizeof line, "%u %zu %08x\n", static_cast<unsigned>(seq), len, static_cast<unsigned>(crc32(jpeg, len)));
    static_cast<std::string *>(ctx)->append(line);
}

// Режим нарезки: fixed > 0 — порции фиксированного размера; иначе случайные
// 0..4999 Б из детерминированного LCG с данным seed (включая пустые порции).
static std::string runParser(const std::string &data, size_t fixed, uint32_t seed) {
    std::string out;
    std::unique_ptr<VideoFrameParser> p(new VideoFrameParser());
    p->setCallback(onFrame, &out);
    uint32_t lcg = seed;
    size_t pos = 0;
    const uint8_t *d = reinterpret_cast<const uint8_t *>(data.data());
    while (pos < data.size()) {
        size_t n = fixed;
        if (fixed == 0) {
            lcg = lcg * 1664525u + 1013904223u;
            n = (lcg >> 8) % 5000u;
        }
        if (n > data.size() - pos) n = data.size() - pos;
        p->feed(d + pos, n);
        pos += n;
    }
    const VideoParserStats &s = p->stats();
    char line[96];
    snprintf(line, sizeof line, "STATS ok=%u bad_hdr=%u bad_crc=%u junk=%u\n", static_cast<unsigned>(s.framesOk),
             static_cast<unsigned>(s.badHeader), static_cast<unsigned>(s.badCrc), static_cast<unsigned>(s.junkBytes));
    out.append(line);
    return out;
}

static void checkStream(const char *bin, const char *expected, unsigned minFrames) {
    std::string data, want;
    const std::string binPath = fixturePath(bin);
    const std::string expPath = fixturePath(expected);
    TEST_ASSERT_TRUE_MESSAGE(readFile(binPath, data), ("нет фикстуры " + binPath).c_str());
    TEST_ASSERT_TRUE_MESSAGE(readFile(expPath, want), ("нет фикстуры " + expPath).c_str());
    unsigned frames = 0;
    for (char ch : want) frames += (ch == '\n') ? 1U : 0U;
    TEST_ASSERT_TRUE_MESSAGE(frames > minFrames, "в ожидаемом результате есть кадры");

    // 16 режимов: 11 фиксированных размеров порций + 5 случайных серий
    const size_t fixedSizes[] = {1, 2, 3, 7, 10, 11, 12, 13, 64, 4096, 1000000};
    for (size_t sz : fixedSizes) {
        char msg[128];
        snprintf(msg, sizeof msg, "%s: порции по %zu Б — расхождение с cam_rx.py", bin, sz);
        TEST_ASSERT_EQUAL_STRING_MESSAGE(want.c_str(), runParser(data, sz, 0).c_str(), msg);
    }
    for (uint32_t seed = 1; seed <= 5; seed++) {
        char msg[128];
        snprintf(msg, sizeof msg, "%s: случайные порции, seed %u — расхождение с cam_rx.py", bin, static_cast<unsigned>(seed));
        TEST_ASSERT_EQUAL_STRING_MESSAGE(want.c_str(), runParser(data, 0, seed).c_str(), msg);
    }
}

static void test_video_cam_stream_matches_reference() { checkStream("cam_stream.bin", "cam_stream.expected", 10); }

static void test_video_extra_stream_matches_reference() { checkStream("extra.bin", "extra.expected", 16); }

static void test_video_max_frame_boundary() {
    // Прямая проверка границы длины без фикстур: 32768 принимается, 32769 — плохой заголовок
    for (uint32_t len : {static_cast<uint32_t>(VIDEO_MAX_FRAME), static_cast<uint32_t>(VIDEO_MAX_FRAME + 1U)}) {
        VideoFrameHeader h{};
        h.seq = 1;
        h.length = len;
        videoHeaderSeal(h);
        std::string s(reinterpret_cast<const char *>(&h), sizeof h);
        std::string jpeg(len, '\x11');
        s += jpeg;
        const uint16_t crc = crc16Update(CRC16_INIT, reinterpret_cast<const uint8_t *>(jpeg.data()), jpeg.size());
        s += static_cast<char>(crc & 0xFF);
        s += static_cast<char>(crc >> 8);
        std::string out = runParser(s, 4096, 0);
        if (len == VIDEO_MAX_FRAME) {
            TEST_ASSERT_TRUE_MESSAGE(out.rfind("1 32768 ", 0) == 0, "кадр 32768 Б принимается");
        } else {
            TEST_ASSERT_TRUE_MESSAGE(out.find("ok=0 bad_hdr=1") != std::string::npos, "длина 32769 — плохой заголовок");
        }
    }
}

int main(int argc, char **argv) {
    // argv[0] = <корень>/.pio/build/native/program -> корень на 4 уровня выше
    if (argc > 0 && argv[0] != nullptr) {
        std::string root = argv[0];
        for (int i = 0; i < 4; i++) root = dirOf(root);
        gProjectRoot = root;
    }
    UNITY_BEGIN();
    RUN_TEST(test_cmd_valid_packet_bytes_and_crc);
    RUN_TEST(test_cmd_timeout_keeps_light);
    RUN_TEST(test_cmd_rounding_and_clamp);
    RUN_TEST(test_cmd_rejects_keep_previous);
    RUN_TEST(test_cmd_arena_big_message_and_repeat);
    RUN_TEST(test_telemetry_chunks_and_crlf);
    RUN_TEST(test_telemetry_overflow);
    RUN_TEST(test_telemetry_length_boundary);
    RUN_TEST(test_telemetry_real_format);
    RUN_TEST(test_telemetry_v2_worst_length);
    RUN_TEST(test_video_cam_stream_matches_reference);
    RUN_TEST(test_video_extra_stream_matches_reference);
    RUN_TEST(test_video_max_frame_boundary);
    return UNITY_END();
}
