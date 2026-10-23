// =============================================================================
//  Хост-тесты сборки видеокадра камеры (контракт №4, docs/ICD.md §7).
//  Запуск: pio test -e native -f test_cam
// =============================================================================
#include <unity.h>

#include <string.h>

#include "protocol.h"
#include "cam/video_frame.h"

void setUp() {}
void tearDown() {}

static const uint8_t REF_DATA[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};

// CRC16-CCITT/FALSE: эталон для "123456789" и потоковый расчёт по частям
static void test_crc16_reference_and_streaming() {
    const uint16_t crc = videoFrameCrc(REF_DATA, sizeof(REF_DATA));
    TEST_ASSERT_EQUAL_HEX16_MESSAGE(0x29B1, crc, "CRC16(\"123456789\") должен быть 0x29B1");

    uint16_t s = CRC16_INIT;
    s = crc16Update(s, REF_DATA, 4);
    s = crc16Update(s, REF_DATA + 4, 5);
    TEST_ASSERT_EQUAL_HEX16_MESSAGE(crc, s, "потоковый CRC16 по частям не совпал с разовым");
}

// Хвост кадра: CRC16 младшим байтом вперёд
static void test_trailer_little_endian() {
    uint8_t tr[VIDEO_TRAILER_LEN];
    videoFrameBuildTrailer(0x29B1, tr);
    TEST_ASSERT_EQUAL_HEX8_MESSAGE(0xB1, tr[0], "первый байт хвоста — младший байт CRC");
    TEST_ASSERT_EQUAL_HEX8_MESSAGE(0x29, tr[1], "второй байт хвоста — старший байт CRC");
}

// Заголовок: поля seq/length/flags, магия и hcrc побайтно
static void test_header_layout() {
    VideoFrameHeader h;
    memset(&h, 0xEE, sizeof(h));
    TEST_ASSERT_TRUE_MESSAGE(videoFrameBuildHeader(0x1234, 9, h), "заголовок для length=9 не собран");
    TEST_ASSERT_TRUE_MESSAGE(videoHeaderValid(h), "собранный заголовок не проходит videoHeaderValid");

    const uint8_t expect[9] = {0xA5, 0x5A, 0x34, 0x12, 0x09, 0x00, 0x00, 0x00, 0x00};
    uint8_t x = 0;
    for (size_t i = 0; i < sizeof(expect); i++) x ^= expect[i];
    const uint8_t *b = reinterpret_cast<const uint8_t *>(&h);
    TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE(expect, b, 9, "байты 0..8 заголовка не совпали с ICD §7");
    TEST_ASSERT_EQUAL_HEX8_MESSAGE(x, b[9], "hcrc должен быть XOR байт 0..8");
}

// Границы length: 0 и > VIDEO_MAX_FRAME отвергаются, 1 и VIDEO_MAX_FRAME принимаются
static void test_length_bounds() {
    VideoFrameHeader h;
    TEST_ASSERT_TRUE_MESSAGE(videoFrameBuildHeader(1, 1, h) && videoHeaderValid(h), "length=1 должен приниматься");
    TEST_ASSERT_TRUE_MESSAGE(videoFrameBuildHeader(1, VIDEO_MAX_FRAME, h) && videoHeaderValid(h),
                             "length=32768 должен приниматься");
    VideoFrameHeader h2 = h;
    TEST_ASSERT_FALSE_MESSAGE(videoFrameBuildHeader(2, 0, h2), "length=0 должен отвергаться");
    TEST_ASSERT_FALSE_MESSAGE(videoFrameBuildHeader(3, VIDEO_MAX_FRAME + 1, h2), "length>32768 должен отвергаться");
    TEST_ASSERT_FALSE_MESSAGE(videoFrameBuildHeader(4, 32769, h2), "length=32769 должен отвергаться");

    // Вручную запечатанные заголовки с недопустимой длиной отвергает проверка буя
    VideoFrameHeader z = {};
    z.length = 0;
    videoHeaderSeal(z);
    TEST_ASSERT_FALSE_MESSAGE(videoHeaderValid(z), "заголовок с length=0 прошёл videoHeaderValid");
    z.length = 40000;
    videoHeaderSeal(z);
    TEST_ASSERT_FALSE_MESSAGE(videoHeaderValid(z), "заголовок с length=40000 прошёл videoHeaderValid");
}

// Испорченный заголовок не проходит проверку
static void test_corrupted_header_rejected() {
    VideoFrameHeader h;
    TEST_ASSERT_TRUE(videoFrameBuildHeader(7, 100, h));
    h.seq ^= 1;
    TEST_ASSERT_FALSE_MESSAGE(videoHeaderValid(h), "заголовок с изменённым seq прошёл проверку hcrc");
}

// -----------------------------------------------------------------------------
//  Совместимость с буем: кадры, собранные функциями прошивки в один буфер,
//  разбираются проверками protocol.h (videoHeaderValid + crc16Update).
// -----------------------------------------------------------------------------
static size_t buildFrame(uint8_t *out, uint16_t seq, const uint8_t *jpeg, uint32_t len) {
    VideoFrameHeader h;
    if (!videoFrameBuildHeader(seq, len, h)) return 0;
    uint8_t tr[VIDEO_TRAILER_LEN];
    videoFrameBuildTrailer(videoFrameCrc(jpeg, len), tr);
    memcpy(out, &h, sizeof(h));
    memcpy(out + sizeof(h), jpeg, len);
    memcpy(out + sizeof(h) + len, tr, VIDEO_TRAILER_LEN);
    return sizeof(h) + len + VIDEO_TRAILER_LEN;
}

static void test_buoy_compatibility() {
    static uint8_t jpegs[3][700];
    static const uint32_t lens[3] = {600, 1, 700};
    static const uint16_t seqs[3] = {777, 778, 0xFFFF};
    for (int k = 0; k < 3; k++) {
        for (uint32_t i = 0; i < lens[k]; i++) jpegs[k][i] = static_cast<uint8_t>(i * 37 + 11 + k);
        if (lens[k] >= 4) {
            jpegs[k][0] = 0xFF;
            jpegs[k][1] = 0xD8;
            jpegs[k][lens[k] - 2] = 0xFF;
            jpegs[k][lens[k] - 1] = 0xD9;
        }
        if (lens[k] > 102) {
            jpegs[k][100] = VIDEO_MAGIC1;   // ложная магия внутри JPEG
            jpegs[k][101] = VIDEO_MAGIC2;
        }
    }

    static uint8_t stream[3 * (sizeof(VideoFrameHeader) + 700 + VIDEO_TRAILER_LEN)];
    size_t total = 0;
    for (int k = 0; k < 3; k++) {
        const size_t n = buildFrame(stream + total, seqs[k], jpegs[k], lens[k]);
        TEST_ASSERT_NOT_EQUAL_MESSAGE(0, n, "кадр не собран функциями прошивки");
        total += n;
    }

    // Разбор так, как это делает буй: заголовок -> JPEG -> CRC16 LE
    size_t pos = 0;
    for (int k = 0; k < 3; k++) {
        VideoFrameHeader h;
        memcpy(&h, stream + pos, sizeof(h));
        TEST_ASSERT_TRUE_MESSAGE(videoHeaderValid(h), "буй не принял заголовок кадра");
        TEST_ASSERT_EQUAL_UINT16_MESSAGE(seqs[k], h.seq, "seq кадра искажён");
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(lens[k], h.length, "length кадра искажён");
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(0, h.flags, "flags должен быть 0");
        pos += sizeof(h);
        const uint16_t crc = crc16Update(CRC16_INIT, stream + pos, h.length);
        TEST_ASSERT_EQUAL_MEMORY_MESSAGE(jpegs[k], stream + pos, h.length, "JPEG в кадре искажён");
        pos += h.length;
        const uint16_t rx = static_cast<uint16_t>(stream[pos] | (stream[pos + 1] << 8));
        TEST_ASSERT_EQUAL_HEX16_MESSAGE(crc, rx, "буй не принял CRC16 кадра");
        pos += VIDEO_TRAILER_LEN;
    }
    TEST_ASSERT_EQUAL_MESSAGE(total, pos, "в потоке остались лишние байты");

    // Порча одного байта JPEG первого кадра обнаруживается по CRC16
    stream[sizeof(VideoFrameHeader) + 10] ^= 0x40;
    const uint16_t bad = crc16Update(CRC16_INIT, stream + sizeof(VideoFrameHeader), lens[0]);
    const size_t trPos = sizeof(VideoFrameHeader) + lens[0];
    const uint16_t rx0 = static_cast<uint16_t>(stream[trPos] | (stream[trPos + 1] << 8));
    TEST_ASSERT_NOT_EQUAL_MESSAGE(rx0, bad, "испорченный JPEG не обнаружен по CRC16");
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_crc16_reference_and_streaming);
    RUN_TEST(test_trailer_little_endian);
    RUN_TEST(test_header_layout);
    RUN_TEST(test_length_bounds);
    RUN_TEST(test_corrupted_header_rejected);
    RUN_TEST(test_buoy_compatibility);
    return UNITY_END();
}
