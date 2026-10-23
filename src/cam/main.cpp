// =============================================================================
//  Курсовая камера ESP32-CAM AI-Thinker + OV2640 (env:cam, отсек 5).
//
//  Снимает JPEG и шлёт кадры по UART1 (GPIO13 -> RS-485 -> пара 4) в буй
//  по контракту №4 (docs/ICD.md §7):
//    [VideoFrameHeader 10 Б][JPEG length Б][CRC16-CCITT(JPEG) 2 Б, LE]
//
//  Wi-Fi и Bluetooth выключены. Цикл без delay(): конечный автомат
//  IDLE -> HEADER -> JPEG -> TRAILER, данные отдаются в UART порциями
//  по свободному месту в TX-буфере. Буфер кадра (fb) возвращается драйверу
//  камеры, когда весь JPEG скопирован в TX-буфер UART (дальше байты уходят
//  из буфера драйвера); новый кадр берётся только после ухода CRC-хвоста.
// =============================================================================
#include <Arduino.h>
#include <WiFi.h>
#include <stdarg.h>
#include <stdio.h>

#include "esp_camera.h"
#include "soc/soc_caps.h"

#include "config.h"
#include "protocol.h"

#include "camera_pins.h"
#include "video_frame.h"

static_assert(CAM_QUALITY_MIN <= CAM_QUALITY_MAX, "CAM_QUALITY_MIN > CAM_QUALITY_MAX");
static_assert(CAM_TARGET_FPS > 0, "CAM_TARGET_FPS must be > 0");
static_assert(CAM_VIDEO_TXBUF > SOC_UART_FIFO_LEN + CAM_TX_CHUNK_MIN, "CAM_VIDEO_TXBUF too small");

static constexpr uint32_t FRAME_PERIOD_MS = 1000UL / CAM_TARGET_FPS;
static constexpr int QUALITY_BASE =
    CAM_JPEG_QUALITY < CAM_QUALITY_MIN ? CAM_QUALITY_MIN
    : (CAM_JPEG_QUALITY > CAM_QUALITY_MAX ? CAM_QUALITY_MAX : CAM_JPEG_QUALITY);
static constexpr uint32_t SMALL_FRAME_BYTES =
    static_cast<uint32_t>(VIDEO_MAX_FRAME) * CAM_QUALITY_RELAX_PCT / 100U;

// Видеолиния: UART1. RX не нужен (линия только на передачу), поэтому RX = -1:
// в core 2.0.x HardwareSerial::begin подставляет пины по умолчанию (RX1=9/TX1=10,
// это линии SPI-флеша) только когда оба пина < 0; при RX = -1 и заданном TX
// драйвер не трогает ни один GPIO под приём.
static HardwareSerial &videoSerial = Serial1;
static constexpr int8_t VIDEO_RX_PIN_UNUSED = -1;

// -----------------------------------------------------------------------------
//  Лог в USB Serial без кучи: Print::printf в core 2.0.x делает malloc для строк
//  длиннее 64 байт, поэтому форматируем в статический буфер и пишем Serial.write.
// -----------------------------------------------------------------------------
static char logBuf[256];   // строка статистики с кириллицей (UTF-8) около 170 байт

static void camLog(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void camLog(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    const int n = vsnprintf(logBuf, sizeof(logBuf), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    const size_t len = static_cast<size_t>(n) < sizeof(logBuf) ? static_cast<size_t>(n) : sizeof(logBuf) - 1;
    Serial.write(reinterpret_cast<const uint8_t *>(logBuf), len);
}

// -----------------------------------------------------------------------------
//  Состояние
// -----------------------------------------------------------------------------
enum class TxState : uint8_t { CamDown, Idle, Header, Jpeg, Trailer };

static TxState txState = TxState::CamDown;
static camera_fb_t *curFb = nullptr;           // кадр в отправке, держим до конца
static VideoFrameHeader curHeader;
static uint8_t curTrailer[VIDEO_TRAILER_LEN];
static uint16_t curCrc = CRC16_INIT;
static size_t txOffset = 0;                    // смещение внутри текущего фрагмента
static uint16_t frameSeq = 0;

static bool psramOk = false;
static uint32_t lastInitTryMs = 0;
static uint32_t lastGrabMs = 0;
static uint32_t fbFailStreak = 0;

static int quality = QUALITY_BASE;
static uint32_t smallStreak = 0;
static uint32_t qualityHoldFrames = 0;         // после смены quality в буферах ещё старые кадры

// Статистика окна CAM_LOG_PERIOD_MS
static uint32_t winStartMs = 0;
static uint32_t winFrames = 0;
static uint32_t winBytes = 0;
static uint32_t winOversize = 0;
static uint32_t winFbFail = 0;
static uint32_t winMaxFrame = 0;

// -----------------------------------------------------------------------------
//  Камера
// -----------------------------------------------------------------------------
static bool cameraInit() {
    camera_config_t c = {};
    c.pin_pwdn = CAM_PIN_PWDN;
    c.pin_reset = CAM_PIN_RESET;
    c.pin_xclk = CAM_PIN_XCLK;
    c.pin_sccb_sda = CAM_PIN_SIOD;
    c.pin_sccb_scl = CAM_PIN_SIOC;
    c.pin_d7 = CAM_PIN_D7;
    c.pin_d6 = CAM_PIN_D6;
    c.pin_d5 = CAM_PIN_D5;
    c.pin_d4 = CAM_PIN_D4;
    c.pin_d3 = CAM_PIN_D3;
    c.pin_d2 = CAM_PIN_D2;
    c.pin_d1 = CAM_PIN_D1;
    c.pin_d0 = CAM_PIN_D0;
    c.pin_vsync = CAM_PIN_VSYNC;
    c.pin_href = CAM_PIN_HREF;
    c.pin_pclk = CAM_PIN_PCLK;
    c.xclk_freq_hz = CAM_XCLK_FREQ_HZ;
    c.ledc_timer = LEDC_TIMER_0;
    c.ledc_channel = LEDC_CHANNEL_0;
    c.pixel_format = PIXFORMAT_JPEG;
    c.frame_size = CAM_FRAME_SIZE;
    c.jpeg_quality = quality;
    c.grab_mode = CAMERA_GRAB_LATEST;
    c.sccb_i2c_port = 0;
    if (psramOk) {
        c.fb_count = CAM_FB_COUNT;
        c.fb_location = CAMERA_FB_IN_PSRAM;
    } else {
        // Аварийный режим без PSRAM: один буфер во внутренней памяти
        c.fb_count = 1;
        c.fb_location = CAMERA_FB_IN_DRAM;
    }

    const esp_err_t err = esp_camera_init(&c);
    if (err != ESP_OK) {
        camLog("[cam] ОШИБКА инициализации камеры 0x%x, повтор через %u мс\n",
                      static_cast<unsigned>(err), static_cast<unsigned>(CAM_INIT_RETRY_MS));
        esp_camera_deinit();
        return false;
    }
    camLog("[cam] камера готова: framesize %d, quality %d, fb %u в %s\n",
                  static_cast<int>(CAM_FRAME_SIZE), quality, static_cast<unsigned>(c.fb_count),
                  psramOk ? "PSRAM" : "DRAM");
    return true;
}

static void applyQuality(int q) {
    if (q < CAM_QUALITY_MIN) q = CAM_QUALITY_MIN;
    if (q > CAM_QUALITY_MAX) q = CAM_QUALITY_MAX;
    if (q == quality) return;
    sensor_t *s = esp_camera_sensor_get();
    if (s == nullptr || s->set_quality == nullptr) return;
    if (s->set_quality(s, q) == 0) {
        quality = q;
        qualityHoldFrames = CAM_FB_COUNT;   // уже снятые кадры ещё со старым quality
        smallStreak = 0;
    }
}

// Учёт размера кадра для обратного улучшения quality
static void noteSentFrameSize(uint32_t len) {
    if (qualityHoldFrames > 0) {
        qualityHoldFrames--;
        return;
    }
    if (len < SMALL_FRAME_BYTES && quality > QUALITY_BASE) {
        if (++smallStreak >= CAM_QUALITY_RELAX_FRAMES) {
            applyQuality(quality - 1);
        }
    } else {
        smallStreak = 0;
    }
}

static void releaseFrame() {
    if (curFb != nullptr) {
        esp_camera_fb_return(curFb);
        curFb = nullptr;
    }
}

static void cameraDown(const char *reason) {
    releaseFrame();
    esp_camera_deinit();
    txState = TxState::CamDown;
    lastInitTryMs = millis();
    camLog("[cam] камера остановлена (%s), переинициализация через %u мс\n",
                  reason, static_cast<unsigned>(CAM_INIT_RETRY_MS));
}

// -----------------------------------------------------------------------------
//  Неблокирующая отдача фрагмента в UART.
//  availableForWrite() в core 2.0.x при полностью занятом кольцевом буфере
//  возвращает свободное место аппаратного FIFO (до SOC_UART_FIFO_LEN), а драйвер
//  кладёт в кольцевой буфер ещё и служебный заголовок на каждую запись. Поэтому
//  пишем только когда свободно больше FIFO-запаса и порциями не меньше
//  CAM_TX_CHUNK_MIN (кроме остатка фрагмента): тогда write() не ждёт места.
//  crc != nullptr — отправленные байты добавляются в CRC16.
//  Возвращает true, когда фрагмент отправлен целиком.
// -----------------------------------------------------------------------------
static bool pumpBytes(const uint8_t *data, size_t len, size_t &off, uint16_t *crc) {
    while (off < len) {
        const int avail = videoSerial.availableForWrite();
        if (avail <= static_cast<int>(SOC_UART_FIFO_LEN)) return false;
        const size_t room = static_cast<size_t>(avail) - SOC_UART_FIFO_LEN;
        const size_t left = len - off;
        size_t n = left < room ? left : room;
        if (n < left && n < CAM_TX_CHUNK_MIN) return false;   // ждём места под крупную порцию
        n = videoSerial.write(data + off, n);
        if (n == 0) return false;
        if (crc != nullptr) *crc = crc16Update(*crc, data + off, n);
        off += n;
    }
    return true;
}

// Взять кадр и подготовить заголовок. true — можно начинать отправку.
static bool grabFrame() {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb == nullptr) {
        winFbFail++;
        if (++fbFailStreak >= CAM_FB_FAIL_REINIT) {
            fbFailStreak = 0;
            cameraDown("нет кадров от сенсора");
        }
        return false;
    }
    fbFailStreak = 0;

    if (fb->format != PIXFORMAT_JPEG || fb->len == 0) {
        winFbFail++;
        esp_camera_fb_return(fb);
        return false;
    }

    const uint32_t len = static_cast<uint32_t>(fb->len);
    if (len > winMaxFrame) winMaxFrame = len;

    if (!videoFrameBuildHeader(frameSeq, len, curHeader)) {
        // Кадр больше VIDEO_MAX_FRAME: по контракту не шлём, ухудшаем quality
        winOversize++;
        esp_camera_fb_return(fb);
        if (qualityHoldFrames > 0) {
            qualityHoldFrames--;
        } else {
            applyQuality(quality + CAM_QUALITY_STEP_UP);
        }
        return false;
    }

    frameSeq++;
    curFb = fb;
    curCrc = CRC16_INIT;
    txOffset = 0;
    return true;
}

static void serviceVideo(uint32_t now) {
    switch (txState) {
        case TxState::CamDown:
            if (now - lastInitTryMs >= CAM_INIT_RETRY_MS) {
                lastInitTryMs = now;
                if (cameraInit()) {
                    fbFailStreak = 0;
                    txState = TxState::Idle;
                }
            }
            break;

        case TxState::Idle:
            if (now - lastGrabMs < FRAME_PERIOD_MS) break;
            // Держим ритм CAM_TARGET_FPS; при отставании не догоняем пачкой
            lastGrabMs = (now - lastGrabMs < 2 * FRAME_PERIOD_MS) ? lastGrabMs + FRAME_PERIOD_MS : now;
            if (grabFrame()) txState = TxState::Header;
            break;

        case TxState::Header:
            if (!pumpBytes(reinterpret_cast<const uint8_t *>(&curHeader), sizeof(curHeader), txOffset, nullptr)) break;
            txOffset = 0;
            txState = TxState::Jpeg;
            [[fallthrough]];

        case TxState::Jpeg:
            if (!pumpBytes(curFb->buf, curFb->len, txOffset, &curCrc)) break;
            {
                const uint32_t len = static_cast<uint32_t>(curFb->len);
                releaseFrame();   // JPEG целиком в UART-буфере, fb больше не нужен
                videoFrameBuildTrailer(curCrc, curTrailer);
                winFrames++;
                winBytes += len;
                noteSentFrameSize(len);
            }
            txOffset = 0;
            txState = TxState::Trailer;
            [[fallthrough]];

        case TxState::Trailer:
            if (!pumpBytes(curTrailer, VIDEO_TRAILER_LEN, txOffset, nullptr)) break;
            txOffset = 0;
            txState = TxState::Idle;
            break;
    }
}

static void serviceLog(uint32_t now) {
    const uint32_t dt = now - winStartMs;
    if (dt < CAM_LOG_PERIOD_MS) return;
    // к/с в десятых, целочисленно: %f в newlib может обращаться к куче
    const uint32_t fps10 = static_cast<uint32_t>((static_cast<uint64_t>(winFrames) * 10000ULL) / dt);
    const unsigned avg = winFrames ? static_cast<unsigned>(winBytes / winFrames) : 0U;
    camLog("[cam] %u.%u к/с, ср. кадр %u Б, макс. %u Б, пропуск >%u Б: %u, сбоев fb: %u, quality %d%s\n",
                  static_cast<unsigned>(fps10 / 10), static_cast<unsigned>(fps10 % 10), avg, static_cast<unsigned>(winMaxFrame), static_cast<unsigned>(VIDEO_MAX_FRAME),
                  static_cast<unsigned>(winOversize), static_cast<unsigned>(winFbFail), quality,
                  txState == TxState::CamDown ? ", КАМЕРА НЕ ГОТОВА" : "");
    winStartMs = now;
    winFrames = 0;
    winBytes = 0;
    winOversize = 0;
    winFbFail = 0;
    winMaxFrame = 0;
}

// -----------------------------------------------------------------------------
void setup() {
    // Вспышка выключена с первых строк: GPIO4 — она же SD DATA1, SD_MMC не используется
    pinMode(CAM_FLASH_LED_PIN, OUTPUT);
    digitalWrite(CAM_FLASH_LED_PIN, LOW);

    Serial.setTxBufferSize(USB_LOG_TX_BUFFER);   // до begin(): строка лога не ждёт FIFO
    Serial.begin(USB_LOG_BAUD);                  // UART0 по USB — только лог
    camLog("\n[cam] fw %s, protocol v%d, видео UART1 TX GPIO%d @ %lu бод\n", FW_VERSION,
                  PROTOCOL_VERSION, CAM_VIDEO_TX_PIN, static_cast<unsigned long>(VIDEO_BAUD));

    // Под водой радио бесполезно и только греет/шумит: Wi-Fi и BT выключены
    WiFi.mode(WIFI_OFF);
    btStop();

    psramOk = psramFound();
    if (!psramOk) {
        camLog("[cam] ВНИМАНИЕ: PSRAM не найдена, 1 буфер кадра во внутренней памяти\n");
    }

    videoSerial.setTxBufferSize(CAM_VIDEO_TXBUF);   // до begin(), иначе не применится
    videoSerial.begin(VIDEO_BAUD, SERIAL_8N1, VIDEO_RX_PIN_UNUSED, CAM_VIDEO_TX_PIN);

    const uint32_t now = millis();
    winStartMs = now;
    lastGrabMs = now;
    if (cameraInit()) {
        txState = TxState::Idle;
    } else {
        txState = TxState::CamDown;
        lastInitTryMs = now;
    }
}

void loop() {
    const uint32_t now = millis();
    serviceVideo(now);
    serviceLog(now);
}
