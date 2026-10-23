// =============================================================================
//  video — реализация приёма видео по UART1.
// =============================================================================
#include "video.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>

#include "buoy_timing.h"
#include "config.h"
#include "protocol.h"
#include "video_parser.h"
#include "web.h"

namespace {

VideoFrameParser parser;              // статический буфер ~32 КБ
uint8_t rxChunk[BUOY_VIDEO_READ_CHUNK];
VideoStats stats = {};

// Счётчики ошибок UART обновляются в задаче uart_event_task ядра
portMUX_TYPE uartErrMux = portMUX_INITIALIZER_UNLOCKED;
uint32_t uartBufferFull = 0;
uint32_t uartFifoOvf = 0;

void onUartError(hardwareSerial_error_t err) {
    portENTER_CRITICAL(&uartErrMux);
    if (err == UART_BUFFER_FULL_ERROR) {
        uartBufferFull++;
    } else if (err == UART_FIFO_OVF_ERROR) {
        uartFifoOvf++;
    }
    portEXIT_CRITICAL(&uartErrMux);
}

void onFrame(void *ctx, const uint8_t *jpeg, size_t len, uint16_t seq) {
    (void)ctx;
    (void)seq;
    switch (webBroadcastVideo(jpeg, len)) {
        case VideoSendResult::Sent:
            stats.sent++;
            break;
        case VideoSendResult::NoClients:
            stats.skippedNoClient++;
            break;
        case VideoSendResult::Busy:
            stats.skippedBusy++;
            break;
        case VideoSendResult::Discarded:
        default:
            stats.discarded++;
            break;
    }
}

}  // namespace

void videoBegin() {
    parser.setCallback(onFrame, nullptr);
    // Размер RX-буфера драйвера задаётся только до begin()
    Serial1.setRxBufferSize(BUOY_VIDEO_RXBUF);
    // Только приём. TX = -1: ядро 2.0.17 подставляет пины по умолчанию (9/10,
    // линии SPI-флеша) лишь когда оба пина < 0; при RX >= 0 TX не трогается.
    Serial1.begin(VIDEO_BAUD, SERIAL_8N1, BUOY_VIDEO_RX_PIN, -1);
    Serial1.onReceiveError(onUartError);
}

void videoLoop() {
    const int avail = Serial1.available();
    if (avail <= 0) return;
    const size_t want = static_cast<size_t>(avail) < sizeof(rxChunk) ? static_cast<size_t>(avail) : sizeof(rxChunk);
    const size_t got = Serial1.read(rxChunk, want);
    if (got == 0) return;
    if (got > stats.maxReadBytes) stats.maxReadBytes = static_cast<uint32_t>(got);
    parser.feed(rxChunk, got);
}

VideoStats videoTakeStats() {
    const VideoParserStats &p = parser.stats();
    stats.framesOk = p.framesOk;
    stats.badHeader = p.badHeader;
    stats.badCrc = p.badCrc;
    stats.junkBytes = p.junkBytes;
    portENTER_CRITICAL(&uartErrMux);
    stats.uartBufferFull = uartBufferFull;
    stats.uartFifoOvf = uartFifoOvf;
    portEXIT_CRITICAL(&uartErrMux);
    const VideoStats copy = stats;
    stats.maxReadBytes = 0;
    return copy;
}
