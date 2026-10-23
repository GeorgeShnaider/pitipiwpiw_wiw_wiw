// =============================================================================
//  Надводный буй (env:buoy), узел 6.
//  Wi-Fi AP + HTTP/WebSocket (web), UART2 к ROV_CORE (link),
//  видео с камеры по UART1 (video), диагностика в USB Serial.
// =============================================================================
#include <Arduino.h>
#include <esp_system.h>
#include <stdio.h>

#include "buoy_log.h"
#include "buoy_timing.h"
#include "config.h"
#include "link.h"
#include "protocol.h"
#include "video.h"
#include "web.h"

namespace {

uint32_t lastDiagMs = 0;
WebStats prevWeb = {};
LinkStats prevLink = {};
VideoStats prevVideo = {};
uint32_t maxLoopUs = 0;   // самый долгий проход loop() за период диагностики
uint32_t lastLoopUs = 0;
char diagBuf[512];  // худший случай второй строки ~490 Б

// Частота события за период, в сотых долях «в секунду»
uint32_t perSecX100(uint32_t delta, uint32_t periodMs) {
    return periodMs == 0 ? 0 : static_cast<uint32_t>((static_cast<uint64_t>(delta) * 100000ULL) / periodMs);
}

unsigned long ul(uint32_t v) { return static_cast<unsigned long>(v); }

void writeDiag(int n) {
    if (n <= 0) return;
    const size_t len = static_cast<size_t>(n) < sizeof(diagBuf) ? static_cast<size_t>(n) : sizeof(diagBuf) - 1U;
    Serial.write(reinterpret_cast<const uint8_t *>(diagBuf), len);
    Serial.write('\n');
}

void printDiag(uint32_t nowMs) {
    const uint32_t period = nowMs - lastDiagMs;
    lastDiagMs = nowMs;
    const WebStats w = webGetStats();
    const LinkStats l = linkGetStats();
    const VideoStats v = videoTakeStats();

    const uint32_t cmdRate = perSecX100(w.cmdOk - prevWeb.cmdOk, period);
    const uint32_t pktRate = perSecX100(l.packetsSent - prevLink.packetsSent, period);
    const uint32_t telRate = perSecX100(l.linesReceived - prevLink.linesReceived, period);
    const uint32_t dropped = (w.cmdBad - prevWeb.cmdBad) + (w.cmdForeign - prevWeb.cmdForeign) +
                             (w.cmdFragment - prevWeb.cmdFragment);

    writeDiag(snprintf(diagBuf, sizeof(diagBuf),
                       "[buoy] клиентов %u (управляет: %s) | cmd/с %lu.%02lu | вниз/с %lu.%02lu (armed %lu, пропуск %lu)"
                       " | телем/с %lu.%02lu (переполн. %lu) | отброшено cmd %lu (битых %lu, чужих %lu, фрагм. %lu)",
                       static_cast<unsigned>(webClientCount()), webHasController() ? "да" : "нет",
                       ul(cmdRate / 100U), ul(cmdRate % 100U), ul(pktRate / 100U), ul(pktRate % 100U),
                       ul(l.packetsArmed - prevLink.packetsArmed), ul(l.packetsSkipped - prevLink.packetsSkipped),
                       ul(telRate / 100U), ul(telRate % 100U), ul(l.linesOverflow - prevLink.linesOverflow),
                       ul(dropped), ul(w.cmdBad - prevWeb.cmdBad), ul(w.cmdForeign - prevWeb.cmdForeign),
                       ul(w.cmdFragment - prevWeb.cmdFragment)));

    const uint32_t okRate = perSecX100(v.framesOk - prevVideo.framesOk, period);
    const uint32_t sentRate = perSecX100(v.sent - prevVideo.sent, period);
    writeDiag(snprintf(diagBuf, sizeof(diagBuf),
                       "[buoy] видео: ок %lu (%lu.%02lu к/с), битых заголовков %lu, битых CRC %lu, мусор %lu Б"
                       " | в WS %lu (%lu.%02lu к/с), пропуск: очередь %lu, нет клиентов %lu, отказ %lu"
                       " | UART1: буфер полон %lu, FIFO overflow %lu, макс. чтение %lu Б"
                       " | loop макс %lu мкс | куча %lu (мин %lu)",
                       ul(v.framesOk - prevVideo.framesOk), ul(okRate / 100U), ul(okRate % 100U),
                       ul(v.badHeader - prevVideo.badHeader), ul(v.badCrc - prevVideo.badCrc),
                       ul(v.junkBytes - prevVideo.junkBytes), ul(v.sent - prevVideo.sent), ul(sentRate / 100U),
                       ul(sentRate % 100U), ul(v.skippedBusy - prevVideo.skippedBusy),
                       ul(v.skippedNoClient - prevVideo.skippedNoClient), ul(v.discarded - prevVideo.discarded),
                       ul(v.uartBufferFull - prevVideo.uartBufferFull), ul(v.uartFifoOvf - prevVideo.uartFifoOvf),
                       ul(v.maxReadBytes), ul(maxLoopUs), ul(esp_get_free_heap_size()),
                       ul(esp_get_minimum_free_heap_size())));

    prevWeb = w;
    prevLink = l;
    prevVideo = v;
    maxLoopUs = 0;
}

}  // namespace

void setup() {
    // TX-буфер до begin(): диагностика (~2 строки раз в 5 с) уходит в кольцевой
    // буфер драйвера и не блокирует loop() на время передачи по 115200
    Serial.setTxBufferSize(USB_LOG_TX_BUFFER);
    Serial.begin(USB_LOG_BAUD);
    logLine("[buoy] прошивка %s, протокол v%d", FW_VERSION, PROTOCOL_VERSION);
    // Канал вниз стартует первым: пакеты armed=0 идут ещё до поднятия Wi-Fi
    linkBegin();
    webBegin();
    videoBegin();
    lastDiagMs = millis();
    lastLoopUs = micros();
}

void loop() {
    const uint32_t startUs = micros();
    // Полный период между проходами, включая время вытеснения другими задачами
    const uint32_t passUs = startUs - lastLoopUs;
    lastLoopUs = startUs;
    if (passUs > maxLoopUs) maxLoopUs = passUs;

    const uint32_t now = millis();
    linkLoop(now);   // управление первым
    videoLoop();     // не более BUOY_VIDEO_READ_CHUNK байт за проход
    webLoop(now);
    if (now - lastDiagMs >= BUOY_DIAG_PERIOD_MS) printDiag(now);
}
