// =============================================================================
//  video — приём видео с камеры по UART1 (пара 4, контракт №4) и пересылка
//  JPEG в WebSocket binary. Только приём: TX UART1 не назначается.
// =============================================================================
#pragma once

#include <stdint.h>

struct VideoStats {
    uint32_t framesOk;       // целые кадры с линии
    uint32_t badHeader;      // плохой заголовок
    uint32_t badCrc;         // плохой CRC16 / обрезанный кадр
    uint32_t junkBytes;      // байт мусора между кадрами
    uint32_t sent;           // кадров отправлено в WS
    uint32_t skippedBusy;    // пропущено: предыдущий кадр ещё в очереди клиента
    uint32_t skippedNoClient;// пропущено: нет клиентов (без копирования)
    uint32_t discarded;      // библиотека не приняла (очередь клиента полна)
    uint32_t uartBufferFull; // событие UART_BUFFER_FULL (RX-буфер драйвера полон)
    uint32_t uartFifoOvf;    // событие UART_FIFO_OVF (байты потеряны)
    uint32_t maxReadBytes;   // максимум байт, прочитанных за проход (с прошлого сброса)
};

// UART1: setRxBufferSize(BUOY_VIDEO_RXBUF) до begin; вызывать из setup()
void videoBegin();

// Чтение не более BUOY_VIDEO_READ_CHUNK байт, разбор, отправка; из loop()
void videoLoop();

// Снимок счётчиков; maxReadBytes сбрасывается при каждом вызове
VideoStats videoTakeStats();
