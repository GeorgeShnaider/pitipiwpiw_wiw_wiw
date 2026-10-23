// =============================================================================
//  web — точка доступа, HTTP (LittleFS) и WebSocket /ws (контракт №1).
//  Коллбэки AsyncWebSocket работают в задаче async_tcp; общая с loop()
//  последняя команда защищена spinlock-ом (portMUX), захват короткий.
// =============================================================================
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "cmd_codec.h"

// Снимок последней команды для канала вниз
struct WebCmdSnapshot {
    BuoyCmd cmd;     // последняя принятая команда (light сохраняется после таймаута)
    bool fresh;      // есть живой управляющий клиент и cmd не старше BUOY_WS_TIMEOUT_MS
};

// Счётчики WebSocket (монотонные, переполнение допустимо)
struct WebStats {
    uint32_t cmdOk;        // принятые cmd от управляющего клиента
    uint32_t cmdBad;       // битый JSON / неверный type / неверные поля
    uint32_t cmdForeign;   // cmd от неуправляющих клиентов
    uint32_t cmdFragment;  // фрагментированные / не-text сообщения
    uint32_t telemetrySent;
};

// Wi-Fi AP, LittleFS, HTTP и WS. Вызывать из setup().
void webBegin();

// Периодические задачи (cleanupClients). Вызывать из loop().
void webLoop(uint32_t nowMs);

// Снимок команды под защитой; nowMs берётся внутри после копирования.
WebCmdSnapshot webGetCmd();

// Отправить строку телеметрии всем клиентам (без '\n'). Только из loop().
void webBroadcastText(const char *text, size_t len);

// Результат попытки отправить видеокадр (контракт №4)
enum class VideoSendResult : uint8_t {
    Sent,       // поставлен в очередь хотя бы одному клиенту
    NoClients,  // клиентов нет — кадр даже не копировался
    Busy,       // предыдущий кадр ещё в очереди какого-то клиента — пропуск
    Discarded,  // библиотека не приняла кадр ни одному клиенту (очередь полна)
};

// Отправить JPEG всем клиентам одним binary-сообщением. Только из loop().
// Один общий буфер на всех клиентов, выделяется один раз в webBegin().
VideoSendResult webBroadcastVideo(const uint8_t *jpeg, size_t len);

size_t webClientCount();
bool webHasController();
WebStats webGetStats();
