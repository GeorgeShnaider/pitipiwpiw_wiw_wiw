// =============================================================================
//  web — реализация точки доступа, HTTP и WebSocket.
// =============================================================================
#include "web.h"

#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <stdio.h>
#include <string.h>

#include <atomic>
#include <memory>
#include <vector>

#include "buoy_log.h"
#include "buoy_timing.h"
#include "config.h"
#include "protocol.h"

namespace {

constexpr uint16_t kHttpPort = 80;
const char kIndexPath[] = "/index.html";
const char kIndexGzPath[] = "/index.html.gz";
const char kMimeHtml[] = "text/html";
const char kMimeText[] = "text/plain; charset=utf-8";

AsyncWebServer server(kHttpPort);
AsyncWebSocket ws(WS_PATH);

// Что отдавать на "/": определяется один раз при старте
enum class IndexSource : uint8_t { None, Plain, Gzip };
bool fsMounted = false;
IndexSource indexSource = IndexSource::None;

// --- Общее состояние async_tcp <-> loop(), только под cmdMux ---
portMUX_TYPE cmdMux = portMUX_INITIALIZER_UNLOCKED;
BuoyCmd sharedCmd = {};
uint32_t sharedCmdMs = 0;
bool sharedLive = false;        // false: команды не было или управляющий ушёл
uint32_t controllerId = 0;      // id управляющего клиента, 0 — нет
WebStats stats = {};

uint32_t lastCleanupMs = 0;

// Общий буфер видеокадра. Ёмкость VIDEO_MAX_FRAME резервируется один раз в
// webBegin(), дальше буфер переиспользуется без выделений. Очередь каждого
// клиента держит копию shared_ptr, пока кадр не передан в TCP целиком;
// use_count()==1 означает, что кадр ушёл всем (или клиенты отключились) и
// буфер снова принадлежит только нам — его можно перезаписывать.
AsyncWebSocketSharedBuffer videoBuf;

void handleWsData(AsyncWebSocketClient *client, const AwsFrameInfo *info, const uint8_t *data, size_t len) {
    const uint32_t id = client->id();
    // Только целое нефрагментированное text-сообщение
    const bool whole = info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT;

    bool isController;
    portENTER_CRITICAL(&cmdMux);
    isController = (id == controllerId);
    if (!whole) {
        stats.cmdFragment++;
    } else if (!isController) {
        stats.cmdForeign++;
    }
    portEXIT_CRITICAL(&cmdMux);
    if (!whole || !isController) return;

    // Разбор вне критической секции (не реентерабелен, но вызывается только из async_tcp)
    BuoyCmd parsed;
    const CmdParseResult res = parseCmdJson(data, len, parsed);
    const uint32_t now = millis();

    portENTER_CRITICAL(&cmdMux);
    if (res == CmdParseResult::Ok && id == controllerId) {
        sharedCmd = parsed;
        sharedCmdMs = now;
        sharedLive = true;
        stats.cmdOk++;
    } else if (res != CmdParseResult::Ok) {
        stats.cmdBad++;
    } else {
        stats.cmdForeign++;
    }
    portEXIT_CRITICAL(&cmdMux);
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    (void)server;
    switch (type) {
        case WS_EVT_CONNECT: {
            const uint32_t id = client->id();
            portENTER_CRITICAL(&cmdMux);
            // Управление переходит к новому клиенту; старая команда сразу протухает
            controllerId = id;
            sharedLive = false;
            portEXIT_CRITICAL(&cmdMux);
            logLine("[buoy] WS: клиент #%lu подключён, теперь он управляет", static_cast<unsigned long>(id));
            break;
        }
        case WS_EVT_DISCONNECT: {
            const uint32_t id = client->id();
            bool wasController = false;
            portENTER_CRITICAL(&cmdMux);
            if (id == controllerId) {
                controllerId = 0;
                sharedLive = false;  // не ждём BUOY_WS_TIMEOUT_MS
                wasController = true;
            }
            portEXIT_CRITICAL(&cmdMux);
            logLine("[buoy] WS: клиент #%lu отключён%s", static_cast<unsigned long>(id),
                    wasController ? " (управлявший, команда сброшена)" : "");
            break;
        }
        case WS_EVT_DATA:
            handleWsData(client, static_cast<const AwsFrameInfo *>(arg), data, len);
            break;
        case WS_EVT_ERROR:
            logLine("[buoy] WS: ошибка клиента #%lu", static_cast<unsigned long>(client->id()));
            break;
        case WS_EVT_PING:
        case WS_EVT_PONG:
        default:
            break;
    }
}

void handleRoot(AsyncWebServerRequest *request) {
    switch (indexSource) {
        case IndexSource::Gzip: {
            AsyncWebServerResponse *resp = request->beginResponse(LittleFS, kIndexGzPath, kMimeHtml);
            resp->addHeader("Content-Encoding", "gzip");
            request->send(resp);
            break;
        }
        case IndexSource::Plain:
            request->send(LittleFS, kIndexPath, kMimeHtml);
            break;
        case IndexSource::None:
        default:
            request->send(503, kMimeText,
                          fsMounted ? "Буй: в LittleFS нет index.html. Выполните pio run -e buoy -t uploadfs."
                                    : "Буй: LittleFS не смонтирована. Выполните pio run -e buoy -t uploadfs.");
            break;
    }
}

void handleNotFound(AsyncWebServerRequest *request) {
    // Любой другой путь -> на главную (captive-проверки Android)
    request->redirect("/");
}

void startAccessPoint() {
    WiFi.persistent(false);
    WiFi.mode(WIFI_AP);
    const IPAddress ip(WIFI_AP_IP);
    const IPAddress mask(WIFI_AP_MASK);
    if (!WiFi.softAPConfig(ip, ip, mask)) {
        logLine("[buoy] Wi-Fi: ошибка softAPConfig");
    }
    if (!WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS, WIFI_AP_CHANNEL, 0, WIFI_AP_MAX_CONN)) {
        logLine("[buoy] Wi-Fi: ошибка запуска точки доступа");
    }
    WiFi.setSleep(false);
    const IPAddress real = WiFi.softAPIP();
    logLine("[buoy] Wi-Fi AP \"%s\", канал %d, IP %u.%u.%u.%u", WIFI_AP_SSID, WIFI_AP_CHANNEL,
            real[0], real[1], real[2], real[3]);
}

void mountFs() {
    fsMounted = LittleFS.begin(false);
    if (!fsMounted) {
        logLine("[buoy] LittleFS: не смонтирована, WebApp недоступен (нужен uploadfs)");
        indexSource = IndexSource::None;
        return;
    }
    // LittleFS.exists() в core 2.0.x реализован через open() и логирует ошибку
    // vfs_api на отсутствующем файле, поэтому один раз обходим корень и сверяем пути
    bool hasGz = false;
    bool hasPlain = false;
    File root = LittleFS.open("/");
    if (root && root.isDirectory()) {
        for (File f = root.openNextFile(); f; f = root.openNextFile()) {
            if (!f.isDirectory()) {
                const char *path = f.path();
                if (path != nullptr && strcmp(path, kIndexGzPath) == 0) hasGz = true;
                if (path != nullptr && strcmp(path, kIndexPath) == 0) hasPlain = true;
            }
            f.close();
        }
    }
    root.close();

    if (hasGz) {
        indexSource = IndexSource::Gzip;
        logLine("[buoy] LittleFS: отдаю %s (gzip)", kIndexGzPath);
    } else if (hasPlain) {
        indexSource = IndexSource::Plain;
        logLine("[buoy] LittleFS: отдаю %s", kIndexPath);
    } else {
        indexSource = IndexSource::None;
        logLine("[buoy] LittleFS: нет index.html, WebApp недоступен");
    }
}

}  // namespace

void webBegin() {
    startAccessPoint();
    mountFs();

    videoBuf = std::make_shared<std::vector<uint8_t>>();
    videoBuf->reserve(VIDEO_MAX_FRAME);

    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    server.on("/", HTTP_GET, handleRoot);
    server.onNotFound(handleNotFound);
    server.begin();
    logLine("[buoy] HTTP :%u и WebSocket %s запущены", kHttpPort, WS_PATH);
}

void webLoop(uint32_t nowMs) {
    if (nowMs - lastCleanupMs >= BUOY_WS_CLEANUP_MS) {
        lastCleanupMs = nowMs;
        ws.cleanupClients(WIFI_AP_MAX_CONN);
    }
}

WebCmdSnapshot webGetCmd() {
    WebCmdSnapshot snap;
    uint32_t cmdMs;
    bool live;
    portENTER_CRITICAL(&cmdMux);
    snap.cmd = sharedCmd;
    cmdMs = sharedCmdMs;
    live = sharedLive;
    portEXIT_CRITICAL(&cmdMux);
    // millis() после копирования: cmdMs не может оказаться «из будущего»
    const uint32_t now = millis();
    snap.fresh = live && (now - cmdMs <= BUOY_WS_TIMEOUT_MS);
    return snap;
}

void webBroadcastText(const char *text, size_t len) {
    if (len == 0 || ws.count() == 0) return;
    ws.textAll(text, len);
    portENTER_CRITICAL(&cmdMux);
    stats.telemetrySent++;
    portEXIT_CRITICAL(&cmdMux);
}

VideoSendResult webBroadcastVideo(const uint8_t *jpeg, size_t len) {
    if (len == 0 || len > VIDEO_MAX_FRAME || !videoBuf) return VideoSendResult::Discarded;
    if (ws.count() == 0) return VideoSendResult::NoClients;
    // Предыдущий кадр ещё в очереди хотя бы одного клиента: новый не ставим,
    // чтобы за видео никогда не копилась очередь и телеметрия шла без задержки
    if (videoBuf.use_count() > 1) return VideoSendResult::Busy;
    // Освобождение ссылок в async_tcp должно быть видно до перезаписи данных
    std::atomic_thread_fence(std::memory_order_acquire);
    videoBuf->assign(jpeg, jpeg + len);  // ёмкость зарезервирована — без выделения
    const AsyncWebSocket::SendStatus st = ws.binaryAll(videoBuf);
    return st == AsyncWebSocket::DISCARDED ? VideoSendResult::Discarded : VideoSendResult::Sent;
}

size_t webClientCount() { return ws.count(); }

bool webHasController() {
    portENTER_CRITICAL(&cmdMux);
    const bool has = controllerId != 0;
    portEXIT_CRITICAL(&cmdMux);
    return has;
}

WebStats webGetStats() {
    portENTER_CRITICAL(&cmdMux);
    const WebStats copy = stats;
    portEXIT_CRITICAL(&cmdMux);
    return copy;
}
