// =============================================================================
//  config.h — пины, тайминги, калибровки и матрица микширования.
//  Владелец: ЛИД (чат 0). Агенты узлов читают, но не правят — присылают запрос.
//
//  Секции: [ОБЩЕЕ] [ROV_CORE] [BUOY] [CAM]. Все узлы подключают один файл,
//  чтобы значения, общие для двух сторон кабеля, не расходились.
// =============================================================================
#pragma once

#include <stdint.h>

// =============================================================================
//  [ОБЩЕЕ]
// =============================================================================
#define FW_VERSION "0.1.0"

// Сеть буя
#define WIFI_AP_SSID      "ROV_BUOY_AP"
#define WIFI_AP_PASS      "rov12345"        // WPA2, минимум 8 символов
#define WIFI_AP_CHANNEL   6
#define WIFI_AP_MAX_CONN  2
#define WIFI_AP_IP        192, 168, 4, 1
#define WIFI_AP_MASK      255, 255, 255, 0

// USB Serial (лог всех узлов), совпадает с monitor_speed в platformio.ini
#define USB_LOG_BAUD      115200UL
#define USB_LOG_TX_BUFFER 1024       // TX-буфер USB Serial: запись лога не ждёт FIFO

// Fail-safe
#define FAILSAFE_TIMEOUT_MS  500   // rov_core: нет валидного ControlPacket -> disarm + 1500 мкс
#define BUOY_WS_TIMEOUT_MS   300   // buoy: нет cmd от WebApp -> шлёт вниз armed=0 и нули

// =============================================================================
//  [ROV_CORE] Главный контроллер ТНПА, ESP-WROOM-32 38-pin (отсек 4)
// =============================================================================

// --- Кабель (UART2 через RS-485 MAX3490/MAX3491, full-duplex) ---
#define ROV_LINK_RX_PIN   16   // пара 2: управление сверху
#define ROV_LINK_TX_PIN   17   // пара 3: телеметрия наверх

// --- I2C датчики ---
#define I2C_SDA_PIN       21
#define I2C_SCL_PIN       22
#define I2C_FREQ_HZ       400000UL
#define MS5837_I2C_ADDR   0x76   // MS5837-30BA (MS583730BA01-50)
#define IMU_I2C_ADDR      0x68   // MPU6050/MPU6500 (AD0 = GND)
#define WATER_DENSITY     997.0f // кг/м3; пресная вода. Морская — 1025.0f
#define GRAVITY           9.80665f

// --- Батарея 4S через делитель на АЦП1 ---
#define VBAT_ADC_PIN      34     // только АЦП1 (GPIO32..39), АЦП2 не работает при Wi-Fi
#define VBAT_R_TOP        100000.0f  // Ом, от +BAT к пину
#define VBAT_R_BOTTOM     22000.0f   // Ом, от пина к GND (16.8 В -> 3.03 В)
#define VBAT_CAL_K        1.000f     // поправка по мультиметру: Vреальное / Vизмеренное
#define VBAT_WARN_V       14.0f      // 3.5 В/банку
#define VBAT_CRIT_V       13.2f      // 3.3 В/банку

#define VBAT_POLL_MS      100        // период чтения АЦП
#define VBAT_AVG_WINDOW   16         // окно скользящего среднего, отсчётов

// --- Датчики: тайминги автоматов ---
#define I2C_TIMEOUT_MS        3      // таймаут транзакции Wire: висящая шина не держит loop
#define MS5837_RESET_WAIT_MS  3      // после команды reset
#define MS5837_CONV_WAIT_MS   20     // конверсия OSR 8192 (макс. 18.08 мс)
#define MS5837_RETRY_MS       2000   // повторная инициализация после ошибки
#define DEPTH_ZERO_MS         2000   // окно усреднения нуля глубины от первого валидного замера
#define IMU_CALIB_MS          2000   // калибровка смещения гироскопа, аппарат неподвижен
#define IMU_RETRY_MS          2000   // повторная инициализация после ошибки
#define IMU_COMPL_ALPHA       0.98f  // комплементарный фильтр roll/pitch (≈1 с при 50 Гц)
#define SENSOR_WARN_PERIOD_MS 5000   // период повтора предупреждений об отказах датчиков

// Монтаж IMU: выход[i] = IMU_AXIS_SIGN[i] * сырая_ось[IMU_AXIS_MAP[i]].
// Целевая СК аппарата: X — вперёд, Y — влево, Z — вверх. Уточнить по реальному монтажу.
static const uint8_t IMU_AXIS_MAP[3]  = {0, 1, 2};
static const int8_t  IMU_AXIS_SIGN[3] = {+1, +1, +1};

// --- Индикация и лог ---
#define STATUS_BLINK_SLOW_MS 500     // полупериод мигания: нет связи
#define STATUS_BLINK_FAST_MS 100     // полупериод мигания: armed
#define ROV_LOG_PERIOD_MS    1000    // период строки лога в USB Serial
#define ROV_LOG_LINE_MAX     384     // максимум байт строки лога

// --- Подсветка (LED-лента в отсеке 5 через N-MOSFET) ---
#define LIGHT_PIN         4
#define STATUS_LED_PIN    2

// --- ESC: RC PWM 50 Гц ---
#define ESC_COUNT         6
#define ESC_PWM_FREQ_HZ   50
#define ESC_PWM_RES_BITS  16
#define ESC_US_MIN        1000   // полный реверс (ESC прошиты BLHeli_S Bidirectional!)
#define ESC_US_NEUTRAL    1500   // стоп
#define ESC_US_MAX        2000   // полный вперёд
#define ESC_DEADBAND      0.05f  // |команда| меньше порога -> строго 1500 мкс
#define ESC_ARM_TIME_MS   3000   // после старта держать 1500 мкс, пока ESC инициализируются
#define THRUST_LIMIT      0.50f  // общий ограничитель тяги 0..1 (на первых испытаниях 50%)
#define ESC_SLEW_PER_S    4.0f   // макс. изменение команды мотора в единицах [-1..1] за секунду

// Индексы моторов
enum MotorIndex : uint8_t {
    M_STERN_LEFT  = 0,  // отсек 1, маршевый левый
    M_STERN_RIGHT = 1,  // отсек 1, маршевый правый
    M_BOW_VERT    = 2,  // отсек 2, маневровый вертикальный
    M_BOW_HORIZ   = 3,  // отсек 2, маневровый горизонтальный
    M_SPARE_5     = 4,  // резерв (не используется в текущей конфигурации)
    M_SPARE_6     = 5   // резерв (не используется в текущей конфигурации)
};

// GPIO для ESC. Не использовать 0, 2, 5, 12, 15 (strapping) и 14 (ШИМ при загрузке).
static const uint8_t ESC_PINS[ESC_COUNT] = {25, 26, 27, 13, 32, 33};

// Какие моторы физически установлены. Неактивные всё равно получают 1500 мкс.
static const bool MOTOR_ENABLED[ESC_COUNT] = {true, true, true, true, false, false};

// Инверсия направления винта: +1 норма, -1 реверс (подбирается на стенде)
static const int8_t MOTOR_DIR[ESC_COUNT] = {+1, +1, +1, +1, +1, +1};

// Матрица микширования: мотор[i] = sum(MIX[i][j] * ось[j]), оси {surge, yaw, heave, sway}.
// После умножения, если max|мотор| > 1, все моторы делятся на этот максимум.
// Текущая конфигурация: 2 маршевых + 2 маневровых в носу.
// Примечание: носовые маневровые смещены от центра масс, поэтому heave даёт
// ещё и дифферент на нос, а sway — ещё и рыскание. Компенсацию sway->yaw
// можно ввести коэффициентами в колонке sway для M_STERN_* после испытаний.
enum AxisIndex : uint8_t { AX_SURGE = 0, AX_YAW = 1, AX_HEAVE = 2, AX_SWAY = 3, AX_COUNT = 4 };

static const float MIX[ESC_COUNT][AX_COUNT] = {
    //  surge   yaw    heave  sway
    {   1.0f,  1.0f,  0.0f,  0.0f },  // M_STERN_LEFT
    {   1.0f, -1.0f,  0.0f,  0.0f },  // M_STERN_RIGHT
    {   0.0f,  0.0f,  1.0f,  0.0f },  // M_BOW_VERT
    {   0.0f,  0.0f,  0.0f,  1.0f },  // M_BOW_HORIZ
    {   0.0f,  0.0f,  0.0f,  0.0f },  // M_SPARE_5
    {   0.0f,  0.0f,  0.0f,  0.0f },  // M_SPARE_6
};

// Частоты задач rov_core
#define SENSOR_POLL_HZ     50   // IMU
#define DEPTH_POLL_HZ      10   // MS5837 (конверсия ~20 мс, опрашивать конечным автоматом)

// =============================================================================
//  [BUOY] Надводный буй, ESP-WROOM-32 (узел 6)
// =============================================================================
#define BUOY_LINK_RX_PIN   16   // пара 3: телеметрия снизу
#define BUOY_LINK_TX_PIN   17   // пара 2: управление вниз
#define BUOY_VIDEO_RX_PIN  18   // пара 4: видео от камеры (UART1, только приём)
#define BUOY_STATUS_LED    2
#define BUOY_VIDEO_RXBUF   16384  // размер RX-буфера UART1 под видеопоток
#define BUOY_VIDEO_READ_CHUNK 4096U  // максимум байт видео из UART1 за один проход loop()
#define BUOY_DIAG_PERIOD_MS   5000U  // диагностика в USB Serial
#define BUOY_WS_CLEANUP_MS    1000U  // период ws.cleanupClients()

// =============================================================================
//  [CAM] ESP32-CAM AI-Thinker, OV2640 (отсек 5)
// =============================================================================
#define CAM_VIDEO_TX_PIN   13   // UART1 TX -> RS-485 -> пара 4 (SD-карта не используется)
#define CAM_FLASH_LED_PIN  4    // встроенный светодиод-вспышка, держать выключенным
#define CAM_FRAME_SIZE     FRAMESIZE_QVGA   // 320x240; поднимать после замеров
#define CAM_JPEG_QUALITY   12               // 0..63, меньше = лучше и больше байт
#define CAM_TARGET_FPS     10
#define CAM_FB_COUNT             2      // буферов кадра в PSRAM
#define CAM_QUALITY_MIN          10     // лучшее quality при автоподстройке
#define CAM_QUALITY_MAX          30     // худшее quality при автоподстройке
#define CAM_QUALITY_STEP_UP      3      // ухудшение quality при кадре > VIDEO_MAX_FRAME
#define CAM_QUALITY_RELAX_FRAMES 30     // столько подряд «маленьких» кадров -> quality на 1 лучше
#define CAM_QUALITY_RELAX_PCT    50     // «маленький» кадр: меньше этой доли VIDEO_MAX_FRAME, %
#define CAM_INIT_RETRY_MS        2000   // повтор инициализации камеры
#define CAM_FB_FAIL_REINIT       10     // сбоев esp_camera_fb_get подряд -> переинициализация
#define CAM_LOG_PERIOD_MS        5000   // период строки статистики
#define CAM_VIDEO_TXBUF          8192   // TX-буфер UART1, байт
#define CAM_TX_CHUNK_MIN         1024   // минимальная порция записи в UART1
