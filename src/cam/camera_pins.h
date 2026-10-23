// =============================================================================
//  camera_pins.h — разводка камеры OV2640 на плате ESP32-CAM AI-Thinker.
//  Это описание платы (схема AI-Thinker), а не настройки проекта: значения
//  совпадают со стандартным набором CAMERA_MODEL_AI_THINKER из esp32-camera.
//  Пины видеолинии и вспышки — в include/config.h ([CAM]).
//
//  Свободные от камеры линии AI-Thinker: GPIO 2, 4, 12, 13, 14, 15 (слот SD).
//  SD-карта не используется, SD_MMC нигде не инициализируется:
//  GPIO13 (HS2_DATA3) занят под UART1 TX видео, GPIO4 (HS2_DATA1) — вспышка.
// =============================================================================
#pragma once

#define CAM_PIN_PWDN      32   // питание сенсора (HIGH = выключен)
#define CAM_PIN_RESET     -1   // аппаратный сброс не разведён
#define CAM_PIN_XCLK       0
#define CAM_PIN_SIOD      26   // SCCB SDA
#define CAM_PIN_SIOC      27   // SCCB SCL

#define CAM_PIN_D7        35   // Y9
#define CAM_PIN_D6        34   // Y8
#define CAM_PIN_D5        39   // Y7
#define CAM_PIN_D4        36   // Y6
#define CAM_PIN_D3        21   // Y5
#define CAM_PIN_D2        19   // Y4
#define CAM_PIN_D1        18   // Y3
#define CAM_PIN_D0         5   // Y2
#define CAM_PIN_VSYNC     25
#define CAM_PIN_HREF      23
#define CAM_PIN_PCLK      22

#define CAM_XCLK_FREQ_HZ  20000000   // типовая тактовая OV2640 на AI-Thinker
