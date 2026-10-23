// =============================================================================
//  imu_id.h — классификация WHO_AM_I для MPU-семейства (чистая логика, только
//  заголовок, без Arduino.h — покрыта хост-тестом test/test_rov_core).
// =============================================================================
#pragma once

#include <stdint.h>

enum class ImuIdClass : uint8_t {
    Mpu6050,       // 0x68 — оригинал
    Mpu6050Clone,  // 0x72, 0x98 — клоны на GY-521, карта регистров 6050
    Mpu6500,       // 0x70
    Mpu925x,       // 0x71, 0x73 — совместимы, магнитометр не используется
    Unknown
};

inline ImuIdClass imuClassifyWhoAmI(uint8_t id) {
    switch (id) {
        case 0x68: return ImuIdClass::Mpu6050;
        case 0x72:
        case 0x98: return ImuIdClass::Mpu6050Clone;
        case 0x70: return ImuIdClass::Mpu6500;
        case 0x71:
        case 0x73: return ImuIdClass::Mpu925x;
        default:   return ImuIdClass::Unknown;
    }
}

inline const char *imuModelName(ImuIdClass c) {
    switch (c) {
        case ImuIdClass::Mpu6050:      return "MPU6050";
        case ImuIdClass::Mpu6050Clone: return "MPU6050 (клон)";
        case ImuIdClass::Mpu6500:      return "MPU6500";
        case ImuIdClass::Mpu925x:      return "MPU925x (совместимый, магнитометр не используется)";
        case ImuIdClass::Unknown:      return "неизвестный WHO_AM_I";
    }
    return "неизвестный WHO_AM_I";
}

// Совместим ли датчик с используемой настройкой регистров (общая карта 6050/6500)
inline bool imuSupported(ImuIdClass c) { return c != ImuIdClass::Unknown; }

// Нужно ли предупреждение в логе (не оригинальные 6050/6500)
inline bool imuNeedsWarning(ImuIdClass c) {
    return c == ImuIdClass::Mpu6050Clone || c == ImuIdClass::Mpu925x;
}
