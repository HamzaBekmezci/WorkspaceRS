#ifndef SENSOR_ENGINE_H
#define SENSOR_ENGINE_H

#include "math_engine.h"

// Sensörün üretim karakteristiklerini tutan konfigürasyon yapısı
typedef struct {
    Vector3_t accel_bias;      // İvmeölçer sabit hatası (m/s^2)
    float accel_noise_std;     // İvmeölçer beyaz gürültü standart sapması (1 sigma)
    
    Vector3_t gyro_bias;       // Jiroskop sabit hatası (rad/s)
    float gyro_noise_std;      // Jiroskop beyaz gürültü standart sapması (1 sigma)
    
    // Rastgele gezinim (Random Walk) gibi ileri düzey hatalar buraya eklenebilir
} ImuConfig_t;

// Uçuş bilgisayarına (FSW) gönderilecek olan nihai, gürültülü "Ham" sensör verisi
typedef struct {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
} ImuSensorData_t;

/*
 * SENSÖR MODELLEME FONKSİYONU
 * ideal_state: Kinematik motordan çıkan kusursuz durum
 * config: MPU9250 veya başka bir IMU'nun hata profili
 * output: Uçuş yazılımına basılacak olan nihai gürültülü veri (Çıktı)
 */
void apply_sensor_model(const KinematicState_t *ideal_state, const ImuConfig_t *config, 
                        ImuSensorData_t *output);

#endif /* SENSOR_ENGINE_H */