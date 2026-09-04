#ifndef SIM_SET_H
#define SIM_SET_H

#include "sensor_engine.h"
#include "physics_engine.h"

// Simülasyonun genel konfigürasyonunu tutan yapı
typedef struct {
    ImuConfig_t imu_settings;     // Bias ve gürültü ayarları (Arayüzden değişecek)
    float update_rate_hz;         // Simülasyon frekansı (örn: 100 Hz)
    Vector3_t external_force;     // Dış etkenler
    int is_running;               // Simülasyon durumu (1: Çalışıyor, 0: Duraklatıldı)
    RigidBodyParams_t rigid_body; // Fizik motoru parametreleri
    Vector3_t initial_orientation;// Başlangıç Açıları (Roll, Pitch, Yaw)
} SimSettings_t;

// Arayüzün çağıracağı kontrol fonksiyonları
void sim_init_default(SimSettings_t *settings);
void sim_update_noise_levels(SimSettings_t *settings, float accel_noise, float gyro_noise);
void sim_set_state(SimSettings_t *settings, int state);
void sim_update_bias(SimSettings_t *settings, Vector3_t a_bias, Vector3_t g_bias);
void sim_update_hz(SimSettings_t *settings, float new_hz);
void sim_update_target_forces(SimSettings_t *settings, Vector3_t accel, Vector3_t gyro);
void sim_update_initial_orientation(SimSettings_t *settings, float roll, float pitch, float yaw);

#endif // SIM_SET_H