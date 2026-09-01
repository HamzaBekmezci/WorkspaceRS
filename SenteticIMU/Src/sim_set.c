#include "sim_set.h"


void sim_init_default(SimSettings_t *settings) {
    settings->is_running = 0;             // Başlangıçta duruyor
    settings->update_rate_hz = 100.0f;    // 100 Hz veri üretimi

    settings->target_accel = (Vector3_t){0.0f, 0.0f, 0.0f};
    settings->target_gyro = (Vector3_t){0.0f, 0.0f, 0.0f};

    settings->initial_orientation = (Vector3_t){0.0f, 0.0f, 0.0f};

    settings->imu_settings.accel_noise_std = 0.0f;
    settings->imu_settings.gyro_noise_std = 0.0f;

    settings->imu_settings.accel_bias = (Vector3_t){0.0f, 0.0f, 0.0f};
    settings->imu_settings.gyro_bias = (Vector3_t){0.0f, 0.0f, 0.0f};

}

void sim_set_state(SimSettings_t *settings, int state) {
    settings->is_running = state;
}

void sim_update_noise_levels(SimSettings_t *settings, float accel_noise, float gyro_noise) {
    // Arayüzdeki slider'lar hareket ettikçe bu fonksiyon çağrılacak
    settings->imu_settings.accel_noise_std = accel_noise;
    settings->imu_settings.gyro_noise_std = gyro_noise;
}

// Arayüzden gelen yeni bias (kayma) değerlerini anlık olarak uygular
void sim_update_bias(SimSettings_t *settings, Vector3_t a_bias, Vector3_t g_bias) {
    settings->imu_settings.accel_bias = a_bias;
    settings->imu_settings.gyro_bias = g_bias;
}

// Arayüzden gelen yeni frekans (Hz) değerini uygular
void sim_update_hz(SimSettings_t *settings, float new_hz) {
    if (new_hz > 0.0f) { // Güvenlik: 0 veya negatif Hz olamaz
        settings->update_rate_hz = new_hz;
    }
}

void sim_update_target_forces(SimSettings_t *settings, Vector3_t accel, Vector3_t gyro) {
    settings->target_accel = accel;
    settings->target_gyro = gyro;
} 

// Başlangıç açılarını (Roll, Pitch, Yaw) yapıya kaydeden fonksiyon
void sim_update_initial_orientation(SimSettings_t *settings, float roll, float pitch, float yaw) {
    settings->initial_orientation = (Vector3_t){roll, pitch, yaw};
}