#include "sim_set.h"
#include "physics_engine.h"

void sim_init_default(SimSettings_t *settings) {
    settings->is_running = 0;             // Başlangıçta duruyor
    settings->update_rate_hz = 100.0f;    // 100 Hz veri üretimi

    settings->initial_orientation = (Vector3_t){0.0f, 0.0f, 0.0f};

    settings->imu_settings.accel_noise_std = 0.0f;
    settings->imu_settings.gyro_noise_std = 0.0f;

    settings->imu_settings.accel_bias = (Vector3_t){0.0f, 0.0f, 0.0f};
    settings->imu_settings.gyro_bias = (Vector3_t){0.0f, 0.0f, 0.0f};

    // Fizik motoru varsayılan değerleri
    settings->rigid_body.mass = 1.0f; 
    settings->rigid_body.inertia_diag = (Vector3_t){0.01f, 0.01f, 0.01f}; 
    settings->rigid_body.linear_damping = 0.0f;
    settings->rigid_body.angular_damping = 0.0f;
    settings->rigid_body.applied_force = (Vector3_t){0.0f, 0.0f, 0.0f};
    settings->rigid_body.applied_torque = (Vector3_t){0.0f, 0.0f, 0.0f};

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

// Başlangıç açılarını (Roll, Pitch, Yaw) yapıya kaydeden fonksiyon
void sim_update_initial_orientation(SimSettings_t *settings, float roll, float pitch, float yaw) {
    settings->initial_orientation = (Vector3_t){roll, pitch, yaw};
}