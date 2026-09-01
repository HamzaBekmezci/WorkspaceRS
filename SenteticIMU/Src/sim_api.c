#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "math_engine.h"
#include "kinematics.h"
#include "sensor_engine.h"
#include "sim_set.h"
#include "logger.h"
#include "sim_api.h" 

// Hem ideal hem gürültülü durumları ayrı ayrı takip etmek için iki payload
static KinematicState_t ideal_payload; 
static KinematicState_t noisy_payload; 
static float t;
static float time_accumulator = 0.0f;
SimSettings_t sim_settings;


// İki ayrı dosya işaretçisi
static FILE *ideal_csv;
static FILE *noisy_csv;

SIM_API void sim_init(void) {
    srand((unsigned int)time(NULL));
    
    ideal_payload = (KinematicState_t){0};
    ideal_payload.orientation.w = 1.0f;
    
    noisy_payload = (KinematicState_t){0};
    noisy_payload.orientation.w = 1.0f;
    
    sim_init_default(&sim_settings); 
    sim_set_state(&sim_settings, 0); 
    t = 0.0f;

    // Log dosyalarını başlat
    const char *header = "Time_s,Roll,Pitch,Yaw,Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z";
    ideal_csv = logger_init("ideal_output.csv", header);
    noisy_csv = logger_init("noisy_output.csv", header);
}

SIM_API void sim_step_auto(float elapsed_time_s, float* acc_x, float* acc_y, float* acc_z, 
                           float* gyro_x, float* gyro_y, float* gyro_z) {
    
    if (sim_settings.is_running == 0) return;

    // Arayüzden belirlenen güncel Hz'ye göre 1 adımın süresi
    float dt = 1.0f / sim_settings.update_rate_hz; 
    ImuSensorData_t sensor_output = {0}; // Başlangıç değeri

    // C#'tan gelen süreyi (örneğin 0.016 sn) havuza ekle
    time_accumulator += elapsed_time_s;

    // Havuzda en az 1 simülasyon adımı atacak kadar zaman (dt) biriktiği sürece dön
    while (time_accumulator >= dt) {
        
        Vector3_t target_accel = sim_settings.target_accel;
        Vector3_t target_gyro = sim_settings.target_gyro;

        // 1. İdeal model entegrasyonu
        integrate_kinematics(&ideal_payload, &target_accel, &target_gyro, dt);

        // 2. Sensör modeli 
        apply_sensor_model(&ideal_payload, &sim_settings.imu_settings, &sensor_output);

        // 3. Gürültülü model entegrasyonu
        Vector3_t noisy_accel = {sensor_output.accel_x, sensor_output.accel_y, sensor_output.accel_z};
        Vector3_t noisy_gyro = {sensor_output.gyro_x, sensor_output.gyro_y, sensor_output.gyro_z};
        integrate_kinematics(&noisy_payload, &noisy_accel, &noisy_gyro, dt);

        // 4. İdeal veriyi CSV'ye yaz
        if (ideal_csv != NULL) {
            logger_write_row(ideal_csv, t, 
                             ideal_payload.euler_angles.roll, ideal_payload.euler_angles.pitch, ideal_payload.euler_angles.yaw,
                             target_accel.x, target_accel.y, target_accel.z,
                             target_gyro.x, target_gyro.y, target_gyro.z);
        }
        
        // 5. Gürültülü veriyi CSV'ye yaz
        if (noisy_csv != NULL) {
            logger_write_row(noisy_csv, t, 
                             noisy_payload.euler_angles.roll, noisy_payload.euler_angles.pitch, noisy_payload.euler_angles.yaw,
                             noisy_accel.x, noisy_accel.y, noisy_accel.z,
                             noisy_gyro.x, noisy_gyro.y, noisy_gyro.z);
        }
        
        t += dt;                 // Toplam simülasyon süresini ilerlet
        time_accumulator -= dt;  // Havuzdan işlenen 1 adımlık zamanı düş
    }

    // Arayüze güncel sensör verilerini aktar
    if(acc_x) *acc_x = sensor_output.accel_x;
    if(acc_y) *acc_y = sensor_output.accel_y;
    if(acc_z) *acc_z = sensor_output.accel_z;
    if(gyro_x) *gyro_x = sensor_output.gyro_x;
    if(gyro_y) *gyro_y = sensor_output.gyro_y;
    if(gyro_z) *gyro_z = sensor_output.gyro_z;
}

SIM_API void sim_close(void) {
    if (ideal_csv != NULL) logger_close(ideal_csv);
    if (noisy_csv != NULL) logger_close(noisy_csv);
}