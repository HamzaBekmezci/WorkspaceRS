#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "math_engine.h"
#include "kinematics.h"
#include "sensor_engine.h"
#include "sim_set.h"
#include "logger.h"
#include "sim_api.h" 
#include "physics_engine.h"

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
    
    // C# arayüzünden sim_init öncesi gelen başlangıç açılarını kaybetmemek için sakla
    Vector3_t saved_init_orient = sim_settings.initial_orientation;

    sim_init_default(&sim_settings); 
    
    // Saklanan açıları geri yükle
    sim_settings.initial_orientation = saved_init_orient;
    
    sim_set_state(&sim_settings, 0); 
    t = 0.0f;

    ideal_payload = (KinematicState_t){0};
    noisy_payload = (KinematicState_t){0};

    // Başlangıç açılarını (derece) radyana çevir ve kuaterniyona dönüştür
    EulerAngles_t init_euler_rad = {
        .roll  = sim_settings.initial_orientation.x * (3.14159265f / 180.0f),
        .pitch = sim_settings.initial_orientation.y * (3.14159265f / 180.0f),
        .yaw   = sim_settings.initial_orientation.z * (3.14159265f / 180.0f)
    };
    
    euler_to_quat(&init_euler_rad, &ideal_payload.orientation);
    quat_normalize(&ideal_payload.orientation);
    quat_to_euler(&ideal_payload.orientation, &ideal_payload.euler_angles);

    noisy_payload.orientation = ideal_payload.orientation;
    noisy_payload.euler_angles = ideal_payload.euler_angles;

    // Log dosyalarını başlat
    const char *header = "Time_s,Roll,Pitch,Yaw,Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z";
    ideal_csv = logger_init("ideal_output.csv", header);
    noisy_csv = logger_init("noisy_output.csv", header);
}

// Arayüzden anlık açı değiştiğinde doğrudan payload yönelimini güncelleyen köprü
SIM_API void sim_api_set_initial_orientation(float roll, float pitch, float yaw) {
    sim_update_initial_orientation(&sim_settings, roll, pitch, yaw);

    EulerAngles_t init_euler_rad = {
        .roll  = roll * (3.14159265f / 180.0f),
        .pitch = pitch * (3.14159265f / 180.0f),
        .yaw   = yaw * (3.14159265f / 180.0f)
    };

    euler_to_quat(&init_euler_rad, &ideal_payload.orientation);
    quat_normalize(&ideal_payload.orientation);
    quat_to_euler(&ideal_payload.orientation, &ideal_payload.euler_angles);

    noisy_payload.orientation = ideal_payload.orientation;
    noisy_payload.euler_angles = ideal_payload.euler_angles;
}

SIM_API void sim_step_auto(float elapsed_time_s, float* acc_x, float* acc_y, float* acc_z, 
                           float* gyro_x, float* gyro_y, float* gyro_z) {
    
    if (sim_settings.is_running == 0) return;

    float dt = 1.0f / sim_settings.update_rate_hz; 
    ImuSensorData_t sensor_output = {0};

    time_accumulator += elapsed_time_s;

    while (time_accumulator >= dt) {
        
        // 1. İDEAL SİSTEM: FİZİK VE KİNEMATİK ENTEGRASYONU
        // Bu adım kendi içinde net kuvveti bulup integrate_kinematics'i tetikler.
        physics_step(&ideal_payload, &sim_settings.rigid_body, dt);

        // 2. SENSÖR MODELLEMESİ (Fizik motorunun bulduğu ideal ivme ve gyroyu okur)
        apply_sensor_model(&ideal_payload, &sim_settings.imu_settings, &sensor_output);

        // 3. GÜRÜLTÜLÜ SİSTEM: KİNEMATİK ENTEGRASYON
        // hatalı ivme ve gyro ile uzaydaki konumunu tahmin etmeye çalışır.
        Vector3_t noisy_accel = {sensor_output.accel_x, sensor_output.accel_y, sensor_output.accel_z};
        Vector3_t noisy_gyro = {sensor_output.gyro_x, sensor_output.gyro_y, sensor_output.gyro_z};
        integrate_kinematics(&noisy_payload, &noisy_accel, &noisy_gyro, dt);

        // 4. LOGLAMA
        if (ideal_csv != NULL) {
            // Loglamada artık target_accel yerine ideal_payload içindeki hesaplanmış fiziksel ivmeyi yazdırıyoruz
            logger_write_row(ideal_csv, t, 
                             ideal_payload.euler_angles.roll, ideal_payload.euler_angles.pitch, ideal_payload.euler_angles.yaw,
                             ideal_payload.acceleration.x, ideal_payload.acceleration.y, ideal_payload.acceleration.z,
                             ideal_payload.angular_rate.x, ideal_payload.angular_rate.y, ideal_payload.angular_rate.z);
        }
        
        if (noisy_csv != NULL) {
            logger_write_row(noisy_csv, t, 
                             noisy_payload.euler_angles.roll, noisy_payload.euler_angles.pitch, noisy_payload.euler_angles.yaw,
                             noisy_accel.x, noisy_accel.y, noisy_accel.z,
                             noisy_gyro.x, noisy_gyro.y, noisy_gyro.z);
        }
        
        t += dt;                 
        time_accumulator -= dt;  
    }

    if(acc_x) *acc_x = sensor_output.accel_x;
    if(acc_y) *acc_y = sensor_output.accel_y;
    if(acc_z) *acc_z = sensor_output.accel_z;
    if(gyro_x) *gyro_x = sensor_output.gyro_x;
    if(gyro_y) *gyro_y = sensor_output.gyro_y;
    if(gyro_z) *gyro_z = sensor_output.gyro_z;
}