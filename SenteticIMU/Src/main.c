#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <windows.h>
#include "math_engine.h"
#include "kinematics.h"
#include "sensor_engine.h"
#include "sim_set.h"
#include "logger.h"

int main() {
    srand((unsigned int)time(NULL));

    KinematicState_t ideal_payload = {0}; 
    ideal_payload.orientation.w = 1.0f;

    KinematicState_t noisy_payload = {0}; 
    noisy_payload.orientation.w = 1.0f;

    SimSettings_t sim_settings;
    sim_init_default(&sim_settings); 
    // sim_set_state(&sim_settings, 1); // Manuel olarak başlatma komutu

    const char *header_format = "Time_s,Roll,Pitch,Yaw,Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z";
    
    FILE *ideal_csv = logger_init("ideal_output.csv", header_format);
    FILE *noisy_csv = logger_init("noisy_output.csv", header_format);
    
    if (ideal_csv == NULL || noisy_csv == NULL) {
        printf("Hata: CSV dosyalari acilamadi!\n");
        return -1;
    }

    float t = 0.0f;

    while (1) {
        if (sim_settings.is_running == 0) {
            Sleep(1);
            continue; 
        }

        float dt = 1.0f / sim_settings.update_rate_hz;

        // sim_settings üzerinden güncel veriyi alıyoruz
        Vector3_t target_accel = sim_settings.target_accel;
        Vector3_t target_gyro = sim_settings.target_gyro;

        integrate_kinematics(&ideal_payload, &target_accel, &target_gyro, dt);

        ImuSensorData_t sensor_output;
        apply_sensor_model(&ideal_payload, &sim_settings.imu_settings, &sensor_output);

        Vector3_t noisy_accel = {sensor_output.accel_x, sensor_output.accel_y, sensor_output.accel_z};
        Vector3_t noisy_gyro = {sensor_output.gyro_x, sensor_output.gyro_y, sensor_output.gyro_z};
        
        integrate_kinematics(&noisy_payload, &noisy_accel, &noisy_gyro, dt);

        logger_write_row(ideal_csv, t, 
                         ideal_payload.euler_angles.roll, ideal_payload.euler_angles.pitch, ideal_payload.euler_angles.yaw,
                         target_accel.x, target_accel.y, target_accel.z,
                         target_gyro.x, target_gyro.y, target_gyro.z);
                
        logger_write_row(noisy_csv, t, 
                         noisy_payload.euler_angles.roll, noisy_payload.euler_angles.pitch, noisy_payload.euler_angles.yaw,
                         sensor_output.accel_x, sensor_output.accel_y, sensor_output.accel_z,
                         sensor_output.gyro_x, sensor_output.gyro_y, sensor_output.gyro_z);

        t += dt;
    }

    logger_close(ideal_csv);
    logger_close(noisy_csv);
    
    return 0;
}