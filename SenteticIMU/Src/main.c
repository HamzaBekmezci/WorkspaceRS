#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "math_engine.h"
#include "kinematics.h"
#include "sensor_engine.h"

int main() {
    // 1. Rastgele sayı üretecini (AWGN için) sistem saatiyle başlat
    srand((unsigned int)time(NULL));

    // 2. Sistemin Başlangıç Durumu (Kinematic State)
    // Uydu rampada duruyor. Hız, ivme ve konum sıfır.
    KinematicState_t rasat_payload = {0}; 
    rasat_payload.orientation.w = 1.0f; // Kuaterniyonu identity (0 derece yatıklık) olarak başlat

    // 3. Sensör Profili (MPU9250 Karakteristiği)
    // Bu değerleri sensörün datasheet'inden veya kalibrasyon testlerinden alabilirsin
    ImuConfig_t mpu9250 = {
        .accel_bias = {0.15f, -0.05f, 0.08f},   // m/s^2 cinsinden sabit kayma
        .accel_noise_std = 0.1f,                // İvmeölçer beyaz gürültüsü
        .gyro_bias = {0.02f, -0.01f, 0.01f},    // rad/s cinsinden jiroskop ofseti
        .gyro_noise_std = 0.005f                // Jiroskop beyaz gürültüsü
    };

    // 4. CSV Başlıklarını Konsola Yazdır
    printf("Time_s,Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z\n");

    // 5. Simülasyon Zaman Yönetimi
    float t = 0.0f;
    float dt = 0.01f; // 100 Hz çalışma frekansı
    float sim_duration = 5.0f; // Toplam 5 saniyelik test

    // 6. ANA SİMÜLASYON DÖNGÜSÜ
    while (t <= sim_duration) {
        
        // Bu zaman adımındaki hedef fiziksel kuvvetler
        Vector3_t target_accel = {0.0f, 0.0f, 0.0f};
        Vector3_t target_gyro = {0.0f, 0.0f, 0.0f};

        // --- UÇUŞ PROFİLİ (Durum Makinesi) ---
        if (t > 1.0f && t <= 3.0f) {
            // Fırlatma Fazı: Z ekseninde +25 m/s^2 ivme ve Z ekseninde hafif spin
            target_accel.z = 25.0f; 
            target_gyro.z = 0.5f; // rad/s
        } else if (t > 3.0f) {
           // Serbest Düşüş / İtki Sonu: Motorlar kapandı, sadece yerçekimi ivmesi var (-Z yönünde)
            target_accel.z = -9.80665f;
            target_gyro.z = 0.0f;
        }

        // 1. ADIM: Kinematik Motoru Çalıştır (Fiziği İlerlet)
        integrate_kinematics(&rasat_payload, &target_accel, &target_gyro, dt);

        // 2. ADIM: Sensör Motorunu Çalıştır (Fiziksel veriyi gürültülü IMU verisine çevir)
        ImuSensorData_t sensor_output;
        apply_sensor_model(&rasat_payload, &mpu9250, &sensor_output);

        // 3. ADIM: Çıktıyı CSV formatında bas
        printf("%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
               t,
               sensor_output.accel_x, sensor_output.accel_y, sensor_output.accel_z,
               sensor_output.gyro_x, sensor_output.gyro_y, sensor_output.gyro_z);

        // Zamanı bir adım ileri al
        t += dt;
    }

    return 0;
}