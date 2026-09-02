#include "sensor_engine.h"
#include <stdlib.h>
#include <math.h>

// Pi Sayısı
#ifndef M_PI
    #define M_PI 3.14159265358979323846f
#endif

// Yerçekimi sabiti 
#define GRAVITY 9.80665f 


static float generate_gaussian_noise(float std_dev) {
    // log(0) hatasından kaçınmak için çok küçük bir epsilon ekliyoruz.
    float u1 = ((float)rand() / (float)RAND_MAX) + 1e-6f; 
    float u2 = ((float)rand() / (float)RAND_MAX) + 1e-6f;

    // Box-Muller formülü
    float z0 = sqrtf(-2.0f * logf(u1)) * cosf(2.0f * M_PI * u2);
    
    return z0 * std_dev;
}

void apply_sensor_model(const KinematicState_t *ideal_state, const ImuConfig_t *config, 
                        ImuSensorData_t *output) 
{
    // 1. DÜNYA (GLOBAL) EKSENİNDEN GÖVDE (BODY) EKSENİNE GEÇİŞ
    // Uydunun o anki kuaterniyon yöneliminden 3x3 Dönüşüm Matrisini (DCM) elde et.
    Matrix3x3_t dcm;
    quat_to_dcm(&ideal_state->orientation, &dcm);

    // Dünyadaki yerçekimi vektörü
    Vector3_t global_gravity = {0.0f, 0.0f, GRAVITY};
    Vector3_t body_gravity;

    // Yerçekimini, uydunun o anki açısına göre (DCM kullanarak) sensör eksenlerine dağıt
    mat_vec_mult(&dcm, &global_gravity, &body_gravity);

    // İVMEÖLÇER MODELLEMESİ
    output->accel_x = ideal_state->acceleration.x - body_gravity.x 
                      + config->accel_bias.x 
                      + generate_gaussian_noise(config->accel_noise_std);
                      
    output->accel_y = ideal_state->acceleration.y - body_gravity.y 
                      + config->accel_bias.y 
                      + generate_gaussian_noise(config->accel_noise_std);
                      
    output->accel_z = ideal_state->acceleration.z - body_gravity.z 
                      + config->accel_bias.z 
                      + generate_gaussian_noise(config->accel_noise_std);

    // JİROSKOP MODELLEMESİ 
    output->gyro_x = ideal_state->angular_rate.x 
                     + config->gyro_bias.x 
                     + generate_gaussian_noise(config->gyro_noise_std);
                     
    output->gyro_y = ideal_state->angular_rate.y 
                     + config->gyro_bias.y 
                     + generate_gaussian_noise(config->gyro_noise_std);
                     
    output->gyro_z = ideal_state->angular_rate.z 
                     + config->gyro_bias.z 
                     + generate_gaussian_noise(config->gyro_noise_std);
}