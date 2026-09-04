#include "kinematics.h"
#include "stddef.h"

// Kuaterniyonun boyunu 1'e eşitleyen (Normalize) güvenlik fonksiyonu
void quat_normalize(Quaternion_t *q) {
    float mag = sqrtf((q->w * q->w) + (q->x * q->x) + (q->y * q->y) + (q->z * q->z));
    if (mag > 0.0001f) {
        q->w /= mag;
        q->x /= mag;
        q->y /= mag;
        q->z /= mag;
    }
}

void integrate_kinematics(KinematicState_t *state, const Vector3_t *body_accel, 
                          const Vector3_t *body_gyro, float dt) 
{
    /* ---------------------------------------------------------
     * 1. DOĞRUSAL İNTEGRASYON (Konum ve Hız)
     * ---------------------------------------------------------*/

    // body_accel, gövde (body) eksenindedir. Konum/hız ise dünya (world) 
    // ekseninde tutulduğu için, integrasyondan önce body -> world dönüşümü yapılmalı.
    Matrix3x3_t dcm_w2b;   // world -> body (mevcut yönelime göre)
    Matrix3x3_t dcm_b2w;   // body -> world (transpozu)
    Vector3_t world_accel;

    quat_to_dcm(&state->orientation, &dcm_w2b);
    mat_transpose(&dcm_w2b, &dcm_b2w);
    mat_vec_mult(&dcm_b2w, body_accel, &world_accel);

    // state->acceleration alanını body-frame olarak saklamaya devam ediyoruz
    // (sensor_engine.c bunu body-frame ivme olarak bekliyor).
    state->acceleration = *body_accel;

    // Hızı ve konumu artık WORLD-FRAME ivme ile güncelle
    state->velocity.x += world_accel.x * dt;
    state->velocity.y += world_accel.y * dt;
    state->velocity.z += world_accel.z * dt;

    state->position.x += state->velocity.x * dt;
    state->position.y += state->velocity.y * dt;
    state->position.z += state->velocity.z * dt;

    // C Motorunda zemin kontrolü
    if (state->position.z < 0.0f) {
        state->position.z = 0.0f;
        state->velocity.z = 0.0f;
    }

    /* ---------------------------------------------------------
     * 2. AÇISAL HIZ (ANGULAR RATE) GÜNCELLEMESİ
     * ---------------------------------------------------------*/
    
     state->angular_rate = *body_gyro;   // burası zaten doğru, dokunma

    /* ---------------------------------------------------------
     * 3. AÇISAL İNTEGRASYON (Kuaterniyon Güncellemesi)
     * ---------------------------------------------------------*/
    
    float p = state->angular_rate.x;
    float q_rad = state->angular_rate.y; 
    float r = state->angular_rate.z;

    float q_w = state->orientation.w;
    float q_x = state->orientation.x;
    float q_y = state->orientation.y;
    float q_z = state->orientation.z;

    // Kuaterniyon türevleri
    float dot_w = 0.5f * (-q_x * p - q_y * q_rad - q_z * r);
    float dot_x = 0.5f * ( q_w * p + q_y * r - q_z * q_rad);
    float dot_y = 0.5f * ( q_w * q_rad - q_x * r + q_z * p);
    float dot_z = 0.5f * ( q_w * r + q_x * q_rad - q_y * p);

    // Türevi zamanla çarparak (Euler metodu) yeni kuaterniyonu bul
    state->orientation.w += dot_w * dt;
    state->orientation.x += dot_x * dt;
    state->orientation.y += dot_y * dt;
    state->orientation.z += dot_z * dt;

    // Floating point hataları birikip sistemi bozmasın diye normalize et
    quat_normalize(&state->orientation);

    /* ---------------------------------------------------------
     * 4. EULER AÇILARININ GÜNCELLENMESİ
     * ---------------------------------------------------------*/
    
    quat_to_euler(&state->orientation, &state->euler_angles);
}