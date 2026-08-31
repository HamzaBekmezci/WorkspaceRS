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

    state->acceleration = *body_accel;

    // Hızı güncelle (V = V0 + a*t)[cite: 1]
    state->velocity.x += state->acceleration.x * dt;
    state->velocity.y += state->acceleration.y * dt;
    state->velocity.z += state->acceleration.z * dt;

    // Konumu güncelle (X = X0 + v*t)[cite: 1]
    state->position.x += state->velocity.x * dt;
    state->position.y += state->velocity.y * dt;
    state->position.z += state->velocity.z * dt;

    /* ---------------------------------------------------------
     * 2. AÇISAL HIZ (ANGULAR RATE) GÜNCELLEMESİ
     * ---------------------------------------------------------*/
    
     state->angular_rate = *body_gyro;

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