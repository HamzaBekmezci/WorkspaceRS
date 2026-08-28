#include "kinematics.h"

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
     * Euler Integrasyonu: V_yeni = V_eski + (A * dt)
     *                     X_yeni = X_eski + (V * dt)
     * ---------------------------------------------------------*/
    
    // Uydunun ivmesini kaydet (Sensör modelinde kullanılacak)
    state->acceleration = *body_accel;
    state->angular_rate = *body_gyro;

    // Hızı güncelle (V = V0 + a*t)
    state->velocity.x += body_accel->x * dt;
    state->velocity.y += body_accel->y * dt;
    state->velocity.z += body_accel->z * dt;

    // Konumu güncelle (X = X0 + v*t)
    state->position.x += state->velocity.x * dt;
    state->position.y += state->velocity.y * dt;
    state->position.z += state->velocity.z * dt;

    /* ---------------------------------------------------------
     * 2. AÇISAL İNTEGRASYON (Kuaterniyon Güncellemesi)
     * Formül: q_dot = 0.5 * q * omega
     * Burada omega, uydunun açısal hız vektörüdür (p, q, r).
     * ---------------------------------------------------------*/
    
    float p = body_gyro->x;
    float q_rad = body_gyro->y; // 'q' harfi kuaterniyonla karışmasın diye q_rad dedik
    float r = body_gyro->z;

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
}