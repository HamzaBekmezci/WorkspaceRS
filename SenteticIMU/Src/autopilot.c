#include "autopilot.h"
#include "math_engine.h"
#include <math.h>

#ifndef M_PI
    #define M_PI 3.14159265358979323846f
#endif

// --- STATİK (GİZLİ) DEĞİŞKENLER ---
static Vector3_t current_waypoint = {0.0f, 0.0f, 0.0f};
static int has_waypoint = 0;

// PID (PD) Katsayıları
static float kp_angle = 280.0f;  
static float kd_angle = 90.0f;   
static float thrust_force = 1000.0f;

void autopilot_init(void) {
    has_waypoint = 0;
    current_waypoint = (Vector3_t){0.0f, 0.0f, 0.0f};
}

void autopilot_set_waypoint(float x, float y, float z) {
    current_waypoint.x = x;
    current_waypoint.y = y;
    current_waypoint.z = z;
    has_waypoint = 1;
}

void autopilot_step(KinematicState_t *state, RigidBodyParams_t *body, float dt) {
    (void)dt;

    // 1. Hedef Yoksa Durdur
    if (!has_waypoint) {
        body->applied_force = (Vector3_t){0.0f, 0.0f, 0.0f};
        body->applied_torque = (Vector3_t){0.0f, 0.0f, 0.0f};
        return;
    }

    // 2. Mesafe Hesaplama
    Vector3_t pos_error;
    vec_sub(&current_waypoint, &state->position, &pos_error);
    float distance = sqrtf(vec_dot(&pos_error, &pos_error));

    // Hedefin merkezine çok yaklaşıldıysa süzül
    if (distance < 5.0f) {
        body->applied_force = (Vector3_t){0.0f, 0.0f, 0.0f};
        body->applied_torque = (Vector3_t){0.0f, 0.0f, 0.0f};
        return;
    }

    // 3. Açı ve Hata Hesaplamaları (ÖNCE YAPILMALI)
    float target_yaw = atan2f(pos_error.y, pos_error.x);

    // Füzenin yerçekimine yenilmemesi için mesafeye orantılı bir "sanal yükseklik" avansı veriyoruz.
    float gravity_bias = 0.0f;
    if (distance > 100.0f && distance < 3000.0f) {
        // Katsayı 0.08 ve fminf ile maksimum 40 metre avansla sınırlandırıldı
        gravity_bias = fminf((distance - 100.0f) * 0.05f, 40.0f);
    }
    float virtual_target_z = pos_error.z + gravity_bias;

    float target_pitch = atan2f(-virtual_target_z, sqrtf(pos_error.x * pos_error.x + pos_error.y * pos_error.y));

    float yaw_err = target_yaw - state->euler_angles.yaw;
    float pitch_err = target_pitch - state->euler_angles.pitch;

    // Açıları Sarmalama (-PI, PI arasına alma)
    while(yaw_err > M_PI) yaw_err -= 2.0f * M_PI;
    while(yaw_err < -M_PI) yaw_err += 2.0f * M_PI;
    while(pitch_err > M_PI) pitch_err -= 2.0f * M_PI;
    while(pitch_err < -M_PI) pitch_err += 2.0f * M_PI;

    float roll_err = 0.0f - state->euler_angles.roll;

    // 4. Tork Üretimi (DİNAMİK KATSAYILAR KULLANILDI)
    float total_angle_error = fabsf(yaw_err) + fabsf(pitch_err);

    // Temel PID katsayıları
    float active_kp = kp_angle;
    float active_kd = kd_angle;
    float current_max_torque = 450.0f;

    // ANI DÖNÜŞ MODU: Eğer açı hatası büyükse (keskin bir dönüş yapılıyorsa)
    if (total_angle_error > 0.6f) { // Yaklaşık 45 derece ve üzeri hatalar
        active_kp = kp_angle * 2.0f;       // Dönüş kas gücünü 2 katına çıkar
        active_kd = kd_angle * 0.5f;       // Frenlemeyi yarıya düşür ki dönüşü engellemesin
        current_max_torque = 900.0f;       // Tork sınırını 900 Nm'ye fırlat (Ani manevra gücü)
    }
    else if (distance < 400.0f) {
        // Hedefe yakın mesafe sörfü
        active_kp = kp_angle * 1.5f; 
        active_kd = kd_angle * 0.2f; 
        current_max_torque = 450.0f;
    }
    
    // Tork denklemleri
    float torque_y = (active_kp * pitch_err) - (active_kd * state->angular_rate.y); 
    float torque_z = (active_kp * yaw_err) - (active_kd * state->angular_rate.z);   
    float torque_x = (active_kp * 0.1f * roll_err) - (active_kd * 0.2f * state->angular_rate.x); 

    // EKSENLERE GÖRE TORK SINIRLARI
    float max_torque_roll = 10.0f;      
    float max_torque_pitch_yaw = current_max_torque; // Artık dinamik tork sınırımız var

    // Sınırlandırmalar
    if (torque_x > max_torque_roll) torque_x = max_torque_roll;
    if (torque_x < -max_torque_roll) torque_x = -max_torque_roll;
    
    if (torque_y > max_torque_pitch_yaw) torque_y = max_torque_pitch_yaw;
    if (torque_y < -max_torque_pitch_yaw) torque_y = -max_torque_pitch_yaw;
    
    if (torque_z > max_torque_pitch_yaw) torque_z = max_torque_pitch_yaw;
    if (torque_z < -max_torque_pitch_yaw) torque_z = -max_torque_pitch_yaw;

    body->applied_torque = (Vector3_t){torque_x, torque_y, torque_z};
    // 5. Akıllı İtki Kontrolü (GÜVENLİ VE STALL KORUMALI)

    if (distance < 400.0f) {
        // 1. ÖNCELİK: HEDEFE YAKINLIK
        if (total_angle_error > 0.3f) {
            // DÜZELTME 2: Güç ASLA %35'in (350 Newton) altına inmemeli!
            body->applied_force = (Vector3_t){thrust_force * 0.35f, 0.0f, 0.0f}; 
        } else {
            // Hedefe doğru güzelce bakıyor, süzülerek girmesi için %50 güç
            body->applied_force = (Vector3_t){thrust_force * 0.5f, 0.0f, 0.0f};
        }
    }
    else if (total_angle_error > 0.3f) {
        // 2. ÖNCELİK: UZAK MESAFEDE DÖNÜŞ
        // Hızlıca yeni rotaya yönelirken enerjiyi korumak için %60 güç
        body->applied_force = (Vector3_t){thrust_force * 0.6f, 0.0f, 0.0f}; 
    } 
    else {
        // 3. ÖNCELİK: DÜZ UÇUŞ
        body->applied_force = (Vector3_t){thrust_force, 0.0f, 0.0f}; 
    }
}