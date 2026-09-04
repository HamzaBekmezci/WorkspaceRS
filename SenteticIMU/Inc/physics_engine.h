#ifndef PHYSICS_ENGINE_H
#define PHYSICS_ENGINE_H

#include "math_engine.h"

/* ==========================================================================
 * VERİ YAPILARI
 * ========================================================================== */
/*
 * @brief Rijit cismin (uydunun/roketin) fiziksel özelliklerini ve 
 * o an üzerindeki dış etkileri tutan veri yapısı.
 */
typedef struct {
    float mass;                 // Cisim kütlesi (kg)
    Vector3_t inertia_diag;     // Asal eksen eylemsizlik momentleri (Ixx, Iyy, Izz) [kg*m^2]
    
    // Ortam Sürtünme/Sönümleme Katsayıları
    float linear_damping;       // Doğrusal hava sürtünme katsayısı [N*s/m]
    float angular_damping;      // Açısal sönümleme katsayısı [Nm*s/rad]
    
    // Dışarıdan Uygulanan Yükler (Senaryo Motoru/Kullanıcı tarafından beslenir)
    Vector3_t applied_force;    // Gövdeye uygulanan net dış kuvvet [N] (Örn: Motor itkisi)
    Vector3_t applied_torque;   // Gövdeye uygulanan net dış tork [Nm] (Örn: Kanatçık etkisi)
} RigidBodyParams_t;

/* ==========================================================================
 * FONKSİYON PROTOTİPLERİ
 * ========================================================================== */

/*
 * @brief Fizik motorunun ana döngü adımı. Kuvvet ve torkları alıp, 
 * sürtünmeleri uygulayarak yeni ivme ve açısal hızı hesaplar. Ardından 
 * kinematik entegrasyonu çağırır.
 * 
 * @param state Cismin o anki konumu, hızı, yönelimi vb. (Güncellenir)[cite: 10]
 * @param body  Cismin kütle, atalet ve dış kuvvet/tork parametreleri
 * @param dt    Zaman adımı (Delta time) [saniye]
 */
void physics_step(KinematicState_t *state, const RigidBodyParams_t *body, float dt);

#endif /* PHYSICS_ENGINE_H */