#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "math_engine.h"

// Kuaterniyonun zamanla bozulmasını engellemek için normalizasyon (1'e tamamlama) 
// işlemi gereklidir. Bu yardımcı fonksiyonu da buraya ekliyoruz.
void quat_normalize(Quaternion_t *q);

/*
 * KİNEMATİK İNTEGRATÖR FONKSİYONU
 * state: Uydunun o anki durumu (Girdi ve Çıktı olarak güncellenir)
 * body_accel: Uydunun o an maruz kaldığı net doğrusal ivme (X, Y, Z)
 * body_gyro: Uydunun o anki açısal hızı (Roll, Pitch, Yaw oranları - rad/s)
 * dt: Zaman adımı (örneğin 0.01 saniye)
 */
void integrate_kinematics(KinematicState_t *state, const Vector3_t *body_accel, 
                          const Vector3_t *body_gyro,  float dt);

#endif /* KINEMATICS_H */