#ifndef MATH_ENGINE_H
#define MATH_ENGINE_H

#include <math.h>

/* ==========================================================================
 * 1. VERİ YAPILARI (DATA STRUCTURES)
 * ========================================================================== */

// 3D Vektör Yapısı (İvme, Hız, Konum, Manyetik Alan vb. için)
typedef struct {
    float x;
    float y;
    float z;
} Vector3_t;

// 3x3 Dönüşüm Matrisi (DCM - Direction Cosine Matrix)
typedef struct {
    float data[3][3];
} Matrix3x3_t;

// Kuaterniyon Yapısı (Gimbal Lock'u önlemek için Yönelim durumu)
typedef struct {
    float w; // Skaler (reel) kısım
    float x; // Vektörel (imajiner) i
    float y; // Vektörel (imajiner) j
    float z; // Vektörel (imajiner) k
} Quaternion_t;

// Euler Açıları (Kullanıcı etkileşimi ve CSV okuma/yazma için)
typedef struct {
    float roll;   // X ekseni etrafında dönüş
    float pitch;  // Y ekseni etrafında dönüş
    float yaw;    // Z ekseni etrafında dönüş
} EulerAngles_t;

typedef struct {
    Vector3_t position;           // Konum [m]
    Vector3_t velocity;           // Hız [m/s]
    Vector3_t acceleration;       // İvme [m/s^2]

    Quaternion_t orientation;     // Açısal Konum (Quarterniyon)
    Vector3_t angular_rate;       // Açısal Hız (p, q, r) [rad/s]

    EulerAngles_t euler_angles;   // Açısal konum (Roll, Pitch, Yaw)
} KinematicState_t;

/* ==========================================================================
 * 2. FONKSİYON PROTOTİPLERİ
 * ========================================================================== */

/*
 * MİMARİ NOTU: Parametrelerde neden işaretçi (pointer) ve 'const' kullanıyoruz?
 * 1. Pointer (*): Struct'ların bellekte kopyalanmasını engeller (Gömülü sistemler için şarttır).
 * 2. const: Fonksiyonun girdi değerini (v1, v2) DEĞİŞTİRMEYECEĞİNİ garanti eder (Contract/Arayüz güvenliği).
 * 3. Çıktılar her zaman 'result' işaretçisi üzerinden döndürülür.
 */

// --- Vektör İşlemleri ---
void vec_add(const Vector3_t *v1, const Vector3_t *v2, Vector3_t *result);
void vec_sub(const Vector3_t *v1, const Vector3_t *v2, Vector3_t *result);
void vec_scale(const Vector3_t *v, float scalar, Vector3_t *result);
float vec_dot(const Vector3_t *v1, const Vector3_t *v2);
void vec_cross(const Vector3_t *v1, const Vector3_t *v2, Vector3_t *result);

// --- Matris İşlemleri ---
void mat_add(const Matrix3x3_t *m1, const Matrix3x3_t *m2, Matrix3x3_t *result);
void mat_mult(const Matrix3x3_t *m1, const Matrix3x3_t *m2, Matrix3x3_t *result);
void mat_vec_mult(const Matrix3x3_t *m, const Vector3_t *v, Vector3_t *result);
void mat_transpose(const Matrix3x3_t *m, Matrix3x3_t *result);

// --- Yönelim ve Koordinat Dönüşümleri ---
// Euler açılarından 3x3 Dönüşüm Matrisi oluşturur
void euler_to_dcm(const EulerAngles_t *euler, Matrix3x3_t *dcm);

// Kuaterniyondan 3x3 Dönüşüm Matrisi oluşturur
void quat_to_dcm(const Quaternion_t *q, Matrix3x3_t *dcm);

// Euler açılarından Kuaterniyon oluşturur
void euler_to_quat(const EulerAngles_t *euler, Quaternion_t *q);

// Kuaterniyondan Euler açılarına dönüşüm (Sonuçlar radyan cinsindendir)
void quat_to_euler(const Quaternion_t *q, EulerAngles_t *euler);

#endif /* MATH_ENGINE_H */