#include "math_engine.h"

/* =========================================
 * 1. VEKTÖR İŞLEMLERİ
 * ========================================= */

void vec_add(const Vector3_t *v1, const Vector3_t *v2, Vector3_t *result) {
    result->x = v1->x + v2->x;
    result->y = v1->y + v2->y;
    result->z = v1->z + v2->z;
}

void vec_sub(const Vector3_t *v1, const Vector3_t *v2, Vector3_t *result) {
    result->x = v1->x - v2->x;
    result->y = v1->y - v2->y;
    result->z = v1->z - v2->z;
}

void vec_scale(const Vector3_t *v, float scalar, Vector3_t *result) {
    result->x = v->x * scalar;
    result->y = v->y * scalar;
    result->z = v->z * scalar;
}

float vec_dot(const Vector3_t *v1, const Vector3_t *v2) {
    return (v1->x * v2->x) + (v1->y * v2->y) + (v1->z * v2->z);
}

// Çapraz çarpım (Özellikle manyetometre ve tork hesaplamaları için hayati)
void vec_cross(const Vector3_t *v1, const Vector3_t *v2, Vector3_t *result) {
    result->x = (v1->y * v2->z) - (v1->z * v2->y);
    result->y = (v1->z * v2->x) - (v1->x * v2->z);
    result->z = (v1->x * v2->y) - (v1->y * v2->x);
}

/* =========================================
 * 2. MATRİS İŞLEMLERİ
 * ========================================= */

void mat_add(const Matrix3x3_t *m1, const Matrix3x3_t *m2, Matrix3x3_t *result) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result->data[i][j] = m1->data[i][j] + m2->data[i][j];
        }
    }
}

// 3x3 Matris çarpımı (M1 * M2)
void mat_mult(const Matrix3x3_t *m1, const Matrix3x3_t *m2, Matrix3x3_t *result) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result->data[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                result->data[i][j] += m1->data[i][k] * m2->data[k][j];
            }
        }
    }
}

void mat_vec_mult(const Matrix3x3_t *m, const Vector3_t *v, Vector3_t *result) {
    result->x = (m->data[0][0] * v->x) + (m->data[0][1] * v->y) + (m->data[0][2] * v->z);
    result->y = (m->data[1][0] * v->x) + (m->data[1][1] * v->y) + (m->data[1][2] * v->z);
    result->z = (m->data[2][0] * v->x) + (m->data[2][1] * v->y) + (m->data[2][2] * v->z);
}

// Transpoz: Gövde (Body) ekseninden Dünya (Global) eksenine geri dönüş için matrisi ters çevirir
void mat_transpose(const Matrix3x3_t *m, Matrix3x3_t *result) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result->data[i][j] = m->data[j][i];
        }
    }
}

/* =========================================
 * 3. YÖNELİM VE KOORDİNAT DÖNÜŞÜMLERİ
 * ========================================= */

void euler_to_dcm(const EulerAngles_t *euler, Matrix3x3_t *dcm) {
    float cos_r = cosf(euler->roll);
    float sin_r = sinf(euler->roll);
    float cos_p = cosf(euler->pitch);
    float sin_p = sinf(euler->pitch);
    float cos_y = cosf(euler->yaw);
    float sin_y = sinf(euler->yaw);

    dcm->data[0][0] = cos_p * cos_y;
    dcm->data[0][1] = (sin_r * sin_p * cos_y) - (cos_r * sin_y);
    dcm->data[0][2] = (cos_r * sin_p * cos_y) + (sin_r * sin_y);

    dcm->data[1][0] = cos_p * sin_y;
    dcm->data[1][1] = (sin_r * sin_p * sin_y) + (cos_r * cos_y);
    dcm->data[1][2] = (cos_r * sin_p * sin_y) - (sin_r * cos_y);

    dcm->data[2][0] = -sin_p;
    dcm->data[2][1] = sin_r * cos_p;
    dcm->data[2][2] = cos_r * cos_p;
}

// Kuaterniyondan 3x3 Dönüşüm Matrisi Oluşturma (Gimbal Lock'suz dönüşüm)
void quat_to_dcm(const Quaternion_t *q, Matrix3x3_t *dcm) {
    // İşlem yükünü hafifletmek için kareleri önceden alıyoruz
    float ww = q->w * q->w;
    float xx = q->x * q->x;
    float yy = q->y * q->y;
    float zz = q->z * q->z;
    float wx = q->w * q->x;
    float wy = q->w * q->y;
    float wz = q->w * q->z;
    float xy = q->x * q->y;
    float xz = q->x * q->z;
    float yz = q->y * q->z;

    dcm->data[0][0] = ww + xx - yy - zz;
    dcm->data[0][1] = 2.0f * (xy - wz);
    dcm->data[0][2] = 2.0f * (xz + wy);

    dcm->data[1][0] = 2.0f * (xy + wz);
    dcm->data[1][1] = ww - xx + yy - zz;
    dcm->data[1][2] = 2.0f * (yz - wx);

    dcm->data[2][0] = 2.0f * (xz - wy);
    dcm->data[2][1] = 2.0f * (yz + wx);
    dcm->data[2][2] = ww - xx - yy + zz;
}

// Z-Y-X (Yaw-Pitch-Roll) sırasına göre Euler'den Kuaterniyon oluşturma
void euler_to_quat(const EulerAngles_t *euler, Quaternion_t *q) {
    // Açıların yarısının trigonometrik değerleri
    float cy = cosf(euler->yaw * 0.5f);
    float sy = sinf(euler->yaw * 0.5f);
    float cp = cosf(euler->pitch * 0.5f);
    float sp = sinf(euler->pitch * 0.5f);
    float cr = cosf(euler->roll * 0.5f);
    float sr = sinf(euler->roll * 0.5f);

    q->w = cr * cp * cy + sr * sp * sy;
    q->x = sr * cp * cy - cr * sp * sy;
    q->y = cr * sp * cy + sr * cp * sy;
    q->z = cr * cp * sy - sr * sp * cy;
}

void quat_to_euler(const Quaternion_t *q, EulerAngles_t *euler) {
    // 1. ROLL (X ekseni etrafında dönüş)
    float sinr_cosp = 2.0f * (q->w * q->x + q->y * q->z);
    float cosr_cosp = 1.0f - 2.0f * (q->x * q->x + q->y * q->y);
    euler->roll = atan2f(sinr_cosp, cosr_cosp);

    // 2. PITCH (Y ekseni etrafında dönüş)
    float sinp = 2.0f * (q->w * q->y - q->z * q->x);
    // Güvenlik: Floating point hatalarından dolayı sinp 1'i veya -1'i çok küçük farkla geçebilir.
    // asinf() fonksiyonu -1 ile 1 dışındaki değerlerde NaN döndürür. Bunu engellemek için sınırlandırıyoruz.
    if (sinp >= 1.0f) {
        euler->pitch = 1.570796f; // pi/2 (90 derece - Yukarı tam dikilme)
    } else if (sinp <= -1.0f) {
        euler->pitch = -1.570796f; // -pi/2 (-90 derece - Aşağı tam dikilme)
    } else {
        euler->pitch = asinf(sinp);
    }

    // 3. YAW (Z ekseni etrafında dönüş)
    float siny_cosp = 2.0f * (q->w * q->z + q->x * q->y);
    float cosy_cosp = 1.0f - 2.0f * (q->y * q->y + q->z * q->z);
    euler->yaw = atan2f(siny_cosp, cosy_cosp);
}