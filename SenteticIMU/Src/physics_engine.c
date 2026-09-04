#include "physics_engine.h"
#include "kinematics.h"
#include "math_engine.h"


// 1. ORTAM DİRENCİ VE SÖNÜMLEME (Environmental Damping)

static void apply_environmental_damping(const KinematicState_t *state, const RigidBodyParams_t *body, 
                                        Vector3_t *net_force, Vector3_t *net_torque) 
{
    // 1. DÜNYA (WORLD) HIZINI GÖVDE (BODY) HIZINA ÇEVİR
    Matrix3x3_t dcm_w2b;
    quat_to_dcm(&state->orientation, &dcm_w2b);

    Vector3_t body_velocity;
    mat_vec_mult(&dcm_w2b, &state->velocity, &body_velocity);

    // 2. SÜRTÜNMEYİ GÖVDE HIZI ÜZERİNDEN HESAPLA
    Vector3_t drag_force;
    vec_scale(&body_velocity, -body->linear_damping, &drag_force); 
    
    // Açısal hız zaten Gövde (Body) eksenindedir
    Vector3_t damping_torque;
    vec_scale(&state->angular_rate, -body->angular_damping, &damping_torque); 

    // 3. KUVVET VE TORKLARI TOPLA
    vec_add(&body->applied_force, &drag_force, net_force);
    vec_add(&body->applied_torque, &damping_torque, net_torque);
}

 // 2. RİJİT CİSİM DÖNME DİNAMİĞİ 
static void compute_rotational_dynamics(KinematicState_t *state, const RigidBodyParams_t *body, 
                                        const Vector3_t *net_torque, float dt) 
{
    // Açısal momentum (I * omega)
    Vector3_t i_omega = {
        body->inertia_diag.x * state->angular_rate.x,
        body->inertia_diag.y * state->angular_rate.y,
        body->inertia_diag.z * state->angular_rate.z
    };

    // Jiroskopik Tork: omega x (I * omega)
    Vector3_t gyro_torque;
    vec_cross(&state->angular_rate, &i_omega, &gyro_torque); 

    // Etkin Tork = Net Tork - Jiroskopik Tork
    Vector3_t effective_torque;
    vec_sub(net_torque, &gyro_torque, &effective_torque); 

    // Açısal İvme (alpha) = I^-1 * effective_torque
    Vector3_t alpha = {
        effective_torque.x / body->inertia_diag.x,
        effective_torque.y / body->inertia_diag.y,
        effective_torque.z / body->inertia_diag.z
    };

    // Açısal Hız (omega) güncellemesi: omega = omega + alpha * dt
    state->angular_rate.x += alpha.x * dt;
    state->angular_rate.y += alpha.y * dt;
    state->angular_rate.z += alpha.z * dt;
}


 // 3. ÖTELEME DİNAMİĞİ (Translational Dynamics - Newton's 2nd Law)
static void compute_translational_dynamics(const RigidBodyParams_t *body, const Vector3_t *net_force, 
                                           Vector3_t *body_accel) 
{
    // Gövde İvmesi: a = F / m
    vec_scale(net_force, 1.0f / body->mass, body_accel); 
}


// 4. ANA FİZİK MOTORU ADIMI (Main Physics Step)
void physics_step(KinematicState_t *state, const RigidBodyParams_t *body, float dt) 
{
    Vector3_t net_force;
    Vector3_t net_torque;
    Vector3_t body_accel;

    // 1. Adım: Sürtünme ve Sönümlemeyi hesaplayıp net kuvvet/torku bul
    apply_environmental_damping(state, body, &net_force, &net_torque);

    // 2. Adım: Net tork üzerinden Euler rijit cisim dinamiği ile açısal hızı (omega) güncelle
    compute_rotational_dynamics(state, body, &net_torque, dt);

    // 3. Adım: Net kuvvet üzerinden öteleme ivmesini (a) bul
    compute_translational_dynamics(body, &net_force, &body_accel);

    // --- YERÇEKİMİNİ GÖVDE EKSENİNE ÇEVİR VE İVMEYE EKLE ---
    Matrix3x3_t dcm_w2b;
    // Mevcut kuaterniyon yöneliminden Dünya -> Gövde dönüşüm matrisini al
    quat_to_dcm(&state->orientation, &dcm_w2b); 
    
    // Dünya (World) düzlemindeki sabit yerçekimi vektörü
    Vector3_t global_gravity = {0.0f, 0.0f, -9.80665f}; 
    Vector3_t body_gravity;
    
    // Sabit yerçekimini gövdenin o anki açısına göre döndür
    mat_vec_mult(&dcm_w2b, &global_gravity, &body_gravity);
    
    // Gövdeye etki eden fiziksel kuvvetlerden doğan ivmeye, yerçekimi ivmesini ekle
    vec_add(&body_accel, &body_gravity, &body_accel);
    // -------------------------------------------------------

    // 4. Adım: Kinematik modeli çağırarak ivme ve açısal hızdan pozisyon/yönelim (kuaterniyon) türevlerini entegre et
    integrate_kinematics(state, &body_accel, &state->angular_rate, dt); 
}