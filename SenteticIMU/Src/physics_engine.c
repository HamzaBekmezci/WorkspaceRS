#include "physics_engine.h"
#include "kinematics.h"
#include "math_engine.h"


// 1. ORTAM DİRENCİ VE SÖNÜMLEME (Environmental Damping)

static void apply_environmental_and_aero_damping(const KinematicState_t *state, 
                                                 const RigidBodyParams_t *body, 
                                                 const Vector3_t *body_velocity, 
                                                 Vector3_t *net_force, 
                                                 Vector3_t *net_torque) 
{
    // --- 1. SÜRTÜNME (DRAG) VE YANAL TAŞIMA (LIFT) KUVVETİ ---
    Vector3_t aero_force = {0.0f, 0.0f, 0.0f};

    // İleri yön (X ekseni) sürtünmesi (Mevcut Drag)
    aero_force.x = -body->linear_damping * body_velocity->x;

    // Yanal Yönler (Y ve Z eksenleri) Taşıma/Direnç Kuvveti (Normal Force / Lift)
    // float lift_coeff = 2.5f; // RigidBodyParams_t'ye eklenecek yeni katsayı
    
    // Yanal hız ne kadar yüksekse (hücum açısı ne kadar büyükse), 
    // yanal yüzeylere o kadar yüksek bir kuvvet etki eder.
    aero_force.y = -body->aero_lift_coeff * body_velocity->y * fabsf(body_velocity->y); 
    aero_force.z = -body->aero_lift_coeff * body_velocity->z * fabsf(body_velocity->z);

    // Hızın karesi (v * |v|) dinamik basıncı simüle eder ve yönü korur.


    // --- 2. AÇISAL SÖNÜMLEME (Dynamic Angular Damping) ---
    float speed = sqrtf(body_velocity->x * body_velocity->x + 
                        body_velocity->y * body_velocity->y + 
                        body_velocity->z * body_velocity->z);

    float dynamic_damping_factor = body->angular_damping + (body->aero_damping_coeff * speed);
    
    Vector3_t damping_torque;
    vec_scale(&state->angular_rate, -dynamic_damping_factor, &damping_torque); 


    // --- 3. KUVVET VE TORKLARI TOPLA ---
    vec_add(&body->applied_force, &aero_force, net_force);
    vec_add(&body->applied_torque, &damping_torque, net_torque);
}

static void apply_aerodynamic_restoring_torque(const RigidBodyParams_t *body, 
                                               const Vector3_t *body_velocity, 
                                               Vector3_t *net_torque) 
{
    // Hızın büyüklüğünü hesapla
    float speed = sqrtf(body_velocity->x * body_velocity->x + 
                        body_velocity->y * body_velocity->y + 
                        body_velocity->z * body_velocity->z);

    // Hız çok düşükse aerodinamik kuvvet oluşmaz, işlem yapmadan çık
    if (speed < 0.1f) {
        return; 
    }

    // Cismin referans "ileri" yönü (Gövde koordinat sisteminde +X eksenini burun kabul ediyoruz)
    Vector3_t nose_dir = {1.0f, 0.0f, 0.0f}; 

    // Yönelim sapmasını (Hücum açısı kaynaklı hata) vektörel çarpım ile bul
    // error_axis = nose_dir x body_velocity
    // Bu vektör dönüş eksenini ve sin(alpha) sapma miktarını barındırır
    Vector3_t error_axis;
    vec_cross(body_velocity, &nose_dir, &error_axis);

    // Aerodinamik Tork = Katsayı * Hız * Sapma_Ekseni
    // error_axis büyüklüğü |v| ile orantılıdır. Bunu bir kez daha 'speed' ile çarpmak,
    // üretilen torkun dinamik basınç kuralına uygun olarak V^2 ile orantılı olmasını sağlar.
    Vector3_t aero_torque;
    vec_scale(&error_axis, body->aero_stability_coeff * speed, &aero_torque);

    // Üretilen aerodinamik torku toplam net torka ekle
    vec_add(net_torque, &aero_torque, net_torque);
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

    // --- 1. DÜNYA (WORLD) HIZINI GÖVDE (BODY) HIZINA ÇEVİR ---
    Matrix3x3_t dcm_w2b;
    quat_to_dcm(&state->orientation, &dcm_w2b); // Kuaterniyondan Dönüşüm Matrisi elde et

    Vector3_t body_velocity;
    mat_vec_mult(&dcm_w2b, &state->velocity, &body_velocity);

    // --- 2. SÖNÜMLEME VE AERODİNAMİK HESAPLAMALARI ---
    // a) Sürtünme ve hıza bağlı açısal sönümlemeyi uygula
    apply_environmental_and_aero_damping(state, body, &body_velocity, &net_force, &net_torque);

    // b) Yönelimi hıza hizalayan aerodinamik (kanatçık) torkunu uygula
    apply_aerodynamic_restoring_torque(body, &body_velocity, &net_torque);


    // --- 3. DİNAMİK HESAPLAMALAR ---
    // Rotasyonel dinamik (Jiroskopik etkiler dahil açısal ivmeyi bul ve omega'yı güncelle)
    compute_rotational_dynamics(state, body, &net_torque, dt);

    // Öteleme dinamiği (a = F/m)       
    compute_translational_dynamics(body, &net_force, &body_accel);


    // --- 4. YERÇEKİMİ ENTEGRASYONU ---
    // Dünya eksenindeki yerçekimini gövde eksenine çevir
    Vector3_t global_gravity = {0.0f, 0.0f, -9.80665f}; 
    Vector3_t body_gravity;
    
    mat_vec_mult(&dcm_w2b, &global_gravity, &body_gravity);
    
    // İvmeye yerçekimini ekle
    vec_add(&body_accel, &body_gravity, &body_accel);


    // --- 5. KİNEMATİK ENTEGRASYON ---
    // İvme ve açısal hızı entegre ederek yeni pozisyon ve yönelimi (kuaterniyon) bul
    integrate_kinematics(state, &body_accel, &state->angular_rate, dt); 
}