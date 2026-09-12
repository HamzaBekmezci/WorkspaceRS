#include "physics_engine.h"
#include "kinematics.h"
#include "math_engine.h"


// 1. ORTAM DİRENCİ VE SÖNÜMLEME (Environmental Damping)

static void apply_environmental_and_aero_damping(const KinematicState_t *state, 
                                                 const RigidBodyParams_t *body, 
                                                 const Vector3_t *body_velocity, 
                                                 float dt,
                                                 Vector3_t *net_force, 
                                                 Vector3_t *net_torque) 
{
    // --- 1. SÜRTÜNME (DRAG) VE YANAL TAŞIMA (LIFT) KUVVETİ ---
    Vector3_t aero_force = {0.0f, 0.0f, 0.0f};

    // İleri yön (X ekseni) sürtünmesi (Mevcut Drag)
    aero_force.x = -body->linear_damping * body_velocity->x;

    // --- YANAL KUVVETLER VE STALL (TUTUNMA KAYBI) MODELİ ---
    // Füze hafif açılı uçarken çok yüksek Lift (Taşıma) üretir, böylece manevra yapabilir.
    // Ancak yan düştüğünde (Hücum Açısı > 25-30 derece) kanatçıklar hava akışını koparır (Stall).
    // Bu durumda Lift kuvveti dramatik şekilde çöker ve paraşüt etkisi ortadan kalkar.
    
    float fy = -body->aero_lift_coeff * body_velocity->y * fabsf(body_velocity->y); 
    float fz = -body->aero_lift_coeff * body_velocity->z * fabsf(body_velocity->z);

    float v_lat_sq = (body_velocity->y * body_velocity->y) + (body_velocity->z * body_velocity->z);
    float v_tot_sq = (body_velocity->x * body_velocity->x) + v_lat_sq;

    if (v_tot_sq > 0.1f) {
        float sin2_alpha = v_lat_sq / v_tot_sq; // Hücum açısının sinüs karesi
        float stall_threshold = 0.18f;          // Yaklaşık 25 derece hücum açısı limiti

        if (sin2_alpha > stall_threshold) {
            // Açıklık arttıkça (füze yan döndükçe) Lift kuvveti hızla azalır (Stall Çöküşü)
            float excess = (sin2_alpha - stall_threshold) / (1.0f - stall_threshold); // 0.0 - 1.0 arası
            float stall_factor = 1.0f - 0.95f * excess; // Kuvvet %5'ine kadar düşer (Paraşüt etkisini yokedir)
            fy *= stall_factor;
            fz *= stall_factor;
        }
    }

    aero_force.y = fy;
    aero_force.z = fz;

    // Hızın karesi (v * |v|) dinamik basıncı simüle eder ve yönü korur.


    // --- 2. AÇISAL SÖNÜMLEME (Dynamic Angular Damping) ---
    float speed = sqrtf(body_velocity->x * body_velocity->x + 
                        body_velocity->y * body_velocity->y + 
                        body_velocity->z * body_velocity->z);

    float dynamic_damping_factor = body->angular_damping + (body->aero_damping_coeff * speed);
    
    // Sayısal patlamayı (Numerical Instability) önlemek için maksimum sönümleme torkunu sınırla:
    // T_max = (I * omega) / dt. Bu değer omega'yı tam 0'a indirecek torktur.
    float max_tx = fabsf(body->inertia_diag.x * state->angular_rate.x / dt);
    float max_ty = fabsf(body->inertia_diag.y * state->angular_rate.y / dt);
    float max_tz = fabsf(body->inertia_diag.z * state->angular_rate.z / dt);

    Vector3_t damping_torque;
    damping_torque.x = -dynamic_damping_factor * state->angular_rate.x;
    damping_torque.y = -dynamic_damping_factor * state->angular_rate.y;
    damping_torque.z = -dynamic_damping_factor * state->angular_rate.z;

    // Eğer sönümleme torku, hızı tersine çevirecek kadar büyükse (dt * T / I > omega), sınırla:
    if (damping_torque.x > max_tx) damping_torque.x = max_tx;
    if (damping_torque.x < -max_tx) damping_torque.x = -max_tx;
    
    if (damping_torque.y > max_ty) damping_torque.y = max_ty;
    if (damping_torque.y < -max_ty) damping_torque.y = -max_ty;
    
    if (damping_torque.z > max_tz) damping_torque.z = max_tz;
    if (damping_torque.z < -max_tz) damping_torque.z = -max_tz;


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
    vec_cross(&nose_dir, body_velocity, &error_axis);

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
    Matrix3x3_t dcm_b2w; // quat_to_dcm Body'den World'e dönüşüm matrisi üretir
    quat_to_dcm(&state->orientation, &dcm_b2w); 

    Matrix3x3_t dcm_w2b;
    mat_transpose(&dcm_b2w, &dcm_w2b); // Transpozu World'den Body'ye dönüşüm matrisidir

    Vector3_t body_velocity;
    mat_vec_mult(&dcm_w2b, &state->velocity, &body_velocity);

    // --- 2. SÖNÜMLEME VE AERODİNAMİK HESAPLAMALARI ---
    apply_environmental_and_aero_damping(state, body, &body_velocity, dt, &net_force, &net_torque);
    apply_aerodynamic_restoring_torque(body, &body_velocity, &net_torque);

    // --- 3. DİNAMİK HESAPLAMALAR ---
    compute_rotational_dynamics(state, body, &net_torque, dt);
    compute_translational_dynamics(body, &net_force, &body_accel);

    // --- 4. YERÇEKİMİ ENTEGRASYONU ---
    Vector3_t global_gravity = {0.0f, 0.0f, -9.80665f}; 
    Vector3_t body_gravity;
    
    // Doğru matris (w2b) ile yerçekimini body eksenine çevir
    mat_vec_mult(&dcm_w2b, &global_gravity, &body_gravity);
    
    vec_add(&body_accel, &body_gravity, &body_accel);

    // --- 5. KİNEMATİK ENTEGRASYON ---
    integrate_kinematics(state, &body_accel, &state->angular_rate, dt); 
}