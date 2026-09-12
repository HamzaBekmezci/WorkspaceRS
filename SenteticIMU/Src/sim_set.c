#include "sim_set.h"
#include "physics_engine.h"

void sim_init_default(SimSettings_t *settings) {
    settings->is_running = 0;             // Başlangıçta duruyor
    settings->update_rate_hz = 100.0f;    // 100 Hz veri üretimi

    settings->initial_orientation = (Vector3_t){0.0f, -90.0f, 0.0f};

    settings->imu_settings.accel_noise_std = 0.0f;
    settings->imu_settings.gyro_noise_std = 0.0f;

    settings->imu_settings.accel_bias = (Vector3_t){0.0f, 0.0f, 0.0f};
    settings->imu_settings.gyro_bias = (Vector3_t){0.0f, 0.0f, 0.0f};

    // Fizik motoru varsayılan değerleri
    settings->rigid_body.mass = 25.0f; 
    settings->rigid_body.inertia_diag = (Vector3_t){0.1f, 15.0f, 15.0f}; 
    settings->rigid_body.linear_damping = 0.05f;
    settings->rigid_body.angular_damping = 0.01f;

    // Aerodinamik varsayılanlar
    settings->rigid_body.aero_stability_coeff = 0.005f;
    settings->rigid_body.aero_damping_coeff = 0.5f;
    settings->rigid_body.aero_lift_coeff = 1.0f;

    settings->rigid_body.applied_force = (Vector3_t){0.0f, 0.0f, 0.0f};
    settings->rigid_body.applied_torque = (Vector3_t){0.0f, 0.0f, 0.0f};
}

void sim_set_state(SimSettings_t *settings, int state) {
    settings->is_running = state;
}

void sim_update_noise_levels(SimSettings_t *settings, float accel_noise, float gyro_noise) {
    settings->imu_settings.accel_noise_std = accel_noise;
    settings->imu_settings.gyro_noise_std = gyro_noise;
}

void sim_update_bias(SimSettings_t *settings, Vector3_t a_bias, Vector3_t g_bias) {
    settings->imu_settings.accel_bias = a_bias;
    settings->imu_settings.gyro_bias = g_bias;
}

void sim_update_hz(SimSettings_t *settings, float new_hz) {
    if (new_hz > 0.0f) { 
        settings->update_rate_hz = new_hz;
    }
}

void sim_update_initial_orientation(SimSettings_t *settings, float roll, float pitch, float yaw) {
    settings->initial_orientation = (Vector3_t){roll, pitch, yaw};
}

void sim_update_body_params(SimSettings_t *settings, float mass, float ixx, float iyy, float izz, 
                            float lin_damp, float ang_damp,
                            float aero_stab, float aero_damp, float aero_lift) {
                                
    settings->rigid_body.mass = (mass > 0.001f) ? mass : 1.0f;
    settings->rigid_body.inertia_diag.x = (ixx > 0.0001f) ? ixx : 0.01f;
    settings->rigid_body.inertia_diag.y = (iyy > 0.0001f) ? iyy : 0.01f;
    settings->rigid_body.inertia_diag.z = (izz > 0.0001f) ? izz : 0.01f;
    
    settings->rigid_body.linear_damping = lin_damp;
    settings->rigid_body.angular_damping = ang_damp;

    settings->rigid_body.aero_stability_coeff = (aero_stab > 0.0f) ? aero_stab : 0.0f;
    settings->rigid_body.aero_damping_coeff   = (aero_damp > 0.0f) ? aero_damp : 0.0f;
    settings->rigid_body.aero_lift_coeff      = (aero_lift > 0.0f) ? aero_lift : 0.0f;
}

void sim_update_applied_forces(SimSettings_t *settings, Vector3_t force, Vector3_t torque) {
    settings->rigid_body.applied_force = force;
    settings->rigid_body.applied_torque = torque;
}