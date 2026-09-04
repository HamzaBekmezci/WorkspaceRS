#include "sim_set.h"

#ifdef _WIN32
    #define SIM_API __declspec(dllexport)
#else
    #define SIM_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

extern SimSettings_t sim_settings; 

SIM_API void sim_api_set_state(int state) {
    sim_set_state(&sim_settings, state);
}

SIM_API void sim_api_update_hz(float new_hz) {
    sim_update_hz(&sim_settings, new_hz);
}

SIM_API void sim_api_update_noise(float accel_noise, float gyro_noise) {
    sim_update_noise_levels(&sim_settings, accel_noise, gyro_noise);
}

SIM_API void sim_api_update_bias(float a_bias_x, float a_bias_y, float a_bias_z, 
                                 float g_bias_x, float g_bias_y, float g_bias_z) {
    Vector3_t a_bias = {a_bias_x, a_bias_y, a_bias_z};
    Vector3_t g_bias = {g_bias_x, g_bias_y, g_bias_z};
    sim_update_bias(&sim_settings, a_bias, g_bias);
}

SIM_API void sim_api_set_body_params(float mass, float ixx, float iyy, float izz, 
                                     float lin_damp, float ang_damp) 
{
    // Güvenlik: Kütle ve eylemsizlik 0 veya negatif olamaz
    sim_settings.rigid_body.mass = (mass > 0.001f) ? mass : 1.0f;
    sim_settings.rigid_body.inertia_diag.x = (ixx > 0.0001f) ? ixx : 0.01f;
    sim_settings.rigid_body.inertia_diag.y = (iyy > 0.0001f) ? iyy : 0.01f;
    sim_settings.rigid_body.inertia_diag.z = (izz > 0.0001f) ? izz : 0.01f;
    
    sim_settings.rigid_body.linear_damping = lin_damp;
    sim_settings.rigid_body.angular_damping = ang_damp;
}

SIM_API void sim_api_set_applied_forces(float fx, float fy, float fz, 
                                        float tx, float ty, float tz) 
{
    sim_settings.rigid_body.applied_force = (Vector3_t){fx, fy, fz};
    sim_settings.rigid_body.applied_torque = (Vector3_t){tx, ty, tz};
}

#ifdef __cplusplus
}
#endif