#ifndef SIM_API_H
#define SIM_API_H

// Windows ortamında DLL dışa aktarımı için gerekli makro
#ifdef _WIN32
    #define SIM_API __declspec(dllexport)
#else
    #define SIM_API
#endif

// Derleyicinin fonksiyon isimlerini bozmasını (mangling) engeller
#ifdef __cplusplus
extern "C" {
#endif

// C#'tan çağrılacak fonksiyonların prototipleri
SIM_API void sim_init(void);
SIM_API void sim_step_auto(float elapsed_time_s, float* acc_x, float* acc_y, float* acc_z, 
                           float* gyro_x, float* gyro_y, float* gyro_z);
SIM_API void sim_close(void);

SIM_API void sim_api_set_state(int state);
SIM_API void sim_api_update_hz(float new_hz);
SIM_API void sim_api_update_noise(float accel_noise, float gyro_noise);
SIM_API void sim_api_update_bias(float a_bias_x, float a_bias_y, float a_bias_z, 
                                 float g_bias_x, float g_bias_y, float g_bias_z);
SIM_API void sim_api_update_target_forces(float accel_x, float accel_y, float accel_z, 
                                          float gyro_x, float gyro_y, float gyro_z);
                                          
// Yeni Eklenen Başlangıç Açısı Fonksiyonu
SIM_API void sim_api_set_initial_orientation(float roll, float pitch, float yaw);

#ifdef __cplusplus
}
#endif

#endif // SIM_API_H