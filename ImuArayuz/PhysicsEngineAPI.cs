using System;
using System.Runtime.InteropServices; // <--- BU SATIR EKSİK VEYA UNUTULMUŞ

public static class PhysicsEngineAPI {
    private const string DllName = "libImuSimulator.dll"; // Derlediğiniz DLL adı (CMake'de add_library ile verdiğiniz isim)

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void sim_init();

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void sim_close();

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void sim_api_set_state(int state);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void sim_api_update_hz(float new_hz);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void sim_api_update_noise(float accel_noise, float gyro_noise);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void sim_api_update_bias(float a_bias_x, float a_bias_y, float a_bias_z, 
                                                  float g_bias_x, float g_bias_y, float g_bias_z);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void sim_api_set_initial_orientation(float roll, float pitch, float yaw);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void sim_api_set_body_params(float mass, float ixx, float iyy, float izz, 
                                                      float lin_damp, float ang_damp);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void sim_api_set_applied_forces(float fx, float fy, float fz, 
                                                         float tx, float ty, float tz);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void sim_step_auto(float elapsed_time_s, out float acc_x, out float acc_y, out float acc_z, 
                                        out float gyro_x, out float gyro_y, out float gyro_z,
                                        out float pos_x, out float pos_y, out float pos_z);
}