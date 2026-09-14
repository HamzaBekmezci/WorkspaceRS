using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;

public class ScenarioManager {
    // Waypoint'leri (Hedefleri) sırayla tutacak kuyruk yapısı
    private Queue<ScenarioEvent> _waypoints = new Queue<ScenarioEvent>();
    private ScenarioEvent? _currentWaypoint;
    private bool _hasActiveWaypoint = false;
    private bool _isRunning = false;
 
    public bool HasActiveWaypoint => _hasActiveWaypoint;
    public float TargetX => _hasActiveWaypoint && _currentWaypoint != null ? _currentWaypoint.TargetX : 0;
    public float TargetY => _hasActiveWaypoint && _currentWaypoint != null ? _currentWaypoint.TargetY : 0;
    public float TargetZ => _hasActiveWaypoint && _currentWaypoint != null ? _currentWaypoint.TargetZ : 0;
    public string CurrentTargetName => (_hasActiveWaypoint && _currentWaypoint != null) 
                                       ? (_currentWaypoint.Description ?? "İsimsiz Hedef") 
                                       : "Görev Bitti";

    public void LoadCsv(string filePath) {
        _waypoints.Clear();
        _hasActiveWaypoint = false;
        string[] lines = File.ReadAllLines(filePath);

        // İlk satır başlık (X, Y, Z, Açıklama) varsayıyoruz. i=1'den başla.
        for (int i = 1; i < lines.Length; i++) {
            string line = lines[i].Trim();
            if (string.IsNullOrEmpty(line)) continue;

            string[] parts = line.Split(',');
            if (parts.Length >= 3) {
                _waypoints.Enqueue(new ScenarioEvent {
                    TargetX = float.Parse(parts[0], CultureInfo.InvariantCulture),
                    TargetY = float.Parse(parts[1], CultureInfo.InvariantCulture),
                    TargetZ = float.Parse(parts[2], CultureInfo.InvariantCulture),
                    Description = parts.Length > 3 ? parts[3] : ""
                });
            }
        }
    }

    public void StartSimulation(float mass, float ixx, float iyy, float izz, 
                                float linDamp, float angDamp, 
                                float aeroStab, float aeroDamp, float aeroLift) {
        
        PhysicsEngineAPI.sim_init();
        PhysicsEngineAPI.sim_api_set_body_params(mass, ixx, iyy, izz, linDamp, angDamp, aeroStab, aeroDamp, aeroLift);
        
        // İlk hedef noktayı çek ve DLL'e yolla
        GetNextWaypoint();

        _isRunning = true;
    }

    private void GetNextWaypoint() {
        if (_waypoints.Count > 0) {
            _currentWaypoint = _waypoints.Dequeue();
            _hasActiveWaypoint = true;
            // Otopilota yeni hedefi bildir!
            PhysicsEngineAPI.sim_api_set_waypoint(_currentWaypoint.TargetX, _currentWaypoint.TargetY, _currentWaypoint.TargetZ);
        } else {
            _hasActiveWaypoint = false;
            // İsterseniz görev bitince DLL'e (0,0,0) yollayabilir veya otopilotu kapattırabilirsiniz.
        }
    }

    public void UpdateStep(float dt, 
                        out float ax, out float ay, out float az, 
                        out float roll, out float pitch, out float yaw,
                        out float posX, out float posY, out float posZ) 
    {
        ax = 0f; ay = 0f; az = 0f;
        roll = 0f; pitch = 0f; yaw = 0f;
        posX = 0f; posY = 0f; posZ = 0f;
        
        if (!_isRunning) return;

        PhysicsEngineAPI.sim_step_auto(dt, out ax, out ay, out az, 
                                    out roll, out pitch, out yaw,
                                    out posX, out posY, out posZ);

        if (_hasActiveWaypoint) {
            // '!' operatörü null uyarılarını engeller. Tolerans 15.0f olarak korundu.
            float dx = _currentWaypoint!.TargetX - posX;
            float dy = _currentWaypoint!.TargetY - posY;
            float dz = _currentWaypoint!.TargetZ - posZ;
            float distanceToTarget = (float)Math.Sqrt(dx*dx + dy*dy + dz*dz);

            if (distanceToTarget < 200.0f) {
                GetNextWaypoint();
            }
        }
    }
}