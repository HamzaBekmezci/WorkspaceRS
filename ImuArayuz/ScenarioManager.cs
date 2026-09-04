using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;

public class ScenarioManager {
    private List<ScenarioEvent> _events = new List<ScenarioEvent>();
    private float _simTime = 0.0f;
    private bool _isRunning = false;

    public void LoadCsv(string filePath) {
        _events.Clear();
        string[] lines = File.ReadAllLines(filePath);

        // İlk satır başlık olduğu için i = 1'den başlıyoruz
        for (int i = 1; i < lines.Length; i++) {
            string line = lines[i].Trim();
            if (string.IsNullOrEmpty(line)) continue;

            string[] parts = line.Split(',');
            if (parts.Length >= 8) {
                _events.Add(new ScenarioEvent {
                    StartTime = float.Parse(parts[0], CultureInfo.InvariantCulture),
                    EndTime   = float.Parse(parts[1], CultureInfo.InvariantCulture),
                    Fx        = float.Parse(parts[2], CultureInfo.InvariantCulture),
                    Fy        = float.Parse(parts[3], CultureInfo.InvariantCulture),
                    Fz        = float.Parse(parts[4], CultureInfo.InvariantCulture),
                    Tx        = float.Parse(parts[5], CultureInfo.InvariantCulture),
                    Ty        = float.Parse(parts[6], CultureInfo.InvariantCulture),
                    Tz        = float.Parse(parts[7], CultureInfo.InvariantCulture),
                    Description = parts.Length > 8 ? parts[8] : ""
                });
            }
        }
    }

    public void StartSimulation(float mass, float ixx, float iyy, float izz, float linDamp, float angDamp) {
        PhysicsEngineAPI.sim_init();
        PhysicsEngineAPI.sim_api_set_body_params(mass, ixx, iyy, izz, linDamp, angDamp);
        
        _simTime = 0.0f;
        _isRunning = true;
    }

    public void UpdateStep(float dt, 
                       out float ax, out float ay, out float az, 
                       out float gx, out float gy, out float gz,
                       out float posX, out float posY, out float posZ) 
    {
        ax = 0f; ay = 0f; az = 0f;
        gx = 0f; gy = 0f; gz = 0f;
        posX = 0f; posY = 0f; posZ = 0f;
        if (!_isRunning) return;

        float netFx = 0, netFy = 0, netFz = 0;
        float netTx = 0, netTy = 0, netTz = 0;

        foreach (var ev in _events) {
            if (_simTime >= ev.StartTime && _simTime < ev.EndTime) {
                netFx += ev.Fx;
                netFy += ev.Fy;
                netFz += ev.Fz;
                netTx += ev.Tx;
                netTy += ev.Ty;
                netTz += ev.Tz;
            }
        }

        PhysicsEngineAPI.sim_api_set_applied_forces(netFx, netFy, netFz, netTx, netTy, netTz);
        PhysicsEngineAPI.sim_step_auto(dt, out ax, out ay, out az, out gx, out gy, out gz,
                                       out posX, out posY, out posZ);

        _simTime += dt;
    }
}