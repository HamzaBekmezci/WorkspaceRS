using System;
using System.Diagnostics;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Media3D;

namespace ImuArayuz
{
    public partial class MainWindow : Window
    {
        private ScenarioManager simManager = new ScenarioManager();

        private System.Windows.Threading.DispatcherTimer uiTimer = new System.Windows.Threading.DispatcherTimer();
        private Stopwatch stopwatch = new Stopwatch();
        private int isSimRunning = 0;
        private bool isInitialized = false;

        // 1. Küp (PRY) İçin Açısal Dönüş Referansları
        private RotateTransform3D rotateX1 = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(1, 0, 0), 0));
        private RotateTransform3D rotateY1 = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 1, 0), 0));
        private RotateTransform3D rotateZ1 = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 0, 1), 0));

        // 2. Küp (Etki Kuvvetleri / Vektör) İçin Dinamik Eksen-Açı Referansı
        private AxisAngleRotation3D forceAxisAngle = new AxisAngleRotation3D(new Vector3D(0, 0, 1), 0);

        public MainWindow()
        {
            InitializeComponent();

            Transform3DGroup group1, group2;
            ModelVisual3D cube1 = CreateCubeModel(out group1, rotateX1, rotateY1, rotateZ1);
            ModelVisual3D cube2 = CreateCubeModelCustom(out group2, new RotateTransform3D(forceAxisAngle));

            ViewportPRY.Children.Add(cube1);
            ViewportForces.Children.Add(cube2);

            try
            {
                PhysicsEngineAPI.sim_init();
                PhysicsEngineAPI.sim_api_set_body_params(1.0f, 0.01f, 0.01f, 0.01f, 0.0f, 0.0f);
            }
            catch (Exception ex)
            {
                MessageBox.Show($"C Motoru DLL hatası:\n\n{ex.Message}", "Kritik Hata", MessageBoxButton.OK, MessageBoxImage.Error);
                return; 
            }

            isInitialized = true;
            stopwatch.Start();

            uiTimer.Interval = TimeSpan.FromMilliseconds(16);
            uiTimer.Tick += UiTimer_Tick;
            uiTimer.Start();
        }

        private ModelVisual3D CreateCubeModel(out Transform3DGroup transformGroup, RotateTransform3D rx, RotateTransform3D ry, RotateTransform3D rz)
        {
            Transform3DGroup tGroup = new Transform3DGroup();
            tGroup.Children.Add(rx);
            tGroup.Children.Add(ry);
            tGroup.Children.Add(rz);

            transformGroup = tGroup;
            return CreateCubeModelCustom(out _, tGroup);
        }

        private ModelVisual3D CreateCubeModelCustom(out Transform3DGroup transformGroup, Transform3D customTransform)
        {
            Model3DGroup cubeGroup = new Model3DGroup();

            Point3D[] pts = {
                new Point3D(-0.5, -0.5, -0.5),
                new Point3D( 0.5, -0.5, -0.5),
                new Point3D( 0.5,  0.5, -0.5),
                new Point3D(-0.5,  0.5, -0.5),
                new Point3D(-0.5, -0.5,  0.5),
                new Point3D( 0.5, -0.5,  0.5),
                new Point3D( 0.5,  0.5,  0.5),
                new Point3D(-0.5,  0.5,  0.5)
            };

            var faces = new[]
            {
                new { Indices = new int[] { 4, 5, 6,  4, 6, 7 }, Brush = Brushes.Red },
                new { Indices = new int[] { 1, 0, 3,  1, 3, 2 }, Brush = Brushes.DarkGray },
                new { Indices = new int[] { 0, 4, 7,  0, 7, 3 }, Brush = Brushes.MidnightBlue },
                new { Indices = new int[] { 5, 1, 2,  5, 2, 6 }, Brush = Brushes.Navy },
                new { Indices = new int[] { 7, 6, 2,  7, 2, 3 }, Brush = Brushes.ForestGreen },
                new { Indices = new int[] { 0, 1, 5,  0, 5, 4 }, Brush = Brushes.DarkGreen }
            };

            foreach (var face in faces)
            {
                MeshGeometry3D mesh = new MeshGeometry3D();
                foreach (var pt in pts) mesh.Positions.Add(pt);
                foreach (int idx in face.Indices) mesh.TriangleIndices.Add(idx);

                GeometryModel3D model = new GeometryModel3D(mesh, new DiffuseMaterial(face.Brush));
                cubeGroup.Children.Add(model);
            }

            cubeGroup.Children.Add(CreateAxisBar(new Point3D(0, 0, 0), new Point3D(1.0, 0, 0), Brushes.Green));
            cubeGroup.Children.Add(CreateAxisBar(new Point3D(0, 0, 0), new Point3D(0, 1.0, 0), Brushes.Red));
            cubeGroup.Children.Add(CreateAxisBar(new Point3D(0, 0, 0), new Point3D(0, 0, 1.0), Brushes.Blue));

            Transform3DGroup rootGroup = new Transform3DGroup();
            rootGroup.Children.Add(customTransform);
            rootGroup.Children.Add(new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(1, 0, 0), -90)));

            cubeGroup.Transform = rootGroup;
            transformGroup = rootGroup;

            return new ModelVisual3D { Content = cubeGroup };
        }

        private GeometryModel3D CreateAxisBar(Point3D p1, Point3D p2, Brush color)
        {
            MeshGeometry3D mesh = new MeshGeometry3D();
            double th = 0.02;
            double length = (p2 - p1).Length;

            Point3D[] pts;
            if (p2.X > 0)
            {
                pts = new[] {
                    new Point3D(0, -th, -th), new Point3D(length, -th, -th), new Point3D(length, th, -th), new Point3D(0, th, -th),
                    new Point3D(0, -th, th),  new Point3D(length, -th, th),  new Point3D(length, th, th),  new Point3D(0, th, th)
                };
            }
            else if (p2.Y > 0)
            {
                pts = new[] {
                    new Point3D(-th, 0, -th), new Point3D(th, 0, -th), new Point3D(th, length, -th), new Point3D(-th, length, -th),
                    new Point3D(-th, 0, th),  new Point3D(th, 0, th),  new Point3D(th, length, th),  new Point3D(-th, length, th)
                };
            }
            else
            {
                pts = new[] {
                    new Point3D(-th, -th, 0), new Point3D(th, -th, 0), new Point3D(th, th, 0), new Point3D(-th, th, 0),
                    new Point3D(-th, -th, length), new Point3D(th, -th, length), new Point3D(th, th, length), new Point3D(-th, th, length)
                };
            }

            foreach (var pt in pts) mesh.Positions.Add(pt);

            int[] indices = { 
                0, 1, 2, 0, 2, 3, 
                4, 6, 5, 4, 7, 6, 
                0, 4, 5, 0, 5, 1, 
                1, 5, 6, 1, 6, 2, 
                2, 6, 7, 2, 7, 3, 
                3, 7, 4, 3, 4, 0 
            };
            foreach (int idx in indices) mesh.TriangleIndices.Add(idx);

            return new GeometryModel3D(mesh, new DiffuseMaterial(color));
        }

       private void ToggleSim_Click(object? sender, RoutedEventArgs? e)
        {
            isSimRunning = (isSimRunning == 0) ? 1 : 0;
            PhysicsEngineAPI.sim_api_set_state(isSimRunning);

            if (isSimRunning == 1)
            {
                BtnToggleSim.Content = "Simülasyonu Durdur";
                BtnToggleSim.Background = new SolidColorBrush(Colors.Red);
            }
            else
            {
                BtnToggleSim.Content = "Simülasyonu Başlat";
                BtnToggleSim.Background = new SolidColorBrush(Color.FromRgb(76, 175, 80));
            }
        }

        private void LoadScenario_Click(object sender, RoutedEventArgs e)
        {
            Microsoft.Win32.OpenFileDialog openFileDialog = new Microsoft.Win32.OpenFileDialog();
            openFileDialog.Filter = "CSV Dosyaları (*.csv)|*.csv|Tüm Dosyalar (*.*)|*.*";
            
            if (openFileDialog.ShowDialog() == true)
            {
                try
                {
                    simManager.LoadCsv(openFileDialog.FileName);
                    TxtScenarioStatus.Text = $"Durum: {System.IO.Path.GetFileName(openFileDialog.FileName)} Yüklendi";
                    TxtScenarioStatus.Foreground = new SolidColorBrush(Colors.Green);

                    var inv = System.Globalization.CultureInfo.InvariantCulture;
                    var numStyle = System.Globalization.NumberStyles.Float;

                    float.TryParse(TxtMass.Text, numStyle, inv, out float mass);
                    float.TryParse(TxtIxx.Text, numStyle, inv, out float ixx);
                    float.TryParse(TxtIyy.Text, numStyle, inv, out float iyy);
                    float.TryParse(TxtIzz.Text, numStyle, inv, out float izz);
                    float.TryParse(TxtLinDamp.Text, numStyle, inv, out float linDamp);
                    float.TryParse(TxtAngDamp.Text, numStyle, inv, out float angDamp);

                    simManager.StartSimulation(mass, ixx, iyy, izz, linDamp, angDamp);
                    
                    if (isSimRunning == 0) ToggleSim_Click(null, null);
                }
                catch (Exception ex)
                {
                    MessageBox.Show("CSV yüklenirken hata oluştu: " + ex.Message);
                }
            }
        }

        private void ApplyHz_Click(object sender, RoutedEventArgs e)
        {
            if (float.TryParse(HzTextBox.Text, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out float newHz))
            {
                PhysicsEngineAPI.sim_api_update_hz(newHz);
            }
        }

        private void InitOrientation_TextChanged(object sender, System.Windows.Controls.TextChangedEventArgs e)
        {
            if (!isInitialized) return;
            if (TxtInitRoll == null || TxtInitPitch == null || TxtInitYaw == null) return;

            var inv = System.Globalization.CultureInfo.InvariantCulture;
            var numStyle = System.Globalization.NumberStyles.Float;

            float.TryParse(TxtInitRoll.Text, numStyle, inv, out float roll);
            float.TryParse(TxtInitPitch.Text, numStyle, inv, out float pitch);
            float.TryParse(TxtInitYaw.Text, numStyle, inv, out float yaw);

            ((AxisAngleRotation3D)rotateX1.Rotation).Angle = roll;
            ((AxisAngleRotation3D)rotateY1.Rotation).Angle = pitch;
            ((AxisAngleRotation3D)rotateZ1.Rotation).Angle = yaw;

            try { PhysicsEngineAPI.sim_api_set_initial_orientation(roll, pitch, yaw); } catch { }
        }

        private void Noise_TextChanged(object sender, System.Windows.Controls.TextChangedEventArgs e)
        {
            if (!isInitialized) return;
            if (TxtAccelNoise == null || TxtGyroNoise == null) return;

            var inv = System.Globalization.CultureInfo.InvariantCulture;
            var numStyle = System.Globalization.NumberStyles.Float;

            float.TryParse(TxtAccelNoise.Text, numStyle, inv, out float aStdDev);
            float.TryParse(TxtGyroNoise.Text, numStyle, inv, out float gStdDev);

            PhysicsEngineAPI.sim_api_update_noise(Math.Clamp(aStdDev, 0.0f, 5.0f), Math.Clamp(gStdDev, 0.0f, 5.0f));
        }

        private void Bias_TextChanged(object sender, System.Windows.Controls.TextChangedEventArgs e)
        {
            if (!isInitialized) return;
            if (TxtAccelBiasX == null || TxtGyroBiasX == null) return;

            var inv = System.Globalization.CultureInfo.InvariantCulture;
            var numStyle = System.Globalization.NumberStyles.Float;

            float.TryParse(TxtAccelBiasX.Text, numStyle, inv, out float abX);
            float.TryParse(TxtAccelBiasY.Text, numStyle, inv, out float abY);
            float.TryParse(TxtAccelBiasZ.Text, numStyle, inv, out float abZ);
            float.TryParse(TxtGyroBiasX.Text, numStyle, inv, out float gbX);
            float.TryParse(TxtGyroBiasY.Text, numStyle, inv, out float gbY);
            float.TryParse(TxtGyroBiasZ.Text, numStyle, inv, out float gbZ);

            PhysicsEngineAPI.sim_api_update_bias(abX, abY, abZ, gbX, gbY, gbZ);
        }

        private void UiTimer_Tick(object? sender, EventArgs e)
        {
            float elapsedSeconds = (float)stopwatch.Elapsed.TotalSeconds;
            stopwatch.Restart();

            try 
            {
                simManager.UpdateStep(elapsedSeconds, out float ax, out float ay, out float az, 
                                                      out float gx, out float gy, out float gz);
                
                TxtAccel.Text = $"İvme (X, Y, Z): {ax:F2}, {ay:F2}, {az:F2}";
                TxtGyro.Text  = $"Gyro (X, Y, Z): {gx:F2}, {gy:F2}, {gz:F2}";

                ((AxisAngleRotation3D)rotateX1.Rotation).Angle += gx * elapsedSeconds * 50;
                ((AxisAngleRotation3D)rotateY1.Rotation).Angle += gy * elapsedSeconds * 50;
                ((AxisAngleRotation3D)rotateZ1.Rotation).Angle += gz * elapsedSeconds * 50;

                Vector3D refDir = new Vector3D(0, 0, -1); 
                Vector3D accDir = new Vector3D(ax, ay, az);

                if (accDir.LengthSquared > 0.0001)
                {
                    accDir.Normalize();
                    Vector3D rotAxis = Vector3D.CrossProduct(refDir, accDir);
                    double dot = Vector3D.DotProduct(refDir, accDir);
                    dot = Math.Clamp(dot, -1.0, 1.0);
                    double rotAngle = Math.Acos(dot) * (180.0 / Math.PI);

                    if (rotAxis.LengthSquared < 0.0001)
                    {
                        rotAxis = new Vector3D(1, 0, 0);
                        rotAngle = (dot < 0) ? 180.0 : 0.0;
                    }
                    else
                    {
                        rotAxis.Normalize();
                    }

                    forceAxisAngle.Axis = rotAxis;
                    forceAxisAngle.Angle = rotAngle;
                }
            }
            catch 
            {
                // Döngü sırasında hata olursa yoksay
            }
        }

        protected override void OnClosed(EventArgs e)
        {
            try { PhysicsEngineAPI.sim_close(); } catch { }
            base.OnClosed(e);
        }
    }
}