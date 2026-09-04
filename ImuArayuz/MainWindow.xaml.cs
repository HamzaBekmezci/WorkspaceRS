using System;
using System.Diagnostics;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf; // 3D model yükleyici için Helix Toolkit

namespace ImuArayuz
{
    public partial class MainWindow : Window
    {
        // Senaryo Yönetimi ve Zamanlayıcı
        private ScenarioManager simManager = new ScenarioManager();
        private System.Windows.Threading.DispatcherTimer uiTimer = new System.Windows.Threading.DispatcherTimer();
        private Stopwatch stopwatch = new Stopwatch();
        private int isSimRunning = 0;
        private bool isInitialized = false;

        // Füzenin 6-DOF (Dönme + Konum) Transformasyon Referansları
        private RotateTransform3D rocketRotateX = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(1, 0, 0), 0));
        private RotateTransform3D rocketRotateY = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 1, 0), 0));
        private RotateTransform3D rocketRotateZ = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 0, 1), 0));
        private TranslateTransform3D rocketTranslate = new TranslateTransform3D(0, 0, 0);
        
        // Yörünge İzi (Kuyruk) Noktaları Koleksiyonu
        private Point3DCollection trailPoints = new Point3DCollection();

        // Konum hesaplaması için geçici hız/konum tutucuları
        private double _velX = 0, _velY = 0, _velZ = 0;
        private double _posX = 0, _posY = 0, _posZ = 0;
        private double lastPosX = 0, lastPosY = 0, lastPosZ = 0;

        public MainWindow()
        {
            InitializeComponent();

            // 1. AIM-9 Roket Modelini Yükle
            LoadRocketModel();

            // 2. DLL (Fizik Motoru) Başlatma
            try
            {
                PhysicsEngineAPI.sim_init();
                // Başlangıç için varsayılan kütle ve atalet değerleri
                PhysicsEngineAPI.sim_api_set_body_params(1.0f, 0.1f, 0.1f, 0.05f, 0.2f, 0.1f);
            }
            catch (Exception ex)
            {
                MessageBox.Show($"C Motoru DLL hatası:\n\n{ex.Message}", "Kritik Hata", MessageBoxButton.OK, MessageBoxImage.Error);
                return; 
            }

            isInitialized = true;
            stopwatch.Start();

            // UI Zamanlayıcısını Başlat (~60 Hz / FPS)
            uiTimer.Interval = TimeSpan.FromMilliseconds(16);
            uiTimer.Tick += UiTimer_Tick;
            uiTimer.Start();
        }

        /// <summary>
        /// .OBJ dosyasını okuyup Helix Viewport üzerindeki Visual3D elementine bağlar
        /// </summary>
      private void LoadRocketModel()
        {
            try
            {
                ModelImporter importer = new ModelImporter();
                Model3DGroup? model = importer.Load("Models/AIM-9 SIDEWINDER.obj"); 

                if (model != null)
                {
                    Transform3DGroup transformGroup = new Transform3DGroup();

                    // 1. ÖLÇEKLENDİRME
                    ScaleTransform3D scaleDown = new ScaleTransform3D(0.01, 0.01, 0.01);
                    transformGroup.Children.Add(scaleDown);

                    // 2. EKSEN HİZALAMASI (ÖN DÖNÜŞ)
                    RotateTransform3D initialAlignment = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 1, 0), -90)); 
                    transformGroup.Children.Add(initialAlignment);

                    // --- AĞIRLIK MERKEZİ (CoG) DÜZELTMESİ ---
                    // Modelin fiziksel ağırlık merkezini dönüş eksenine (0,0,0) çekiyoruz.
                    // NOT: Buradaki X, Y, Z değerlerini modelinizin görselindeki sapmaya göre 
                    // deneme yanılma ile (örneğin -2.5, 0, 0 gibi) ayarlamanız gerekecektir.
                    TranslateTransform3D cogOffset = new TranslateTransform3D(-0.5, -0.8, 0.0); 
                    transformGroup.Children.Add(cogOffset);
                    // ----------------------------------------

                    // 3. FİZİK MOTORU DÖNÜŞLERİ (Mutlaka CoG ötelemesinden SONRA gelmeli)
                    transformGroup.Children.Add(rocketRotateX); // Roll
                    transformGroup.Children.Add(rocketRotateY); // Pitch
                    transformGroup.Children.Add(rocketRotateZ); // Yaw

                    // 4. UZAYDAKİ KONUM (Öteleme)
                    transformGroup.Children.Add(rocketTranslate);
                    
                    model.Transform = transformGroup;
                    RocketVisual.Content = model; 
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Füze modeli yüklenemedi.\n\nDetay: {ex.Message}");
            }
        }
       // MainWindow.xaml.cs içerisinde mevcut UiTimer_Tick fonksiyonunu bununla değiştirin[cite: 8]
        private void UiTimer_Tick(object? sender, EventArgs e)
        {
            float dt = (float)stopwatch.Elapsed.TotalSeconds;
            stopwatch.Restart();

            try 
            {
                // 1. Fizik motorundan adımı işlet (Sensör verileri VE Gerçek Konum değerlerini al)
                // NOT: C# tarafındaki DllImport sim_step_auto fonksiyonu imzasını ve 
                // ScenarioManager.UpdateStep fonksiyonunu bu 3 yeni argümanı alacak şekilde güncellemeyi unutmayın.
                simManager.UpdateStep(dt, out float ax, out float ay, out float az, 
                                        out float gx, out float gy, out float gz,
                                        out float posX, out float posY, out float posZ);
                
                // 2. Arayüzdeki yazı (Label) güncellemeleri[cite: 8]
                TxtAccel.Text = $"İvme (X,Y,Z): {ax:F2}, {ay:F2}, {az:F2} m/s²";
                TxtGyro.Text  = $"Gyro (X,Y,Z): {gx:F2}, {gy:F2}, {gz:F2} rad/s";
                TxtPosition.Text = $"Konum (X,Y,Z): {posX:F2}, {posY:F2}, {posZ:F2} m";

                // 3. DÖNME (Rotation): Gyro (rad/s) verisini dereceye çevirip füzeyi döndürüyoruz
                // NOT: X=Roll, Y=Pitch, Z=Yaw'dır. Görsel uyum için Pitch (gy) değerini çıkarıyoruz (-).
                ((AxisAngleRotation3D)rocketRotateX.Rotation).Angle += gx * dt * (180.0 / Math.PI); // Roll (X ekseni)
                ((AxisAngleRotation3D)rocketRotateY.Rotation).Angle -= gy * dt * (180.0 / Math.PI); // Pitch (Y ekseni) -> EKSİ YAPILDI
                ((AxisAngleRotation3D)rocketRotateZ.Rotation).Angle += gz * dt * (180.0 / Math.PI); // Yaw (Z ekseni)

                // 4. ÖTELEME (Translation): Doğrudan motorun verdiği gerçek konumu kullanıyoruz
                // Görselin ekrandan taşmaması için uzaydaki pozisyonu 1/10 oranında ölçeklendiriyoruz
                double scaledPosX = posX / 10.0;
                double scaledPosY = posY / 10.0;
                double scaledPosZ = posZ / 10.0;

                rocketTranslate.OffsetX = scaledPosX;
                rocketTranslate.OffsetY = scaledPosY;
                rocketTranslate.OffsetZ = scaledPosZ;

                // 5. KAMERA TAKİBİ (CHASE CAMERA)[cite: 8]
                // Füzenin bu frame'de uzayda ne kadar yer değiştirdiğini (delta) bul
                double dx = scaledPosX - lastPosX;
                double dy = scaledPosY - lastPosY;
                double dz = scaledPosZ - lastPosZ;

                // Kamerayı doğrudan ismiyle (FollowCamera) güncelliyoruz[cite: 8]
                if (FollowCamera != null)
                {
                    FollowCamera.Position = new Point3D(FollowCamera.Position.X + dx, 
                                                        FollowCamera.Position.Y + dy, 
                                                        FollowCamera.Position.Z + dz);
                }

                // Bir sonraki adımın delta hesabı için şu anki ölçeklenmiş konumu kaydet[cite: 8]
                lastPosX = scaledPosX;
                lastPosY = scaledPosY;
                lastPosZ = scaledPosZ;

                // 6. İZ ÇİZİMİ (Trajectory Trail): Uzayda füzenin arkasından kırmızı bir çizgi bırak[cite: 8]
                // Her karede değil, füze yarım metre (0.5) ilerlediğinde bir nokta koyuyoruz (Performans için)
                if (trailPoints.Count == 0 || 
                    Math.Abs(trailPoints[^1].X - scaledPosX) > 0.5 || 
                    Math.Abs(trailPoints[^1].Y - scaledPosY) > 0.5 || 
                    Math.Abs(trailPoints[^1].Z - scaledPosZ) > 0.5)
                {
                    trailPoints.Add(new Point3D(scaledPosX, scaledPosY, scaledPosZ));
                    
                    // İz çok uzayıp bilgisayarı yormasın diye sadece son 200 noktayı tutuyoruz[cite: 8]
                    if (trailPoints.Count > 200) 
                    {
                        trailPoints.RemoveAt(0);
                    }
                    
                    // Çizgiyi arayüze (HelixToolkit) yansıt[cite: 8]
                    TrajectoryTrail.Points = trailPoints;
                }
            }
            catch 
            {
                // Render çakışması veya Null hatası durumunda uygulamanın çökmesini engeller[cite: 8]
            }
        }

        /// <summary>
        /// İvmeyi entegre ederek basit doğrusal yörünge simülasyonu yapar
        /// </summary>
        private void SimulatePositionLocally(float ax, float ay, float az, float dt, out double pX, out double pY, out double pZ)
        {
            // Euler entegrasyonu (İvme -> Hız -> Konum)
            _velX += ax * dt;
            _velY += ay * dt;
            _velZ += az * dt;

            _posX += _velX * dt;
            _posY += _velY * dt;
            _posZ += _velZ * dt;

            // --- ZEMİN ÇARPIŞMA (GROUND COLLISION) KONTROLÜ ---
            // Eğer Z ekseni (Yükseklik) 0'ın (yerin) altına düşerse:
            if (_posZ < 0)
            {
                _posZ = 0;      // Modelin yerin altına girmesini engelle
                _velZ = 0;      // Düşüş hızını sıfırla (yerde sekme olmasın)
                
                // İsteğe bağlı: Füze yere çarptığında durmasını isterseniz yatay hızları da sıfırlayabilirsiniz:
                _velX = 0; 
                _velY = 0;
            }
            // ---------------------------------------------------

            // Görsel ölçeklendirme (Uzayda kameradan çıkmasın diye pozisyonu ölçeklendiriyoruz, /10 idealdir)
            pX = _posX / 10.0;
            pY = _posY / 10.0;
            pZ = _posZ / 10.0;
        }
        private void LoadScenario_Click(object sender, RoutedEventArgs e)
        {
            Microsoft.Win32.OpenFileDialog openFileDialog = new Microsoft.Win32.OpenFileDialog();
            openFileDialog.Filter = "CSV Dosyaları (*.csv)|*.csv|Tüm Dosyalar (*.*)|*.*";
            
            if (openFileDialog.ShowDialog() == true)
            {
                try
                {
                    // Senaryo dosyasını yükle
                    simManager.LoadCsv(openFileDialog.FileName);
                    TxtScenarioStatus.Text = $"Durum: {System.IO.Path.GetFileName(openFileDialog.FileName)} Yüklendi";
                    TxtScenarioStatus.Foreground = new SolidColorBrush(Colors.Green);

                    // Arayüzden güncel fizik parametrelerini (Kütle, Sürtünme vb.) al
                    var inv = System.Globalization.CultureInfo.InvariantCulture;
                    var numStyle = System.Globalization.NumberStyles.Float;

                    float.TryParse(TxtMass.Text, numStyle, inv, out float mass);
                    float.TryParse(TxtIxx.Text, numStyle, inv, out float ixx);
                    float.TryParse(TxtIyy.Text, numStyle, inv, out float iyy);
                    float.TryParse(TxtIzz.Text, numStyle, inv, out float izz);
                    float.TryParse(TxtLinDamp.Text, numStyle, inv, out float linDamp);
                    float.TryParse(TxtAngDamp.Text, numStyle, inv, out float angDamp);

                    // Değerler sıfır veya mantıksız girildiyse çökmemesi için ufak koruma
                    if (mass <= 0) mass = 1.0f;
                    if (ixx <= 0) ixx = 0.01f;

                    // Sistemi sıfırla ve başlat
                    _velX = _velY = _velZ = 0;
                    _posX = _posY = _posZ = 0;
                    trailPoints.Clear();

                    simManager.StartSimulation(mass, ixx, iyy, izz, linDamp, angDamp);
                    
                    // Simülasyon kapalıysa otomatik başlat
                    if (isSimRunning == 0) ToggleSim_Click(null, null);
                }
                catch (Exception ex)
                {
                    MessageBox.Show("CSV yüklenirken hata oluştu: " + ex.Message);
                }
            }
        }

        // --- Diğer Arayüz Kontrol Fonksiyonları ---

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

            // WPF'in Y ekseni fizik motoruna göre terstir, bu yüzden görsel pitch'i eksi (-) yapıyoruz
            ((AxisAngleRotation3D)rocketRotateX.Rotation).Angle = roll;
            ((AxisAngleRotation3D)rocketRotateY.Rotation).Angle = -pitch; // EKSİ YAPILDI
            ((AxisAngleRotation3D)rocketRotateZ.Rotation).Angle = yaw;

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

        protected override void OnClosed(EventArgs e)
        {
            // Program kapatılırken C DLL'i bellekten temizle
            try { PhysicsEngineAPI.sim_close(); } catch { }
            base.OnClosed(e);
        }
    }
}