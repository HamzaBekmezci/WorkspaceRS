cmake_minimum_required(VERSION 3.10)

project(SentetikIMU LANGUAGES C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED True)

# Performans optimizasyonu ve uyarılar
add_compile_options(-Wall -Wextra -O2)

# 1. Ortak Dosyalar (Büyük harfli Inc/Src hiyerarşisine uyarlandı)
set(MOTOR_KAYNAKLAR 
    Src/main.c 
    Src/math_engine.c 
    Src/kinematics.c
    Src/sensor_engine.c
)

message(STATUS ">>> SentetikIMU: CSV Çıktı Modu Aktif <<<")

# 2. Yürütülebilir Dosyayı Oluştur[cite: 1]
add_executable(ImuSimulator ${MOTOR_KAYNAKLAR})

# 3. Arayüz (Header) Klasörünü Tanıtmak (Büyük harfli Inc)
target_include_directories(ImuSimulator PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/Inc)

# 4. Matematik Kütüphanesi Bağlantısı (sinf, cosf, atan2f için)
target_link_libraries(ImuSimulator PRIVATE m)