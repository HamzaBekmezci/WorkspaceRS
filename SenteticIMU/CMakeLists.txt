cmake_minimum_required(VERSION 3.10)

project(SentetikIMU LANGUAGES C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED True)

# Performans optimizasyonu ve uyarılar
add_compile_options(-Wall -Wextra -O2)

# 1. Ortak Kaynak Dosyaları
set(MOTOR_KAYNAKLAR 
    Src/main.c 
    Src/math_engine.c 
    Src/kinematics.c
    Src/sensor_engine.c
    Src/logger.c
    Src/sim_set.c
)

message(STATUS ">>> SentetikIMU: DLL Modu Aktif <<<")

# 2. Dinamik Kütüphaneyi (DLL) Oluştur
# Burada manuel dosya isimleri yerine yukarıda tanımladığın listeyi kullanıyoruz.
add_library(ImuSimulator SHARED ${MOTOR_KAYNAKLAR})

# 3. DLL Çıktı Klasörünü Sabitleme
set_target_properties(ImuSimulator PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/output"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/output"
)

# 4. Arayüz (Header) Klasörünü Tanıtmak
target_include_directories(ImuSimulator PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/Inc)

# 5. Matematik Kütüphanesi Bağlantısı
target_link_libraries(ImuSimulator PRIVATE m)