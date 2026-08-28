project(MiniAvionik)

# 1. Ortam Değişkeni (Terminalden değiştirebilmek için)
set(HEDEF_ORTAM "SIL_TEST" CACHE STRING "Derleme hedefi: DONANIM veya SIL_TEST")

# 2. Ortak Dosyalar (İş mantığı)
set(ORTAK_KAYNAKLAR 
    src/main.c 
    src/ucus_alg.c 
    src/zaman_yonetimi.c
)

# 3. Ortama Göre Adaptör Seçimi
if(${HEDEF_ORTAM} STREQUAL "SIL_TEST")
    message(STATUS ">>> MiniAvionik: SIL Modu (Bilgisayar Testi) Aktif <<<")
    
    # Bilgisayar testine özel sahte sensör
    set(ADAPTOR_KAYNAKLAR src/sensor_mock.c src/eyleyici_mock.c)
    
    # C kodundaki #ifdef SIMULATION makrosunu tetikler
    add_compile_definitions(SIMULATION)
    
else()
    message(STATUS ">>> MiniAvionik: DONANIM Modu (STM32) Aktif <<<")
    
    # Gerçek donanım sensörü
    set(ADAPTOR_KAYNAKLAR src/sensor_stm32.c)
endif()

# 4. Yürütülebilir Dosyayı (Exe/Hex) Oluştur
add_executable(UcusYazilimi ${ORTAK_KAYNAKLAR} ${ADAPTOR_KAYNAKLAR})

# 5. EN KRİTİK NOKTA: Arayüz (Header) Klasörünü Tanıtmak
# Bu sayede C kodlarında #include "sensor_arayuzu.h" yazıldığında derleyici dosyayı bulur
target_include_directories(UcusYazilimi PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/Inc)    