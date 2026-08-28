#include "sensor_arayuzu.h"
#include "zaman_arayuzu.h"
#include "eyleyici_arayuzu.h"
#include "stdio.h"
#include "stdint.h"


static float en_yuksek_irtifa = 0.0f;
static float onceki_irtifa = 0.0f;
static int parasut_acildi_mi = 0; // 0: Kapalı, 1: Açık
static uint32_t son_calisma_zamani = 0; // Son çalıştığı milisaniyeyi tutar

void ucus_gorevi(void) {
    uint32_t simdiki_zaman = zaman_oku();
 
    if ((simdiki_zaman - son_calisma_zamani) < 500) {
        return;
    }
    son_calisma_zamani = simdiki_zaman;

    float anlik_irtifa = irtifa_oku();

    // 1. SİMÜLASYON BAŞA SARMA KONTROLÜ
    // Eğer anlık irtifa ani bir şekilde düşerse (başa sarma durumu)
    if (parasut_acildi_mi == 1 && anlik_irtifa > onceki_irtifa && onceki_irtifa < 10.0f) {
        en_yuksek_irtifa = 0.0f;
        parasut_acildi_mi = 0;
        onceki_irtifa = 0.0f; // <--- KRİTİK DÜZELTME: Onceki irtifayı da sıfırlıyoruz!
        printf("\n[ALGORITMA] Similasyon basa sardigi icin ucusa sifirdan baslaniyor...\n\n");
        return; // Bu turu burada kesip bir sonraki döngüye temiz girmesini sağlayalım
    }

    // 2. TEPE NOKTASINI GÜNCELLE
    if (anlik_irtifa > en_yuksek_irtifa) {
        en_yuksek_irtifa = anlik_irtifa;
    }

    // 3. DÜŞÜŞ TESPİTİ VE PARAŞÜT TETİKLEME
    if ((en_yuksek_irtifa - anlik_irtifa > 10.0f) && (parasut_acildi_mi == 0)) {
        parasut_ac();
        parasut_acildi_mi = 1; 
    }

    // 4. ŞU ANKİ İRTİFAYI BİR SONRAKİ TUR İÇİN KAYDET
    onceki_irtifa = anlik_irtifa;

    // 5. TELEMETRİ ÇIKTISI
    printf("[Telemetri] Zaman: %u ms | Irtifa: %.1f m | Tepe: %.1f m | Durum: %s\n",
             simdiki_zaman,
             anlik_irtifa, 
             en_yuksek_irtifa,
             parasut_acildi_mi ? "INISTE" : "TIRMANISTA");
}