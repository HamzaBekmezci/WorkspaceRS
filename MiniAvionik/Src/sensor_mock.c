#include "sensor_arayuzu.h"
#include <stdio.h>

static FILE *csv_dosyasi = NULL;
static float son_okunan_deger = 0.0f;

static float sahte_veri[] = {10.5, 50.2, 105.0, 90.5};
static int i = 0;

void sensor_baslat(void) {
    csv_dosyasi = fopen("UcusVerisi.csv", "r");
    
    if (csv_dosyasi == NULL) {
        printf("[HATA] ucus_verisi.csv bulunamadi! Lutfen exe'nin calistigi dizine ekleyin.\n");
    } else {
        printf("[MOCK] Ucus verisi (CSV) basariyla baglandi.\n");
    }
}

float irtifa_oku(void) {
    if (csv_dosyasi != NULL) {
        if (fscanf(csv_dosyasi, "%f", &son_okunan_deger) != 1) {
            
            rewind(csv_dosyasi);
         
            fscanf(csv_dosyasi, "%f", &son_okunan_deger);
            printf("\n[MOCK] --- Similasyon Basa Sarildi ---\n\n");
        }
    }
    return son_okunan_deger;
}