#include "sensor_arayuzu.h"

void ucus_gorevi(void);

int main(void) {
    sensor_baslat();
    
    while(1) {
        ucus_gorevi();
    }
    
    return 0;
}