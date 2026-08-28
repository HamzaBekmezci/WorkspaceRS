#include "zaman_arayuzu.h"

#ifdef SIMULATION
    #include <time.h>
    uint32_t zaman_oku(void) {
        return (uint32_t)(clock() * 1000 / CLOCKS_PER_SEC);
    }
#else
    // #include "stm32f4xx_hal.h"
    uint32_t zaman_oku(void) {
       // return HAL_GetTick();
    }
#endif