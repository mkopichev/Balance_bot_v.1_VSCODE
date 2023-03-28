#include "../include/utils.h"

uint8_t G_T2counter = 0;

uint8_t digit(uint16_t d, uint8_t m) {
    uint8_t i = 5, a;
    while(i) {
        a = d % 10;
        if(i-- == m)
            break;
        d /= 10;
    }
    return (a);
}

void t2Delay(uint8_t d) // 1-200 * 2.5ms
{
    G_T2counter = 0;
    while(G_T2counter < d)
        continue;
}