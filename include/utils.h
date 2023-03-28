#ifndef UTILS_H
#define UTILS_H

#include <avr/interrupt.h>
#include <avr/io.h>
#include <math.h>
#include <stdint.h>
#include <util/twi.h>

extern uint8_t G_T2counter;

uint8_t digit(uint16_t d, uint8_t m);
void t2Delay(uint8_t d);

#endif