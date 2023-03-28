#ifndef UART_H
#define UART_H

#include "utils.h"
#include <avr/io.h>
#include <stdint.h>

void uartInit(void);
void uartTransmitStr(char *string);
void uartTransmitDec(int16_t val);
void uartTransmitBin(uint8_t val);
void uartTransmitHex(uint16_t val);

#endif