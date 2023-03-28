#include "../include/uart.h"

void uartInit(void) {
    // uart transmitter, 9600, 8 bits, 1 stop, no parity
    UCSRB = (1 << TXEN);
    UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
    UBRRH = 0x00;
    UBRRL = 0x47; // 9600    11.0592Mhz
    uartTransmitStr("uart_on\r\n");
}

// transmit byte of data
void uartTransmitByte(uint8_t byte) {
    // waiting for UDRE0 set, then put data into UDR0
    while(!(UCSRA & (1 << UDRE)))
        continue;
    UDR = byte;
}

// transmit string, phrase or word
void uartTransmitStr(char *string) {
    /* transmit data byte-by-byte with pointer *
    pointer – address of the first element of data package*/
    while(*string)
        uartTransmitByte(*string++);
}

// transmit decimal value from mk to terminal
void uartTransmitDec(int16_t val) {
    uint8_t i, j;
    if(val < 0) {
        uartTransmitStr("-");
        val = -val;
    }
    // if 5-digit value is nonNull
    if(val != 0) {
        /* check the number of zero digits in 5-digit value
        for instance value 64 has 3 insignificant zeroes - 00064 */
        j = 1;
        while(!(digit(val, j++)))
            continue;
        // eliminating insignificant zeroes
        for(i = j - 1; i <= 5; i++) uartTransmitByte(digit(val, i) + '0');
    }
    // if value is zero just return zero
    else
        uartTransmitByte('0');
}

// transmit binary value from mk to terminal
void uartTransmitBin(uint8_t val) {
    uartTransmitByte('b');
    if(val != 0) {
        for(uint8_t i = (1 << 7); i >= 1; i >>= 1) {
            if(val & i)
                uartTransmitDec(1);
            else
                uartTransmitDec(0);
        }
    }
    // if value is zero just return zero
    else
        uartTransmitDec(00000000);
}

// transmit hex value from mk to terminal
void uartTransmitHex(uint16_t val) {
    uint8_t msb = (val & 0xF0) >> 4, lsb = val & 0x0F;
    msb += msb > 9 ? 'A' - 10 : '0';
    lsb += lsb > 9 ? 'A' - 10 : '0';
    uartTransmitByte(msb);
    uartTransmitByte(lsb);
}