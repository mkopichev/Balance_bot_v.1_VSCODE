#include "../include/twi.h"

void twiInit(void) {
    TWBR = 19; // 200 kHz
}

void twiStart(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while(!(TWCR & (1 << TWINT)))
        continue;
}

void twiStop(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

void twiTransmitByte(uint8_t byte) {
    TWDR = byte;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while(!(TWCR & (1 << TWINT)))
        continue;
}

uint8_t twiReceiveByte(uint8_t is_last_byte) {
    if(is_last_byte)
        TWCR = (1 << TWINT) | (1 << TWEN);
    else
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while(!(TWCR & (1 << TWINT)))
        continue;
    return (TWDR);
}

bool twiMasterTransmit(uint8_t slave_addr, uint8_t *tx_data_buf, uint8_t tx_number_of_bytes) {
    twiStart();
    twiTransmitByte((slave_addr << 1) | W);
    for(uint8_t i = 0; i < tx_number_of_bytes; i++) twiTransmitByte(tx_data_buf[i]);
    twiStop();
    return true;
}

bool twiMasterReceive(uint8_t slave_addr, uint8_t *rx_data_buf, uint8_t rx_number_of_bytes) {
    twiStart();
    twiTransmitByte((slave_addr << 1) | R);
    for(uint8_t i = 0; i < rx_number_of_bytes; i++) twiReceiveByte(rx_data_buf[i]);
    twiStop();
    return true;
}