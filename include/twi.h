#ifndef TWI_H
#define TWI_H

#include <avr/io.h>
#include <stdbool.h>
#include <stdint.h>

#define W 0
#define R 1

void twiInit(void);
bool twiMasterTransmit(uint8_t slave_addr, uint8_t *tx_data_buf, uint8_t tx_number_of_bytes);
bool twiMasterReceive(uint8_t slave_addr, uint8_t *rx_data_buf, uint8_t rx_number_of_bytes);

#endif