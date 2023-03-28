#ifndef MPU9250_H
#define MPU9250_H

#include "twi.h"
#include "utils.h"
#include <math.h>
#include <stdint.h>

#define MPU_I2C_ADDRESS 0b1101001
#define AVERAGE_NUMBER  1000
#define WHO_AM_I        0x75

extern uint8_t tx_buffer2[2];
extern uint8_t z[14];    // raw data mpu9250 (also used for random numbers)
extern int16_t ACCEL[3]; // raw accel data
extern int16_t GYRO[3];  // raw gyro data

// Angle calculation var---------

long int G_avs_AX = 0, G_avs_AY = 0, G_avs_AZ = 0; // average sum
long int G_avs_GX = 0, G_avs_GY = 0, G_avs_GZ = 0; // average sum

int16_t G_a_AX = 0, G_a_AY = 0, G_a_AZ = 0; // average
extern int16_t G_a_GX, G_a_GY, G_a_GZ; // average

int16_t G_Acc_angle_X;
int16_t G_Gyr_angle_X = 0;
extern int16_t REAL_X;
int16_t G_controlX = 0;
int16_t G_iPid = 0;
int16_t G_iSpeed = 0;
int16_t G_angle_speedX = 0;

// Angle calculation var---------

void mpuSendCmd(uint8_t register_addr, uint8_t register_value);
void mpuInit(void);
void mpuGetData(void);
void mpuAngleCalculation(void);

#endif