#include "../include/mpu9250.h"

uint8_t tx_buffer2[2];
uint8_t z[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // raw data mpu9250 (also used for random numbers)
int16_t ACCEL[3] = {0, 0, 0};                               // raw accel data
int16_t GYRO[3] = {0, 0, 0};                                // raw gyro data
int16_t G_a_GX = 0, G_a_GY = 0, G_a_GZ = 0;
int16_t REAL_X;

void mpuSendCmd(uint8_t register_addr, uint8_t register_value) {
    tx_buffer2[0] = register_addr;
    tx_buffer2[1] = register_value;
    twiMasterTransmit(MPU_I2C_ADDRESS, tx_buffer2, 2);
}

void mpuInit(void) {
    mpuSendCmd(107, 0b00000000); // normal power mode
    t2Delay(10);
    mpuSendCmd(108, 0b00000000); // POWER CMD2
    mpuSendCmd(26, 0b00000000);  // DLPF 260hz(max bandwidth)
    // mpuSendCmd(26,0b00000011);//DLPF 41hz bandwidth
    mpuSendCmd(27, 0b00011000); // gyro config 2000
    mpuSendCmd(28, 0b00011000); //// Accel Config1 //16g
    mpuSendCmd(29, 0b00000100); //// Accel Config2 //20 hz bandwidth
}

void mpuGetData(void) {
    tx_buffer2[0] = 59;
    twiMasterTransmit(MPU_I2C_ADDRESS, tx_buffer2, 1);
    twiMasterReceive(MPU_I2C_ADDRESS, z, 14);
    // CALIBR----------------------------------------------------------------------
    ACCEL[0] = (((int)z[0] << 8) | ((int)z[1])) - 20;  // x    range10g   +-205  =  +-1g
    ACCEL[1] = (((int)z[2] << 8) | ((int)z[3])) - 10;  // y
    ACCEL[2] = (((int)z[4] << 8) | ((int)z[5])) - 490; // z
    // TEMPERATURE=(((((int)z[6]<<8) | ((int)z[7])))/34+365)/10;
    // for range 2000
    GYRO[0] = (((uint16_t)z[8] << 8) | ((uint16_t)z[9])) + 35;
    GYRO[1] = (((uint16_t)z[10] << 8) | ((uint16_t)z[11])) - 6;
    GYRO[2] = (((uint16_t)z[12] << 8) | ((uint16_t)z[13])) + 24;
}

void mpuAngleCalculation(void) {
    float fX;
    int16_t ErrorGX;
    int16_t Acc_X;

    G_avs_AX = G_avs_AX - G_a_AX + ACCEL[0];

    G_angle_speedX = ((long)GYRO[1] * 100) / 1638;  // degree per second
    G_Gyr_angle_X = G_Gyr_angle_X - G_angle_speedX; // degree*400

    G_avs_GX = G_avs_GX - G_a_GX + G_Gyr_angle_X / 10;

    G_a_AX = G_avs_AX / AVERAGE_NUMBER;

    G_a_GX = G_avs_GX / AVERAGE_NUMBER;

    Acc_X = G_a_AX;
    if(Acc_X > 2050)
        Acc_X = 2050;
    if(Acc_X < -2050)
        Acc_X = -2050;

    fX = Acc_X;
    fX = fX / 2050;
    fX = asin(fX);
    G_Acc_angle_X = (int)(fX * 180 * 40 / M_PI); // in degrees *40

    ErrorGX = G_Acc_angle_X - G_a_GX;

    G_Gyr_angle_X = G_Gyr_angle_X + ErrorGX * 10;
    G_avs_GX = G_avs_GX + (long int)ErrorGX * AVERAGE_NUMBER;

    REAL_X = (G_Gyr_angle_X / 10); // in degrees *40
    //    REAL_X=(G_Gyr_angle_X/10) + 127;//in degrees *40
    //    REAL_X=(G_Gyr_angle_X/10) + 167;//in degrees *40
    //    REAL_X=(G_Gyr_angle_X/10) + 90;//in degrees *40
}