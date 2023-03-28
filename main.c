#include "include/mpu9250.h"
#include "include/twi.h"
#include "include/uart.h"
#include "include/utils.h"

#define VL53_I2C_ADDRESS 0x29
#define LED_ON           PORTD |= (1 << 3)
#define LED_OFF          PORTD &= ~(1 << 3)

ISR(TIMER2_OVF_vect);
void PIDR_control(int16_t ZAD);

uint8_t dir1, dir2;

uint8_t G_vl53T2counter = 0;
uint8_t G_flag_acs_on = 0;
uint8_t G_whileCounter = 0;
uint16_t G_distance = 0;

int main(void) {
    DDRC |= 0x0F;     // motors
    DDRD |= (1 << 3); // blue LED

    // pull-up for i2c
    DDRC &= ~(1 << 4);
    DDRC &= ~(1 << 5);
    PORTC |= (1 << 4);
    PORTC |= (1 << 5);

    // Timer/Counter 1 initialization
    // Clock source: System Clock
    // Clock value: 1382,400 kHz
    // Mode: Fast PWM top=0x03FF
    // OC1A output: Disconnected
    // OC1B output: Disconnected
    // Noise Canceler: Off
    // Input Capture on Falling Edge
    // Timer Period: 0,74074 ms
    // Timer1 Overflow Interrupt: On
    // Input Capture Interrupt: Off
    // Compare A Match Interrupt: On
    // Compare B Match Interrupt: On
    TCCR1A = (1 << WGM11) | (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS11);

    OCR1A = 1023;
    OCR1B = 1023;

    // Timer/Counter 2 initialization
    // Clock source: System Clock
    // Clock value: 43,200 kHz
    // Mode: Normal top=0xFF
    // OC2 output: Disconnected
    // Timer Period: 2,5 ms
    TCCR2 = (1 << CS22) | (1 << CS21);
    TCNT2 = 0x94;
    OCR2 = 0x00;

    // Timer(s)/Counter(s) Interrupt(s) initialization
    TIMSK = (1 << TOIE2) | (1 << OCIE1A) | (1 << OCIE1B) | (1 << TOIE1);

    uartInit();

    twiInit();

    sei();

    t2Delay(40);
    mpuInit();
    t2Delay(10);
    G_flag_acs_on = 1;

    while(1) {
        if(G_whileCounter > 50) {
            // OCR1A = 100;
            uartTransmitDec(G_a_GX);
            uartTransmitStr("\t");
            uartTransmitDec(REAL_X);
            uartTransmitStr("\t");
            uartTransmitDec(G_distance);
            uartTransmitStr("\r\n");
            G_whileCounter = 0;
        }
    }
}

// program PWM on COMPA COMPB and OVF of the 1st timer
ISR(TIMER1_OVF_vect) {
    PORTC &= ~((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3));
    // PORTC .0 = 0;
    // PORTC .1 = 0;
    // PORTC .2 = 0;
    // PORTC .3 = 0;
}

ISR(TIMER1_COMPA_vect) {
    if(dir1 == 1)
        // PORTC .0 = 1;
        PORTC |= (1 << 0);
    else
        // PORTC .1 = 1;
        PORTC |= (1 << 1);
}

ISR(TIMER1_COMPB_vect) {
    if(dir2 == 1)
        // PORTC .3 = 1;
        PORTC |= (1 << 3);
    else
        // PORTC .2 = 1;
        PORTC |= (1 << 2);
}

void vl53TofGetData(void) {
    tx_buffer2[0] = 0;
    tx_buffer2[1] = 1;

    twiMasterTransmit(VL53_I2C_ADDRESS, tx_buffer2, 2);
    tx_buffer2[0] = 0x14;
    twiMasterTransmit(VL53_I2C_ADDRESS, tx_buffer2, 1);
    twiMasterReceive(VL53_I2C_ADDRESS, z + 2, 1);
    // 64=unknown or less than 10mm
    // 94=data ok
    // 78=too far

    tx_buffer2[0] = 30;
    twiMasterTransmit(VL53_I2C_ADDRESS, tx_buffer2, 1);
    twiMasterReceive(VL53_I2C_ADDRESS, z, 2);

    if(z[2] == 94) {
        G_distance = (((uint16_t)z[0]) << 8) + z[1];
        // if(G_distance>70)G_distance=G_distance-70;//for black board Derror=70 for green board Derror=90
        // else G_distance=0;
    } else {
        if(z[2] == 78) // too far
            G_distance = 3000;
    }
}

// main control loop 2.5ms 400Hz
ISR(TIMER2_OVF_vect) {
    // Reinitialize Timer2 value
    TCNT2 = 0x94;
    // Place your code here
    G_T2counter++;
    if(G_T2counter == 0)
        LED_ON;
    else
        LED_OFF;

    if(G_flag_acs_on != 0) {
        mpuGetData();
        mpuAngleCalculation();
        PIDR_control(G_controlX - 24);
    }

    if(G_vl53T2counter <= 10) // reading distance from vl53l0x at 40 Hz
    {
        G_vl53T2counter++;
    } else {
        G_vl53T2counter = 0;
        vl53TofGetData();
        G_controlX = (200 - (int)G_distance) / 4;
        if(G_controlX > 50)
            G_controlX = 50;
        if(G_controlX < -50)
            G_controlX = -50;
    }

    G_whileCounter++; // decreasing frequency of main function
}

void PIDR_control(int16_t ZAD) {
    int16_t control_X = 0;
    //    G_iPid = G_iPid + G_angle_speedX;
    G_iPid = G_iPid + (REAL_X - ZAD);
    if(G_iPid >= 5000)
        G_iPid = 5000;
    if(G_iPid <= -5000)
        G_iPid = -5000;
    G_iSpeed = G_iSpeed + control_X;
    if(G_iSpeed >= 5000)
        G_iSpeed = 5000;
    if(G_iSpeed <= -5000)
        G_iSpeed = -5000;
    control_X = (10 * (REAL_X - ZAD)) - (4 * G_angle_speedX) + (G_iPid / 11); //- G_iSpeed/20;
    if(control_X > 674)
        control_X = 674;
    if(control_X < -674)
        control_X = -674;

    if((REAL_X >= -20 * 40) && (REAL_X <= 20 * 40)) {
        if(control_X > 0) {
            dir1 = 0;
            dir2 = 0;
            OCR1A = 674 - control_X;
            OCR1B = 704 - control_X;
        } else {
            dir1 = 1;
            dir2 = 1;
            OCR1A = 674 + control_X;
            OCR1B = 704 + control_X;
        }
    } else {
        OCR1A = 1023;
        OCR1B = 1023;
    }
}