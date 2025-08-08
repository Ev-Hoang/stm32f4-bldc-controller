/*
 * _hall_sensor.c
 *
 *  Created on: Aug 8, 2025
 *      Author: ADMIN
 */

#include "_hall_sensor.h"

int8_t const hallCWLookup[8] = {
    -1, // 0b000 (0): invalid
     0, // 0b001 (1)
     1, // 0b010 (2)
     2, // 0b011 (3)
     3, // 0b100 (4)
     4, // 0b101 (5)
     5, // 0b110 (6)
    -1  // 0b111 (7): invalid
};

uint8_t hallState = 0;
uint8_t currentCommStep = 0;

void handleCommutation(uint8_t step, uint8_t pwmVal) {
    switch (step) {
        case 0:
            SET_PWM_1_H3(pwmVal);
            SET_PWM_1_L2(lowMax);
            SET_PWM_1_H1(highMin);
            SET_PWM_1_H2(highMin);
            SET_PWM_1_L1(lowMin);
            SET_PWM_1_L3(lowMin);
            break;
        case 1:
			SET_PWM_1_H2(pwmVal);
			SET_PWM_1_L1(lowMax);
			SET_PWM_1_H1(highMin);
			SET_PWM_1_H3(highMin);
			SET_PWM_1_L2(lowMin);
			SET_PWM_1_L3(lowMin);
            break;
        case 2:
            SET_PWM_1_H3(pwmVal);
			SET_PWM_1_L1(lowMax);
			SET_PWM_1_H1(highMin);
			SET_PWM_1_H2(highMin);
			SET_PWM_1_L2(lowMin);
			SET_PWM_1_L3(lowMin);
            break;
        case 3:
            SET_PWM_1_H1(pwmVal);
			SET_PWM_1_L3(lowMax);
			SET_PWM_1_H2(highMin);
			SET_PWM_1_H3(highMin);
			SET_PWM_1_L1(lowMin);
			SET_PWM_1_L2(lowMin);
            break;
        case 4:
            SET_PWM_1_H1(pwmVal);
			SET_PWM_1_L2(lowMax);
			SET_PWM_1_H2(highMin);
			SET_PWM_1_H3(highMin);
			SET_PWM_1_L1(lowMin);
			SET_PWM_1_L3(lowMin);
            break;
        case 5:
            SET_PWM_1_H2(pwmVal);
			SET_PWM_1_L3(lowMax);
			SET_PWM_1_H1(highMin);
			SET_PWM_1_H3(highMin);
			SET_PWM_1_L1(lowMin);
			SET_PWM_1_L2(lowMin);
            break;
        default: break;
    }
}

int readHallSensor(void)
{
    uint32_t idr = GPIOA->IDR;
    uint8_t hallA = (idr >> 5) & 0x01;
    uint8_t hallB = (idr >> 6) & 0x01;
    uint8_t hallC = (idr >> 7) & 0x01;

    hallState = (hallA << 2) | (hallB << 1) | hallC;

    int8_t step = hallCWLookup[hallState];
    if (step >= 0) return step;
    return 0;
}


