/*
 * _hall_sensor.h
 *
 *  Created on: Aug 8, 2025
 *      Author: Ev
 */

#ifndef INC__HALL_SENSOR_H_
#define INC__HALL_SENSOR_H_

#ifdef __cplusplus
extern "C" {
#endif

//Include
#include "stm32f4xx_hal.h"

#define lowMax 0x00
#define lowMin 0xFF
#define highMax 0xFF
#define highMin 0x00

// Changing the duty of PWM
#define SET_PWM_1_H1(val)  (TIM2->CCR1 = (val))
#define SET_PWM_1_H2(val)  (TIM2->CCR2 = (val))
#define SET_PWM_1_H3(val)  (TIM2->CCR3 = (val))
#define SET_PWM_1_L1(val)  (TIM2->CCR4 = (val))

#define SET_PWM_1_L2(val)  (TIM3->CCR1 = (val))
#define SET_PWM_1_L3(val)  (TIM3->CCR2 = (val))
#define SET_PWM_2_H1(val)  (TIM3->CCR3 = (val))
#define SET_PWM_2_H2(val)  (TIM3->CCR4 = (val))

#define SET_PWM_2_H3(val)  (TIM2->CCR1 = (val))
#define SET_PWM_2_L1(val)  (TIM2->CCR2 = (val))
#define SET_PWM_2_L2(val)  (TIM2->CCR3 = (val))
#define SET_PWM_2_L3(val)  (TIM2->CCR4 = (val))

extern const int8_t hallCWLookup[8];

extern uint8_t hallState;
extern uint8_t currentCommStep;

extern void handleCommutation(uint8_t step, uint8_t pwmVal);
extern int readHallSensor(void);

#ifdef __cplusplus
}
#endif

#endif /* INC__HALL_SENSOR_H_ */
