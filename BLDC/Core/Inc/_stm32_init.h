#ifndef _STM32_INIT_H
#define _STM32_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

//Include
#include "stm32f4xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

//Function declaration
extern void SystemClock_Config(void);
extern void GPIO_Init(void);
extern void TIM2_Init(void);
extern void TIM3_Init(void);
extern void TIM4_Init(void);
extern void USART1_UART_Init(void);
extern void STM32_Init(void);

#ifdef __cplusplus
}
#endif

#endif //_stm32_init.h
