/*
 * _bufferHandler.h
 *
 *  Created on: Aug 21, 2025
 *      Author: Ev Hoang
 */
#ifndef INC__STM32_USB_CDC_H_
#define INC__STM32_USB_CDC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define RX_BUF_SIZE 256

extern char usb_rx_buffer[RX_BUF_SIZE];
extern uint16_t usb_rx_index;
extern uint8_t line_ready;


#ifdef __cplusplus
}
#endif

#endif /* INC__STM32_USB_CDC_H_ */
