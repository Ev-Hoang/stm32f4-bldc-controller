/*
 * _bufferHandler.h
 *
 *  Created on: Aug 6, 2025
 *      Author: Ev Hoang
 */

#ifndef INC__BUFFERHANDLER_H_
#define INC__BUFFERHANDLER_H_

#ifdef __cplusplus
extern "C" {
#endif

//Include
#include "stm32f4xx_hal.h"
//#include "usb_device.h"
//#include "usbd_cdc_if.h"

#define BUFFER_SIZE 2
extern uint8_t arrayBuffer[BUFFER_SIZE];
extern uint8_t bufferHead;
extern uint8_t bufferTail;

extern void bufferAdd(uint8_t buffer);
extern int bufferGet();

#ifdef __cplusplus
}
#endif

#endif /* INC__BUFFERHANDLER_H_ */
