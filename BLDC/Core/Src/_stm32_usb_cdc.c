/*
 * _stm32_usb_cdc.c
 *
 *  Created on: Aug 21, 2025
 *      Author: ADMIN
 */

#include "_stm32_usb_cdc.h"

char usb_rx_buffer[RX_BUF_SIZE];
uint16_t usb_rx_index = 0;
uint8_t uart_line_ready = 0;

