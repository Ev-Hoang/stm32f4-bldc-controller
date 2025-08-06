/*
 * _bufferHandler.c
 *
 *  Created on: Aug 6, 2025
 *      Author: ADMIN
 */

#include "_bufferHandler.h"

uint8_t bufferHead;
uint8_t bufferTail;
uint8_t arrayBuffer[BUFFER_SIZE];

void bufferAdd(uint8_t buffer) {
    uint8_t nextHead = (bufferHead + 1) % BUFFER_SIZE;
    if (nextHead != bufferTail) {
        arrayBuffer[bufferHead] = buffer;
        bufferHead = nextHead;
    }
}

int isBufferReady() {
	return (bufferTail != bufferHead);
}

int bufferGet() {
	if (bufferTail != bufferHead) {
	  uint8_t buffer = arrayBuffer[bufferTail];
	  bufferTail = (bufferTail + 1) % BUFFER_SIZE;
	  return buffer;
	}
}
