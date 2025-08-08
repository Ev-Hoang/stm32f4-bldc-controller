#include "main.h"

#include "_stm32_init.h"
#include "_bufferHandler.h"
#include "_hall_sensor.h"

//======================================================
//TEST FUNCTION & VARIABLES
//======================================================

//This value will be used for the PID_Controller
uint8_t pwmVal = 50;

void CDC_Transmit(char *msg)
{
    uint16_t len = strlen(msg);
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t*)msg, len);
    USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}

//======================================================
//FUNCTIONS
//======================================================


//Interupted is called when HALL SENSOR got a change of data
//used to find the right hall sequence, and provide the next step for handleCommutation
void EXTI9_5_IRQHandler(void)
{
    bufferAdd(readHallSensor());

    EXTI->PR |= (1 << 5);
    EXTI->PR |= (1 << 6);
    EXTI->PR |= (1 << 7);
}

//Function initialize the BLDC, by picking the first HALL sequence,
//or create 1 if its undefined
void BLDC_Start() {
    bufferAdd(readHallSensor());
}

//======================================================
//MAIN
//======================================================

int main(void)
{
  STM32_Init();
  BLDC_Start();

  //Program loop
  while (1)
  {
	//Handling Buffers
	if(isBufferReady()) {
		handleCommutation(bufferGet(), pwmVal);
	}
	//Test sending
//	for(int i = 0 ; i < 100000; i++) {
//	}
//	CDC_Transmit("hello \r\n");

  }
}

//======================================================
//ERROR HANDLER
//======================================================
void Error_Handler(void)
{
    printf("Error Handler invoked!\n");
    while(1);
}
