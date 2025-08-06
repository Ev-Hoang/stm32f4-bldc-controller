#include "main.h"

#include "_stm32_init.h"
#include "_bufferHandler.h"

// Value in the PWM
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

//HALL sequence during the rotations
const int8_t hallCWLookup[8] = {
    -1, // 0b000 (0): invalid
     0, // 0b001 (1)
     1, // 0b010 (2)
     2, // 0b011 (3)
     3, // 0b100 (4)
     4, // 0b101 (5)
     5, // 0b110 (6)
    -1  // 0b111 (7): invalid
};

//This value will be used for the PID_Controller
uint8_t pwmVal = 50;

volatile uint8_t hallState = 0;
volatile uint8_t currentCommStep = 0; // Trạng thái commutation

//======================================================
//TEST FUNCTION
//======================================================

void CDC_Transmit(char *msg)
{
    uint16_t len = strlen(msg);
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t*)msg, len);
    USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}

//======================================================
//FUNCTIONS
//======================================================

//Function use to handle the hall sequences, provided through "Steps",
//and given the value "pwmVal" to change duty cycle for specific step.
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
        case 255:
            // Tắt tất cả các pha khi không xác định
            SET_PWM_1_H1(highMin);
			SET_PWM_1_H2(highMin);
			SET_PWM_1_H3(highMin);
			SET_PWM_1_L1(lowMin);
			SET_PWM_1_L2(lowMin);
			SET_PWM_1_L3(lowMin);
    }
}

//Interupted is called when HALL SENSOR got a change of data
//used to find the right hall sequence, and provide the next step for handleCommutation
void EXTI9_5_IRQHandler(void)
{
    uint32_t idr = GPIOA->IDR;
    uint8_t hallA = (idr >> 5) & 0x01;
    uint8_t hallB = (idr >> 6) & 0x01;
    uint8_t hallC = (idr >> 7) & 0x01;

    hallState = (hallA << 2) | (hallB << 1) | hallC;

    int8_t step = hallCWLookup[hallState];
    if (step >= 0) {currentCommStep = step;}
    else {currentCommStep = 0;}

    bufferAdd(currentCommStep);

    EXTI->PR |= (1 << 5);
    EXTI->PR |= (1 << 6);
    EXTI->PR |= (1 << 7);
}

//Function initialize the BLDC, by picking the first HALL sequence,
//or create 1 if its undefined
void BLDC_Start() {
    uint32_t idr = GPIOA->IDR;
    uint8_t hallA = (idr >> 5) & 0x01;
    uint8_t hallB = (idr >> 6) & 0x01;
    uint8_t hallC = (idr >> 7) & 0x01;

    hallState = (hallA << 2) | (hallB << 1) | hallC;

    switch (hallState) {
        case hallSequenceCW[0]: currentCommStep = 0; break;
        case hallSequenceCW[1]: currentCommStep = 1; break;
        case hallSequenceCW[2]: currentCommStep = 2; break;
        case hallSequenceCW[3]: currentCommStep = 3; break;
        case hallSequenceCW[4]: currentCommStep = 4; break;
        case hallSequenceCW[5]: currentCommStep = 5; break;
        default:    currentCommStep = 0; break;
    }

    bufferAdd(currentCommStep);
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
