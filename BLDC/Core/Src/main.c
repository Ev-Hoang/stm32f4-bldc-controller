#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

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
const uint8_t hallSequenceCW[6]  = {0b001, 0b010, 0b011, 0b100, 0b101, 0b110};

//This value will be used for the PID_Controller
uint8_t pwmVal = 50;

//Initilization of circular buffer
#define BUFFER_SIZE 2
volatile uint8_t commBuffer[BUFFER_SIZE];
volatile uint8_t bufferHead = 0;
volatile uint8_t bufferTail = 0;

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
    // 1. Đọc trực tiếp giá trị các chân PA5, PA6, PA7 từ GPIOA->IDR
    uint32_t idr = GPIOA->IDR;
    uint8_t hallA = (idr >> 5) & 0x01;
    uint8_t hallB = (idr >> 6) & 0x01;
    uint8_t hallC = (idr >> 7) & 0x01;

    // 2. Gộp thành hallState
    hallState = (hallA << 2) | (hallB << 1) | hallC;

    // 3. Tra bảng commutation
    switch (hallState) {
        case 0b001: currentCommStep = 0; break;
        case 0b011: currentCommStep = 1; break;
        case 0b010: currentCommStep = 2; break;
        case 0b110: currentCommStep = 3; break;
        case 0b100: currentCommStep = 4; break;
        case 0b101: currentCommStep = 5; break;
        default:    currentCommStep = 255; break;
    }

    // 4. Ghi vào buffer nếu còn trống
    uint8_t nextHead = (bufferHead + 1) % BUFFER_SIZE;
    if (nextHead != bufferTail) {
        commBuffer[bufferHead] = currentCommStep;
        bufferHead = nextHead;
    }

    // 5. Xóa cờ ngắt EXTI line 5~9 (tránh ngắt lặp lại)
    EXTI->PR |= (1 << 5);
    EXTI->PR |= (1 << 6);
    EXTI->PR |= (1 << 7);
}

//Function initialize the BLDC, by picking the first HALL sequence,
//or create 1 if its undefined
void BLDC_Start() {
  // Đọc giá trị các chân PA5, PA6, PA7 trực tiếp từ thanh ghi
  uint32_t idr = GPIOA->IDR;
  uint8_t hallA = (idr >> 5) & 0x01;
  uint8_t hallB = (idr >> 6) & 0x01;
  uint8_t hallC = (idr >> 7) & 0x01;

  hallState = (hallA << 2) | (hallB << 1) | hallC;

  // Nếu hallState không hợp lệ (000), đặt bước đầu tiên
  if (hallState == 0) {
	  currentCommStep = 0;
  } else {
	  // Tra trong bảng hallSequenceCW[]
	  for (uint8_t i = 0; i < 6; i++) {
		  if (hallState == hallSequenceCW[i]) {
			  currentCommStep = i;
			  break;
		  }
	  }
  }
    handleCommutation(currentCommStep, pwmVal);
}

static void SystemClock_Config(void);
static void GPIO_Init(void);
static void TIM2_Init(void);
static void TIM3_Init(void);
static void TIM4_Init(void);
static void USART1_UART_Init(void);

//======================================================
//MAIN
//======================================================

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_USB_DEVICE_Init();

  GPIO_Init();

  TIM2_Init();
  TIM3_Init();
  TIM4_Init();

  USART1_UART_Init();

  BLDC_Start();

  while (1)
  {
	if (bufferTail != bufferHead) {
	  uint8_t step = commBuffer[bufferTail];
	  bufferTail = (bufferTail + 1) % BUFFER_SIZE;
	  handleCommutation(step, pwmVal);
	}

	for(int i = 0 ; i < 100000; i++) {
	}
	CDC_Transmit("hello \r\n");
  }
}

//======================================================
//SYSTEMCLOCK CONFIG
//======================================================
void SystemClock_Config(void)
{
    // 1. Enable HSI (High-Speed Internal Clock)
    RCC->CR |= RCC_CR_HSION;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0); // Wait till HSI is ready

    // 2. Reset PLL
    RCC->CR &= ~RCC_CR_PLLON; // Turn off PLL
    while (RCC->CR & RCC_CR_PLLRDY); // Wait PLL disabled

    // 3. Configure PLL
    /*
     * HSI = 16 MHz
     * PLL_M = 16
     * PLL_N = 336
     * PLL_P = 4 (00: /2, 01: /4, ...)
     * PLL_Q = 7
     *
     * SYSCLK = ((16 MHz / 16) * 336) / 4 = 84 MHz
     */
    RCC->PLLCFGR = (16 << RCC_PLLCFGR_PLLM_Pos) |
                   (336 << RCC_PLLCFGR_PLLN_Pos) |
                   (1 << RCC_PLLCFGR_PLLP_Pos) |  // PLLP = 4 -> 01b
                   (7 << RCC_PLLCFGR_PLLQ_Pos) |
                   (RCC_PLLCFGR_PLLSRC_HSI);      // PLL source = HSI

    // 4. Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0); // Wait till PLL is ready

    // 5. Configure Flash Latency
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_2WS;

    // 6. Set AHB, APB1, APB2 prescaler
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;   // AHB prescaler = /1 -> 84 MHz
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;  // APB1 prescaler = /2 -> 42 MHz
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;  // APB2 prescaler = /1 -> 84 MHz

    // 7. Select PLL as system clock
    RCC->CFGR &= ~RCC_CFGR_SW;         // Clear SW bits
    RCC->CFGR |= RCC_CFGR_SW_PLL;      // Select PLL as system clock
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // Wait till PLL is system clock
}

//======================================================
//TIMER INITIALIZE
//======================================================
void TIM2_Init(void) {
    // Enable TIM2 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Set prescaler and auto-reload for 1kHz PWM
    TIM2->PSC = 3;     // 84MHz / (3+1) = 21MHz
    TIM2->ARR = 1000;   // PWM frequency ~ 20.9Khz

    // PWM mode 1, preload enable for CH1-CH4
    TIM2->CCMR1 |= (6 << 4) | TIM_CCMR1_OC1PE;  // CH1
    TIM2->CCMR1 |= (6 << 12) | TIM_CCMR1_OC2PE; // CH2
    TIM2->CCMR2 |= (6 << 4) | TIM_CCMR2_OC3PE;  // CH3
    TIM2->CCMR2 |= (6 << 12) | TIM_CCMR2_OC4PE; // CH4

    // Enable outputs
    TIM2->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E |
                  TIM_CCER_CC3E | TIM_CCER_CC4E;

    // Force update
    TIM2->EGR |= TIM_EGR_UG;

    // Set duty cycle = 0
    TIM2->CCR1 = 0;
    TIM2->CCR2 = 0;
    TIM2->CCR3 = 0;
    TIM2->CCR4 = 0;

    // Enable counter
    TIM2->CR1 |= TIM_CR1_CEN;
}

void TIM3_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->PSC = 3;
    TIM3->ARR = 1000;

    TIM3->CCMR1 |= (6 << 4) | TIM_CCMR1_OC1PE;
    TIM3->CCMR1 |= (6 << 12) | TIM_CCMR1_OC2PE;
    TIM3->CCMR2 |= (6 << 4) | TIM_CCMR2_OC3PE;
    TIM3->CCMR2 |= (6 << 12) | TIM_CCMR2_OC4PE;

    TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E |
                  TIM_CCER_CC3E | TIM_CCER_CC4E;

    TIM3->EGR |= TIM_EGR_UG;

    TIM3->CCR1 = 0;
    TIM3->CCR2 = 0;
    TIM3->CCR3 = 0;
    TIM3->CCR4 = 0;

    TIM3->CR1 |= TIM_CR1_CEN;
}

void TIM4_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    TIM4->PSC = 3;
    TIM4->ARR = 1000;

    TIM4->CCMR1 |= (6 << 4) | TIM_CCMR1_OC1PE;
    TIM4->CCMR1 |= (6 << 12) | TIM_CCMR1_OC2PE;
    TIM4->CCMR2 |= (6 << 4) | TIM_CCMR2_OC3PE;
    TIM4->CCMR2 |= (6 << 12) | TIM_CCMR2_OC4PE;

    TIM4->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E |
                  TIM_CCER_CC3E | TIM_CCER_CC4E;

    TIM4->EGR |= TIM_EGR_UG;

    TIM4->CCR1 = 0;
    TIM4->CCR2 = 0;
    TIM4->CCR3 = 0;
    TIM4->CCR4 = 0;

    TIM4->CR1 |= TIM_CR1_CEN;
}

//======================================================
//GPIO INITIALIZE
//======================================================
void GPIO_Init(void) {
    // Enable GPIOA and GPIOB clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;


    // SET TIMER PIN FOR Alternate Function
    // PA0–PA3 (TIM2_CH1–CH4)
    GPIOA->MODER &= ~(0xFF << (0 * 2));          // Clear mode bits for PA0–PA3
    GPIOA->MODER |=  (0xAA << (0 * 2));          // Set mode = AF (10)
    GPIOA->AFR[0] &= ~0xFFFF;                    // Clear AFRL[15:0]
    GPIOA->AFR[0] |= (0x1111 << 0);              // AF1 for PA0–PA3

    // PA6–PA7 (TIM3_CH1–CH2), AF2
    GPIOA->MODER &= ~(0xF << (6 * 2));
    GPIOA->MODER |=  (0xA << (6 * 2));
    GPIOA->AFR[0] &= ~(0xFF << (6 * 4));
    GPIOA->AFR[0] |=  (0x22 << (6 * 4));

    // PB0–PB1 (TIM3_CH3–CH4), AF2
    GPIOB->MODER &= ~(0xF << (0 * 2));
    GPIOB->MODER |=  (0xA << (0 * 2));
    GPIOB->AFR[0] &= ~(0xFF << (0 * 4));
    GPIOB->AFR[0] |=  (0x22 << (0 * 4));

    // PB6–PB9 (TIM4_CH1–CH4), AF2
    GPIOB->MODER &= ~(0xFF << (6 * 2));
    GPIOB->MODER |=  (0xAA << (6 * 2));

    GPIOB->AFR[0] &= ~((0xF << (6 * 4)) | (0xF << (7 * 4)));
    GPIOB->AFR[0] |=  ((0x2 << (6 * 4)) | (0x2 << (7 * 4)));
    GPIOB->AFR[1] &= ~((0xF << (0 * 4)) | (0xF << (1 * 4)));
    GPIOB->AFR[1] |=  ((0x2 << (0 * 4)) | (0x2 << (1 * 4)));


    // SET INPUT PIN FOR HALL SENSOR ( PA5 6 7)
    GPIOA->MODER &= ~(0b11 << (5 * 2));  // Clear PA5
    GPIOA->MODER &= ~(0b11 << (6 * 2));  // Clear PA6
    GPIOA->MODER &= ~(0b11 << (7 * 2));  // Clear PA7

    GPIOA->PUPDR &= ~(0b11 << (5 * 2));
    GPIOA->PUPDR |=  (0b01 << (5 * 2));  // Pull-up cho PA5

    GPIOA->PUPDR &= ~(0b11 << (6 * 2));
    GPIOA->PUPDR |=  (0b01 << (6 * 2));  // Pull-up cho PA6

    GPIOA->PUPDR &= ~(0b11 << (7 * 2));
    GPIOA->PUPDR |=  (0b01 << (7 * 2));  // Pull-up cho PA7

    //SET PA9 (TX) và PA10 (RX) ở AF7 FOR USART1
    GPIOA->MODER &= ~((0b11 << (9 * 2)) | (0b11 << (10 * 2))); // Clear
    GPIOA->MODER |=  ((0b10 << (9 * 2)) | (0b10 << (10 * 2))); // AF mode

    GPIOA->AFR[1] &= ~((0xF << (1 * 4)) | (0xF << (2 * 4)));   // Clear AFRH9/10
    GPIOA->AFR[1] |=  ((0x7 << (1 * 4)) | (0x7 << (2 * 4)));   // AF7 = USART1

}

//======================================================
//UART INITIALIZE
//======================================================
void USART1_UART_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;  // Enable USART1 (nằm trên APB2)

    USART1->CR1 = 0;  // Tắt USART trước khi cấu hình

    // Baudrate: ví dụ 115200 với PCLK2 = 84MHz
    // USARTDIV = 84MHz / (16 * 115200) ≈ 45.5625
    // Mantissa = 45, Fraction = 0.5625 * 16 ≈ 9 → BRR = 0x2D9
    USART1->BRR = (45 << 4) | 9;

    // 8-bit data, 1 stop bit, no parity, no flow control
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;  // Enable TX + RX
    USART1->CR1 |= USART_CR1_UE;                 // Bật USART
}

//======================================================
//ERROR HANDLER
//======================================================
void Error_Handler(void)
{
    printf("Error Handler invoked!\n");
    while(1);
}
