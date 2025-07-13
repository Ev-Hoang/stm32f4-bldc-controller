#include "main.h"
#include "stm32f4xx_it.h"

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

void SysTick_Handler(void)
{
  HAL_IncTick();
}

void OTG_FS_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}
