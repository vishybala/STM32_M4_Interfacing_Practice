/*
 * led_toggle.c
 *
 *  Created on: Feb. 24, 2023
 *      Author: altai
 */

#include "stm32f446xx.h"

void led_toggle(GPIO_RegDef_t *pGPIOx)
 {
	 GPIO_DeInit(GPIOA);
	 GPIO_ClockCtrl(pGPIOx, DISABLE);
 }
void delay1(void)
{
	for(uint32_t i = 0; i < 100000; i++);
}

 int main(void)
 {
	 GPIO_Handler_t GPIOLed;

	 GPIOLed.pGPIOx = GPIOA;
	 GPIOLed.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NO_5;
	 GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	 GPIOLed.GPIO_PinConfig.GPIO_PinSpd = GPIO_OUT_SPD_FAST;
	 GPIOLed.GPIO_PinConfig.GPIO_PinOutType = GPIO_OUT_TYPE_OD;
	 GPIOLed.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;

	 GPIO_ClockCtrl(GPIOLed.pGPIOx, ENABLE);
	 GPIO_Init(&GPIOLed);

	 while(1)
	 {
		 GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		 delay1();
	 }

	 return 0;
 }
 
