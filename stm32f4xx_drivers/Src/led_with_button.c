/*
 * led_with_button.c
 *
 *  Created on: Mar. 1, 2023
 *      Author: altai
 */

#include "stm32f446xx.h"

void delay1(void)
{
	for(uint32_t i = 0; i < 250000; i++);
}

 int main(void)
 {
	 GPIO_Handler_t GPIOLed, GPIOBtn;

	 //GPIO LED INIT
	 GPIOLed.pGPIOx = GPIOA;
	 GPIOLed.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NO_5;
	 GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	 GPIOLed.GPIO_PinConfig.GPIO_PinSpd = GPIO_OUT_SPD_FAST;
	 GPIOLed.GPIO_PinConfig.GPIO_PinOutType = GPIO_OUT_TYPE_OD;
	 GPIOLed.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;

	 GPIO_ClockCtrl(GPIOLed.pGPIOx, ENABLE);
	 GPIO_Init(&GPIOLed);

	 //GPIO Button INIT
	 GPIOBtn.pGPIOx = GPIOC;
	 GPIOBtn.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NO_13;
	 GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	 GPIOBtn.GPIO_PinConfig.GPIO_PinSpd = GPIO_OUT_SPD_FAST;
	 GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;

	 GPIO_ClockCtrl(GPIOBtn.pGPIOx, ENABLE);
	 GPIO_Init(&GPIOBtn);

	 while(1)
	 {
		 uint8_t value = GPIO_ReadInputPin(GPIOBtn.pGPIOx, GPIOBtn.GPIO_PinConfig.GPIO_PinNum);
		 if(value == RESET)
		 {
			 GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
			 delay1();
		 }
		 else
		 {
			 GPIO_WriteOutputPin(GPIOA,GPIO_PIN_NO_5,RESET);
		 }
	 }

	 return 0;
 }
