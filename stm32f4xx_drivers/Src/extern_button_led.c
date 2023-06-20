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

void EXTI0_IRQHandler(void)
{
	// Handle the Interrupt
	GPIO_IRQHandling(0);
}

 int main(void)
 {
	 GPIO_Handler_t GPIOLed, GPIOBtn;

	 //GPIO LED INIT
	 GPIOLed.pGPIOx = GPIOA;
	 GPIOLed.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NO_8;
	 GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	 GPIOLed.GPIO_PinConfig.GPIO_PinSpd = GPIO_OUT_SPD_FAST;
	 GPIOLed.GPIO_PinConfig.GPIO_PinOutType = GPIO_OUT_TYPE_PP;
	 GPIOLed.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;

	 GPIO_ClockCtrl(GPIOLed.pGPIOx, ENABLE);
	 GPIO_Init(&GPIOLed);

	 //GPIO Button INIT
	 GPIOBtn.pGPIOx = GPIOB;
	 GPIOBtn.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NO_5;
	 GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	 GPIOBtn.GPIO_PinConfig.GPIO_PinSpd = GPIO_OUT_SPD_FAST;
	 GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;

	 GPIO_ClockCtrl(GPIOB, ENABLE);
	 GPIO_Init(&GPIOBtn);

	 //GPIO_ToggleOutputPin(GPIOLed.pGPIOx, GPIOLed.GPIO_PinConfig.GPIO_PinNum);

	 while(1)
	 {
		 uint8_t value = GPIO_ReadInputPin(GPIOBtn.pGPIOx, GPIOBtn.GPIO_PinConfig.GPIO_PinNum);
		 if(value == RESET)
		 {
			 delay1();
			 GPIO_ToggleOutputPin(GPIOLed.pGPIOx, GPIOLed.GPIO_PinConfig.GPIO_PinNum);
		 }
		 /*else
		 {
			 GPIO_WriteOutputPin(GPIOLed.pGPIOx,GPIOLed.GPIO_PinConfig.GPIO_PinNum,RESET);
		 }*/

	 }

	 return 0;
 }
