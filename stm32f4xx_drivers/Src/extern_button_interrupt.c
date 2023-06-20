/*
 * led_with_button.c
 *
 *  Created on: Mar. 1, 2023
 *      Author: altai
 */

#include "stm32f446xx.h"
#include <string.h>

void delay1(void)
{
	for(uint32_t i = 0; i < 200000; i++);
}

void EXTI9_5_IRQHandler(void)
{
	delay1();
	// Handle the Interrupt
	GPIO_IRQHandling(GPIO_PIN_NO_5);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_10);
}

 int main(void)
 {
	 GPIO_Handler_t GPIOLed, GPIOBtn;
	 memset(&GPIOLed,0,sizeof(GPIOLed));
	 memset(&GPIOBtn,0,sizeof(GPIOBtn));

	 //GPIO LED INIT
	 GPIOLed.pGPIOx = GPIOA;
	 GPIOLed.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NO_10;
	 GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	 GPIOLed.GPIO_PinConfig.GPIO_PinSpd = GPIO_OUT_SPD_FAST;
	 GPIOLed.GPIO_PinConfig.GPIO_PinOutType = GPIO_OUT_TYPE_PP;
	 GPIOLed.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;

	 GPIO_ClockCtrl(GPIOLed.pGPIOx, ENABLE);
	 GPIO_Init(&GPIOLed);

	 //GPIO Button INIT
	 GPIOBtn.pGPIOx = GPIOB;
	 GPIOBtn.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NO_5;
	 GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INT_FT;
	 GPIOBtn.GPIO_PinConfig.GPIO_PinSpd = GPIO_OUT_SPD_FAST;
	 GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;

	 GPIO_ClockCtrl(GPIOBtn.pGPIOx, ENABLE);
	 GPIO_Init(&GPIOBtn);

	 // IRQ Configurations
	 GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRIO15);
	 GPIO_IRQEnableConfig(IRQ_NO_EXTI9_5, ENABLE);

	 return 0;
 }
