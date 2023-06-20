/*
 * stm32f446xx.h
 *
 *  Created on: Feb 23, 2023
 *      Author: VON
 */

#ifndef STM32F446XX_H_
#define STM32F446XX_H_

#include <stdint.h>
#include <stddef.h>

#define __vo volatile
#define __weak __attribute__((weak))

/************************************************************
 **************** PROCESSOR SPECIFIC DETAILS ****************
 ************************************************************/

/************* ARM CORTEX M4 NVIC ISERx BASE ADDRESSES *************/

#define NVIC_ISER0_ADDR				((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1_ADDR				((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2_ADDR				((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3_ADDR				((__vo uint32_t*)0xE000E10C)

/************* ARM CORTEX M4 NVIC ICERx BASE ADDRESSES *************/

#define NVIC_ICER0_ADDR				((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1_ADDR				((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2_ADDR				((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3_ADDR				((__vo uint32_t*)0xE000E18C)

/************* ARM CORTEX M4 NVIC PRIORITY REGISTER BASE ADDRESS *************/

#define NVIC_PR_ADDR				((__vo uint32_t*)0xE000E400)
#define NO_PR_BITS_IMPLEMENTED		4

/*************************************************************
 ************** MICROCONTROLLER SPECIFIC DETAILS *************
 *************************************************************/

/************* BASE ADDRESS OF MEMORY LOCATIONS MACROS *************/

/*
 * Base Addresses of Flash and SRAM Memory
 */

#define FLASH_BASEADDR			0x08000000U
#define SRAM1_BASEADDR			0x20000000U
#define SRAM2_BASEADDR			0x2001C000U
#define ROM						0x1FFF0000U
#define SRAM1					SRAM1_BASEADDR
#define SRAM2					SRAM2_BASEADDR

/*
 * Base Addresses of Peripheral Buses
 */

#define APB1_BASEADDR			0x40000000U
#define APB2_BASEADDR			0x40010000U

#define	AHB1_BASEADDR			0x40020000U
#define AHB2_BASEADDR			0x50000000U
#define AHB3_BASEADDR			0xA0001000U

#define PERIPH_BASEADDR			APB1_BASEADDR

/*
 * Base Addresses of Peripherals on AHB1 Bus
 */

#define GPIOA_BASEADDR			(AHB1_BASEADDR + 0x0000)
#define GPIOB_BASEADDR			(AHB1_BASEADDR + 0x0400)
#define GPIOC_BASEADDR			(AHB1_BASEADDR + 0x0800)
#define GPIOD_BASEADDR			(AHB1_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR			(AHB1_BASEADDR + 0x1000)
#define GPIOF_BASEADDR			(AHB1_BASEADDR + 0x1400)
#define GPIOG_BASEADDR			(AHB1_BASEADDR + 0x1800)
#define GPIOH_BASEADDR			(AHB1_BASEADDR + 0x1C00)

/*
 * Base Addresses of RCC on AHB1 Bus
 */

#define RCC_BASEADDR			(AHB1_BASEADDR + 0x3800)

/*
 * Base Addresses of Peripherals on APB1 Bus
 */

#define I2C1_BASEADDR			(APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1_BASEADDR + 0x5800)
#define I2C3_BASEADDR			(APB1_BASEADDR + 0x5C00)

#define SPI2_BASEADDR			(APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR			(APB1_BASEADDR + 0x3C00)

#define UART4_BASEADDR			(APB1_BASEADDR + 0x4C00)
#define UART5_BASEADDR			(APB1_BASEADDR + 0x5000)

#define USART2_BASEADDR			(APB1_BASEADDR + 0x4400)
#define USART3_BASEADDR			(APB1_BASEADDR + 0x4800)

/*
 * Base Addresses of Peripherals on APB2 Bus
 */

#define SPI1_BASEADDR			(APB2_BASEADDR + 0x3000)
#define SPI4_BASEADDR			(APB2_BASEADDR + 0x3400)

#define USART1_BASEADDR			(APB2_BASEADDR + 0x1000)
#define USART6_BASEADDR			(APB2_BASEADDR + 0x1400)

#define EXTI_BASEADDR			(APB2_BASEADDR + 0x3C00)

#define SYSCFG_BASEADDR			(APB2_BASEADDR + 0x3800)

/************* GPIO PERIPHERAL REGISTER DEFINITION STRUCTS *************/

typedef struct{
	__vo uint32_t MODER;			/* Address offset 0x00 Mode*/
	__vo uint32_t OTYPER;			/* Address offset 0x04 Output Type*/
	__vo uint32_t OSPEEDR;			/* Address offset 0x08 Output Speed*/
	__vo uint32_t PUPDR;			/* Address offset 0x0C Pull Up/Down Resistor*/
	__vo uint32_t IDR;				/* Address offset 0x10 Input Data Register*/
	__vo uint32_t ODR;				/* Address offset 0x14 Output Data Register*/
	__vo uint32_t BSRR;				/* Address offset 0x18 */
	__vo uint32_t LCKR;				/* Address offset 0x1C */
	__vo uint32_t AFRL;				/* Address offset 0x20 Alternate Function Low*/
	__vo uint32_t AFRH;				/* Address offset 0x24 Alternate Function High*/

}GPIO_RegDef_t;

/************* SPI PERIPHERAL REGISTER DEFINITION STRUCTS *************/

typedef struct{
	__vo uint32_t CR1;				/* Address offset 0x00 */
	__vo uint32_t CR2;				/* Address offset 0x04 */
	__vo uint32_t SR;				/* Address offset 0x08 */
	__vo uint32_t DR;				/* Address offset 0x0C */
	__vo uint32_t CRCPR;			/* Address offset 0x10 */
	__vo uint32_t RXCRCR;			/* Address offset 0x14 */
	__vo uint32_t TXCRCR;			/* Address offset 0x18 */
	__vo uint32_t I2SCFGR;			/* Address offset 0x1C */
	__vo uint32_t I2sPR;			/* Address offset 0x20 */

}SPI_RegDef_t;

/************* I2C PERIPHERAL REGISTER DEFINITION STRUCTS *************/

typedef struct{
	__vo uint32_t CR1;				/* Address offset 0x00 */
	__vo uint32_t CR2;				/* Address offset 0x04 */
	__vo uint32_t OAR1;				/* Address offset 0x08 */
	__vo uint32_t OAR2;				/* Address offset 0x0C */
	__vo uint32_t DR;				/* Address offset 0x10 */
	__vo uint32_t SR1;				/* Address offset 0x14 */
	__vo uint32_t SR2;				/* Address offset 0x18 */
	__vo uint32_t CCR;				/* Address offset 0x1C */
	__vo uint32_t TRISE;			/* Address offset 0x20 */
	__vo uint32_t FLTR;				/* Address offset 0x24 */
}I2C_RegDef_t;

/************* USART PERIPHERAL REGISTER DEFINITION STRUCTS *************/

typedef struct{
	__vo uint32_t SR;				/* Address offset 0x00 */
	__vo uint32_t DR;				/* Address offset 0x04 */
	__vo uint32_t BRR;				/* Address offset 0x08 */
	__vo uint32_t CR1;				/* Address offset 0x0C */
	__vo uint32_t CR2;				/* Address offset 0x10 */
	__vo uint32_t CR3;				/* Address offset 0x14 */
	__vo uint32_t GTPR;				/* Address offset 0x18 */
}USART_RegDef_t;

/************* EXTI PERIPHERAL REGISTER DEFINITION STRUCTS *************/

typedef struct{
	__vo uint32_t IMR;			/* Address offset 0x00 */
	__vo uint32_t EMR;			/* Address offset 0x04 */
	__vo uint32_t RTSR;			/* Address offset 0x08 */
	__vo uint32_t FTSR;			/* Address offset 0x0C */
	__vo uint32_t SWIER;		/* Address offset 0x10 */
	__vo uint32_t PR;			/* Address offset 0x14 */
}EXTI_RegDef_t;

/************* SYSCFG REGISTER DEFINITION STRUCTS *************/

typedef struct{
	__vo uint32_t MEMRMP;			/* Address offset 0x00 */
	__vo uint32_t PMC;				/* Address offset 0x04 */
	__vo uint32_t EXTICR[4];		/* Address offset 0x08-0x14 */
	uint32_t RESERVED1;				/* Address offset 0x18 */
	uint32_t RESERVED2;				/* Address offset 0x1C */
	__vo uint32_t CMPCR;			/* Address offset 0x20 */
	uint32_t RESERVED3;				/* Address offset 0x24 */
	uint32_t RESERVED4;				/* Address offset 0x28 */
	__vo uint32_t CFGR;				/* Address offset 0x2C */
}SYSCFG_RegDef_t;

/************* RCC REGISTER DEFINITION STRUCTS *************/

typedef struct{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	uint32_t RESERVED1;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t RESERVED2;
	uint32_t RESERVED3;
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	uint32_t RESERVED4;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t RESERVED5;
	uint32_t RESERVED6;
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	uint32_t RESERVED7;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t RESERVED8;
	uint32_t RESERVED9;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t RESERVEDA;
	uint32_t RESERVEDB;
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
	__vo uint32_t CKGATENR;
	__vo uint32_t DCKCFGR2;

}RCC_RegDef_t;

/************* PERIPHERAL REGISTER DEFINITION FOR GPIOx *************/
/*
 * GPIOx Registers Macros Typecasted as Struct GPIO_RegDef_t
 */

#define GPIOA		((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF		((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG		((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH		((GPIO_RegDef_t*)GPIOH_BASEADDR)

/************* PERIPHERAL REGISTER DEFINITION FOR SPIx *************/
/*
 * SPIx Registers Macros Typecasted as Struct SPI_RegDef_t
 */

#define SPI1		((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3		((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4		((SPI_RegDef_t*)SPI4_BASEADDR)

/************* PERIPHERAL REGISTER DEFINITION FOR I2Cx *************/
/*
 * I2Cx Registers Macros Typecasted as Struct I2C_RegDef_t
 */
#define I2C1		((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2		((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3		((I2C_RegDef_t*)I2C3_BASEADDR)

/************* PERIPHERAL REGISTER DEFINITION FOR UARTx/USARTx *************/
/*
 *  UARTx/USARTx Registers Macros Typecasted as Struct USART_RegDef_t
 */
#define USART1		((USART_RegDef_t*)USART1_BASEADDR)
#define USART2		((USART_RegDef_t*)USART2_BASEADDR)
#define USART3		((USART_RegDef_t*)USART3_BASEADDR)
#define UART4		((USART_RegDef_t*)UART4_BASEADDR)
#define UART5		((USART_RegDef_t*)UART5_BASEADDR)
#define USART6		((USART_RegDef_t*)USART6_BASEADDR)

/************* PERIPHERAL MISCELANEOUS REGISTER DEFINITION *************/

#define RCC			((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI		((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG		((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/************* CLOCK ENABLE MACROS FOR GPIOx PERIPHERALS *************/

#define GPIOA_CLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_CLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_CLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_CLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_CLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_CLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_CLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_CLK_EN()		(RCC->AHB1ENR |= (1 << 7))

/************* CLOCK DISABLE MACROS FOR GPIOx PERIPHERALS *************/

#define GPIOA_CLK_DI()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_CLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_CLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_CLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_CLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_CLK_DI()		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_CLK_DI()		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_CLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))

/************* RESET MACROS FOR GPIOx PERIPHERALS *************/

#define GPIOA_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));}while(0)
#define GPIOG_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));}while(0)
#define GPIOH_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));}while(0)

/************* RESET MACROS FOR SPIx PERIPHERALS *************/

#define SPI1_REG_RESET()	do{(RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12));}while(0)
#define SPI2_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14));}while(0)
#define SPI3_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15));}while(0)
#define SPI4_REG_RESET()	do{(RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13));}while(0)

/************* RESET MACROS FOR I2Cx PERIPHERALS *************/

#define I2C1_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21));}while(0)
#define I2C2_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22));}while(0)
#define I2C3_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23));}while(0)

/************* RESET MACROS FOR UART/USARTx PERIPHERALS *************/

#define USART1_REG_RESET()	do{(RCC->APB2RSTR |= (1 << 4)); (RCC->APB2RSTR &= ~(1 << 4));}while(0)
#define USART2_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 17)); (RCC->APB1RSTR &= ~(1 << 17));}while(0)
#define USART3_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 18)); (RCC->APB1RSTR &= ~(1 << 18));}while(0)
#define UART4_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 19)); (RCC->APB1RSTR &= ~(1 << 19));}while(0)
#define UART5_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 20)); (RCC->APB1RSTR &= ~(1 << 20));}while(0)
#define USART6_REG_RESET()	do{(RCC->APB2RSTR |= (1 << 5)); (RCC->APB2RSTR &= ~(1 << 5));}while(0)

/************* CLOCK ENABLE MACROS FOR I2Cx PERIPHERALS *************/

#define I2C1_CLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_CLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_CLK_EN()		(RCC->APB1ENR |= (1 << 23))

/************* CLOCK DISABLE MACROS FOR I2Cx PERIPHERALS *************/

#define I2C1_CLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_CLK_DI()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_CLK_DI()		(RCC->APB1ENR &= ~(1 << 23))

/************* CLOCK ENABLE MACROS FOR SPIx PERIPHERALS *************/

#define SPI1_CLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_CLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_CLK_EN()		(RCC->APB1ENR |= (1 << 15))
#define SPI4_CLK_EN()		(RCC->APB2ENR |= (1 << 13))

/************* CLOCK DISABLE MACROS FOR SPIx PERIPHERALS *************/

#define SPI1_CLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_CLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_CLK_DI()		(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_CLK_DI()		(RCC->APB2ENR &= ~(1 << 13))

/************* CLOCK ENABLE MACROS FOR UART/USARTx PERIPHERALS *************/

#define USART1_CLK_EN()		(RCC->APB2ENR |= (1 << 4))
#define USART2_CLK_EN()		(RCC->APB1ENR |= (1 << 17))
#define USART3_CLK_EN()		(RCC->APB1ENR |= (1 << 18))
#define UART4_CLK_EN()		(RCC->APB2ENR |= (1 << 19))
#define UART5_CLK_EN()		(RCC->APB1ENR |= (1 << 20))
#define USART6_CLK_EN()		(RCC->APB2ENR |= (1 << 5))

/************* CLOCK DISABLE MACROS FOR UART/USARTx PERIPHERALS *************/

#define USART1_CLK_DI()		(RCC->APB2ENR &= ~(1 << 4))
#define USART2_CLK_DI()		(RCC->APB1ENR &= ~(1 << 17))
#define USART3_CLK_DI()		(RCC->APB1ENR &= ~(1 << 18))
#define UART4_CLK_DI()		(RCC->APB2ENR &= ~(1 << 19))
#define UART5_CLK_DI()		(RCC->APB1ENR &= ~(1 << 20))
#define USART6_CLK_DI()		(RCC->APB2ENR &= ~(1 << 5))

/************* CLOCK ENABLE/DIABLE MACROS FOR SYSCFG PERIPHERALS *************/

#define SYSCFG_CLK_EN()		(RCC->APB2ENR |= (1 << 14))
#define SYSCFG_CLK_DI()		(RCC->APB2ENR &= ~(1 << 14))

/************* GPIO BASEADDRESS TO CODE MACROS FOR SYSCFG EXTICR PERIPHERALS *************/

#define GPIO_BASEADDR_TO_EXTICODE(x)	(	(x == GPIOA) ? 0:\
											(x == GPIOB) ? 1:\
											(x == GPIOC) ? 2:\
											(x == GPIOD) ? 3:\
											(x == GPIOE) ? 4:\
											(x == GPIOF) ? 5:\
											(x == GPIOG) ? 6:\
											(x == GPIOH) ? 7:0	)

/************* IRQ CODES AND THEIR POSITION ON VECTOR TABLE MACROS *************/

#define IRQ_NO_EXTI0				6
#define IRQ_NO_EXTI1				7
#define IRQ_NO_EXTI2				8
#define IRQ_NO_EXTI3				9
#define IRQ_NO_EXTI4				10
#define IRQ_NO_EXTI9_5				23
#define IRQ_NO_EXTI15_10			40
#define IRQ_SPI1_GLOBAL				35
#define IRQ_SPI2_GLOBAL				36
#define IRQ_SPI3_GLOBAL				51
#define IRQ_SPI4_GLOBAL				84
#define IRQ_I2C1_EV					31
#define IRQ_I2C1_ER					32
#define IRQ_I2C2_EV					33
#define IRQ_I2C2_ER					34
#define IRQ_I2C3_EV					72
#define IRQ_I2C3_ER					73
#define IRQ_USART1					37
#define IRQ_USART2					38
#define IRQ_USART3					39
#define IRQ_UART4					52
#define IRQ_UART5					53
#define IRQ_USART6					71

/************* MACROS FOR SETTING PRIORITY LEVELS OF IRQ *************/

#define NVIC_IRQ_PRIO0				0
#define NVIC_IRQ_PRIO1				1
#define NVIC_IRQ_PRIO2				2
#define NVIC_IRQ_PRIO3				3
#define NVIC_IRQ_PRIO4				4
#define NVIC_IRQ_PRIO5				5
#define NVIC_IRQ_PRIO6				6
#define NVIC_IRQ_PRIO7				7
#define NVIC_IRQ_PRIO8				8
#define NVIC_IRQ_PRIO9				9
#define NVIC_IRQ_PRIO10				10
#define NVIC_IRQ_PRIO11				11
#define NVIC_IRQ_PRIO12				12
#define NVIC_IRQ_PRIO13				13
#define NVIC_IRQ_PRIO14				14
#define NVIC_IRQ_PRIO15				15
#define NVIC_IRQ_PRIO16				16
#define NVIC_IRQ_PRIO17				17
#define NVIC_IRQ_PRIO18				18
#define NVIC_IRQ_PRIO19				19
#define NVIC_IRQ_PRIO20				20
#define NVIC_IRQ_PRIO21				21
#define NVIC_IRQ_PRIO22				22
#define NVIC_IRQ_PRIO23				23

// PLACEHOLDER GENERIC MACRO
#define ENABLE			1
#define DISABLE			0
#define SET				ENABLE
#define RESET			DISABLE
#define READ			ENABLE
#define WRITE			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_RESET		RESET
#define FLAG_SET		SET

/*****************************************************************
 ************** BIT POS DEFINITIONS OF SPI REGISTERS *************
 *****************************************************************/

// SPI_CR1 BIT POSITONS
#define SPI_CR1_CPHA				0
#define SPI_CR1_CPOL				1
#define SPI_CR1_MSTR				2
#define SPI_CR1_BR					3
#define SPI_CR1_SPE					6
#define SPI_CR1_LSBFIRST			7
#define SPI_CR1_SSI					8
#define SPI_CR1_SSM					9
#define SPI_CR1_RXONLY				10
#define SPI_CR1_DFF					11
#define SPI_CR1_CRCNEXT				12
#define SPI_CR1_CRCEN				13
#define SPI_CR1_BIDIOE				14
#define SPI_CR1_BIDIMODE			15

// SPI_CR2 BIT POSITONS
#define SPI_CR2_RXDMAEN				0
#define SPI_CR2_TXDMAEN				1
#define SPI_CR2_SSOE				2
#define SPI_CR2_FRF					4
#define SPI_CR2_ERRIE				5
#define SPI_CR2_RXNEIE				6
#define SPI_CR2_TXEIE				7

// SPI_SR BIT POSITIONS
#define SPI_SR_RXNE					0
#define SPI_SR_TXE					1
#define SPI_SR_CHSIDE				2
#define SPI_SR_UDR					3
#define SPI_SR_CRCERR				4
#define SPI_SR_MODF					5
#define SPI_SR_OVR					6
#define SPI_SR_BSY					7
#define SPI_SR_FRE					8

/*****************************************************************
 ************** BIT POS DEFINITIONS OF I2C REGISTERS *************
 *****************************************************************/

// I2C_CR1 BIT POSITONS
#define I2C_CR1_PE					0
#define I2C_CR1_SMBUS				1
#define I2C_CR1_SMBTYPE				3
#define I2C_CR1_ENARP				4
#define I2C_CR1_ENPEC				5
#define I2C_CR1_ENGC				6
#define I2C_CR1_NOSTRETCH			7
#define I2C_CR1_START				8
#define I2C_CR1_STOP				9
#define I2C_CR1_ACK					10
#define I2C_CR1_POS					11
#define I2C_CR1_PEC					12
#define I2C_CR1_ALERT				13
#define I2C_CR1_SWRST				15

// I2C_CR2 BIT POSITONS
#define I2C_CR2_FREQ				0
#define I2C_CR2_ITERREN				8
#define I2C_CR2_ITEVTEN				9
#define I2C_CR2_ITBUFEN				10
#define I2C_CR2_DMAEN				11
#define I2C_CR2_LAST				12

// I2C_SR1 BIT POSITIONS
#define I2C_SR1_SB					0
#define I2C_SR1_ADDR				1
#define I2C_SR1_BTF					2
#define I2C_SR1_ADD10				3
#define I2C_SR1_STOPF				4
#define I2C_SR1_RXNE				6
#define I2C_SR1_TXE					7
#define I2C_SR1_BERR				8
#define I2C_SR1_ARLO				9
#define I2C_SR1_AF					10
#define I2C_SR1_OVR					11
#define I2C_SR1_PECERR				12
#define I2C_SR1_TIMEOUT				14
#define I2C_SR1_SMBALERT			15

// I2C_SR2 BIT POSITIONS
#define I2C_SR2_MSL					0
#define I2C_SR2_BUSY				1
#define I2C_SR2_TRA					2
#define I2C_SR2_GENCALL				4
#define I2C_SR2_SMBDEFAULT			5
#define I2C_SR2_SMBHOST				6
#define I2C_SR2_DUALF				7
#define I2C_SR2_PEC					8

/************************************************************************
 ************** BIT POS DEFINITIONS OF UART/USART REGISTERS *************
 ************************************************************************/
// USART_CR1 BIT POSITONS
#define USART_CR1_SBK				0
#define USART_CR1_RWU				1
#define USART_CR1_RE				2
#define USART_CR1_TE				3
#define USART_CR1_IDLEIE			4
#define USART_CR1_RXNEIE			5
#define USART_CR1_TCIE				6
#define USART_CR1_TXEIE				7
#define USART_CR1_PEIE				8
#define USART_CR1_PS				9
#define USART_CR1_PCE				10
#define USART_CR1_WAKE				11
#define USART_CR1_M					12
#define USART_CR1_UE				13
#define USART_CR1_OVER8				15

// USART_CR2 BIT POSITONS
#define USART_CR2_ADD				0
#define USART_CR2_LBDL				5
#define USART_CR2_LBDIE				6
#define USART_CR2_LBCL				8
#define USART_CR2_CPHA				9
#define USART_CR2_CPOL				10
#define USART_CR2_CLKEN				11
#define USART_CR2_STOP				12
#define USART_CR2_LINEN				14

// USART_CR3 BIT POSITONS
#define USART_CR3_EIE				0
#define USART_CR3_IREN				0
#define USART_CR3_IRLP				0
#define USART_CR3_HDSEL				0
#define USART_CR3_NACK				0
#define USART_CR3_SCEN				0
#define USART_CR3_DMAR				0
#define USART_CR3_DMAT				0
#define USART_CR3_RTSE				0
#define USART_CR3_CTSE				0
#define USART_CR3_CTSIE				0
#define USART_CR3_ONEBIT			0

// USART_SR BIT POSITONS
#define USART_SR_PE					0
#define USART_SR_FE					1
#define USART_SR_NF					2
#define USART_SR_ORE				3
#define USART_SR_IDLE				4
#define USART_SR_RXNE				5
#define USART_SR_TC					6
#define USART_SR_TXE				7
#define USART_SR_LBD				8
#define USART_SR_CTS				9


#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_i2c_driver.h"
#include "stm32f446xx_usart_driver.h"
#include "stm32f446xx_rcc_driver.h"

#endif /* STM32F446XX_H_ */
