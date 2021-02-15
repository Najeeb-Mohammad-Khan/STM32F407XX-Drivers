/*
 * stm32f407xx.h
 *
 *  Created on: Dec 18, 2020
 *      Author: Najeeb Mohammad Khan
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#define __VOL	volatile


/*
 * ***************    PROCESSOR SPECIFIC DETAILS    *******************
 */

//ARM CORETEX Mx NVIC ISERx REGISTER ADDRESSES
#define NVIC_ISER0					((__VOL uint32_t*)0xE000E100)
#define NVIC_ISER1					((__VOL uint32_t*)0xE000E104)
#define NVIC_ISER2					((__VOL uint32_t*)0xE000E108)
#define NVIC_ISER3					((__VOL uint32_t*)0xE000E10C)

//ARM CORETEX Mx NVIC ICERx REGISTER ADDRESS
#define NVIC_ICER0					((__VOL uint32_t*)0XE000E180)
#define NVIC_ICER1					((__VOL uint32_t*)0XE000E184)
#define NVIC_ICER2					((__VOL uint32_t*)0XE000E188)
#define NVIC_ICER3					((__VOL uint32_t*)0XE000E18C)

//ARM CORETEX Mx Priority REGISTER ADDRESS
#define NVIC_PR_BASEADDR			((__VOL uint32_t*)0xE000E400)
#define NO_PR_BITS_IMPLEMENTED		4
/*********************************************************************/

/* Base Address Of Flash And SRAM Memory */
#define FLASH_BASEADDR				0x08000000U	/*Flash Base Address*/

#define SRAM1_BASEADDR				0x20000000U	/*SRAM1 Base Address*/
#define SRAM2_BASEADDR				0x2001C000U	/*SRAM2 Base Address*/
#define SRAM SRAM1_BASEADDR							/*SRAM1 is used as SRAM(SRAM2 Not Used)*/

#define ROM							0x1FFF0000U	/*ROM Base Address*/


/*Peripheral Base Address*/
#define PERIPH_BASE					0x40000000U	/*Base Address For Peripherals*/

#define APB1_PERIPH_BASE			PERIPH_BASE		/*Base Address For APB1 Bus*/
#define APB2_PERIPH_BASE			0x40010000U	/*Base Address For APB2 Bus*/

#define AHB1_PERIPH_BASE			0x40020000U	/*Base Address For AHB1 Bus*/
#define AHB2_PERIPH_BASE			0x50000000U	/*Base Address For AHB2 Bus*/


/*AHB1 BUS PERIPHERALS*/
#define GPIOA_BASE_ADDR				(AHB1_PERIPH_BASE + 0x0000)	/*GPIOA OFFSET =  0x0000*/
#define GPIOB_BASE_ADDR				(AHB1_PERIPH_BASE + 0x0400)	/*GPIOB OFFSET =  0x0400*/
#define GPIOC_BASE_ADDR				(AHB1_PERIPH_BASE + 0x0800)	/*GPIOC OFFSET =  0x0800*/
#define GPIOD_BASE_ADDR				(AHB1_PERIPH_BASE + 0x0C00)	/*GPIOD OFFSET =  0x0C00*/
#define GPIOE_BASE_ADDR				(AHB1_PERIPH_BASE + 0x1000)	/*GPIOE OFFSET =  0x1000*/
#define GPIOF_BASE_ADDR				(AHB1_PERIPH_BASE + 0x1400)	/*GPIOF OFFSET =  0x1400*/
#define GPIOG_BASE_ADDR				(AHB1_PERIPH_BASE + 0x1800)	/*GPIOG OFFSET =  0x1800*/
#define GPIOH_BASE_ADDR				(AHB1_PERIPH_BASE + 0x1C00)	/*GPIOH OFFSET =  0x1C00*/
#define GPIOI_BASE_ADDR				(AHB1_PERIPH_BASE + 0x2000)	/*GPIOI OFFSET =  0x2000*/
#define GPIOJ_BASE_ADDR				(AHB1_PERIPH_BASE + 0x2400)	/*GPIOJ OFFSET =  0x2400*/
#define GPIOK_BASE_ADDR				(AHB1_PERIPH_BASE + 0x2800)	/*GPIOK OFFSET =  0x2800*/

#define RCC_BASE_ADDR				(AHB1_PERIPH_BASE + 0x3800) /*RCC OFFSET = 0x3800*/

#define DMA_BASE_ADDR				(AHB1_PERIPH_BASE + 0x6000)	/*DMA OFFSET =  0x6000*/


/*APB1 BUS PERIPHERALS*/
#define I2C1_BASE_ADDR				(APB1_PERIPH_BASE + 0x5400) /*I2C1 OFFSET = 0x5400*/
#define I2C2_BASE_ADDR				(APB1_PERIPH_BASE + 0x5800) /*I2C2 OFFSET = 0x5800*/
#define I2C3_BASE_ADDR				(APB1_PERIPH_BASE + 0x5C00) /*I2C3 OFFSET = 0x5C00*/

#define SPI2_BASE_ADDR				(APB1_PERIPH_BASE + 0x3800) /*SPI2 OFFSET = 0x3800*/
#define SPI3_BASE_ADDR				(APB1_PERIPH_BASE + 0x3C00) /*SPI3 OFFSET = 0x3C00*/

#define USART2_BASE_ADDR			(APB1_PERIPH_BASE + 0x4400) /*USART2 OFFSET = 0x4400*/
#define USART3_BASE_ADDR			(APB1_PERIPH_BASE + 0x4800) /*USART3 OFFSET = 0x4800*/

#define UART4_BASE_ADDR				(APB1_PERIPH_BASE + 0x4C00) /*UART4 OFFSET = 0x4C00*/
#define UART5_BASE_ADDR				(APB1_PERIPH_BASE + 0x5000) /*UART5 OFFSET = 0x5000*/

#define PWR_BASE_ADDR				(APB1_PERIPH_BASE + 0x7000) /*PWR OFFSET = 0x7000*/


/*APB2 BUS PERIPHERALS*/
#define SPI1_BASE_ADDR				(APB2_PERIPH_BASE + 0x3000) /*SPI1 OFFSET = 0x3000*/
#define SPI4_BASE_ADDR				(APB2_PERIPH_BASE + 0x3400) /*SPI4 OFFSET = 0x3400*/

#define USART1_BASE_ADDR			(APB2_PERIPH_BASE + 0x1000) /*USART1 OFFSET = 0x1000*/
#define USART6_BASE_ADDR			(APB2_PERIPH_BASE + 0x1400) /*USART6 OFFSET = 0x1400*/

#define EXTI_BASE_ADDR				(APB2_PERIPH_BASE + 0x3C00) /*EXT1 OFFSET = 0x3C00*/
#define SYSCFG_BASE_ADDR			(APB2_PERIPH_BASE + 0x3800) /*SYSCFG OFFSET = 0x3800*/


/*
 * Macro to calculate required Bits to be written in EXTICRx.
 */
#define GPIO_Base_Addr_To_Code(GPIOx)	( (GPIOx == GPIOA) ? 0 :\
										  (GPIOx == GPIOB) ? 1 :\
										  (GPIOx == GPIOC) ? 2 :\
										  (GPIOx == GPIOD) ? 3 :\
										  (GPIOx == GPIOE) ? 4 :\
										  (GPIOx == GPIOF) ? 5 :\
										  (GPIOx == GPIOG) ? 6 :\
										  (GPIOx == GPIOH) ? 7 :\
										  (GPIOx == GPIOA) ? 8 :0 )


typedef struct
{
	__VOL uint32_t CR;		/*PWR power control register.						ADDRESS OFFSET = 0x00*/
	__VOL uint32_t CSR;		/*PWR power control/status register.				ADDRESS OFFSET = 0x04*/

}PWR_Reg;


typedef struct
{
	__VOL uint32_t CR;			/*RCC clock control register.						ADDRESS OFFSET = 0x00*/
	__VOL uint32_t PLLCFGR;		/*RCC PLL configuration register.					ADDRESS OFFSET = 0x04*/
	__VOL uint32_t CFGR;		/*RCC clock configuration register.					ADDRESS OFFSET = 0x08*/
	__VOL uint32_t CIR;			/*RCC clock interrupt register.						ADDRESS OFFSET = 0x0C*/
	__VOL uint32_t AHB1RSTR;	/*RCC AHB1 peripheral reset register.				ADDRESS OFFSET = 0x10*/
	__VOL uint32_t AHB2RSTR;	/*RCC AHB2 peripheral reset register.				ADDRESS OFFSET = 0x14*/
	__VOL uint32_t AHB3RSTR;	/*RCC AHB3 peripheral reset register.				ADDRESS OFFSET = 0x18*/
	__VOL uint32_t Reserved0;	/*RCC Reserved register.							ADDRESS OFFSET = 0x1C*/
	__VOL uint32_t APB1RSTR;	/*RCC APB1 peripheral reset register.				ADDRESS OFFSET = 0x20*/
	__VOL uint32_t Reserved1;	/*RCC Reserved register.							ADDRESS OFFSET = 0x28*/
	__VOL uint32_t Reserved2;	/*RCC Reserved register.							ADDRESS OFFSET = 0x2C*/
	__VOL uint32_t APB2RSTR;	/*RCC APB2 peripheral reset register.				ADDRESS OFFSET = 0x24*/
	__VOL uint32_t AHB1ENR;		/*RCC AHB1 peripheral clock enable register.		ADDRESS OFFSET = 0x30*/
	__VOL uint32_t AHB2ENR;		/*RCC AHB2 peripheral clock enable register.		ADDRESS OFFSET = 0x34*/
	__VOL uint32_t AHB3ENR;		/*RCC AHB3 peripheral clock enable register.		ADDRESS OFFSET = 0x38*/
	__VOL uint32_t Reserved3;	/*RCC Reserved register.							ADDRESS OFFSET = 0x3C*/
	__VOL uint32_t APB1ENR;		/*RCC APB1 peripheral clock enable register.		ADDRESS OFFSET = 0x40*/
	__VOL uint32_t APB2ENR;		/*RCC APB2 peripheral clock enable register.		ADDRESS OFFSET = 0x44*/
	__VOL uint32_t Reserved4;	/*RCC Reserved register.							ADDRESS OFFSET = 0x48*/
	__VOL uint32_t Reserved5;	/*RCC Reserved register.							ADDRESS OFFSET = 0x4C*/
	__VOL uint32_t AHB1LPENR;	/*RCC AHB1 peripheral clk en in low power mode.		ADDRESS OFFSET = 0x50*/
	__VOL uint32_t AHB2LPENR;	/*RCC AHB2 peripheral clk en in low power mode.		ADDRESS OFFSET = 0x54*/
	__VOL uint32_t AHB3LPENR;	/*RCC AHB3 peripheral clk en in low power mode.		ADDRESS OFFSET = 0x58*/
	__VOL uint32_t Reserved6;	/*RCC Reserved register.							ADDRESS OFFSET = 0x5C*/
	__VOL uint32_t APB1LPENR;	/*RCC APB1 peripheral clk en in low power mode.		ADDRESS OFFSET = 0x60*/
	__VOL uint32_t APB2LPENR;	/*RCC APB2 peripheral clk en in low power mode.		ADDRESS OFFSET = 0x64*/
	__VOL uint32_t Reserved7;	/*RCC Reserved register.							ADDRESS OFFSET = 0x68*/
	__VOL uint32_t Reserved8;	/*RCC Reserved register.							ADDRESS OFFSET = 0x6C*/
	__VOL uint32_t BDCR;		/*RCC Backup domain control register 	.			ADDRESS OFFSET = 0x70*/
	__VOL uint32_t CSR;			/*RCC clock control & status register.				ADDRESS OFFSET = 0x74*/
	__VOL uint32_t Reserved9;	/*RCC Reserved register.							ADDRESS OFFSET = 0x78*/
	__VOL uint32_t Reserved10;	/*RCC Reserved register.							ADDRESS OFFSET = 0x7C*/
	__VOL uint32_t SSCGR;		/*RCC spread spectrum clock generation register.	ADDRESS OFFSET = 0x80*/
	__VOL uint32_t PLLI2SCFGR;	/*RCC PLLI2S configuration register.				ADDRESS OFFSET = 0x84*/
	__VOL uint32_t PLLSAICFGR;	/* RCC PLL configuration register.					ADDRESS OFFSET = 0x88*/
	__VOL uint32_t DCKCFGR;		/*RCC Dedicated Clock Configuration Register.		ADDRESS OFFSET = 0x8C*/

}RCC_Reg;


typedef struct
{
	__VOL uint32_t IMR;		/* Interrupt mask register 								ADDRESS OFFSET = 0x00*/
	__VOL uint32_t EMR;		/* Event mask register 									ADDRESS OFFSET = 0x04*/
	__VOL uint32_t RTSR;	/* Rising trigger selection register 					ADDRESS OFFSET = 0x08*/
	__VOL uint32_t FTSR;	/* Falling trigger selection register 					ADDRESS OFFSET = 0x0C*/
	__VOL uint32_t SWIER;	/* Software interrupt event register 					ADDRESS OFFSET = 0x10*/
	__VOL uint32_t PR;		/* Pending register 									ADDRESS OFFSET = 0x14*/
}EXTI_Reg;


/*
 * System configuration controller Register(SYSCFG)
 */
typedef struct
{
	__VOL uint32_t MEMRMP;		/* SYSCFG memory remap register 						ADDRESS OFFSET = 0x00*/
	__VOL uint32_t PMC;			/* SYSCFG peripheral mode configuration register		ADDRESS OFFSET = 0x04*/
	__VOL uint32_t EXTICR1;		/* SYSCFG external interrupt configuration register 1	ADDRESS OFFSET = 0x08*/
	__VOL uint32_t EXTICR2;		/* SYSCFG external interrupt configuration register 2	ADDRESS OFFSET = 0x0C*/
	__VOL uint32_t EXTICR3;		/* SYSCFG external interrupt configuration register 3	ADDRESS OFFSET = 0x10*/
	__VOL uint32_t EXTICR4;		/* SYSCFG external interrupt configuration register 4	ADDRESS OFFSET = 0x14*/
	__VOL uint32_t Reserved0;	/* RCC Reserved register.							ADDRESS OFFSET = 0x18*/
	__VOL uint32_t Reserved1;	/* RCC Reserved register.							ADDRESS OFFSET = 0x1C*/
	__VOL uint32_t CMPCR;		/* Compensation cell control register					ADDRESS OFFSET = 0x20*/
	__VOL uint32_t Reserved2;	/* RCC Reserved register.							ADDRESS OFFSET = 0x24*/
	__VOL uint32_t Reserved3;	/* RCC Reserved register.							ADDRESS OFFSET = 0x28*/
}SYSCFG_Reg;


typedef struct
{
	__VOL uint32_t MODER;		/*REGISTER TO SET MODE OF GPIO PORT PIN.						ADDRESS OFFSET = 0x00*/
	__VOL uint32_t OTYPER;		/*REGISTER TO SET OUTPUT TYPE OF GPIO PORT PIN					ADDRESS OFFSET = 0x04*/
	__VOL uint32_t OSPEEDR;		/*REGISTER TO SET OUTPUT SPEED OF GPIO PORT PIN.				ADDRESS OFFSET = 0x08*/
	__VOL uint32_t PUPDR;		/*REGISTER TO SET PULL-UP/PULL-DOWN OF GPIO PORT PIN.			ADDRESS OFFSET = 0x0C*/
	__VOL uint32_t IDR;			/*REGISTER TO READ DATA OF GPIO PORT PIN.						ADDRESS OFFSET = 0x10*/
	__VOL uint32_t ODR;			/*REGISTER TO READ/WRITE DATA OF GPIO PORT PIN.					ADDRESS OFFSET = 0x14*/
	__VOL uint32_t BSRR;		/*REGISTER TO SET/RESET THE GPIO PORT PIN.						ADDRESS OFFSET = 0x18*/
	__VOL uint32_t LCKR;		/*REGISTER TO LOCK CONF. OF GPIO PORT PIN.						ADDRESS OFFSET = 0x1C*/
	__VOL uint32_t AFRL;		/*REGISTER TO SET ALT. FUNC. LOWER (0-7)BITS OF GPIO PORT PIN.	ADDRESS OFFSET = 0x20*/
	__VOL uint32_t AFRH;		/*REGISTER TO SET ALT. FUNC. HIGHER(8-15)BITS OF GPIO PORT PIN.	ADDRESS OFFSET = 0x24*/

}GPIO_Reg;


typedef struct
{
	__VOL uint32_t LISR;		/*DMA low interrupt status register					ADDRESS OFFSET = 0x00*/
	__VOL uint32_t HISR;		/*DMA high interrupt status register				ADDRESS OFFSET = 0x04*/
	__VOL uint32_t LIFCR;		/*DMA low interrupt flag clear register				ADDRESS OFFSET = 0x08*/
	__VOL uint32_t HIFCR;		/*DMA high interrupt flag clear register			ADDRESS OFFSET = 0x0C*/
	__VOL uint32_t SxCR;		/*DMA stream x configuration register				ADDRESS OFFSET = 0x10*/
	__VOL uint32_t SxNDTR;		/*DMA stream x number of data register				ADDRESS OFFSET = 0x14*/
	__VOL uint32_t SxPAR;		/*DMA stream x peripheral address register			ADDRESS OFFSET = 0x18*/
	__VOL uint32_t SxM0AR;		/*DMA stream x memory 0 address register			ADDRESS OFFSET = 0x1C*/
	__VOL uint32_t SxM1AR;		/*DMA stream x memory 1 address register			ADDRESS OFFSET = 0x20*/
	__VOL uint32_t SxFCR;		/*DMA stream x FIFO control register				ADDRESS OFFSET = 0x24*/

}DMA_Reg;


typedef struct
{
	__VOL uint32_t CR1;			/*SPI control register 1 (not used in I2S mode)		ADDRESS OFFSET = 0x00*/
	__VOL uint32_t CR2;			/*SPI control register 2							ADDRESS OFFSET = 0x04*/
	__VOL uint32_t SR;			/*SPI status register								ADDRESS OFFSET = 0x08*/
	__VOL uint32_t DR;			/*SPI data register									ADDRESS OFFSET = 0x0C*/
	__VOL uint32_t CRCPR;		/*SPI CRC polynomial register (not used in I2Smode)	ADDRESS OFFSET = 0x10*/
	__VOL uint32_t RXCRCR;		/*SPI RX CRC register (not used in I2S mode)		ADDRESS OFFSET = 0x14*/
	__VOL uint32_t TXCRCR;		/*SPI TX CRC register (not used in I2S mode)		ADDRESS OFFSET = 0x18*/
	__VOL uint32_t I2SCFGR;		/*SPI_I2S configuration register 					ADDRESS OFFSET = 0x1C*/
	__VOL uint32_t I2SPR;		/*SPI_I2S prescaler register 						ADDRESS OFFSET = 0x20*/

}SPI_Reg;


/*
 * PWR Definition (PWR Base Address Type Casted To PWR_Reg)
 */
#define PWR			((PWR_Reg*)PWR_BASE_ADDR)


/*
 * RCC Definition (RCC Base Address Type Casted To RCC_Reg)
 */
#define RCC			((RCC_Reg*)RCC_BASE_ADDR)


/*
 * Peripheral Definition (Peripheral Base Address Type Casted To GPIO_Reg )
 */
#define GPIOA		((GPIO_Reg*)GPIOA_BASE_ADDR)
#define GPIOB		((GPIO_Reg*)GPIOB_BASE_ADDR)
#define GPIOC		((GPIO_Reg*)GPIOC_BASE_ADDR)
#define GPIOD		((GPIO_Reg*)GPIOD_BASE_ADDR)
#define GPIOE		((GPIO_Reg*)GPIOE_BASE_ADDR)
#define GPIOF		((GPIO_Reg*)GPIOF_BASE_ADDR)
#define GPIOG		((GPIO_Reg*)GPIOG_BASE_ADDR)
#define GPIOH		((GPIO_Reg*)GPIOH_BASE_ADDR)
#define GPIOI		((GPIO_Reg*)GPIOI_BASE_ADDR)
#define GPIOJ		((GPIO_Reg*)GPIOJ_BASE_ADDR)
#define GPIOK		((GPIO_Reg*)GPIOK_BASE_ADDR)


/*
 * RCC Definition (RCC Base Address Type Casted To RCC_Reg)
 */
#define SYSCFG		((SYSCFG_Reg*)SYSCFG_BASE_ADDR)


/*
 * PWR Definition (PWR Base Address Type Casted To PWR_Reg)
 */
#define DMA			((DMA_Reg*)DMA_BASE_ADDR)


/*EXTI Definition (EXTI Base Address Type Casted To EXTI_Reg)
 *
 */
#define EXTI	((EXTI_Reg*)EXTI_BASE_ADDR)


/*
 *SPI Definition (SPI Base Address Type Casted To SPI_Reg)
 */
#define SPI1	((SPI_Reg*)SPI1_BASE_ADDR)
#define SPI2	((SPI_Reg*)SPI2_BASE_ADDR)
#define SPI3	((SPI_Reg*)SPI3_BASE_ADDR)


/*
 * Clock Enable Macro For GPIO Peripherals
 */

#define GPIOA_PCLK_EN()	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()	(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()	(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()	(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()	(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()	(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()	(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()	(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()	(RCC->AHB1ENR |= (1 << 8))


/*
 * Clock Enable Macro For I2Cx Peripherals
 */

#define I2C1_PCLK_EN()	(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()	(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()	(RCC->APB1ENR |= (1 << 23))


/*
 * Clock Enable Macro For SPIx Peripherals
 */

#define SPI1_PCLK_EN()	(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()	(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()	(RCC->APB1ENR |= (1 << 15))


/*
 * Clock Enable Macro For USARTx Peripherals
 */

#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1 << 18))
#define USART4_PCLK_EN()	(RCC->APB1ENR |= (1 << 19))
#define USART5_PCLK_EN()	(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1 << 5))


/*
 * Clock Enable For SYSCFG Peripheral
 */

#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))


/*
 * Clock Disable Macro For GPIO Peripherals
 */

#define GPIOA_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 8))


/*
 * Clock Disable Macro For I2Cx Peripherals
 */

#define I2C1_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 23))


/*
 * Clock Disable Macro For SPIx Peripherals
 */

#define SPI1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 15))


/*
 * Clock Disable Macro For USARTx Peripherals
 */

#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 18))
#define USART4_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 19))
#define USART5_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 5))


/*
 * Clock Disable For SYSCFG Peripheral
 */

#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 14))


/*
 * GPIOx Reset Macros
 */
#define GPIOA_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));} while(0)
#define GPIOB_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));} while(0)
#define GPIOC_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));} while(0)
#define GPIOD_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));} while(0)
#define GPIOE_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));} while(0)
#define GPIOF_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));} while(0)
#define GPIOG_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));} while(0)
#define GPIOH_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));} while(0)
#define GPIOI_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8));} while(0)


/*
 * IRO (INTERUPT REQUEST NUMBER) For STM32F407x MCU
 * Update These Macros According To Your Micro-controller
 */
#define IRO_No_EXTI0	6
#define IRO_No_EXTI1	7
#define IRO_No_EXTI2	8
#define IRO_No_EXTI3	9
#define IRO_No_EXTI4	10
#define IRO_No_EXTI9_5	23
#define IRO_No_EXTI15_10	40


/*
 * Macros of all the possible levels
 */
#define NVIC_IRQ_PRI0	0
#define NVIC_IRQ_PRI15	15


/*
 * Generic Maros
 */
#define ENABLE				1
#define DISABLE				0
#define SET 				ENABLE
#define RESET 				DISABLE


/*
 * BIT POSITION DEFINITION FOR SPI PERIPHERALS
 */
// For CR1 Register
#define SPI_CR1_CPHA		0x0
#define SPI_CR1_CPOL		0x1
#define SPI_CR1_MSTR		0x2
#define SPI_CR1_BR			0x3
#define SPI_CR1_SPE			0x6
#define SPI_CR1_LSB_FIRST	0x7
#define SPI_CR1_SSI			0x8
#define SPI_CR1_SSM			0x9
#define SPI_CR1_RX_ONLY		0xA
#define SPI_CR1_DFF			0xB
#define SPI_CR1_CRC_NEXT	0xC
#define SPI_CR1_CRC_EN		0xD
#define SPI_CR1_BIDI_OE		0xE
#define SPI_CR1_BIDI_MODE	0xF

//For CR2 Register
#define SPI_CR2_RXDMAEN		0x0
#define SPI_CR2_TXDMAEN		0x1
#define SPI_CR2_SSOE		0x2
#define SPI_CR2_FRF			0x4
#define SPI_CR2_ERRIE		0x5
#define SPI_CR2_RXNEIE		0x6
#define SPI_CR2_TXEIE		0x7

//For SR Register
#define SPI_SR_RXNE			0x0
#define SPI_SR_TXE			0x1
#define SPI_SR_CHSIDE		0x2
#define SPI_SR_UDR			0x3
#define SPI_SR_CRC_ERR		0x4
#define SPI_SR_MODF			0x5
#define SPI_SR_OVR			0x6
#define SPI_SR_BSY			0x7
#define SPI_SR_FRE			0x8



#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

#endif /* INC_STM32F407XX_H_ */
