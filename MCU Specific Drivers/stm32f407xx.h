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

#define EXT1_BASE_ADDR				(APB2_PERIPH_BASE + 0x3C00) /*EXT1 OFFSET = 0x3C00*/
#define SYSCFG_BASE_ADDR			(APB2_PERIPH_BASE + 0x3800) /*SYSCFG OFFSET = 0x3800*/


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



/*
 * System configuration controller Register(SYSCFG)
 */
typedef struct
{
	__VOL uint32_t MEMRMP;		/*SYSCFG memory remap register.						ADDRESS OFFSET = 0x00*/
	__VOL uint32_t PMC;			/*SYSCFG peripheral mode conf. register				ADDRESS OFFSET = 0x04*/
	__VOL uint32_t EXTICR1;		/*SYSCFG external interrupt conf. register 1		ADDRESS OFFSET = 0x08*/
	__VOL uint32_t EXTICR2;		/*SYSCFG external interrupt conf. register 2		ADDRESS OFFSET = 0x0C*/
	__VOL uint32_t EXTICR3;		/*SYSCFG external interrupt conf. register 3		ADDRESS OFFSET = 0x10*/
	__VOL uint32_t EXTICR4;		/*SYSCFG external interrupt conf. register 4		ADDRESS OFFSET = 0x14*/
	__VOL uint32_t CMPCR;		/*Compensation cell control register				ADDRESS OFFSET = 0x20*/

}SYSCFG_Reg;


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
#define SPI3_PCLK_Di()	(RCC->APB1ENR &= ~(1 << 15))


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
 * Generic Maros
 */
#define ENABLE				0
#define DISABLE				1
#define SET 				ENABLE
#define RESET 				DISABLE


#endif /* INC_STM32F407XX_H_ */
