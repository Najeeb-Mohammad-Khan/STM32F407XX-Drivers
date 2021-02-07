/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: 26-Dec-2020
 *      Author: Najeeb Mohammad Khan
 */


#include "stm32f407xx_spi_driver.h"



/*
 * GPIO Clock Setup
 */
/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	GPIO_PCLK_Control
 * @Brief		-	This function Enables or disables the GPIO peripheral clock
 *
 * @Input Parameters:
 * 1. GPIO_Reg *pGPIOx
 * 				- Base address of the GPIO peripheral
 * 2. ENorDI	- ENABLE or DISABLE
 *
 * @Return		-	None
 *
 * @Note		-	None
 *
 *************************************************************************/
void SPI_PCLK_Control(SPI_Reg *pSPIx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}

		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}

		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}

		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}

		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}
}
