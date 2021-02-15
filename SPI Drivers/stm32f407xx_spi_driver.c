/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: 26-Dec-2020
 *      Author: Najeeb Mohammad Khan
 */


#include "stm32f407xx_spi_driver.h"



/*
 * SPI Clock Setup
 */
/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	SPI_PCLK_Control
 * @Brief		-	This function Enables or disables the SPI peripheral clock
 *
 * @Input Parameters:
 * 1. SPI_Reg *pSPIx
 * 				- Base address of the SPI peripheral
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


/*
 * SPI Peripheral Control
 */
/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	SPI_PControl
 * @Brief		-	This function Enables or disables the SPI peripheral
 *
 * @Input Parameters:
 * 1. SPI_Reg *pSPIx
 * 				- Base address of the SPI peripheral
 * 2. ENorDI	- ENABLE or DISABLE
 *
 * @Return		-	None
 *
 * @Note		-	None
 *
 *************************************************************************/
void SPI_PControl(SPI_Reg *pSPIx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
	{
		if((!(Get_Flag_Status(pSPIx, SPI_CR1_SSI))) & (Get_Flag_Status(pSPIx, SPI_CR1_SSM)))
		{
			pSPIx->CR1 |= (1 << SPI_CR1_SSI); //Setting SSI to prevent MODF Bit triggering and Resetting Master Bit in CR1.
		}

		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}


/*
 * SPI Init & DeInit
 */

/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	SPI_Init
 * @Brief		-	This function initializes the SPI Port
 *
 * @Input Parameters:
 * 1. SPI_Handle *pSPI_Handle
 * 				- Base address of the SPI peripheral
 *
 * @Return		-	None
 *
 * @Note		-	None
 *
 *************************************************************************/
void SPI_Init(SPI_Handle *pSPI_Handle)
{
	uint32_t tempreg = 0; //Temp. register to store the values to be stored in the CR1 register

	//Enabling the SPI LOCK
	SPI_PCLK_Control(pSPI_Handle->pSPIx, ENABLE);

	//Setting the Device in Master Mode
	tempreg |= pSPI_Handle->SPIConfig.SPI_DeviceMode << 0x2;

	if(pSPI_Handle->SPIConfig.SPI_BusConfig == SPI_BUS_CNFG_FD)
	{
		//Clearing the BIDI MODE BIT
		tempreg &= ~(1 << 0xF);
	}

	else if(pSPI_Handle->SPIConfig.SPI_BusConfig == SPI_BUS_CNFG_HD)
	{
		//Setting the BIDI MODE BIT
		tempreg |= (1 << SPI_CR1_BIDI_MODE);
	}

	else if(pSPI_Handle->SPIConfig.SPI_BusConfig == SPI_BUS_CNFG_SIMPLEX_RxONLY)
	{
		//Clearing the BIDI MODE BIT
		tempreg &= ~(1 << SPI_CR1_BIDI_MODE);

		//Setting the RXONLY BIT
		tempreg |= (1 << SPI_CR1_RX_ONLY);
	}

	//Configuring the SPI CLOCK
	tempreg |= (pSPI_Handle->SPIConfig.SPI_Sclk_Speed << SPI_CR1_BR);

	//Configuring the Data Frame Format (DFF) BIT
	tempreg |= (pSPI_Handle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	//Configuring the CLOCK POLARITY (CPOL) BIT
	tempreg |= (pSPI_Handle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	//Configuring the CLOCK PHSAE (CPHA) BIT
	tempreg |= (pSPI_Handle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	//Configuring the Software Slave Management(SSM) BIT
	tempreg |= (pSPI_Handle->SPIConfig.SPI_SSM << SPI_CR1_SSM);

	//Deactivating (if Activated) The SPI Peripheral Before Changing Control Register
	SPI_PControl(pSPI_Handle->pSPIx, DISABLE);

	//Transfer the Value of tempreg into CR1 Register
	pSPI_Handle->pSPIx->CR1 = tempreg;

}


/*
 * SPI FLAG STATUS
 */

/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	SPI_Init
 * @Brief		-	This function returns the status of the SPI Flags
 *
 * @Input Parameters:
 * 1. SPI_Reg*pSPIx
 * 				- Base address of the SPI peripheral
 * 2. FlagName	- Name of the flag for which you want to get status.
 * @Return		-	uint8_t 	(ENABLE OR DISABLE)
 *
 * @Note		-	None
 *
 *************************************************************************/
uint8_t Get_Flag_Status(SPI_Reg *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return SET;
	}

	return RESET;
}


/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	SPI_SSOE_Config
 * @Brief		-	This function Configures the SSOE Register Bit of CR2 SPI Register
 *
 * @Input Parameters:
 * 1. SPI_Reg*pSPIx
 * 				- Base address of the SPI peripheral
 * 2. ENorDI	- ENABLE or DISABLE
 * @Return		-	None
 *
 * @Note		-	None
 *
 *************************************************************************/
void SPI_SSOE_Config(SPI_Reg *pSPIx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}

	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}



/*
 * SPI Send Data
 */

/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	SPI_SendData
 * @Brief		-	This function Sends Data on SPI Port
 *
 * @Input Parameters:
 * 1. SPI_Reg *pSPIx
 * 				- Base address of the SPI peripheral
 * 2. uint8_t *pTxBuffer
 * 				- Transmit Buffer Variable
 * 3. uint32_t Len
 * 				- Length of the Data Word
 * @Return		-	None
 *
 * @Note		-	This is BLOCKING CALL
 *
 *************************************************************************/
void SPI_Send_Data(SPI_Reg *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	//ENABLING SSOE FOR SINGLE MATER CONFIGURATION (MSTR = 1 & SSM = 0)
	if(Get_Flag_Status(pSPIx, SPI_CR1_MSTR) & !(Get_Flag_Status(pSPIx, SPI_CR1_SSM)))
	{
		SPI_SSOE_Config(pSPIx, ENABLE);
	}

	//Activating The SPI Peripheral Before Changing the Control Register
	SPI_PControl(pSPIx, ENABLE);

	while(Len > 0)
	{
		//Wait until TXE is Set ie The TX Buffer is Empty
		while(Get_Flag_Status(pSPIx, SPI_TXE_FLAG) == RESET);

		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			// 16 BIT DATA FRAME FORMAT
			//Loading data into DR register
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len -= 2;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			// 8 BIT DATA FRAME FORMAT
			//Loading data into DR register
			pSPIx->DR = *(pTxBuffer);
			Len --;
			pTxBuffer++;
		}
	}

	//Waiting For The SPI To Conplete Its Data Sending Task
	while(!Get_Flag_Status(pSPIx, SPI_SR_BSY));

	//Deactivating The SPI Peripheral After Changing the Control Register
	SPI_PControl(pSPIx, DISABLE);

}


/*
 * SPI Receive Data
 */

/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	SPI_ReceiveData
 * @Brief		-	This function Receives Data on SPI Port
 *
 * @Input Parameters:
 * 1. SPI_Reg *pSPIx
 * 				- Base address of the SPI peripheral
 * 2. uint8_t *pRxBuffer
 * 				- Receive Buffer Variable
 * 3. uint32_t Len
 * 				- Length of the Data Word
 * @Return		-	None
 *
 * @Note		-	This is BLOCKING CALL
 *
 *************************************************************************/
void SPI_Send_Data(SPI_Reg *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{

}
