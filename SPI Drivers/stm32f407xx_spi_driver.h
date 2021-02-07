/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: 26-Dec-2020
 *      Author: Najeeb Mohammad Khan
 */

#ifndef STM32F407XX_SPI_DRIVER_H_
#define STM32F407XX_SPI_DRIVER_H_


#include "stm32f407xx.h"

/*
 * SPIx Configuring Peripheral Structure
 */
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_Sclk_Speed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config;


/*
 * SPIx Peripheral Handle Structure
 */
typedef struct
{
	SPI_Reg		*pSPIx;
	SPI_Config		SPIConfig;
}SPI_Handle;


/*
 * Generic SPI REGESTER Micros
 */
//SPI PIN MODE { @SPI_DeviceMode }
#define SPI_MASTER	0x1
#define SPI_SLAVE	0x0

//SPI BUS CONFIG {@SPI_BusConfig}
#define SPI_BUS_CNFG_FD				0x1
#define SPI_BUS_CNFG_HD				0x2
#define SPI_BUS_CNFG_SIMPLEX_TxONLY	0x3
#define SPI_BUS_CNFG_SIMPLEX_RxONLY	0x4

//SPI CLOCK SPEED { @SPI_Sclk_Speed }
#define SPI_SCLK_SPEED_DIV2		0x0
#define SPI_SCLK_SPEED_DIV4		0x1
#define SPI_SCLK_SPEED_DIV8		0x2
#define SPI_SCLK_SPEED_DIV16	0x3
#define SPI_SCLK_SPEED_DIV32	0x4
#define SPI_SCLK_SPEED_DIV64	0x5
#define SPI_SCLK_SPEED_DIV128	0x6
#define SPI_SCLK_SPEED_DIV256	0x7

//SPI DATA FRAME FORMAT { @SPI_DFF }
#define SPI_DFF_8_BITS	0x0
#define SPI_DFF_16_BITS	0x1

//SPI CLOCK POLARITY { @SPI_CPOL}
#define SPI_CPOL_LOW	0x0
#define SPI_CPOL_HIGH	0x1

//SPI CLOCK PHASE { @SPI_CPHA }
#define SPI_CPHA_LOW	0x0
#define SPI_CPHA_HIGH	0x1

//SPI SOFTWARE SLAVE MANAGEMENT { @SPI_SSM }
#define SPI_SSM_SW	0x0
#define SPI_SSM_HW	0x1


/*
 * SPI Init & DeInit
 */
void SPI_Init(SPI_Handle *pSPI_Handle);
void SPI_DeInit(SPI_Reg *pSPIx);


/*
 * SPI Clock Setup
 */
void SPI_PCLK_Control(SPI_Reg *pSPIx, uint8_t ENorDI);


/*
 * Data Send And Receive
 */
void SPI_Send_Data(SPI_Reg *pSPIx, uint8_t *pTx_Buffer, uint32_t Len);
void SPI_Receive_Data(SPI_Reg *pSPIx, uint8_t *pRx_Buffer, uint32_t Len);


/*
 * GPIO IRQ Configuration and ISR Handling
 */
void SPI_IRQ_Config(uint8_t IRQNumber, uint8_t ENorDI);
void SPI_IRQ_PriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQ_Handling(SPI_Handle *pHandle);



#endif /* STM32F407XX_SPI_DRIVER_H_ */
