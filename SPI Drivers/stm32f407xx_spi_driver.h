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
	SPI_Reg			*pSPIx;
	SPI_Config		SPIConfig;

	uint8_t 		*pTxBuffer;		//Stores Tx Buffer Address
	uint8_t 		*pRxBuffer;		//Stores Rx Buffer Address
	uint32_t		TxLen;			//Stores Rx Length
	uint32_t		RxLen;			//Stores Tx Length
	uint8_t			TxState;		//Stores Tx State
	uint8_t			RxState;		//Stores Rx State
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
#define SPI_BUS_CNFG_SIMPLEX_RxONLY	0x3
//#define SPI_BUS_CNFG_SIMPLEX_TxONLY	0x4

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
#define SPI_SSM_DI	0x0
#define SPI_SSM_EN	0x1


/*
 * SPI Init & DeInit
 */
void SPI_Init(SPI_Handle *pSPI_Handle);
void SPI_DeInit(SPI_Reg *pSPIx);


/*
 * SPI Clock Setup & Peripheral Control
 */
void SPI_PCLK_Control(SPI_Reg *pSPIx, uint8_t ENorDI);
void SPI_PControl(SPI_Reg *pSPIx, uint8_t ENorDI);

/*
 * Send And Receive Data (Without Interrupt)
 */
void SPI_Send_Data(SPI_Reg *pSPIx, uint8_t *pTx_Buffer, uint32_t Len);
void SPI_Receive_Data(SPI_Reg *pSPIx, uint8_t *pRx_Buffer, uint32_t Len);

/*
 *	Send And Receive Data (With Interrupt)
 */
uint8_t SPI_Send_Data_IT(SPI_Handle *pSPI_Handle, uint8_t *pTx_Buffer, uint32_t Len);
uint8_t SPI_Receive_Data_IT(SPI_Handle *pSPI_Handle, uint8_t *pRx_Buffer, uint32_t Len);

/*
 * SPI Opening And Closing Transmission
 */
void SPI_CLOSE_RECEPTION(SPI_Handle *pSPI_Handle);
void SPI_CLOSE_TRANSMISSION(SPI_Handle *pSPI_Handle);


/*
 * SPI IRQ Configuration and ISR Handling
 */
void SPI_IRQ_Config(uint8_t IRQNumber, uint8_t ENorDI);
void SPI_IRQ_PriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQ_Handling(SPI_Handle *pHandle);


/*
 * Other Peripheral And Control APIs
 */
uint8_t Get_SPI_Flag_Status(SPI_Reg *pSPIx, uint32_t FlagName);
void SPI_SSOE_Config(SPI_Reg *pSPIx, uint8_t ENorDI);
void SPI_CLR_OVR_FLAG(SPI_Reg *pSPIx);

/*
 * SPI FLAG STATUS DEFITIONS
 * (PROVIDES  MASKING INFO OF VARIOUS FLAGS IN SR REGISTER)
 */
#define SPI_RXNE_FLAG 		(1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG		(1 << SPI_SR_TXE)
#define SPI_CHSIDE_FLAG		(1 << SPI_SR_CHSIDE)
#define SPI_UDR_FLAG		(1 << SPI_SR_UDR)
#define SPI_CRC_ERR_FLAG	(1 << SPI_SR_CRC_ERR)
#define SPI_MODF_FLAG		(1 << SPI_SR_MODF)
#define SPI_OVR_FLAG		(1 << SPI_SR_OVR)
#define SPI_BSY_FLAG		(1 << SPI_SR_BSY)
#define SPI_FRE_FLAG		(1 << SPI_SR_FRE)

/*
 * Possible SPI Application State
 */
#define SPI_READY			0x1
#define SPI_BUSY_IN_RX		0x2
#define SPI_BUSY_IN_TX		0x3


/*
 * Possible SPI APPLICATION EVENTS
 */
#define SPI_TX_EVNT_CMPLT		0x1
#define SPI_RX_EVNT_CMPLT		0x2
#define SPI_OVR_EVNT_ERROR		0x3

/*
 * SPI APPLICATION CALLBACK
 */
void SPI_APP_EVNT_CALLBACK(SPI_Handle *pHandle,uint8_t SPI_EVNT);


#endif /* STM32F407XX_SPI_DRIVER_H_ */
