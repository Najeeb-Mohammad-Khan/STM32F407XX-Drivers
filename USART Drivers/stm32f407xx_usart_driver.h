/*
 * stm32f407xx_usart_driver.h
 *
 *  Created on: 07-Mar-2021
 *      Author: Najeeb Mohammad Khan
 */

#ifndef STM32F407XX_USART_DRIVER_H_
#define STM32F407XX_USART_DRIVER_H_

#include <stm32f407xx.h>

/*
 * USARTx Configuring Peripheral Structure
 */
typedef struct
{
	uint8_t USART_MODE;
	uint32_t USART_BAUD_RATE;
	uint8_t USART_NO_STOP_BITS;
	uint8_t USART_WORD_LENGTH;
	uint8_t USART_PARITY_CONTROL;
	uint8_t USART_HW_FLOW_CONTROL;
}USART_Config;


/*
 * I2Cx Peripheral Handle Structure
 */
typedef struct
{
	USART_Reg *pUSARTx;
	USART_Config USART_Config;

	uint8_t 	*pTxBuffer;		/* !< To store App. Tx Buffer Address> */
	uint8_t 	*pRxBuffer;		/* !< To store App. Rx Buffer Address> */
	uint8_t		RxBusyState;	/* !< To store USART Receiver State > */
	uint8_t		TxBusyState;	/* !< To store USART Transmitter State > */
	uint32_t 	TxLen;			/* !< To store Tx Length > */
	uint32_t 	RxLen;			/* !< To store Rx Length > */
}USART_Handle;


/*
 * Generic USART Peripheral Macros
 */
//USART MODE (@USART_MODE)
#define USART_MODE_Tx	0x1
#define USART_MODE_Rx	0x2
#define USART_MODE_TxRx	0x3

//USART BAUD RATE (@USART_BAUD_RATE)
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					2400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000

//USART NUMBER OF STOP BITS (@USART_NO_STOP_BITS)
#define USART_STOP_BITS_1		0x0
#define USART_STOP_BITS_0_5		0x1
#define USART_STOP_BITS_2		0x2
#define USART_STOP_BITS_1_5		0x3

//USART WORD LENGTH (@USART_WORD_LENGTH)
#define USART_WORD_LENGTH_8BITS		0x1
#define USART_WORD_LENGTH_9BITS		0x2

//USART PARITY CONTROL (@USART_PARITY_CONTROL)
#define USART_PARITY_EN_EVEN  0x1
#define USART_PARITY_EN_ODD	  0x2
#define USART_PARITY_DI		  0x3

//USART HW FLOW CONTROL (@USART_HW_FLOW_CONTROL)
#define USART_HW_FLOW_CTRL_NONE    	0x1
#define USART_HW_FLOW_CTRL_CTS    	0x2
#define USART_HW_FLOW_CTRL_RTS    	0x3
#define USART_HW_FLOW_CTRL_CTS_RTS	0x4

/*
 * USART Init & DeInit
 */
void USART_Init(USART_Handle *pUSART_Handle);
void USART_DeInit(USART_Reg *pUSARTx);

/*
 * USART Clock Setup & Peripheral Control
 */
void USART_PCLK_Control(USART_Reg *pUSARTx, uint8_t ENorDI);
void USART_PControl(USART_Reg *pUSARTx, uint8_t ENorDI);

/*
 * USART Data Send and Receive Data (Without Interrupt)
 */
void USART_Send_Data(USART_Handle *pUSART_Handle, uint8_t *pTxBuffer, uint32_t Len);
void USART_Receive_Data(USART_Handle *pUSART_Handle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * USART Data Send and Receive Data (With Interrupt)
 */
uint8_t USART_Send_Data_IT(USART_Handle *pUSART_Handle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_Receive_Data_IT(USART_Handle *pUSART_Handle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR Handling
 */
void USART_IRQ_INTERRUPT_CONFG(uint8_t IRQNumber, uint8_t ENorDI);
void USART_IRQ_PRIORITY_CONFG(uint8_t IRQNumber, uint8_t IRQPriority);
void USART_IRQ_Handling(USART_Handle *pUSART_Handle);

/*
 * Other Peripheral APIs
 */
uint8_t Get_USART_Flag_Status(USART_Reg *pUSARTx, uint32_t FlagName);
void USART_CLR_FLAG(USART_Reg *pUSARTx, uint32_t FlagName);
void USART_Set_Baud_Rate(USART_Reg *pUSARTx, uint32_t BaudRate);

/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle *pUSART_Handle,uint8_t AppEv);

/*
 * USART FLAG STATUS DEFITIONS
 * (PROVIDES  MASKING INFO OF VARIOUS FLAGS IN SR REGISTERs)
 */
#define USART_PE_FLAG 		(1 << USART_SR_PE)
#define USART_FE_FLAG 		(1 << USART_SR_FE)
#define USART_NF_FLAG 		(1 << USART_SR_NF)
#define USART_ORE_FLAG 		(1 << USART_SR_ORE)
#define USART_IDLE_FLAG 	(1 << USART_SR_IDLE)
#define USART_RXNE_FLAG 	(1 << USART_SR_RXNE)
#define USART_TC_FLAG 		(1 << USART_SR_TC)
#define USART_TXE_FLAG 		(1 << USART_SR_TXE)
#define USART_LBD_FLAG 		(1 << USART_SR_LBD)
#define USART_CTS_FLAG 		(1 << USART_SR_CTS)


/*
 * USART Application State
 */
#define USART_READY				0x0
#define USART_BUSY_IN_RX		0x1
#define USART_BUSY_IN_TX		0x2

/*
 * Possible USART APPLICATION EVENTS
 */
#define SPI_TX_EVNT_CMPLT		0x1
#define SPI_RX_EVNT_CMPLT		0x2
#define SPI_OVR_EVNT_ERROR		0x3

/*
 * Possible SPI APPLICATION EVENTS
 */
#define USART_EVENT_TX_CMPLT		0x1
#define USART_EVENT_RX_CMPLT		0x2
#define USART_EVENT_CTS				0x3
#define USART_EVENT_IDLE			0x4
#define USART_EVENT_ORE				0x5
#define USART_ERREVENT_FE			0x6
#define USART_ERREVENT_NF			0x7
#define USART_ERREVENT_ORE			0x8

#endif /* STM32F407XX_USART_DRIVER_H_ */
