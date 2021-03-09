/*
 * stm32f407_usart_driver.c
 *
 *  Created on: 07-Mar-2021
 *      Author: Najeeb Mohammad Khan
 */

#include "stm32f407xx_usart_driver.h"

static void USART_CLR_ORE_FLAG(USART_Reg *pUSARTx);


static void USART_CLR_ORE_FLAG(USART_Reg *pUSARTx)
{
	uint32_t dummy_read;
	dummy_read = pUSARTx->SR;
	dummy_read = pUSARTx->DR;
	(void)dummy_read;
}


/*
 * USART Clock Setup
 */
/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	USART_PCLK_Control
 * @Brief		-	This function Enables or disables the USART peripheral clock
 *
 * @Input Parameters:
 * 1. USART_Reg *pUSARTx
 * 				- Base address of the USART peripheral
 * 2. ENorDI	- ENABLE or DISABLE
 *
 * @Return		-	None
 *
 * @Note		-	None
 *
 *************************************************************************/
void USART_PCLK_Control(USART_Reg *pUSARTx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}

		else if(pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}

		else if(pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}

		else if(pUSARTx == UART4)
		{
			USART4_PCLK_EN();
		}

		else if(pUSARTx == UART5)
		{
			USART5_PCLK_EN();
		}

		else if(pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		}
	}
	else
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}

		else if(pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}

		else if(pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		}

		else if(pUSARTx == UART4)
		{
			USART4_PCLK_DI();
		}

		else if(pUSARTx == UART5)
		{
			USART5_PCLK_DI();
		}

		else if(pUSARTx == USART6)
		{
			USART6_PCLK_DI();
		}
	}
}


/*
 * USART Peripheral Control
 */
/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	USART_PControl
 * @Brief		-	This function Enables or disables the USART peripheral
 *
 * @Input Parameters:
 * 1. USART_Reg *pUSARTx
 * 				- Base address of the USART peripheral
 * 2. ENorDI	- ENABLE or DISABLE
 *
 * @Return		-	None
 *
 * @Note		-	None
 *
 *************************************************************************/
void USART_PControl(USART_Reg *pUSARTx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
	{
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	}
	else
	{
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}


/*
 * USART FLAG STATUS
 */

/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	Get_USART_Flag_Status
 * @Brief		-	This function returns the status of the USART Flags
 *
 * @Input Parameters:
 * 1. USART_Reg *pUSARTx
 * 				- Base address of the USART peripheral
 * 2. FlagName	- Name of the flag for which you want to get status.
 * @Return		-	uint8_t 	(SET OR RESET)
 *
 * @Note		-	None
 *
 *************************************************************************/
uint8_t Get_USART_Flag_Status(USART_Reg *pUSARTx, uint32_t FlagName)
{
	if(pUSARTx->SR & FlagName)
	{
		return SET;
	}

	return RESET;
}

/*
 * USART FLAG CLEAR
 */

/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	USART_CLR_FLAG
 * @Brief		-	This function CLEARS the USART Flag
 *
 * @Input Parameters:
 * 1. USART_Reg *pUSARTx
 * 				- Base address of the USART peripheral
 * 2. FlagName	- Name of the flag you want to CLEAR
 * @Return		-	None
 *
 * @Note		-	None
 *
 *************************************************************************/
void USART_CLR_FLAG(USART_Reg *pUSARTx, uint32_t FlagName)
{
	pUSARTx->SR &= ~(FlagName);
}


/*
 * USART IRQ Configuration and ISR Handling
 */
/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	USART_IRQ_INTERRUPT_CONFG
 * @Brief		-	This function (ENABLE / DISABLE) the IRQ
 *
 * @Input Parameters:
 * 1. IRQNumber		- IRQ number of the Interrupt
 * 2. ENorDI		- ENABLE or DISABLE
 *
 * @Return		-	None
 *
 * @Note		-	None
 *
 *************************************************************************/
void USART_IRQ_INTERRUPT_CONFG(uint8_t IRQNumber, uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
	{
		//Configuring required ISER Register To ENABLE THE INTERRUPT.
		if(IRQNumber < 32)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}

		else if(IRQNumber >= 32 && IRQNumber < 64)
		{
			*NVIC_ISER1 |= (1 << IRQNumber % 32);
		}

		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ISER2 |= (1 << IRQNumber % 32);
		}
	}

	else
	{
		//Configuring Required ICER Register To DISABLE THE INTERRUPT.
		if(IRQNumber < 32)
		{
			*NVIC_ICER0 |= (1 << IRQNumber);
		}

		else if(IRQNumber >= 32 && IRQNumber < 64)
		{
			*NVIC_ICER1 |= (1 << IRQNumber % 32);
		}

		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ICER2 |= (1 << IRQNumber % 32);
		}

	}

}


/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	USART_IRQ_PRIORITY_CONFG
 * @Brief		-	This function configures the Priority of IRQ
 *
 * @Input Parameters:
 * 1. IRQNumber		- IRQ number of the Interrupt
 * 2. IRQPriority	- Interrupt priority
 *
 * @Return		-	None
 *
 * @Note		-	None
 *
 *************************************************************************/
void USART_IRQ_PRIORITY_CONFG(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//Configuring required IPR Register To SET THE INTERRUPT.
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t PR_Bits_Shift = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASEADDR + (iprx * 4)) |= (IRQPriority << PR_Bits_Shift);

}

/*
 * USART Init & DeInit
 */
/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	USART_Init
 * @Brief		-	This function initializes the USART Port
 *
 * @Input Parameters:
 * 1. USART_Handle *pUSART_Handle
 * 				- Base address of the USART peripheral
 *
 * @Return		-	None
 *
 * @Note		-	None
 *
 *************************************************************************/
void USART_Init(USART_Handle *pUSART_Handle)
{
	//Temporary Variable Register
	uint32_t tempreg=0;

	//**************  CONFIGURING THE CR1 REGISTER   **************//
	//Enabling the CLOCK for the USART peripheral
	USART_PCLK_Control(pUSART_Handle->pUSARTx, ENABLE);

	//Enabling USART Tx and Rx engines according to the USART_Mode configuration item
	if (pUSART_Handle->USART_Config.USART_MODE == USART_MODE_Rx)
	{
		//Enabling the Receiver bit field
		tempreg|= (1 << USART_CR1_RE);

	}

	else if (pUSART_Handle->USART_Config.USART_MODE == USART_MODE_Tx)
	{
		//Enabling the Transmitter bit field
		tempreg |= ( 1 << USART_CR1_TE );

	}

	else if (pUSART_Handle->USART_Config.USART_MODE == USART_MODE_TxRx)
	{
		//Enabling both the Transmitter and Receiver bit fields
		tempreg |= (( 1 << USART_CR1_TE) | ( 1 << USART_CR1_RE));
	}

    //Configuring the Word length configuration item
	tempreg |= pUSART_Handle->USART_Config.USART_WORD_LENGTH << USART_CR1_M;


    //Configuring parity control bit fields
	if ( pUSART_Handle->USART_Config.USART_PARITY_CONTROL == USART_PARITY_EN_EVEN)
	{
		//Enabling the parity control
		tempreg |= ( 1 << USART_CR1_PCE);

		//Enabling EVEN parity (When PS Bit is 0)
		tempreg &= ~(1 << USART_CR1_PS);

	}

	else if (pUSART_Handle->USART_Config.USART_PARITY_CONTROL == USART_PARITY_EN_ODD )
	{
		//Enabling the parity control
	    tempreg |= ( 1 << USART_CR1_PCE);

	    //Enabling ODD parity (When PS Bit is 1)
	    tempreg |= ( 1 << USART_CR1_PS);

	}

   //Programming the CR1 register
	pUSART_Handle->pUSARTx->CR1 = tempreg;


	//**************  CONFIGURING THE CR1 REGISTER   **************//
	//Reset the Temporary Variable Register
	tempreg = 0;

	//Configuring the number of stop bits inserted during USART frame transmission
	tempreg |= pUSART_Handle->USART_Config.USART_NO_STOP_BITS << USART_CR2_STOP;

	//Programming the CR2 register
	pUSART_Handle->pUSARTx->CR2 = tempreg;


	//**************  CONFIGURING THE CR3 REGISTER   **************//
	//RESET the Temporary Variable Register
	tempreg = 0;

	//Configuration of USART hardware flow control
	if ( pUSART_Handle->USART_Config.USART_HW_FLOW_CONTROL == USART_HW_FLOW_CTRL_CTS)
	{
		//Enabling the CTS flow control Bit
		tempreg |= (1 << USART_CR3_CTSE);


	}

	else if (pUSART_Handle->USART_Config.USART_HW_FLOW_CONTROL == USART_HW_FLOW_CTRL_RTS)
	{
		//Enabling RTS flow control Bit
		tempreg |= (1 << USART_CR3_RTSE);

	}

	else if (pUSART_Handle->USART_Config.USART_HW_FLOW_CONTROL == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		//Enabling both CTS and RTS Flow control
		tempreg |= ((1 << USART_CR3_CTSE) | (1 << USART_CR3_RTSE));
	}


	pUSART_Handle->pUSARTx->CR3 = tempreg;

/******************************** Configuration of BRR(Baudrate register)******************************************/

	//Configuring the baud rate
	USART_Set_Baud_Rate(pUSART_Handle->pUSARTx, pUSART_Handle->USART_Config.USART_BAUD_RATE);

	//Activating USART Peripheral
	USART_PControl(pUSART_Handle->pUSARTx, ENABLE);
}

/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	USART_DeInit
 * @Brief		-	This function de-initializes the USART Port
 *
 * @Input Parameters:
 * 1. USART_Handle *pUSART_Handle
 * 				- Base address of the USART peripheral
 *
 * @Return		-	None
 *
 * @Note		-	None
 *
 *************************************************************************/
void USART_DeInit(USART_Reg *pUSARTx)
{
	if(pUSARTx == USART1)
	{
		USART1_REG_RESET();		//RESET USART1
	}

	else if(pUSARTx == USART2)
	{
		USART2_REG_RESET();		//RESET USART2
	}

	else if(pUSARTx == USART3)
	{
		USART3_REG_RESET();		//RESET USART3
	}

	else if(pUSARTx == UART4)
	{
		USART4_REG_RESET();		//RESET USART4
	}

	else if(pUSARTx == UART5)
	{
		USART5_REG_RESET();		//RESET USART5
	}

	else if(pUSARTx == USART6)
	{
		USART6_REG_RESET();		//RESET USART6
	}

	USART_PControl(pUSARTx, DISABLE);
}


/*
 * USART SEND AND RECEIVE DATA WITHOUT INTERRUPT
 */
/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	USART_Send_Data
 * @Brief		-	This function SENDS Data using USART
 *
 * @Input Parameters:
 * 1. USART_Reg *pUSARTx
 * 				- Base address of the USART peripheral
 * 2. uint8_t *pTxBuffer
 * 				- Transmitter Buffer Register
 * 3. uint32_t Len
 * 				- Length of the Data to be Transmitted
 * @Return		- None
 *
 * @Note		- Its an BLOCKING FUNCTION
 *
 *************************************************************************/
void USART_Send_Data(USART_Handle *pUSART_Handle, uint8_t *pTxBuffer, uint32_t Len)
{
	//Declaring Data Pointer
	uint16_t *pdata;

	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Wait until TXE flag is set in the SR Register
		while(!Get_USART_Flag_Status(pUSART_Handle->pUSARTx, USART_TXE_FLAG));

         //Checking the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSART_Handle->USART_Config.USART_WORD_LENGTH == USART_WORD_LENGTH_9BITS)
		{
			//If 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*)pTxBuffer;
			pUSART_Handle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			//Checking for USART_PARITY_CONTROL
			if(pUSART_Handle->USART_Config.USART_PARITY_CONTROL == USART_PARITY_DI)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				//Incrementing pTxBuffer twice as TWO BYTES are sent at once
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//For 8bit data transfer
			pUSART_Handle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);

			//Implement the code to increment the buffer address
			pTxBuffer++;
		}
	}

	//Implement the code to wait till TC flag is set in the SR
	while(!Get_USART_Flag_Status(pUSART_Handle->pUSARTx, USART_TC_FLAG));
}


/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	USART_Receive_Data
 * @Brief		-	This function RECEIVE Data using USART *
 * @Input Parameters:
 * 1. USART_Reg *pUSARTx
 * 				- Base address of the USART peripheral
 * 2. uint8_t *pRxBuffer
 * 				- Receiver Buffer Register
 * 3. uint32_t Len
 * 				- Length of the Data to be Received
 * @Return		- None
 *
 * @Note		- Its an BLOCKING FUNCTION
 *
 *************************************************************************/
void USART_Receive_Data(USART_Handle *pUSART_Handle, uint8_t *pRxBuffer, uint32_t Len)
{
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until RXNE flag is set in the SR
		while(!Get_USART_Flag_Status(pUSART_Handle->pUSARTx, USART_RXNE_FLAG));
		//Checking the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSART_Handle->USART_Config.USART_WORD_LENGTH == USART_WORD_LENGTH_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSART_Handle->USART_Config.USART_PARITY_CONTROL == USART_PARITY_DI)
			{
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*)pRxBuffer) = (pUSART_Handle->pUSARTx->DR  & (uint16_t)0x01FF);

				//Incrementing the pRxBuffer two times as TWO BYTES are sent at once
				//This is because in order to hold 9BITS we need TWO BYTES
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				 *pRxBuffer = (pUSART_Handle->pUSARTx->DR  & (uint8_t)0xFF);

				 //Increment the pRxBuffer
				pRxBuffer++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSART_Handle->USART_Config.USART_PARITY_CONTROL == USART_PARITY_DI)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				 *pRxBuffer = pUSART_Handle->pUSARTx->DR;
			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//Read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = ((uint8_t)pUSART_Handle->pUSARTx->DR & (uint8_t)0X7F);

			}

			//Incrementing the pRxBuffer
			pRxBuffer++;
		}
	}

}

/*
 * USART SEND AND RECEIVE DATA WITH INTERRUPT
 */
/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	USART_Send_Data_IT
 * @Brief		-	This function SENDS Data using USART
 *
 * @Input Parameters:
 * 1. USART_Reg *pUSARTx
 * 				- Base address of the USART peripheral
 * 2. uint8_t *pTxBuffer
 * 				- Transmitter Buffer Register
 * 3. uint32_t Len
 * 				- Length of the Data to be Transmitted
 * @Return		- uint8_t
 *
 * @Note		- Its a non BLOCKING FUNCTION
 *
 *************************************************************************/
uint8_t USART_Send_Data_IT(USART_Handle *pUSART_Handle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSART_Handle->TxBusyState;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSART_Handle->TxLen = Len;
		pUSART_Handle->pTxBuffer = pTxBuffer;
		pUSART_Handle->TxBusyState = USART_BUSY_IN_TX;

		//Implement the code to enable interrupt for TXE
		pUSART_Handle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

		//Implement the code to enable interrupt for TC
		pUSART_Handle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);


	}

	return txstate;

}


/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	USART_Receive_Data_IT
 * @Brief		-	This function CLEARS the USART Flag
 *
 * @Input Parameters:
 * 1. USART_Reg *pUSARTx
 * 				- Base address of the USART peripheral
 * 2. uint8_t *pRxBuffer
 * 				- Receiver Buffer Register
 * 3. uint32_t Len
 * 				- Length of the Data to be Transmitted
 * @Return		-	uint8_t
 *
 * @Note		-	Its an BLOCKING FUNCTION
 *
 *************************************************************************/
uint8_t USART_Receive_Data_IT(USART_Handle *pUSART_Handle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSART_Handle->RxBusyState;
	uint8_t Rx_Count = Len;

	if(rxstate == USART_READY)
	{
		//When the Rxlen is 0 ie all Bytes received
		//TxLen(Acting as Flag in this function) is 1 ie Flag indicating
		//that function under went Data reception of desired Length.
		if((pUSART_Handle->RxLen == 0) && (pUSART_Handle->TxLen == 1))
		{
			 return rxstate;
		}
	}

	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSART_Handle->RxLen = Len;
		pUSART_Handle->pRxBuffer = pRxBuffer;
		pUSART_Handle->RxBusyState = USART_BUSY_IN_RX;

		//Updating the rxstate
		rxstate = pUSART_Handle->RxBusyState;

		//Using TxLen as a Flag in order to avoid looping again and again even after Receiving complete Data Once
		pUSART_Handle->TxLen++;

		//Implement the code to enable interrupt for RXNE
		pUSART_Handle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);

	}

	return rxstate;

}

/*
 *  FUnction to Set BAUD RATE of the USART Communication
 */
/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	USART_Set_Baud_Rate
 * @Brief		-	This function sets the BAUD RATE for USART communication
 *
 * @Input Parameters:
 * 1. USART_Reg *pUSARTx
 * 				- Base address of the USART peripheral
 * 2. uint32_t BaudRate
 * 				- Baud Rate of the USART Communication
 * @Return		-	None
 *
 * @Note		-	None
 *
 *************************************************************************/
void USART_Set_Baud_Rate(USART_Reg *pUSARTx, uint32_t BaudRate)
{
	//Variables to hold the APB clock and USART DIVISIONS
	uint32_t PCLKx;
	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t Mantissa, Fraction;

	//Temporary Variable Register
	uint32_t tempreg = 0;

	//Get the value of APB bus clock into the variable PCLKx
	if(pUSARTx == USART1 || pUSARTx == USART6)
	{
		//USART1 and USART6 are hanging on APB2 bus
		PCLKx = Get_RCC_APB2_PCLK_info();

	}else
	{
	   PCLKx = Get_RCC_APB1_PCLK_info();
	}

	//Check for OVER8 configuration bit
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
		//OVER8 = 1 , over sampling by 8
		usartdiv = ((25 * PCLKx) / (2 *BaudRate));
	}

	else
	{
		//OVER8 = 0, over sampling by 16
		usartdiv = ((25 * PCLKx) / (4 *BaudRate));
	}

	//Calculate the Mantissa part
	Mantissa = usartdiv/100;

	//Place the Mantissa part in appropriate bit position . refer USART_BRR
	tempreg |= Mantissa << 4;

	//Extract the fraction part
	Fraction = (usartdiv - (Mantissa * 100));

	//Calculate the final fractional
	if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
	{
		//OVER8 = 1 , over sampling by 8
		Fraction = ((( Fraction * 8)+ 50) / 100)& ((uint8_t)0x07);

	}

	else
	{
		//over sampling by 16
		Fraction = ((( Fraction * 16)+ 50) / 100) & ((uint8_t)0x0F);

	}

	//Place the fractional part in appropriate bit position . refer USART_BRR
	tempreg |= Fraction;

	//copy the value of tempreg in to BRR register
	pUSARTx->BRR = tempreg;
}

/*
 *  Function to Handle Interrupt during USART Communication
 */
/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	USART_IRQ_Handling
 * @Brief		-	This function Handles the interrupts during USART communication
 *
 * @Input Parameters:
 * 1. USART_Handle *pUSART_Handle
 * 				- Base address of the USART peripheral
 * @Return		-	None
 *
 * @Note		-	None
 *
 *************************************************************************/
void USART_IRQ_Handling(USART_Handle *pUSART_Handle)
{
	uint32_t temp1 , temp2;

	//Declaring Data Pointer
	uint16_t *pdata;
/*************************Check for TC flag ********************************************/

    //Implement the code to check the state of TC bit in the SR
	temp1 = pUSART_Handle->pUSARTx->SR & ( 1 << USART_SR_TC);

	 //Implement the code to check the state of TCEIE bit
	temp2 = pUSART_Handle->pUSARTx->CR1 & ( 1 << USART_CR1_TCIE);

	if(temp1 && temp2 )
	{
		//printf("INSIDE TC HANDLER \n");
		//this interrupt is because of TC

		//close transmission and call application callback if TxLen is zero
		if ( pUSART_Handle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Check the TxLen . If it is zero then close the data transmission
			if(! pUSART_Handle->TxLen )
			{
				//Implement the code to clear the TC flag
				pUSART_Handle->pUSARTx->SR &= ~( 1 << USART_SR_TC);

				//Implement the code to clear the TCIE control bit
				pUSART_Handle->pUSARTx->CR1 &= ~(1 << USART_CR1_TCIE);

				//Reset the application state
				pUSART_Handle->TxBusyState = USART_READY;

				//Reset Buffer address to NULL
				pUSART_Handle->pTxBuffer = NULL;

				//Reset the length to zero
				pUSART_Handle->TxLen =0;

				//Call the application call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSART_Handle,USART_EVENT_TX_CMPLT);
			}
		}
	}

/*************************Check for TXE flag ********************************************/

	//Implement the code to check the state of TXE bit in the SR
	temp1 = pUSART_Handle->pUSARTx->SR & ( 1 << USART_SR_TXE);

	//Implement the code to check the state of TXEIE bit in CR1
	temp2 = pUSART_Handle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE);


	if(temp1 && temp2 )
	{
		//this interrupt is because of TXE

		if(pUSART_Handle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Keep sending data until Txlen reaches to zero
			if(pUSART_Handle->TxLen > 0)
			{
				//Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if(pUSART_Handle->USART_Config.USART_WORD_LENGTH == USART_WORD_LENGTH_9BITS)
				{
					//if 9BIT , load the DR with 2bytes masking the bits other than first 9 bits
					pdata = (uint16_t*) pUSART_Handle->pTxBuffer;

					//loading only first 9 bits , so we have to mask with the value 0x01FF
					pUSART_Handle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					//check for USART_ParityControl
					if(pUSART_Handle->USART_Config.USART_PARITY_CONTROL == USART_PARITY_DI)
					{
						//No parity is used in this transfer , so, 9bits of user data will be sent
						//Implement the code to increment pTxBuffer twice
						pUSART_Handle->pTxBuffer++;
						pUSART_Handle->pTxBuffer++;

						//Implement the code to decrement the length
						pUSART_Handle->TxLen--;
						pUSART_Handle->TxLen--;
					}
					else
					{
						//Parity bit is used in this transfer . so , 8bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						pUSART_Handle->pTxBuffer++;

						//Implement the code to decrement the length
						pUSART_Handle->TxLen--;
					}
				}
				else
				{
					//This is 8bit data transfer
					pUSART_Handle->pUSARTx->DR = (*(pUSART_Handle->pTxBuffer)  & (uint8_t)0xFF);

					//Implement the code to increment the buffer address
					pUSART_Handle->pTxBuffer++;

					//Implement the code to decrement the length
					pUSART_Handle->TxLen--;
				}

			}
			if (pUSART_Handle->TxLen == 0 )
			{
				//TxLen is zero
				//Implement the code to clear the TXEIE bit (disable interrupt for TXE flag )
				pUSART_Handle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);
			}
		}
	}

/*************************Check for RXNE flag ********************************************/

	temp1 = pUSART_Handle->pUSARTx->SR & ( 1 << USART_SR_RXNE);
	temp2 = pUSART_Handle->pUSARTx->CR1 & ( 1 << USART_CR1_RXNEIE);


	if(temp1 && temp2 )
	{
		//printf("INSIDE THE RXNE HANDLER \n");
		//this interrupt is because of rxne
		//this interrupt is because of txe
		if(pUSART_Handle->RxBusyState == USART_BUSY_IN_RX)
		{
			//TXE is set so send data
			if(pUSART_Handle->RxLen > 0)
			{
				//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if(pUSART_Handle->USART_Config.USART_WORD_LENGTH == USART_WORD_LENGTH_9BITS)
				{
					//We are going to receive 9bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSART_Handle->USART_Config.USART_PARITY_CONTROL == USART_PARITY_DI)
					{
						//No parity is used. so, all 9bits will be of user data

						//read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) pUSART_Handle->pRxBuffer) = (pUSART_Handle->pUSARTx->DR  & (uint16_t)0x01FF);

						//Now increment the pRxBuffer two times
						pUSART_Handle->pRxBuffer++;
						pUSART_Handle->pRxBuffer++;

						//Implement the code to decrement the length
						pUSART_Handle->RxLen--;
					}
					else
					{
						//Parity is used. so, 8bits will be of user data and 1 bit is parity
						 *(pUSART_Handle->pRxBuffer) = (pUSART_Handle->pUSARTx->DR  & (uint8_t)0xFF);

						 //Now increment the pRxBuffer
						 pUSART_Handle->pRxBuffer++;

						 //Implement the code to decrement the length
						 pUSART_Handle->RxLen--;
					}
				}
				else
				{
					//We are going to receive 8bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSART_Handle->USART_Config.USART_PARITY_CONTROL == USART_PARITY_DI)
					{
						//No parity is used , so all 8bits will be of user data

						//read 8 bits from DR
						 *(pUSART_Handle->pRxBuffer) = (uint8_t) (pUSART_Handle->pUSARTx->DR  & (uint8_t)0xFF);
					}

					else
					{
						//Parity is used, so , 7 bits will be of user data and 1 bit is parity

						//read only 7 bits , hence mask the DR with 0X7F
						 *(pUSART_Handle->pRxBuffer) = (uint8_t) (pUSART_Handle->pUSARTx->DR  & (uint8_t)0x7F);

					}

					//Now , increment the pRxBuffer
					pUSART_Handle->pRxBuffer++;

					//Implement the code to decrement the length
					pUSART_Handle->RxLen--;
				}


			}//if of >0

			if(! pUSART_Handle->RxLen)
			{
				//printf("Received all BYTES \n");
				//disable the rxne
				pUSART_Handle->pUSARTx->CR1 &= ~( 1 << USART_CR1_RXNEIE );
				pUSART_Handle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSART_Handle,USART_EVENT_RX_CMPLT);
			}
		}
	}


/*************************Check for CTS flag ********************************************/
//Note : CTS feature is not applicable for UART4 and UART5

	//Implement the code to check the status of CTS bit in the SR
	temp1 = pUSART_Handle->pUSARTx->SR & (1 << USART_SR_CTS);

	//Implement the code to check the state of CTSE bit in CR1
	temp2 = pUSART_Handle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE);

	//Implement the code to check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.)
	//temp3 = pUSART_Handle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSIE);


	if(temp1  && temp2 )
	{
		//Implement the code to clear the CTS flag in SR
		pUSART_Handle->pUSARTx->SR &= ~(1 << USART_SR_CTS);

		//this interrupt is because of cts
		USART_ApplicationEventCallback(pUSART_Handle,USART_EVENT_CTS);
	}

/*************************Check for IDLE detection flag ********************************************/

	//Implement the code to check the status of IDLE flag bit in the SR
	temp1 = pUSART_Handle->pUSARTx->SR & (1 << USART_SR_IDLE);

	//Implement the code to check the state of IDLEIE bit in CR1
	temp2 = pUSART_Handle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE);


	if(temp1 && temp2)
	{
		//Implement the code to clear the IDLE flag. Refer to the RM to understand the clear sequence
		pUSART_Handle->pUSARTx->SR &= ~(1 << USART_SR_IDLE);
		//this interrupt is because of idle
		USART_ApplicationEventCallback(pUSART_Handle,USART_EVENT_IDLE);
	}

/*************************Check for Overrun detection flag ********************************************/

	//Implement the code to check the status of ORE flag  in the SR
	temp1 = pUSART_Handle->pUSARTx->SR & USART_SR_ORE;

	//Implement the code to check the status of RXNEIE  bit in the CR1
	temp2 = pUSART_Handle->pUSARTx->CR1 & USART_CR1_RXNEIE;


	if(temp1  && temp2 )
	{
		//Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag .
		USART_CLR_ORE_FLAG(pUSART_Handle->pUSARTx);
		//this interrupt is because of Overrun error
		USART_ApplicationEventCallback(pUSART_Handle,USART_EVENT_ORE);
	}



/*************************Check for Error Flag ********************************************/

//Noise Flag, Overrun error and Framing Error in multibuffer communication
//We dont discuss multibuffer communication in this course. please refer to the RM
//The below code will get executed in only if multibuffer mode is used.

	temp2 =  pUSART_Handle->pUSARTx->CR3 & ( 1 << USART_CR3_EIE) ;

	if(temp2 )
	{
		temp1 = pUSART_Handle->pUSARTx->SR;
		if(temp1 & ( 1 << USART_SR_FE))
		{
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSART_Handle,USART_ERREVENT_FE);
		}

		if(temp1 & ( 1 << USART_SR_NF) )
		{
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSART_Handle,USART_ERREVENT_NF);
		}

		if(temp1 & ( 1 << USART_SR_ORE) )
		{
			USART_ApplicationEventCallback(pUSART_Handle,USART_ERREVENT_ORE);
		}
	}


}

/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	USART_ApplicationEventCallback
 * @Brief		-	This function Handles the interrupts during USART communication
 *
 * @Input Parameters:
 * 1. USART_Handle *pUSART_Handle
 * 				- Base address of the USART peripheral
 * 2. uint8_t event
 * 				- Event Occurred during USART Communication
 * @Return		-	None
 *
 * @Note		-	None
 *
 *************************************************************************/
__Weak void USART_ApplicationEventCallback(USART_Handle *pUSART_Handle, uint8_t event)
{

}
