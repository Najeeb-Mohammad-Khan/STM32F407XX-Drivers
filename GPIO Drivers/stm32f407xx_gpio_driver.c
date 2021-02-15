/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: 16-Dec-2020
 *      Author: Najeeb Mohammad Khan
 */

#include "stm32f407xx_gpio_driver.h"

/*
 * GPIO Init & DeInit
 */

/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	GPIO_Init
 * @Brief		-	This function initializes the GPIO Port
 *
 * @Input Parameters:
 * 1. GPIO_Handle *pGPIO_Handle
 * 				- Base address of the GPIO peripheral
 *
 * @Return		-	None
 *
 * @Note		-	None
 *
 *************************************************************************/
void GPIO_Init(GPIO_Handle *pGPIO_Handle)
{	uint32_t temp = 0;

	//Enabling the CLOCK
	GPIO_PCLK_Control(pGPIO_Handle->pGPIOx, ENABLE);

	//CONFIGURING PIN MODE
	if(pGPIO_Handle->GPIO_PinCFGN.GPIO_PinMode <= GPIO_MODE_ANG)
	{
		//NON INTERRUPT MODE
		temp = (pGPIO_Handle->GPIO_PinCFGN.GPIO_PinMode << (2*pGPIO_Handle->GPIO_PinCFGN.GPIO_PinNumber));
		pGPIO_Handle->pGPIOx->MODER &= ~(0x3 << pGPIO_Handle->GPIO_PinCFGN.GPIO_PinNumber);
		pGPIO_Handle->pGPIOx->MODER |= temp;
	}
	else
	{
		//INTERRUPT MODE
		if(pGPIO_Handle->GPIO_PinCFGN.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			EXTI->FTSR |= (1 << pGPIO_Handle->GPIO_PinCFGN.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIO_Handle->GPIO_PinCFGN.GPIO_PinNumber);
		}

		else if(pGPIO_Handle->GPIO_PinCFGN.GPIO_PinAltFunMode == GPIO_MODE_IT_RT)
		{
			EXTI->RTSR |= (1 << pGPIO_Handle->GPIO_PinCFGN.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << pGPIO_Handle->GPIO_PinCFGN.GPIO_PinNumber);
		}

		else if(pGPIO_Handle->GPIO_PinCFGN.GPIO_PinAltFunMode == GPIO_MODE_IT_RFT)
		{
			EXTI->RTSR |= (1 << pGPIO_Handle->GPIO_PinCFGN.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIO_Handle->GPIO_PinCFGN.GPIO_PinNumber);
		}

		//Configuring The GPIO Port Selection in SYSCFG_EXTICR Register.
		uint8_t PortCode = GPIO_Base_Addr_To_Code(pGPIO_Handle->pGPIOx);
		SYSCFG_PCLK_EN();

		if(pGPIO_Handle->GPIO_PinCFGN.GPIO_PinNumber < 4)
		{
			SYSCFG->EXTICR1 |= (PortCode << 4*(pGPIO_Handle->GPIO_PinCFGN.GPIO_PinNumber));
		}

		else if((pGPIO_Handle->GPIO_PinCFGN.GPIO_PinNumber >= 4) && (pGPIO_Handle->GPIO_PinCFGN.GPIO_PinNumber < 8))
		{
			SYSCFG->EXTICR2 |= (PortCode << 4*((pGPIO_Handle->GPIO_PinCFGN.GPIO_PinNumber) % 4));
		}

		else if((pGPIO_Handle->GPIO_PinCFGN.GPIO_PinNumber >= 8) && (pGPIO_Handle->GPIO_PinCFGN.GPIO_PinNumber < 12))
		{
			SYSCFG->EXTICR2 |= (PortCode << 4*((pGPIO_Handle->GPIO_PinCFGN.GPIO_PinNumber) % 4));
		}

		else if((pGPIO_Handle->GPIO_PinCFGN.GPIO_PinNumber >= 12) && (pGPIO_Handle->GPIO_PinCFGN.GPIO_PinNumber < 16))
		{
			SYSCFG->EXTICR2 |= (PortCode << 4*((pGPIO_Handle->GPIO_PinCFGN.GPIO_PinNumber) % 4));
		}

		//Enabling EXTI Interrupt delivering using IMR Register.
		EXTI->IMR |= (1 << pGPIO_Handle->GPIO_PinCFGN.GPIO_PinNumber);
	}


	temp = 0;
	//CONFIGURING PIN SPEED
	if(pGPIO_Handle->GPIO_PinCFGN.GPIO_PinSpeed <= GPIO_OUT_SPD_HI)
	{
		//NON INTERRUPT MODE
		temp = (pGPIO_Handle->GPIO_PinCFGN.GPIO_PinSpeed << (2*pGPIO_Handle->GPIO_PinCFGN.GPIO_PinNumber));
		pGPIO_Handle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIO_Handle->GPIO_PinCFGN.GPIO_PinNumber);
		pGPIO_Handle->pGPIOx->OSPEEDR |= temp;
	}


	temp = 0;
	//CONFIGURING PIN OUTPUT TYPE
	if(pGPIO_Handle->GPIO_PinCFGN.GPIO_PinOPType <= GPIO_OUT_TYP_OD)
	{
		temp = (pGPIO_Handle->GPIO_PinCFGN.GPIO_PinOPType << (1*pGPIO_Handle->GPIO_PinCFGN.GPIO_PinNumber));
		pGPIO_Handle->pGPIOx->OTYPER &= ~(0x1 << pGPIO_Handle->GPIO_PinCFGN.GPIO_PinNumber);
		pGPIO_Handle->pGPIOx->OTYPER |= temp;
	}


	temp = 0;
	//CONFIGURING PIN PULL-UP / PULL-DOWN
	if(pGPIO_Handle->GPIO_PinCFGN.GPIO_PinPuPdControl <= GPIO_PIN_PD)
	{
		temp = (pGPIO_Handle->GPIO_PinCFGN.GPIO_PinPuPdControl << (2*pGPIO_Handle->GPIO_PinCFGN.GPIO_PinNumber));
		pGPIO_Handle->pGPIOx->PUPDR &= ~(0x3 << pGPIO_Handle->GPIO_PinCFGN.GPIO_PinNumber);
		pGPIO_Handle->pGPIOx->PUPDR |= temp;
	}


	temp = 0;
	//CONFIGURING PIN ALTERNATE FUNCTION
	if(pGPIO_Handle->GPIO_PinCFGN.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t ALFN_Reg_Select, ALFN_Pin_Select;
		ALFN_Reg_Select = (pGPIO_Handle->GPIO_PinCFGN.GPIO_PinNumber / 8);
		ALFN_Pin_Select = (pGPIO_Handle->GPIO_PinCFGN.GPIO_PinNumber % 8);

		if(ALFN_Reg_Select >= 1)
		{
			pGPIO_Handle->pGPIOx->AFRH &= ~(0xF << ALFN_Pin_Select);
			pGPIO_Handle->pGPIOx->AFRH |= (pGPIO_Handle->GPIO_PinCFGN.GPIO_PinAltFunMode << (0x4*ALFN_Pin_Select));

		}

		if(ALFN_Reg_Select < 1)
		{
			pGPIO_Handle->pGPIOx->AFRL &= ~(0xF << ALFN_Pin_Select);
			pGPIO_Handle->pGPIOx->AFRL |= (pGPIO_Handle->GPIO_PinCFGN.GPIO_PinAltFunMode << (0x4*ALFN_Pin_Select));

		}
	}

}

/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	GPIO_DeInit
 * @Brief		-	This function De-Initializes the GPIO Port
 *
 * @Input Parameters:
 * 1. GPIO_Reg *pGPIOx
 * 				- Base address of the GPIO peripheral
 *
 * @Return		-	None
 *
 * @Note		-	None
 *
 *************************************************************************/
void GPIO_DeInit(GPIO_Reg *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}

	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}

	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}

	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}

	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}

	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}

	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}

	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}

	else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}

}


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
void GPIO_PCLK_Control(GPIO_Reg *pGPIOx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}

		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}

		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}

		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}

		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}

		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}

		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}

		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}

		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}

	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}

		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}

		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}

		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}

		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}

		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}

		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}

		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}

		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}

	}
}


/*
 * Read From Input GPIO Pin & Port
 */
/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	GPIO_Read_Input_Pin
 * @Brief		-	This function reads the value of GPIO peripheral input pin
 *
 * @Input Parameters:
 * 1. GPIO_Reg *pGPIOx
 * 				- Base address of the GPIO peripheral
 * 2. PinNumber	- GPIO pin number
 *
 * @Return		-	uint8_t
 *
 * @Note		-	None
 *
 *************************************************************************/
uint8_t GPIO_Read_Input_Pin(GPIO_Reg *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & (0x00000001));
	return value;
}


/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	GPIO_Read_Input_Port
 * @Brief		-	This function reads the value of GPIO peripheral input port
 *
 * @Input Parameters:
 * 1. GPIO_Reg *pGPIOx
 * 				- Base address of the GPIO peripheral
 *
 * @Return		-	uint16_t
 *
 * @Note		-	None
 *
 *************************************************************************/
uint16_t GPIO_Read_Input_Port(GPIO_Reg *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;

}


/*
 * Write To Output GPIO Pin & Port
 */
/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	GPIO_Write_Output_Pin
 * @Brief		-	This function writes the value to the GPIO peripheral output pin
 *
 * @Input Parameters:
 * 1. GPIO_Reg *pGPIOx
 * 				- Base address of the GPIO peripheral
 * 2. PinNumber	- GPIO pin number
 * 3. Value		- Value to write on GPIO pin
 *
 * @Return		-	None
 *
 * @Note		-	None
 *
 *************************************************************************/
void GPIO_Write_Output_Pin(GPIO_Reg *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == SET)
	{
		pGPIOx->ODR |= (1<<PinNumber);
	}

	else
	{
		pGPIOx->ODR &= ~(1<<PinNumber);
	}
}


/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	GPIO_Write_Output_Port
 * @Brief		-	This function writes the value to the GPIO peripheral output port
 *
 * @Input Parameters:
 * 1. GPIO_Reg *pGPIOx
 * 				- Base address of the GPIO peripheral
 * 2. Value		- Value to write on GPIO pin
 *
 * @Return		-	None
 *
 * @Note		-	None
 *
 *************************************************************************/
void GPIO_Write_Output_Port(GPIO_Reg *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}


/*
 * Toggle GPIO Output Pin
 */
/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	GPIO_Toggle_Pin
 * @Brief		-	This function toggles the GPIO peripheral pin
 *
 * @Input Parameters:
 * 1. GPIO_Reg *pGPIOx
 * 				- Base address of the GPIO peripheral
 * 2. PinNumber	- GPIO pin number to be toggled
 *
 * @Return		-	None
 *
 * @Note		-	None
 *
 *************************************************************************/
void GPIO_Toggle_Pin(GPIO_Reg *pGPIOx, uint8_t PinNumber)
{
	if(!(pGPIOx->ODR & (1 << PinNumber)))
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}


/*
 * GPIO IRQ Configuration and ISR Handling
 */
/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	IRQ_Config
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
void GPIO_IRQ_Config(uint8_t IRQNumber, uint8_t ENorDI)
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
 * @Function	-	IRQ_PriorityConfig
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
void GPIO_IRQ_PriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//Configuring required IPR Register To SET THE INTERRUPT.
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t PR_Bits_Shift = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASEADDR + (iprx * 4)) |= (IRQPriority << PR_Bits_Shift);

}


/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	IRQ_Handling
 * @Brief		-	This function Handles(CLEAR) IRQ
 *
 * @Input Parameters:
 * 1. PinNumber	- GPIO pin number
 *
 * @Return		-	None
 *
 * @Note		-	None
 *
 *************************************************************************/
void GPIO_IRQ_Handling(uint8_t PinNumber)
{
	//Clearing the Triggered Interrupt
	if(EXTI->PR & (1 << PinNumber))  //Checking if Interrupt occurred
	{
		//Clearing by setting the bit to 1.
		EXTI->PR |= (1 << PinNumber);
	}
}
