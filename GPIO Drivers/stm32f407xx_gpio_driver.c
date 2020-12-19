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

		if(ALFN_Reg_Select > 1)
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
 * @Return		-	None
 *
 * @Note		-	None
 *
 *************************************************************************/
uint8_t GPIO_Read_Input_Pin(GPIO_Reg *pGPIOx, uint8_t PinNumber)
{

}


/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	GPIO_Read_Input_Port
 * @Brief		-	This function reads the value of GPIO peripheral input port
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
uint16_t GPIO_Read_Input_Port(GPIO_Reg *pGPIOx)
{

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

}


/*
 * GPIO IRQ Configuration and ISR Handling
 */
/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	IRQ_Config
 * @Brief		-	This function configures the IRQ
 *
 * @Input Parameters:
 * 1. IRQNumber		- IRQ number of the Interrupt
 * 2. IRQPriority	- Interrupt priority
 * 3. ENorDI		- ENABLE or DISABLE
 *
 * @Return		-	None
 *
 * @Note		-	None
 *
 *************************************************************************/
void IRQ_Config(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t ENorDI)
{

}


/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	IRQ_Handling
 * @Brief		-	This function Handles IRQ
 *
 * @Input Parameters:
 * 1. PinNumber	- GPIO pin number
 *
 * @Return		-	None
 *
 * @Note		-	None
 *
 *************************************************************************/
void IRQ_Handling(uint8_t PinNumber)
{

}
