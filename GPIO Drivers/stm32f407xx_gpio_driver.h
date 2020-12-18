/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: 16-Dec-2020
 *      Author: Najeeb Mohammad Khan
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

typedef struct{

	uint8_t GPIO_PinNumber;				// Possible Values From @GPIO_PIN_NUMBER
	uint8_t GPIO_PinMode;				// Possible Values From @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;				// Possible Values From @GPIO_PIN_SPEED
	uint8_t GPIO_PinPuPdControl;		// Possible Values From @GPIO_PIN_PUPD
	uint8_t GPIO_PinOPType;				// Possible Values From @GPIO_PIN_OUT_TYPE
	uint8_t GPIO_PinAltFunMode;			// Possible Values From @GPIO_PIN_ALT_FUNC


}GPIO_PinConfig;


/*
 * Handle Structure For GPIO pin
 */
typedef struct{

	/*Pointer To Hold The Base Address Of The GPIO Peripheral*/
	GPIO_Reg	*pGPIOx;	//Hold the base address of the GPIO port to which the pin belongs
	GPIO_PinConfig	GPIO_PinCFGN;
}GPIO_Handle;



/*
 * Generic GPIO REGESTER Micros
 */
//GPIO PIN NUMBER { @GPIO_PIN_NUMBER }
#define GPIO_PNUM_0			0x0			//GPIO PIN NUMBER 0
#define GPIO_PNUM_1			0x1			//GPIO PIN NUMBER 1
#define GPIO_PNUM_2			0x2			//GPIO PIN NUMBER 2
#define GPIO_PNUM_3			0x3			//GPIO PIN NUMBER 3
#define GPIO_PNUM_4			0x4			//GPIO PIN NUMBER 4
#define GPIO_PNUM_5			0x5			//GPIO PIN NUMBER 5
#define GPIO_PNUM_6			0x6			//GPIO PIN NUMBER 6
#define GPIO_PNUM_7			0x7			//GPIO PIN NUMBER 7
#define GPIO_PNUM_8			0x8			//GPIO PIN NUMBER 8
#define GPIO_PNUM_9			0x9			//GPIO PIN NUMBER 9
#define GPIO_PNUM_10		0xA			//GPIO PIN NUMBER 10
#define GPIO_PNUM_11		0xB			//GPIO PIN NUMBER 11
#define GPIO_PNUM_12		0xC			//GPIO PIN NUMBER 12
#define GPIO_PNUM_13		0xD			//GPIO PIN NUMBER 13
#define GPIO_PNUM_14		0xE			//GPIO PIN NUMBER 14
#define GPIO_PNUM_15		0xF			//GPIO PIN NUMBER 15


//GPIO PIN POSSIBLE MODES { @GPIO_PIN_MODES }
	//NON INTERRUPT MODES
#define GPIO_MODE_IN		0x0			//GPIO Mode As INPUT
#define GPIO_MODE_OUT		0x1			//GPIO Mode As OUTPUT
#define GPIO_MODE_ALTFN		0x2			//GPIO Mode As ALTERNATE FUNCTION
#define GPIO_MODE_ANG		0x3			//GPIO Mode As ANALOG
	//INTRRUPT MODES
#define GPIO_MODE_IN_FT		0x4			//GPIO Mode As INPUT FALLING EDGE TRIGGER
#define GPIO_MODE_IN_RT		0x5			//GPIO Mode As INPUT RISING EDGE TRIGGER
#define GPIO_MODE_IN_RFT	0x6			//GPIO Mode As INPUT RISING EDGE - FALLING EDGE TRIGGER

//GPIO PIN POSSIBLE OUTPUT TYPES { @GPIO_PIN_OUT_TYPE }
#define GPIO_OUT_TYP_PP		0x0			//GPIO OUTPUT TYPE As PUSH - PULL
#define GPIO_OUT_TYP_OD		0x1			//GPIO OUTPUT TYPE As OPEN DRAIN

//GPIO PIN POSSIBLE OUTPUT SPEEDS { @GPIO_PIN_SPEED }
#define GPIO_OUT_SPD_LOW	0x0			//GPIO OUTPUT SPEED LOW
#define GPIO_OUT_SPD_MDM	0x1			//GPIO OUTPUT SPEED MEDIUM
#define GPIO_OUT_SPD_FST	0x2			//GPIO OUTPUT SPEED FAST
#define GPIO_OUT_SPD_HI		0x3			//GPIO OUTPUT SPEED HIGH

//GPIO PIN PULL-UP & PULL-DOWN CONFIGRATION MACROS { @GPIO_PIN_PUPD }
#define GPIO_PIN_NO_PUPD	0x0			//GPIO NO PULL-UP & PULL-DOWN
#define GPIO_PIN_PU			0x1			//GPIO PULL-UP
#define GPIO_PIN_PD			0x2			//GPIO PULL-DOWN




/*
 * GPIO Init & DeInit
 */
void GPIO_Init(GPIO_Handle *pGPIO_Handle);
void GPIO_DeInit(GPIO_Reg *pGPIOx);


/*
 * GPIO Clock Setup
 */
void GPIO_PCLK_Control(GPIO_Reg *pGPIOx, uint8_t ENorDI);


/*
 * Read From Input GPIO Pin & Port
 */
uint8_t GPIO_Read_Input_Pin(GPIO_Reg *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_Read_Input_Port(GPIO_Reg *pGPIOx);


/*
 * Write To Output GPIO Pin & Port
 */
void GPIO_Write_Output_Pin(GPIO_Reg *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_Write_Output_Port(GPIO_Reg *pGPIOx, uint16_t Value);


/*
 * Toggle GPIO Output Pin
 */
void GPIO_Toggle_Pin(GPIO_Reg *pGPIOx, uint8_t PinNumber);


/*
 * GPIO IRQ Configuration and ISR Handling
 */
void IRQ_Config(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t ENorDI);
void IRQ_Handling(uint8_t PinNumber);


#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
