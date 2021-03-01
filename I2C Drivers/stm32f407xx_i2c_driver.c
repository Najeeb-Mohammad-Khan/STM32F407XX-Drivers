/*
 * stm32f407xxx_i2c_deriver.c
 *
 *  Created on: 18-Feb-2021
 *      Author: Najeeb Mohammad Khan
 */

#include "stm32f407xx_i2c_driver.h"

#define READ	0x1
#define WRITE	0x2

static void GENERATE_START_COND(I2C_Reg *pI2Cx);
static void GENERATE_STOP_COND(I2C_Reg *pI2Cx);
static void EXECUTE_ADDR_PHASE(I2C_Reg *pI2Cx, uint8_t SLAVE_ADDR, uint8_t READorWRITE);
static void I2C_CLR_ADDR_FLAG(I2C_Reg *pI2Cx);

static void GENERATE_START_COND(I2C_Reg *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void GENERATE_STOP_COND(I2C_Reg *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

static void EXECUTE_ADDR_PHASE(I2C_Reg *pI2Cx, uint8_t SLAVE_ADDR, uint8_t READorWRITE)
{
	SLAVE_ADDR = (SLAVE_ADDR << 1);

	if(READorWRITE == WRITE)
	{
			SLAVE_ADDR &= ~(0x1);	//SETTING THE LAST BIT AS 0 ie WRITE
	}
	else
	{
		SLAVE_ADDR |= (0x1);		//SETTING THE LAST BIT AS 1 ie READ
	}

	pI2Cx->DR = SLAVE_ADDR;
}

static void I2C_CLR_ADDR_FLAG(I2C_Reg *pI2Cx)
{
	//Clearing the ADDR FLAG by reading the SR1 and SR2 Flags
	uint16_t dummy_read;
	dummy_read = pI2Cx->SR1;
	dummy_read = pI2Cx->SR2;
	(void)dummy_read;		//Type Casting to void to avoid UNUSED VARABLE warning
}

uint32_t Get_RCC_PCLK1_info()
{
	uint32_t PCLK1, SYS_CLK, AHB_PRESCALER,APB1_PRESCALER;
	uint8_t CLK_SRC, SYS_CLK_DIV, APB_CLK_DIV;
	uint16_t AHB_PRESCALER_ARR[] = {2, 4, 8, 16, 64, 128, 256, 512};
	uint16_t APB1_PRESCALER_ARR[] = {2, 4, 8, 16};

	CLK_SRC = (RCC->CFGR >> 0x2) & (0x3); //We are shifting the SWS bits to 0 and 1 positions and masking all other bits except them.
	if(CLK_SRC == 0)	// FOR HSI
	{
		SYS_CLK = 16000000; //16MHz
	}

	else if(CLK_SRC == 1) // FOR HSE
	{
		SYS_CLK = 8000000; //8Mhz
	}

	SYS_CLK_DIV = (RCC->CFGR >> 4) & (0xF); //We are shifting the HPRE bits to first 4 positions and masking all other bits except them.
	if(SYS_CLK_DIV < 0x8)
	{
		AHB_PRESCALER = 0x1;
	}

	else
	{
		uint8_t temp =  (SYS_CLK_DIV % 0x8);
		AHB_PRESCALER = AHB_PRESCALER_ARR[temp];
	}

	APB_CLK_DIV = (RCC->CFGR >> 10) & (0xF);
	if(APB_CLK_DIV < 0x4)
	{
		APB1_PRESCALER = 0x1;
	}
	else
	{	uint8_t temp = (APB_CLK_DIV % 4);
		APB1_PRESCALER = APB1_PRESCALER_ARR[temp];
	}
	//I have not implemented PLL CLOCK HERE
	PCLK1 = ((SYS_CLK / AHB_PRESCALER) / APB1_PRESCALER);
	return PCLK1;
}


/*
 * I2C FLAG STATUS
 */

/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	Get_I2C_Flag_Status
 * @Brief		-	This function returns the status of the I2C Flags
 *
 * @Input Parameters:
 * 1. SPI_Reg*pI2Cx
 * 				- Base address of the I2C peripheral
 * 2. FlagRegister	- Name of the flag register in which the flag bit is present.
 * 3. FlagName	- Name of the flag for which you want to get status.
 * @Return		-	uint8_t 	(SET/RESET)
 *
 * @Note		-	None
 *
 *************************************************************************/
uint8_t Get_I2C_Flag_Status(I2C_Reg *pI2Cx, uint8_t FlagRegister, uint32_t FlagName)
{
	if(FlagRegister == I2C_SR1_FLAG_REG)
	{
		if(pI2Cx->SR1 & FlagName)
		{
			return SET;
		}
	}

	else if(FlagRegister == I2C_SR2_FLAG_REG)
	{
		if(pI2Cx->SR2 & FlagName)
		{
			return SET;
		}
	}

	return RESET;
}

/*
 * I2C Clock Setup
 */
/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	I2C_PCLK_Control
 * @Brief		-	This function Enables or disables the I2C peripheral clock
 *
 * @Input Parameters:
 * 1. I2C_Reg *pI2Cx
 * 				- Base address of the I2C peripheral
 * 2. ENorDI	- ENABLE or DISABLE
 *
 * @Return		-	None
 *
 * @Note		-	None
 *
 *************************************************************************/
void I2C_PCLK_Control(I2C_Reg *pI2Cx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}

		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}

		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}

		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}

		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}

}

/*
 * I2C Peripheral Control
 */
/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	I2C_PControl
 * @Brief		-	This function Enables or disables the I2C peripheral
 *
 * @Input Parameters:
 * 1. I2C_Reg *pI2Cx
 * 				- Base address of the I2C peripheral
 * 2. ENorDI	- ENABLE or DISABLE
 *
 * @Return		-	None
 *
 * @Note		-	None
 *
 *************************************************************************/
void I2C_PControl(I2C_Reg *pI2Cx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
	{	/*
		if((!(Get_Flag_Status(pI2Cx, SPI_CR1_SSI))) & (Get_Flag_Status(pSPIx, SPI_CR1_SSM)))
		{
			pSPIx->CR1 |= (1 << SPI_CR1_SSI); //Setting SSI to prevent MODF Bit triggering and Resetting Master Bit in CR1.
		}
		*/
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

/*
 * I2C Init & DeInit
 */
/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	I2C_Init
 * @Brief		-	This function initializes the I2C Port
 *
 * @Input Parameters:
 * 1. I2C_Handle *pI2C_Handle
 * 				- Base address of the I2C peripheral
 *
 * @Return		-	None
 *
 * @Note		-	None
 *
 *************************************************************************/
void I2C_Init(I2C_Handle *pI2C_Handle)
{
	//Activating the I2C CLOCK
	I2C_PCLK_Control(pI2C_Handle->pI2Cx, ENABLE);

	//DISABLING THE I2C PERIPHERALS BEFORE MODIFYING THE I2C REGISTERS
	I2C_PControl(pI2C_Handle->pI2Cx, DISABLE);

	uint32_t temp_reg = 0; //Temp. register to store the values to be stored in the CR1 register
	//ACK CONTROL BIT
	temp_reg |= (pI2C_Handle->I2C_Config.ACK_CONTROL << I2C_CR1_ACK);
	pI2C_Handle->pI2Cx->CR1 |= temp_reg;

	//CONFIGURING THE FREQ FEILD OF CR2
	temp_reg = 0;
	temp_reg |= Get_RCC_PCLK1_info() / 1000000U;
	pI2C_Handle->pI2Cx->CR2 = (temp_reg & (0x3F));

	//CONFIGURING THE SLAVE ADDRESS(ADD) OF OAR1
	temp_reg = 0;
	temp_reg |= pI2C_Handle->I2C_Config.I2C_DEVICE_ADDR << 1;
	pI2C_Handle->pI2Cx->OAR1 = temp_reg & (0xFE);

	//SETTING THE 14th BIT OF THE OAR1 AS IT SHOULD BE ONE ACCORDING TO DATASHEET FOR PROPER WORKING OF I2C.
	pI2C_Handle->pI2Cx->OAR1 |= 1 << 0xE;

	//CONFIGURING THE CLOCK CONTROL REGISTER
	uint16_t CCR_VALUE;
	temp_reg = 0;

	if(pI2C_Handle->I2C_Config.I2C_SCL_SPEED <= I2C_SCL_SPD_SM_1K)
	{
		//STANDARD MODE SPEED
		pI2C_Handle->pI2Cx->CCR &= ~(1 << I2C_CCR_FS);	//Selecting Standard Mode
		CCR_VALUE = (Get_RCC_PCLK1_info() / (2*pI2C_Handle->I2C_Config.I2C_SCL_SPEED));
		temp_reg |= (CCR_VALUE & 0xFFF);
		//THERE IS NO DUTY CYCLE SETTING IN SM AND ITS TAKEN AS 50% BY DEFAULT
	}
	else
	{
		uint8_t FM_DUTY;
		FM_DUTY = pI2C_Handle->I2C_Config.I2C_FM_DUTYCYCLE;

		//FAST MODE SPEED
		pI2C_Handle->pI2Cx->CCR |= (1 << I2C_CCR_FS); //Selecting Fast Mode
		pI2C_Handle->pI2Cx->CCR |= (FM_DUTY << I2C_CCR_DUTY);

		if(FM_DUTY == 0)	// (T(low) / (T(high)) = 2
		{
			CCR_VALUE = (Get_RCC_PCLK1_info() / (3*pI2C_Handle->I2C_Config.I2C_SCL_SPEED));
		}
		else
		{
			CCR_VALUE = (Get_RCC_PCLK1_info() / (25*pI2C_Handle->I2C_Config.I2C_SCL_SPEED));
		}

		temp_reg |= (CCR_VALUE & 0xFFF);
	}
	pI2C_Handle->pI2Cx->CCR = temp_reg;

	//TRISE CONFIGURATION
	temp_reg = 0;
	if(pI2C_Handle->I2C_Config.I2C_SCL_SPEED <= I2C_SCL_SPD_SM_1K)
	{
		//SCL SPEED (STANDARD MODE)
		temp_reg |= (Get_RCC_PCLK1_info() / 1000000U) + 1;	//(T(rise)max for SM is 1us)
	}
	else
	{
		//SCL SPEED (FAST MODE)
		temp_reg |= ((Get_RCC_PCLK1_info()*300) / 1000000000U + 1);	 //(T(rise)max for FM is 300ns)
	}
	pI2C_Handle->pI2Cx->TRISE = (temp_reg & 0x3F);
}

/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	I2C_DeInit
 * @Brief		-	This function de-initializes the I2C Port
 *
 * @Input Parameters:
 * 1. I2C_Handle *pI2C_Handle
 * 				- Base address of the I2C peripheral
 *
 * @Return		-	None
 *
 * @Note		-	None
 *
 *************************************************************************/
void I2C_DeInit(I2C_Reg *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();		//RESET I2C1
	}

	else if(pI2Cx == I2C2)
	{
		I2C2_REG_RESET();		//RESET I2C2
	}

	else if(pI2Cx == I2C3)
	{
		I2C3_REG_RESET();		//RESET I2C3
	}
}

/*
 * I2C Master Send Data
 */
/*********************  FUNCTION DOCUMENTATION  ***************************
 * @Function	-	I2C_MASTER_SendData
 * @Brief		-	This function Sends Data on I2C Port
 *
 * @Input Parameters:
 * 1. I2C_Handle *pI2C_Handle
 * 				- Base address of the I2C peripheral
 * 2. uint8_t *pTxBuffer
 * 				- Transmit Buffer Variable
 * 3. uint32_t Len
 * 				- Length of the Data Word
 * 4. uint8_t Slave_Addr
 * 				- Address of the Slave Device
 * @Return		-	None
 *
 * @Note		-	This is BLOCKING CALL
 *
 *************************************************************************/
void I2C_MASTER_Send_Data(I2C_Handle *pI2C_Handle, uint8_t *pTx_Buffer, uint32_t Len, uint8_t Slave_Addr)
{
	//Activating the I2C PERIPHREALS
	I2C_PControl(pI2C_Handle->pI2Cx, ENABLE);

	//Generating the START CONDITION
	GENERATE_START_COND(pI2C_Handle->pI2Cx);

	//Confirming the STAR GENERATION is complete
	while(!Get_I2C_Flag_Status(pI2C_Handle->pI2Cx, I2C_SR1_FLAG_REG, I2C_SB_FLAG));

	//Send the address of the SLAVE with READ/WRITE last bit
	EXECUTE_ADDR_PHASE(pI2C_Handle->pI2Cx, Slave_Addr, WRITE);

	//Confirming the ADDRESS PHASE Completion
	while(!Get_I2C_Flag_Status(pI2C_Handle->pI2Cx, I2C_SR1_FLAG_REG, I2C_ADDR_FLAG));

	//Clearing the ADDR FLAG (Until ADDR is cleared the SCL will be stretched ie PULLED TO LOW )
	I2C_CLR_ADDR_FLAG(pI2C_Handle->pI2Cx);

	//Sending the DATA after confirming if the data register is EMPTY or NOT EMPTY (BY CHEKING THE TXE FLAG)
	while(Len > 0)
	{
		//Wait till Data register is empty
		while(!Get_I2C_Flag_Status(pI2C_Handle->pI2Cx, I2C_SR1_FLAG_REG, I2C_TxE_FLAG));
		pI2C_Handle->pI2Cx->DR = *pTx_Buffer;
		pTx_Buffer++;
		Len--;
	}

	//Terminating the I2C communication
	//TXE = 1 and BTF = 1, Condition for staring a next transmission where we are going to stop the transmission.
	//When BTF = 1, SCL will be stretched and PULLED TO LOW
	while(!Get_I2C_Flag_Status(pI2C_Handle->pI2Cx, I2C_SR1_FLAG_REG, I2C_TxE_FLAG));
	while(!Get_I2C_Flag_Status(pI2C_Handle->pI2Cx, I2C_SR1_FLAG_REG, I2C_BTF_FLAG));

	//Generating the STOP CONDITION
	GENERATE_STOP_COND(pI2C_Handle->pI2Cx);

	//Deactivating the I2C PERIPHREALS
	I2C_PControl(pI2C_Handle->pI2Cx, DISABLE);
}
