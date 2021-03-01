/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: 18-Feb-2021
 *      Author: Najeeb Mohammad Khan
 */

#ifndef STM32F407XX_I2C_DRIVER_H_
#define STM32F407XX_I2C_DRIVER_H_

#include <stm32f407xx.h>

/*
 * SPIx Configuring Peripheral Structure
 */
typedef struct
{
	uint32_t I2C_SCL_SPEED;
	uint8_t I2C_DEVICE_ADDR;
	uint8_t ACK_CONTROL;
	uint16_t I2C_FM_DUTYCYCLE;
}I2C_Config;


/*
 * SPIx Peripheral Handle Structure
 */
typedef struct
{
	I2C_Reg *pI2Cx;
	I2C_Config	I2C_Config;
}I2C_Handle;


/*
 * Generic I2C Peripheral Macros
 */
//I2C SCL SPEED (@I2C_SCL_SPEED)
#define I2C_SCL_SPD_SM_1K	100000
#define I2C_SCL_SPD_FM_2K	200000
#define I2C_SCL_SPD_FM_4K	400000

//I2C ACKNOWLEDGE ENABLE/DISABLE (@ACK_CONTROL)
#define I2C_ACK_EN		0x1
#define I2C_ACK_DI		0x0

// I2C DUTY CYCLE (@I2C_FM_DUTYCYCLE)
#define I2C_FM_DUTYCYCLE_2			0x0
#define I2C_FM_DUTYCYCLE_16BY9		0x1


/*
 * I2C Init & DeInit
 */
void I2C_Init(I2C_Handle *pI2C_Handle);
void I2C_DeInit(I2C_Reg *pI2Cx);

/*
 * I2C Clock Setup & Peripheral Control
 */
void SPI_PCLK_Control(SPI_Reg *pSPIx, uint8_t ENorDI);
void SPI_PControl(SPI_Reg *pSPIx, uint8_t ENorDI);

/*
 * Master Send And Receive Data (Without Interrupt)
 */
void I2C_MASTER_Send_Data(I2C_Handle *pI2C_Handle, uint8_t *pTx_Buffer, uint32_t Len, uint8_t Slave_Addr);
void I2C_MASTER_Receive_Data(I2C_Handle *pI2C_Handle, uint8_t *pRx_Buffer, uint32_t Len);

/*
 * Slave Send And Receive Data (Without Interrupt)
 */
void I2C_SLAVE_Send_Data(I2C_Handle *pI2C_Handle, uint8_t *pTx_Buffer, uint32_t Len);
void I2C_SLAVE_Receive_Data(I2C_Handle *pI2C_Handle, uint8_t *pRx_Buffer, uint32_t Len);


/*
 * Other Peripheral APIS
 */
uint8_t Get_I2C_Flag_Status(I2C_Reg *pI2Cx, uint8_t FlagRegister, uint32_t FlagName);


/*
 * I2C FLAG STATUS DEFITIONS
 * (PROVIDES  MASKING INFO OF VARIOUS FLAGS IN SR1 & SR2 REGISTERs)
 */
// FOR SR1
#define I2C_SB_FLAG 		(1 << I2C_SR1_SB)
#define I2C_ADDR_FLAG 		(1 << I2C_SR1_ADDR)
#define I2C_BTF_FLAG 		(1 << I2C_SR1_BTF)
#define I2C_ADD10_FLAG 		(1 << I2C_SR1_ADD10)
#define I2C_STOPF_FLAG 		(1 << I2C_SR1_STOPF)
#define I2C_RxNE_FLAG 		(1 << I2C_SR1_RxNE)
#define I2C_TxE_FLAG 		(1 << I2C_SR1_TxE)
#define I2C_BERR_FLAG 		(1 << I2C_SR1_BERR)
#define I2C_ARLO_FLAG 		(1 << I2C_SR1_ARLO)
#define I2C_AF_FLAG 		(1 << I2C_SR1_AF)
#define I2C_OVR_FLAG 		(1 << I2C_SR1_OVR)
#define I2C_PEC_ERR_FLAG	(1 << I2C_SR1_PEC_ERR)
#define I2C_TIME_OUT_FLAG 	(1 << I2C_SR1_TIME_OUT)
#define I2C_SMR_ALERT_FLAG 	(1 << I2C_SR1_SMB_ALERT)

// FOR SR2
#define I2C_MSL_FLAG 			(1 << I2C_SR2_MSL)
#define I2C_BUSY_FLAG 			(1 << I2C_SR2_BUSY)
#define I2C_TRA_FLAG 			(1 << I2C_SR2_TRA)
#define I2C_GEN_CALL_FLAG 		(1 << I2C_SR2_GEN_CALL)
#define I2C_SMBDE_FAULT_FLAG 	(1 << I2C_SR2_SMBDE_FAULT)
#define I2C_SMB_HOST_FLAG 		(1 << I2C_SR2_SMB_HOST)
#define I2C_DUALF_FLAG 			(1 << I2C_SR2_DUALF)

//General Flag Register Definitions
#define I2C_SR1_FLAG_REG		0x1
#define I2C_SR2_FLAG_REG		0x2


#endif /* STM32F407XX_I2C_DRIVER_H_ */
