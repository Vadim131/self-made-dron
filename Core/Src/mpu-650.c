/*
 * mpu-650.c
 *
 *  Created on: 2 нояб. 2023 г.
 *      Author: archuser
 */

#include "main.h"
#include "mpu-650.h"
#include "stm32f30x_i2c.h"
#include "stm32f30x_rcc.h"

static void MPU650_IT_I2C(void);
static void MPU650_LowLevel_Init(void);
static uint32_t MPU650_TIMEOUT_UserCallback(void);



void MPU650_Init(){
	uint8_t ctrl;

	MPU650_LowLevel_Init();
	/* power management register 0X6B we should write all 0's to wake the sensor up */
	ctrl = 0x00;
	MPU650_Write(MPU650_I2C_ADDRESS << 1, MPU650_PWR_MGMT_1, &ctrl);

	/* Set data rate of 1KHz by writing SMPLRT_DIV register */
	ctrl = 0x07;
	MPU650_Write(MPU650_I2C_ADDRESS << 1, MPU650_SMPLRT_DIV, &ctrl);
	// Set accelerometer configuration in ACCEL_CONFIG Register
	// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> 2g
	ctrl = 0x00;
	MPU650_Write(MPU650_I2C_ADDRESS << 1, MPU650_ACCEL_CONFIG, &ctrl);

	// Set Gyroscopic configuration in GYRO_CONFIG Register
	// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> 250 dps
	ctrl = 0x00;
	MPU650_Write(MPU650_I2C_ADDRESS << 1, MPU650_GYRO_CONFIG, &ctrl);

}


static void MPU650_LowLevel_Init(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);

	/* Enable GPIO peripheral clock for port B*/
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	/* Set up Alternate function for pin 9 and 8 on port B to i2c */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_4);

	GPIO_InitStructure.GPIO_Pin = MPU650_I2C_SDA_PIN | MPU650_I2C_SCK_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* APB2 - 32Mhz for i2c1 */
	RCC->CFGR3 |= RCC_CFGR3_I2C1SW;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	I2C_InitTypeDef I2C_InitStructure;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_OwnAddress1 = 0;
	I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
	I2C_InitStructure.I2C_DigitalFilter = 0x00;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_Timing = 0xF010C0FF;
	I2C_Init(I2C1, &I2C_InitStructure);

	I2C_Cmd(I2C1, ENABLE);
}

uint16_t MPU650_Write(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer)
{
	/* Test on BUSY Flag */
	uint16_t MPU650_Timeout = MPU650_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(MPU650_I2C, I2C_ISR_BUSY) != RESET)
	{
		if((MPU650_Timeout--) == 0) return MPU650_TIMEOUT_UserCallback();
	}

	/* Configure slave address, nbytes, reload, end mode and start or stop generation */
	I2C_TransferHandling(MPU650_I2C, DeviceAddr, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);

	/* Wait until TXIS flag is set */
	MPU650_Timeout = MPU650_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(MPU650_I2C, I2C_ISR_TXIS) == RESET)
	{
		if((MPU650_Timeout--) == 0) return MPU650_TIMEOUT_UserCallback();
	}

	/* Send Register address */
	I2C_SendData(MPU650_I2C, (uint8_t) RegAddr);

	/* Wait until TCR flag is set */
	MPU650_Timeout = MPU650_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(MPU650_I2C, I2C_ISR_TCR) == RESET)
	{
		if((MPU650_Timeout--) == 0) return MPU650_TIMEOUT_UserCallback();
	}

	/* Configure slave address, nbytes, reload, end mode and start or stop generation */
	I2C_TransferHandling(MPU650_I2C, DeviceAddr, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);

	/* Wait until TXIS flag is set */
	MPU650_Timeout = MPU650_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(MPU650_I2C, I2C_ISR_TXIS) == RESET)
  	{
		if((MPU650_Timeout--) == 0) return MPU650_TIMEOUT_UserCallback();
  	}

	/* Write data to TXDR */
	I2C_SendData(MPU650_I2C, *pBuffer);

	/* Wait until STOPF flag is set */
	MPU650_Timeout = MPU650_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(MPU650_I2C, I2C_ISR_STOPF) == RESET)
	{
		if((MPU650_Timeout--) == 0) return MPU650_TIMEOUT_UserCallback();
	}

	/* Clear STOPF flag */
	I2C_ClearFlag(MPU650_I2C, I2C_ICR_STOPCF);

	return MPU650_OK;
}


uint16_t MPU650_Read(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer, uint16_t NumByteToRead)
{
  /* Test on BUSY Flag */
	uint16_t MPU650_Timeout = MPU650_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(MPU650_I2C, I2C_ISR_BUSY) != RESET)
	{
		if((MPU650_Timeout--) == 0) return MPU650_TIMEOUT_UserCallback();
	}

	/* Configure slave address, nbytes, reload, end mode and start or stop generation */
	I2C_TransferHandling(MPU650_I2C, DeviceAddr, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

	/* Wait until TXIS flag is set */
	MPU650_Timeout = MPU650_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(MPU650_I2C, I2C_ISR_TXIS) == RESET)
	{
		if((MPU650_Timeout--) == 0) return MPU650_TIMEOUT_UserCallback();
	}

	if(NumByteToRead>1)
		RegAddr |= 0x80;


	/* Send Register address */
 	I2C_SendData(MPU650_I2C, (uint8_t)RegAddr);

 	/* Wait until TC flag is set */
 	MPU650_Timeout = MPU650_LONG_TIMEOUT;
 	while(I2C_GetFlagStatus(MPU650_I2C, I2C_ISR_TC) == RESET)
 	{
    if((MPU650_Timeout--) == 0) return MPU650_TIMEOUT_UserCallback();
 	}

 	/* Configure slave address, nbytes, reload, end mode and start or stop generation */
 	I2C_TransferHandling(MPU650_I2C, DeviceAddr, NumByteToRead, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

 	/* Wait until all data are received */
 	while (NumByteToRead)
 	{
 		/* Wait until RXNE flag is set */
 		MPU650_Timeout = MPU650_LONG_TIMEOUT;
 		while(I2C_GetFlagStatus(MPU650_I2C, I2C_ISR_RXNE) == RESET)
 		{
 			if((MPU650_Timeout--) == 0) return MPU650_TIMEOUT_UserCallback();
 		}

 		/* Read data from RXDR */
 		*pBuffer = I2C_ReceiveData(MPU650_I2C);
 		/* Point to the next location where the byte read will be saved */
 		pBuffer++;

 		/* Decrement the read bytes counter */
 		NumByteToRead--;
 	}

 	/* Wait until STOPF flag is set */
 	MPU650_Timeout = MPU650_LONG_TIMEOUT;
 	while(I2C_GetFlagStatus(MPU650_I2C, I2C_ISR_STOPF) == RESET)
 	{
 		if((MPU650_Timeout--) == 0) return MPU650_TIMEOUT_UserCallback();
 	}

 	/* Clear STOPF flag */
 	I2C_ClearFlag(MPU650_I2C, I2C_ICR_STOPCF);

 	/* If all operations OK */
 	return MPU650_OK;
}


#warning "TODO: default time-out handler!"
/* Default time-out handler */
static uint32_t MPU650_TIMEOUT_UserCallback(){
	while (1){

	}

}
