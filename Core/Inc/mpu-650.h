/*
 * mpu-650.h
 *
 *  Created on: 2 нояб. 2023 г.
 *      Author: archuser
 *
 * This file was written in aims to have API with MPU-6050 board. It has the same structure as
 * file "stm32f3_discovery_lsm303dlhc.h", cause lsm303dlhc was broken and there is nothing no do
 * with it.
 */

#ifndef INC_MPU_650_H_
#define INC_MPU_650_H_


#define MPU650_I2C_ADDRESS				 ((uint8_t)0x68)
#define MPU650_READ_ADDRESS				 ((uint8_t)((MPU650_I2C_ADDRESS & 0x7f) << 1))
#define MPU650_WRITE_ADDRESS			 ((uint8_t)(MPU650_I2C_ADDRESS & 0b01111111))
#define MPU650_I2C                       I2C1

#define MPU650_I2C_SCK_PIN             	 GPIO_Pin_8
#define MPU650_I2C_SCK_GPIO_PORT         GPIOB
#define MPU650_I2C_SCK_GPIO_CLK          RCC_AHBPeriph_GPIOB
#define	MPU650_I2C_SCK_SOURCE            GPIO_PinSource8
#define MPU650_I2C_SCK_AF                GPIO_AF_4

#define MPU650_I2C_SDA_PIN               GPIO_Pin_9
#define	MPU650_I2C_SDA_GPIO_PORT         GPIOB
#define MPU650_I2C_SDA_GPIO_CLK          RCC_AHBPeriph_GPIOB
#define	MPU650_I2C_SDA_SOURCE            GPIO_PinSource9
#define MPU650_I2C_SDA_AF                GPIO_AF_4


/* MPU-6050 registers */
#define MPU650_WHO_AM_I 				((uint8_t)0x75)

#define MPU650_SMPLRT_DIV				((uint8_t)0x19)
#define MPU650_CONFIG					((uint8_t)0x1A)
#define MPU650_GYRO_CONFIG				((uint8_t)0x1B)
#define MPU650_ACCEL_CONFIG				((uint8_t)0x1C)
#define MPU650_MOT_THR					((uint8_t)0x1F)
#define MPU650_FIFO_EN					((uint8_t)0x23)
#define MPU650_I2C_MST_CTRL				((uint8_t)0x24)
#define MPU650_I2C_SLV0_ADDR			((uint8_t)0x25)
#define MPU650_I2C_SLV0_REG				((uint8_t)0x26)
#define MPU650_I2C_SLV0_CTRL			((uint8_t)0x27)
#define MPU650_I2C_SLV1_ADDR			((uint8_t)0x28)
#define MPU650_I2C_SLV1_REG				((uint8_t)0x29)
#define MPU650_I2C_SLV1_CTRL			((uint8_t)0x2A)
#define MPU650_I2C_SLV2_ADDR			((uint8_t)0x2B)
#define MPU650_I2C_SLV2_REG				((uint8_t)0x2C)
#define MPU650_I2C_SLV2_CTRL			((uint8_t)0x2D)
#define MPU650_I2C_SLV3_ADDR			((uint8_t)0x2E)
#define MPU650_I2C_SLV3_REG				((uint8_t)0x2F)
#define MPU650_I2C_SLV3_CTRL			((uint8_t)0x30)
#define MPU650_I2C_SLV4_ADDR			((uint8_t)0x31)
#define MPU650_I2C_SLV4_REG				((uint8_t)0x32)
#define MPU650_I2C_SLV4_DO				((uint8_t)0x33)
#define MPU650_I2C_SLV4_CTRL			((uint8_t)0x34)
#define MPU650_I2C_SLV4_DI				((uint8_t)0x35)
#define MPU650_I2C_MST_STATUS			((uint8_t)0x36)
#define MPU650_INT_PIN_CFG 				((uint8_t)0x37)
#define MPU650_INT_ENABLE				((uint8_t)0x38)
#define MPU650_INT_STATUS 				((uint8_t)0x3A)
#define MPU650_ACCEL_XOUT_H				((uint8_t)0x3B)
#define MPU650_ACCEL_XOUT_L				((uint8_t)0x3C)
#define MPU650_ACCEL_YOUT_H				((uint8_t)0x3D)
#define MPU650_ACCEL_YOUT_L				((uint8_t)0x3E)
#define MPU650_ACCEL_ZOUT_H				((uint8_t)0x3F)
#define MPU650_ACCEL_ZOUT_L				((uint8_t)0x40)
#define MPU650_TEMP_OUT_H				((uint8_t)0x41)
#define MPU650_TEMP_OUT_L				((uint8_t)0x42)
#define MPU650_GYRO_XOUT_H				((uint8_t)0x43)
#define MPU650_GYRO_XOUT_L				((uint8_t)0x44)
#define MPU650_GYRO_YOUT_H				((uint8_t)0x45)
#define MPU650_GYRO_YOUT_L				((uint8_t)0x46)
#define MPU650_GYRO_ZOUT_H				((uint8_t)0x47)
#define MPU650_GYRO_ZOUT_L				((uint8_t)0x48)
#define MPU650_EXT_SENS_DATA_00			((uint8_t)0x49)
#define MPU650_EXT_SENS_DATA_01			((uint8_t)0x4A)
#define MPU650_EXT_SENS_DATA_02			((uint8_t)0x4B)
#define MPU650_EXT_SENS_DATA_03			((uint8_t)0x4C)
#define MPU650_EXT_SENS_DATA_04			((uint8_t)0x4D)
#define MPU650_EXT_SENS_DATA_05			((uint8_t)0x4E)
#define MPU650_EXT_SENS_DATA_06			((uint8_t)0x4F)
#define MPU650_EXT_SENS_DATA_07			((uint8_t)0x50)
#define MPU650_EXT_SENS_DATA_08			((uint8_t)0x51)
#define MPU650_EXT_SENS_DATA_09			((uint8_t)0x52)
#define MPU650_EXT_SENS_DATA_10			((uint8_t)0x53)
#define MPU650_EXT_SENS_DATA_11			((uint8_t)0x54)
#define MPU650_EXT_SENS_DATA_12			((uint8_t)0x55)
#define MPU650_EXT_SENS_DATA_13			((uint8_t)0x56)
#define MPU650_EXT_SENS_DATA_14			((uint8_t)0x57)
#define MPU650_EXT_SENS_DATA_15			((uint8_t)0x58)
#define MPU650_EXT_SENS_DATA_16			((uint8_t)0x59)
#define MPU650_EXT_SENS_DATA_17			((uint8_t)0x5A)
#define MPU650_EXT_SENS_DATA_18			((uint8_t)0x5B)
#define MPU650_EXT_SENS_DATA_19			((uint8_t)0x5C)
#define MPU650_EXT_SENS_DATA_20			((uint8_t)0x5D)
#define MPU650_EXT_SENS_DATA_21			((uint8_t)0x5E)
#define MPU650_EXT_SENS_DATA_22			((uint8_t)0x5F)
#define MPU650_EXT_SENS_DATA_23			((uint8_t)0x60)
#define MPU650_I2C_SLV0_DO				((uint8_t)0x63)
#define MPU650_I2C_SLV1_DO				((uint8_t)0x64)
#define MPU650_I2C_SLV2_DO				((uint8_t)0x65)
#define MPU650_I2C_SLV3_DO				((uint8_t)0x66)
#define MPU650_I2C_MST_DELAY_CTRL		((uint8_t)0x67)
#define MPU650_I2C_SIG_PATH_RESET		((uint8_t)0x68)
#define MPU650_MOT_DETECT_CTRL			((uint8_t)0x69)
#define MPU650_USER_CTRL				((uint8_t)0x6A)
#define MPU650_PWR_MGMT_1				((uint8_t)0x6B)
#define MPU650_PWR_MGMT_2				((uint8_t)0x6C)
#define MPU650_FIFO_COUNTH				((uint8_t)0x72)
#define MPU650_FIFO_COUNTL 				((uint8_t)0x73)
#define MPU650_FIFO_R_W 				((uint8_t)0x74)

/* needed preferences */
/* Reg MPU650_SMPLRT_DIV */
#define MPU650_SMPLRT_DIV_scale      		((uint8_t)0b00000000)       // 8000Hz
/* Reg MPU650_CONFIG */
#define MPU650_DLPF_4 						((uint8_t)0b00000100)
#define MPU650_SYNK_accel_zout				((uint8_t)0b00111000)
/* Reg MPU650_ACCEL_CONFIG */
#define MPU650_SCALERANGE					((uint8_t)0b11101000)
/* Reg MPU650_PWR_MGMT_1 */
#define MPU650_PWR_1_CLKSEL					((uint8_t)0b00000000)
#define MPU650_PWR_1_TEMPSENSOR				((uint8_t)0b00001000)
#define MPU650_PWR_1_CYCLE					((uint8_t)0b00000000)
/* Reg MPU_650_PWR_MGMT_2 */
#define MPU650_PWR_2_WAKEUP_FREQ			((uint8_t)0b10000000)
#define MPU650_PWR_2_STB					((uint8_t)0b00000111)

#define MPU650_GYRO_250dps					((float)(131))


#define MPU650_OK						((uint32_t)0x0)

#define MPU650_FLAG_TIMEOUT             ((uint32_t)0x1000)
#define MPU650_LONG_TIMEOUT             ((uint32_t)(10 * MPU650_FLAG_TIMEOUT))

void MPU650_Init();
uint16_t MPU650_Write(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer);
uint16_t MPU650_Read(uint8_t DeviceAddr, uint8_t RegAddr,uint8_t* pBuffer, uint16_t NumByteToRead);



#endif /* INC_MPU_650_H_ */
