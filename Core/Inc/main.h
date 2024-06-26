/*
 * main.h
 *
 *  Created on: 20 июл. 2023 г.
 *      Author: archuser
 */

#ifndef SRC_MAIN_H_
#define SRC_MAIN_H_
/* Include CMSIS files */
#include "stm32f30x.h"
#include "stdbool.h"
#include "stddef.h"
#include "stdio.h"
#include "string.h"
#include "malloc.h"


/*
void RaiseCriticalError();
void EXTI_init(void);
*/

/** defines **/
/* LEDS */
#define BLUE0_LED_ON SET_BIT(GPIOE->ODR, GPIO_ODR_8)
#define BLUE0_LED_OFF CLEAR_BIT(GPIOE->ODR, GPIO_ODR_8)
#define RED0_LED_ON SET_BIT(GPIOE->ODR, GPIO_ODR_9)
#define RED0_LED_OFF CLEAR_BIT(GPIOE->ODR, GPIO_ODR_9)
#define ORANGE0_LED_ON SET_BIT(GPIOE->ODR, GPIO_ODR_10)
#define ORANGE0_LED_OFF CLEAR_BIT(GPIOE->ODR, GPIO_ODR_10)
#define GREEN0_LED_ON SET_BIT(GPIOE->ODR, GPIO_ODR_11)
#define GREEN0_LED_OFF CLEAR_BIT(GPIOE->ODR, GPIO_ODR_11)
#define BLUE1_LED_ON SET_BIT(GPIOE->ODR, GPIO_ODR_12)
#define BLUE1_LED_OFF CLEAR_BIT(GPIOE->ODR, GPIO_ODR_12)
#define RED1_LED_ON SET_BIT(GPIOE->ODR, GPIO_ODR_13)
#define RED1_LED_OFF CLEAR_BIT(GPIOE->ODR, GPIO_ODR_13)
#define ORANGE1_LED_ON SET_BIT(GPIOE->ODR, GPIO_ODR_14)
#define ORANGE1_LED_OFF CLEAR_BIT(GPIOE->ODR, GPIO_ODR_14)
#define GREEN1_LED_ON SET_BIT(GPIOE->ODR, GPIO_ODR_15)
#define GREEN1_LED_OFF CLEAR_BIT(GPIOE->ODR, GPIO_ODR_15)
/* Using for ARMED status */
#define ARMED_STATUS_LED_ON ORANGE0_LED_ON; ORANGE1_LED_ON; BLUE0_LED_ON; BLUE1_LED_ON;
#define ARMED_STATUS_LED_OFF ORANGE0_LED_OFF; ORANGE1_LED_OFF; BLUE0_LED_OFF; BLUE1_LED_OFF;
/* SYSCLCK */
#define SYSCLCK 72000000U // Hz
#define APB1CLCK 36000000U
#define APB2CLCK SYSCLCK

#define USER_BUTTON_STATE_ON (GPIOA->IDR & GPIO_IDR_0)
#define ENABLE_TIMx(TIMx) SET_BIT(TIMx->CR1, TIM_CR1_CEN);
#define DISABLE_TIMx(TIMx) CLEAR_BIT(TIMx->CR1, TIM_CR1_CEN);


typedef struct imu_angles {
	float Roll; /* X angle */
	float Pictch; /* Y angle */
	float Yaw; /* Z angle */
} IMU_ANGLES;

#endif /* SRC_MAIN_H_ */
