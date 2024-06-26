/*
 * error.c
 *
 *  Created on: Nov 3, 2023
 *      Author: archuser
 */


/* This file contains error handlers for several situations */
#include "main.h"
#include "timers.h"
#include "error.h"

/* Hardware error handlers */


/* Drone could not fly without IMU */
void IMU_test_error(){
	RED0_LED_ON; RED1_LED_ON; BLUE0_LED_ON;
	while (1){
		SysTick_Delay(200);
	}
}

/* Use HSI and enable red led */
void RCC_error(void){
	RCC->CR |= RCC_CR_HSION;
	while (!(RCC->CR & RCC_CR_HSIRDY)){}

	if (!(RCC->AHBENR | RCC_AHBENR_GPIOEEN)){
		RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
	}
	GPIOE->MODER |= GPIO_MODER_MODER13_0;
	while (1){
		SET_BIT(GPIOE->ODR, GPIO_ODR_13);
		for (uint16_t i = 0; i < UINT16_MAX; ++i);
		CLEAR_BIT(GPIOE->ODR, GPIO_ODR_13);
		for (uint16_t i = 0; i < UINT16_MAX; ++i);
	}
}

