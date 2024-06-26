/*
 * timers.c
 *
 *  Created on: 27 дек. 2023 г.
 *      Author: archuser
 */


/* This file contains timers functions and timer's handlers*/
#include "main.h"
#include "timers.h"


/* SysTick */
static uint32_t MSDELAY;

bool SysTick_Init(uint32_t ticks){
	/* LOAD register has only 24 bits */
	if (ticks > 0x00FFFFFF || ticks < 1) return 1;
	SysTick->LOAD = ticks;
	SysTick->VAL = 0; /* Writing any number would clear VAL register */
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk
			| SysTick_CTRL_TICKINT_Msk
			| SysTick_CTRL_ENABLE_Msk;
	return 0;

}

void SysTick_Handler(void){
	if (MSDELAY > 0) --MSDELAY;
}

/* This function uses SYSTICK to delay in ms. Be careful!
 * Be aware of other interruptions!
 */
void SysTick_Delay(uint32_t ms){
	MSDELAY = ms;
	SysTick->VAL = 0;
	while(MSDELAY);
}


/* Timer 2*/

static uint32_t TIM2_dt = 0; /* 0,1 ms */
void TIM2_IRQHandler(void){
	if (TIM2->SR, TIM_SR_UIF){
	    TIM2->SR  &= ~TIM_SR_UIF;
	}
	TIM2_dt += 1;
	if (TIM2_dt%2 == 0){
		GREEN0_LED_ON;
	}
	else {
		GREEN0_LED_OFF;
	}
	//Gyro_Get_XYZspeed(&Gyro_Xspeed, &Gyro_Yspeed , &Gyro_Zspeed);
	//Flight_Set_Real_Pos(Gyro_Xspeed*(float)0.01, Gyro_Yspeed*(float)0.01, Gyro_Zspeed*(float)0.01);

}

uint32_t TIM2_Get_dt(){
	return TIM2_dt;

}

void TIM2_dt_reset(){
	TIM2_dt = 0;
}

void TIM2_Init(uint16_t prescaler, uint32_t reloadvalue){
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	TIM2->CR1 |= (1<<TIM_CR1_DIR);
	TIM2->DIER |= TIM_DIER_UIE;
	TIM2->PSC = prescaler;
	TIM2->ARR = reloadvalue;
	NVIC_EnableIRQ(TIM2_IRQn);
	ENABLE_TIMx(TIM2);

}

void DWT_Start_time(){
	DWT->CYCCNT = 0;
}

/* Returns ~time period in seconds */
float DWT_Stop_time(){
	return (float)DWT->CYCCNT / (float)SYSCLCK;
}


