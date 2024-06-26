/*
 * timers.h
 *
 *  Created on: 27 дек. 2023 г.
 *      Author: archuser
 */

#ifndef SRC_TIMERS_H_
#define SRC_TIMERS_H_

void SysTick_Delay(uint32_t ms);
bool SysTick_Init(uint32_t ticks);

void TIM2_Init(uint16_t prescaler, uint32_t reloadvalue);
uint32_t TIM2_Get_dt(void);
void TIM2_dt_reset();

void DWT_Start_time(void);
float DWT_Stop_time(void);

#endif /* SRC_TIMERS_H_ */
