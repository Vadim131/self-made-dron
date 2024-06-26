/*
 * sim800c.h
 *
 *  Created on: 29 дек. 2023 г.
 *      Author: archuser
 */

#ifndef SRC_SIM800C_H_
#define SRC_SIM800C_H_

#include "stm32f30x.h"


/* Functions */
void SIM800L_GSM_Init();
void USART_SendStr(USART_TypeDef* USARTx, const uint8_t* s, uint32_t len);
void USART_SendByte(USART_TypeDef* USARTx, uint8_t byte);
uint32_t SIM800L_TIMEOUT_UserCallback(void);
void USART_get_answ(uint8_t* str);

/* test */
void SIM800L_SMS_Send();

/* TX */
#define SIM800L_USART_TX_PIN 				GPIO_Pin_4
#define SIM800L_USART_TX_PORT 				GPIOC
#define SIM800L_USART_TX_GPIO_CLK 			RCC_AHBPeriph_GPIOC;
#define SIM800L_USART_TX_SOURCE 			GPIO_PinSource4
#define SIM800L_USART_TX_AF					GPIO_AF_7

/* RX */
#define SIM800L_USART_RX_PIN 				GPIO_Pin_5
#define SIM800L_USART_RX_PORT 				GPIOC
#define SIM800L_USART_RX_GPIO_CLK 			RCC_AHBPeriph_GPIOC;
#define SIM800L_USART_RX_SOURCE 			GPIO_PinSource5
#define SIM800L_USART_RX_AF					GPIO_AF_7

#define USART_AT_MAX_LENGTH					((uint8_t)255)
#define USART_TIMEOUT             			((uint32_t)0x1000)

#endif /* SRC_SIM800C_H_ */
