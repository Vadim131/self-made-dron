/*
 * sim800c.c
 *
 *  Created on: 29 дек. 2023 г.
 *      Author: archuser
 */


#include "sim800l.h"
#include "main.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_usart.h"
#include "stm32f30x_gpio.h"
#include "malloc.h"

static void USART_LowLevel_Init();


void SIM800L_GSM_Init(){
	USART_LowLevel_Init();

}

/*void SIM800L_SMS_Send(){

	UART_SendStr(USART1, "AT+CMGF=1");
	SysTick_Delay(1000);
	UART_SendStr(USART1, "AT+CMGS=\"+79134571019\"\r");
	SysTick_Delay(1000);
	UART_SendStr(USART1, "This msg is from sim800c");
	SysTick_Delay(100);
	UART_SendStr(USART1, (char)26);
	SysTick_Delay(100);
}*/


static uint8_t buffer[USART_AT_MAX_LENGTH];
static uint8_t i = 0;
uint8_t byte = 0;

void USART1_IRQHandler(void){
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET){
	    USART_ReceiveData(USART1);

	    if (i == 0){
	    	BLUE1_LED_ON;
	    }
	    if (i == 1){
	    	ORANGE0_LED_ON;
	    }
	    if (i == 2){
	    	GREEN0_LED_ON;
	    }

		i++;
		if (i > 2){
			i = 0;
		}
	}


}

void USART_get_answ(uint8_t* str){
	uint8_t c = 0;
	for (; c <= i; c++){
		str[c] = buffer[c];
	}
	i = 0;
}

void USART_SendStr(USART_TypeDef* USARTx, const uint8_t* s, uint32_t len){
	uint32_t p = 0;
	while (p < len){
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
		USART_SendData(USARTx, s[p]);
		p++;
	}
}

static void USART_IT_Init(){
	NVIC_InitTypeDef NVIC_InitStructure;
	  /* Configure the Priority Group to 2 bits */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	  /* Enable the USARTx Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; //прерывание по uart1
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //задаем приоритет в группе
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //задаем приоритет в подгруппе
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //разрешаем прерывание
	NVIC_Init(&NVIC_InitStructure); //инициализируем

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	//USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

}

static void USART_LowLevel_Init(){

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);

	/* ENABLE USART clock */
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	// PС4 -> TX UART
	GPIOC->MODER |= GPIO_MODER_MODER4_1; /* AF, push-pull */
	GPIOC->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR4_0 | GPIO_OSPEEDER_OSPEEDR4_1); /* max speed */
	GPIOC->AFR[0] |= (7U << 16U); /* AF7 */
	//GPIOC->PUPDR |= GPIO_PUPDR_PUPDR4_1;

	//PС5  -> RX UART, AF, push-pull
	GPIOC->MODER |= GPIO_MODER_MODER5_1; /* AF, push-pull */
	GPIOC->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR5_0 | GPIO_OSPEEDER_OSPEEDR5_1); /* max speed */
	GPIOC->AFR[0] |= (7U << 20U); /* AF7 */
	//GPIOC->PUPDR |= GPIO_PUPDR_PUPDR5_1;

	/* Configure USART */
	USART_InitTypeDef USART_InitStructure;
	USART_StructInit(&USART_InitStructure);

	/* UART mode (RX, TX) */
	USART_InitStructure.USART_Mode = (USART_Mode_Rx | USART_Mode_Tx);
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

	USART_Init(USART1, &USART_InitStructure);

	/* USART Start */
	USART_Cmd(USART1, ENABLE);

	USART_IT_Init();
	NVIC_EnableIRQ(USART1_IRQn);

}

uint32_t SIM800L_TIMEOUT_UserCallback(void){
  /* Block communication and all processes */
  while (1)
  {
  }
}

