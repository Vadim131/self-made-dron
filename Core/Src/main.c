

/**
 **********************************************************
 * This is basic flight-firmware, which was written 	  *
 * from scratch only for stm32F3Discovery board and 	  *
 * it's stm32F303VCT6 processor							  *
 * @author vadikdav12345@gmail.com						  *
 * @license GNU GPL v.3									  *
 * @file main.c									  		  *
 **********************************************************
 **/

/* This is main file, there is located some basic stuff. Start reading code from main */
#include "sim800l.h"
#include "main.h"
#include "imu.h"
#include "error.h"
#include "timers.h"
#include "inet.h"

/* Static function defines */
static bool RCC_Init(void);
static void Enable_FPU(void);
static void Enable_DWT(void);
static void Enable_Stuff(void);
/* Static function defines end */

/* Static variables defines */
static bool ARMED_FLAG;
static uint32_t dt = 0;

/* Static variables defines end */

/* Using 8 Mhz crystal, PLLMUL = 9, SYSCLCK = PLLCLCK = 72 Mhz.
 * Return 1 in case of error */
static bool RCC_Init(void){

	/* Using 8Mhz crystal */
	RCC->CR |= RCC_CR_HSEBYP | RCC_CR_HSEON;
	/* Wait for crystal */
	uint16_t i = 0;
	while (!(RCC->CR & RCC_CR_HSERDY)){
		if (i == UINT16_MAX){
			return 1;
		}
		++i;
	}
	/* Flash 48 <= HCLK <= 72 */
	FLASH->ACR |= FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_1;
	/* PLL input = 8Mhz, output = 72Mhz	 */
	RCC->CFGR |= RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9;
	RCC->CR |= RCC_CR_PLLON;
	i = 0;
	/* Wait for PLL */
	while (!(RCC->CR & RCC_CR_PLLRDY)){
		if (i == UINT16_MAX){
			return 1;
		}
		++i;
	}
	/* Divide PLL for tires */
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
	/* Switch on PLL */
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	/* Wait for switching on PLL */
	i = 0;
	while (!(RCC->CFGR & RCC_CFGR_SWS_PLL)){
		if (i == UINT16_MAX){
			return 1;
		}
		++i;
	}
	/* Disable HSI */
	RCC->CR &= ~(RCC_CR_HSION);
	return 0;
}

/* Exti handler for button */
void EXTI0_IRQHandler(void){
	/* Reset EXTI flag */
	EXTI->PR = EXTI_PR_PR0;
	for (uint8_t i = 0; i < UINT8_MAX; ++i){}
	if (USER_BUTTON_STATE_ON){
		ARMED_FLAG = ARMED_FLAG ? 0 : 1;
	}

#ifdef SHOW_ARMED_STATE
	if (ARMED_FLAG){
		ARMED_STATUS_LED_ON;

	}
	else {
		ARMED_STATUS_LED_OFF;
	}
#endif
}

static void Init_EXTI(void){
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	SYSCFG->EXTICR[0] |= (0x000<<SYSCFG_EXTICR1_EXTI0);
	EXTI->RTSR |= EXTI_RTSR_TR0;
	EXTI->PR = EXTI_PR_PR0;
	EXTI->IMR |= EXTI_IMR_MR0;
	NVIC_EnableIRQ(EXTI0_IRQn);

}

static void Enable_FPU(void){
	SCB->CPACR |= ((3U << 20)|(3U << 22));
}

static void Enable_DWT(void){
	/* DWT init */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

}

/* GPIOA, GPIOB, GPIOE + button & LEDS*/
static void Enable_Stuff(void){
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOEEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;
	/* All LEDS */
	GPIOE->MODER |= 1431633920;
	/* User button & EXTI */
	GPIOA->PUPDR |= (2U << 0);

}

static void Enable_GSM(){
	SIM800L_GSM_Init();
}



void flightloop(){

	while (1){
		if (ARMED_FLAG){

		}
		else {
			SysTick_Delay(100);
		}
	}
}

int main(void ){
	if (RCC_Init() || SysTick_Init(SYSCLCK/1000)){
		RCC_error();
	}
	Enable_Stuff();
	Enable_FPU();
	Enable_DWT();
	Init_EXTI();
	/*
	ENABLE_TIMx(TIM2);
	TIM2_Init(35999, 1);
	*/

	/* Prepare IMU system on MPU-6050*/
	MPU650_Init();
	if (IMU_MPU6050_Test()){
		IMU_test_error();
	}
	IMU_Gyro_get_meas_error();
	IMU_Accel_get_meas_error();
	/* IMU system is ready */
	BLUE0_LED_ON;
	SysTick_Delay(1000);
	BLUE0_LED_OFF;

	/* Prepare control system */
	//SIM800L_GSM_Init(); /* usart1 */

	IMU_ANGLES Angles = {0, 0, 0};

	float gX, gY, gZ;
	float aX, aY, aZ;
	float delta_time;
	while (1){
		if (ARMED_FLAG){

			/* Get accelerometer measurements */
			IMU_Get_Accel_XYZ(&aX, &aY, &aZ);
			/* Get gyroscope measurements */
			IMU_Get_Gyro_XYZ(&gX, &gY, &gZ);

			delta_time = DWT_Stop_time();

			IMU_get_angles(0.01f, gX, gY, gZ, aX, aY, aZ, &Angles);

			DWT_Start_time();

			GREEN1_LED_OFF;
			GREEN0_LED_OFF;
			RED0_LED_OFF;
			RED1_LED_OFF;

			if (Angles.Roll > 0){
				GREEN0_LED_ON;
			}
			else {
				GREEN1_LED_ON;
			}

			SysTick_Delay(10);
		}


	}
	//flightloop();


	return 0;
}
