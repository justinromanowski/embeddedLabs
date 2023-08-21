/**
  ******************************************************************************
  * @file    main.c
  * @author  Justin Romanowski
  * @version V1.0
  * @date    11/27/2022
  * @brief   Lab 6 CPE3000
  ******************************************************************************
*/


#include <stdio.h>
#include "stm32l4xx.h"
#include "stm32l4xx_nucleo.h"
			
// PA.5  <--> Green LED
// PC.13 <--> Blue user button
#define LED_PIN    5
#define BUTTON_PIN 13
volatile int buttonperiod; //Denote period length of LED pulse (in seconds)
volatile int buttonflag; //Denote if button has been held for 1s or longer
volatile int buttonif; //holds the value of button if it is pressed down (0) or not pressed down (1)
int mask = (1UL<<BUTTON_PIN); //bitwise and with button's IDR

void System_Clock_Init(void){

	uint32_t HSITrim;

	// To correctly read data from FLASH memory, the number of wait states (LATENCY)
  	// must be correctly programmed according to the frequency of the CPU clock
	// (HCLK) and the supply voltage of the device.
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |=  FLASH_ACR_LATENCY_2WS;

	// Enable the Internal High Speed oscillator (HSI)
	RCC->CR |= RCC_CR_HSION;
	while((RCC->CR & RCC_CR_HSIRDY) == 0);
	// Adjusts the Internal High Speed oscillator (HSI) calibration value
	// RC oscillator frequencies are factory calibrated by ST for 1 % accuracy at 25oC
	// After reset, the factory calibration value is loaded in HSICAL[7:0] of RCC_ICSCR
	HSITrim = 16; // user-programmable trimming value that is added to HSICAL[7:0] in ICSCR.
	RCC->ICSCR &= ~RCC_ICSCR_HSITRIM;
	RCC->ICSCR |= HSITrim << 24;

	RCC->CR    &= ~RCC_CR_PLLON;
	while((RCC->CR & RCC_CR_PLLRDY) == RCC_CR_PLLRDY);

	// Select clock source to PLL
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLSRC;
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI; // 00 = No clock, 01 = MSI, 10 = HSI, 11 = HSE

	// Make PLL as 80 MHz
	// f(VCO clock) = f(PLL clock input) * (PLLN / PLLM) = 16MHz * 20/2 = 160 MHz
	// f(PLL_R) = f(VCO clock) / PLLR = 160MHz/2 = 80MHz
	RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_PLLN) | 20U << 8;
	RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_PLLM) | 1U << 4; // 000: PLLM = 1, 001: PLLM = 2, 010: PLLM = 3, 011: PLLM = 4, 100: PLLM = 5, 101: PLLM = 6, 110: PLLM = 7, 111: PLLM = 8

	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLR;  // 00: PLLR = 2, 01: PLLR = 4, 10: PLLR = 6, 11: PLLR = 8
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN; // Enable Main PLL PLLCLK output

	RCC->CR   |= RCC_CR_PLLON;
	while((RCC->CR & RCC_CR_PLLRDY) == 0);

	// Select PLL selected as system clock
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL; // 00: MSI, 01:HSI, 10: HSE, 11: PLL

	// Wait until System Clock has been selected
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

	// The maximum frequency of the AHB, the APB1 and the APB2 domains is 80 MHz.
	RCC->CFGR &= ~RCC_CFGR_HPRE;  // AHB prescaler = 1; SYSCLK not divided
	RCC->CFGR &= ~RCC_CFGR_PPRE1; // APB high-speed prescaler (APB1) = 1, HCLK not divided
	RCC->CFGR &= ~RCC_CFGR_PPRE2; // APB high-speed prescaler (APB2) = 1, HCLK not divided

	// RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM;
	// RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN;
	// RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP;
	// RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLQ;
	// RCC->PLLCFGR |= RCC_PLLCFGR_PLLPEN; // Enable Main PLL PLLSAI3CLK output enable
	// RCC->PLLCFGR |= RCC_PLLCFGR_PLLQEN; // Enable Main PLL PLL48M1CLK output enable

	RCC->CR &= ~RCC_CR_PLLSAI1ON;  // SAI1 PLL enable
	while ( (RCC->CR & RCC_CR_PLLSAI1ON) == RCC_CR_PLLSAI1ON );

	// Configure and enable PLLSAI1 clock to generate 11.294MHz
	// 8 MHz * 24 / 17 = 11.294MHz
	// f(VCOSAI1 clock) = f(PLL clock input) *  (PLLSAI1N / PLLM)
	// PLLSAI1CLK: f(PLLSAI1_P) = f(VCOSAI1 clock) / PLLSAI1P
	// PLLUSB2CLK: f(PLLSAI1_Q) = f(VCOSAI1 clock) / PLLSAI1Q
	// PLLADC1CLK: f(PLLSAI1_R) = f(VCOSAI1 clock) / PLLSAI1R
	RCC->PLLSAI1CFGR &= ~RCC_PLLSAI1CFGR_PLLSAI1N;
	RCC->PLLSAI1CFGR |= 24U<<8;

	// SAI1PLL division factor for PLLSAI1CLK
	// 0: PLLSAI1P = 7, 1: PLLSAI1P = 17
	RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1P;
	RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1PEN;

	// SAI1PLL division factor for PLL48M2CLK (48 MHz clock)
	// RCC->PLLSAI1CFGR &= ~RCC_PLLSAI1CFGR_PLLSAI1Q;
	// RCC->PLLSAI1CFGR |= U<<21;
	// RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1QEN;

	// PLLSAI1 division factor for PLLADC1CLK (ADC clock)
	// 00: PLLSAI1R = 2, 01: PLLSAI1R = 4, 10: PLLSAI1R = 6, 11: PLLSAI1R = 8
	// RCC->PLLSAI1CFGR &= ~RCC_PLLSAI1CFGR_PLLSAI1R;
	// RCC->PLLSAI1CFGR |= U<<25;
	// RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1REN;

	RCC->CR |= RCC_CR_PLLSAI1ON;  // SAI1 PLL enable
	while ( (RCC->CR & RCC_CR_PLLSAI1ON) == 0);

	// SAI1 clock source selection
	// 00: PLLSAI1 "P" clock (PLLSAI1CLK) selected as SAI1 clock
	// 01: PLLSAI2 "P" clock (PLLSAI2CLK) selected as SAI1 clock
	// 10: PLL "P" clock (PLLSAI3CLK) selected as SAI1 clock
	// 11: External input SAI1_EXTCLK selected as SAI1 clock
	RCC->CCIPR &= ~RCC_CCIPR_SAI1SEL;

	RCC->APB2ENR |= RCC_APB2ENR_SAI1EN;
}

void configure_LEDPIN(){
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; //Enable GPIOA clock

	GPIOA->MODER &= ~(3UL<<(2*LED_PIN)); //Reset MODER pin 5
	GPIOA->MODER |= 1UL<<(2*LED_PIN); //Set MODER to output (0b01 = 1UL)

	GPIOA->OSPEEDR &= ~(3UL<<(2*LED_PIN)); //Low output speed (0b00)

	GPIOA->OTYPER &= ~(1UL<<LED_PIN); //Push-pull

	GPIOA->PUPDR &= ~(3UL<<(2*LED_PIN)); //No PU, no PD (0b00)
}

void configure_BUTTONPIN(){
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN; //Enable GPIOC clock

	GPIOC->MODER &= ~(3UL<<(2*BUTTON_PIN)); //Set MODER to input (0b00)

	GPIOC->PUPDR &= ~(3UL<<(2*BUTTON_PIN)); //No PU, no PD (0b00)


	NVIC_EnableIRQ(EXTI15_10_IRQn); 	//GPIO Pin 13, so use EXTI15-10
    	NVIC_SetPriority(EXTI15_10_IRQn, 1); //Set priority to highest

	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;	//connect to GPIO
	SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13;     // SYSCFG external interrupt configuration registers 70
	SYSCFG->EXTICR[3] |=  SYSCFG_EXTICR4_EXTI13_PC;

	EXTI->IMR1 |= EXTI_IMR1_IM13;     // 0 = marked, 1 = not masked (i.e., enabled), enable bit 13

	EXTI->RTSR1 |= EXTI_RTSR1_RT13;  //rising trigger selection, 0=disable 1=enable
	EXTI->FTSR1 |= ~(EXTI_FTSR1_FT13);  //falling trigger selection, 0=disable 1=enable

}

void configure_TIM2_CH1()
{
	RCC->APB1ENR1 		|= RCC_APB1ENR1_TIM2EN;   // Enable timer clock

	// Counting direction: 0 = up-counting, 1 = down-counting
	TIM2->CR1 &= ~TIM_CR1_DIR; //count direction, 0b0 = upcount, 0b1=downcount

	TIM2->PSC = 79999999UL; //prescaler, set to 79999999 for 1KHz clock
	TIM2->ARR = 999UL;   //autoreload, make sure to include -1! for when to restart the count (3E7 = 999 = 1000-1)
	TIM2->CCMR1 &= ~TIM_CCMR1_OC1M;  					// Clear output compare mode bits for channel 1
	TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; // OC1M = 110 for PWM Mode 1 output on ch1
	TIM2->CCMR1 |= TIM_CCMR1_OC1PE;                     // Output 1 preload enable 1: Preload register on TIMx_CCR1 enabled. Read/Write operations access the preloadregister. TIMx_CCR1 preload value is loaded in the active register at each update event.

	TIM2->CCMR1 |= TIM_CCER_CC1NP; //0=active high, 1=active low

	TIM2->CCER |= (1<<1 | 1<<3); // detect rising and falling edges

	TIM2->CCR1  = 500UL;         // Output Compare Register for channel 1
	TIM2->CR1  |= (1UL<<7U); // ARPE = 1 (updates ARR on event)

	//INTERRUPT CONFIG
	TIM2->DIER |= 0x02; // Enable Capture/Compare interrupts for channel 1 & UIF

	NVIC_EnableIRQ(TIM2_IRQn); // Enable interrupt for timer 2
	NVIC_SetPriority(TIM2_IRQn, 2);
}


void TIM2_IRQHandler(void){
	if((TIM2->SR & TIM_SR_UIF) != 0) { //if update event (rising or falling edge)
		LED_Toggle();
		TIM2->SR &= ~TIM_SR_UIF; //clear UIF flag
	}
}

void EXTI15_10_IRQHandler(void){
	if ((EXTI->PR1 & EXTI_PR1_PIF13) != 0) {
		if (buttonflag>=1000) {//if button pressed for 1s or more
			TIM2->CR1  &= ~(TIM_CR1_CEN); // Disable counter
			buttonperiod=0;
			buttonflag=0;
			LED_off();
		} else {
			TIM2->CR1  |= TIM_CR1_CEN; // Enable counter
			if (buttonperiod<=3) { //if loop to signify the period of the button
				buttonperiod++;
				} else {
				buttonperiod=1;
				}
		}

		switch (buttonperiod) { //use a switch to assign ARR & CCR based on button's state
			case 1:
				TIM2->ARR = 999UL; //count to 1000 (1s) w 1KHz clock
				TIM2->CCR1  = 500UL; //falling edge @ 500ms
				break;
			case 2:
				TIM2->ARR = 1999UL;  //count to 2000 (2s) w 1KHz clock
				TIM2->CCR1  = 1000UL; //falling edge @ 1000ms
				break;
			case 3:
				TIM2->ARR = 2999UL;  //count to 3000 (3s) w 1KHz clock
				TIM2->CCR1  = 1500UL; //falling edge @ 1500ms
				break;
			case 4:
				TIM2->ARR = 3999UL;  //count to 4000 (4s) w 1KHz clock
				TIM2->CCR1  = 2000UL; //falling edge @ 2000ms
				break;
			default:
				break;
		}


	EXTI->PR1 |= EXTI_PR1_PIF3; //Clear interrupt flag (comment if want repeated output)
	}
	}

void configure_SysTick(void){
	SysTick->CTRL = 0;	// Enable/disable SysTick IRQ & Counter

	SysTick->LOAD = 80000000/1000 - 1;    //1ms total, default clock is 80MHz & loads on 79999

	SysTick->VAL = 0;	//Start Value

	NVIC_SetPriority(SysTick_IRQn, 0);		// Set Priority to 1
	NVIC_EnableIRQ(SysTick_IRQn);			// Enable SysTick_IRQn interrupt in NVIC

	// Enables SysTick exception request
	// 1 = counting down to zero asserts the SysTick exception request
	// 0 = counting down to zero does not assert the SysTick exception request
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;	//1=processor clock, 0= external clock
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; 	// Enable SysTick IRQ and SysTick Timer
}

void SysTick_Handler(void){
	buttonif = (GPIOC->IDR & mask)	>> BUTTON_PIN;
	if (buttonif == 0){ //if GPIOC active, then increase counter (buttonflag)
		buttonflag++;
	}
	else {
		buttonflag = 0; //if GPIOC is not active, then set counter (buttonflag) to 0
	}
}

void LED_on(){
	GPIOA->ODR |= 1 << LED_PIN;
}

void LED_off(){
	GPIOA->ODR &= ~(1 << LED_PIN);
}

void LED_Toggle(void)
{
	GPIOA->ODR ^= (1UL<<LED_PIN);
}

int main(void){
	//set all button attributes to 0
	buttonflag=0;
	buttonperiod=0;
	//initialize systemclock, LED GPIO, Button GPIO, General Timer 2, & SysTick
	System_Clock_Init();
	configure_LEDPIN();
	configure_BUTTONPIN();
	configure_TIM2_CH1(); //Used to control the button's clock pulse
	configure_SysTick();
	LED_off();

	while(1);
}
