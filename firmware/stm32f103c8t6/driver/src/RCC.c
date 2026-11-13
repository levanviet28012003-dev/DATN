#include "RCC.h"

void RCC_Config_72Mhz(void){
	//Enable HSE
	RCC->CR.BITS.HSEON = 1;
	while(!RCC->CR.BITS.HSERDY){}
	
	ACR->BITS.LATENCY = 2;
		
	RCC->CFGR.BITS.PLLSRC = 1;
	RCC->CFGR.BITS.PLLMUL = 7;//0111 x9 PLL
  RCC->CFGR.BITS.USBPRE = 0;
	RCC->CR.BITS.PLLON = 1;
	while(!RCC->CR.BITS.PLLRDY){}
	
	//AHB
	RCC->CFGR.BITS.HPRE = 0;// 1
	//APB1
	RCC->CFGR.BITS.PPRE1 = 4;// 2
	//APB2
	RCC->CFGR.BITS.PPRE2 = 0;// 1
	
	RCC->CFGR.BITS.SW = 2;
	while(RCC->CFGR.BITS.SWS != 2){}
}

void RCC_Enable_PortA(void){
	RCC->APB2ENR.BITS.IOPAEN = 1;
}

void RCC_Enable_PortB(void){
	RCC->APB2ENR.BITS.IOPBEN = 1;
}

void RCC_Enable_PortC(void){
	RCC->APB2ENR.BITS.IOPCEN = 1;
}
void RCC_Enable_AFIO(void){
	RCC->APB2ENR.BITS.AFIOEN = 1;
}

void RCC_Enable_TIM2(void){
	RCC->APB1ENR.BITS.TIM2EN = 1;
}

void RCC_Enable_ADC01(void){
	RCC->APB2ENR.BITS.ADC1EN = 1;
}

void RCC_Enable_UART1(void){
	RCC->APB2ENR.BITS.USART1EN = 1;
}
void RCC_Enable_SPI1(void){
	RCC->APB2ENR.BITS.SPI1EN = 1;
}

void RCC_Enable_I2C1(void){
	RCC->APB1ENR.BITS.I2C1EN = 1;
}
void RCC_Enable_USB(void){
	RCC->APB1ENR.BITS.USBEN = 1;
}