#include "UART.h"
#include "GPIO.h"
#include "EXTI.h"
void USART1_IRQHandler(void){
	if(USART1_SR & (1 << 5)){
		//Doing sth
		char c = (char)(USART1_DR & (0xFF));
		UART1_SendChar(c);
	}
}

void UART1_Init(void){
	
	  GPIO_Config(GPIOA,GPIO_PIN_9,GPIO_MODE_AF_PP);
	  GPIO_Config(GPIOA,GPIO_PIN_10,GPIO_MODE_INPUT_FLOATING);
	
    USART1_BRR = 0x0341;

    USART1_CR1 |= (1 << 13);  
    USART1_CR1 |= (1 << 3);   
    USART1_CR1 |= (1 << 2);

		USART1_CR1 |= (1 << 5);
		NVIC_UART_En();

}
void UART1_SendChar(char c) {
    while (!(USART1_SR & (1 << 7))); 
    USART1_DR = c;
}

void UART1_SendString(const char *str) {
    while (*str) {
        UART1_SendChar(*str++);
    }
}


	
	