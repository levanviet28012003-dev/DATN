#include "EXTI.h"
#include "AFIO.h"

void EXTI0_IRQHandler(void){
		if(EXTI->PR.BITS.BIT0){
			//Doing sth
			EXTI->PR.REG |= (uint32_t)(1 << 0);
		}
}

uint8_t get_Pin_Number(uint16_t gpio_pin){
	for(uint8_t i = 0 ; i < 16 ;i++){
		if(gpio_pin & (1 << i)){
			return i;
		}
	}
	return 0xFF;
}

void EXTI_Init(uint16_t gpio_pin, volatile GPIO_Typedef *Port, uint8_t type){
	uint8_t port_code;
	uint8_t pin;
	if(Port == GPIOA){
		port_code = 0;
	}else if(Port == GPIOB){
		port_code = 1;
	}else if(Port == GPIOC){
		port_code = 2;
	}else{
		return ;
	}
	
	pin = get_Pin_Number(gpio_pin); // thu tu cua bit
	uint32_t shift = (pin % 4) * 4;
	
	if(pin < 4 ){
	  AFIO->EXTICR1.REG  &= (uint32_t)~(0xF << shift);
		AFIO->EXTICR1.REG |= (uint32_t)(port_code << shift);
	}else if(pin < 8){
		AFIO->EXTICR2.REG  &= (uint32_t)~(0xF << shift);
		AFIO->EXTICR2.REG |= (uint32_t)(port_code << shift);
	}else if(pin < 12){
		AFIO->EXTICR3.REG  &= (uint32_t)~(0xF << shift);
		AFIO->EXTICR3.REG |= (uint32_t)(port_code << shift);
	}else {
		AFIO->EXTICR4.REG  &= (uint32_t)~(0xF << shift);
		AFIO->EXTICR4.REG |= (uint32_t)(port_code << shift);
	}
	
	EXTI->IMR.REG |= (1 << pin);
	
	if(type == EXTI_RISING_MODE){
		EXTI->RTSR.REG |= (1 << pin);
		EXTI->FTSR.REG &= ~(1 << pin);
	}else if(type == EXTI_FALLING_MODE){
		EXTI->FTSR.REG |= (1 << pin);
		EXTI->RTSR.REG &= ~(1 << pin);
	}else{
		EXTI->RTSR.REG |= (1 << pin);
		EXTI->FTSR.REG |= (1 << pin);
	}
	
	if(pin <= 4){
		NVIC_ISER0 |= (1 << (6 + pin));
	}else if(pin <= 9){
		NVIC_ISER0 |= (1 << 23);
	}else if(pin <= 15){
		NVIC_ISER1 |= (1 << (40 - 32));
	}

}

void NVIC_UART_En(void){
	NVIC_ISER1 |= (1 << (37 - 32));
}

void NVIC_USB_En(void){
	NVIC_ISER0 |= (1 << 20); 
}
