#include "GPIO.h"

void GPIO_Config(volatile GPIO_Typedef *GPIOx, uint16_t pin, uint32_t mode){
	uint32_t position = 0;
	uint32_t config = 0;
	for(position = 0 ; position < 16 ; position++){
		if(pin & (1UL << position)){
			switch(mode){
				case GPIO_MODE_OUTPUT_PP:
					config = (0x03 << 0) | (0x00 << 2);
					break;
				case GPIO_MODE_OUTPUT_OD:
					config = (0x03 << 0) | (0x01 << 2);
					break;
				case GPIO_MODE_AF_PP:
					config = (0x03 << 0) | (0x02 << 2);
					break;
				case GPIO_MODE_AF_OD:
					config = (0x03 << 0) | (0x03 << 2);
					break;
				case GPIO_MODE_INPUT_ANALOG:
					config = (0x00 << 2);
					break;
				case GPIO_MODE_INPUT_FLOATING:
					config = (0x01 << 2);
					break;
				case GPIO_MODE_INPUT_PU:
					config = (0x02 << 2);
					GPIOx->ODR.REG |= (1UL << position);
					break;
				case GPIO_MODE_INPUT_PD:
					config = (0x02 << 2);
					GPIOx->ODR.REG &= ~(1UL << position);
					break;
			}
			if (mode == GPIO_MODE_INPUT_FLOATING ||
                mode == GPIO_MODE_INPUT_PD || mode == GPIO_MODE_INPUT_PU) {
                config |= 0x00; 
            }
			
			// Ghi mode một lần, tránh chuyển tạm sang analog giữa hai lần ghi.
			if(position < 8){
				GPIOx->CRL.REG = (GPIOx->CRL.REG & ~(0xFUL << (position * 4))) |
					(config << (position * 4));
			}else{
				GPIOx->CRH.REG = (GPIOx->CRH.REG & ~(0xFUL << ((position - 8) * 4))) |
					(config << ((position - 8) * 4));
			}
			
		}
	}
}




void GPIO_Write_Pin(volatile GPIO_Typedef *GPIOx, uint16_t pin, uint8_t state){
	
	if(state){
		GPIOx->BSRR.REG = pin;
	}else{
		GPIOx->BRR.REG = pin;
	}

}
uint8_t GPIO_Read_Pin(volatile GPIO_Typedef *GPIOx, uint16_t pin){
	
	return ((GPIOx->IDR.REG & pin) ? 1 : 0);

}
	
