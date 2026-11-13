#ifndef __RCC_H
#define __RCC_H
#include <stdint.h>

//-------------------------------------STRUCT----------------------------------------
typedef struct {
    union {
        uint32_t REG;
        struct {
            volatile uint32_t HSION     : 1;
            volatile uint32_t HSIRDY    : 1;
            volatile  uint32_t Reserved1 : 1;
            volatile  uint32_t HSITRIM   : 5;
            volatile  uint32_t HSICAL    : 8;
            volatile  uint32_t HSEON     : 1;
            volatile  uint32_t HSERDY    : 1;
            volatile  uint32_t HSEBYP    : 1;
            volatile  uint32_t CSSON     : 1;
            volatile  uint32_t Reserved2 : 4;
            volatile  uint32_t PLLON     : 1;
            volatile  uint32_t PLLRDY    : 1;
						volatile  uint32_t PLL2      : 1;
            volatile  uint32_t PLL2RDY   : 1;
						volatile  uint32_t PLL3      : 1;
            volatile  uint32_t PLL3RDY   : 1;
            volatile  uint32_t Reserved3 : 2;
        } BITS;
    } CR;
    
    union {
        uint32_t REG;
        struct {
            volatile  uint32_t SW        : 2;
            volatile  uint32_t SWS       : 2;
            volatile  uint32_t HPRE      : 4;
            volatile  uint32_t PPRE1     : 3;
            volatile  uint32_t PPRE2     : 3;
            volatile  uint32_t ADCPRE    : 2;
            volatile  uint32_t PLLSRC    : 1;
            volatile  uint32_t PLLXTPRE  : 1;
            volatile  uint32_t PLLMUL    : 4;
            volatile  uint32_t USBPRE    : 1;
            volatile  uint32_t Reserved1 : 1;
            volatile  uint32_t MCO       : 4;
            volatile  uint32_t Reserved2 : 4;
        } BITS;
    } CFGR;
    
    union {
        uint32_t REG;
        struct {
            volatile uint32_t LSIRDYF   : 1;
            volatile uint32_t LSERDYF   : 1;
            volatile uint32_t HSIRDYF   : 1;
            volatile uint32_t HSERDYF   : 1;
            volatile uint32_t PLLRDYF   : 1;
						volatile uint32_t PLL2RDYF	 : 1;
						volatile uint32_t PLL3RDYF	 : 1;
            volatile uint32_t CSSF      : 1;
            volatile uint32_t LSIRDYIE  : 1;
            volatile uint32_t LSERDYIE  : 1;
            volatile uint32_t HSIRDYIE  : 1;
            volatile uint32_t HSERDYIE  : 1;
            volatile uint32_t PLLRDYIE  : 1;
						volatile uint32_t PLL2RDYIE  : 1;
						volatile uint32_t PLL3RDYIE  : 1;
            volatile uint32_t Reserved2 : 1;
            volatile uint32_t LSIRDYC   : 1;
            volatile uint32_t LSERDYC   : 1;
            volatile uint32_t HSIRDYC   : 1;
            volatile uint32_t HSERDYC   : 1;
            volatile uint32_t PLLRDYC   : 1;
						volatile uint32_t PLL2RDYC   : 1;
						volatile uint32_t PLL3RDYC   : 1;
            volatile uint32_t CSSC      : 1;
            volatile uint32_t Reserved4 : 8;
        } BITS;
    } CIR;
    
    union {
        uint32_t REG;
        struct {
            volatile uint32_t AFIORST   : 1;
            volatile uint32_t Reserved1 : 1;
            volatile uint32_t IOPARST   : 1;
            volatile uint32_t IOPBRST   : 1;
            volatile uint32_t IOPCRST   : 1;
            volatile uint32_t IOPDRST   : 1;
            volatile uint32_t IOPERST   : 1;
            volatile uint32_t Reserved2 : 2;
            volatile uint32_t ADC1RST   : 1;
            volatile uint32_t ADC2RST   : 1;
            volatile uint32_t TIM1RST   : 1;
            volatile uint32_t SPI1RST   : 1;
            volatile uint32_t Reserved3 : 1;
            volatile uint32_t USART1RST : 1;
            volatile uint32_t Reserved4 : 17;
        } BITS;
    } APB2RSTR;
		
		union {
        uint32_t REG;
        struct {
            volatile uint32_t TIM2RST  : 1;
            volatile uint32_t TIM3RST  : 1;
            volatile uint32_t TIM4RST  : 1;
            volatile uint32_t TIM5RST  : 1;
            volatile uint32_t TIM6RST  : 1;
            volatile uint32_t TIM7RST  : 1;
            volatile uint32_t Reserved1: 5;
            volatile uint32_t WWDGRST  : 1;
            volatile uint32_t Reserved2: 2;
            volatile uint32_t SPI2RST  : 1;
            volatile uint32_t SPI3RST  : 1;
            volatile uint32_t Reserved3: 1;
            volatile uint32_t USART2RST: 1;
            volatile uint32_t USART3RST: 1;
            volatile uint32_t UART4RST : 1;
            volatile uint32_t UART5RST : 1;
            volatile uint32_t I2C1RST  : 1;
            volatile uint32_t I2C2RST  : 1;
            volatile uint32_t Reserved4: 2;
            volatile uint32_t CAN1RST  : 1;
            volatile uint32_t CAN2RST  : 1;
            volatile uint32_t BKPRST   : 1;
            volatile uint32_t PWRRST   : 1;
            volatile uint32_t DACRST   : 1;
            volatile uint32_t Reserved5: 2;
        } BITS;
    } APB1RSTR;
    union {
			uint32_t REG;
			struct {
				volatile uint32_t DMA1EN   : 1;
				volatile uint32_t DMA2EN   : 1;
				volatile uint32_t SRAMEN   : 1;
				volatile uint32_t Reserved1: 1;
				volatile uint32_t FLITFEN  : 1;
				volatile uint32_t Reserved2: 1;
				volatile uint32_t CRCEN    : 1;
				volatile uint32_t Reserved3: 1;
				volatile uint32_t FSMCEN   : 1;
				volatile uint32_t Reserved4: 1;
				volatile uint32_t SDIOEN   : 1;
				volatile uint32_t Reserved5: 21;
			} BITS;
		} RCC_AHBENR;
		
	union {
        uint32_t REG;
        struct {
            volatile uint32_t AFIOEN    : 1;
            volatile uint32_t Reserved1 : 1;
            volatile uint32_t IOPAEN    : 1;
            volatile uint32_t IOPBEN    : 1;
            volatile uint32_t IOPCEN    : 1;
            volatile uint32_t IOPDEN    : 1;
            volatile uint32_t IOPEEN    : 1;
						volatile uint32_t IOPFEN    : 1;
            volatile uint32_t IOPGEN    : 1;
            volatile uint32_t ADC1EN    : 1;
            volatile uint32_t ADC2EN    : 1;
            volatile uint32_t TIM1EN    : 1;
            volatile uint32_t SPI1EN    : 1;
            volatile uint32_t TIM8EN    : 1;
            volatile uint32_t USART1EN  : 1;
            volatile uint32_t ADC3EN    : 1;
            volatile uint32_t Reserved3 : 3;
						volatile uint32_t TIM9EN    : 1;
						volatile uint32_t TIM10EN   : 1;
						volatile uint32_t TIM11EN   : 1;
						volatile uint32_t Reserved4 : 10;
        } BITS;
    } APB2ENR;
		
    union {
        uint32_t REG;
        struct {
            volatile uint32_t TIM2EN    : 1;
            volatile uint32_t TIM3EN    : 1;
            volatile uint32_t TIM4EN    : 1;
            volatile uint32_t TIM5EN    : 1;
            volatile uint32_t TIM6EN    : 1;
            volatile uint32_t TIM7EN    : 1;
						volatile uint32_t TIM12EN    : 1;
						volatile uint32_t TIM13EN    : 1;
						volatile uint32_t TIM14EN    : 1;
            volatile uint32_t Reserved1 : 2;
            volatile uint32_t WWDGEN    : 1;
            volatile uint32_t Reserved2 : 2;
            volatile uint32_t SPI2EN    : 1;
            volatile uint32_t SPI3EN    : 1;
            volatile uint32_t Reserved3 : 1;
            volatile uint32_t USART2EN  : 1;
            volatile uint32_t USART3EN  : 1;
            volatile uint32_t UART4EN   : 1;
            volatile uint32_t UART5EN   : 1;
            volatile uint32_t I2C1EN    : 1;
            volatile uint32_t I2C2EN    : 1;
            volatile uint32_t USBEN     : 1;
            volatile uint32_t Reserved4 : 1;
						volatile uint32_t CANEN : 1;
						volatile uint32_t Reserved5 : 1;
						volatile uint32_t BKPEN : 1;
						volatile uint32_t PWREN : 1;
						volatile uint32_t DACEN : 1;
						volatile uint32_t Reserved6 : 2;
        } BITS;
    } APB1ENR;
		
    union {
    uint32_t REG;
    struct {
    		volatile uint32_t LSEON     : 1;
    		volatile uint32_t LSERDY    : 1;
    		volatile uint32_t LSEBYP    : 1;
    		volatile uint32_t Reserved1 : 5;
    		volatile uint32_t RTCSEL    : 2;
    		volatile uint32_t Reserved2 : 5;
    		volatile uint32_t RTCEN     : 1;
    		volatile uint32_t BDRST     : 1;
    		volatile uint32_t Reserved3 : 15;
    } BITS;
    } RCC_BDCR;
    
		union {
			uint32_t REG;
			struct {
				volatile uint32_t LSION     : 1;
				volatile uint32_t LSIRDY    : 1;
				volatile uint32_t Reserved1 : 22;
				volatile uint32_t RMVF      : 1;
				volatile uint32_t Reserved2 : 1;
				volatile uint32_t PINRSTF   : 1;
				volatile uint32_t PORRSTF   : 1;
				volatile uint32_t SFTRSTF   : 1;
				volatile uint32_t IWDGRSTF  : 1;
				volatile uint32_t WWDGRSTF  : 1;
				volatile uint32_t LPWRRSTF  : 1;
			} BITS;
		} RCC_CSR;
    
} RCC_TypeDef;

typedef union{
	 uint32_t REG;
	struct{
		volatile uint32_t LATENCY:3;
		volatile uint32_t HLFCYA :1;
		volatile uint32_t PRFTBE :1;
		volatile uint32_t PRFTBS :1;
		volatile uint32_t Res    :26;
	}BITS;
}FLASH_ACR;


#define ADDRESS_RCC         0x40021000UL
#define ADDRESS_FLASH_ACR   0x40022000UL

#define RCC 					((volatile RCC_TypeDef*)ADDRESS_RCC)
#define ACR 					((volatile FLASH_ACR*)ADDRESS_FLASH_ACR)

void RCC_Config_72Mhz(void);

void RCC_Enable_PortA(void);
void RCC_Enable_PortB(void);
void RCC_Enable_PortC(void);
void RCC_Enable_TIM2(void);
void RCC_Enable_AFIO(void);
void RCC_Enable_ADC01(void);
void RCC_Enable_UART1(void);
void RCC_Enable_SPI1(void);
void RCC_Enable_I2C1(void);
void RCC_Enable_USB(void);
#endif