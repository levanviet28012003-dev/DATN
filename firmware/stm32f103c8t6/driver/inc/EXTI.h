#ifndef __EXTI_H
#define __EXTI_H
#include "TYPE.h"
#include "AFIO.h"
#include "GPIO.h"

#define EXTI_RISING_MODE 0x00
#define EXTI_FALLING_MODE 0x01
#define EXTI_BOTH_MODE 0x02

#define NVIC_ISER0 *((uint32_t *)(0xE000E100))
#define NVIC_ISER1 *((uint32_t *)(0xE000E104))

typedef struct
{
    __32BIT IMR;
    __32BIT EMR;
    __32BIT RTSR;
    __32BIT FTSR;
    __32BIT SWIER;
    __32BIT PR;
} EXTI_Typedef;

#define EXTI ((volatile EXTI_Typedef *)0x40010400UL)

void EXTI0_IRQHandler(void); // 0%
uint8_t get_Pin_Number(uint16_t gpio_pin);
void EXTI_Init(uint16_t gpio_pin, volatile GPIO_Typedef *Port, uint8_t type);

void NVIC_UART_En(void);
#endif