#ifndef __UART_H
#define __UART_H
#include <stdint.h>
//--------------------ADDRESS BASE UART--------------------
#define UART_ADD_BASE		0x40013800

//-----------------------ADDRESS REGISTER------------------
#define USART_SR_OFFSET        0x00
#define USART_DR_OFFSET        0x04
#define USART_BRR_OFFSET       0x08
#define USART_CR1_OFFSET       0x0C
#define USART_CR2_OFFSET       0x10
#define USART_CR3_OFFSET       0x14
#define USART_GTPR_OFFSET      0x18


//----------------------UART01------------------------------
#define USART1_SR   *((volatile uint32_t *)(UART_ADD_BASE + USART_SR_OFFSET ))
#define USART1_DR   *((volatile uint32_t *)(UART_ADD_BASE + USART_DR_OFFSET))
#define USART1_BRR  *((volatile uint32_t *)(UART_ADD_BASE + USART_BRR_OFFSET))
#define USART1_CR1  *((volatile uint32_t *)(UART_ADD_BASE + USART_CR1_OFFSET ))
#define USART1_CR2  *((volatile uint32_t *)(UART_ADD_BASE + USART_CR2_OFFSET ))
#define USART1_CR3  *((volatile uint32_t *)(UART_ADD_BASE + USART_CR3_OFFSET ))
#define USART1_GTPR *((volatile uint32_t *)(UART_ADD_BASE + USART_GTPR_OFFSET ))

//-----------------------API-------------------------------------
void UART1_Init(void);
void UART1_SendChar(char c);
void UART1_SendString(const char *str);
void USART1_IRQHandler(void);

#endif