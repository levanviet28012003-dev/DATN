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

//----------------------BIT THANH GHI USART------------------
#define USART_SR_PE            (1UL << 0)
#define USART_SR_FE            (1UL << 1)
#define USART_SR_NE            (1UL << 2)
#define USART_SR_ORE           (1UL << 3)
#define USART_SR_RXNE          (1UL << 5)
#define USART_SR_TC            (1UL << 6)
#define USART_SR_TXE           (1UL << 7)

#define USART_CR1_RE           (1UL << 2)
#define USART_CR1_TE           (1UL << 3)
#define USART_CR1_RXNEIE       (1UL << 5)
#define USART_CR1_TXEIE        (1UL << 7)
#define USART_CR1_UE           (1UL << 13)

#define USART_CR3_DMAR         (1UL << 6)

//----------------------DMA1 KENH 5 = USART1_RX (RM0008 bang 78)------
#define DMA1_ADD_BASE          0x40020000UL
#define DMA1_CCR5   *((volatile uint32_t *)(DMA1_ADD_BASE + 0x58UL))
#define DMA1_CNDTR5 *((volatile uint32_t *)(DMA1_ADD_BASE + 0x5CUL))
#define DMA1_CPAR5  *((volatile uint32_t *)(DMA1_ADD_BASE + 0x60UL))
#define DMA1_CMAR5  *((volatile uint32_t *)(DMA1_ADD_BASE + 0x64UL))

#define DMA_CCR_EN             (1UL << 0)
#define DMA_CCR_CIRC           (1UL << 5)
#define DMA_CCR_MINC           (1UL << 7)

//----------------------CAU HINH---------------------------------
// USART1 nằm trên APB2; RCC_Config_72Mhz() đặt PCLK2 = 72 MHz.
#define UART1_PCLK_HZ          72000000UL
#define UART1_BAUDRATE         115200UL
// BRR = PCLK / baud, làm tròn: 72 MHz / 115200 = 625 = 0x0271 (mantissa 39, fraction 1).
#define UART1_BRR_VALUE        ((UART1_PCLK_HZ + UART1_BAUDRATE / 2UL) / UART1_BAUDRATE)

// Kích thước phải là lũy thừa của 2. RX 512 byte giữ được khoảng 44 ms dữ liệu ở 115200 baud.
#define UART1_RX_BUFFER_SIZE   512U
#define UART1_TX_BUFFER_SIZE   256U

//-----------------------API-------------------------------------
// PA9 = TX (AF push-pull), PA10 = RX (input pull-up), 8N1.
// RX dùng DMA1 kênh 5 vòng tròn nên không mất byte khi I2C tạm tắt ngắt;
// TX dùng ngắt TXE và hàng đợi vòng, không chặn vòng lặp chính.
void UART1_Init(void);

// Trả về 1 và ghi vào *byte nếu có dữ liệu chưa đọc.
uint8_t UART1_ReadByte(uint8_t *byte);

// Đưa toàn bộ length byte vào hàng đợi phát; trả về 0 (không ghi gì) nếu không đủ chỗ.
uint8_t UART1_Write(const uint8_t *data, uint16_t length);

uint16_t UART1_TxFree(void);

void USART1_IRQHandler(void);

#endif
