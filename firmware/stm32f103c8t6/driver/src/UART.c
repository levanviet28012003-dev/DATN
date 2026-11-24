#include "UART.h"
#include "GPIO.h"
#include "RCC.h"

// NVIC_ISER1 bit 5 bật ngắt USART1 (IRQ 37).
#define NVIC_ISER1          (*(volatile uint32_t *)0xE000E104UL)
#define USART1_IRQ_BIT      (1UL << (37U - 32U))

_Static_assert(UART1_BRR_VALUE == 0x0271UL, "BRR cho 115200 baud tai 72 MHz phai la 0x0271");
_Static_assert((UART1_RX_BUFFER_SIZE & (UART1_RX_BUFFER_SIZE - 1U)) == 0U,
               "Kich thuoc bo dem RX phai la luy thua cua 2");
_Static_assert((UART1_TX_BUFFER_SIZE & (UART1_TX_BUFFER_SIZE - 1U)) == 0U,
               "Kich thuoc bo dem TX phai la luy thua cua 2");
_Static_assert(UART1_TX_BUFFER_SIZE <= 0x8000U, "Chi so TX dung uint16_t");

// DMA ghi rx_buffer; vị trí ghi suy ra từ CNDTR, chỉ vòng lặp chính đọc rx_tail.
static volatile uint8_t rx_buffer[UART1_RX_BUFFER_SIZE];
static uint16_t rx_tail;

// tx_head do vòng lặp chính ghi, tx_tail do ngắt ghi; mỗi biến chỉ có một nơi ghi.
static volatile uint8_t tx_buffer[UART1_TX_BUFFER_SIZE];
static volatile uint16_t tx_head;
static volatile uint16_t tx_tail;

void USART1_IRQHandler(void){
	uint32_t sr = USART1_SR;

	if((USART1_CR1 & USART_CR1_TXEIE) && (sr & USART_SR_TXE)){
		uint16_t tail = tx_tail;
		if(tail != tx_head){
			USART1_DR = tx_buffer[tail];
			tx_tail = (uint16_t)((tail + 1U) & (UART1_TX_BUFFER_SIZE - 1U));
		}else{
			// Hết dữ liệu: tắt ngắt TXE, UART1_Write sẽ bật lại.
			USART1_CR1 &= ~USART_CR1_TXEIE;
		}
	}
}

void UART1_Init(void){
	RCC_Enable_PortA();
	RCC_Enable_UART1();
	RCC_Enable_DMA1();

	USART1_CR1 = 0U;
	DMA1_CCR5 = 0U;
	rx_tail = 0U;
	tx_head = 0U;
	tx_tail = 0U;

	GPIO_Config(GPIOA,GPIO_PIN_9,GPIO_MODE_AF_PP);
	// Kéo lên để đường RX ở mức nghỉ khi chưa nối Pi, tránh nhận nhiễu.
	GPIO_Config(GPIOA,GPIO_PIN_10,GPIO_MODE_INPUT_PU);

	// DMA: USART1_DR -> rx_buffer, 8 bit, tăng địa chỉ bộ nhớ, vòng tròn.
	DMA1_CPAR5 = (uint32_t)&USART1_DR;
	DMA1_CMAR5 = (uint32_t)rx_buffer;
	DMA1_CNDTR5 = UART1_RX_BUFFER_SIZE;
	DMA1_CCR5 = DMA_CCR_MINC | DMA_CCR_CIRC;
	DMA1_CCR5 |= DMA_CCR_EN;

	USART1_BRR = UART1_BRR_VALUE;
	USART1_CR2 = 0U;                 // 1 stop bit
	USART1_CR3 = USART_CR3_DMAR;     // yêu cầu DMA khi RXNE
	USART1_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;   // 8 bit, không parity

	NVIC_ISER1 = USART1_IRQ_BIT;     // ISER chỉ có tác dụng khi ghi 1, không cần đọc-sửa-ghi
}

uint8_t UART1_ReadByte(uint8_t *byte){
	// CNDTR đếm ngược từ kích thước bộ đệm; giá trị 0 không xuất hiện ở chế độ vòng tròn.
	uint16_t head = (uint16_t)((UART1_RX_BUFFER_SIZE - (DMA1_CNDTR5 & 0xFFFFU)) &
		(UART1_RX_BUFFER_SIZE - 1U));
	if(head == rx_tail){
		return 0U;
	}
	*byte = rx_buffer[rx_tail];
	rx_tail = (uint16_t)((rx_tail + 1U) & (UART1_RX_BUFFER_SIZE - 1U));
	return 1U;
}

uint16_t UART1_TxFree(void){
	uint16_t used = (uint16_t)((tx_head - tx_tail) & (UART1_TX_BUFFER_SIZE - 1U));
	return (uint16_t)(UART1_TX_BUFFER_SIZE - 1U - used);
}

uint8_t UART1_Write(const uint8_t *data, uint16_t length){
	uint16_t head = tx_head;
	if(length == 0U){
		return 1U;
	}
	if(length > UART1_TxFree()){
		return 0U;
	}
	for(uint16_t i = 0U; i < length; i++){
		tx_buffer[head] = data[i];
		head = (uint16_t)((head + 1U) & (UART1_TX_BUFFER_SIZE - 1U));
	}
	tx_head = head;
	// Ngắt chỉ xóa TXEIE còn ở đây chỉ bật, nên đọc-sửa-ghi không làm mất trạng thái.
	USART1_CR1 |= USART_CR1_TXEIE;
	return 1U;
}
