#include "cmsis/stm32f1xx.h"

void _close(void){}
void _lseek(void){}
void _read(void){}
void _write(void){}

#define BD 115200

void dma_init(DMA_Channel_TypeDef* dma, uint32_t peri_addr, uint32_t mem_addr, uint32_t data_size) {
	dma->CPAR = peri_addr;
	dma->CMAR = mem_addr;
	dma->CNDTR = data_size;
	dma->CCR = (
		0U << DMA_CCR_MEM2MEM_Pos |
		DMA_CCR_PL 				  |
		0U << DMA_CCR_MSIZE_Pos	  |
		0U << DMA_CCR_PSIZE_Pos   |
		DMA_CCR_MINC 			  |
		0U << DMA_CCR_PINC_Pos	  |
		DMA_CCR_CIRC			  |
		0U << DMA_CCR_DIR_Pos
	);
	dma->CCR |= DMA_CCR_EN;
}

void uart_init(USART_TypeDef* usart, uint8_t rx, uint8_t tx, uint8_t dmar, uint8_t rx_int_en) {
	usart->CR1 &= ~USART_CR1_M;
	usart->CR2 &= ~(USART_CR2_STOP_0|USART_CR2_STOP_1);
	usart->BRR = ((((SystemCoreClock / BD) / 16) << USART_BRR_DIV_Mantissa_Pos) | ((SystemCoreClock / BD) % 16));
	usart->CR1 |= (tx * USART_CR1_TE) | (rx * USART_CR1_RE);
	usart->CR1 |= (rx_int_en * USART_CR1_RXNEIE);
	usart->CR3 |= dmar * USART_CR3_DMAR;
	usart->CR1 |= USART_CR1_UE;
}

void uart_send(USART_TypeDef* usart, const char* data) {
	uint32_t i = 0;
	for(;*data; ++data) {
		while(!(usart->SR & USART_SR_TXE));
		usart->DR = *data;
		++i;
	}
	if (i > 0 && data[i-1] == '\n') {
		while(!(usart->SR & USART_SR_TC));
		for(;i != 0; --i) {
			while(!(usart->SR & USART_SR_TXE));
			usart->DR = '\b';
		}
	}
	while(!(usart->SR & USART_SR_TC));
}

void uart_echo(USART_TypeDef* usart) {
	while(!(usart->SR & USART_SR_RXNE));
	const char tmp = usart->DR;
	while(!(usart->SR & USART_SR_TXE));
	usart->DR = tmp;	
}

char uart_receive(USART_TypeDef* usart) {
	while(!(usart->SR & USART_SR_RXNE));
	return usart->DR;
}

void uart_transmit(USART_TypeDef* usart, char c) {
	while(!(usart->SR & USART_SR_TXE));
	usart->DR = c;
}

void USART2_IRQHandler(void) {
	const char c = uart_receive(USART2);
	uart_transmit(USART3, c);
}

char ring_buffer[4096];
volatile uint32_t reader_cursor = 0;
volatile uint32_t* writer_cursor = &DMA1_Channel3->CNDTR;

int main(void) {
	RCC->CR |= RCC_CR_HSION;

    while (!(RCC->CR & RCC_CR_HSIRDY)) {}

    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_HSI;

    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) {}

	SystemCoreClockUpdate();

  	///ENABLING CLOCK
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN;
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN | RCC_APB1ENR_USART3EN;
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	// config uart2 pins (PA)
	GPIOA->CRH &= ~(GPIO_CRH_MODE9|GPIO_CRH_CNF9|GPIO_CRH_MODE10|GPIO_CRH_CNF10);
    GPIOA->CRH |= ( 
		(0x1 << GPIO_CRH_MODE9_Pos) |
        (0x2 << GPIO_CRH_CNF9_Pos)  |
        (0x0 << GPIO_CRH_MODE10_Pos) |
        (0x1 << GPIO_CRH_CNF10_Pos) 
	);
	// config uart3 pin (PC)
	AFIO->MAPR &= ~AFIO_MAPR_USART3_REMAP_1;
	AFIO->MAPR |= AFIO_MAPR_USART3_REMAP_0;
	GPIOC->CRH &= ~(GPIO_CRH_MODE10|GPIO_CRH_CNF10|GPIO_CRH_MODE11|GPIO_CRH_CNF11);
    GPIOC->CRH |= ( 
		(0x1 << GPIO_CRH_MODE10_Pos) |
        (0x2 << GPIO_CRH_CNF10_Pos)  |
        (0x0 << GPIO_CRH_MODE11_Pos) |
        (0x1 << GPIO_CRH_CNF11_Pos) 
	);

	uart_init(USART2, 1, 1, 0, 1);
	uart_init(USART3, 1, 1, 1, 0);
	dma_init(DMA1_Channel3, (uint32_t)&USART3->DR, (uint32_t)ring_buffer, sizeof(ring_buffer));

	__NVIC_SetPriority(USART2_IRQn, 0);
	__NVIC_EnableIRQ(USART2_IRQn);

  	while(1) {
		if ((sizeof(ring_buffer) - *writer_cursor) != reader_cursor) {
			uart_transmit(USART2, ring_buffer[reader_cursor]);
			reader_cursor = ((reader_cursor + 1) % sizeof(ring_buffer));
		}
  	}

  	return 0;
}