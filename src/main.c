#include "cmsis/stm32f1xx.h"

void delay(int32_t ticks) {
  for(;ticks > 0; --ticks) {}
}

int main(void) {
  ///ENABLING CLOCK ON PORTA
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	
	// TURNING UP DIODE 50MHz PUSH PULL
	GPIOA->CRL &= ~GPIO_CRL_CNF5; // set as push-pull output
  GPIOA->CRL &= ~GPIO_CRL_MODE5;
	GPIOA->CRL |= GPIO_CRL_MODE5_0; // max 10MHz

  const int32_t delay_time = SystemCoreClock/32;
  while(1) {
    delay(delay_time);
    GPIOA->BSRR = GPIO_BSRR_BS5;
    delay(delay_time);
    GPIOA->BSRR = GPIO_BSRR_BR5;
  }

  return 0;
}