#include<stdint.h>
#include<stm32f10x.h>

void En_clock(void);
void gpio_setup(void);
void Uart_config(void);
void delay_ms(void);
void delay(uint32_t count);
void systick_config(void);
//uint8_t debounce(uint8_t last);
void ADC_config(void);
void timer_config(void);
void TIM2_IRQHandler(void);

int main(void){
En_clock();
gpio_setup();
systick_config();
timer_config();
	
while(1){

GPIOA->ODR |= GPIO_ODR_ODR0;
delay(200);
GPIOA->ODR &= ~GPIO_ODR_ODR0;
delay(200);

}

	return 0;
}

void En_clock(void){
	RCC->APB1ENR |= 	RCC_APB1ENR_USART2EN
									|	RCC_APB1ENR_TIM2EN	;
	
	RCC->APB2ENR |= 	RCC_APB2ENR_AFIOEN 
									| RCC_APB2ENR_IOPAEN
									| RCC_APB2ENR_USART1EN
									| RCC_APB2ENR_ADC1EN;
}

void gpio_setup(void){
//PA0 signal for uart1:GPIO output pushpull 0011
GPIOA->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0 );
GPIOA->CRL |= GPIO_CRL_MODE0;
	
//PA1 response of photoreg:GPIO output pushpull 0011
GPIOA->CRL &= ~(GPIO_CRL_CNF1 | GPIO_CRL_MODE1 );
GPIOA->CRL |= GPIO_CRL_MODE1;
	
//PA2 UART2: Tx output AF push-pull 1011
GPIOA->CRL &= ~(GPIO_CRL_CNF2 | GPIO_CRL_MODE2 );
GPIOA->CRL |= GPIO_CRL_CNF2_1 | GPIO_CRL_MODE2;
	
//PA3 UART2 Rx: input Floating 0100
GPIOA->CRL &= ~(GPIO_CRL_CNF3 | GPIO_CRL_MODE3 );
GPIOA->CRL |= GPIO_CRL_CNF3_0;	

//PA4 Output for random use 0011
GPIOA->CRL &= ~(GPIO_CRL_CNF4 | GPIO_CRL_MODE4 );
GPIOA->CRL |= GPIO_CRL_MODE4;
	
//PA5 signal for UART2: GPIO output push pull 0011
GPIOA->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_MODE5 );
GPIOA->CRL |= GPIO_CRL_MODE5;

//PA6 response of computer: GPIO output pushpull 0011
GPIOA->CRL &= ~(GPIO_CRL_CNF6 | GPIO_CRL_MODE6 );
GPIOA->CRL |= GPIO_CRL_MODE6;

//PA7 Input for photo register: analog input 0000
GPIOA->CRL &= ~(GPIO_CRL_CNF7 | GPIO_CRL_MODE7 );

//PA9 UART1 Tx : Output AF push-pull 1011
GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9 );
GPIOA->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9;

//PA10 UART1 RX : input floating 0100
GPIOA->CRH &= ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10 );
GPIOA->CRH |= GPIO_CRH_CNF10_0;
}

void systick_config(void){
SysTick->LOAD = 72000-1;
SysTick->VAL = 0;
SysTick->CTRL = SysTick_CTRL_CLKSOURCE | SysTick_CTRL_ENABLE;
}

void delay_ms(void){
while(!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG)){}
}

void delay(uint32_t count){

while (count--){
delay_ms();
}

}

void timer_config(void){
TIM2->CNT = 0;
TIM2->PSC = 7200-1;
TIM2->ARR = 500;
TIM2->DIER |= TIM_DIER_UIE;
TIM2->CR1 |= TIM_CR1_DIR ;
TIM2->CR1 |=TIM_CR1_CEN;

NVIC_EnableIRQ(TIM2_IRQn);
}

void TIM2_IRQHandler(void){
		if(TIM2->SR & TIM_SR_UIF){
		TIM2->SR &= ~TIM_SR_UIF;

		GPIOA->ODR ^= GPIO_ODR_ODR4;
			
		}
	
}
void Uart_config(void);
//uint8_t debounce(uint8_t last);
void ADC_config(void);

