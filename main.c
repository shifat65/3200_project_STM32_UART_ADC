#include <stdint.h>
#include <stm32f10x.h>

void En_clock(void);
void gpio_setup(void);
void Uart1_config(void);
void Uart2_config(void);
void delay_ms(void);
void delay(uint32_t count);
void systick_config(void);
void ADC_config(void);
void timer_config(void);
void TIM2_IRQHandler(void);
void USART1_IRQHandler(void);
void ADC1_2_IRQHandler(void);
void Exi_config(void);

uint32_t Rdata = 0;
uint32_t Sdata = 3;
uint8_t ind = 1;
uint32_t Rdata2 = 0;
uint32_t Sdata2 = 0;
uint16_t Adc_data = 0;
uint8_t present = 1;
uint8_t past = 0;

int main(void)
{
	En_clock();
	gpio_setup();
	systick_config();
	timer_config();
	Uart1_config();
	Exi_config();
	ADC_config();

	while (1)
	{
	}

	return 0;
}

void En_clock(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN | RCC_APB1ENR_TIM2EN;

	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC1EN;
}

void gpio_setup(void)
{
	// PA0 Tx signal for uart1:GPIO output pushpull 0011
	GPIOA->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0);
	GPIOA->CRL |= GPIO_CRL_MODE0;

	// PA1 Rx signal for uart1:GPIO output pushpull 0011
	GPIOA->CRL &= ~(GPIO_CRL_CNF1 | GPIO_CRL_MODE1);
	GPIOA->CRL |= GPIO_CRL_MODE1;

	// PA2 user input 1
	GPIOA->CRL &= ~(GPIO_CRL_CNF2 | GPIO_CRL_MODE2);
	GPIOA->CRL |= GPIO_CRL_CNF2_0;

	// PA3 user input 0;
	GPIOA->CRL &= ~(GPIO_CRL_CNF3 | GPIO_CRL_MODE3);
	GPIOA->CRL |= GPIO_CRL_CNF3_0;

	// PA4 Output for USART1 receive use 0011
	GPIOA->CRL &= ~(GPIO_CRL_CNF4 | GPIO_CRL_MODE4);
	GPIOA->CRL |= GPIO_CRL_MODE4;

	// PA5 ADC response (other stm) push pull 0011
	GPIOA->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_MODE5);
	GPIOA->CRL |= GPIO_CRL_MODE5;

	// PA6 EXT interrupt response GPIO output push pull 0011
	GPIOA->CRL &= ~(GPIO_CRL_CNF6 | GPIO_CRL_MODE6);
	GPIOA->CRL |= GPIO_CRL_MODE6;

	// PA7 Output for USART1 receive use 0011
	GPIOA->CRL &= ~(GPIO_CRL_CNF7 | GPIO_CRL_MODE7);
	GPIOA->CRL |= GPIO_CRL_MODE7;

	// PA9 UART1 Tx : Output AF push-pull 1011
	GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9);
	GPIOA->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9;

	// PA10 UART1 RX : input floating 0100
	GPIOA->CRH &= ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10);
	GPIOA->CRH |= GPIO_CRH_CNF10_0;

	// PB0 analog input for ADC 0000
	GPIOB->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0);

	// PB1 Response on ADC watchdog 0011 (light is very high)
	// GPIOB->CRL &= ~(GPIO_CRL_CNF1 | GPIO_CRL_MODE1);
	// GPIOB->CRL |= GPIO_CRL_MODE1;

	// PB5 Response on ADC watchdog on arrived from other stm  (2) 0011
	// GPIOB->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_MODE5);
	// GPIOB->CRL |= GPIO_CRL_MODE5;
	//
	// PB10 ADC  response internal
	// GPIOB->CRH &= ~(GPIO_CRH_CNF10| GPIO_CRH_MODE10);
	// GPIOB->CRH |= GPIO_CRH_MODE10;
}

void systick_config(void)
{
	SysTick->LOAD = 72000 - 1;
	SysTick->VAL = 0;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE | SysTick_CTRL_ENABLE;
}

void delay_ms(void)
{
	while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG))
	{
	}
}

void delay(uint32_t count)
{

	while (count--)
	{
		delay_ms();
	}
}

void timer_config(void)
{
	TIM2->CNT = 0;
	TIM2->PSC = 7200 - 1;
	TIM2->ARR = 50000;
	TIM2->DIER |= TIM_DIER_UIE;
	TIM2->CR1 |= TIM_CR1_DIR;
	TIM2->CR1 |= TIM_CR1_CEN;

	NVIC_EnableIRQ(TIM2_IRQn);
}

void TIM2_IRQHandler(void)
{
	if (TIM2->SR & TIM_SR_UIF)
	{
		TIM2->SR &= ~TIM_SR_UIF;

		GPIOA->ODR |= GPIO_ODR_ODR0;

		USART1->DR = Sdata;

		delay(25);
		GPIOA->ODR &= ~GPIO_ODR_ODR0;

		Sdata++;
		if (Sdata == 4)
		{
			Sdata = 0;
		}
	}
}
void Uart1_config(void)
{
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;

	// Baud rate is 9600;
	USART1->BRR = 0x1DCC;

	NVIC_EnableIRQ(USART1_IRQn);
}

void USART1_IRQHandler(void)
{

	if (USART1->SR & USART_SR_RXNE)
	{
		Rdata = USART1->DR;
		USART1->SR &= ~USART_SR_RXNE;

		// Receving data from USART1 rx
		GPIOA->ODR |= GPIO_ODR_ODR1;

		if (Rdata == 0)
		{
			GPIOA->ODR &= ~GPIO_ODR_ODR4;
			GPIOA->ODR &= ~GPIO_ODR_ODR7;
		}
		else if (Rdata == 1)
		{
			GPIOA->ODR |= GPIO_ODR_ODR4;
			GPIOA->ODR &= ~GPIO_ODR_ODR7;
		}
		else if (Rdata == 2)
		{
			GPIOA->ODR &= ~GPIO_ODR_ODR4;
			GPIOA->ODR |= GPIO_ODR_ODR7;
		}
		else if (Rdata == 3)
		{
			GPIOA->ODR |= GPIO_ODR_ODR4;
			GPIOA->ODR |= GPIO_ODR_ODR7;
		}

		if (Rdata == 'A')
		{
			GPIOA->ODR ^= GPIO_ODR_ODR6;
			// GPIOA->ODR ^= GPIO_ODR_ODR7;
		}

		if (Rdata == 'B')
		{
			GPIOA->ODR |= GPIO_ODR_ODR5;
		}
		else if (Rdata == 'C')
		{
			GPIOA->ODR ^= GPIO_ODR_ODR5;
		}

		delay(25);
		GPIOA->ODR &= ~GPIO_ODR_ODR1;
	}
}

void Exi_config(void)
{
	AFIO->EXTICR[1] = 0;

	EXTI->IMR |= EXTI_IMR_MR2 | EXTI_IMR_MR3;
	EXTI->RTSR |= EXTI_RTSR_TR2 | EXTI_RTSR_TR3;

	NVIC_EnableIRQ(EXTI2_IRQn);
	// NVIC_EnableIRQ(EXTI3_IRQn);
}

void EXTI2_IRQHandler(void)
{
	if (EXTI->PR & EXTI_PR_PR2)
	{
		EXTI->PR |= EXTI_PR_PR2;
	}
	GPIOA->ODR |= GPIO_ODR_ODR0;
	USART1->DR = 'A';
	delay(25);
	GPIOA->ODR &= ~GPIO_ODR_ODR0;
}

void ADC_config(void)
{
	ADC1->CR1 |= ADC_CR1_AWDEN | ADC_CR1_AWDIE | ADC_CR1_AWDCH_3;

	// B0 analog input pin
	ADC1->SQR3 = ADC_SQR3_SQ1_3;

	ADC1->CR2 |= ADC_CR2_CONT;

	ADC1->CR2 |= ADC_CR2_ADON;

	ADC1->HTR = 0xA00;
	ADC1->LTR = 0x600;

	delay(500);
	ADC1->CR2 |= ADC_CR2_ADON;

	NVIC_EnableIRQ(ADC1_2_IRQn);
}

void ADC1_2_IRQHandler(void)
{
	Adc_data = ADC1->DR;

	if (ADC1->SR & ADC_SR_AWD)
	{
		if (Adc_data > ADC1->HTR)
		{
			present = 1;
		}
		else if (Adc_data < ADC1->LTR)
		{
			present = 0;
		}

		if ((present == 1) && (present != past))
		{
			GPIOA->ODR |= GPIO_ODR_ODR0;
			USART1->DR = 'B';
			past = present;
			delay(25);
			GPIOA->ODR &= ~GPIO_ODR_ODR0;
		}
		else if ((present == 0) && (present != past))
		{
			GPIOA->ODR |= GPIO_ODR_ODR0;
			USART1->DR = 'C';
			past = present;
			delay(25);
			GPIOA->ODR &= ~GPIO_ODR_ODR0;
		}

		ADC1->SR &= ~ADC_SR_AWD;
	}
}
