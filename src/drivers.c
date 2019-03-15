/*
 * drivers.c
 *
 *  Created on: 30 nov. 2018
 *      Author: Myren
 */

#include "drivers.h"



static void ADC_config(ADC_TypeDef* hadc, uint8_t channel);
static void DMA_config(DMA_Stream_TypeDef* hdma, DMA_InitVals idma);

void initADC_DAC(uint16_t *bufferIn1, uint16_t *bufferIn2, uint16_t *bufferIn3,
					uint16_t *bufferOut1, uint16_t *bufferOut2, uint16_t bufferSize)
{
	DMA_InitVals DMA_Init;

	/****** GPIO config. ******/
	// GPIOC and GPIOA clock enable
	RCC->AHB1ENR |= (1 << 2) | (1 << 0);
	// Configure GPIO’s for analog mode
	// Outputs for DAC's, PA4, PA5
	GPIOA->MODER |= (1 << 11) | (1 << 10) | (1 << 9) | (1 << 8);
	// Inputs for ADC's, PC4, PC2, PC0
	GPIOC->MODER |= (1 << 9) | (1 << 8) | (1 << 5) | (1 << 4) | (1 << 1) | (1 << 0);
	/******/

	/****** ADC’s config. *****/
	// ADC1, ADC2 and ADC3 clock enable
	RCC->APB2ENR |= (1 << 10) | (1 << 9) | (1 << 8);

	ADC_config(ADC1, 14);
	ADC_config(ADC2, 12);
	ADC_config(ADC3, 10);
	/******/

	/****** DMA2 config. (periph. -> memory)******/
	// DMA1 and DMA2 clock enable
	RCC->AHB1ENR |= (1 << 22) | (1 << 21);

	DMA_Init.channel = 0;
	DMA_Init.direction = 0;
	DMA_Init.bufferSize = bufferSize;
	DMA_Init.memAddr = (uint32_t)bufferIn1;
	DMA_Init.periphAddr = (uint32_t)&ADC1->DR;
	DMA_Init.memDataSize = SIZE_HALF_WORD;
	DMA_Init.periphDataSize = SIZE_HALF_WORD;
	DMA_Init.mode = 1;
	DMA_Init.compIntEnable = 1;
	DMA_Init.halfIntEnable = 1;
	DMA_config(DMA2_Stream0, DMA_Init);

	DMA_Init.channel = 1;
	DMA_Init.memAddr = (uint32_t)bufferIn2;
	DMA_Init.periphAddr = (uint32_t)&ADC2->DR;
	DMA_Init.compIntEnable = 0;
	DMA_Init.halfIntEnable = 0;
	DMA_config(DMA2_Stream3, DMA_Init);

	DMA_Init.channel = 2;
	DMA_Init.memAddr = (uint32_t)bufferIn3;
	DMA_Init.periphAddr = (uint32_t)&ADC3->DR;
	DMA_config(DMA2_Stream1, DMA_Init);

	// Enable global interrupt for DMA2 stream 0
	NVIC_SetPriorityGrouping(2);
	NVIC_SetPriority (DMA2_Stream0_IRQn, 6);
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	/*
	// Enable global interrupt for DMA2 stream 3
	NVIC_SetPriorityGrouping(2);
	NVIC_SetPriority (DMA2_Stream3_IRQn, 6);
	NVIC_EnableIRQ(DMA2_Stream3_IRQn);
	// Enable global interrupt for DMA2 stream 1
	NVIC_SetPriorityGrouping(2);
	NVIC_SetPriority (DMA2_Stream1_IRQn, 6);
	NVIC_EnableIRQ(DMA2_Stream1_IRQn);
	*/
	/******/

	/****** DAC config. ******/
	// DAC clock enable
	RCC->APB1ENR |= (1 << 29);
	// TIM8 TRGO as trigger source, trigger enable, enable channels,
	DAC->CR |= (1 << 19) | (1 << 3) | (1 << 18) | (1 << 2) | (1 << 16) | (1 << 0);
	// DMA enable
	DAC->CR |= (1 << 28) | (1 << 12);
	/******/

	/****** DMA1 config. (memory -> periph.) ******/
	DMA_Init.channel = 7;
	DMA_Init.direction = 1;
	DMA_Init.bufferSize = bufferSize;
	DMA_Init.memAddr = (uint32_t)bufferOut1;
	DMA_Init.periphAddr = (uint32_t)&DAC->DHR12R1;
	DMA_Init.memDataSize = SIZE_HALF_WORD;
	DMA_Init.periphDataSize = SIZE_HALF_WORD;
	DMA_Init.mode = 1;
	DMA_Init.compIntEnable = 0;
	DMA_Init.halfIntEnable = 0;
	DMA_config(DMA1_Stream5, DMA_Init);

	DMA_Init.memAddr = (uint32_t)bufferOut2;
	DMA_Init.periphAddr = (uint32_t)&DAC->DHR12R2;
	DMA_config(DMA1_Stream6, DMA_Init);
	/******/

	/****** Timer 8 config. ******/
	// TIM8 clock enable
	RCC->APB2ENR |= (1 << 1);
	// Update event is selected as trigger output (TRGO) (Overflow in this case)
	TIM8->CR2 |= (1 << 5);
	// Prescaler
	TIM8->PSC = 180-1; // Internal clock freq. is 180MHz so this gives a tim freq. of 1MHz
	// Auto reload
	TIM8->ARR = 20-1;//1000000/SAMPLING_FREQUENCY;//20; // This gives a sampling freq. of 50kHz
	// Enable counter
	TIM8->CR1 |= (1 << 0);
	/******/

}



void UART_Init(USART_TypeDef* huart, uint32_t baud, uint8_t* pDst, uint8_t size)
{
	// DMA1 and DMA2 clock enable
	RCC->AHB1ENR |= (1 << 22) | (1 << 21);
	DMA_InitVals DMA_Init;
	uint32_t pclk1;
	uint32_t div;
	uint8_t carry;
	// DMA1 clock enable
	//RCC->AHB1ENR |= (1 << 21);
	// UART4 clock enable
	RCC->APB1ENR |= (1 << 19);
	// GPIOC clock enable
	RCC->AHB1ENR |= (1 << 2);
	// PC10 -> TX, PC11 -> RX
	GPIOC->MODER |= (1 << 23) | (1 << 21);
	GPIOC->OSPEEDR |= (1 << 23) | (1 << 22) | (1 << 21) | (1 << 20);
	GPIOC->AFR[1] |= (8 << 12) | (8 << 8);
	// Calculate division factor
	pclk1 = HAL_RCC_GetPCLK1Freq();
	div = pclk1 << 1;
	div = div/baud;
	carry = (uint8_t)(div & 0x01);
	div >>= 1;
	div += carry;
	huart->BRR |= (0x0000ffff & div);

	// These two also work
	//huart->BRR |= (45000000L)/9600L;//baud;
	//huart->BRR |= (pclk1)/baud;

	// Idle line interrupt enable
	huart->CR1 |= (1 << 4);

	// UART enable, transmitter & receiver enable
	huart->CR1 |= (1 << 13) | (1 << 3) | (1 << 2);

	// UART4 RX DMA is on channel 4, stream 2 on DMA1
	// UART DMA receiver
	huart->CR3 |= (1 << 6);

	DMA_Init.channel = 4;
	DMA_Init.direction = 0; // Peripheral to memory
	DMA_Init.bufferSize = size;
	DMA_Init.memAddr = (uint32_t)pDst;
	DMA_Init.periphAddr = (uint32_t)(&huart->DR);
	DMA_Init.memDataSize = SIZE_BYTE;
	DMA_Init.periphDataSize = SIZE_BYTE;
	DMA_Init.mode = 1;	// Circular mode
	DMA_Init.compIntEnable = 1;
	DMA_Init.halfIntEnable = 0;

	DMA_config(DMA1_Stream2, DMA_Init);

	NVIC_SetPriorityGrouping(2);
	NVIC_SetPriority (DMA1_Stream2_IRQn, 8);
	NVIC_EnableIRQ(DMA1_Stream2_IRQn);

	NVIC_SetPriorityGrouping(2);
	NVIC_SetPriority (UART4_IRQn, 7);
	NVIC_EnableIRQ(UART4_IRQn);
}

// Data should be a null terminated string if size == 0.
// if size > 0 then it will send size number of elements
void UART_Transmit(USART_TypeDef* huart, uint8_t* data, uint8_t size)
{
	if(size == 0)
	{
		for(uint8_t i=0; data[i] != '\0'; i++)
		{

			huart->DR = data[i];
			while(!(huart->SR & (1 << 6)));
		}
	}
	else
	{
		for(uint8_t i=0; i<size; i++)
		{

			huart->DR = data[i];
			while(!(huart->SR & (1 << 6)));
		}
	}
}



//static void ADC_config(ADC_TypeDef* hadc, uint8_t adc)
static void ADC_config(ADC_TypeDef* hadc, uint8_t channel)
{
	// Trigger detection on rising edge
	hadc->CR2 |= (1 << 28);
	// Timer 8 TRGO event as external trigger
	hadc->CR2 |= (1 << 27) | (1 << 26) | (1 << 25);
	// ADC DMA
	hadc->CR2 |= (1 << 9) | (1 << 8);

	/*** ONLY WORKS FOR CHANNELS 10-15 ***/
	// 15 cycles sampling time
	hadc->SMPR1 |= (1 << ((channel-10)*3));
	// First conversion
	hadc->SQR3 |= channel;

	// ADC enable
	hadc->CR2 |= (1 << 0);
}

static void DMA_config(DMA_Stream_TypeDef* hdma, DMA_InitVals idma)
{
	// Channel 7
	hdma->CR |= (idma.channel << 25);
	// Priority very high
	hdma->CR |= (3 << 16);
	// Data size for transfers
	hdma->CR |= (idma.memDataSize << 13) | (idma.periphDataSize << 11);
	// Memory increment mode
	hdma->CR |= (1 << 10);
	// Circular mode
	hdma->CR |= (1 << 8);
	// Number of transfers
	hdma->NDTR = idma.bufferSize;
	// Address to peripheral data registers
	hdma->PAR = idma.periphAddr;
	// Memory address
	hdma->M0AR = idma.memAddr;
	// Direction
	hdma->CR |= (idma.direction << 6);
	// Transfer and half transfer complete interrupt enable
	hdma->CR |= (idma.compIntEnable << 4) | (idma.halfIntEnable << 3);
	// Circular mode
	hdma->CR |= (idma.mode << 8);
	// Enable DMA
	hdma->CR |= (1 << 0);
}
