/*
 * drivers.h
 *
 *  Created on: 30 nov. 2018
 *      Author: Myren
 */

#ifndef DRIVERS_H_
#define DRIVERS_H_

#include "stm32f4xx.h"
#include "stm32f429xx.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdlib.h>

#define SAMPLING_FREQUENCY	50000





typedef enum
{
	SIZE_BYTE = 0,
	SIZE_HALF_WORD = 1,
	SIZE_WORD = 2
}DMA_DataSize;

typedef struct
{
	uint32_t channel;				// DMA channel
	DMA_DataSize periphDataSize;	// Peripheral data size
	DMA_DataSize memDataSize;		// Memory data size
	uint32_t direction;				// Direction, memory->peripheral(1) or peripheral->memory(0)
	uint32_t bufferSize;			// Number of element in the DMA buffer
	uint32_t periphAddr;			// Address to peripheral data register
	uint32_t memAddr;				// Address to first element of DMA buffer
	uint32_t halfIntEnable;			// DMA half transfer complete interrupt enable
	uint32_t compIntEnable;			// DMA transfer complete interrupt enable
	uint32_t mode;					// Circular mode or not
}DMA_InitVals;


void initADC_DAC(uint16_t *bufferIn1, uint16_t *bufferIn2, uint16_t *bufferIn3,
					uint16_t *bufferOut1, uint16_t *bufferOut2, uint16_t bufferSize);
void UART_Init(USART_TypeDef* huart, uint32_t baud, uint8_t* pDst, uint8_t size);
void UART_Transmit(USART_TypeDef* huart, uint8_t* data, uint8_t size);

//void initUART(uint32_t BAUD);
//void UART_puts(uint8_t* data, uint16_t size);

#endif /* DRIVERS_H_ */
