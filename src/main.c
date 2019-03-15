/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f429i_discovery.h"
#include "drivers.h"
#include "filter.h"
#include "routing.h"
/* Includes for freeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
			

#define FILTER_ORDER 	30

#define UART_BUFF_SIZE 50

xSemaphoreHandle ADCDMAInterruptSignal1;
xSemaphoreHandle ADCDMAInterruptSignal2;
xSemaphoreHandle COMInterruptSignal;

volatile uint8_t UART_buff[UART_BUFF_SIZE];

// Buffers for ADC's and DAC's
uint16_t ADC1InBuff[BUFFER_SIZE];
uint16_t ADC2InBuff[BUFFER_SIZE];
uint16_t ADC3InBuff[BUFFER_SIZE];
uint16_t DAC1OutBuff[BUFFER_SIZE];
uint16_t DAC2OutBuff[BUFFER_SIZE];

// Destination buffers for the different blocks
float ADC1Buff[BUFFER_SIZE/2];
float ADC2Buff[BUFFER_SIZE/2];
float ADC3Buff[BUFFER_SIZE/2];
float ch1_1fBuff[BUFFER_SIZE/2];
float ch1_2fBuff[BUFFER_SIZE/2];
float ch1_3fBuff[BUFFER_SIZE/2];
float ch2_1fBuff[BUFFER_SIZE/2];
float ch2_2fBuff[BUFFER_SIZE/2];
float sum1Buff[BUFFER_SIZE/2];
float sum2Buff[BUFFER_SIZE/2];

// Buffers used in the filter channels
float ch1_1Buff1[BUFFER_SIZE/2];
float ch1_1Buff2[BUFFER_SIZE/2];

float ch1_2Buff1[BUFFER_SIZE/2];
float ch1_2Buff2[BUFFER_SIZE/2];

float ch1_3Buff1[BUFFER_SIZE/2];
float ch1_3Buff2[BUFFER_SIZE/2];

float ch2_1Buff1[BUFFER_SIZE/2];
float ch2_1Buff2[BUFFER_SIZE/2];

float ch2_2Buff1[BUFFER_SIZE/2];
float ch2_2Buff2[BUFFER_SIZE/2];


// Declarations for filter types
Filter_TypeDef ch1_1F[MAX_FILTERS];
Filter_TypeDef ch1_2F[MAX_FILTERS];
Filter_TypeDef ch1_3F[MAX_FILTERS];
Filter_TypeDef ch2_1F[MAX_FILTERS];
Filter_TypeDef ch2_2F[MAX_FILTERS];
Ch_TypeDef ch1_1;
Ch_TypeDef ch1_2;
Ch_TypeDef ch1_3;
Ch_TypeDef ch2_1;
Ch_TypeDef ch2_2;
SUM_TypeDef sum1;
SUM_TypeDef sum2;


void SystemClock_Config(void);

static uint8_t tokenize(uint8_t* in, uint8_t out[10][10], uint8_t del)
{
	uint16_t i = 0, j = 0;
	uint8_t args = 0;
	while((in[i] != '|') && (i < 25))
	{
		while((in[i] != del) && (i < 25))
		{
			out[args][j] = in[i];
			i++; j++;
		}
		out[args][j] = '\0';
		args++; i++; j = 0;
	}
	return args;
}
//C11:GL:SS:ADC1:|

static void DSPTask(void* params)
{
	while(1)
	{
		/*** WAIT FOR FIRST HALF OF BUFFER ***/
		if(xSemaphoreTake(ADCDMAInterruptSignal1, 0xffff) == pdTRUE)
		{
			for(uint16_t i=0; i<BUFFER_SIZE/2; i++)
			{
				ADC1Buff[i] = (float)(ADC1InBuff[i]-2048);
				ADC2Buff[i] = (float)(ADC2InBuff[i]-2048);
				ADC3Buff[i] = (float)(ADC3InBuff[i]-2048);
			}

			exec_Filters(&ch1_1);
			exec_Filters(&ch1_2);
			exec_Filters(&ch1_3);

			exec_Sum(&sum1);
			exec_Sum(&sum2);

			exec_Filters(&ch2_1);
			exec_Filters(&ch2_2);

			for(uint16_t i=0; i<BUFFER_SIZE/2; i++)
			{
				DAC1OutBuff[i] = (uint16_t)(ch2_1fBuff[i] + 2048);
				DAC2OutBuff[i] = (uint16_t)(ch2_2fBuff[i] + 2048);
			}
		}

		/*** WAIT FOR SECOND HALF OF BUFFER ***/
		if(xSemaphoreTake(ADCDMAInterruptSignal2, 0xffff) == pdTRUE)
		{
			for(uint16_t i=0; i<BUFFER_SIZE/2; i++)
			{
				ADC1Buff[i] = (float)(ADC1InBuff[i+BUFFER_SIZE/2]-2048);
				ADC2Buff[i] = (float)(ADC2InBuff[i+BUFFER_SIZE/2]-2048);
				ADC3Buff[i] = (float)(ADC3InBuff[i+BUFFER_SIZE/2]-2048);
			}

			exec_Filters(&ch1_1);
			exec_Filters(&ch1_2);
			exec_Filters(&ch1_3);

			exec_Sum(&sum1);
			exec_Sum(&sum2);

			exec_Filters(&ch2_1);
			exec_Filters(&ch2_2);

			for(uint16_t i=0; i<BUFFER_SIZE/2; i++)
			{
				DAC1OutBuff[i+BUFFER_SIZE/2] = (uint16_t)(ch2_1fBuff[i]+2048);
				DAC2OutBuff[i+BUFFER_SIZE/2] = (uint16_t)(ch2_2fBuff[i]+2048);
			}
		}
	}
}

static void COMTask(void* params)
{
	uint8_t res[10][10];
	Ch_TypeDef* chTemp = NULL;
	SUM_TypeDef* sumTemp = NULL;
	while(1)
	{
		if(xSemaphoreTake(COMInterruptSignal, 0xffff) == pdTRUE)
		{
				tokenize((uint8_t*)UART_buff, res, ':');

				if(!strcmp("C11", (char*)res[0]))
					chTemp = &ch1_1;
				else if(!strcmp("C12", (char*)res[0]))
					chTemp = &ch1_2;
				else if(!strcmp("C13", (char*)res[0]))
					chTemp = &ch1_3;
				else if(!strcmp("C21", (char*)res[0]))
					chTemp = &ch2_1;
				else if(!strcmp("C22", (char*)res[0]))
					chTemp = &ch2_2;
				else if(!strcmp("S1", (char*)res[0]))
					sumTemp = &sum1;
				else if(!strcmp("S2", (char*)res[0]))
					sumTemp = &sum2;

				if(!strcmp("S1", (char*)res[0]) || !strcmp("S2", (char*)res[0]))
				{
					sumTemp->ChActive[0] = atoi((char*)res[1]);
					sumTemp->ChActive[1] = atoi((char*)res[2]);
					sumTemp->ChActive[2] = atoi((char*)res[3]);
				}
				else if(!strcmp("GL", (char*)res[1])) // Global parameters to a whole channel
				{
					if(!strcmp("SS", (char*)res[2]))
					{
						if(!strcmp("ADC1", (char*)res[3]))
						{
							setInput(chTemp, ADC_1);
							UART_Transmit(UART4, (uint8_t*)"OK\0", 3);
						}
						else if(!strcmp("ADC2", (char*)res[3]))
						{
							setInput(chTemp, ADC_2);
							UART_Transmit(UART4, (uint8_t*)"OK\0", 3);
						}
						else if(!strcmp("ADC3", (char*)res[3]))
						{
							setInput(chTemp, ADC_3);
							UART_Transmit(UART4, (uint8_t*)"OK\0", 3);
						}
					}
					else if(!strcmp("INI", (char*)res[2]))
					{
						chTemp->active_flag = IS_ACTIVE;
						UART_Transmit(UART4, (uint8_t*)"OK\0", 3);
					}
					else if(!strcmp("DIN", (char*)res[2]))
					{
						chTemp->active_flag = NOT_ACTIVE;
						UART_Transmit(UART4, (uint8_t*)"OK\0", 3);
					}
				}
				else if(!strcmp("LO", (char*)res[1])) // Local parameters to a filter
				{
					uint8_t id = atoi((char*)res[2]);
					//chTemp->filters[id]
					if(!strcmp("STO", (char*)res[3]))
					{
						if(!strcmp("FIR", (char*)res[4])) // Set filter topology
						{
							chTemp->filters[id].topologyBuff = FIR;
							UART_Transmit(UART4, (uint8_t*)"OK\0", 3);
						}
						else if(!strcmp("IIR", (char*)res[4]))
						{
							chTemp->filters[id].topologyBuff = IIR;
							UART_Transmit(UART4, (uint8_t*)"OK\0", 3);
						}
					}
					else if(!strcmp("STY", (char*)res[3])) // Set filter type
					{
						if(!strcmp("LP", (char*)res[4]))
						{
							chTemp->filters[id].type = LP;
							UART_Transmit(UART4, (uint8_t*)"OK\0", 3);
						}
						else if(!strcmp("HP", (char*)res[4]))
						{
							chTemp->filters[id].type = HP;
							UART_Transmit(UART4, (uint8_t*)"OK\0", 3);
						}
						else if(!strcmp("BP", (char*)res[4]))
						{
							chTemp->filters[id].type = BP;
							UART_Transmit(UART4, (uint8_t*)"OK\0", 3);
						}
						else if(!strcmp("BS", (char*)res[4]))
						{
							chTemp->filters[id].type = BS;
							UART_Transmit(UART4, (uint8_t*)"OK\0", 3);
						}
					}
					else if(!strcmp("SCU", (char*)res[3])) // Set filter cut off
					{
						uint32_t f_cl = atoi((char*)res[4]);
						uint32_t f_cu = atoi((char*)res[5]);
						chTemp->filters[id].cutOff[0] = f_cl;
						chTemp->filters[id].cutOff[1] = f_cu;
						UART_Transmit(UART4, (uint8_t*)"OK\0", 3);
					}
					else if(!strcmp("SOR", (char*)res[3])) // Set filter order
					{
						uint16_t order = atoi((char*)res[4]);
						chTemp->filters[id].order = order;
						UART_Transmit(UART4, (uint8_t*)"OK\0", 3);
					}
					else if(!strcmp("BLD", (char*)res[3])) // Build filter
					{
						if(chTemp->filters[id].topologyBuff == FIR)
						{
							design_FIR(&chTemp->filters[id]);
							UART_Transmit(UART4, (uint8_t*)"OK\0", 3);
						}
						else if(chTemp->filters[id].topologyBuff == IIR)
						{
							design_IIR(&chTemp->filters[id]);
							UART_Transmit(UART4, (uint8_t*)"OK\0", 3);
						}
					}
					else if(!strcmp("ACT", (char*)res[3])) // Activate filter
					{
						chTemp->filters[id].used_flag = IS_USED;
						UART_Transmit(UART4, (uint8_t*)"OK\0", 3);
					}
					else if(!strcmp("DCT", (char*)res[3])) // Deactivate filter
					{
						chTemp->filters[id].used_flag = NOT_USED;
						UART_Transmit(UART4, (uint8_t*)"OK\0", 3);
					}
				}
//				else if(!strcmp("ADD", (char*)res[1]))
//				{
//					sumTemp->ChActive[0] = atoi((char*)res[2]);
//					sumTemp->ChActive[1] = atoi((char*)res[3]);
//					sumTemp->ChActive[2] = atoi((char*)res[4]);
//					UART_Transmit(UART4, (uint8_t*)"OK\0", 3);
//				}
		}
	}
}

int main(void)
{
	HAL_Init();
	SystemClock_Config();
	UART_Init(UART4, 230400, (uint8_t*)UART_buff, UART_BUFF_SIZE);
	/*** FOR DEBUG ***/
	// PE8, PE9, PE10
	RCC->AHB1ENR |= (1 << 4);
	GPIOE->MODER |= (1 << 16) | (1 << 18) | (1 << 20);

	ADCDMAInterruptSignal1 = xSemaphoreCreateBinary();
	ADCDMAInterruptSignal2 = xSemaphoreCreateBinary();
	COMInterruptSignal = xSemaphoreCreateBinary();

	ch1_1.filters = ch1_1F;
	ch1_2.filters = ch1_2F;
	ch1_3.filters = ch1_3F;
	ch2_1.filters = ch2_1F;
	ch2_2.filters = ch2_2F;
	ch1_1.ChSrc = ADC_1;
	ch1_2.ChSrc = ADC_1;
	ch1_3.ChSrc = ADC_3;
	ch2_1.ChSrc = SUM_1;
	ch2_2.ChSrc = SUM_2;
	ch2_1.active_flag = IS_ACTIVE;
	ch2_2.active_flag = IS_ACTIVE;




	init_Channel(&ch1_1, ch1_1fBuff, ch1_1Buff1, ch1_1Buff2);
	init_Channel(&ch1_2, ch1_2fBuff, ch1_2Buff1, ch1_2Buff2);
	init_Channel(&ch1_3, ch1_3fBuff, ch1_3Buff1, ch1_3Buff2);
	init_Sum(&sum1, sum1Buff);
	init_Sum(&sum2, sum2Buff);
	init_Channel(&ch2_1, ch2_1fBuff, ch2_1Buff1, ch2_1Buff2);
	init_Channel(&ch2_2, ch2_2fBuff, ch2_2Buff1, ch2_2Buff2);
/*
	ch1_1.filters[0].topology = IIR;
	ch1_1.filters[0].order = 4;
	ch1_1.filters[0].IIR_instance->order = 4;
	ch1_1.filters[0].IIR_instance->sNs = 2;
	ch1_1.filters[0].IIR_instance->fNs = 0;
	ch1_1.filters[0].IIR_instance->sCoeffs[0] = -1.5610; 	// a_1
	ch1_1.filters[0].IIR_instance->sCoeffs[1] = 0.6414;		// a_2
	ch1_1.filters[0].IIR_instance->sCoeffs[2] = 0.0201;		// b_2
	ch1_1.filters[0].IIR_instance->sCoeffs[3] = 0.0201;		// b_0
	ch1_1.filters[0].IIR_instance->sCoeffs[4] = 0.0402;		// b_1
*/
	sum1.ChActive[0] = 0;
	sum1.ChActive[1] = 0;
	sum1.ChActive[2] = 0;

	sum2.ChActive[0] = 0;
	sum2.ChActive[1] = 0;
	sum2.ChActive[2] = 0;


	initADC_DAC(ADC1InBuff, ADC2InBuff, ADC3InBuff,
			    DAC1OutBuff, DAC2OutBuff, BUFFER_SIZE);


	xTaskCreate(DSPTask, "DSPTask", 2000, NULL, 5, NULL);
	xTaskCreate(COMTask, "COMTask", 2000, NULL, 6, NULL);
	vTaskStartScheduler();

	return 0;
}

// Synchronization for ADC1 buffer
void DMA2_Stream0_IRQHandler(void)
{
	portBASE_TYPE higherPrio = pdFALSE;
	// Half transfer complete interrupt?
	if(DMA2->LISR & (1 << 4))
	{
		// Clear interrupt flag
		DMA2->LIFCR |= 1 << 4;
		xSemaphoreGiveFromISR(ADCDMAInterruptSignal1, &higherPrio);
		portEND_SWITCHING_ISR(higherPrio);
	}
	// Transfer complete interrupt?
	else if(DMA2->LISR & (1 << 5))
	{
		// Clear interrupt flag
		DMA2->LIFCR |= 1 << 5;
		xSemaphoreGiveFromISR(ADCDMAInterruptSignal2, &higherPrio);
		portEND_SWITCHING_ISR(higherPrio);
	}
}

void DMA1_Stream2_IRQHandler(void)
{
	portBASE_TYPE higherPrio = pdFALSE;
	// Transfer complete interrupt?
	if(DMA1->LISR & (1 << 21))
	{
		// Clear interrupt flag
		DMA1->LIFCR |= 1 << 21;
		// Enable DMA stream again
		DMA1_Stream2->NDTR = UART_BUFF_SIZE;
		DMA1_Stream2->CR |= (1 << 0);
		xSemaphoreGiveFromISR(COMInterruptSignal, &higherPrio);
		portEND_SWITCHING_ISR(higherPrio);
	}
}

void UART4_IRQHandler(void)
{
	// Idle line detect interrupt?
	if(UART4->SR & (1 << 4))
	{
		// Disable DMA stream to force transfer complete interrupt
		DMA1_Stream2->CR &= ~(1 << 0);
		uint8_t dummy = UART4->DR;
		(void)dummy; // Prevent compiler warnings
	}
}



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
#if 0
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16; // 1MHz clock input to PLL
  RCC_OscInitStruct.PLL.PLLN = 360; // PLL multiplier, output of PLL = 1*360 = 360MHz
  RCC_OscInitStruct.PLL.PLLP = 2; // System clock = output of PLL / 2 = 180MHz
  RCC_OscInitStruct.PLL.PLLQ = 7;
#else
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  //RCC_OscInitStruct.HSECalibrationValue = 8;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8; // 1MHz clock input to PLL
  RCC_OscInitStruct.PLL.PLLN = 360; // PLL multiplier, output of PLL = 1*360 = 360MHz
  RCC_OscInitStruct.PLL.PLLP = 2; // System clock = output of PLL / 2 = 180MHz
  RCC_OscInitStruct.PLL.PLLQ = 7;
#endif
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    //_Error_Handler(__FILE__, __LINE__);
  }

  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK |
		                        RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    //_Error_Handler(__FILE__, __LINE__);
  }

  SystemCoreClockUpdate();


  /**Configure the Systick
  */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  /**Configure the Systick interrupt time
  */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}
