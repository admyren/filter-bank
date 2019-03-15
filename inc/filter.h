/*
 * filter.h
 *
 *  Created on: 30 nov. 2018
 *      Author: Myren
 */

#ifndef FILTER_H_
#define FILTER_H_

#include <stdint.h>
#include "drivers.h"

#define BASE_ID	6

#define BUFFER_SIZE 	400

#define MAX_ORDER_FIR	100
#define MAX_ORDER_IIR	20
#define MAX_FILTERS		5

// Destination buffers for the different blocks
extern float ADC1Buff[BUFFER_SIZE/2];
extern float ADC2Buff[BUFFER_SIZE/2];
extern float ADC3Buff[BUFFER_SIZE/2];
extern float ch1_1fBuff[BUFFER_SIZE/2];
extern float ch1_2fBuff[BUFFER_SIZE/2];
extern float ch1_3fBuff[BUFFER_SIZE/2];
extern float ch2_1fBuff[BUFFER_SIZE/2];
extern float ch2_2fBuff[BUFFER_SIZE/2];
extern float sum1Buff[BUFFER_SIZE/2];
extern float sum2Buff[BUFFER_SIZE/2];

typedef enum
{
	FIR,
	IIR
}Filter_Topology;

typedef enum
{
	LP,
	HP,
	BP,
	BS
}Filter_Type;

typedef enum
{
	NOT_USED,
	IS_USED
}Used;

typedef enum
{
	NOT_ACTIVE,
	IS_ACTIVE
}Active;

typedef enum
{
	ADC_1,
	ADC_2,
	ADC_3,
	SUM_1,
	SUM_2
}ChSrc_TypeDef;

typedef struct {
	uint16_t order;
	float* h;
	float* state_buffer;
	float* pSrc;
	float* pDst;
	uint16_t index;
}FIR_TypeDef;

typedef struct
{
	uint16_t order;
	uint16_t sNs; // Number of second order sections
	uint16_t fNs; // Number of first order sections (0 or 1)
	float* sCoeffs; // Second order coefficients
	float* fCoeffs; // First order coefficients
	float* sBuff;
	float* fBuff;
	float* pSrc;
	float* pDst;
}IIR_TypeDef;

typedef struct
{
	uint8_t ChActive[3];
	float* pSrc1;
	float* pSrc2;
	float* pSrc3;
	float* pDst;
}SUM_TypeDef;

typedef struct
{
	FIR_TypeDef* FIR_instance;
	IIR_TypeDef* IIR_instance;
	Filter_Topology topology;
	Filter_Topology topologyBuff;
	Filter_Type type;
	uint32_t cutOff[2];
	uint16_t order;
	Used used_flag;
	float* pSrc;
	float* pDst;
}Filter_TypeDef;

typedef struct
{
	Active active_flag;
	Filter_TypeDef* filters;
	ChSrc_TypeDef ChSrc;
	float* pSrc; // Channel source
	float* pDst; // Channel sink
}Ch_TypeDef;




void FIR_Filter(FIR_TypeDef* hFIR, uint16_t blkSize);
void IIR_Filter(IIR_TypeDef* hIIR, uint16_t blkSize);
void design_FIR(Filter_TypeDef* hFilter);
void design_IIR(Filter_TypeDef* hFilter);

void init_Channel(Ch_TypeDef* hCh, float* pDst, float* buff1, float* buff2);
void init_Sum(SUM_TypeDef* hsum, float* outBuff);
void exec_Filters(Ch_TypeDef* hCh);
void exec_Sum(SUM_TypeDef* hsum);





#endif /* FILTER_H_ */
