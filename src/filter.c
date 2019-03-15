/*
 * filter.c
 *
 *  Created on: 30 nov. 2018
 *      Author: Myren
 */

#include "filter.h"
#include "drivers.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

static float sinc(float x)
{
	if(x == 0)
	{
		return 1.0;
	}
	else
	{
		return sin(M_PI*x)/(M_PI*x);
	}
}

// Calculates coefficients for an FIR filter
void design_FIR(Filter_TypeDef* hFilter)
{
	uint16_t N = hFilter->order;
	hFilter->FIR_instance->order = hFilter->order;
	hFilter->topology = hFilter->topologyBuff;
	float w_cl = M_PI*(float)hFilter->cutOff[0]/(SAMPLING_FREQUENCY/2);
	float w_cu = M_PI*(float)hFilter->cutOff[1]/(SAMPLING_FREQUENCY/2);
	float n;
	if(!(N%2))
	{
		n = -(float)(N/2.0) + 0.5;
	}
	else
	{
		n = -floor(N/2.0);
	}

	if(hFilter->type == LP)
	{
		uint16_t k;
		for(k=0; k<N; k++)
		{
			hFilter->FIR_instance->h[k] = (w_cl/M_PI)*sinc((w_cl*n)/M_PI);
			n = n + 1;
		}

	}
	else if(hFilter->type == HP)
	{
		uint16_t k;
		for(k=0; k<N; k++)
		{
			hFilter->FIR_instance->h[k] = (sinc(n) - (w_cl/M_PI)*sinc((w_cl*n)/M_PI));
			n = n + 1;
		}
	}
	else if(hFilter->type == BP)
	{
		uint16_t k;
		for(k=0; k<N; k++)
		{
			hFilter->FIR_instance->h[k] = ((w_cu/M_PI)*sinc((w_cu*n)/M_PI) - (w_cl/M_PI)*sinc((w_cl*n)/M_PI));
			n = n + 1;
		}
	}
	else if(hFilter->type == BS)
	{
		uint16_t k;
		for(k=0; k<N; k++)
		{
			hFilter->FIR_instance->h[k] = (sinc(n) - (((w_cu/M_PI)*sinc((w_cu*n)/M_PI) - (w_cl/M_PI)*sinc((w_cl*n)/M_PI))));
			n = n + 1;
		}
	}
}

// Calculates coefficients for an IIR filter
void design_IIR(Filter_TypeDef* hFilter)
{
	IIR_TypeDef* hIIR = hFilter->IIR_instance;
	hFilter->topology = hFilter->topologyBuff;
	hIIR->order = hFilter->order;
	hIIR->sNs = hFilter->order/2;
	hIIR->fNs = hFilter->order%2;
	float w_cl = 2*M_PI*(float)hFilter->cutOff[0];///(SAMPLING_FREQUENCY/2);
	float w_cu = 2*M_PI*(float)hFilter->cutOff[1];///(SAMPLING_FREQUENCY/2);

	// Storage for continuous time coefficients
	float as_s[3], bs_s[3], af_s[2], bf_s[2];

	as_s[0] = 0;
	as_s[1] = 0;
	as_s[2] = 0;
	bs_s[0] = 0;
	bs_s[1] = 0;
	bs_s[2] = 0;
	af_s[0] = 0;
	af_s[1] = 0;
	bf_s[0] = 0;
	bf_s[1] = 0;

	// Storage for discrete time coefficients
	float as_z[3], bs_z[3], af_z[2], bf_z[2];
	//float K = 2*SAMPLING_FREQUENCY;//15800;//w_cl/(tan((w_cl/SAMPLING_FREQUENCY)/(2.0)));
	float K = 0;//w_cl/(tan((w_cl/SAMPLING_FREQUENCY)/(2.0)));

	if(hFilter->type == LP)
	{
		K = w_cl/(tan((w_cl/SAMPLING_FREQUENCY)/(2.0)));
		if(hIIR->sNs > 0)
		{
			bs_s[0] = 0; bs_s[1] = 0; bs_s[2] = w_cl*w_cl;
			as_s[0] = 1; as_s[1] = M_SQRT2*w_cl; as_s[2] = w_cl*w_cl;
		}
		if(hIIR->fNs > 0)
		{
			bf_s[0] = 0; bf_s[1] = w_cl;
			af_s[0] = 1; af_s[1] = w_cl;
		}
	}
	else if(hFilter->type == HP)
	{
		K = w_cl/(tan((w_cl/SAMPLING_FREQUENCY)/(2.0)));
		if(hIIR->sNs > 0)
		{
			bs_s[0] = 1; bs_s[1] = 0; bs_s[2] = 0;
			as_s[0] = 1; as_s[1] = M_SQRT2*w_cl; as_s[2] = w_cl*w_cl;
		}
		if(hIIR->fNs > 0)
		{
			bf_s[0] = 1; bf_s[1] = 0;
			af_s[0] = 1; af_s[1] = w_cl;
		}
	}
	else if(hFilter->type == BP)
	{
		// Center frequency
		float w_cf = (w_cl+w_cu)/2;
		K = w_cf/(tan((w_cf/SAMPLING_FREQUENCY)/(2.0)));
		bs_s[0] = 0; bs_s[1] = w_cu-w_cl; bs_s[2] = 0;
		as_s[0] = 1; as_s[1] = w_cu-w_cl; as_s[2] = w_cl*w_cu;

		hIIR->sNs = hFilter->order;
		hIIR->fNs = 0;


	}
	else if(hFilter->type == BS)
	{
		// Center frequency
		float w_cf = (w_cl+w_cu)/2;
		K = w_cf/(tan((w_cf/SAMPLING_FREQUENCY)/(2.0)));
		bs_s[0] = 1; bs_s[1] = 0; bs_s[2] = w_cl*w_cu;
		as_s[0] = 1; as_s[1] = w_cu-w_cl; as_s[2] = w_cl*w_cu;

		hIIR->sNs = hFilter->order;
		hIIR->fNs = 0;
	}

	float den = as_s[0]*K*K + as_s[1]*K + as_s[2];
	as_z[0] = 1.0;
	as_z[1] = (2*as_s[2] - 2*as_s[0]*K*K)/den;
	as_z[2] = (as_s[0]*K*K - as_s[1]*K + as_s[2])/den;

	bs_z[0] = (bs_s[0]*K*K + bs_s[1]*K + bs_s[2])/den;
	bs_z[1] = (2*bs_s[2] - 2*bs_s[0]*K*K)/den;
	bs_z[2] = (bs_s[0]*K*K - bs_s[1]*K + bs_s[2])/den;

	den = af_s[0]*K + af_s[1];

	af_z[0] = 1.0;
	af_z[1] = (af_s[1] - af_s[0]*K)/den;

	bf_z[0] = (bf_s[0]*K + bf_s[1])/den;
	bf_z[1] = (bf_s[1] - bf_s[0]*K)/den;

	hIIR->sCoeffs[0] = as_z[1]; 	// a_1
	hIIR->sCoeffs[1] = as_z[2];		// a_2
	hIIR->sCoeffs[2] = bs_z[2];		// b_2
	hIIR->sCoeffs[3] = bs_z[0];		// b_0
	hIIR->sCoeffs[4] = bs_z[1];		// b_1

	hIIR->fCoeffs[0] = af_z[1];		// a_1
	hIIR->fCoeffs[1] = bf_z[1];		// b_1
	hIIR->fCoeffs[2] = bf_z[0];		// b_0
	// -u _printf_float

#if 1
	char str[40];

	sprintf(str, "%f  ", hIIR->sCoeffs[0]);
	UART_Transmit(UART4, (uint8_t*)str, 0);
	sprintf(str, "%f  ", hIIR->sCoeffs[1]);
	UART_Transmit(UART4, (uint8_t*)str, 0);
	sprintf(str, "%f  ", hIIR->sCoeffs[3]);
	UART_Transmit(UART4, (uint8_t*)str, 0);
	sprintf(str, "%f  ", hIIR->sCoeffs[4]);
	UART_Transmit(UART4, (uint8_t*)str, 0);
	sprintf(str, "%f  \n", hIIR->sCoeffs[2]);
	UART_Transmit(UART4, (uint8_t*)str, 0);

	sprintf(str, "%f  ", hIIR->fCoeffs[0]);
	UART_Transmit(UART4, (uint8_t*)str, 0);
	sprintf(str, "%f  ", hIIR->fCoeffs[2]);
	UART_Transmit(UART4, (uint8_t*)str, 0);
	sprintf(str, "%f  ", hIIR->fCoeffs[1]);
	UART_Transmit(UART4, (uint8_t*)str, 0);


	UART_Transmit(UART4, (uint8_t*)"\n\n", 0);

	sprintf(str, "%f rad/s SOS: %d FOS: %d\n", w_cl, hIIR->sNs, hIIR->fNs);
	UART_Transmit(UART4, (uint8_t*)str, 0);
#endif


}
/*  Initializes and allocates memory to all buffers in the signal chain.
 *  This function should only be called once during startup.
 *  The memory for the filter buffers is allocated dynamically but is never freed,
 *  basically acting as a static allocation.
 */
void init_Channel(Ch_TypeDef* hCh, float* pDst, float* buff1, float* buff2)
{
	uint8_t i;
	Filter_TypeDef* hFilters = hCh->filters;
	hCh->active_flag = NOT_ACTIVE;
	hCh->pDst = pDst;

	switch(hCh->ChSrc)
	{
		case ADC_1:
			hCh->pSrc = ADC1Buff;
			break;
		case ADC_2:
			hCh->pSrc = ADC2Buff;
			break;
		case ADC_3:
			hCh->pSrc = ADC3Buff;
			break;
		case SUM_1:
			hCh->pSrc = sum1Buff;
			break;
		case SUM_2:
			hCh->pSrc = sum2Buff;
			break;
		default:
			break;
	}

	for(i=0; i<MAX_FILTERS; i++)
	{
		if(i == 0)
		{
			hFilters[0].pSrc = hCh->pSrc;
			hFilters[0].pDst = buff2;
		}
		else if((i%2) == 0)
		{
			hFilters[i].pSrc = buff1;
			hFilters[i].pDst = buff2;
		}
		else
		{
			hFilters[i].pSrc = buff2;
			hFilters[i].pDst = buff1;
		}

		if(i == MAX_FILTERS-1)
		{
			hFilters[i].pDst = hCh->pDst;
		}

		hFilters[i].topologyBuff = FIR;
		hFilters[i].topology = FIR;
		hFilters[i].type = LP;
		hFilters[i].cutOff[0] = 1000;
		hFilters[i].cutOff[1] = 2000;
		hFilters[i].order = 10;
		hFilters[i].used_flag = NOT_USED;

		hFilters[i].FIR_instance = malloc(sizeof(FIR_TypeDef));
		hFilters[i].FIR_instance->h = malloc(MAX_ORDER_FIR*sizeof(float));
		hFilters[i].FIR_instance->state_buffer = malloc((MAX_ORDER_FIR+BUFFER_SIZE/2)*sizeof(float));
		hFilters[i].FIR_instance->order = hFilters[i].order;
		hFilters[i].FIR_instance->index = 0;
		hFilters[i].FIR_instance->pSrc = hFilters[i].pSrc;
		hFilters[i].FIR_instance->pDst = hFilters[i].pDst;


		hFilters[i].IIR_instance = malloc(sizeof(IIR_TypeDef));
		hFilters[i].IIR_instance->sCoeffs = malloc(5*sizeof(float));
		hFilters[i].IIR_instance->fCoeffs = malloc(3*sizeof(float));
		hFilters[i].IIR_instance->sBuff = malloc(MAX_ORDER_IIR*sizeof(float));
		hFilters[i].IIR_instance->fBuff = malloc(sizeof(float));
		hFilters[i].IIR_instance->order = hFilters[i].order;
		hFilters[i].IIR_instance->sNs = hFilters[i].order/2;
		hFilters[i].IIR_instance->fNs = hFilters[i].order%2;
		hFilters[i].IIR_instance->pSrc = hFilters[i].pSrc;
		hFilters[i].IIR_instance->pDst = hFilters[i].pDst;

		design_FIR(&hFilters[i]);
		design_IIR(&hFilters[i]);


	}
}


// Execute a filter channel
void exec_Filters(Ch_TypeDef* hCh)
{
	uint8_t i,j = 0;
	Filter_TypeDef* hFilters = hCh->filters;

	// If channel is not active, copy channel source buffer to destination buffer
	if(hCh->active_flag == NOT_ACTIVE)
	{
		for(i=0; i<BUFFER_SIZE/2; i++)
		{
			hCh->pDst[i] = hCh->pSrc[i];
		}
	}
	else
	{
		for(i=0; i<MAX_FILTERS; i++)
		{
			// Execute filter if it is used
			if(hFilters[i].used_flag == IS_USED)
			{
				if(hFilters[i].topology == FIR)
				{
					FIR_Filter(hFilters[i].FIR_instance, BUFFER_SIZE/2);
				}
				else if(hFilters[i].topology == IIR)
				{
					IIR_Filter(hFilters[i].IIR_instance, BUFFER_SIZE/2);

				}
			}
			// Else, copy it to the next buffer
			else if(hFilters[i].used_flag == NOT_USED)
			{
				for(j=0; j<BUFFER_SIZE/2; j++)
				{
					hFilters[i].pDst[j] = hFilters[i].pSrc[j];
				}
			}

		}
	}
}

// Sets the source and destination buffers of a sum node
void init_Sum(SUM_TypeDef* hsum, float* outBuff)
{
	hsum->pSrc1 = ch1_1fBuff;
	hsum->pSrc2 = ch1_2fBuff;
	hsum->pSrc3 = ch1_3fBuff;
	hsum->pDst = outBuff;
}

// This function could be optimized
void exec_Sum(SUM_TypeDef* hsum)
{
	for(uint16_t i=0; i<BUFFER_SIZE/2; i++)
	{
		hsum->pDst[i] = (hsum->ChActive[0]*hsum->pSrc1[i] +
				         hsum->ChActive[1]*hsum->pSrc2[i] +
						 hsum->ChActive[2]*hsum->pSrc3[i]);
		if(hsum->pDst[i] > 2047)
		{
			hsum->pDst[i] = 2047;
		}
		else if(hsum->pDst[i] < -2047)
		{
			hsum->pDst[i] = -2047;
		}
	}
}

// Cascaded IIR filter
void IIR_Filter(IIR_TypeDef* hIIR, uint16_t blkSize)
{
	uint16_t i,n,l,s;
	float temp16;
	float w_0,temp32;

	s = hIIR->sNs*2; // Set up circular buffer w[]

	for (l=0,n=0; n<blkSize; n++) // IIR filtering
	{
		w_0 = hIIR->pSrc[n]; // Scale input to prevent overflow
		// Calculate second order sections
		for (i=0; i<hIIR->sNs; i++)
		{
			// temp32 = a_1*v[n-1]
			temp32 = (hIIR->sBuff[l])*hIIR->sCoeffs[0];
			l=(l+hIIR->sNs)%s;
			w_0 = w_0 - temp32;

			// temp32 = a_2*v[n-2]
			temp32 = (hIIR->sBuff[l])*hIIR->sCoeffs[1];
			// w_0 now holds v[n]
			w_0 = w_0 - temp32;

			// temp16 = v[n-2]
			temp16 = hIIR->sBuff[l];
			// Save v[n]
			hIIR->sBuff[l] = (w_0); // Save in Q15 format
			// w_0 = b_2*v[n-2]
			w_0 = temp16*hIIR->sCoeffs[2];

			// temp32 = b_0*v[n]
			temp32 = (int32_t)(hIIR->sBuff[l])*hIIR->sCoeffs[3];

			l=(l+hIIR->sNs)%s;
			w_0 = w_0 + temp32;
			// temp32 = b_1*v[n-1]
			temp32 = (hIIR->sBuff[l])*hIIR->sCoeffs[4];

			l=(l+1)%s;
			w_0 = w_0 + temp32;

		}
		// Calculate first order section
		if(hIIR->fNs == 1)
		{
			// temp32 = a_1*v[n-1]
			w_0 -= *hIIR->fBuff*hIIR->fCoeffs[0];
			// w_0 now holds v[n]
			//w_0 -= temp32;

			// temp16 = v[n-1]
			temp16 = *hIIR->fBuff;
			// Save v[n]
			*hIIR->fBuff = w_0; // Save in Q15 format
			// w_0 = b_1*v[n-1]
			w_0 = temp16*hIIR->fCoeffs[2];
			//w_0 >>= 1;

			// temp32 = b_0*v[n]
			w_0 += *hIIR->fBuff*hIIR->fCoeffs[1];
			//w_0 += temp32;
			//w_0 <<= 1;
		}
		hIIR->pDst[n] = w_0;
	}

}

void FIR_Filter(FIR_TypeDef* hFIR, uint16_t blkSize)
{
	int16_t i,j,k;
	float sum;
	float *c;
	float *x = hFIR->pSrc;
	float *y = hFIR->pDst;
	k = hFIR->index;;
	for (j=0; j<blkSize; j++) // Block processing
	{
		hFIR->state_buffer[k] = *x++; // Get current data to delay line

		c = hFIR->h;
		for (sum=0, i=0; i<hFIR->order; i++) // FIR filter processing
		{
			sum += *c++*hFIR->state_buffer[k++];
			//sum += vmull_n_s16(*c++, w[k++]);
			if (k == hFIR->order) // Simulate circular buffer
				k = 0;
		}
		*y++ = sum; // Save filter output
		//hFIR->pDst++;
		if (k-- <= 0) // Update index for next time
			k = hFIR->order-1;
	}
	hFIR->index = k; // Update circular buffer index

}





