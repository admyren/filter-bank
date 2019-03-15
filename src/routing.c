/*
 * routing.c
 *
 *  Created on: 30 nov. 2018
 *      Author: Myren
 */

#include "routing.h"


void setInput(Ch_TypeDef* hCh, ChSrc_TypeDef Ch_Src)
{
	switch(Ch_Src)
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
	default:
		break;
	}
	hCh->filters[0].pSrc = hCh->pSrc;
	hCh->filters[0].FIR_instance->pSrc = hCh->pSrc;
	hCh->filters[0].IIR_instance->pSrc = hCh->pSrc;
}


