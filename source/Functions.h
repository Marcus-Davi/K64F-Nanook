/*
 * functions.h
 *
 *  Created on: 28 de dez de 2017
 *      Author: davi2
 */

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

#include "fsl_pit.h"
#include "fsl_gpio.h"




//Canal do PIT para delays
#define NON_INTERRUPT_PIT_CHN kPIT_Chnl_3


//Delay temporizado em "time" microssegundos.
static inline void delay_us(uint32_t time){
	PIT_SetTimerPeriod(PIT,NON_INTERRUPT_PIT_CHN , USEC_TO_COUNT(time,CLOCK_GetBusClkFreq()));
	PIT_StartTimer(PIT, NON_INTERRUPT_PIT_CHN);
	while(PIT_GetStatusFlags(PIT, NON_INTERRUPT_PIT_CHN) != kPIT_TimerFlag);
	PIT_StopTimer(PIT, NON_INTERRUPT_PIT_CHN);
	PIT_ClearStatusFlags(PIT, NON_INTERRUPT_PIT_CHN , kPIT_TimerFlag);
}




#endif /* FUNCTIONS_H_ */
