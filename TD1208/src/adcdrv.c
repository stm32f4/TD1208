/*
 * adcdrv.c
 *
 *  Created on: 2 juin 2015
 *      Author: thierry
 */

#include <adcdrv.h>

#include <stdio.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_cmu.h"

void TD_USER_InitAdc(ADC_SingleInput_TypeDef channel) {
	/***************************************************************************//**
	 * @brief
	 *   Configure ADC usage for this application.
	 *******************************************************************************/
	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
	ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;

	/* Init common settings for both single conversion and scan mode */
	init.timebase = ADC_TimebaseCalc(0);
	/* Set ADC clock to 1 MHz, use default HFPERCLK */
	init.prescale = ADC_PrescaleCalc(100000, 0);

	ADC_Init(ADC0, &init);

	/* Init for single conversion use */
	singleInit.reference = adcRef2V5;
	singleInit.input = channel;
	singleInit.resolution = adcRes12Bit;

	/* The datasheet specifies a minimum acquisition time when sampling vdd/3 */
	/* 32 cycles should be safe for all ADC clock frequencies */
	singleInit.acqTime = adcAcqTime32;

	ADC_InitSingle(ADC0, &singleInit);
}

bool TD_USER_ReadAdc(uint32_t *value) {

	/* Start ADC clock */
	CMU_ClockEnable(cmuClock_ADC0, true);

	ADC_Start(ADC0, adcStartSingle);

	uint32_t now = RTC->CNT;

	/* Wait while conversion is active */
	while ((ADC0->STATUS & ADC_STATUS_SINGLEACT) && (RTC->CNT - now < 32))
		;

	if (ADC0->STATUS & ADC_STATUS_SINGLEDV) {
		/* Get ADC result */
		*value = ADC_DataSingleGet(ADC0);
		now = RTC->CNT - now;
		return true;
	} else {
		*value = 0;
		return false;
	}
}
