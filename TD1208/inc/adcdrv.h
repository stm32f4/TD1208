/*
 * adcdrv.h
 *
 *  Created on: 2 juin 2015
 *      Author: thierry
 */

#ifndef __ADCDRV_H_
#define __ADCDRV_H_

#include <stdbool.h>
#include <stdint.h>
#include <em_adc.h>

void TD_USER_InitAdc(ADC_SingleInput_TypeDef channel);
bool TD_USER_ReadAdc(uint32_t *value);

#endif /* __ADCDRV_H_ */
