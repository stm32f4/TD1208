/***************************************************************************//**
 * @file
 * @brief Temperature/Supply Voltage measure API for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 2.1.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012-2014 Telecom Design S.A., http://www.telecomdesign.fr</b>
 ******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Telecom Design SA has no
 * obligation to support this Software. Telecom Design SA is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Telecom Design SA will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
  ******************************************************************************/

#include <em_chip.h>
#include <em_emu.h>
#include <em_vcmp.h>
#include <em_cmu.h>
#include <em_emu.h>
#include <em_adc.h>

#include "td_core.h"
#include "td_rtc.h"
#include "td_measure.h"

/***************************************************************************//**
 * @addtogroup MEASURE
 * @brief Temperature/Supply Voltage measure API for the TDxxxx RF modules
 * @{
 ******************************************************************************/

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup MEASURE_LOCAL_FUNCTIONS Local Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Convert ADC sample values to 1/10 of degrees Celsius.
 *
 * @note
 *   See section 2.3.4 in the reference manual for details on this calculation.
 *
 * @param adcSample
 *   Raw value from ADC to be converted to Celsius.
 *
 * @return
 *   The temperature in 1/10 of degrees Celsius.
 ******************************************************************************/
static int32_t convertToCelsiusThenth(int32_t adc_result)
{
	// Factory calibration temperature from device information page
	// See table 5.4 in EFM32TG Reference Manual
	int32_t cal_temp_0 = ((DEVINFO->CAL & _DEVINFO_CAL_TEMP_MASK)
						  >> _DEVINFO_CAL_TEMP_SHIFT);

	// Factory calibration value from device information page
	// See table 5.4 in EFM32TG Reference Manual
	int32_t adc0_temp_0_read_1v25 = ((DEVINFO->ADC0CAL2 & _DEVINFO_ADC0CAL2_TEMP1V25_MASK)
									 >> _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);

	/* Temperature measurement is given by the following formula from section
	 * 24.3.4.2 in the EFM32TG Reference Manual:
	 *
	 * t_celsius  = cal_temp_0 - (adc0_temp_0_read_1v25 - adc_result) * vref
	 *              / (4096 * tgrad_adcth)
	 *
	 * Where:
	 *   - vref = 1.25 V
	 *   - tgrad_adcth is -1.92e-3 from table 3.15 in EFM32TG210F32 Datasheet
	 *
	 * Thus:
	 *   coeff = vref / (4096 * tgrad_adcth) = -1.25 / (4096 * -1.92e-3)
	 *         = -0.159
	 *         = 1 / -6.3
	 *
	 * And:
	 * t_celsius = cal_temp_0 - (adc0_temp_0_read_1v25 - adc_result) * coeff
	 *           = cal_temp_0 + (adc0_temp_0_read_1v25 - adc_result) / 6.3
	 *           = cal_temp_0 + ((adc0_temp_0_read_1v25 - adc_result) * 10) / 63
	 *
	 * Eventually:
	 * t_celsius_thenth = round(cal_temp_0 * 10 + ((adc0_temp_0_read_1v25 - adc_result) * 100) / 63)
	 *
	 * We just pre-multiply this computation by 2 and post-divide it by 2, adding 1 for rounding
	 * if intermediate LSB is 1.
	 */
	int32_t t_celsius_tenth = cal_temp_0 * 20 + ((adc0_temp_0_read_1v25 - adc_result) * 200) / 63;
	return (t_celsius_tenth & 1) ? (t_celsius_tenth >> 1) + 1 : t_celsius_tenth >> 1;
}

/** @} */

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup MEASURE_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Accurately measure the Power Supply voltage or the temperature.
 *
 * @param[in] mode
 *   If true, measure the temperature, if false, measure the power supply voltage.
 *
 * @return
 *   The measured temperature is given in 1/10 degrees Celsius, the power supply
 *    voltage is given in mV.
 ******************************************************************************/
int32_t TD_MEASURE_VoltageTemperatureExtended(bool mode)
{
	int32_t setpoint;

	/* Base the ADC configuration on the default setup. */
	ADC_InitSingle_TypeDef single_init = ADC_INITSINGLE_DEFAULT;
	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;

	/* Initialize timebase */
	init.timebase = ADC_TimebaseCalc(0);
	init.prescale = ADC_PrescaleCalc(40000, 0);
	CMU_ClockEnable(cmuClock_ADC0, true);
	ADC_Init(ADC0, &init);

	/* Set input to temperature sensor. Reference must be 1.25V */
	single_init.reference = adcRef1V25;
	single_init.input = mode ? adcSingleInpTemp : adcSingleInpVDDDiv3;
	ADC_InitSingle(ADC0, &single_init);

	// Start one ADC sample
	ADC_Start(ADC0, adcStartSingle);

	// Active wait for ADC to complete
	while ((ADC0->STATUS & ADC_STATUS_SINGLEDV) == 0) {
		;
	}
	setpoint = ADC_DataSingleGet(ADC0);
	if (mode) {
		setpoint = convertToCelsiusThenth(setpoint);
	} else {
		setpoint = (setpoint * 3 * 1250) / 4096;
	}
	CMU_ClockEnable(cmuClock_ADC0, false);
	return(setpoint);
}

/***************************************************************************//**
 * @brief
 *   Measure the Power Supply voltage or the temperature.
 *
 * @param[in] mode
 *   If true, measure the temperature, if false, measure the power supply voltage.
 *
 * @note
 *   This function is limited to positive temperature in degrees Celsius only.
 *   Please consider using TD_MEASURE_VoltageTemperatureExtended instead for
 *   proper handling of negative temperatures and values in 1/10 degrees Celsius.
 *   This function returns voltages in a non-standard format and in tens of mV,
 *   please consider using TD_MEASURE_VoltageTemperatureExtended instead for
 *   values directly in mV.
 *
 * @return
 *   The measured temperature is given in degrees Celsius, and the power supply
 *    voltage in 1/10s of mV plus 2 V if MSB is 0, or plus 3 V if MSB is 1.
 ******************************************************************************/
uint8_t TD_MEASURE_VoltageTemperature(bool mode)
{
	int32_t measure = TD_MEASURE_VoltageTemperatureExtended(mode);
	uint8_t msb;

	if (mode) {
		if (measure < 0) {
			return 0;
		}

		// Divide by 10 with proper rounding
		measure /= 5;
		return (measure & 1) ? (measure >> 1) + 1 : measure >> 1;
	} else {
		if (measure >= 3000) {
			msb = 0x80;
			measure -= 3000;
		} else {
			msb = 0x00;
			measure -= 2000;
		}

		// Divide by 10 with proper rounding
		measure /= 5;
		return (measure & 1) ? (measure >> 1) + 1 + msb : (measure >> 1) + msb;
	}
}

/** @} */

/** @} (end addtogroup MEASURE) */
