/***************************************************************************//**
 * @file
 * @brief Temperature/Supply Voltage measure API for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 2.0.2
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

#ifndef __TD_MEASURE_H
#define __TD_MEASURE_H

#include <stdint.h>
#include <stdbool.h>

#include <efm32.h>

#ifdef __cplusplus
extern "C" {
#endif

	/***************************************************************************//**
	 * @addtogroup MEASURE
	 * @brief Temperature/Supply Voltage measure API for the TDxxxx RF modules
	 * @{
	 ******************************************************************************/

	/*******************************************************************************
	 *************************   PROTOTYPES   **************************************
	 ******************************************************************************/

	/** @addtogroup MEASURE_USER_FUNCTIONS User Functions
	 * @{ */

	uint8_t TD_MEASURE_VoltageTemperature(bool mode);
	int32_t TD_MEASURE_VoltageTemperatureExtended(bool mode);

	/** @} */

	/***************************************************************************//**
	 * @brief
	 *   Measure the TDxxxx RF module temperature.
	 *
	 * @note
	 *   This function is limited to positive temperature in degrees Celsius only.
	 *   Please consider using TD_MEASURE_TemperatureExtended instead for proper
	 *   handling of negative temperatures and values in 1/10 degrees Celsius.
	 *
	 * @return
	 *   The measured temperature in degrees Celsius.
	 ******************************************************************************/
	static __INLINE uint8_t TD_MEASURE_Temperature(void)
	{
		return TD_MEASURE_VoltageTemperature(true);
	}

	/***************************************************************************//**
	 * @brief
	 *   Measure the TDxxxx RF module temperature.
	 *
	 * @return
	 *   The measured temperature in 1/10 of degrees Celsius.
	 ******************************************************************************/
	static __INLINE int32_t TD_MEASURE_TemperatureExtended(void)
	{
		return TD_MEASURE_VoltageTemperatureExtended(true);
	}

	/***************************************************************************//**
	 * @brief
	 *   Measure the TDxxxx RF module power supply voltage.
	 *
	 * @note
	 *   This function returns voltages in a non-standard format and in tens of mV,
	 *   please consider using TD_MEASURE_VoltageExtended instead for values
	 *   directly in mV.
	 *
	 * @return
	 *   The measured power supply voltage in 10s of mV plus 2 V if MSB is 0,
	 *   plus 3 V if MSB is 1.
	 ******************************************************************************/
	static __INLINE uint8_t TD_MEASURE_Voltage(void)
	{
		return TD_MEASURE_VoltageTemperature(false);
	}

	/***************************************************************************//**
	 * @brief
	 *   Measure the TDxxxx RF module power supply voltage.
	 *
	 * @return
	 *   The measured power supply voltage in 10th of mV
	 ******************************************************************************/
	static __INLINE int32_t TD_MEASURE_VoltageExtended(void)
	{
		return TD_MEASURE_VoltageTemperatureExtended(false);
	}

	/** @} */

	/** @} (end addtogroup MEASURE) */

#ifdef __cplusplus
}
#endif

#endif // __TD_MEASURE_H
