/***************************************************************************//**
 * @file
 * @brief API for sending Data frame type to Sensor
 * @author Telecom Design S.A.
 * @version 1.2.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2013-2014 Telecom Design S.A., http://www.telecomdesign.fr</b>
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

#ifndef __SENSOR_DATA_H
#define __SENSOR_DATA_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

	/***************************************************************************//**
	 * @addtogroup SENSOR_DATA Sensor Data
	 * @{
	 ******************************************************************************/

	/*******************************************************************************
	 ***********************   ENUMERATIONS   **************************************
	 ******************************************************************************/

	/** @addtogroup SENSOR_DATA_ENUMERATIONS Enumerations
	 * @{ */

	/** Data value index */
	typedef enum {
		PHONE_1 = 0,
		PHONE_2 = 1,
		PHONE_3 = 2,
		PHONE_4 = 3
	}
	TD_SENSOR_DATA_PhoneIndex_t;

	/** Data Types*/
	typedef enum {
		DATA_PHONE,
		DATA_GPS,
		DATA_CUSTOM
	} TD_SENSOR_DATA_Types_t;

	/** @} */

	/*******************************************************************************
	 *************************   PROTOTYPES   **************************************
	 ******************************************************************************/

	/** @addtogroup SENSOR_DATA_USER_FUNCTIONS User Functions
	 * @{ */

	bool TD_SENSOR_SendData(TD_SENSOR_DATA_Types_t data_type, uint8_t *data, uint8_t count);
	void TD_SENSOR_EncodeCellPhoneNumber(TD_SENSOR_DATA_PhoneIndex_t index, uint8_t *phone_number, uint8_t *data, uint8_t *len);
	bool TD_SENSOR_SendDataCellPhoneNumber(TD_SENSOR_DATA_PhoneIndex_t index, uint8_t *phone_number);
	bool TD_SENSOR_EncodeLocalVoltage(uint32_t voltage, uint8_t *data, uint8_t *len);
	bool TD_SENSOR_EncodeLocalTemperature(int32_t temperature, uint8_t *data, uint8_t *len);
	void TD_SENSOR_SetDataTransmissionProfile(uint8_t repetition, uint32_t interval);
	bool TD_SENSOR_SetCellPhoneNumber(TD_SENSOR_DATA_PhoneIndex_t index, uint8_t *phone_number);

	/** @} */

	/** @} (end addtogroup SENSOR_DATA) */

#ifdef __cplusplus
}
#endif

#endif // __SENSOR_DATA_H
