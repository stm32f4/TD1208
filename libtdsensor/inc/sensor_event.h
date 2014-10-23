/***************************************************************************//**
 * @file
 * @brief API for sending Event frame type to Sensor
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

#ifndef __SENSOR_EVENT_H
#define __SENSOR_EVENT_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

	/***************************************************************************//**
	 * @addtogroup SENSOR_EVENT Sensor Event
	 * @{
	 ******************************************************************************/

	/*******************************************************************************
	 ***********************   ENUMERATIONS   **************************************
	 ******************************************************************************/

	 /** @addtogroup SENSOR_EVENT_ENUMERATIONS Enumerations
	 * @{ */

	/** Event Type*/
	typedef enum {
		EVENT_BATTERY_LOW,
		EVENT_BATTERY_OK,
		EVENT_CONNECTION_LOST,
		EVENT_CONNECTION_OK,
		EVENT_RSSI_LOW,
		EVENT_RSSI_OK,
		EVENT_TEMP_LOW,
		EVENT_TEMP_HIGH,
		EVENT_TEMP_OK,
		EVENT_BOOT,
		EVENT_SWITCH_ON,
		EVENT_SWITCH_OFF,
		EVENT_ACCELERO_MOVE,
		EVENT_CUSTOM_1 = 0x80,
		EVENT_CUSTOM_2 = 0x81,
		EVENT_CUSTOM_3 = 0x82,
		EVENT_CUSTOM_4 = 0x83,
		EVENT_CUSTOM_5 = 0x84,
		EVENT_CUSTOM_6 = 0x85,
		EVENT_CUSTOM_7 = 0x86,
		EVENT_CUSTOM_8 = 0x87,
		EVENT_CUSTOM_9 = 0x88,
		EVENT_CUSTOM_10 = 0x89,
		EVENT_CUSTOM_11 = 0x8A,
		EVENT_CUSTOM_12 = 0x8B,
		EVENT_CUSTOM_13 = 0x8C,
		EVENT_CUSTOM_14 = 0x8D,
		EVENT_CUSTOM_15 = 0x8E,
		EVENT_CUSTOM_16 = 0x8F,
		EVENT_CUSTOM_17 = 0x90,
		EVENT_CUSTOM_18 = 0x91,
		EVENT_CUSTOM_19 = 0x92,
		EVENT_CUSTOM_20 = 0x93,
		EVENT_CUSTOM_21 = 0x94,
		EVENT_CUSTOM_22 = 0x95,
		EVENT_CUSTOM_23 = 0x96,
		EVENT_CUSTOM_24 = 0x97,
		EVENT_CUSTOM_25 = 0x98,
	}
	TD_SENSOR_EVENT_Types_t;

	/** Boot reason for Event boot */
	typedef enum {
		BOOT_EVENT_POR = 0,
		BOOT_EVENT_WDOG = 1,
		BOOT_EVENT_BROR = 2
	} TD_SENSOR_EVENT_BootReason_t;

	/** @} */

	/*******************************************************************************
	 *************************   PROTOTYPES   **************************************
	 ******************************************************************************/

	/** @addtogroup SENSOR_EVENT_USER_FUNCTIONS User Functions
	 * @{ */

	bool TD_SENSOR_SendEvent(TD_SENSOR_EVENT_Types_t event, uint8_t *data, uint8_t len);
	bool TD_SENSOR_SendEventBattery(bool state, uint8_t battery_level);
	bool TD_SENSOR_SendEventRSSI(bool state, uint8_t EntryID);
	bool TD_SENSOR_SendEventConnection(bool state, uint8_t entryID);
	bool TD_SENSOR_SendEventTemperature(uint8_t state);
	bool TD_SENSOR_SendEventBoot(void);
	bool TD_SENSOR_SendEventBootExt(uint8_t cause, uint8_t custom_cause, uint8_t *data, uint8_t len);
	bool TD_SENSOR_SendEventSwitch(uint8_t port, uint8_t bit, bool state);
	void TD_SENSOR_SetEventTransmissionProfile(uint8_t repetition, uint32_t interval);

	/** @} */

	/** @} (end addtogroup SENSOR_EVENT) */

#ifdef __cplusplus
}
#endif

#endif // __SENSOR_EVENT_H
