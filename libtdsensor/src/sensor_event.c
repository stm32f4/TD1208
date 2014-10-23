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

#include <stdbool.h>
#include <stdint.h>

#include "sensor_send.h"
#include "sensor_data.h"
#include "sensor_event.h"
#include "td_sensor_utils.h"

/***************************************************************************//**
 * @addtogroup SENSOR_EVENT Sensor Event
 * @brief Sensor API for sending an Event frame
 *
 * @details
 *
 *	Event Type values are:
 *
 *	- BATTERY_LOW & BATTERY_OK
 *
 *	Indicate that the device is running out of Battery. The BATTERY_LOW message is sent whenever the battery voltage goes below a specified threshold.
 *	The BATTERY_OK event is being sent after a BATTERY_LOW event has been emitted when batteries are changed or recharged. The Data field is used for
 *	BATTERY_OK event and indicated battery level.
 *
 *	- CONNECTION_LOST & CONNECTION_OK
 *
 *	Indicate whether the connection with a device has been lost or not. The CONNECTION_LOST event indicate that the device didn't send a keep-alive on time.
 *	CONNECTION_OK means the connection with the device has been recovered. The Data field is used for both messages and indicates which device is concerned.
 *
 *	- RSSI_LOW & RSSI_OK
 *
 *	Indicate whether the RSSI level is low or not. The RSSI level from all received message is being compared to a threshold and a RSSI_LOW event is
 *	emitted when the RSSI level is below it. When RSSI level comes back above the RSSI threshold, the RSSI_OK event is emitted. The Data field is used
 *	for both messages and indicates which device is concerned.
 *
 *	- TEMP_LOW & TEMP_HIGH & TEMP_OK
 *
 *	The on-board temperature sensor can provides accurate temperature measurement. A TEMP_LOW and TEMP_HIGH event can be emitted whenever the temperature
 *	falls below or rise above the corresponding threshold. When going back to normal temperature value a TEMP_OK event is emitted.
 *
 *	- BOOT
 *
 *	This message should be sent when the device is booting, ie on Init.
 *
 *	- SWITCH_ON & SWITCH_OFF
 *
 *	Indicate whether a switch has been turned ON or OFF. The Data field is used
 *	for both messages and contains port and bit information for the switch.
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   DEFINES   *****************************************
 ******************************************************************************/

/** @addtogroup SENSOR_EVENT_DEFINES Defines
 * @{ */

#define EVENT_PAYLOAD_SIZE		1	///< Event payload size in bytes
#define EVENT_DEFAULT_REPETITON	0 	///< Default retransmission repetition
#define EVENT_DEFAULT_INTERVAL	0	///< Default retransmission interval in seconds

/** @} */

/*******************************************************************************
 *************************   TYPEDEFS   *****************************************
 ******************************************************************************/

 /** @addtogroup SENSOR_EVENT_TYPEDEFS Typedefs
 * @{ */

/** Event Frame Structure */
typedef struct {
	TD_SENSOR_EVENT_Types_t type : 8;	///< The Sensor event type
	uint8_t data[9];					///< The data payload
} __PACKED TD_SENSOR_EVENT_Frame_t;

/** @} */

/*******************************************************************************
 *************************   PRIVATE VARIABLES   ****************************************
 ******************************************************************************/

/** @addtogroup SENSOR_EVENT_LOCAL_VARIABLES Local Variables
 * @{ */

/** Transmission profile */
static TD_SENSOR_TransmitProfile_t Profile = {EVENT_DEFAULT_REPETITON, EVENT_DEFAULT_INTERVAL};

/** Redundancy counter for Sensor events */
static uint8_t Stamp = -1;

/** @} */

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup SENSOR_EVENT_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Send an event frame to Sensor.
 *
 * @param[in] event
 *   Event Type.
 *
 * @param[in] data
 *   Pointer to the buffer containing the data to be sent.
 *
 * @param[in] length
 *   Length of the data to be sent in bytes.
 *
 * @return
 *   Returs true if the data has been sent over the SIGFOX network, false otherwise.
 ******************************************************************************/
bool TD_SENSOR_SendEvent(TD_SENSOR_EVENT_Types_t event, uint8_t *data, uint8_t length)
{
	int i;
	TD_SENSOR_EVENT_Frame_t frame;
	frame.type = event;

	for (i = 0; i < length; i++) {
		frame.data[i] = data[i];
	}
	Stamp = (Stamp & 0x07) + 1;
	return TD_SENSOR_Send(&Profile, SRV_FRM_EVENT, Stamp, (uint8_t *) &frame, length + 1);
}

/***************************************************************************//**
 * @brief
 *   Send a battery event to Sensor.
 *
 *  @param[in] state
 *  True to send EVENT_BATTERY_OK, false to send EVENT_BATTERY_LOW.
 *
 *  @param[in] battery_level
 *   If battery event is EVENT_BATTERY_OK, the battery level must also be provided.
 *
 * @return
 *   Returns true if the data has been sent (i.e. the gateway has acknowledged
 *   the request), false if the acknowledge from the gateway was never received.
 ******************************************************************************/
bool TD_SENSOR_SendEventBattery(bool state, uint8_t battery_level)
{
	if (state) {
		return TD_SENSOR_SendEvent(EVENT_BATTERY_OK, &battery_level, 1);
	} else {
		return TD_SENSOR_SendEvent(EVENT_BATTERY_LOW, 0, 0);
	}
}

/***************************************************************************//**
 * @brief
 *   Send an RSSI Low event to Sensor.
 *
 *  @param[in] state
 *   True to send EVENT_RSSI_OK, false to send EVENT_RSSI_LOW.
 *
 *  @param[in] EntryID
 *   EntryID of which the RSSI is low.
 *
 * @return
 *   Always true.
 ******************************************************************************/
bool TD_SENSOR_SendEventRSSI(bool state, uint8_t EntryID)
{
	if (state) {
		return TD_SENSOR_SendEvent(EVENT_RSSI_OK, &EntryID, 1);
	} else {
		return TD_SENSOR_SendEvent(EVENT_RSSI_LOW, &EntryID, 1);
	}
}

/***************************************************************************//**
 * @brief
 *   Send a Connection event to Sensor.
 *
 *  @param[in] state
 *  True to send EVENT_CONNECTION_OK, false to send EVENT_CONNECTION_LOST.
 *
 *  @param[in] EntryID
 *   EntryID of which the connection has been lost.
 *
 * @return
 *   Always true.
 ******************************************************************************/
bool TD_SENSOR_SendEventConnection(bool state, uint8_t EntryID)
{
	if (state) {
		return TD_SENSOR_SendEvent(EVENT_CONNECTION_OK, &EntryID, 1);
	} else {
		return TD_SENSOR_SendEvent(EVENT_CONNECTION_LOST, &EntryID, 1);
	}
}

/***************************************************************************//**
 * @brief
 *   Send a temperature event to Sensor.
 *
 *  @param[in] state
 *  0 is below min , 1 is ok, 2 is above max
 *
 * @return
 *   True if the data has been sent (ie. the gateway has acknowledged the request)
 *   False if the ack from the gateway was never received.
 ******************************************************************************/
bool TD_SENSOR_SendEventTemperature(uint8_t state)
{
	switch (state) {
	case 0:
		return TD_SENSOR_SendEvent(EVENT_TEMP_LOW, 0, 0);
		break;

	case 1:
		return TD_SENSOR_SendEvent(EVENT_TEMP_OK, 0, 0);
		break;

	case 2:
		return TD_SENSOR_SendEvent(EVENT_TEMP_HIGH, 0, 0);
		break;
	}
	return false;
}

/***************************************************************************//**
 * @brief
 *   Send a boot event to Sensor.
 *
 * @return
 *   Returns true if the data has been sent (i.e. the gateway has acknowledged the request)
 *   False if the ack from the gateway was never received.
 ******************************************************************************/
bool TD_SENSOR_SendEventBoot(void)
{
	return TD_SENSOR_SendEvent(EVENT_BOOT, 0, 0);
}

/***************************************************************************//**
 * @brief
 *   Send the Boot event to Sensor
 *
 * @return
 *   True if the data has been sent (ie. the gateway has acknowledged the request)
 *   False if the ack from the gateway was never received.
 ******************************************************************************/
bool TD_SENSOR_SendEventBootExt(uint8_t cause, uint8_t custom_cause, uint8_t *data, uint8_t length)
{
	uint8_t msg[9];
	uint8_t total_length = 16;
	uint8_t length_bytes = 0;

	msg[0] = cause;
	msg[1] = custom_cause;
	if (length <= 56) {
		TD_SENSOR_UTILS_BitConcat(msg, &total_length, data, length);
	}
	length_bytes = total_length / 8;
	if (total_length % 8 > 0) {
		length_bytes++;
	}
	return TD_SENSOR_SendEvent(EVENT_BOOT, msg, length_bytes);
}

/***************************************************************************//**
 * @brief
 *   Send a switch event to Sensor.
 *
 * @param[in] port
 * 	Switch port.
 *
 * @param[in] bit
 * 	Switch bit.
 *
 * @param[in] state
 * 	Switch state.
 *
 * @return
 *   True if the data has been sent (ie. the gateway has acknowledged the request)
 *   False if the ack from the gateway was never received.
 ******************************************************************************/
bool TD_SENSOR_SendEventSwitch(uint8_t port, uint8_t bit, bool state)
{
	uint8_t data = 0;
	data = ((port & 0xF) << 4) | (bit & 0xF);
	if (!state) {
		return TD_SENSOR_SendEvent(EVENT_SWITCH_ON, &data, 1);
	} else {
		return TD_SENSOR_SendEvent(EVENT_SWITCH_OFF, &data, 1);
	}
}

/***************************************************************************//**
 * @brief
 *   Set a transmission profile for an event frame type.
 *
 * @param[in] repetition
 *	Number of repetitions.
 *
 * @param[in] interval
 *	Interval between two repetitions in seconds.
 ******************************************************************************/
void TD_SENSOR_SetEventTransmissionProfile(uint8_t repetition, uint32_t interval)
{
	Profile.repetition = repetition;
	Profile.interval = interval;
}

/** @} */

/** @} (end addtogroup SENSOR_EVENT) */
