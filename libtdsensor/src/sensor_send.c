/***************************************************************************//**
 * @file
 * @brief API for sending frames to Sensor
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

#include "td_sensor.h"
#include "td_sensor_device.h"
#include "td_sensor_transmitter.h"
#include "sensor_send.h"
#include "sensor_raw.h"
#include "sensor_event.h"
#include "sensor_data.h"
#include "sensor_register.h"
#include "sensor_service.h"
#include "sensor_keepalive.h"

/***************************************************************************//**
 * @addtogroup SENSOR_SEND Sensor Send
 * @brief Sensor protocol Send function
 *
 * @details
 *	 Sensor frame types are:
 *
 *		- SRV_FRM_EVENT
 *
 *		This frame type allows to send an event information to Sensor. An event
 *		information is usually completed by a status information derived from a
 *		bias value, i.e. Battery Low, Temperature High, Connection Lost, etc.
 *
 *		- SRV_FRM_DATA
 *
 *		This frame type is used to transmit data to Sensor. This can any kind of
 *		data like a phone number or a measurement value.
 *
 *		- SRV_FRM_REGISTER
 *
 *		This frame type allows registering a new device on Sensor so that any
 *		other kind of frame type can be processed properly.
 *
 *		- SRV_FRM_KEEPALIVE
 *
 *		This periodic frame lets Sensor know the device is still alive.
 *
 *		- SRV_FRM_RAW
 *
 *		This frame type is provided for custom user purposes. This frame allows
 *		a 10 bytes user data payload that will not be processed by Sensor,
 *		unless configured to do so.
 *
 *		- SRV_FRM_SERVICE
 *
 *		This frame type triggers a service, like sending an SMS or a Tweet.
 *		Remember that to send a SMS or a tweet you must previously register
 *		a phone number or a twitter account by sending the corresponding
 *		SRV_FRM_DATA frame or by using the Sensor Web Interface.
 * @{
 ******************************************************************************/

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup SENSOR_SEND_GLOBAL_FUNCTIONS Global Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Send a Sensor Frame according to Sensor Protocol. Depending on module type,
 *   the frame can be forwarded or not to a gateway.
 *
 * @param[in] profile
 * 	Retransmission profile.
 *
 * @param[in] frame_type
 * 	Sensor frame type.
 *
 * @param[in] stamp
 * 	Redundancy counter for the given frame type.
 *
 * @param[in] payload
 * 	Pointer to the buffer containing the frame payload data.
 *
 * @param[in] count
 * 	Number of bytes of frame payload data.
 *
 * @return
 *   Returns true if the data has been sent over the SIGFOX network, false
 *   otherwise.
 ******************************************************************************/
bool TD_SENSOR_Send(TD_SENSOR_TransmitProfile_t *profile, TD_SENSOR_FrameType_t frame_type, uint8_t stamp, uint8_t *payload, uint8_t count)
{
	TD_SENSOR_Frame_t frame;
	TD_SENSOR_LAN_AckCode_t code;
	unsigned int i = 0;

	// Think smaller!
	if (count > 10) {
		return false;
	}
	frame.header.retry = 0;
	frame.header.stamp = stamp & 0x7;
	frame.header.cpt = 0;
	frame.header.entry_id = 0;
	frame.header.type = frame_type;
	for (i = 0; i < count; i++) {
		frame.payload[i] = payload[i];
	}
	if (TD_SENSOR_GetModuleType() != SENSOR_DEVICE ||
		(TD_SENSOR_GetModuleType() == SENSOR_DEVICE && TD_SENSOR_DEVICE_GetTxSkipLan())) {
		return TD_SENSOR_TRANSMITTER_SendSigfox(&frame, count + sizeof (TD_SENSOR_FrameHeader_t), 0, profile);
	} else {

		// Try to send via LAN in synchronous or asynchronous mode
		if (TD_SENSOR_DEVICE_IsAsynchronousForward()) {
			code = TD_SENSOR_DEVICE_ForwardAsynch((uint8_t *) &frame,
				count + sizeof (TD_SENSOR_FrameHeader_t),
				profile->repetition,
				profile->interval);
			if (code == ACK_OK || code == SENSOR_LAN_QUEUED) {
				return true;
			} else {
				return false;
			}
		} else {
			code = TD_SENSOR_DEVICE_Forward((uint8_t *) &frame,
				count + sizeof (TD_SENSOR_FrameHeader_t),
				profile->repetition,
				profile->interval);
			if (code == ACK_OK) {
				return true;
			} else {
				return false;
			}
		}
	}
}

/***************************************************************************//**
 * @brief
 *   Set retransmission profile for each frame type.
 *
 * @param[in] type
 * 	The Sensor frame type.
 *
 * @param[in] repetition
 * 	Number of times the frame should be retransmitted.
 *
 * @param[in] interval
 * 	Interval in seconds at which the retransmission should occur. Maximum 600 s.
 ******************************************************************************/
void TD_SENSOR_SetTransmissionProfile(TD_SENSOR_FrameType_t type, uint8_t repetition, uint16_t interval)
{
	switch (type) {
	case SRV_FRM_EVENT:
		TD_SENSOR_SetEventTransmissionProfile(repetition, interval);
		break;

	case SRV_FRM_DATA:
		TD_SENSOR_SetDataTransmissionProfile(repetition, interval);
		break;

	case SRV_FRM_REGISTER:
		TD_SENSOR_SetRegisterTransmissionProfile(repetition, interval);
		break;

	case SRV_FRM_KEEPALIVE:
		TD_SENSOR_SetKeepAliveTransmissionProfile(repetition, interval);
		break;

	case SRV_FRM_RAW:
		TD_SENSOR_SetRawTransmissionProfile(repetition, interval);
		break;

	case SRV_FRM_SERVICE:
		TD_SENSOR_SetServiceTransmissionProfile(repetition, interval);
		break;

	default:
		break;
	}
}

/** @} */

/** @} (end addtogroup SENSOR_SEND) */
