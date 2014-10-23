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

#include <stdbool.h>
#include <stdint.h>

#include <td_utils.h>

#include "sensor_send.h"
#include "sensor_data.h"

/***************************************************************************//**
 * @addtogroup SENSOR_DATA Sensor Data
 * @brief Sensor API for sending Data frame
 *
 * @details
 * 	 Data frames allows you to send data to Sensor. Right now, only phone numbers
 * 	 are handled.
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   DEFINES   *****************************************
 ******************************************************************************/

/** @addtogroup SENSOR_DATA_DEFINES Defines
 * @{ */

#define DATA_PAYLOAD_SIZE		9	///< Payload size to be able to transport a phone number
#define MAX_PHONE_ZEROS			4	///< Maximum number of leading zeros in a phone number

#define DATA_DEFAULT_REPETITON	0	///< Default retransmission repetitions
#define DATA_DEFAULT_INTERVAL	0	///< Default retransmission interval in seconds

/** @} */

/*******************************************************************************
 *************************   TYPEDEFS   *****************************************
 ******************************************************************************/

/** @addtogroup SENSOR_DATA_TYPEDEFS Typedefs
 * @{ */

/** Data Frame Structure */
typedef struct {
	TD_SENSOR_DATA_Types_t type;	///< The Sensor data type
	uint8_t data[9];				///< The data payload
} TD_SENSOR_DATA_Frame_t;

/** @} */

/*******************************************************************************
 *************************   PRIVATE VARIABLES   ****************************************
 ******************************************************************************/

/** @addtogroup SENSOR_DATA_LOCAL_VARIABLES Local Variables
 * @{ */

/** Transmission profile */
static TD_SENSOR_TransmitProfile_t Profile = {DATA_DEFAULT_REPETITON, DATA_DEFAULT_INTERVAL};

/** Redundancy counter for Sensor data */
static uint8_t Stamp = -1;

/** @} */

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup SENSOR_DATA_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Send a DATA frame to Sensor.
 *
 * @param[in] data_type
 *   Type of Data being sent. Used by Sensor for decoding. Refer to the
 *   SensorDataType enumeration for allowed values.
 *
 * @param[in] data
 *   Pointer to the data to be sent. Maximum allowed length for data is 16 bytes.
 *
 * @param[in] count
 *   The length in bytes of the data to be sent.
 *
 * @return
 *   True if the data has been sent over the SIGFOX Network, false otherwise.
 ******************************************************************************/
bool TD_SENSOR_SendData(TD_SENSOR_DATA_Types_t data_type, uint8_t *data, uint8_t count)
{
	TD_SENSOR_DATA_Frame_t frame;
	unsigned int i;
	frame.type = data_type;

	for (i = 0; i < count; i++) {
		frame.data[i] = data[i];
	}
	Stamp = (Stamp & 0x07) + 1;
	return TD_SENSOR_Send(&Profile, SRV_FRM_DATA, Stamp, (uint8_t *) &frame, count + 1);
}

/***************************************************************************//**
 * @brief
 *   Register a new cell phone number for sending SMS on Sensor.
 *
 * @param[in] index
 *	Phone number index. Up to 4 cell phone number can be registered.
 *
 * @param[in] phone_number
 *   Pointer to a buffer containing the phone number. ie 0601020304.
 *
 * @return
 *   True if the data has been sent (ie. the gateway has acknowledged the request)
 *   False if the acknowledgment from the gateway was never received.
 ******************************************************************************/
bool TD_SENSOR_SendDataCellPhoneNumber(TD_SENSOR_DATA_PhoneIndex_t index, uint8_t *phone_number)
{
	uint8_t msg[7];
	uint8_t count;

	TD_SENSOR_EncodeCellPhoneNumber(index, phone_number, msg, &count);
	return TD_SENSOR_SendData(DATA_PHONE, msg, count);
}

/***************************************************************************//**
 * @brief
 *   Encode a cell phone number for sending SMS on Sensor.
 *
 * @param[in] index
 *   Phone number index. Up to 4 cell phone number can be registered.
 *
 * @param[in] phone_number
 *   Pointer to a buffer containing the phone number. i.e. 0601020304.
 *
 * @param[in] data
 *   Pointer to the buffer that will receive the encoded phone number.
 *
 * @param[in] length
 *   Pointer to a byte that will receive the encoded phone number length.
 ******************************************************************************/
void TD_SENSOR_EncodeCellPhoneNumber(TD_SENSOR_DATA_PhoneIndex_t index, uint8_t *phone_number, uint8_t *data, uint8_t *length)
{
	uint64_t encoded;
	uint8_t zeros = 0;

	*length = 0;
	while (phone_number[zeros] == '0') {
		zeros++;
		if (zeros > MAX_PHONE_ZEROS) {
			return;
		}
	}
	encoded = atoll((char *) &phone_number[zeros]);
	data[0] = (zeros & 0x03) << 6 | (index & 0x3F);
	while (encoded != 0 && (*length) < 6) {

		// LSB first
		data[1 + *length] = encoded & 0xFF;
		encoded = encoded >> 8;
		(*length)++;
	}
	(*length)++;
}

/***************************************************************************//**
 * @brief
 *   Encode a voltage for Sensor.
 *
 * @param[in] voltage
 *   The voltage value in mV, range is from 1.85 V to 4.40 V.
 *
 * @param[in] data
 *   Pointer to the buffer that will receive the encoded voltage in 10 mV resolution.
 *
 * @param[in] length
 *   Pointer to a byte that will receive the encoded voltage length.
 ******************************************************************************/
bool TD_SENSOR_EncodeLocalVoltage(uint32_t voltage, uint8_t *data, uint8_t *length)
{
	voltage /= 10;
	if (voltage >= 185 && voltage <= 440) {
		voltage -= 185;
		data[0] = voltage;
		*length = 8;
		return true;
	} else {
		return false;
	}
}

/***************************************************************************//**
 * @brief
 *   Encode a temperature for Sensor.
 *
 * @param[in] temperature
 *   The temperature value in 1/10°C, range is from -30°C to 97.5°C.
 *
 * @param[in] data
 *   Pointer to the buffer that will receive the encoded temperature in 0.5°C resolution.
 *
 * @param[in] length
 *   Pointer to a byte that will receive the encoded temperature length.
 ******************************************************************************/
//temperature in 10th of degrees range -30/97.5 degrees step 0.5
bool TD_SENSOR_EncodeLocalTemperature(int32_t temperature, uint8_t *data, uint8_t *length)
{
	// Round to half degrees
	int modulo = temperature % 5;

	switch (modulo) {
	case 1:
	case 2:
		temperature -= modulo;
		break;

	case 3:
	case 4:
		temperature += modulo;
		break;
	}
	if (temperature >= -300 && temperature <= 975) {
		data[0] = (temperature + 300) / 5;
		*length = 8;
		return true;
	}
	return false;
}

/*****************************************************************************
 * @brief
 *   Set a transmission profile for data frame type.
 *
 * @param[in] repetition
 *	Number of repetition.
 *
 * @param[in] interval
 *	Interval between two repetitions in seconds.
 ******************************************************************************/
void TD_SENSOR_SetDataTransmissionProfile(uint8_t repetition, uint32_t interval)
{
	Profile.repetition = repetition;
	Profile.interval = interval;
}

/***************************************************************************//**
 * @brief
 *   Register a new cell phone number for sending SMS on Sensor.
 *
 * @param[in] index
 *	Phone number index. Up to 4 cell phone number can be registered.
 *
 * @param[in] phone_number
 *   Pointer to a buffer containing the phone number. ie 0601020304.
 *
 * @return
 *   True if the data has been sent (ie. the gateway has acknowledged the request)
 *   False if the acknowledgment from the gateway was never received.
 *
 * @note
 *   This function is depreciated, use TD_SENSOR_SendDataCellPhoneNumber instead.
 ******************************************************************************/
bool TD_SENSOR_SetCellPhoneNumber(TD_SENSOR_DATA_PhoneIndex_t index, uint8_t *phone_number)
{
	return TD_SENSOR_SendDataCellPhoneNumber(index, phone_number);
}

/** @} */

/** @} (end addtogroup SENSOR_DATA) */
