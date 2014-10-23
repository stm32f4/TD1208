/***************************************************************************//**
 * @file
 * @brief API for sending Data frame type to Sensor
 * @author Telecom Design S.A.
 * @version 1.0.1
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

#include <td_core.h>
#include <td_printf.h>
#include <td_utils.h>
#include <td_geoloc.h>
#include <td_sensor_utils.h>

#include "sensor_send.h"
#include "sensor_data.h"
#include "sensor_data_geoloc.h"

/**************************************************************************//**
 * @addtogroup SENSOR_DATA_GEOLOC Geoloc Sensor Encoder
 * @brief Geolocation Sensor data.
 * @{
 ******************************************************************************/

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup SENSOR_DATA_GEOLOC_FUNCTIONS Private Functions
 * @{ */

/***************************************************************************//**
 * @brief
 * 	Long to byte array. Convert an unsigned long to array.
 *
 *   0x12345678-> 0x12 0x34 0x56 0x78
 *
 * @param[in] value
 *  Value to be encoded
 *
 * @param[in] data
 *  Pointer to the array to be filled.
 *
 ******************************************************************************/
static void TD_SENSOR_ltoba(uint32_t value, uint8_t *data)
{
	int i;

	for (i = 3; i >= 0; i--) {
		data[i] = value & 0xFF;
		value >>= 8;
	}
}

/***************************************************************************//**
 * @brief
 *   Convert uint64_t to char
 *
 * @param[in] data
 *   Pointer to the frame data buffer to fill in.
 *
 * @param[in] position
 *   Value to be converted
 ******************************************************************************/
static void TD_SENSOR_store_pos(uint8_t *data, uint64_t position)
{
	int8_t i;

	for (i = 5; i >= 0; i--) {
		data[i] = position & 0xFF;
		position >>= 8;
	}
}

/***************************************************************************//**
 * @brief
 *   Encode a Position/Quality Sensor frame.
 *
 * @param[in] fix
 *   Pointer to the last GPS position.
 *
 * @param[in] data
 *   Pointer to the frame data buffer to fill in.
 *
 * @param[in] len
 *   Pointer the the frame length in bits that will be filled in.
 ******************************************************************************/
static void TD_SENSOR_EncodePositionQuality(TD_GEOLOC_Fix_t *fix, uint8_t *data, uint8_t *len)
{
	uint64_t position;
	uint16_t altitude;
	uint8_t sv;
	uint8_t hdop;
	uint32_t tmp;
	bool sign_latitude = false;
	bool sign_longitude = false;
	bool sign_altitude = false;

	if (fix->position.longitude < 0) {
		sign_longitude = true;
		tmp = -fix->position.longitude;
	} else {
		sign_longitude = false;
		tmp = fix->position.longitude;
	}
	position = tmp / 100;
	position = position * 10000000;
	if (fix->position.latitude < 0) {
		sign_latitude = true;
		tmp = -fix->position.latitude;
	} else {
		sign_latitude = false;
		tmp = fix->position.latitude;
	}
	position += tmp / 100;
	if (fix->position.altitude < 0) {
		sign_altitude = true;
		altitude = (-fix->position.altitude) >> 1;
	} else {
		sign_altitude = false;
		altitude = (fix->position.altitude) >> 1;
	}
	if (altitude > 4095) {
		altitude = 4095;
	}
	if (fix->quality.sat <= 7) {
		sv = fix->quality.sat;
	} else {
		sv = 7;
	}
	if (fix->quality.hdop <= 200) {
		hdop = 0;
	} else if (fix->quality.hdop <= 500) {
		hdop = 1;
	} else if (fix->quality.hdop <= 1000) {
		hdop = 2;
	} else {
		hdop = 3;
	}

#if 0
	uint64_t long_lat	: 48;
	uint16_t altitude	: 12;
	bool long_sign		: 1; //false is positive
	bool lat_sign		: 1; //false is positive
	bool alt_sign		: 1; //false is positive
	uint8_t sv			: 3;
	uint8_t hdop		: 2;
#endif

	if (fix->position.longitude == 0x7FFFFFFF || fix->position.latitude == 0x7FFFFFFF) {
		memset(&data[0], 0xFF, 7);
		data[7] = 0xFE;
		data[7] |= sv >> 2;
		data[8] = (sv & 0x03) << 6;
		data[8] |= (hdop & 0x03) << 4;
	} else {
		TD_SENSOR_store_pos(data, position);
		data[6] = altitude >> 4;
		data[7] = (altitude & 0x0F) << 4;
		data[7] |= (sign_longitude << 3) | (sign_latitude << 2) | (sign_altitude << 1);
		data[7] |= sv >> 2;
		data[8] = (sv & 0x03) << 6;
		data[8] |= (hdop & 0x03) << 4;
	}
	*len = 68;
}

/** @} */

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup SENSOR_DATA_GEOLOC_GLOBAL_FUNCTIONS Global Functions
 * @{ */

/** @} */


/** @addtogroup SENSOR_DATA_GEOLOC_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Encode a Position Sensor frame.
 *
 * @param[in] fix
 *   Pointer to the last GPS position.
 *
 * @param[in] data
 *   Pointer to the frame data buffer to fill in.
 *
 * @param[in] length
 *   Number of bits on which the position should be encoded. Len is related to accuracy according to the
 *   following table (in meters):
 *
 * 32 (4 bytes): latitude -> 152.59 longitude -> 305.48
 * 33: latitude -> 152.59 longitude -> 152.59
 * 34: latitude ->  76.26 longitude -> 152.59
 * 35: latitude -> 76.26 longitude -> 76.26
 * 36: latitude -> 38.12 longitude -> 76.26
 * 37: latitude -> 38.12 longitude -> 38.12
 * 38: latitude -> 19.06 longitude -> 38.12
 * 39: latitude -> 19.06 longitude -> 19.06
 * 40 (5 bytes): latitude -> 9.52 longitude -> 19.06
 * 41: latitude -> 9.52 longitude -> 9.52
 * 42: latitude -> 4.76 longitude -> 9.52
 * 43: latitude -> 4.76 longitude -> 4.76
 * 44: latitude -> 2.39 longitude -> 4.76
 * 45: latitude -> 2.39 longitude -> 2.39
 * 46: latitude -> 1.19 longitude -> 2.39
 * 47: latitude -> 1.19 longitude -> 1.19
 * 48 (6 bytes): latitude -> 0.60 longitude -> 1.19
 *
 * @param[in] nofix
 *   No fix is a value which can be transmitted in case fix failed.
 *   Max value is 4095.
 *
 * @return
 *   Number of bits on which data is encoded. Should be = length or -1 if failed.
 ******************************************************************************/
int8_t TD_SENSOR_EncodePositionXY(TD_GEOLOC_Fix_t *fix, uint8_t *data, uint8_t length, uint16_t nofix)
{
	uint8_t lat_bits, long_bits;
	uint8_t tmp[4];

	// First is 14 bits
	static uint16_t TD_SENSOR_DATA_GEOLOC_BitsToDiv[] = { 55043, 27494, 13740, 6869, 3434, 1717, 859, 430, 215, 108, 54 };

	// Converted latitude and longitude
	uint32_t clat,clong;

	// Decimal lat and long
	uint32_t latitude, longitude;
	uint8_t lat_deg, long_deg;

	// Make sure param are allowed
	if ( (length < 32 && length > 48) || (fix == 0) || (data == 0)) {
		return -1;
	}

	// Find bit length for each value. try to keep longitude longer
	// Don't take signs into account
	length -= 2;

	// if odd
	if (length & 1) {
		lat_bits =  (length - 1) / 2;
		long_bits = (length + 1) / 2;
	} else {
		lat_bits = long_bits = length / 2;
	}

	// Use remaining values for no fix indicator
	if (fix->position.longitude == 0x7FFFFFFF || fix->position.latitude == 0x7FFFFFFF) {

		// No sign, can be used for carry value as well
		data[0] = 0;
		if (nofix > 4095) {
			data[0] = 0xC0;
			clong = 31;
			clat = 31;
		} else {
			if (nofix > 2047) {
				nofix -= 2047;
				data[0] |= 1 << 7;
			}

			if (nofix > 1023) {
				nofix -= 1023;
				data[0] |= 1 << 6;
			}
			clong = nofix / 32;
			clat = nofix - (clong * 32);
			clong = GET_BITFIELD_MASK(long_bits) - clong;
			clat = GET_BITFIELD_MASK(lat_bits) - clat;
		}
	} else {

		// Get rid of sign
		if (fix->position.longitude < 0) {
			data[0] = 1 << 7;
			longitude = -fix->position.longitude;
		} else {
			longitude = fix->position.longitude;
		}
		if (fix->position.latitude < 0) {
			data[0] |= 1 << 6;
			latitude = -fix->position.latitude;
		} else {
			latitude = fix->position.latitude;
		}

		// Get degrees
		lat_deg = latitude / 10000000;
		long_deg = longitude / 10000000;

		// Convert to decimal degrees
		latitude = lat_deg * 10000000 + (((latitude - (lat_deg * 10000000)) * 100) / 60);
		longitude = long_deg * 10000000 + (((longitude - (long_deg * 10000000)) * 100) / 60);
		clat = latitude / TD_SENSOR_DATA_GEOLOC_BitsToDiv[lat_bits - 14];
		clong = longitude / TD_SENSOR_DATA_GEOLOC_BitsToDiv[long_bits- 14 - 1];
	}
	length = 2;

	/*
	 * sign longitude : 1
	 * sign latitude : 1
	 * latitude : length / 2
	 * longitude : length / 2 (+1 if length is odd)
	 */
	// Append latitude
	TD_SENSOR_ltoba(BITS_TO_LEFT(clat, lat_bits), tmp);
	TD_SENSOR_UTILS_BitConcat(data, &length, tmp, lat_bits);
	TD_SENSOR_ltoba(BITS_TO_LEFT(clong, long_bits), tmp);
	TD_SENSOR_UTILS_BitConcat(data, &length, tmp, long_bits);
	return (int8_t) length;
}

/***************************************************************************//**
 * @brief
 *   Encode a Date & Time Sensor frame.
 *
 * @param[in] datetime
 *   Pointer to the last GPS date & time information.
 *
 * @param[in] data
 *   Pointer to the frame data buffer to fill in.
 *
 * @return
 *   Number of bits on which data is encoded (20) or -1 if failed
 ******************************************************************************/
int8_t TD_SENSOR_EncodeDateTime(TD_GEOLOC_DateTime_t *datetime, uint8_t *data)
{
	uint8_t length;
	uint8_t month;
	uint8_t hour;
	uint8_t minutes;
	uint8_t day;

	if (datetime == 0 || data == 0) {
		return -1;
	}
	day = (datetime->day & 0x1F) << 3;
	month = (datetime->month & 0x0F) << 4;
	hour = (datetime->hours & 0x1F) << 3;
	minutes = (datetime->minutes & 0x3F) << 2;

	/*
	uint8_t month		:4;
	uint8_t day			:5;
	uint8_t hour		:5;
	uint8_t mn			:6;
	*/
	length = 0;
	TD_SENSOR_UTILS_BitConcat(data, &length, &month, 4);
	TD_SENSOR_UTILS_BitConcat(data, &length, &day, 5);
	TD_SENSOR_UTILS_BitConcat(data, &length, &hour, 5);
	TD_SENSOR_UTILS_BitConcat(data, &length, &minutes, 6);
	return (int8_t) length;
}

/***************************************************************************//**
 * @brief
 *   Encode a Position Sensor frame.
 *
 * @param[in] type
 *   Type of GPS data.
 *
 * @param[in] fix
 *   Pointer to the last GPS position.
 *
 * @param[in] data
 *   Pointer to the frame data buffer to fill in.
 *
 * @param[in] length
 *   Pointer the the frame length in bits that will be filled in.
 ******************************************************************************/
void TD_SENSOR_EncodePosition(SensorDataGPSTypes type, TD_GEOLOC_Fix_t *fix, uint8_t *data, uint8_t *length)
{
	uint8_t time[4];
	uint8_t time_length;

	switch (type) {
	case GPS_DATA_XYZ_SV_HDOP:
		TD_SENSOR_EncodePositionQuality(fix, data, length);
		break;

	case GPS_DATA_XY_DATE:
		if( (*length = TD_SENSOR_EncodePositionXY(fix, data, 48, 0)) != 0xFF) {
			if((time_length = TD_SENSOR_EncodeDateTime(&fix->datetime, time)) != 0xFF) {
				TD_SENSOR_UTILS_BitConcat(data, length, time, time_length);
			}
		}
		break;

	case GPS_DATA_XY_1M:
		*length = TD_SENSOR_EncodePositionXY(fix, data, 48, 0);
		break;

	case GPS_DATA_XY_20M:
		*length = TD_SENSOR_EncodePositionXY(fix, data, 40, 0);
		break;

	case GPS_DATA_XY_300M:
		*length = TD_SENSOR_EncodePositionXY(fix, data, 32, 0);
		break;
	}
}

/***************************************************************************//**
 * @brief
 *   Send a Position Sensor data frame.
 *
 * @param[in] type
 *   Type of GPS data.
 *
 * @param[in] fix
 *   Pointer to the last GPS position.
 *
 * @param[in] data
 *   Pointer to the frame data buffer to fill in.
 *
 * @param[in] length
 *   Pointer the the frame length in bits that will be filled in.
 ******************************************************************************/
bool TD_SENSOR_SendDataPosition(SensorDataGPSTypes type, TD_GEOLOC_Fix_t *fix, uint8_t *data, uint8_t length)
{
	uint8_t msg[9];
	uint8_t position[9];
	uint8_t total_length = 0;
	uint8_t pos_length;
	uint8_t byte_count = 0;

	// Bits must be on the left side to be concatenated
	uint8_t temp = type << 4;

	TD_SENSOR_EncodePosition(type, fix, position, &pos_length);

	if (pos_length == 0xFF) {
		return false;
	}

	// Append GPS data type and user payload if possible
	if (pos_length + 4 <= 72) {
		TD_SENSOR_UTILS_BitConcat(msg, &total_length, (uint8_t *) &temp, 4);
		TD_SENSOR_UTILS_BitConcat(msg, &total_length, position, pos_length);
		if (length > 0 && total_length + length <= 72) {
			TD_SENSOR_UTILS_BitConcat(msg, &total_length, data, length);
		}
		byte_count = total_length / 8;
		if (total_length % 8 > 0) {
			byte_count++;
		}
		return TD_SENSOR_SendData(DATA_GPS, msg, byte_count);
	} else {
		return false;
	}
}

/** @} */
/** @} */
