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
#ifndef __SENSOR_DATA_GEOLOC_H
#define __SENSOR_DATA_GEOLOC_H

#include <stdbool.h>
#include <stdint.h>

#include <td_geoloc.h>

#ifdef __cplusplus
extern "C" {
#endif

	/**************************************************************************//**
	 * @addtogroup SENSOR_DATA_GEOLOC
	 * @brief Geolocalization Sensor data.
	 * @{
	 ******************************************************************************/

	/*******************************************************************************
	 ***********************   ENUMERATIONS   **************************************
	 ******************************************************************************/

	/** @addtogroup SENSOR_DATA_GEOLOC_ENUMERATIONS Enumerations
	 * @{ */

	/** Sensor GPS data types */
	typedef enum {
		GPS_DATA_XYZ_SV_HDOP = 0,	///< XYZ data + Satellites in View + Horizontal Degree of Precision (no compression)
		GPS_DATA_XY_DATE = 1,		///< XY data + date (no compression)
		GPS_DATA_XY_1M = 2,			///< XY data with a 1 m resolution (6 bytes - compressed)
		GPS_DATA_XY_20M = 3,		///< XY data with a 20 m resolution (5 bytes - compressed)
		GPS_DATA_XY_300M = 4,		///< XY data with a 300 m resolution (4 bytes - compressed)
	}
	SensorDataGPSTypes;

	/** @} */

	/*******************************************************************************
	 *************************   PROTOTYPES   **************************************
	 ******************************************************************************/

	/** @addtogroup SENSOR_DATA_GEOLOC_GLOBAL_FUNCTIONS Global Functions
	 * @{ */
	/** @} */

	/** @addtogroup SENSOR_DATA_GEOLOC_USER_FUNCTIONS User Functions
	* @{ */

	bool TD_SENSOR_SendDataPosition(SensorDataGPSTypes type, TD_GEOLOC_Fix_t *fix, uint8_t *data, uint8_t length);
	void TD_SENSOR_EncodePosition(SensorDataGPSTypes type, TD_GEOLOC_Fix_t *fix, uint8_t *data, uint8_t *length);
	int8_t TD_SENSOR_EncodePositionXY(TD_GEOLOC_Fix_t *fix, uint8_t *data, uint8_t len, uint16_t nofix);
	int8_t TD_SENSOR_EncodeDateTime(TD_GEOLOC_DateTime_t *datetime, uint8_t *data);

	/** @} */
	/** @} (end addtogroup SENSOR_DATA_GEOLOC) */

#ifdef __cplusplus
}
#endif

#endif // __SENSOR_DATA_GEOLOC_H
