/***************************************************************************//**
 * @file
 * @brief Service Send
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

#ifndef __SENSOR_SEND_H
#define __SENSOR_SEND_H

#include <stdbool.h>
#include <stdint.h>
#include "sensor_private.h"

#ifdef __cplusplus
extern "C" {
#endif

	/***************************************************************************//**
	 * @addtogroup SENSOR_SEND Sensor Send
	 * @{
	 ******************************************************************************/

	/*******************************************************************************
	 ***********************   ENUMERATIONS   **************************************
	 ******************************************************************************/

	 /** @addtogroup SENSOR_SEND_ENUMERATIONS Enumerations
	 * @{ */

	/** Sensor Frame Type */
	typedef enum {
		SRV_FRM_EVENT,
		SRV_FRM_DATA,
		SRV_FRM_REGISTER,
		SRV_FRM_KEEPALIVE,
		SRV_FRM_RAW,
		SRV_FRM_SERVICE,
		SRV_FRM_GEOLOC,
	} TD_SENSOR_FrameType_t;

	/** @} */

	/*******************************************************************************
	 **********************************  TYPEDEFS   *******************************
	 ******************************************************************************/

	/** @addtogroup SENSOR_SEND_TYPEDEFS Typedefs
	 * @{ */

	/** Sensor frame header type */
	typedef struct {
		uint8_t cpt : 4;
		uint8_t stamp : 3;
		bool retry : 1;
		TD_SENSOR_FrameType_t type : 4;
		uint8_t entry_id : 4;
	} __PACKED TD_SENSOR_FrameHeader_t;

	/** Sensor frame type */
	typedef struct {
		TD_SENSOR_FrameHeader_t header;
		uint8_t payload[10];
	} __PACKED TD_SENSOR_Frame_t;

	/** Transmit Profile*/
	typedef struct {
		uint8_t repetition : 4;
		uint32_t interval : 28;

	} __PACKED TD_SENSOR_TransmitProfile_t;

	/** @} */

	/*******************************************************************************
	 *************************   PROTOTYPES   **************************************
	 ******************************************************************************/

	/** @addtogroup SENSOR_SEND_GLOBAL_FUNCTIONS Global Functions
	 * @{ */

	bool TD_SENSOR_Send(TD_SENSOR_TransmitProfile_t *profile, TD_SENSOR_FrameType_t frame_type, uint8_t stamp, uint8_t *payload, uint8_t count);
	void TD_SENSOR_SetTransmissionProfile(TD_SENSOR_FrameType_t type, uint8_t repetition, uint16_t interval);

	/** @} */

	/** @} (end addtogroup SENSOR_SEND) */

#ifdef __cplusplus
}
#endif

#endif // __SENSOR_SEND_H
