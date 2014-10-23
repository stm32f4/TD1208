/***************************************************************************//**
 * @file
 * @brief Sensor Transmitter
 * @author Telecom Design S.A.
 * @version 1.1.1
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

#ifndef __TD_SENSOR_TRANSMITTER_H
#define __TD_SENSOR_TRANSMITTER_H

#include "sensor_send.h"
#include "td_sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

	/***************************************************************************//**
	 * @addtogroup TD_SENSOR_TRANSMITTER Sensor Transmitter
	 * @{
	 ******************************************************************************/

	/*******************************************************************************
	 *************************   TYPEDEFS     **************************************
	 ******************************************************************************/

	/** @addtogroup TD_SENSOR_TRANSMITTER_TYPEDEFS Typedefs
	 * @{ */

	/** Retransmission structure */
	typedef struct {
		uint8_t payload[10];					///< Data payload to retransmit
		uint8_t timer; 							///< Timer ID to be able to stop it when receiving an ACK
	} TD_SENSOR_TRANSMITTER_Retransmission_t;

	/** structure tu retransmit a SIGFOX frame */
	typedef struct {
		uint8_t data[12];						///< SIGFOX payload data
		uint8_t count;							///< SIGFOX transmit retry count
	} TD_SENSOR_TRANSMITTER_Transmission_t;

	/* Function pointer for code removal purposes */
	typedef void (*TD_SENSOR_TRANSMITTER_Init_t)(void);

	/* Function pointer for code removal purposes */
	typedef void (*TD_SENSOR_TRANSMITTER_Process_t)(void);

	/** @} */

	/*******************************************************************************
	 *************************   PROTOTYPES   **************************************
	 ******************************************************************************/

	/** @addtogroup TD_SENSOR_TRANSMITTER_GLOBAL_FUNCTIONS Global Functions
	 * @{ */

	bool TD_SENSOR_TRANSMITTER_SendSigfox(TD_SENSOR_Frame_t *frame, uint8_t count, uint8_t entry_id, TD_SENSOR_TransmitProfile_t *profile);
	void TD_SENSOR_TRANSMITTER_Process(void);
	void TD_SENSOR_TRANSMITTER_Init(void);
	void TD_SENSOR_TRANSMITTER_SetRetry(uint8_t retries);
	uint8_t TD_SENSOR_TRANSMITTER_LenToRealDuration(uint8_t length);

	/** @} */

	/** @addtogroup TD_SENSOR_TRANSMITTER_USER_FUNCTIONS User Functions
	 * @{ */

	void TD_SENSOR_TRANSMITTER_MonitorDutyCycle(bool enable);

	/** @} */

	/** @} (end addtogroup TD_SENSOR_TRANSMITTER) */

#ifdef __cplusplus
}
#endif

#endif // __TD_SENSOR_TRANSMITTER_H
