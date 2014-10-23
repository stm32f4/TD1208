/***************************************************************************//**
 * @file
 * @brief API for sending KeepAlive frame type to Sensor
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

#include <td_module.h>
#include <td_sigfox.h>
#include <td_measure.h>
#include <td_utils.h>

#include "td_sensor.h"
#include "sensor_send.h"
#include "sensor_keepalive.h"

/***************************************************************************//**
 * @addtogroup SENSOR_KEEPALIVE Sensor Keep-Alive
 * @brief
 *  Sensor API for sending a Keep-Alive Frame.
 *
 *  Temperature, battery level information and next expected keep-alive time are
 *  sent alongside with this frame.
 * @{
 ******************************************************************************/

/** @addtogroup SENSOR_KEEPALIVE_DEFINES Defines
 * @{ */

#define MAX_KEEPALIVE_PAYLOAD_SIZE 10	///< Keep-alive payload size in bytes
#define KEEPALIVE_DEFAULT_REPETITON 0 	///< Default retransmission repetition
#define KEEPALIVE_DEFAULT_INTERVAL 0	///< Default retransmission interval in seconds

/** @} */

/*******************************************************************************
 *************************   TYPEDEFS   ****************************************
 ******************************************************************************/

/** @addtogroup SENSOR_KEEPALIVE_TYPEDEFS Typedefs
 * @{ */

/** Keep-Alive Frame Structure */
typedef struct {
	uint8_t voltage;					///< Voltage value
	int8_t temperature;					///< Temperature value
	uint8_t interval;					///< Keep-alive interval in hours
} __PACKED TD_SENSOR_KEEPALIVE_Frame_t;

/** @} */

/*******************************************************************************
 *************************   PRIVATE VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup SENSOR_KEEPALIVE_LOCAL_VARIABLES Local Variables
 * @{ */

/** Transmission profile */
static TD_SENSOR_TransmitProfile_t Profile = {KEEPALIVE_DEFAULT_REPETITON, KEEPALIVE_DEFAULT_INTERVAL};

/** Redundancy counter for Sensor keep-alive */
static uint8_t Stamp = -1;

/** @} */

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup SENSOR_KEEPALIVE_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Send a keep-alive frame to Sensor.
 *
 * @return
 *   Returns true if the data has been sent over the SIGFOX network, false
 *   otherwise.
 ******************************************************************************/
bool TD_SENSOR_SendKeepAlive(void)
{
	TD_SENSOR_KEEPALIVE_Frame_t frame;
	TD_SENSOR_Configuration_t *config = TD_SENSOR_GetModuleConfiguration();

	frame.voltage = TD_SIGFOX_PowerVoltage();
	frame.temperature = TD_MEASURE_TemperatureExtended() / 10;
	if (config->keepalive.monitor) {
		frame.interval = config->keepalive.interval;
	} else {
		frame.interval = 0;
	}
	Stamp = (Stamp & 0x07) + 1;
	return TD_SENSOR_Send(&Profile, SRV_FRM_KEEPALIVE, Stamp, (uint8_t *)&frame, 3);
}

/***************************************************************************//**
 * @brief
 *   Set a transmission profile for keep-alive frame type.
 *
 * @param[in] repetition
 *	 Number of repetitions.
 *
 * @param[in] interval
 *	 Interval between two repetitions in seconds.
 ******************************************************************************/
void TD_SENSOR_SetKeepAliveTransmissionProfile(uint8_t repetition, uint32_t interval)
{
	Profile.repetition = repetition;
	Profile.interval = interval;
}

/** @} */

/** @} (end addtogroup SENSOR_KEEPALIVE) */
