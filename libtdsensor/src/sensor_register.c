/***************************************************************************//**
 * @file
 * @brief API for sending Register frame type to Sensor
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

#include <stdint.h>
#include <stdbool.h>

#include "td_sensor.h"
#include "sensor_send.h"
#include "sensor_register.h"

/***************************************************************************//**
 * @addtogroup SENSOR_REGISTER Sensor Register
 * @brief Sensor API for sending a Register Frame
 *
 * The Register frame must be sent to register a new device on Sensor.
 * @{
 ******************************************************************************/

/*******************************************************************************
 ************************   DEFINES   *****************************************
 ******************************************************************************/

/** @addtogroup SENSOR_REGISTER_DEFINES Defines
 * @{ */

#define REGISTER_PAYLOAD_SIZE		7	///< Register payload size in bytes
#define REGISTER_DEFAULT_REPETITON	0	///< Default retransmission repetitions
#define REGISTER_DEFAULT_INTERVAL	0	///< Default retransmission interval in seconds

/** @} */

/*******************************************************************************
 ************************   TYPEDEFS   *****************************************
 ******************************************************************************/

/** @addtogroup SENSOR_REGISTER_TYPEDEFS Typedefs
 * @{ */

/** Register Frame Structure */
typedef struct {
	uint8_t sub_release : 4;			///< UDM sub-release number
	uint8_t release : 4;				///< UDM release number
	uint8_t device_class_byte1 : 8;		///< Device class MSB
	uint8_t device_class_byte2 : 8;		///< Device class LSB
	uint8_t sigfox_id_byte1 : 8;		///< SIGFOX ID MSB
	uint8_t sigfox_id_byte2 : 8;		///< SIGFOX ID lower MSB
	uint8_t sigfox_id_byte3 : 8;		///< SIGFOX ID upper LSB
	uint8_t sigfox_id_byte4 : 8;		///< SIGFOX ID LSB
} __PACKED TD_SENSOR_REGISTER_Frame_t;

/** @} */

/*******************************************************************************
 ************************   PRIVATE VARIABLES   ********************************
 ******************************************************************************/

/** @addtogroup SENSOR_REGISTER_LOCAL_VARIABLES Local Variables
 * @{ */

/** Register Transmit Profile */
static TD_SENSOR_TransmitProfile_t Profile = {REGISTER_DEFAULT_REPETITON, REGISTER_DEFAULT_INTERVAL};

/** Register Stamp */
static uint8_t Stamp = -1;

/** @} */

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup SENSOR_REGISTER_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Send a Register frame to Sensor.
 *
 * @details
 * 	 Only a registered device can be seen on Sensor. This frame must be sent before
 * 	 any other.
 *
 * @return
 *   Returns true if the data has been sent over the SIGFOX network, false
 *   otherwise.
 ******************************************************************************/
bool TD_SENSOR_SendRegister(void)
{
	TD_SENSOR_REGISTER_Frame_t frame;
	TD_SENSOR_Configuration_t *config;
	uint32_t sigfox_id;

	Stamp = (Stamp & 0x07) + 1;
	sigfox_id = TD_SENSOR_GetSigfoxID();
	config = TD_SENSOR_GetModuleConfiguration();
	frame.release = RELEASE & 0x07;
	frame.sub_release = SUB_RELEASE & 0x07;
	frame.device_class_byte1 = config->class >> 8;
	frame.device_class_byte2 = config->class & 0xFF;
	frame.sigfox_id_byte1 = sigfox_id >> 24;
	frame.sigfox_id_byte2 = (sigfox_id >> 16) & 0xFF;
	frame.sigfox_id_byte3 = (sigfox_id >> 8) & 0xFF;
	frame.sigfox_id_byte4 = sigfox_id & 0xFF;
	return TD_SENSOR_Send(&Profile, SRV_FRM_REGISTER, Stamp, (uint8_t *) &frame, REGISTER_PAYLOAD_SIZE);
}

/***************************************************************************//**
 * @brief
 *   Set a transmission profile for a register frame type.
 *
 * @param[in] repetition
 *	Number of repetitions.
 *
 * @param[in] interval
 *	Interval between two repetitions in seconds.
 ******************************************************************************/
void TD_SENSOR_SetRegisterTransmissionProfile(uint8_t repetition, uint32_t interval)
{
	Profile.repetition = repetition;
	Profile.interval = interval;
}

/** @} */

/** @} (end addtogroup SENSOR_REGISTER) */
