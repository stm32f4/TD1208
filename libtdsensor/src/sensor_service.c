/***************************************************************************//**
 * @file
 * @brief API for sending Service frame type to Sensor
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

#include "sensor_send.h"
#include "sensor_service.h"

/***************************************************************************//**
 * @addtogroup SENSOR_SERVICE Sensor Service
 * @brief Sensor API for sending a Service Frame
 *
 * @details
 *	There are two service frames:
 *
 *	- SERVICE_SMS:
 *		Allows sending an SMS to a previously registered cell phone number.
 *
 *	- SERVICE_TWEET
 *		Allows sending a tweet on a previously registered twitter account.
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   DEFINES   *****************************************
 ******************************************************************************/

/** @addtogroup SENSOR_SERVICE_DEFINES Defines
 * @{ */

#define SERVICE_PAYLOAD_SIZE		10		///< Service payload size in bytes
#define MAX_SMS_LEN					9		///< Maximum SMS message size in bytes
#define MAX_TWEET_LEN				9		///< Maximum Tweet message size in bytes
#define SERVICE_DEFAULT_REPETITON	0		///< Default retransmission repetitions
#define SERVICE_DEFAULT_INTERVAL	0		///< Default retransmission interval in seconds

/** @} */

/*******************************************************************************
****************************   TYPEDEFS   **************************************
*******************************************************************************/

/** @addtogroup SENSOR_SERVICE_TYPEDEFS Typedefs
 * @{ */

/** Service Frame Structure */
typedef struct {
	TD_SENSOR_SERVICE_Types_t type : 8;		///< The Sensor service type
	uint8_t data[9];						///< The data payload
} __PACKED TD_SENSOR_SERVICE_Frame_t;

/** @} */

/*******************************************************************************
 *************************   PRIVATE VARIABLES   ****************************************
 ******************************************************************************/

/** @addtogroup SENSOR_SERVICE_LOCAL_VARIABLES Local Variables
 * @{ */

/** Transmission profile */
static TD_SENSOR_TransmitProfile_t Profile = {SERVICE_DEFAULT_REPETITON, SERVICE_DEFAULT_INTERVAL};

/** Redundancy counter for Sensor service data */
static uint8_t Stamp = -1;

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup SENSOR_SERVICE_LOCAL_FUNCTIONS Local Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Send a Service frame to Sensor.
 *
 * @param[in] service
 *   Service Type
 *
 * @param[in] data
 *   Data to be sent.
 *
 * @param[in] count
 *   Data length
 *
 * @return
 *   True if the data has been sent (ie. the gateway has acknowledged the request)
 *   False if count>Data size or if the ack from the gateway was never received.
 ******************************************************************************/
static bool TD_SENSOR_SendService(TD_SENSOR_SERVICE_Types_t service, uint8_t *data, uint8_t count)
{
	unsigned int i;
	TD_SENSOR_SERVICE_Frame_t frame;

	// Think smaller but not too small!
	if (count > 10 || count == 0) {
		return false;
	}

	// First byte is Service type
	frame.type = service;
	for (i = 0; i < count; i++) {
		frame.data[i] = data[i];
	}
	Stamp = (Stamp & 0x07) + 1;
	return TD_SENSOR_Send(&Profile, SRV_FRM_SERVICE, Stamp, (uint8_t *) &frame, count + 1);
}

/** @} */

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup SENSOR_SERVICE_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Send an SMS.
 *
 * @details
 * 	 At most 9 bytes can be sent.
 *
 * @return
 *   Returns true if the data has been sent over the SIGFOX network, false
 *   otherwise.
 ******************************************************************************/
bool TD_SENSOR_SendSMS(uint8_t *SMS)
{
	uint8_t count = 0;

	while (count <= MAX_SMS_LEN) {
		if (SMS[count] == 0) {
			return TD_SENSOR_SendService(SERVICE_SMS, SMS, count);
		} else {
			count++;
		}
	}
	return false;
}

/***************************************************************************//**
 * @brief
 *   Send a tweet.
 *
 * @details
 * 	 At most 9 bytes can be sent.
 *
 * @return
 *   Returns true if the data has been sent over the SIGFOX network, false
 *   otherwise.
 ******************************************************************************/
bool TD_SENSOR_SendTweet(uint8_t *Tweet)
{
	uint8_t count = 0;

	while (count <= MAX_TWEET_LEN) {
		if (Tweet[count] == 0) {
			return TD_SENSOR_SendService(SERVICE_TWEET, Tweet, count);
		} else {
			count++;
		}
	}
	return false;
}

/***************************************************************************//**
 * @brief
 *   Set a transmission profile for a service frame type.
 *
 * @param[in] repetition
 *	Number of repetitions.
 *
 * @param[in] interval
 *	Interval between two repetitions in seconds.
 ******************************************************************************/
void TD_SENSOR_SetServiceTransmissionProfile(uint8_t repetition, uint32_t interval)
{
	Profile.repetition = repetition;
	Profile.interval = interval;
}

/** @} */

/** @} (end addtogroup SENSOR_SERVICE) */
