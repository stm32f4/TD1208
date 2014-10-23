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

#include <stdbool.h>
#include <stdint.h>

#include <td_scheduler.h>
#include <td_sigfox.h>
#include <td_rtc.h>
#include <td_utils.h>

#include "sensor_private.h"
#include "sensor_send.h"
#include "td_sensor.h"
#include "td_sensor_lan.h"
#include "td_sensor_gateway.h"
#include "td_sensor_device.h"
#include "td_sensor_transmitter.h"

/***************************************************************************//**
 * @addtogroup TD_SENSOR_TRANSMITTER Sensor Transmitter
 * @brief
 *  Interface the SIGFOX API by handling Sensor headers and retransmissions
 * @{
 ******************************************************************************/

/*******************************************************************************
 **************************  DEFINES   ****************************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_TRANSMITTER_DEFINES Defines
 * @{ */

/** Minimum interval in seconds between 2 SIGFOX transmissions. Should only
 * apply to gateways
 */
#define TRANSMISSION_INTERVAL	0

/** Buffer size for keeping transmission history. Minimum transmission duration
 *  is 1.12s and maximum number of transmissions per hour is 36 -> 33 frames to
 *  keep for a 1 hour history depth
 */
#define TRANSMIT_HISTORY_SIZE	33

/** Duty cycle period in seconds (1h)*/
#define DUTY_CYCLE_PERIOD		3600

/* Maximum allowed transmission time in 1/100 seconds during the duty_cycle period (36s) */
#define DUTY_CYCLE_ALLOWED_TX	3600

/** @} */

/*******************************************************************************
 ***********************   ENUMERATIONS   **************************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_TRANSMITTER_ENUMERATIONS Enumerations
 * @{ */

/** Possible SIGFOX transmission durations (v1 only) */
typedef enum {
	SIGFOX_LEN_0 = 0,					///< 1.12 s
	SIGFOX_LEN_1 = 1,					///< 1.20 s
	SIGFOX_LEN_2_4 = 2,					///< 1.44 s
	SIGFOX_LEN_5_8 = 3,					///< 1.76 s
	SIGFOX_LEN_9_12 = 4,				///< 2.08 s
} SigfoxTransmitDuration;

/** @} */

/*******************************************************************************
 **************************  TYPEDEFS   **************************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_TRANSMITTER_TYPEDEFS Typedefs
 * @{ */

// Smaller code without bit packing ...
#pragma pack(1)

/** Retransmission parameters, these are saved by the scheduler */
typedef struct {
	uint8_t index; 							///< Index in the retransmission list
	uint8_t entry_id;						///< Device entry ID
	TD_SENSOR_FrameType_t frame_type;		///< Fame type
	uint8_t payload_count;					///< Payload size in bytes
	uint8_t stamp;							///< Time stamp
} __PACKED TD_TRANSMITTER_Retransmission_t;

/** Past transmission history to compute duty cycle */
typedef struct {
	uint16_t delta;							///< Delta time in seconds since last transmission
	SigfoxTransmitDuration duration : 4;	///< Transmission duration in seconds
	uint8_t retries : 4;					///< Number of retries
} __PACKED PastTransmissions;

/** Last transmission parameters and time stamp */
typedef struct {
	uint64_t time;							///< Absolute time in seconds since last transmission
	SigfoxTransmitDuration duration: 4;		///< Transmission duration in seconds
	uint8_t retries : 4;					///< Number of retries
} __PACKED LastTransmissions;

/** @} */

/*******************************************************************************
 **************************  PRIVATE VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_TRANSMITTER_LOCAL_VARIABLES Local Variables
 * @{ */
#pragma pack(1)

/** List of future retransmissions */
extern TD_SENSOR_TRANSMITTER_Retransmission_t *TD_SENSOR_TRANSMITTER_RetransmissionList;

/** SIGFOX frame queue */
extern TD_SENSOR_TRANSMITTER_Transmission_t *TD_SENSOR_TRANSMITTER_TransmissionQueue;

/** Count of future retransmissions */
static uint8_t QueueCount = 0;

/** Index of future retransmissions */
static uint8_t QueueCurrentIndex = 0;

/* Gateway frame counter */
static uint8_t GatewayCpt = 0;

/** Transmission regulation to allow LAN */
static bool CanEmit = true;

/** Transmission regulation timer */
static uint8_t QueueTimer = 0xFF;

/** SIGFOX retries */
static uint8_t SigfoxRetries = 2;

/** Previous transmission history */
static PastTransmissions PastTransmission[TRANSMIT_HISTORY_SIZE + 1];

/** Last transmission */
static LastTransmissions LastTransmission;

/** Index in history */
static uint8_t PastIndex = TRANSMIT_HISTORY_SIZE - 1;

/** Duty cycle monitoring status */
static bool DutyCycleEnabled = false;

#pragma pack()

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_TRANSMITTER_LOCAL_FUNCTIONS Local Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Convert SIGFOX payload length to normalized transmit duration.
 *
 * @param[in] length
 *   The SIGFOX payload length in bytes.
 *
 * @return
 *   Returns a normalized SigfoxTransmitDuration value.
 ******************************************************************************/
static SigfoxTransmitDuration TD_SENSOR_TRANSMITTER_LenToTransmit(uint8_t length)
{
#pragma pack(1)
	static SigfoxTransmitDuration duration[] = {
		SIGFOX_LEN_0,
		SIGFOX_LEN_1,
		SIGFOX_LEN_2_4,
		SIGFOX_LEN_2_4,
		SIGFOX_LEN_2_4,
		SIGFOX_LEN_5_8,
		SIGFOX_LEN_5_8,
		SIGFOX_LEN_5_8,
		SIGFOX_LEN_5_8,
		SIGFOX_LEN_9_12,
		SIGFOX_LEN_9_12,
		SIGFOX_LEN_9_12,
		SIGFOX_LEN_9_12
	};
#pragma pack()

	if (length < (sizeof (duration) / sizeof (duration[0]))) {
		return duration[length];
	}
	return SIGFOX_LEN_0;
}

/***************************************************************************//**
 * @brief
 *   Convert a SigfoxTransmitDuration duration value to duration in 1/100 seconds.
 ******************************************************************************/
static uint8_t TD_SENSOR_TRANSMITTER_TransmitToRealDuration(SigfoxTransmitDuration transmit_duration)
{
	uint8_t duration;

	switch (transmit_duration) {
	case SIGFOX_LEN_0:

		// 1 bit
		duration = 112;
		break;

	case SIGFOX_LEN_1:

		// 1 byte
		duration = 120;
		break;

	case SIGFOX_LEN_2_4:

		// 2-4 bytes
		duration = 144;
		break;

	case SIGFOX_LEN_5_8:

		// 5-8 bytes
		duration = 176;
		break;

	case SIGFOX_LEN_9_12:

		// 9-12 bytes
		duration = 208;
		break;

	default:
		duration = 0;
		break;
	}
	return duration;
}

/***************************************************************************//**
 * @brief
 *   Find whether regulatory duty cycle allows a transmission or not.
 *
 * @param length
 *   Length in bytes of the payload to be transmitted.
 *
 * @param retries
 *   Retries count, it is used to compute the total transmission duration.
 *
 * @return
 *   Returns true if transmission is allowed, false otherwise.
 ******************************************************************************/
static bool TD_SENSOR_TRANSMITTER_IsTxAllowed(uint8_t length, uint8_t retries)
{
	int i, j;
	uint32_t total_duration;
	uint32_t delta_time;

	// If there is no previous transmission return true
	if (LastTransmission.time == 0xFFFFFFFFFFFFFF || !DutyCycleEnabled) {
		return true;
	}

	// Compute the duration of the transmission we are about to perform
	total_duration = TD_SENSOR_TRANSMITTER_LenToRealDuration(length) * (1 + retries);

	// Previous transmission gives last absolute time so compute delta
	delta_time = (TD_SCHEDULER_GetTime() >> 15) - LastTransmission.time;
	if (delta_time < DUTY_CYCLE_PERIOD) {

		// Append duration for previous transmission
		total_duration += TD_SENSOR_TRANSMITTER_TransmitToRealDuration(LastTransmission.duration)
						  * (1 + LastTransmission.retries);
	}

	// As long as total_duration is < duty cycle allowed transmission
	for (i = 0, j = PastIndex; i < TRANSMIT_HISTORY_SIZE && total_duration < DUTY_CYCLE_ALLOWED_TX; i++) {
		delta_time += PastTransmission[j].delta;

		// Only add duration if within window
		if (delta_time < DUTY_CYCLE_PERIOD) {
			total_duration += TD_SENSOR_TRANSMITTER_TransmitToRealDuration(PastTransmission[j].duration) *
				(1 + PastTransmission[j].retries);
			if (--j < 0) {
				j = TRANSMIT_HISTORY_SIZE - 1;
			}
		} else {

			// We went back in time long enough, stop
			break;
		}
	}
	if (total_duration <= DUTY_CYCLE_ALLOWED_TX) {
		return true;
	} else {
		return false;
	}
}

/***************************************************************************//**
 * @brief
 *   Append a transmission to history.
 *
 * @param length
 *   Length in bytes of the payload to be transmitted.
 *
 * @param retries
 *   Retries count, it is used to compute the total transmission duration.
 ******************************************************************************/
static void TD_SENSOR_TRANSMITTER_DutyCycleAppend(uint8_t length, uint8_t retries)
{
	uint64_t temp;

	// Convert to seconds
	uint64_t now = TD_SCHEDULER_GetTime() >> 15;

	// Compute delta if not first transmission
	if (LastTransmission.time != 0xFFFFFFFFFFFFFF) {
		if (++PastIndex >= TRANSMIT_HISTORY_SIZE) {
			PastIndex = 0;
		}
		temp = now - LastTransmission.time;
		if (temp < 0xFFFF) {
			PastTransmission[PastIndex].delta = temp & 0xFFFF;
		} else {
			PastTransmission[PastIndex].delta = 0xFFFF;
		}
		PastTransmission[PastIndex].duration = LastTransmission.duration;
		PastTransmission[PastIndex].retries = LastTransmission.retries;
	}
	LastTransmission.time = now;
	LastTransmission.duration = TD_SENSOR_TRANSMITTER_LenToTransmit(length);
	LastTransmission.retries = retries;
}

/***************************************************************************//**
 * @brief
 *   Transmit the first SIGFOX frame in the queue.
 *
 * @return
 *   True if the SIGFOX transmission went fine, false otherwise.
 ******************************************************************************/
static bool TD_SENSOR_TRANSMITTER_EmitFirstInQueue(void)
{
	bool ret;

	if (TD_SENSOR_TRANSMITTER_IsTxAllowed(TD_SENSOR_TRANSMITTER_TransmissionQueue[QueueCurrentIndex].count,
		SigfoxRetries)) {

		// Send over SIGFOX
		if (TD_SIGFOX_Send(TD_SENSOR_TRANSMITTER_TransmissionQueue[QueueCurrentIndex].data,
						   TD_SENSOR_TRANSMITTER_TransmissionQueue[QueueCurrentIndex].count,
						   SigfoxRetries)) {
			if (DutyCycleEnabled) {

				// Append transmission to history
				TD_SENSOR_TRANSMITTER_DutyCycleAppend(
					TD_SENSOR_TRANSMITTER_TransmissionQueue[QueueCurrentIndex].count,
					SigfoxRetries);
			}

			// Check battery status
			TD_SENSOR_BatteryCallBack();
		}
		ret = true;
	} else {
		ret = false;
	}

	// Remove the frame from queue
	QueueCurrentIndex++;
	if (QueueCurrentIndex >= CONFIG_TD_SENSOR_TRANSMITTER_MAX_TRANSMIT) {
		QueueCurrentIndex = 0;
	}
	QueueCount--;
	return ret;
}

/***************************************************************************//**
 * @brief
 *   Append a SIGFOX frame to the queue. If the queue is full, send first frame
 *   in queue and then append the new frame. Must be called outside of IRQ context.
 *
 * @param[in] data
 *	Pointer to the bufer containing the data to be sent.
 *
 * @param[in] count
 *	Length of the data in bytes.
 ******************************************************************************/
static void TD_SENSOR_TRANSMITTER_AppendInQueue(uint8_t *data, uint8_t count)
{
	int i;
	int index = 0;

	// If the circular buffer is full, send first one
	if (QueueCount >= CONFIG_TD_SENSOR_TRANSMITTER_MAX_TRANSMIT) {
		TD_SENSOR_TRANSMITTER_EmitFirstInQueue();
	}

	// Append to circular buffer
	index = QueueCurrentIndex + QueueCount;
	if (index >= CONFIG_TD_SENSOR_TRANSMITTER_MAX_TRANSMIT) {
		index -= CONFIG_TD_SENSOR_TRANSMITTER_MAX_TRANSMIT;
	}
	for (i = 0; i < count; i++) {
		TD_SENSOR_TRANSMITTER_TransmissionQueue[index].data[i] = data[i];
	}
	TD_SENSOR_TRANSMITTER_TransmissionQueue[index].count = count;
	QueueCount++;
}

/***************************************************************************//**
 * @brief
 *   Callback function called when the minimum period before a new SIGFOX
 *   transmission can occur.
 *
 * @param[in] arg
 *	Timer parameter. Not used.
 *
 * @param[in] repetitions
 *	Timer parameter. Not used.
 ******************************************************************************/
static void inline TD_SENSOR_TRANSMITTER_QueueManagerCallback(uint32_t arg, uint8_t repetitions)
{
	CanEmit = true;
}

/***************************************************************************//**
 * @brief
 *  Sigfox Queue Manager. Throttle SIGFOX transmissions to minimize LAN collisions.
 *
 * @return
 *   Returns true if a SIGFOX frame was sent successfully, false otherwise.
 ******************************************************************************/
static bool TD_SENSOR_TRANSMITTER_QueueManager(void)
{
	bool ret = false;

	// If we can transmit and transmission list is not empty
	if (QueueCount > 0 && CanEmit) {
		CanEmit = false;
		ret = TD_SENSOR_TRANSMITTER_EmitFirstInQueue();

		// Add timer to tell us when we can transmit
		if (TRANSMISSION_INTERVAL > 0) {
			QueueTimer = TD_SCHEDULER_Append(TRANSMISSION_INTERVAL,
				0,
				0,
				1,
				TD_SENSOR_TRANSMITTER_QueueManagerCallback,
				0);

			// If timer count overflow, then don't care about waiting
			if (QueueTimer == 0xFF) {
				CanEmit = true;
			}
		} else {
			CanEmit = true;
		}
	}
	return ret;
}

/***************************************************************************//**
 * @brief
 *   Send a SIGFOX Sensor-encoded frame.
 *
 * @param[in] frame
 *	Pointer to the buffer containing the Sensor frame to send.
 *
 * @param[in] count
 *	Length in bytes of the data to send.
 *
 * @param[in] retry
 *	Set to true if the frame is a retransmission, false otherwise.
 *
 * @return
 *   Returns true if a SIGFOX frame was sent successfully, false otherwise.
 ******************************************************************************/
static bool TD_SENSOR_TRANSMITTER_SendSigfoxPrivate(TD_SENSOR_Frame_t *frame, uint8_t count, bool retry)
{
	frame->header.retry = retry;
	frame->header.cpt = GatewayCpt;
	GatewayCpt = (GatewayCpt + 1) & 0xF;
	TD_SENSOR_TRANSMITTER_AppendInQueue((uint8_t *) frame, count);
	return TD_SENSOR_TRANSMITTER_QueueManager();
}

/***************************************************************************//**
 * @brief
 *   Retransmission handler function called by timer. Tt saves arguments
 *   information for out-of-context retransmission.
 *
 * @param[in] arg
 *	Timer argument, containing a TD_TRANSMITTER_Retransmission_t structure.
 *
 * @param[in] repetitions
 *	Count of timer repetitions left.
 ******************************************************************************/
static void TD_SENSOR_TRANSMITTER_RetransmissionHandler(uint32_t arg, uint8_t repetitions)
{
	TD_SENSOR_Frame_t frame;
	TD_TRANSMITTER_Retransmission_t param;

	param.index = (arg >> 20) & 0xFF;
	param.frame_type = (TD_SENSOR_FrameType_t) ((arg >> 16) & 0xF);
	param.entry_id = (arg >> 12) & 0xF;
	param.stamp = (arg >> 8) & 0x7;
	param.payload_count = (arg >> 4) & 0xF;
	memcpy(&frame.payload[0], &TD_SENSOR_TRANSMITTER_RetransmissionList[param.index].payload[0],
		param.payload_count);
	frame.header.stamp = param.stamp;
	frame.header.type = param.frame_type;
	frame.header.entry_id = param.entry_id;
	TD_SENSOR_TRANSMITTER_SendSigfoxPrivate(&frame, param.payload_count, 1);
	if (repetitions == 0) {
		TD_SENSOR_TRANSMITTER_RetransmissionList[param.index].timer = 0xFF;
	}
}

/***************************************************************************//**
 * @brief
 *   Add a Retransmission to the list.
 *
 * @param[in] frame
 *	Pointer to the buffer containing the Sensor frame to send.
 *
 * @param[in] count
 *	Length in bytes of the data to send.
 *
 * @param[in] interval
 *	Interval in seconds at which the frame should be retransmitted.
 *
 * @param[in] repetitions
 *	How many times the frame should be retransmitted.
 *
 * @return
 * 	returns true is the retransmission was added to the list, false if
 * 	the retransmission list is full.
 ******************************************************************************/
static bool TD_SENSOR_TRANSMITTER_AddRetransmission(TD_SENSOR_Frame_t *frame, uint8_t count, uint32_t interval, uint8_t repetitions)
{
	int index;
	TD_TRANSMITTER_Retransmission_t param;
	uint32_t uint_param = 0;

	// Look for first available slot
	for (index = 0; index < CONFIG_TD_SENSOR_TRANSMITTER_MAX_RETRANSMIT; index++) {
		if (TD_SENSOR_TRANSMITTER_RetransmissionList[index].timer != 0xFF) {
			continue;
		} else {
			param.frame_type = frame->header.type;
			param.entry_id = frame->header.entry_id;
			param.stamp = frame->header.stamp;
			param.payload_count = count;
			param.index = index;

			// Copy payload
			memcpy(&TD_SENSOR_TRANSMITTER_RetransmissionList[index].payload[0], &frame->payload[0], count);
			uint_param = ((param.index & 0xFF) << 20)
						 | ((param.frame_type & 0xF) << 16)
						 | ((param.entry_id & 0xF) << 12)
						 | ((param.stamp & 0x7) << 8)
						 | ((param.payload_count & 0xF) << 4);

			// Setup timer on interval with given repetitions
			TD_SENSOR_TRANSMITTER_RetransmissionList[index].timer =
				TD_SCHEDULER_Append(interval,
									0,
									0,
									repetitions,
									TD_SENSOR_TRANSMITTER_RetransmissionHandler,
									uint_param);
			return true;
		}
	}
	return false;
}

/** @} */

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_TRANSMITTER_GLOBAL_FUNCTIONS Global Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Convert a payload length to duration in 1/100 seconds.
 *
 * @param[in] length
 *   The payload length in bytes.
 *
 * @return
 *   Returns the transmission duration in 1/100 s.
 ******************************************************************************/
uint8_t TD_SENSOR_TRANSMITTER_LenToRealDuration(uint8_t length)
{
#pragma pack(1)
	static uint8_t duration[] = {
		112,	// 0
		120,	// 1
		144,	// 2
		144,	// 3
		144,	// 4
		176,	// 5
		176,	// 6
		176,	// 7
		176,	// 8
		208,	// 9
		208,	// 10
		208,	// 11
		208		// 12
	};
#pragma pack()

	if (length < sizeof (duration)) {
		return duration[length];
	}
	return 0;
}

/***************************************************************************//**
 * @brief
 *   Send a Sensor frame to SIGFOX and save retransmission profile.
 *
 *   Add a Retransmission to the list.
 *
 * @param[in] frame
 *	Pointer to the buffer containing the Sensor frame to send.
 *
 * @param[in] count
 *	Length in bytes of the data to send.
 *
 * @param[in] entry_id
 *	Entry ID of the module sending the frame.
 *
 * @param[in] profile
 *	Pointer to the retransmission profile.
 *
 * @return
 * 	Returns true is the retransmission was added to the list, false if
 * 	the retransmission list is full.
 *
 * @return
 *   Returns true if a SIGFOX frame was sent successfully, false otherwise.
 ******************************************************************************/
bool TD_SENSOR_TRANSMITTER_SendSigfox(TD_SENSOR_Frame_t *frame, uint8_t count, uint8_t entry_id, TD_SENSOR_TransmitProfile_t *profile)
{
	frame->header.entry_id = entry_id;

	//TODO: handle transmission list full case
	if (profile->repetition > 0) {
		TD_SENSOR_TRANSMITTER_AddRetransmission(frame,
			count,
			profile->interval,
			profile->repetition);
	}
	return TD_SENSOR_TRANSMITTER_SendSigfoxPrivate(frame, count, 0);
}

/***************************************************************************//**
 * @brief
 *   Process the transmitter queue.
 ******************************************************************************/
void TD_SENSOR_TRANSMITTER_Process(void)
{
	TD_SENSOR_TRANSMITTER_QueueManager();
}

/***************************************************************************//**
 * @brief
 *   Set the retry count for a given SIGFOX frame.
 *
 * @param[in] retries
 *   The number of retries to set: typical SIGFOX transmissions use 2 retries,
 *   but for duty cycle or energy saving purposes, retries can be reduced.
 ******************************************************************************/
void TD_SENSOR_TRANSMITTER_SetRetry(uint8_t retries)
{
	SigfoxRetries = retries;
}

/***************************************************************************//**
 * @brief
 *   Sensor Transmitter initialization.
 ******************************************************************************/
void TD_SENSOR_TRANSMITTER_Init(void)
{
	int i;

	// Set all retransmission timers to unused
	for (i = 0; i < CONFIG_TD_SENSOR_TRANSMITTER_MAX_RETRANSMIT; i++) {
		TD_SENSOR_TRANSMITTER_RetransmissionList[i].timer = 0xFF;
	}

	// Initialize all past transmissions
	for (i = 0; i < TRANSMIT_HISTORY_SIZE; i++) {
		PastTransmission[i].duration = SIGFOX_LEN_0;
		PastTransmission[i].retries = 0;
		PastTransmission[i].delta = 0xFFFF;
	}

	// Initialize last transmission time
	LastTransmission.time = 0xFFFFFFFFFFFFFF;

	// Reset all values and flags
	GatewayCpt = 0;
	QueueCount = 0;
	QueueCurrentIndex = 0;
	QueueTimer = 0xFF;
	CanEmit = true;
}
/** @} */

/** @addtogroup TD_SENSOR_TRANSMITTER_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Set the duty cycle monitoring feature on or off. This will throttle SIGFOX
 *   transmission to comply with EEC regulatory RF duty cycles.
 *
 * @param enable
 *  Set to true to enable duty cycle monitoring, false otherwise.
 ******************************************************************************/
void TD_SENSOR_TRANSMITTER_MonitorDutyCycle(bool enable)
{
	DutyCycleEnabled = enable;
}

/** @} */

/** @} (end addtogroup TD_SENSOR_TRANSMITTER) */
