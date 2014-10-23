/***************************************************************************//**
 * @file
 * @brief Local RF for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 2.1.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012-2014 Telecom Design S.A., http://www.telecomdesign.fr</b>
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

#ifndef __TD_LAN_H
#define __TD_LAN_H

#include <stdint.h>
#include <stdbool.h>

#include <td_utils.h>

#ifdef __cplusplus
extern "C" {
#endif

	/***************************************************************************//**
	 * @addtogroup TD_LAN TD LAN
	 * @brief Local RF for the TDxxxx RF modules
	 * @{
	 ******************************************************************************/

	/*******************************************************************************
	 *************************   DEFINES   *****************************************
	 ******************************************************************************/

	/** @addtogroup LAN_DEFINES Defines
	 * @{ */

#define ACK_WIDTH				1				/**< Ack flag field width */
#define ACK_SHIFT				31				/**< Ack flag field shift */
#define FRAME_NUMBER_WIDTH		7				/**< Frame number field width */
#define FRAME_NUMBER_SHIFT		24				/**< Frame number field shift */
#define ADDRESS_WIDTH			24				/**< Address field width */
#define ADDRESS_SHIFT			0				/**< Address field shift */

	/** Macro to compute bit mask given its width */
#define MASK(w)					((1 << w) - 1)

#define GET_ACK(b)				GET_BITFIELD(b, ACK)				/**< Get the ack flag value */
#define SET_ACK(b, v)			SET_BITFIELD(b, ACK, v)				/**< Set the ack flag value */
#define GET_FRAME_NUMBER(b)		GET_BITFIELD(b, FRAME_NUMBER)		/**< Get the frame number value */
#define SET_FRAME_NUMBER(b, v)	SET_BITFIELD(b, FRAME_NUMBER, v)	/**< Set the frame number value */
#define GET_ADDRESS(b)			GET_BITFIELD(b, ADDRESS)			/**< Get the address value */
#define SET_ADDRESS(b, v)		SET_BITFIELD(b, ADDRESS, v)			/**< Set the address value */

	/** TD LAN preamble + synchro size */
#define TD_LAN_PRE_SIZE			9

	/** TD LAN payload size */
#define TD_LAN_PAYLOAD_SIZE		17

	/** TD LAN frame size */
#define TD_LAN_FRAME_SIZE		(TD_LAN_PAYLOAD_SIZE + sizeof (uint32_t))

	/** Negative acknowledgment */
#define TD_LAN_NACK				0U

	/** Positive acknowledgment */
#define TD_LAN_ACK				1U

	/** @} */

	/*******************************************************************************
	 ***********************   ENUMERATIONS   **************************************
	 ******************************************************************************/

	/** @addtogroup LAN_ENUMERATIONS Enumerations
	 * @{ */

	/** LAN RF transaction modes */
	typedef enum {
		MODE_SLEEP,					/**< Idle mode */
		MODE_RECEIVER,				/**< Periodic windowed receive mode */
		MODE_TRANSCEIVER,			/**< Transmit mode, then switch to immediate receive mode */
		MODE_ACK,					/**< Immediate receive mode */
		MODE_TRANSMITTER			/**< Transmit mode */
	}
	TD_LAN_mode_t;

	/** LAN result states */
	typedef enum {
		RESULT_PENDING,				/**< Pending radio transaction */
		RESULT_COMPLETE,			/**< Radio transaction completed */
		RESULT_ABORT				/**< No or aborted radio transaction */
	} TD_LAN_result_t;

	/** LAN error code */
	typedef enum {
		ERROR_NONE,					/**< No error */
		ERROR_CANT_SWAP_RADIO,		/**< After a radio priority change, can't restore LAN radio state */
		ERROR_USER_ABORT,			/**< User have aborted radio transmission */
		ERROR_LAN_BUSY,				/**< LAN is busy (RSSI detected over timeout period) */
		ERROR_RECEIVE_TIMEOUT,		/**< Receive has timeout (no message available) */
		ERROR_TRANSMIT,				/**< Transmit error (no IRQ received) */
		ERROR_SYNC_TIMEOUT,			/**< User timeout during synchronous operation */
		ERROR_CANT_START_RECEIVE	/**< A start receive failed. Radio chip probably messed up */
	} TD_LAN_error_t;

	/** @} */

	/*******************************************************************************
	 *************************   TYPEDEFS   ****************************************
	 ******************************************************************************/

	/** @addtogroup LAN_TYPEDEFS Typedefs
	 * @{ */

	/** LAN RF frame structure
	 *
	 * @details
	 *   The TD_LAN frame consists in a a header and a user payload field:
	 *
	 *   | Header  | Payload  |
	 *   | :-----: | :------: |
	 *   | 4 bytes | 17 bytes |
	 *
	 *   The header field is further made up of several fields:
	 *
	 *   | Addr Low | Addr Med | Addr High | Ack   | Frame Counter |
	 *   | :------: | :------: | :-------: | :---: | :-----------: |
	 *   | 8 bits   | 8 bits   | 8 bits    | 1 bit | 7 bits        |
	 *
	 *   The 24-bit address field contains the sender source address for a message
	 *   or the destination address for an answer.
	 *
	 *   The Ack flag is set by the receiver to acknowledge a received message.
	 *
	 *   The frame counter is a monotonic downwards counter that is decremented for
	 *   each retransmission of a message frame during a single retry. This counter
	 *   is used by network devices to compute the delay until the end of the
	 *   current message.
	  */
	typedef struct {
		uint32_t    header;			/**< Header */
		uint8_t     payload[17];	/**< User payload */
	} TD_LAN_frame_t;

	/** User LAN callback function that will be called when an ack frame is
	 * received with pointers to both the sent and received frames. */
	typedef int (*TD_LAN_callback_t)(TD_LAN_frame_t *tx_frame, TD_LAN_frame_t *rx_frame);

	/** @} */

	/*******************************************************************************
	 *************************   PROTOTYPES   **************************************
	 ******************************************************************************/

	/** @addtogroup LAN_USER_FUNCTIONS User Functions
	 * @{ */

	bool TD_LAN_Init(bool init, uint32_t address, uint32_t mask);
	bool TD_LAN_Release(void);
	bool TD_LAN_Process(void);
	void TD_LAN_SetUserCallback(TD_LAN_callback_t callback);
	TD_LAN_callback_t TD_LAN_GetUserCallback(void);
	uint32_t TD_LAN_ComputeAddress(uint32_t id);
	bool TD_LAN_SendReceive(int count, uint8_t retries, TD_LAN_frame_t *tx_frame, TD_LAN_frame_t *rx_frame);
	bool TD_LAN_SendFrame(int32_t txcount, TD_LAN_frame_t *tx_frame, TD_LAN_frame_t *rx_frame);
	bool TD_LAN_ReceiveFrame(uint32_t rx_period, uint32_t limit, TD_LAN_frame_t *rx_frame);
	bool TD_LAN_ReceiveFrameSync(TD_LAN_frame_t *rx_frame);
	int TD_LAN_ReadLatchedRSSI(void);
	void TD_LAN_Abort(void);
	TD_LAN_error_t TD_LAN_LastError(void);

	/** @} */

	/** @addtogroup LAN_GLOBAL_FUNCTIONS Global Functions
	 * @{ */

	bool TD_LAN_Start(TD_LAN_mode_t transfer_mode, uint32_t count, TD_LAN_frame_t *tx_frame, TD_LAN_frame_t *rx_frame);
	void TD_LAN_Restart(void);
	void TD_LAN_Task(void);
	bool TD_LAN_SetFrequencyLevel(uint32_t frequency, int16_t level);
	uint32_t TD_LAN_GetFrequency(void);
	int16_t TD_LAN_GetPowerLevel(void);
	bool TD_LAN_isActive(void);

	/** @} */

	/** @} (end addtogroup TD_LAN) */

#ifdef __cplusplus
}
#endif

#endif // __TD_LAN_H
