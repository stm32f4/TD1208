/***************************************************************************//**
 * @file
 * @brief Sensor Device
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

#include <td_rtc.h>
#include <td_flash.h>
#include <td_lan.h>
#include <td_utils.h>
#include <td_scheduler.h>

#include "sensor_send.h"
#include "td_sensor.h"
#include "td_sensor_lan.h"
#include "td_sensor_transmitter.h"
#include "td_sensor_device.h"

/***************************************************************************//**
 * @addtogroup TD_SENSOR_DEVICE Sensor LAN Device
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   DEFINES   *****************************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_DEVICE_DEFINES Defines
 * @{ */

/** Conditional printf macro */
#define DEBUG_PRINTF(...) //tfp_printf(__VA_ARGS__)

/** Conditional dump macro */
#define DEBUG_DUMP(t,d,s) //tfp_dump(t,d,s)

/* Time in seconds to wait after a forward */
#define SIGFOX_TRANSMISSION_TIME 2

/** Number of entries in the device queue */
#define DEVICE_QUEUE 10

/** Automatic wait time after a send fail, 0 to disable */
#define DEVICE_AUTOMATIC_WAIT_TIME 0

/** @} */

/*******************************************************************************
****************************   TYPEDEFS   **************************************
*******************************************************************************/

/** @addtogroup TD_SENSOR_DEVICE_TYPEDEFS Typedefs
 * @{ */

/** Device queue message entry */
typedef struct {
	TD_SENSOR_LAN_Frame_t frame;		///< Data frame to transmit
	bool tx_on_lan_fail : 1;			///< Transfer over SIGFOX upon LAN failure flag
	uint8_t *data_rx;					///< Received data frame
} __PACKED TD_DeviceQueueFrame;

/** @} */

/*******************************************************************************
 **************************  PRIVATE VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_DEVICE_Private_Variables Private Variables
 * @{ */

/** Received frame */
static TD_LAN_frame_t device_RX;

/** Receive enable flag */
static bool ReceptionEnabled = false;

/** SIGFOX transmission status flag */
static bool TxOnLanFail = false;

/** Direct SIGFOX transmission enable flag */
static bool TxSkipLan = false;

/** Broadcast Mask */
static uint32_t BroadcastMask = 0xFFFFFF;

/** Enable broadcast receive flag */
static bool ListenBroadcast = false;

/** Device Address */
static uint32_t DeviceAddress = 0;

/** LAN Data received callback */
static int8_t (*DataCallback)(bool broadcast, uint32_t address, uint8_t *data, uint8_t len, uint8_t *reply) = 0;

/** LAN Keepalive received callback */
static int8_t (*KeepaliveCallback)(bool broadcast, uint32_t address, uint32_t interval, int8_t rssi, uint8_t *data, uint8_t len, uint8_t *reply) = 0;

/** Gateway busy flag */
static bool GatewayBusy = false;

/** Circular buffer to queue local frame */
static TD_DeviceQueueFrame DeviceQueue[DEVICE_QUEUE];

/** Queue count */
static uint8_t DeviceQueueCount = 0;

/** Queue Index */
static uint8_t DeviceQueueIndex = 0;

/** Asynchronous forward flag */
static bool UseAsynchForward = false;

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_DEVICE_LOCAL_FUNCTIONS Local Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *  Acknowledge a frame.
 *
 * @param[in] rx_frame
 *  Pointer to the buffer containing the received data.
 *
 * @param[in] code
 *  The acknowledge code to send back.
 *
 * @param[in] data
 *  Pointer to the buffer that will receive the data to send back.
 *
 * @param[in] count
 *  Length in bytes of the buffer used to send back data.
 ******************************************************************************/
static void TD_SENSOR_DEVICE_Ack(TD_LAN_frame_t *rx_frame, TD_SENSOR_LAN_AckCode_t code, uint8_t *data, uint8_t count)
{
	int i;
	TD_LAN_frame_t tx_frame;

	// Reply frame to sender with acknowledge flag set
	tx_frame.header = rx_frame->header;
	SET_ACK(tx_frame.header, TD_LAN_ACK);

	// First byte is ack code
	tx_frame.payload[0] = code;

	// If no data or too many bytes then clone everything but last byte from
	// received frame, don't copy ack code into payload
	if (data == 0 || count == 0 || count > TD_LAN_PAYLOAD_SIZE - 1) {
		for (i = 0; i < TD_LAN_PAYLOAD_SIZE - 1; i++) {
			tx_frame.payload[i + 1] = rx_frame->payload[i+1];
		}
	} else {
		for (i = 0; i < count; i++) {
			tx_frame.payload[i + 1] = data[i];
		}

		// Fill with 0s
		for (; i < TD_LAN_PAYLOAD_SIZE - 1; i++) {
			tx_frame.payload[i + 1] = 0;
		}
	}

	// Send an Ack frame
	if (!TD_LAN_SendFrame(1, &tx_frame, rx_frame)) {
		DEBUG_PRINTF("LAN Error %d \r\n", TD_LAN_LastError());
	}
}

/***************************************************************************//**
 * @brief
 *   Gateway busy Handler, called by Scheduler. Release busy flag which delay
 *   device to gateway transmission.
 *
 * @param[in] arg
 *	Timer argument. Not used.
 *
 * @param[in] repetitions
 *	Timer argument. Not used.
 ******************************************************************************/
static void TD_SENSOR_DEVICE_GatewayBusyHandler(uint32_t arg, uint8_t repetitions)
{
	GatewayBusy = false;
}

/***************************************************************************//**
 * @brief
 *   Wait for gateway to be ready in case a forward frame was previously sent.
 *   Passive wait for no consumption.
 ******************************************************************************/
static void TD_SENSOR_DEVICE_WaitForGateway(void)
{
	// Gateway busy is on interrupt, must prevent interrupt
	// to happen right before RTC sleep
	__set_PRIMASK(1);
	while (GatewayBusy) {
		DEBUG_PRINTF("Wait gateway\r\n");

		// TODO: NOT ACCEPTABLE!!!
		TD_RTC_Sleep();
		__set_PRIMASK(0);
		__set_PRIMASK(1);
	}
	__set_PRIMASK(0);
}

/***************************************************************************//**
 * @brief
 *   Send a Frame to the Gateway.
 *
 * @param[in] type
 *   Type of frame to be sent.
 *
 * @param[in] payload
 *   Pointer to the buffer containing the data to be sent.
 *
 * @param[in] count
 *   Length in bytes of the date to send.
 *
 * @param[in] data_rx
 *   Pointer to the buffer that will store received data.
 *
 * @return
 *   Returns the acknowledge result.
 ******************************************************************************/
static TD_SENSOR_LAN_AckCode_t TD_SENSOR_DEVICE_SendFrame(TD_SENSOR_LAN_FrameType_t type, uint8_t *payload, uint8_t count, uint8_t *data_rx)
{
	TD_SENSOR_LAN_AckCode_t ack;
	TD_SENSOR_LAN_Frame_t frame;

	// Wait until gateway isn't transmitting for sure
	// There is NO point trying to transmit when we know for sure that
	// the gateway is busy
	TD_SENSOR_DEVICE_WaitForGateway();
	frame.header = 0;
	SET_LOCALSENSORFRAME_TYPE(frame.header, type);
	SET_LOCALSENSORFRAME_COUNT(frame.header, count);
	memcpy(frame.data, payload, count);
	DEBUG_DUMP("Send frame: ", (uint8_t *) &frame, TD_LAN_PAYLOAD_SIZE);
	ack = TD_SENSOR_LAN_SendFrame(&frame, data_rx);
	if (type == LOCAL_FORWARD) {
		if (ack == ACK_OK) {

			// Gateway is now most likely to be busy
			// Assumes 3 frames are being sent
			DEBUG_PRINTF("Gateway busy for %d\r\n",
				((TD_SENSOR_TRANSMITTER_LenToRealDuration(count - 4) * 3) / 100) + 1);
			if (TD_SCHEDULER_AppendIrq(((TD_SENSOR_TRANSMITTER_LenToRealDuration(count - 4) * 3) / 100) + 1,
				0,
				0,
				1,
				TD_SENSOR_DEVICE_GatewayBusyHandler, 0) != 0xFF) {
				GatewayBusy = true;
			}
		}
	}

#if DEVICE_AUTOMATIC_WAIT_TIME > 0
	if (ack == NOT_ACKED) {

		// Gateway is busy, we just don't know how long
		// TODO: random or custom? JUST REMOVE ALL THIS!!!
		DEBUG_PRINTF("Wait gateway 2\r\n");
		if (TD_SCHEDULER_AppendIrq(DEVICE_AUTOMATIC_WAIT_TIME, 0, 0, 1, TD_SENSOR_DEVICE_GatewayBusyHandler, 0) != 0xFF) {
			GatewayBusy = true;
		}
	}
#endif

	return ack;
}

/***************************************************************************//**
 * @brief
 *   Append a message to the device queue.
 *
 * @param[in] type
 *   Type of frame to be sent.
 *
 * @param[in] payload
 *   Pointer to the buffer containing the data to be sent.
 *
 * @param[in] count
 *   Length in bytes of the date to send.
 *
 * @param[in] data_rx
 *   Pointer to the buffer that will store received data.
 *
 * @return
 *   Returns false if the device queue is full or if the length to send is
 *   too large, true otherwise.
 ******************************************************************************/
static bool TD_SENSOR_DEVICE_AppendInQueue(TD_SENSOR_LAN_FrameType_t type, uint8_t *payload, uint8_t count, uint8_t *data_rx)
{
	int index;

	// If circular buffer is full, return false
	if (DeviceQueueCount >= DEVICE_QUEUE) {
		return false;
	}
	if (count > TD_SENSOR_LAN_PAYLOAD_SIZE) {
		return false;
	}

	// Append to circular buffer
	index = DeviceQueueIndex + DeviceQueueCount;
	if (index >= DEVICE_QUEUE) {
		index -= DEVICE_QUEUE;
	}
	SET_LOCALSENSORFRAME_TYPE(DeviceQueue[index].frame.header, type);
	SET_LOCALSENSORFRAME_COUNT(DeviceQueue[index].frame.header, count);

	// Copy useful data payload
	memcpy(DeviceQueue[index].frame.data, payload, count);

	// Fill remaining bytes with zeros
	memset(&DeviceQueue[index].frame.data[count], 0, TD_SENSOR_LAN_PAYLOAD_SIZE - count);
	DeviceQueue[index].tx_on_lan_fail  = TxOnLanFail;
	DeviceQueueCount++;
	return true;
}

/***************************************************************************//**
 * @brief
 *   Device queue manager.
 ******************************************************************************/
static TD_SENSOR_LAN_AckCode_t TD_SENSOR_DEVICE_QueueManager(void)
{
	TD_SENSOR_LAN_AckCode_t ack;
	TD_SENSOR_LAN_FrameType_t type;
	TD_SENSOR_LAN_ForwardFrame_t forward;
	uint8_t count;

	TD_SENSOR_Frame_t frame;
	TD_SENSOR_TransmitProfile_t profile;

	// If we can transmit and transmission list is not empty
	if (DeviceQueueCount > 0) {
		if (!GatewayBusy) {

			// Treatment will vary depending on type
			type = (TD_SENSOR_LAN_FrameType_t) GET_LOCALSENSORFRAME_TYPE(DeviceQueue[DeviceQueueIndex].frame.header);
			count = GET_LOCALSENSORFRAME_COUNT(DeviceQueue[DeviceQueueIndex].frame.header);
			ack = TD_SENSOR_DEVICE_SendFrame(type, DeviceQueue[DeviceQueueIndex].frame.data, count, 0);

			// If acknowledged then remove from queue
			if (ack != NOT_ACKED) {

				// Remove frame from queue
				DeviceQueueIndex++;
				if (DeviceQueueIndex == DEVICE_QUEUE) {
					DeviceQueueIndex = 0;
				}
				DeviceQueueCount--;
			} else if (type == LOCAL_FORWARD) {

				// If forwarded frame and transmission on LAN failed, send directly over SIGFOX
				if (DeviceQueue[DeviceQueueIndex].tx_on_lan_fail) {

					// Byte copy from array to forward frame (array was created the other way round)
					memcpy((uint8_t *)&forward, DeviceQueue[DeviceQueueIndex].frame.data, count);

					// Get real SIGFOX count
					count -= sizeof(forward.profile);

					// Copy SIGFOX payload to Sensor Frame
					memcpy(frame.payload, forward.sigfox, count);
					profile.interval = GET_LOCALFORWARDFRAME_INTERVAL(forward.profile);
					profile.repetition = GET_LOCALFORWARDFRAME_REPETITION(forward.profile);

					// Send frame directly via SIGFOX
					if (TD_SENSOR_TRANSMITTER_SendSigfox(&frame, count, 0, &profile)) {
						ack = TRANSMITTER_SENT;
					} else {
						ack = TRANSMITTER_BUSY;
					}
				}
			}
			return ack;
		} else {
			return GATEWAY_BUSY;
		}
	}
	return NOT_SENT;
}

/** @} */

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_DEVICE_GLOBAL_FUNCTIONS Global Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *  Send a Forward frame on the LAN.
 *
 * @param[in] payload
 *   Pointer to the buffer containing the ata to be forwarded by the gateway.
 *
 * @param[in] count
 *   Length in bytes of the SIGFOX data (Sensor header + payload data)
 *
 * @param[in] repetition
 *   Retransmissions count.
 *
 * @param[in] interval
 *    Retransmission interval in seconds.
 *
 * @return
 *   Returns the acknowledge result.
 ******************************************************************************/
TD_SENSOR_LAN_AckCode_t TD_SENSOR_DEVICE_Forward(uint8_t *payload, uint8_t count, uint8_t repetition, uint32_t interval)
{
	TD_SENSOR_LAN_ForwardFrame_t local_frame;

	if (count > 12) {
		return SENSOR_LAN_FRAME_ERROR;
	}

	// Fill in frame data
	local_frame.profile = 0;
	SET_LOCALFORWARDFRAME_REPETITION(local_frame.profile, repetition);
	SET_LOCALFORWARDFRAME_INTERVAL(local_frame.profile, interval);

	// Copy sensor frame header + real data
	// Order matters here as frame starts with header then data follows
	memcpy(local_frame.sigfox, payload, count);

	// Useful payload size, not frame size!
	return TD_SENSOR_DEVICE_SendFrame(LOCAL_FORWARD, (uint8_t *) &local_frame, count + sizeof (local_frame.profile), 0);
}

/***************************************************************************//**
 * @brief
 *  Send a Forward frame on LAN asynchronously.
 *
 * @param[in] payload
 *   Pointer to the buffer containing the ata to be forwarded by the gateway.
 *
 * @param[in] count
 *   Length in bytes of the SIGFOX data (Sensor header + payload data)
 *
 * @param[in] repetition
 *  Retransmissions count.
 *
 * @param[in] interval
 *   Retransmission interval in seconds.
 *
 * @return
 *  Returns the acknowledge result if frame could be sent right away.
 ******************************************************************************/
TD_SENSOR_LAN_AckCode_t TD_SENSOR_DEVICE_ForwardAsynch(uint8_t *payload, uint8_t count, uint8_t repetition, uint32_t interval)
{
	TD_SENSOR_LAN_ForwardFrame_t local_frame;

	if (count > 12) {
		return SENSOR_LAN_FRAME_ERROR;
	}

	// Fill in frame data
	local_frame.profile = 0;
	SET_LOCALFORWARDFRAME_REPETITION(local_frame.profile, repetition);
	SET_LOCALFORWARDFRAME_INTERVAL(local_frame.profile, interval);

	// Copy sensor frame header + real data
	// Order matters here as frame starts with header then data follows
	memcpy(local_frame.sigfox, payload, count);

	// Useful payload size, not frame size!
	if (!TD_SENSOR_DEVICE_AppendInQueue(LOCAL_FORWARD, (uint8_t *) &local_frame, count + sizeof (local_frame.profile), 0)) {
		return SENSOR_LAN_QUEUED;
	} else {
		return TD_SENSOR_DEVICE_QueueManager();
	}
}

/***************************************************************************//**
 * @brief
 *  Send a Keep alive frame on LAN
 *
 * @param[in] keepalive
 *   Set to true to enable connection monitoring on gateway side. Set to false to
 *   disable connection monitoring.
 *
 * @param[in] interval
 *   Keep-alive interval monitoring in seconds if keepalive is true.
 *
 * @param[in] rssi
 *   Set to true to enable RSSI monitoring on gateway side. Set to false to
 *   disable RSSI monitoring.
 *
 * @param[in] level_low
 * 	RSSI level low monitoring value if RSSI is true.
 *
 * @param[in] level_ok
 * 	RSSI level ok monitoring value if RSSI is true.
 *
 * @return
  *   Returns the acknowledge result.
 ******************************************************************************/
TD_SENSOR_LAN_AckCode_t TD_SENSOR_DEVICE_KeepAlive(bool keepalive, uint32_t interval, bool rssi, int8_t level_low, int8_t level_ok)
{
	TD_SENSOR_LAN_KeepAliveFrame_t frame;

	frame.keepalive = keepalive;
	frame.interval = interval;
	frame.rssi = rssi;
	frame.level_low = level_low;
	frame.level_ok = level_ok;

	// Useful payload size, not frame size!
	return TD_SENSOR_DEVICE_SendFrame(LOCAL_KEEPALIVE, (uint8_t *) &frame, sizeof(TD_SENSOR_LAN_KeepAliveFrame_t), 0);
}

/** @} */

/** @addtogroup TD_SENSOR_DEVICE_USER_FUNCTIONS
 * @{ */

/***************************************************************************//**
 * @brief
 *  Send a Data frame on LAN.
 *
 * @param[in] data
 *   Pointer to the buffer containing the ata to be forwarded by the gateway.
 *
 * @param[in] count
 *   Data length in bytes.
 *
 * @param[in] data_rx
 *   Pointer to the buffer that will receive the gateway reply.
 *
 * @return
 *   Returns the acknowledge result.
 ******************************************************************************/
TD_SENSOR_LAN_AckCode_t TD_SENSOR_DEVICE_Data(uint8_t *data, uint8_t count, uint8_t data_rx[TD_SENSOR_LAN_PAYLOAD_SIZE - 1])
{
	if (count > 16) {
		return SENSOR_LAN_FRAME_ERROR;
	}

	// Useful payload size, not frame size!
	return TD_SENSOR_DEVICE_SendFrame(LOCAL_DATA, (uint8_t *) data, count, data_rx);
}

/***************************************************************************//**
 * @brief
 *  Send a Register frame on LAN
 *
 * @return
 *   Returns the acknowledge result.
 ******************************************************************************/
TD_SENSOR_LAN_AckCode_t TD_SENSOR_DEVICE_Register(void)
{
	TD_SENSOR_LAN_RegisterFrame_t frame;
	TD_SENSOR_LAN_Address_t lan_address;
	TD_SENSOR_LAN_AckCode_t code;
	uint8_t rx[TD_SENSOR_LAN_PAYLOAD_SIZE];
	const TD_SENSOR_Configuration_t *config;

	// Each time a registration is requested device address is reset
	TD_SENSOR_DEVICE_SetAddress(0, 0xFFFFFF);
	config = TD_SENSOR_GetModuleConfiguration();
	frame.SigfoxID = TD_SENSOR_GetSigfoxID();
	frame.device_class = config->class;

	// Useful payload size, not frame size!
	if ((code = TD_SENSOR_DEVICE_SendFrame(LOCAL_REGISTER,
		(uint8_t *) &frame,
		sizeof (TD_SENSOR_LAN_RegisterFrame_t),
		rx)) == ACK_OK) {
		switch (code) {
		case ACK_OK:
			lan_address.address = rx[0] | rx[1] << 8 | rx[2] << 16 | rx[3] << 24;
			DeviceAddress = lan_address.address;
			TD_SENSOR_LAN_setLanAddress(lan_address.address, BROADCAST_MASK);
			TD_FLASH_WriteVariables();
			break;

		default:
			break;
		}
	}
	return code;
}

/***************************************************************************//**
 * @brief
 *   Device received frame handler.
 *
 * @param[in] tx_frame
 *   Pointer to a buffer containing the transmitted frame.
 *
 * @param[in] rx_frame
 *   Pointer to a buffer containing the received frame.
 *
 * @return
 * 	Always return 1;
 ******************************************************************************/
static int TD_SENSOR_DEVICE_FrameReceived(TD_LAN_frame_t *tx_frame, TD_LAN_frame_t *rx_frame)
{
	uint8_t frame_count;
	int i;
	TD_SENSOR_LAN_Frame_t *frame;
	TD_SENSOR_LAN_FrameType_t type;
	bool broadcast = false;
	uint32_t address;
	uint8_t reply[15];
	int8_t reply_count;

	// We don't want to handle acknowledgments here
	if (GET_ACK(rx_frame->header)) {
		return 1;
	}
	address = GET_ADDRESS(rx_frame->header);
	if (ListenBroadcast) {
		if (address == (DeviceAddress | 0xFF)) {
			broadcast = true;
		} else if ((address & BroadcastMask) != (DeviceAddress & BroadcastMask)) {
			if (ReceptionEnabled) {
				TD_LAN_ReceiveFrame(32768 * 1, 0, &device_RX);
			}
			return 1;
		}
	}
	frame = (TD_SENSOR_LAN_Frame_t *) rx_frame->payload;
	type = (TD_SENSOR_LAN_FrameType_t) GET_LOCALSENSORFRAME_TYPE(frame->header);
	if (type == LOCAL_DATA) {
		frame_count = GET_LOCALSENSORFRAME_COUNT(frame->header);

		// Data count is on 4 bits but we need to handle 0-16 length
		if (frame_count == 0) {

			// If one of the byte is !=0 then  count = 16 otherwise count = 0
			for (i = 0; i < TD_LAN_PAYLOAD_SIZE - 1; i++) {
				if (frame->data[i] != 0) {
					frame_count = 16;
					break;
				}
			}
		}
		if (DataCallback != 0) {
			reply_count = (*DataCallback)(broadcast, address, frame->data, frame_count, reply);
			if (reply_count >= 0 && !broadcast) {
				TD_SENSOR_DEVICE_Ack(rx_frame, ACK_OK, reply, reply_count);
			}
		} else if (!broadcast) {

			// Acknowledge back
			TD_SENSOR_DEVICE_Ack(rx_frame, ACK_OK, frame->data, frame_count);
		}
	} else if (type == LOCAL_KEEPALIVE) {
		TD_SENSOR_LAN_KeepAliveFrame_t *keepalive_frame;
		int8_t rssi = TD_LAN_ReadLatchedRSSI();
		keepalive_frame = (TD_SENSOR_LAN_KeepAliveFrame_t *)(frame->data);
		if (KeepaliveCallback != 0) {
			reply_count = (*KeepaliveCallback)(broadcast,
				address,
				keepalive_frame->interval,
				rssi,
				/* frame->data */ 0,
				/* frame_count */ 0,
				0);
			if (reply_count >= 0 && !broadcast) {
				TD_SENSOR_DEVICE_Ack(rx_frame, ACK_OK, reply, reply_count);
			}
		} else if (!broadcast) {

			// Acknowledge back
			TD_SENSOR_DEVICE_Ack(rx_frame, ACK_OK, 0, 0);
		}
	}
	if (ReceptionEnabled) {
		TD_LAN_ReceiveFrame(32768 * 1, 0, &device_RX);
	}
	return 1;
}

/***************************************************************************//**
 * @brief
 *  Set a data receive callback function.
 *
 * @param[in] user_data_callback
 *   Pointer to the callback function called upon data reception with the
 *   following arguments:
 *     - broadcast: flag set to true if the received frame is broadcasted
 *     - address: LAN address of the sender
 *     - interval: the requested keep-alive interval in seconds
 *     - rssi: the received RSSI level
 *     - data: pointer to a buffer containing the keep-alive frame data
 *     - length: length of the keep-alive data in bytes
 *     - reply: pointer to a buffer that will receive the reply to send
 *   The callback function should return a retry count >= 0 to send an
 *   acknowledgment reply, or < 0 to not send an acknowledgment reply.
 ******************************************************************************/
void TD_SENSOR_DEVICE_SetDataCallback(int8_t (*user_data_callback)(bool broadcast, uint32_t address, uint8_t *data, uint8_t len, uint8_t *reply))
{
	DataCallback = user_data_callback;
}

/***************************************************************************//**
 * @brief
 *   Set a keep-alive callback function.
 *
 * @param[in] user_keepalive_callback
 *   Pointer to the callback function called upon keep-alive reception with the
 *   following arguments:
 *     - broadcast: flag set to true if the received frame is broadcasted
 *     - address: LAN address of the sender
 *     - interval: the requested keep-alive interval in seconds
 *     - rssi: the received RSSI level
 *     - data: pointer to a buffer containing the keep-alive frame data
 *     - length: length of the keep-alive data in bytes
 *     - reply: pointer to a buffer that will receive the reply to send
 *   The callback function should return a retry count >= 0 to send an
 *   acknowledgment reply, or < 0 to not send an acknowledgment reply.
 ******************************************************************************/
void TD_SENSOR_DEVICE_SetKeepAliveCallback(int8_t (*user_keepalive_callback)(bool broadcast, uint32_t address, uint32_t interval, int8_t rssi, uint8_t *data, uint8_t length, uint8_t *reply))
{
	KeepaliveCallback = user_keepalive_callback;
}

/***************************************************************************//**
 * @brief
 *  Start a normal reception.
 ******************************************************************************/
void TD_SENSOR_DEVICE_StartReception(void)
{
	BroadcastMask = 0xFFFFFF;
	TD_LAN_SetUserCallback(TD_SENSOR_DEVICE_FrameReceived);
	ReceptionEnabled = true;
	TD_SENSOR_LAN_setLanAddress(DeviceAddress, BROADCAST_MASK);
	TD_LAN_ReceiveFrame(T1S, 0, &device_RX);
}

/***************************************************************************//**
 * @brief
 *  Process the device queue.
 ******************************************************************************/
void TD_SENSOR_DEVICE_Process(void)
{
	TD_SENSOR_DEVICE_QueueManager();
}

/***************************************************************************//**
 * @brief
 *  Start broadcast reception. Allow to setup an additional mask which will
 *  be applied by software.
 *
 * @param[in] mask
 *  An additional mask address to apply to the received frame.
 ******************************************************************************/
void TD_SENSOR_DEVICE_StartBroadcastReception(uint32_t mask)
{
	BroadcastMask = mask;
	ListenBroadcast = true;
	TD_LAN_SetUserCallback(TD_SENSOR_DEVICE_FrameReceived);
	ReceptionEnabled = true;
	TD_SENSOR_LAN_setLanAddress(DeviceAddress | 0xFF, 0xFFFFFF);
	TD_LAN_ReceiveFrame(T1S, 0, &device_RX);
}

/***************************************************************************//**
 * @brief
 *  Start a synchronous reception. Blocks until a frame is received or
 *  TD_SENSOR_DEVICE_StopReception is used.
 ******************************************************************************/
bool TD_SENSOR_DEVICE_StartSynchReception(void)
{
	TD_LAN_SetUserCallback(TD_SENSOR_DEVICE_FrameReceived);
	ReceptionEnabled = true;
	TD_SENSOR_LAN_setLanAddress(DeviceAddress, BROADCAST_MASK);
	return TD_LAN_ReceiveFrameSync(&device_RX);
}

/***************************************************************************//**
 * @brief
 *  Stop the module reception.
 ******************************************************************************/
void TD_SENSOR_DEVICE_StopReception(void)
{
	TD_LAN_SetUserCallback(0);
	TD_LAN_Abort();
	ReceptionEnabled = false;
	ListenBroadcast = false;
}

/***************************************************************************//**
 * @brief
 *  Check whether the device is registered or not.
 *
 * @return
 *  Returns true if the device is registered, false otherwise.
 ******************************************************************************/
bool TD_SENSOR_DEVICE_isRegistered(void)
{
	if (DeviceAddress == BROADCAST_ADDRESS) {
		return false;
	} else {
		return true;
	}
}

/***************************************************************************//**
 * @brief
 *  Check whether normal reception is enabled or not.
 *
 * @return
 *  Returns true if normal reception is enabled, false otherwise.
 ******************************************************************************/
bool TD_SENSOR_DEVICE_isReceptionEnabled(void)
{
	return ReceptionEnabled;
}

/***************************************************************************//**
 * @brief
 *  Check whether broadcast reception is enabled or not.
 *
 * @return
 *  Returns true if broadcast reception is enabled, false otherwise.
 ******************************************************************************/
bool TD_SENSOR_DEVICE_isBroadcastReceptionEnabled(void)
{
	if (ReceptionEnabled && ListenBroadcast) {
		return true;
	} else {
		return false;
	}
}

/***************************************************************************//**
 * @brief
 *  Get the broadcast mask.
 *
 * @return
 *   Returns the broadcast mask.
 ******************************************************************************/
uint32_t TD_SENSOR_DEVICE_GetBroadcastMask(void)
{
	return BroadcastMask;
}

/***************************************************************************//**
 * @brief
 *  Enable/disable transmitting a frame directly to SIGFOX if the device could
 *  not reach the gateway.
 *
 * @param[in] enable
 *  Enable direct transmission to the SIGFOX network upon LAN failure if set to
 *  true, disable it if set to false.
 ******************************************************************************/
void TD_SENSOR_DEVICE_SetTxOnLanFail(bool enable)
{
	TxOnLanFail = enable;
}

/***************************************************************************//**
 * @brief
 *  Return whether transmitting a frame directly to SIGFOX if the device could
 *  not reach the gateway is enabled.
 *
 * @return
 *  Returns true if transmission to the SIGFOX network upon LAN failure is
 *  enabled, false otherwise.
 ******************************************************************************/
bool TD_SENSOR_DEVICE_GetTxOnLanFail(void)
{
	return TxOnLanFail;
}

/***************************************************************************//**
 * @brief
 *  Enable/disable direct transmission of a frame via SIGFOX without trying to
 *  reach the gateway.
 *
 *  Enable direct transmission to the SIGFOX network without trying to reach the
 *  gateway if set to true, disable it if set to false.
 ******************************************************************************/
void TD_SENSOR_DEVICE_SetTxSkipLan(bool enable)
{
	TxSkipLan = enable;
}

/***************************************************************************//**
 * @brief
 *  Return whether direct transmission of a frame via SIGFOX without trying to
 *  reach the gateway is enabled.
 *
 * @return
 *  Returns true if transmission to the SIGFOX network without trying to reach
 *  the gateway is enabled, false otherwise.
 ******************************************************************************/
bool TD_SENSOR_DEVICE_GetTxSkipLan(void)
{
	return TxSkipLan;
}

/***************************************************************************//**
 * @brief
 *  Reset the device address to broadcast
 ******************************************************************************/
void TD_SENSOR_DEVICE_Reset(void)
{
	DeviceAddress = BROADCAST_ADDRESS;
	TD_SENSOR_LAN_setLanAddress(BROADCAST_ADDRESS, BROADCAST_MASK);
}

/***************************************************************************//**
 * @brief
 *  Force device address and mask
 *
 * @param[in] address
 *  The new device address.
 *
 * @param[in] mask
 *  The new device mask mask.
 *
 * @return
 *  Returns true if the address and mask could be applied successfully, false
 *  otherwise.
 ******************************************************************************/
bool TD_SENSOR_DEVICE_SetAddress(uint32_t address, uint32_t mask)
{
	DeviceAddress = address;
	return TD_SENSOR_LAN_setLanAddress(address, mask);
}

/***************************************************************************//**
 * @brief
 *  Get the device address (may not be CURRENT address,
 *  see TD_SENSOR_LAN_GetAddress for current address).
 *
 * @return
 *  Returns the device address.
 ******************************************************************************/
uint32_t TD_SENSOR_DEVICE_GetAddress(void)
{
	return DeviceAddress;
}

/***************************************************************************//**
 * @brief
 *  Set whether a forward should be synchronous or asynchronous.
 *
 *  In asynchronous mode, the device will retry to send the frame until the
 *  gateway replies.
 *
 *  The retry interval is fixed by the DEVICE_AUTOMATIC_WAIT_TIME constant.
 *
 * @param[in] asynch
 *  Enable asynchronous forward if set to true, disable it otherwise.
 ******************************************************************************/
void TD_SENSOR_DEVICE_SetAsynchronousForward(bool asynch)
{
	UseAsynchForward = asynch;
}

/***************************************************************************//**
 * @brief
 *  Check whether asynchronous mode is currently active or not.
 *
 * @return
 *  Returns whether asynchronous mode is currently active or not.
 ******************************************************************************/
bool TD_SENSOR_DEVICE_IsAsynchronousForward(void)
{
	return UseAsynchForward;
}

/** @} */

/** @} (end addtogroup TD_SENSOR_DEVICE) */
