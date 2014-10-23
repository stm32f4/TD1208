/***************************************************************************//**
 * @file
 * @brief Sensor Gateway
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

#include <td_lan.h>
#include <td_flash.h>
#include <td_rtc.h>
#include <td_scheduler.h>
#include <td_printf.h>

#include "sensor_private.h"
#include "sensor_event.h"
#include "td_sensor.h"
#include "td_sensor_lan.h"
#include "td_sensor_device.h"
#include "td_sensor_transmitter.h"
#include "td_sensor_gateway.h"

/***************************************************************************//**
 * @addtogroup TD_SENSOR_GATEWAY Sensor LAN Gateway
 * @{
 ******************************************************************************/

/*******************************************************************************
 **************************  DEFINES   ****************************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_GATEWAY_DEFINES Defines
 * @{ */

/** Conditional printf macro */
#define DEBUG_PRINTF(...) /*tfp_printf(__VA_ARGS__)*/

/* Time in seconds to wait after a forward */
#define DEBUG_DUMP(t,d,s) /*tfp_dump(t,d,s)*/

/** @} */

/*******************************************************************************
 **************************  PRIVATE VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_GATEWAY_LOCAL_VARIABLES Local Variables
 * @{ */

/** Frame received buffer */
static TD_LAN_frame_t gateway_RX;

/** Computed Gateway address */
static uint32_t GatewayAddress = 0;

/** Registered device list */
extern TD_SENSOR_GATEWAY_Device_t *DeviceList;

/** Registered device count */
static uint32_t DeviceCount = 1;

/** Registration enable flag */
static bool RegistrationEnabled = false;

/** Reception enable flag */
static bool ReceptionEnabled  = false;

/** Reception activated by the registration flag */
static bool ReceptionEnByReg = false;

/** LAN Data received callback */
static int8_t (*DataCallback)(uint8_t *data, uint8_t len, uint8_t *reply) = 0;

/** Registration callback */
static void (*RegistrationCallback)(uint32_t lan_address, uint32_t sigfox_id) = 0;

/* For code removal purposes */
extern TD_LAN_callback_t const TD_SENSOR_GATEWAY_FrameReceived_;

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_GATEWAY_LOCAL_FUNCTIONS Local Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *  Compute a 16 bit address from a 32 bit one.
 *
 * @param[in] address
 *  The input 32 bits address.
 *
 * @return
 *  The output 16 bit address.
 ******************************************************************************/
static uint16_t TD_SENSOR_LAN_ComputeAddressTo16bits(uint32_t address)
{
	uint8_t xor_even = 0, xor_odd = 0;

	xor_even = ((address >> 24) & 0xFF) ^ ((address >> 8) & 0xFF);
	xor_odd = ((address >> 16) & 0xFF) ^ ((address >> 0) & 0xFF);
	return (xor_even << 8) | xor_odd;
}

/***************************************************************************//**
 * @brief
 *  Compute an 8 bit address from a 32 bit one.
 *
 * @param[in] address
 *  The input 32 bits address.
 *
 * @return
 *  The output 8 bit address.
 ******************************************************************************/
static uint8_t TD_SENSOR_LAN_ComputeAddressTo8bits(uint32_t address)
{
	int i;
	uint8_t xor = 0;

	for (i = 0; i < 4; i++) {
		xor ^= (address >> (8 * i));
	}
	return xor;
}

/***************************************************************************//**
 * @brief
 *   Keep-Alive handler, called by Scheduler.
 *
 * @details
 *   This function makes sure that the device with its connection monitored
 *   has emitted its keep-alive frame within schedule.
 *
 * @param[in] arg
 *	Timer argument. Contains the device entry ID.
 *
 * @param[in] repetitions
 *	Timer argument. Not used.
 ******************************************************************************/
static void TD_SENSOR_GATEWAY_KeepAliveHandler(uint32_t arg, uint8_t repetitions)
{
	uint8_t prev_status;
	uint8_t entry_id = arg;

	// Save previous status
	prev_status = DeviceList[entry_id].config.keepalive.status;
	DeviceList[entry_id].config.keepalive.status = DeviceList[entry_id].config.keepalive.validated;

	// Compare status with previous
	if (prev_status != (uint8_t) DeviceList[entry_id].config.keepalive.status) {
		if (!DeviceList[entry_id].config.keepalive.status) {

			// Send connection lost
			TD_SENSOR_SendEventConnection(0, entry_id);
		}
	}

	// Reset keep alive received indicator
	DeviceList[entry_id].config.keepalive.validated = false;
}

/***************************************************************************//**
 * @brief
 *   Append an Device to the registration list.
 *
 * @param[in] device
 *   Pointer to the device to append. Set its lan_address field to zero to
 *   generate an address according to the SIGFOX ID, or set it to 1..15 to force
 *   a device address.
 *
 * @return
 *   Returns an entry ID if the device has been added, 255 otherwise.
 ******************************************************************************/
static uint32_t TD_SENSOR_GATEWAY_AppendDevicePrivate(TD_SENSOR_GATEWAY_Device_t *device)
{
	uint32_t address;
	int i = 0;

	if (DeviceCount > CONFIG_TD_SENSOR_GATEWAY_MAX_DEVICE) {
		return 255;
	}
	if (device->lan_address == 0) {

		// If device is already registered, give it back its previous address
		for (i = 0; i < CONFIG_TD_SENSOR_GATEWAY_MAX_DEVICE; i++) {
			if (device->sigfox_id == DeviceList[i].sigfox_id) {
				return DeviceList[i].lan_address;
			}
		}

		// Otherwise compute a new address
		address = TD_SENSOR_LAN_ComputeAddressTo8bits(device->sigfox_id);

		// Check if address is not already in use
		for (i = 1; i < DeviceCount; i++) {
			if (DeviceList[i].lan_address == address) {

				// Address 0 and 255 are reserved
				if (++address == 255) {
					address = 1;
				}

				// Re-scan all devices again
				i = 1;
			}
		}
		address |= (GatewayAddress & NETWORK_MASK);
	} else {
		address = device->lan_address;
	}

	// Affect address
	DeviceList[DeviceCount].lan_address = address ;
	DeviceList[DeviceCount].sigfox_id = device->sigfox_id;
	DeviceList[DeviceCount].device_class = device->device_class;
	DeviceCount++;
	return address;
}

/***************************************************************************//**
 * @brief
 *   Returns a device index in device list array according to a SIGFOX ID.
 *
 * @param[in] lan_address
 *   The device LAN address.
 *
 * @return
 *   Returns the index of the device in the device list array, or 255 if the
 *   SIGFOX ID was not found.
 ******************************************************************************/
static uint8_t TD_SENSOR_GATEWAY_GetDevice(uint8_t lan_address)
{
	int i;

	for (i = 1; i < DeviceCount; i++) {
		if ((DeviceList[i].lan_address & 0xFF) == lan_address) {
			return i;
		}
	}
	return 255;
}

/***************************************************************************//**
 * @brief
 *   Send an acknowledgment frame on LAN to a given device.
 *
 * @param[in] rx_frame
 * 	Frame to be acknowledged.
 *
 * @param[in] code
 *  Acknowledgment code to be replied.
 *
 * @param[in] data
 *  Optional data to be sent back. If a null pointer is provided, everything but
 *  the received data last byte is sent back.
 *
 * @param[in] count
 *  Length of the data in bytes.
 ******************************************************************************/
static void TD_SENSOR_GATEWAY_Ack(TD_LAN_frame_t *rx_frame, TD_SENSOR_LAN_AckCode_t code, uint8_t *data, uint8_t count)
{
	int i;
	TD_LAN_frame_t tx_frame;

	// Reply frame to sender with acknowledge flag set
	tx_frame.header = rx_frame->header;
	SET_ACK(tx_frame.header, TD_LAN_ACK);

	// First byte is acknowledgment code
	tx_frame.payload[0] = code;

	// If no data provided or too many bytes, then copy everything but the last
	// byte from received frame (header not included)
	if (data == 0 || count == 0 || count > TD_LAN_PAYLOAD_SIZE - 1) {
		for (i = 0; i < TD_LAN_PAYLOAD_SIZE - 1; i++) {
			tx_frame.payload[i + 1] = rx_frame->payload[i + 1];
		}
	} else {

		// Otherwise, just send the data
		for (i = 0; i < count; i++) {
			tx_frame.payload[i + 1] = data[i];
		}

		// Fill with 0s
		for (; i < TD_LAN_PAYLOAD_SIZE - 1; i++) {
			tx_frame.payload[i + 1] = 0;
		}
	}

	// Send the acknowlegment frame
	if (!TD_LAN_SendFrame(1, &tx_frame, rx_frame)) {
		DEBUG_PRINTF("LAN Error %d \r\n", TD_LAN_LastError());
	}
}

/***************************************************************************//**
 * @brief
 *   Send an RSSI event for a given device.
 *
 * @param[in] entry_id
 * 	The device entry ID to check for.
 ******************************************************************************/
static void TD_SENSOR_GATEWAY_CheckRSSI(uint8_t entry_id)
{
	int8_t rssi;
	uint8_t old_status;

	if (DeviceList[entry_id].config.rssi.monitor) {
		rssi = TD_LAN_ReadLatchedRSSI();
		old_status = DeviceList[entry_id].config.rssi.status;
		if (rssi > DeviceList[entry_id].config.rssi.level_ok) {
			DeviceList[entry_id].config.rssi.status = true;
		} else if (rssi < DeviceList[entry_id].config.rssi.level_low) {
			DeviceList[entry_id].config.rssi.status = false;
		}
		if (old_status != (uint8_t) DeviceList[entry_id].config.rssi.status) {
			if (DeviceList[entry_id].config.rssi.status) {
				TD_SENSOR_SendEventRSSI(1, entry_id);
			} else {
				TD_SENSOR_SendEventRSSI(0, entry_id);
			}
		}
	}
}

/***************************************************************************//**
 * @brief
 *  Process data from a keep-alive frame to configure RSSI/connection monitoring
 *  for the given deivce.
 *
 * @param[in] entry
 * 	The device entry ID to configure.
 *
 * @param[in] frame
 * 	Pointer to the keep-alive frame.
 ******************************************************************************/
static void TD_SENSOR_GATEWAY_ConfigureDeviceMonitoring(uint8_t entry, TD_SENSOR_LAN_KeepAliveFrame_t *frame)
{
	if (frame->keepalive == false && DeviceList[entry].config.keepalive.monitor == true) {
		DeviceList[entry].config.keepalive.monitor = false;
		TD_SCHEDULER_Remove(DeviceList[entry].config.keepalive.timer);
	} else if (frame->keepalive == true) {
		if (DeviceList[entry].config.keepalive.monitor == false) {

			// Add a keep-alive timer for this device
			DeviceList[entry].config.keepalive.timer = TD_SCHEDULER_Append(frame->interval,
				0, frame->interval / 2,
				0xFF,
				TD_SENSOR_GATEWAY_KeepAliveHandler,
				entry);
		} else {
			if (frame->interval != DeviceList[entry].config.keepalive.interval) {

				// Update an existing keep-alive timer
				TD_SCHEDULER_SetInterval(DeviceList[entry].config.keepalive.timer,
					frame->interval,
					0,
					frame->interval / 2);
			}
		}
		DeviceList[entry].config.keepalive.monitor = true;
		DeviceList[entry].config.keepalive.interval = frame->interval;
		if (!frame->rssi) {
			DeviceList[entry].config.rssi.monitor = false;
		} else {
			DeviceList[entry].config.rssi.monitor = true;
			DeviceList[entry].config.rssi.status = true;
			DeviceList[entry].config.rssi.level_low = frame->level_low;
			DeviceList[entry].config.rssi.level_ok = frame->level_ok;
		}
	}
}

/***************************************************************************//**
 * @brief
 *   Gateway received frame handler.
 *
 * @param[in] rx_frame
 *   Pointer to a buffer containing the eceived frame.
 *
 * @param[out] tx_frame
 *   Pointer to a buffer that will receive the frame to transmit.
 *
 * @return
 * 	Always returns 1;
 ******************************************************************************/
int TD_SENSOR_GATEWAY_FrameReceived(TD_LAN_frame_t *tx_frame, TD_LAN_frame_t *rx_frame)
{
	int i;
	TD_SENSOR_LAN_Frame_t frame;
	TD_SENSOR_LAN_ForwardFrame_t forward_frame;
	TD_SENSOR_TransmitProfile_t profile;
	uint8_t lan_address;
	uint8_t frame_count;
	uint8_t entry;
	TD_SENSOR_LAN_FrameType_t frame_type;
	uint8_t reply[15];
	int8_t reply_count;
	uint32_t new_lan_address;

	/* Acknowledgment reply is time-critical, adding traces may slow down a lot and
	 * may prevent acknoledgments from being received on time
	 */
	// We don't want to handle frame with acknowledge flag here
	if (GET_ACK(rx_frame->header)) {
		return 1;
	}

	//TODO: Improve to get it fine for different network
	// Get device address filtered with the network mask
	lan_address = GET_ADDRESS(gateway_RX.header) & ((~NETWORK_MASK));

	//TODO: THIS IS WRONG (about broadcast)
	// If address is 0, then it is a broadcast
	if (lan_address == 0) {
		entry = 0;
	} else {

		// Otherwise find corresponding entryID
		entry = TD_SENSOR_GATEWAY_GetDevice(lan_address);
	}
	DEBUG_PRINTF("E %d\r\n", entry)
	DEBUG_DUMP("RX: ", gateway_RX.payload, TD_LAN_PAYLOAD_SIZE);
	memcpy(&frame, gateway_RX.payload, TD_LAN_PAYLOAD_SIZE);
	frame_count = GET_LOCALSENSORFRAME_COUNT(frame.header);
	frame_type = (TD_SENSOR_LAN_FrameType_t) GET_LOCALSENSORFRAME_TYPE(frame.header);
	DEBUG_DUMP("Frame: ", &frame, 16);

	// Data count is on 4 bits but we need to handle 0-16 length
	if (frame_count == 0) {

		// If one of the byte is != 0, then  count = 16 otherwise count = 0
		for (i = 0; i < TD_LAN_PAYLOAD_SIZE - 1; i++) {
			if (frame.data[i] != 0) {
				frame_count = 16;
				break;
			}
		}
	}
	DEBUG_PRINTF("RXT %d %d\r\n", frame_type, frame_count);
	if (entry != 255) {

		// If device is found or frame is broadcast
		if (entry != 0) {

			// If device is registered
			if (entry < DeviceCount) {
				switch (frame_type) {
				case LOCAL_FORWARD: {
					TD_SENSOR_GATEWAY_Ack(&gateway_RX, ACK_OK, 0, 0);
					memcpy((uint8_t *)&forward_frame, frame.data, frame_count);
					profile.interval = GET_LOCALFORWARDFRAME_INTERVAL(forward_frame.profile);
					profile.repetition = GET_LOCALFORWARDFRAME_REPETITION(forward_frame.profile);
					DEBUG_DUMP("Forward: ", &forward_frame, frame_count);
					DEBUG_PRINTF(" \r\nProfile:%d %d\r\n", profile.interval, profile.repetition);

					//TODO: Replace SIGFOX frame and profile decoding!
					TD_SENSOR_TRANSMITTER_SendSigfox((TD_SENSOR_Frame_t *) forward_frame.sigfox,
						frame_count - sizeof (TD_SENSOR_TransmitProfile_t),
						entry,
						&profile);
				}
				break;

				case LOCAL_KEEPALIVE: {
					TD_SENSOR_LAN_KeepAliveFrame_t *keepalive_frame;
					TD_SENSOR_GATEWAY_Ack(&gateway_RX, ACK_OK, 0, 0);
					keepalive_frame = (TD_SENSOR_LAN_KeepAliveFrame_t *)(frame.data);
					TD_SENSOR_GATEWAY_ConfigureDeviceMonitoring(entry, keepalive_frame);
					if (DeviceList[entry].config.keepalive.monitor) {

						// If keep-alive status indicates a connection lost (battery was replaced
						// for example), then we need to resynchronize, i.e. restart the timer
						// on keep-alive reception with very same parameters
						if (DeviceList[entry].config.keepalive.status == false) {
							TD_SCHEDULER_SetInterval(DeviceList[entry].config.keepalive.timer,
								DeviceList[entry].config.keepalive.interval,
								0,
								DeviceList[entry].config.keepalive.interval / 2);
							TD_SENSOR_SendEventConnection(1, entry);
							DeviceList[entry].config.keepalive.status = true;
						}
						TD_SENSOR_GATEWAY_Ack(&gateway_RX, ACK_OK, 0, 0);
						DeviceList[entry].config.keepalive.validated = true;
					} else {
						TD_SENSOR_GATEWAY_Ack(&gateway_RX, ACK_ERROR, 0, 0);
					}
				}
				break;

				case LOCAL_REGISTER:
					TD_SENSOR_GATEWAY_Ack(&gateway_RX, ACK_ERROR, 0, 0);
					break;

				case LOCAL_DATA:
					if (DataCallback != 0) {
						reply_count = (*DataCallback)(frame.data, frame_count, reply);
						if (reply_count > 0) {

							// Custom acknowledgment
							TD_SENSOR_GATEWAY_Ack(&gateway_RX, ACK_OK, reply, reply_count);
						} else if (reply_count == 0) {

							// Simple-copy acknowledgment
							TD_SENSOR_GATEWAY_Ack(&gateway_RX, ACK_OK, frame.data, frame_count);
						} else {

							// No acknowledgment
						}
					} else {

						// Simple-copy acknowledgment
						TD_SENSOR_GATEWAY_Ack(&gateway_RX, ACK_OK, frame.data, frame_count);
					}
					break;

				}
				TD_SENSOR_GATEWAY_CheckRSSI(entry);
			}
		} else if (frame_count > 0) {

			// Only a LAN registration frame is allowed if a device is not already paired
			if (frame_type == LOCAL_REGISTER) {
				if (RegistrationEnabled) {
					TD_SENSOR_GATEWAY_Device_t new_device;
					TD_SENSOR_LAN_RegisterFrame_t *reg_frame;
					TD_SENSOR_LAN_KeepAliveFrame_t *keep_frame;
					reg_frame = (TD_SENSOR_LAN_RegisterFrame_t *) (frame.data);
					keep_frame = (TD_SENSOR_LAN_KeepAliveFrame_t *)
						(&frame.data[sizeof (TD_SENSOR_LAN_RegisterFrame_t)]);
					new_device.sigfox_id = reg_frame->SigfoxID;
					new_device.device_class = reg_frame->device_class;
					new_device.lan_address = 0;

					// Get an address for the device
					new_lan_address = TD_SENSOR_GATEWAY_AppendDevicePrivate(&new_device);
					if (new_lan_address == 255) {

						// If no address could be assigned to the device
						TD_SENSOR_GATEWAY_Ack(&gateway_RX, ACK_ERROR, 0, 0);
					} else {
						// If an address could be given to the device
						TD_SENSOR_LAN_Address_t newAdress;
						newAdress.address = new_lan_address;
						newAdress.mask = NETWORK_MASK;
						TD_SENSOR_GATEWAY_Ack(&gateway_RX, ACK_OK, (uint8_t *) &newAdress, 8);
						TD_SENSOR_GATEWAY_ConfigureDeviceMonitoring(entry, keep_frame);
						if (RegistrationCallback != 0) {
							(*RegistrationCallback)(newAdress.address, new_device.sigfox_id);
						}

						// Save configuration changes in Flash memory
						TD_FLASH_WriteVariables();
					}
				}
			}
		}
	}

	// Restart reception if it was not stopped within function
	if (ReceptionEnabled) {
		TD_SENSOR_GATEWAY_StartReception();
	}
	return 1;
}

/** @} */

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_GATEWAY_GLOBAL_FUNCTIONS Global Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *  Gateway Initialization.
 *
 * @return
 *  Returns false if LAN initialization failed, true otherwise.
 ******************************************************************************/
bool TD_SENSOR_GATEWAY_Init(void)
{
	int i;

	GatewayAddress = (TD_SENSOR_LAN_ComputeAddressTo16bits(SigfoxID)) << 8;
	if (!TD_FLASH_DeclareVariable((uint8_t *) &DeviceCount, sizeof (uint32_t), 0)) {
		DeviceCount = 1;
	}
	if (!TD_FLASH_DeclareVariable((uint8_t *) DeviceList,
		sizeof (TD_SENSOR_GATEWAY_Device_t) * CONFIG_TD_SENSOR_GATEWAY_MAX_DEVICE,
		0)) {
		DeviceList[0].sigfox_id = SigfoxID;
		DeviceList[0].lan_address = GatewayAddress;
		for (i = 1; i < CONFIG_TD_SENSOR_GATEWAY_MAX_DEVICE; i++) {
			DeviceList[i].sigfox_id = 0;
		}
	}

	// Reset keep-alive and RSSI for all devices
	for (i = 1; i < CONFIG_TD_SENSOR_GATEWAY_MAX_DEVICE; i++) {
		DeviceList[i].config.keepalive.monitor = false;
		DeviceList[i].config.keepalive.status = false;
		DeviceList[i].config.rssi.status = false;
		DeviceList[i].config.rssi.monitor = false;
	}
	return TD_SENSOR_LAN_setLanAddress(GatewayAddress, NETWORK_MASK);
}

/** @} */

/** @addtogroup TD_SENSOR_GATEWAY_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *  Send a data frame to a specific device given by its address.
 *
 * @param[in] address
 *  The address to which the frame should be sent.
 *
 * @param[in] data
 *  Pointer to the buffer containing the data to be sent.
 *
 * @param[in] length
 *  Size in bytes of data to be sent.
 *
 * @param[out] data_rx
 *  Pointer to a buffer that will received the reply.
 ******************************************************************************/
TD_SENSOR_LAN_AckCode_t TD_SENSOR_GATEWAY_SendDataByAddress(uint32_t address, uint8_t *data, uint8_t length, uint8_t *data_rx)
{
	return TD_SENSOR_LAN_SendFrameTo(address, LOCAL_DATA, data, length, data_rx);
}

/***************************************************************************//**
 * @brief
 *  Broadcast a data frame to all devices.
 *
 * @param[in] data
 *  Pointer to the buffer containing the data to be broadcasted.
 *
 * @param length
 *  Size in bytes of data to be sent.
 ******************************************************************************/
void TD_SENSOR_GATEWAY_SendDataBroadcast(uint8_t *data, uint8_t length)
{
	uint8_t temp = TD_SENSOR_LAN_GetFrameRetry();

	// No acknowledgment as this is a broadcast
	TD_SENSOR_LAN_SetFrameRetry(0);
	TD_SENSOR_LAN_SendFrameTo(GatewayAddress | 0xFF, LOCAL_DATA, data, length, 0);
	TD_SENSOR_LAN_SetFrameRetry(temp);
}

/***************************************************************************//**
 * @brief
 *  Broadcast a keep-alive frame to all devices.
 *
 * @param[in] enabled
 *  Reserved for future use.
 *
 * @param[in] interval
 *  Interval in seconds at which the keep-alive frame is sent.
 *
 * @param[in] data
 *   Reserved for future use.
 *
 * @param[in] length
 *   Reserved for future use.
 ******************************************************************************/
void TD_SENSOR_GATEWAY_SendKeepAliveBroadcast(bool enabled, uint8_t interval, uint8_t *data, uint8_t length)
{
	TD_SENSOR_LAN_KeepAliveFrame_t frame;
	uint8_t temp = TD_SENSOR_LAN_GetFrameRetry();

	// No acknowledgment as this is abroadcast
	TD_SENSOR_LAN_SetFrameRetry(0);
	frame.keepalive = true;
	frame.interval = interval;
	frame.rssi = true;
	frame.level_low = 0;
	frame.level_ok = 0;
	TD_SENSOR_LAN_SendFrameTo(GatewayAddress | 0xFF,
		LOCAL_KEEPALIVE,
		(uint8_t *) &frame,
		sizeof (TD_SENSOR_LAN_KeepAliveFrame_t),
		0);
	TD_SENSOR_LAN_SetFrameRetry(temp);
}


/***************************************************************************//**
 * @brief
 *  Get a pointer to the Device list.
 *
 * @return
 *  Returns a pointer to the device list.
 ******************************************************************************/
const TD_SENSOR_GATEWAY_Device_t *TD_SENSOR_GATEWAY_GetDeviceList(void)
{
	return DeviceList;
}

/***************************************************************************//**
 * @brief
 *  Get the count of registered devices.
 *
 * @return
 *  Returns the count of currently registered devices.
 ******************************************************************************/
uint8_t TD_SENSOR_GATEWAY_GetDeviceCount(void)
{

	// Do not count in the gateway itself
	return DeviceCount - 1;
}

/***************************************************************************//**
 * @brief
 *  Delete all registered devices.
 ******************************************************************************/
void TD_SENSOR_GATEWAY_DeleteAllDevices(void)
{
	int i;

	for (i = 1; i < CONFIG_TD_SENSOR_GATEWAY_MAX_DEVICE; i++) {
		if (DeviceList[i].config.keepalive.monitor) {
			TD_SCHEDULER_Remove(DeviceList[i].config.keepalive.timer);
		}
		DeviceList[i].sigfox_id = 0;
		DeviceList[i].config.keepalive.monitor = 0;
		DeviceList[i].config.rssi.monitor = 0;
		DeviceList[i].type = 0;
	}

	// The only device left is the gateway itself
	DeviceCount = 1;
}

/***************************************************************************//**
 * @brief
 *  Delete one registered device from the device list.
 *
 * @param [in] lan_address
 *  The device ALN address to delete.
 ******************************************************************************/
void TD_SENSOR_GATEWAY_DeleteDevice(uint32_t lan_address)
{
	int i;
	uint8_t entry;

	entry = TD_SENSOR_GATEWAY_GetDevice(lan_address);

	// Stop device keep-alive timer if any
	if (DeviceList[entry].config.keepalive.monitor) {
		TD_SCHEDULER_Remove(DeviceList[entry].config.keepalive.timer);
	}

	//TODO: replace with memcpy
	if (entry < DeviceCount) {
		for (i = entry; i < DeviceCount; i++) {
			DeviceList[i].sigfox_id = DeviceList[i + 1].sigfox_id;
			DeviceList[i].lan_address = DeviceList[i + 1].lan_address;
			DeviceList[i].type = DeviceList[i + 1].type;
			DeviceList[i].config.keepalive = DeviceList[i + 1].config.keepalive;
			DeviceList[i].config.rssi = DeviceList[i + 1].config.rssi;
			DeviceList[i].device_class = DeviceList[i + 1].device_class;
		}
		DeviceList[DeviceCount].sigfox_id = 0;
		DeviceCount--;
	}
}

/***************************************************************************//**
 * @brief
 *  Set a callback on Local Data Frame Reception
 *
 * @param[in] user_data_callback
 *   Pointer to a callback function that will be called upon local data frame
 *   reception with the following arguments:
 *     - data ill be a pointer to a buffer containing the received payload
 *     - length will be the size in bytes of the received payload
 *     - reply will be a pointer to the buffer to be filled with reply if used
 *   The callback function should return the reply length which must be <=15, or
 *   < 0 if no acknowledgment is required.
 *   This function must return as quickly as possible in the case where an
 *   acknowledgment should be sent.
 ******************************************************************************/
void TD_SENSOR_GATEWAY_SetDataCallback(int8_t (*user_data_callback)(uint8_t *data, uint8_t length, uint8_t *reply))
{
	DataCallback = user_data_callback;
}

/***************************************************************************//**
 * @brief
 *  Start listening for local devices.
 ******************************************************************************/
void TD_SENSOR_GATEWAY_StartReception(void)
{
	ReceptionEnabled = true;
	TD_LAN_SetUserCallback(TD_SENSOR_GATEWAY_FrameReceived_);
	//tfp_printf("TD_SENSOR_GATEWAY_StartReception:0x%08X\r\n",&gateway_RX);
	TD_LAN_ReceiveFrame(CONFIG_LAN_PERIOD, 0, &gateway_RX);
}

/***************************************************************************//**
 * @brief
 *  Stop listening for local devices.
 ******************************************************************************/
void TD_SENSOR_GATEWAY_StopReception(void)
{
	ReceptionEnabled = false;
	TD_LAN_Abort();
}

/***************************************************************************//**
 * @brief
 *  Get the current reception state.
 *
 * @return
 *  Returns true if reception enabled. false otherwise.
 ******************************************************************************/
bool TD_SENSOR_GATEWAY_IsReceptionEnabled(void)
{
	return ReceptionEnabled;
}

/***************************************************************************//**
 * @brief
 *   Start Device registration.
 *
 * @param[in] callback
 *   Pointer to the user callback function which will be called when a
 *   registration occurs, with the following arguments:
 *     - lan_address will be the LAN address of the newly registered device
 *     - sigfox_id will be the corresponding SIGFOX ID of he device
 ******************************************************************************/
void TD_SENSOR_GATEWAY_StartRegistration(void (*callback)(uint32_t lan_address, uint32_t sigfox_id))
{
	bool rx_en = ReceptionEnabled;

	// Stop reception if enabled
	if (rx_en) {
		ReceptionEnByReg = false;
		TD_SENSOR_GATEWAY_StopReception();
	} else {
		ReceptionEnByReg = true;
	}
	RegistrationCallback = callback;
	TD_SENSOR_LAN_setLanAddress(BROADCAST_ADDRESS, BROADCAST_MASK);
	RegistrationEnabled = true;

	// Always start reception
	TD_SENSOR_GATEWAY_StartReception();
}

/***************************************************************************//**
 * @brief
 *  Stop device registration.
 ******************************************************************************/
void TD_SENSOR_GATEWAY_StopRegistration(void)
{
	TD_SENSOR_GATEWAY_StopReception();
	RegistrationEnabled = false;
	RegistrationCallback = 0;
	TD_SENSOR_LAN_setLanAddress(GatewayAddress, NETWORK_MASK);

	// If reception was not started by registration, restart it
	if (!ReceptionEnByReg) {
		TD_SENSOR_GATEWAY_StartReception();
	}
}

/***************************************************************************//**
 * @brief
 *   Manually append a device.
 *
 * @param[in] sigfox_id
 *   The SIGFOX ID for the device to add.
 *
 * @param[in] class
 *   The class to use for the device to add.
 *
 * @param[in] lan_address
 *   The LAN address to assign to the device to add. Set to zero to generate an
 *   address according to the SIGFOX ID, or set it to 1..15 to force a device
 *   address.
 *
 * @return
 *   Returns an entry ID if the device has been added, 255 otherwise.
 ******************************************************************************/
uint32_t TD_SENSOR_GATEWAY_AppendDevice(uint32_t sigfox_id, uint16_t class, uint32_t lan_address)
{
	TD_SENSOR_GATEWAY_Device_t device;
	device.sigfox_id = sigfox_id;
	device.device_class = class;
	device.lan_address = lan_address;
	return (TD_SENSOR_GATEWAY_AppendDevicePrivate(&device) | (GatewayAddress & NETWORK_MASK));
}

/** @} */

/** @} (end addtogroup TD_SENSOR_GATEWAY) */
