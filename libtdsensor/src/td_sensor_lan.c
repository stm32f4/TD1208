/***************************************************************************//**
 * @file
 * @brief Sensor LAN
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

#include <td_lan.h>
#include <td_flash.h>
#include <td_trap.h>

#include "sensor_private.h"
#include "td_sensor.h"
#include "td_sensor_gateway.h"
#include "td_sensor_device.h"
#include "td_sensor_lan.h"

/***************************************************************************//**
 * @addtogroup TD_SENSOR_LAN Sensor LAN
 * @brief LAN protocol management
 * @{
 ******************************************************************************/

/*******************************************************************************
 **************************  DEFINES   *******************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_LAN_DEFINES Defines
 * @{ */

//#define SENSOR_DEBUG

#ifdef SENSOR_DEBUG

/** Conditional printf macro */
#define DEBUG_PRINTF(...)	tfp_printf(__VA_ARGS__)

#else

/** Conditional printf macro */
#define DEBUG_PRINTF(...)

#endif

/** @} */

/*******************************************************************************
 **************************  PRIVATE VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_LAN_LOCAL_VARIABLES Local Variables
 * @{ */

/** Current LAN address and mask */
static TD_SENSOR_LAN_Address_t LanConfig;

/** Retries on LAN */
static uint8_t FrameRetry = 0;

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_LAN_LOCAL_FUNCTIONS Local Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *  Send a LAN frame.
 *
 * @param[in] data
 *  Pointer to a buffer containing the data to be sent.
 *
 * @param[out] data_rx
 *  Pointer to a buffer that will receive the reply.
 *
 * @param[in] address
 *  The address of the device to send data to.
 *
 * @return
 *   Returns the acknowledge result.
 ******************************************************************************/
static TD_SENSOR_LAN_AckCode_t TD_SENSOR_LAN_SendFramePrivate(uint8_t *data, uint8_t *data_rx, uint32_t address)
{
	TD_LAN_frame_t TX, RX;
	TD_SENSOR_ModuleType_t type;
	bool rx_enabled = false;
	bool rx_broad_enabled = false;
	uint32_t broad_mask = 0;
	int i;
	TD_SENSOR_LAN_AckCode_t ret;

	TX.header = 0;
	SET_ADDRESS(TX.header, address);
	memcpy(TX.payload, data, TD_LAN_PAYLOAD_SIZE);
	type = TD_SENSOR_GetModuleType();
	if (type == SENSOR_GATEWAY) {
		if (TD_SENSOR_GATEWAY_IsReceptionEnabled()) {
			rx_enabled = true;
			DEBUG_PRINTF("I'm a GATEWAY and I stop reception\r\n");
			TD_SENSOR_GATEWAY_StopReception();
		}
	} else if (type == SENSOR_DEVICE) {
		if (TD_SENSOR_DEVICE_isReceptionEnabled()) {
			rx_enabled = true;
			if (TD_SENSOR_DEVICE_isBroadcastReceptionEnabled()) {
				rx_broad_enabled = true;
				broad_mask = TD_SENSOR_DEVICE_GetBroadcastMask();
			}
			DEBUG_PRINTF("I'm a DEVICE and I stop reception\r\n");
			TD_SENSOR_DEVICE_StopReception();
		}
		if (address != LanConfig.address) {
			TD_SENSOR_LAN_setLanAddress(address, 0xFFFFFF);
		}
	}
	DEBUG_PRINTF("do SendReceive\r\n");
	if (!TD_LAN_SendReceive(-1, 1 + FrameRetry, &TX, &RX)) {
		switch (TD_LAN_LastError()) {
		case ERROR_NONE:

			// No error

		case ERROR_RECEIVE_TIMEOUT:

			// Receive has timeout (no message available)

		case ERROR_USER_ABORT:
			ret = NOT_ACKED;
			break;

		case ERROR_LAN_BUSY:

			// LAN is busy (RSSI detected over timeout period)
			ret = LAN_CHANNEL_BUSY;
			break;

		case ERROR_CANT_SWAP_RADIO:

			// After a radio priority change, can't restore LAN radio state

		case ERROR_TRANSMIT:

			// Transmit error (no IRQ received)

		case ERROR_SYNC_TIMEOUT:

			// User timeout during synchronous operation

		case ERROR_CANT_START_RECEIVE:

			// A start receive failed. Radio chip probably messed up

		default:
			ret = LAN_ERROR;

			//TODO: restart radio?
			break;
		}
	} else {
		if (data_rx != 0) {
			// Don't copy acknowledgment code
			for (i = 0; i < TD_SENSOR_LAN_PAYLOAD_SIZE - 1; i++) {
				data_rx[i] = RX.payload[i + 1];
			}
		}
		ret = (TD_SENSOR_LAN_AckCode_t) RX.payload[0];
	}
	if (type == SENSOR_GATEWAY && rx_enabled) {
		DEBUG_PRINTF("I'm a GATEWAY and I start reception\r\n");
		TD_SENSOR_GATEWAY_StartReception();
	} else if (type == SENSOR_DEVICE) {
		if (rx_broad_enabled) {
			DEBUG_PRINTF("I'm a DEVICE and I start BROADCAST reception\r\n");
			TD_SENSOR_DEVICE_StartBroadcastReception(broad_mask);
		} else if (rx_enabled) {
			DEBUG_PRINTF("I'm a DEVICE and I start reception\r\n");
			TD_SENSOR_DEVICE_StartReception();
		}
	}
	return ret;
}

/***************************************************************************//**
 * @brief
 *  Define the current module as being a gateway.
 *
 * @return
 *   Returns true if the gateway initialized correctly, false otherwise.
 ******************************************************************************/
static bool TD_SENSOR_LAN_DefineAsGateway(void)
{
	return TD_SENSOR_GATEWAY_Init();
}

/***************************************************************************//**
 * @brief
 *  Define the current module as being a device.
 *
 * @return
 *   Returns true if the device initialized correctly, false otherwise.
 ******************************************************************************/
static bool TD_SENSOR_LAN_DefineAsDevice(void)
{
	return TD_SENSOR_DEVICE_SetAddress(LanConfig.address, LanConfig.mask);
}

/** @} */

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_LAN_GLOBAL_FUNCTIONS Global Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *  Set the LAN ddress.
 *
 * @param[in] address
 *   THeLAN address to set.
 *
 * @param[in] mask
 *  The LAN mask to set.
 *
 * @return
 *  Returns true if the operation was successful, false otherwise.
 ******************************************************************************/
bool TD_SENSOR_LAN_setLanAddress(uint32_t address, uint32_t mask)
{
	LanConfig.address = address;
	LanConfig.mask = mask;

	return TD_LAN_Init(false, LanConfig.address, LanConfig.mask);
}

/***************************************************************************//**
 * @brief
 *  Set the retries count for LAN transmissions. If a frame is no  acknowledged,
 *  it will be sent again up to the given retry count.
 *
 * @param[in] retry
 *  Number of retries to perform.
 ******************************************************************************/
void TD_SENSOR_LAN_SetFrameRetry(uint8_t retry)
{
	FrameRetry = retry;
}

/***************************************************************************//**
 * @brief
 *  Get the current retry count for LAN transmissions.
 *
 * @return
 *  Returns the current retry count for LAN transmissions.
 ******************************************************************************/
uint8_t TD_SENSOR_LAN_GetFrameRetry(void)
{
	return FrameRetry;
}

/***************************************************************************//**
 * @brief
 *   Send a Frame on the LAN according to the TD protocol.
 *
 * @param[in] frame
 *   Pointer to a buffer containing the data to be sent.
 *
 * @param[out] data_rx
 *   Pointer to a buffer that will receive the reply.
 *
 * @return
 *   Returns the acknowledge result.
 ******************************************************************************/
TD_SENSOR_LAN_AckCode_t TD_SENSOR_LAN_SendFrame(TD_SENSOR_LAN_Frame_t *frame, uint8_t *data_rx)
{
	TD_SENSOR_ModuleType_t module_type;
	uint32_t address;

	module_type = TD_SENSOR_GetModuleType();
	if (module_type == SENSOR_GATEWAY) {
		address = LanConfig.address;
	} else if (module_type == SENSOR_DEVICE) {
		address = TD_SENSOR_DEVICE_GetAddress();
	} else {
		address = 0;

		//TODO: replace by trap
		//DEBUG_PRINTF("TD_SENSOR_LAN_SendFrame Module type error %d\r\n",module_type);
	}
	return TD_SENSOR_LAN_SendFramePrivate((uint8_t *) frame, data_rx, address);
}

/***************************************************************************//**
 * @brief
 *   Send a Frame on the LAN to a specific address according to the TD protocol
 *
 * @param[in] address
 *   LAN address of the device to send to.
 *
 * @param[in] type
 *   Type of frame to be sent.
 *
 * @param[in] payload
 *   Pointer to a buffer containing the data to be sent.
 *
 * @param[in] count
 *   Data length in bytes.
 *
 * @param[out] data_rx
 *   Pointer to a buffer that will receive the reply.
 *
 * @return
 *   Returns the acknowledge result.
 ******************************************************************************/
TD_SENSOR_LAN_AckCode_t TD_SENSOR_LAN_SendFrameTo(uint32_t address, TD_SENSOR_LAN_FrameType_t type, uint8_t *payload, uint8_t count, uint8_t *data_rx)
{
	TD_SENSOR_LAN_Frame_t frame;

	frame.header = 0;
	SET_LOCALSENSORFRAME_TYPE(frame.header, type);
	SET_LOCALSENSORFRAME_COUNT(frame.header, count);
	if (count + 1 > (TD_LAN_PAYLOAD_SIZE)) {
		return NOT_SENT;
	}
	memcpy(frame.data, payload, count);
	return TD_SENSOR_LAN_SendFramePrivate((uint8_t *) &frame, data_rx, address);
}

/***************************************************************************//**
 * @brief
 *   Stop the Sensor LAN.
 *******************************************************************************/
void TD_SENSOR_LAN_Stop(void)
{
	TD_LAN_Release();
}

/***************************************************************************//**
 * @brief
 *  Initialize the Sensor LAN. Lookup into Flash memory for the LAN address to
 *  use, otherwise use broadcast.
 *
 * @param[in] gateway
 *  If set to true, initialize the LAN node as a gateway, if set to false,
 *  initialize the LAN node as a device.
 *
 * @param[in] lan_frequency
 *	The LAN frequency in Hz. Must be in the range 868000000..869700000 Hz.
 *
 * @param[in] lan_power_level
 *	The LAN transmit power level in dBm, maximum is 14 dBm.
 *
 * @return
 *  Returns true if the operation was successful, false otherwise.
 ******************************************************************************/
bool TD_SENSOR_LAN_Init(bool gateway, uint32_t lan_frequency, int16_t lan_power_level)
{
	if (!TD_FLASH_DeclareVariable((uint8_t *) &LanConfig, sizeof (TD_SENSOR_LAN_Address_t), 0)) {

		// Cannot find our LAN configuration from Flash memory, use broadcast
		LanConfig.address = BROADCAST_ADDRESS;
		LanConfig.mask = BROADCAST_MASK;
	}

	// Apply frequency and power configuration
	if (TD_LAN_SetFrequencyLevel(lan_frequency, lan_power_level) == false) {
		TD_Trap(TRAP_ILLEGAL_FREQ, lan_frequency);
		return false;
	}

	// First initialize with default address
	if (!TD_LAN_Init(true, 0, 0)) {
		return false;
	}

	// Then perform node-specific initialization
	if (gateway) {
		return TD_SENSOR_LAN_DefineAsGateway();
	} else {
		return TD_SENSOR_LAN_DefineAsDevice();
	}
}

/** @} */

/** @addtogroup TD_SENSOR_LAN_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *  Returns the current LAN address.
 *
 * @return
 *  Returns a pointer to a Lan_Adress structure.
 ******************************************************************************/
const TD_SENSOR_LAN_Address_t *TD_SENSOR_LAN_GetAddress(void)
{
	return &LanConfig;
}

/** @} */

/** @} (end addtogroup TD_SENSOR_LAN) */
