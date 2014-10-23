/***************************************************************************//**
 * @file
 * @brief Sensor Gateway
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

#ifndef __TD_SENSOR_GATEWAY_H
#define __TD_SENSOR_GATEWAY_H

#include <td_trap.h>
#include "td_lan.h"
#include "td_sensor_lan.h"
#include "td_sensor_transmitter.h"

#ifdef __cplusplus
extern "C" {
#endif

	/***************************************************************************//**
	 * @addtogroup TD_SENSOR_GATEWAY Sensor LAN Gateway
	 * @{
	 ******************************************************************************/

	/*******************************************************************************
	 **************************  TYPEDEFS   ****************************************
	 ******************************************************************************/

	/** @addtogroup TD_SENSOR_GATEWAY_TYPEDEFS Typedefs
	 * @{ */

	/** Sensor Device type */
	typedef uint16_t TD_SENSOR_DeviceType_t;

	/** Distant device keep-alive configuration */
	typedef struct {
		uint32_t interval : 32;	///< Keep alive checking interval
		uint8_t timer : 8;		///< Keep alive timer id
		bool monitor : 1;		///< Monitoring enabled?
		bool status : 1;		///< Keep alive status (lost/OK)
		bool validated : 1;		///< Keep alive validated?
	} __PACKED TD_SENSOR_GATEWAY_KeepaliveConfig_t;

	/** Distant RSSI keep-alive configuration */
	typedef struct {
		uint8_t level_ok : 8;	///< RSSI level OK
		uint8_t level_low : 8;	///< RSSI level low
		bool monitor : 1;		///< Monitoring enabled?
		bool status : 1;		///< Current status (low/OK)
	} __PACKED TD_SENSOR_GATEWAY_RSSIConfig_t;

	/** Distant device configuration */
	typedef struct {
		TD_SENSOR_GATEWAY_RSSIConfig_t rssi;				///< RSSI configuration
		TD_SENSOR_GATEWAY_KeepaliveConfig_t keepalive;		///< Keepalive configuration
	} __PACKED TD_SENSOR_GATEWAY_DeviceConfig_t;

	/** Distant device data */
	typedef struct {
		TD_SENSOR_GATEWAY_DeviceConfig_t config;
		uint32_t sigfox_id : 32; 							///< Device Sigfox ID
		uint32_t lan_address : 24; 							///< Device Lan Address
		TD_SENSOR_DeviceType_t type : 8; 					///< Device Type
		uint16_t device_class : 16; 						///< Device class
	} __PACKED TD_SENSOR_GATEWAY_Device_t;

	/** @} */

	/*******************************************************************************
	 *************************   PROTOTYPES   **************************************
	 ******************************************************************************/

	/** @addtogroup TD_SENSOR_GATEWAY_GLOBAL_FUNCTIONS Global Functions
	 * @{ */

	bool TD_SENSOR_GATEWAY_Init(void);
	int TD_SENSOR_GATEWAY_FrameReceived(TD_LAN_frame_t *tx_frame, TD_LAN_frame_t *rx_frame);

	/** @} */

	/** @addtogroup TD_SENSOR_GATEWAY_USER_FUNCTIONS User Functions
	 * @{ */

	void TD_SENSOR_GATEWAY_StartRegistration(void (*callback)(uint32_t lan_address, uint32_t sigfox_id));
	void TD_SENSOR_GATEWAY_StopRegistration(void);
	void TD_SENSOR_GATEWAY_StartReception(void);
	void TD_SENSOR_GATEWAY_StopReception(void);
	bool TD_SENSOR_GATEWAY_IsReceptionEnabled(void);
	uint32_t TD_SENSOR_GATEWAY_AppendDevice(uint32_t sigfox_id, uint16_t class, uint32_t lan_address);
	void TD_SENSOR_GATEWAY_DeleteAllDevices(void);
	void TD_SENSOR_GATEWAY_DeleteDevice(uint32_t lan_address);
	TD_SENSOR_LAN_AckCode_t TD_SENSOR_GATEWAY_SendDataByAddress(uint32_t address, uint8_t *data, uint8_t length, uint8_t *data_rx);
	void TD_SENSOR_GATEWAY_SendDataBroadcast(uint8_t *data, uint8_t length);
	void TD_SENSOR_GATEWAY_SendKeepAliveBroadcast(bool enabled, uint8_t interval, uint8_t *data, uint8_t length);
	void TD_SENSOR_GATEWAY_SetDataCallback(int8_t (*user_data_callback)(uint8_t *data, uint8_t length, uint8_t *reply));
	uint8_t TD_SENSOR_GATEWAY_GetDeviceCount(void);
	const TD_SENSOR_GATEWAY_Device_t *TD_SENSOR_GATEWAY_GetDeviceList(void);

	/** @} */

	/** @} (end addtogroup TD_SENSOR_GATEWAY) */

#ifdef __cplusplus
}
#endif

#endif // __TD_SENSOR_GATEWAY_H
