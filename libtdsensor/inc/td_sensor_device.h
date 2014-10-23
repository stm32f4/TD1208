/***************************************************************************//**
 * @file
 * @brief Sensor LAN Device
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

#ifndef __TD_SENSOR_DEVICE_H
#define __TD_SENSOR_DEVICE_H

#include "sensor_private.h"
#include "td_sensor_lan.h"
#include "sensor_send.h"

#ifdef __cplusplus
extern "C" {
#endif

	/***************************************************************************//**
	 * @addtogroup TD_SENSOR_DEVICE Sensor LAN Device
	 * @brief
	 *	Devices LAN functions to Register, Forward Sensor Frames, send custom Data
	 *	and send Keep-alive to the gateway.
	 *
	 * @{
	 ******************************************************************************/

	/*******************************************************************************
	 **************************  DEFINES   ****************************************
	 ******************************************************************************/

	/** @addtogroup TD_SENSOR_DEVICE_DEFINES Defines
	 * @{ */
#define LOCAL_REGISTER_CHECK 0x37
	/** @} */

	/*******************************************************************************
	 *************************   PROTOTYPES   **************************************
	 ******************************************************************************/

	/** @addtogroup TD_SENSOR_DEVICE_GLOBAL_FUNCTIONS Global Functions
	 * @{ */

	TD_SENSOR_LAN_AckCode_t TD_SENSOR_DEVICE_Forward(uint8_t *payload, uint8_t count, uint8_t repetition, uint32_t interval);
	TD_SENSOR_LAN_AckCode_t TD_SENSOR_DEVICE_ForwardAsynch(uint8_t *payload, uint8_t count, uint8_t repetition, uint32_t interval);
	TD_SENSOR_LAN_AckCode_t TD_SENSOR_DEVICE_KeepAlive(bool keepalive, uint32_t interval, bool rssi, int8_t level_low, int8_t level_ok);
	void TD_SENSOR_DEVICE_Process(void);

	/** @} */

	/** @addtogroup TD_SENSOR_DEVICE_USER_FUNCTIONS User Functions
	 * @{ */

	TD_SENSOR_LAN_AckCode_t TD_SENSOR_DEVICE_Data(uint8_t *data, uint8_t count, uint8_t *data_rx);
	TD_SENSOR_LAN_AckCode_t TD_SENSOR_DEVICE_Register(void);
	bool TD_SENSOR_DEVICE_isRegistered(void);
	void TD_SENSOR_DEVICE_Reset(void);
	void TD_SENSOR_DEVICE_SetDataCallback(int8_t (*user_data_callback)(bool broadcast, uint32_t address, uint8_t *data, uint8_t len, uint8_t *reply));
	void TD_SENSOR_DEVICE_SetKeepAliveCallback(int8_t (*user_keepalive_callback)(bool broadcast, uint32_t address, uint32_t interval, int8_t rssi, uint8_t *data, uint8_t len, uint8_t *reply));
	void TD_SENSOR_DEVICE_StartReception(void);
	void TD_SENSOR_DEVICE_StartBroadcastReception(uint32_t mask);
	bool TD_SENSOR_DEVICE_StartSynchReception(void);
	void TD_SENSOR_DEVICE_StopReception(void);
	bool TD_SENSOR_DEVICE_isReceptionEnabled(void);
	bool TD_SENSOR_DEVICE_isBroadcastReceptionEnabled(void);
	void TD_SENSOR_DEVICE_SetTxOnLanFail(bool enable);
	bool TD_SENSOR_DEVICE_GetTxOnLanFail();
	void TD_SENSOR_DEVICE_SetTxSkipLan(bool enable);
	bool TD_SENSOR_DEVICE_GetTxSkipLan(void);
	uint32_t TD_SENSOR_DEVICE_GetAddress(void);
	bool TD_SENSOR_DEVICE_SetAddress(uint32_t address, uint32_t mask);
	uint32_t TD_SENSOR_DEVICE_GetBroadcastMask(void);
	void TD_SENSOR_DEVICE_SetAsynchronousForward(bool asynch);
	bool TD_SENSOR_DEVICE_IsAsynchronousForward(void);

	/** @} */

	/** @} (end addtogroup TD_SENSOR_DEVICE) */

#ifdef __cplusplus
}
#endif

#endif // __TD_SENSOR_DEVICE_H
