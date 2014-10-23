/***************************************************************************//**
 * @file
 * @brief Sensor Monitoring
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

#ifndef __TD_SENSOR_H
#define __TD_SENSOR_H

#include <stdbool.h>
#include <stdint.h>
#include <em_gpio.h>
#include "sensor_private.h"
#include "sensor_event.h"
#include "sensor_keepalive.h"

#include "td_sensor_transmitter.h"
#include "td_sensor_gateway.h"

#ifdef __cplusplus
extern "C" {
#endif

	/***************************************************************************//**
	 * @addtogroup TD_SENSOR Sensor Monitoring
	 * @{ */

	/*******************************************************************************
	 **************************  DEFINES   ****************************************
	 ******************************************************************************/

	/** @addtogroup TD_SENSOR_DEFINES Defines
	 * @{ */

	/** UDM Protocol Version release */
#define RELEASE 2

	/** UDM Protocol Version sub-release */
#define SUB_RELEASE 2

	/** For API backwards compatibility */
#define TD_SENSOR_GetKeepAliveConfig() (&(TD_SENSOR_GetModuleConfiguration()->keepalive));
#define TD_SENSOR_GetConnectionConfig() (&(TD_SENSOR_GetModuleConfiguration()->connection));
#define TD_SENSOR_GetSwitchesConfig() (&(TD_SENSOR_GetModuleConfiguration()->switches));
#define TD_SENSOR_GetRSSIConfig() (&(TD_SENSOR_GetModuleConfiguration()->rssi));
#define TD_SENSOR_GetBatteryConfig() (&(TD_SENSOR_GetModuleConfiguration()->battery));

	/** @} */

	/*******************************************************************************
	 **************************  EXTERNAL   ****************************************
	 ******************************************************************************/

	/** @addtogroup TD_SENSOR_EXTERN External Definitions
	 * @{ */

	extern uint32_t SigfoxID;

	/** @} */

	/*******************************************************************************
	 ***********************   ENUMERATIONS   **************************************
	 ******************************************************************************/

	 /** @addtogroup TD_SENSOR_ENUMERATIONS Enumerations
	 * @{ */

	/** Module Type */
	typedef enum {
		SENSOR_DEVICE,
		SENSOR_GATEWAY,
		SENSOR_TRANSMITTER
	} TD_SENSOR_ModuleType_t;

	/** Temperature Monitoring state */
	typedef enum {
		TEMPERATURE_LOW,
		TEMPERATURE_OK,
		TEMPERATURE_HIGH,
	} TD_SENSOR_TemperatureState_t;

	/** Sensor Event */
	typedef enum {
		SENSOR_SEND_BATTERY,
		SENSOR_SEND_LOCAL_KEEPALIVE,
		SENSOR_SEND_SENSOR_KEEPALIVE,
		SENSOR_SEND_TEMPERATURE,
		SENSOR_SEND_SWITCH,
		SENSOR_SEND_BOOT,
		SENSOR_SEND_USER
	} TD_SENSOR_Event_t;

	/** @} */

	/*******************************************************************************
	 **************************  TYPEDEFS   ****************************************
	 ******************************************************************************/

	/** @addtogroup TD_SENSOR_TYPEDEFS Typedefs
	 * @{ */

	/** Sensor Switch configuration */
	typedef struct {
		bool (*user_callback)(GPIO_Port_TypeDef port, unsigned int bit,	bool state);		///< Switch monitoring callback function
		GPIO_Port_TypeDef port : 8;							///< Switch monitoring I/O port
		uint8_t bit;										///< Switch monitoring I/O bit
		bool monitor : 1;									///< Switch monitoring enable flag
		bool falling : 1;									///< Switch monitoring IRQ on falling edge flag
		bool rising : 1;									///< Switch monitoring IRQ on rising edge flag
		bool pull : 1;										///< Switch monitoring pull-up/pull-down resistor enable flag
		bool pull_state : 1;								///< Switch monitoring pull state
		bool state : 1;										///< Switch monitoring I/O state
		bool on_irq : 1;									///< Switch monitoring enable IRQ flag
	} __PACKED TD_SENSOR_SwitchConfiguration_t;

	/** Switch monitoring state type */
	typedef struct {
		uint8_t param;										///< Switch monitoring I/O state
	} TD_SENSOR_SwitchState_t;

	/** Battery monitoring configuration type */
	typedef struct {
		bool (*user_callback)(bool state, uint16_t level);	///< Battery monitoring callback function
		uint16_t level;										///< Battery monitoring current level
		uint16_t level_low;									///< Battery monitoring low level
		uint16_t level_ok;									///< Battery monitoring OK level
		bool monitor;										///< Battery monitoring enable flag
		bool state;											///< Battery monitoring state
	} __PACKED TD_SENSOR_BatteryConfiguration_t;

	/** Temperature monitoring configuration type */
	typedef struct {
		bool (*user_callback)(TD_SENSOR_TemperatureState_t state, int16_t level);	///< User temperature monitoring callback function
		uint32_t interval;									///< Temperature monitoring interval period in seconds
		int16_t level_low;									///< Temperature monitoring low level
		int16_t level_high;									///< Temperature monitoring high level
		int16_t level;										///< Temperature monitoring current level
		uint8_t timer;										///< Temperature monitoring timer ID
		TD_SENSOR_TemperatureState_t state : 4;				///< Temperature monitoring state
		bool monitor : 1;									///< Temperature monitoring enable flag
	} __PACKED TD_SENSOR_TemperatureConfiguration_t;

	/** LAN Connection monitoring configuration type */
	typedef struct {
		uint32_t interval;									///< LAN connection interval period in hours
		uint8_t timer;										///< LAN connection timer ID
		bool monitor;										///< LAN connection monitoring enable flag
	} __PACKED TD_SENSOR_ConnectionConfiguration_t;

	/** Keepalive monitoring configuration type */
	typedef struct {
		uint8_t interval;									///< Keepalive interval period in hours
		uint8_t timer;										///< Keepalive timer ID
		bool monitor;										///< Keepalive monitoring enable flag
	} __PACKED TD_SENSOR_KeepaliveConfiguration_t;

	/** RSSI monitoring configuration type */
	typedef struct {
		int8_t level_low;									///< RSSI low level
		int8_t level_ok;									///< RSSI OK level
		bool monitor;										///< RSSI monitoring enable flag
	} __PACKED TD_SENSOR_RSSIConfiguration_t;

	/** Boot monitoring configuration type */
	typedef struct {
		bool (*user_callback)(void);						///< User boot callback function
		bool monitor;										///< Boot monitoring enable flag
	} __PACKED TD_SENSOR_BootConfiguration_t;

	/* Function pointer for code removal purposes */
	typedef void (*TD_SENSOR_MonitorInit_t)(void);

	/* Device Class type */
	typedef uint16_t TD_SENSOR_DeviceClass_t;

	/** Sensor configuration */
	typedef struct {
		TD_SENSOR_ModuleType_t type;						///< Module type (Device, Gateway, Transmitter)
		TD_SENSOR_DeviceClass_t class;						///< Device Class
		TD_SENSOR_SwitchConfiguration_t *switches;			///< Switch list
		uint8_t switch_count;								///< Switch count
		uint32_t switch_mask;								///< Switch IRQ mask
		TD_SENSOR_BatteryConfiguration_t battery;			///< Battery configuration
		TD_SENSOR_BootConfiguration_t boot;					///< Boot configuration
		TD_SENSOR_TemperatureConfiguration_t temperature;	///< Temperature configuration
		TD_SENSOR_ConnectionConfiguration_t connection;		///< Connection configuration
		TD_SENSOR_RSSIConfiguration_t rssi;					///< RSSI configuration
		TD_SENSOR_KeepaliveConfiguration_t keepalive;		///< Sensor Keep-alive configuration
	} TD_SENSOR_Configuration_t;

	/** @} */

	/*******************************************************************************
	 *************************   PROTOTYPES   **************************************
	 ******************************************************************************/

	/** @addtogroup TD_SENSOR_GLOBAL_FUNCTIONS Global Functions
	 * @{ */

	void TD_SENSOR_MonitorInit(void);
	void TD_SENSOR_BatteryCallBack(void);
	void TD_SENSOR_InternalInit(void);
	void TD_SENSOR_Dump(void);

	/** @} */

	/** @addtogroup TD_SENSOR_USER_FUNCTIONS User Functions
	 * @{ */

	bool TD_SENSOR_Init(TD_SENSOR_ModuleType_t type, uint32_t lan_frequency, int16_t lan_power_level);
	void TD_SENSOR_Process(void);
	void TD_SENSOR_Reset(void);
	void TD_SENSOR_MonitorBattery(bool enable, uint16_t level_low, uint16_t level_ok, bool (*callback)(bool state, uint16_t level));
	void TD_SENSOR_MonitorBoot(bool enable, bool (*callback)(void));
	bool TD_SENSOR_MonitorConnection(bool enable, uint32_t interval);
	bool TD_SENSOR_MonitorKeepAlive(bool enable, uint8_t interval);
	bool TD_SENSOR_MonitorRSSI(bool enable, int8_t level_low, int8_t level_ok);
	bool TD_SENSOR_MonitorSwitch(bool enable, GPIO_Port_TypeDef port, unsigned int bit, bool falling, bool rising, bool pull, bool state, bool (*switch_callback)(GPIO_Port_TypeDef port, unsigned int bit,	bool state));
	bool TD_SENSOR_MonitorSwitchIrq(bool enable, GPIO_Port_TypeDef port, unsigned int bit, bool falling, bool rising, bool pull, bool state, bool (*switch_callback)(GPIO_Port_TypeDef port, unsigned int bit,	bool state));
	void TD_SENSOR_MonitorTemperature(bool enable, uint32_t interval, int16_t level_low, int16_t level_high, bool (*callback)(TD_SENSOR_TemperatureState_t state, int16_t level));
	TD_SENSOR_ModuleType_t TD_SENSOR_GetModuleType(void);
	void TD_SENSOR_SetModuleType(TD_SENSOR_ModuleType_t type);
	TD_SENSOR_Configuration_t *TD_SENSOR_GetModuleConfiguration(void);
	void TD_SENSOR_SetModuleConfiguration(TD_SENSOR_Configuration_t *config);
	uint32_t TD_SENSOR_GetSigfoxID(void);
	uint8_t TD_SENSOR_GetCustomBootCause(void);
	uint32_t TD_SENSOR_GetBootCause(void);
	void TD_SENSOR_SetDeviceClass(uint16_t class);
	void TD_SENSOR_SetCustomBootCause(uint8_t cause);
	bool TD_SENSOR_IsBatteryDead(void);
	void TD_SENSOR_ClearBatteryDead(void);

	/** @} */

	/** @} (end addtogroup TD_SENSOR) */

#ifdef __cplusplus
}
#endif

#endif // __TD_SENSOR_H
