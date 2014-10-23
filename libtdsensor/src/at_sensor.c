/***************************************************************************//**
 * @file
 * @brief AT Sensor
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

#include <stdint.h>
#include <stdbool.h>

#include <at_parse.h>
#include <td_flash.h>
#include <td_utils.h>
#include <td_trap.h>

#include "td_sensor.h"
#include "td_sensor_device.h"
#include "td_measure.h"
#include "at_sensor.h"

/***************************************************************************//**
 * @addtogroup AT_SENSOR Sensor AT Command Extension
 * @brief Sensor AT command extension for the TDxxxx RF modules
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   DEFINES   *****************************************
 ******************************************************************************/

/** @addtogroup AT_SENSOR_DEFINES Defines
 * @{ */

#define AT_SENSOR_PROFILE_TYPE_COUNT 6

/** @} */

/*******************************************************************************
 ***********************   ENUMERATIONS   **************************************
 ******************************************************************************/

/** @addtogroup AT_SENSOR_ENUMERATIONS Enumerations
 * @{ */

/** Sensor AT command tokens */
typedef enum sensor_tokens_t {
	AT_EXTENSION_BASE = AT_BASE_LAST,
	AT_SENSOR_SET_TYPE,
	AT_SENSOR_QUERY_TYPE,
	AT_SENSOR_GET_TYPE,
	AT_SENSOR_QUERY_DEVICE_CLASS,
	AT_SENSOR_SET_DEVICE_CLASS,
	AT_SENSOR_GET_DEVICE_CLASS,
	AT_SENSOR_QUERY_BATTERY_MONITORING,
	AT_SENSOR_GET_BATTERY_MONITORING,
	AT_SENSOR_SET_BATTERY_MONITORING,
	AT_SENSOR_QUERY_BOOT_MONITORING,
	AT_SENSOR_GET_BOOT_MONITORING,
	AT_SENSOR_SET_BOOT_MONITORING,
	AT_SENSOR_QUERY_TEMPERATURE_MONITORING,
	AT_SENSOR_GET_TEMPERATURE_MONITORING,
	AT_SENSOR_SET_TEMPERATURE_MONITORING,
	AT_SENSOR_QUERY_RSSI_MONITORING,
	AT_SENSOR_GET_RSSI_MONITORING,
	AT_SENSOR_SET_RSSI_MONITORING,
	AT_SENSOR_QUERY_CONNECTION_MONITORING,
	AT_SENSOR_GET_CONNECTION_MONITORING,
	AT_SENSOR_SET_CONNECTION_MONITORING,
	AT_SENSOR_QUERY_SWITCH_MONITORING,
	AT_SENSOR_GET_SWITCH_MONITORING,
	AT_SENSOR_SET_SWITCH_MONITORING,
	AT_SENSOR_QUERY_KEEPALIVE_MONITORING,
	AT_SENSOR_GET_KEEPALIVE_MONITORING,
	AT_SENSOR_SET_KEEPALIVE_MONITORING,
	AT_SENSOR_QUERY_RETRANSMISSION_PROFILE,
	AT_SENSOR_SET_RETRANSMISSION_PROFILE,
	AT_SENSOR_GET_RETRANSMISSION_PROFILE,
	AT_SENSOR_SENSOR_RESET,
	AT_SENSOR_SENSOR_DUMP,
	AT_SENSOR_DELETE_FLASH_VARIABLES,
	AT_CRASH
} sensor_tokens;

/** @} */

/*******************************************************************************
 *************************  CONSTANTS   ****************************************
 ******************************************************************************/

/** @addtogroup AT_SENSOR_CONSTANTS Constants
 * @{ */

/** Sensor AT command set */
static AT_command_t const sensor_commands[] = {
	{"ATS500=?", AT_SENSOR_QUERY_TYPE},
	{"ATS500=", AT_SENSOR_SET_TYPE},
	{"ATS500?", AT_SENSOR_GET_TYPE},
	{"ATS501=?", AT_SENSOR_QUERY_DEVICE_CLASS},
	{"ATS501=", AT_SENSOR_SET_DEVICE_CLASS},
	{"ATS501?", AT_SENSOR_GET_DEVICE_CLASS},
	{"ATS502=?", AT_SENSOR_QUERY_BATTERY_MONITORING},
	{"ATS502=", AT_SENSOR_SET_BATTERY_MONITORING},
	{"ATS502?", AT_SENSOR_GET_BATTERY_MONITORING},
	{"ATS503=?", AT_SENSOR_QUERY_TEMPERATURE_MONITORING},
	{"ATS503=", AT_SENSOR_SET_TEMPERATURE_MONITORING},
	{"ATS503?", AT_SENSOR_GET_TEMPERATURE_MONITORING},
	{"ATS504=?", AT_SENSOR_QUERY_RSSI_MONITORING},
	{"ATS504=", AT_SENSOR_SET_RSSI_MONITORING},
	{"ATS504?", AT_SENSOR_GET_RSSI_MONITORING},
	{"ATS505=?", AT_SENSOR_QUERY_CONNECTION_MONITORING},
	{"ATS505=", AT_SENSOR_SET_CONNECTION_MONITORING},
	{"ATS505?", AT_SENSOR_GET_CONNECTION_MONITORING},
	{"ATS506=?", AT_SENSOR_QUERY_SWITCH_MONITORING},
	{"ATS506=", AT_SENSOR_SET_SWITCH_MONITORING},
	{"ATS506?", AT_SENSOR_GET_SWITCH_MONITORING},
	{"ATS507=?", AT_SENSOR_QUERY_BOOT_MONITORING},
	{"ATS507=", AT_SENSOR_SET_BOOT_MONITORING},
	{"ATS507?", AT_SENSOR_GET_BOOT_MONITORING},
	{"ATS508=?", AT_SENSOR_QUERY_KEEPALIVE_MONITORING},
	{"ATS508=", AT_SENSOR_SET_KEEPALIVE_MONITORING},
	{"ATS508?", AT_SENSOR_GET_KEEPALIVE_MONITORING},
	{"ATS509=?", AT_SENSOR_QUERY_RETRANSMISSION_PROFILE},
	{"ATS509=", AT_SENSOR_SET_RETRANSMISSION_PROFILE},
	{"ATS509?", AT_SENSOR_GET_RETRANSMISSION_PROFILE},
	{"AT$SZ", AT_SENSOR_SENSOR_RESET},
	{"AT$SD", AT_SENSOR_SENSOR_DUMP},
	{"AT$DF", AT_SENSOR_DELETE_FLASH_VARIABLES},
	{"AT$CRASH", AT_CRASH},
	{0, 0}
};

/** @} */

/*******************************************************************************
 ************************   PRIVATE VARIABLES   ********************************
 ******************************************************************************/

/** @addtogroup AT_SENSOR_LOCAL_VARIABLES Local Variables
 * @{ */

/** The Sensor module type */
static TD_SENSOR_ModuleType_t type;

/** The Sensor transmission profiles */
static TD_SENSOR_TransmitProfile_t Profiles[AT_SENSOR_PROFILE_TYPE_COUNT];

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup AT_SENSOR_LOCAL_FUNCTIONS Local Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Initialization AT extension function for Sensor.
 ******************************************************************************/
static void sensor_init(void)
{
	int i;

	type = SENSOR_TRANSMITTER;
	for (i = 0; i < AT_SENSOR_PROFILE_TYPE_COUNT; i++) {
		Profiles[i].interval = 0;
		Profiles[i].repetition = 0;
	}
	TD_SENSOR_InternalInit();
}

/***************************************************************************//**
 * @brief
 *   Help AT extension function for Sensor.
 ******************************************************************************/
static void sensor_help(void)
{
	AT_printf(
		"ATS500 => Module type\r\n"
		"ATS501 => Device class\r\n"
		"ATS502 => Battery monitoring\r\n"
		"ATS503= => Temperature monitoring\r\n"
		"ATS504= => RSSI monitoring\r\n"
		"ATS505= => Connection monitoring\r\n"
		"ATS506= => Switch monitoring\r\n"
		"ATS507 => Boot monitoring\r\n"
		"ATS508= => Keep-alive monitoring\r\n"
		"AT$SZ= => Sensor reset\r\n"
	);
};

/***************************************************************************//**
 * @brief
 *   Persistence AT extension function for Sensor.
 *
 * @param[in] write
 *   Flag set to true for writing persistent data, false for reading persistent
 *   data.
 *
 * @param[out] buffer
 *   Pointer to the persistent data buffer to read/write.
 *
 * @param[in] count
 *   The length in bytes of the persistent data buffer.
 *
 * @return
 *   The number of bytes read from/written to the persistent data buffer.
 ******************************************************************************/
static uint8_t sensor_persist(bool write, uint8_t *buffer, uint8_t count)
{
	int i;
	TD_SENSOR_Configuration_t *temp;
	TD_SENSOR_Configuration_t config;

	if (buffer != 0 && count != 0) {
		if (write == true) {
			for (i = 0; i < AT_SENSOR_PROFILE_TYPE_COUNT; i++) {
				*buffer++ = Profiles[i].repetition;
				*buffer++ = (Profiles[i].interval >> 8) & 0xFF;
				*buffer++ = Profiles[i].interval & 0xFF;
			}

			// Get module configuration
			temp = TD_SENSOR_GetModuleConfiguration();

			// Apply locally saved new AT type
			((TD_SENSOR_Configuration_t *) temp)->type = type;

			// Copy configuration
			memcpy(buffer, (uint8_t *) temp, sizeof(TD_SENSOR_Configuration_t));
			buffer += sizeof(TD_SENSOR_Configuration_t);

		} else {

			// Profiles must be set before sensor is initialized as it might trigger sending frames
			for (i = 0; i < AT_SENSOR_PROFILE_TYPE_COUNT; i++) {
				Profiles[i].repetition = *buffer++;
				Profiles[i].interval = (*buffer++) << 8;
				Profiles[i].interval |= *buffer++;
				TD_SENSOR_SetTransmissionProfile((TD_SENSOR_FrameType_t) i,
												 Profiles[i].repetition,
												 Profiles[i].interval);
			}

			//cCopy sensor configuration
			memcpy(&config, buffer, sizeof(TD_SENSOR_Configuration_t));

			// Copy type to local configuration
			type = config.type;

			// Apply sensor configuration
			TD_SENSOR_SetModuleConfiguration(&config);
			buffer += sizeof(TD_SENSOR_Configuration_t);
		}
	}
	return sizeof (TD_SENSOR_Configuration_t) + (3 * AT_SENSOR_PROFILE_TYPE_COUNT);
}

/***************************************************************************//**
 * @brief
 *   Parser AT extension function for Sensor.
 *
 * @param[in] token
 *   The token to parse.
 *
 * @return
 *   The parse result.
 ******************************************************************************/
static int8_t sensor_parse(uint8_t token)
{
	int i;
	int8_t result = AT_OK;
	TD_SENSOR_Configuration_t *config = TD_SENSOR_GetModuleConfiguration();
	uint8_t profile, repetitions;
	uint16_t interval;

	switch (token) {
	case AT_SENSOR_QUERY_TYPE:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			AT_printf("Device: 0, Gateway: 1, Transmitter: 2\r\n");
		}
		break;

	case AT_SENSOR_GET_TYPE:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			AT_printf("%02X\r\n", type);
		}
		break;

	case AT_SENSOR_SET_TYPE:
		if (AT_argc == 1) {
			if (AT_atoll(AT_argv[0]) >= 0 && AT_atoll(AT_argv[0]) <= 2) {
				type = (TD_SENSOR_ModuleType_t) AT_atoll(AT_argv[0]);

				// To make sure LAN address is reset on reboot.
				if (type == SENSOR_DEVICE) {
					TD_SENSOR_DEVICE_Reset();
				}
			} else {
				result = AT_ERROR;
			}
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_SENSOR_QUERY_DEVICE_CLASS:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			AT_printf("0..65535\r\n");
		}
		break;

	case AT_SENSOR_GET_DEVICE_CLASS:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			AT_printf("0x%04X\r\n", config->class);
		}
		break;

	case AT_SENSOR_SET_DEVICE_CLASS:
		if (AT_argc == 1) {
			if (AT_atoll(AT_argv[0]) >= 0 && AT_atoll(AT_argv[0]) <= 65535) {
				TD_SENSOR_SetDeviceClass(AT_atoll(AT_argv[0]));
			} else {
				result = AT_ERROR;
			}
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_SENSOR_QUERY_BATTERY_MONITORING:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			AT_printf("0..1, 2100..3300, 2100..3300\r\n");
		}
		break;

	case AT_SENSOR_GET_BATTERY_MONITORING:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			AT_printf("%d,%d,%d\r\n", config->battery.monitor, config->battery.level_low, config->battery.level_ok);
		}
		break;

	case AT_SENSOR_SET_BATTERY_MONITORING:
		if (AT_argc == 3) {
			if (AT_atoll(AT_argv[0]) == 1 && AT_atoll(AT_argv[1]) >= 1667
					&& AT_atoll(AT_argv[1]) <= 3300
					&& AT_atoll(AT_argv[2]) >= 2100
					&& AT_atoll(AT_argv[2]) <= 3300) {
				TD_SENSOR_MonitorBattery(true, AT_atoll(AT_argv[1]), AT_atoll(AT_argv[2]), 0);
			} else {
				result = AT_ERROR;
			}
		} else if (AT_argc == 1) {
			if (AT_atoll(AT_argv[0]) == 0) {
				TD_SENSOR_MonitorBattery(false, 0, 0, 0);
			}
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_SENSOR_QUERY_BOOT_MONITORING:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			AT_printf("0..1\r\n");
		}
		break;

	case AT_SENSOR_GET_BOOT_MONITORING:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			AT_printf("%d\r\n", config->boot.monitor);
		}
		break;

	case AT_SENSOR_SET_BOOT_MONITORING:
		if (AT_argc == 1) {
			if (AT_atoll(AT_argv[0]) == 1) {
				TD_SENSOR_MonitorBoot(true, 0);
			} else if (AT_atoll(AT_argv[0]) == 0) {
				TD_SENSOR_MonitorBoot(false, 0);
			}
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_SENSOR_QUERY_TEMPERATURE_MONITORING:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			AT_printf("0..1, 0..4294967295, -30..85, -30..85\r\n");
		}
		break;

	case AT_SENSOR_GET_TEMPERATURE_MONITORING:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			AT_printf("%d,%d,%d,%d\r\n", config->temperature.monitor,
					  config->temperature.interval,
					  config->temperature.level_low / 10,
					  config->temperature.level_high / 10);
		}
		break;

	case AT_SENSOR_SET_TEMPERATURE_MONITORING:
		if (AT_argc == 4) {
			if (AT_atoll(AT_argv[0]) == 1 && AT_atoll(AT_argv[1]) >= 1
					&& AT_atoll(AT_argv[1]) <= 4294967295UL
					&& AT_atoll(AT_argv[2]) >= -30 && AT_atoll(AT_argv[2]) <= 85
					&& AT_atoll(AT_argv[3]) >= -30
					&& AT_atoll(AT_argv[3]) <= 85) {
				TD_SENSOR_MonitorTemperature(true,
					AT_atoll(AT_argv[1]),
					AT_atoll(AT_argv[2]) * 10,
					AT_atoll(AT_argv[3]) * 10,
					0);
			} else {
				result = AT_ERROR;
			}
		} else if (AT_argc == 1) {
			if (AT_atoll(AT_argv[0]) == 0) {
				TD_SENSOR_MonitorTemperature(false, 0, 0, 0, 0);
			} else {
				result = AT_ERROR;
			}
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_SENSOR_QUERY_RSSI_MONITORING:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			AT_printf("0..1, -122..14, -122..14\r\n");
		}
		break;

	case AT_SENSOR_GET_RSSI_MONITORING:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			AT_printf("%d,%d,%d\r\n", config->rssi.monitor,
				config->rssi.level_low,
				config->rssi.level_ok);
		}
		break;

	case AT_SENSOR_SET_RSSI_MONITORING:
		if (AT_argc == 3) {
			if (AT_atoll(AT_argv[0]) == 1 && AT_atoll(AT_argv[1]) >= -122
					&& AT_atoll(AT_argv[1]) <= 14
					&& AT_atoll(AT_argv[2]) >= -122
					&& AT_atoll(AT_argv[2]) <= 14) {
				if (!TD_SENSOR_MonitorRSSI(true,
					AT_atoll(AT_argv[1]),
					AT_atoll(AT_argv[2]))) {
					result = AT_ERROR;
				}
			} else {
				result = AT_ERROR;
			}
		} else if (AT_argc == 1) {
			if (AT_atoll(AT_argv[0]) == 0) {
				TD_SENSOR_MonitorRSSI(false, 0, 0);
			} else {
				result = AT_ERROR;
			}
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_SENSOR_QUERY_CONNECTION_MONITORING:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			AT_printf("0..1, 10..4294967295\r\n");
		}
		break;

	case AT_SENSOR_GET_CONNECTION_MONITORING:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			AT_printf("%d,%d\r\n",
				config->connection.monitor,
				config->connection.interval);
		}
		break;

	case AT_SENSOR_SET_CONNECTION_MONITORING:
		if (AT_argc == 2) {
			if (AT_atoll(AT_argv[0]) == 1 && AT_atoll(AT_argv[1]) >= 10
					&& AT_atoll(AT_argv[1]) <= 4294967295UL) {
				if (!TD_SENSOR_MonitorConnection(true, AT_atoll(AT_argv[1]))) {
					result = AT_ERROR;
				}
			} else {
				result = AT_ERROR;
			}
		} else if (AT_argc == 1) {
			if (AT_atoll(AT_argv[0]) == 0) {
				TD_SENSOR_MonitorConnection(false, 0);
			} else {
				result = AT_ERROR;
			}
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_SENSOR_QUERY_KEEPALIVE_MONITORING:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			AT_printf("0..1, 1..255\r\n");
		}
		break;

	case AT_SENSOR_GET_KEEPALIVE_MONITORING:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			AT_printf("%d,%d\r\n",
				config->keepalive.monitor,
				config->keepalive.interval);
		}
		break;

	case AT_SENSOR_SET_KEEPALIVE_MONITORING:
		if (AT_argc == 2) {
			if (AT_atoll(AT_argv[0]) == 1 && AT_atoll(AT_argv[1]) >= 1
					&& AT_atoll(AT_argv[1]) <= 255) {
				if (!TD_SENSOR_MonitorKeepAlive(true, AT_atoll(AT_argv[1]))) {
					result = AT_ERROR;
				}
			} else {
				result = AT_ERROR;
			}
		} else if (AT_argc == 1) {
			if (AT_atoll(AT_argv[0]) == 0) {
				if (!TD_SENSOR_MonitorKeepAlive(false, 0)) {
					result = AT_ERROR;
				}
			} else {
				result = AT_ERROR;
			}
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_SENSOR_QUERY_SWITCH_MONITORING:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			AT_printf("0..1, 0..5, 0..15, 0..1, 0..1, 0..1, 0..1\r\n");
		}
		break;

	case AT_SENSOR_GET_SWITCH_MONITORING:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			for (i = 0; i < CONFIG_TD_SENSOR_MAX_SWITCH; i++) {
				if (config->switches[i].monitor == true) {
					AT_printf("%d,%d",
						config->switches[i].port,
						config->switches[i].bit);
					AT_printf("%d,%d",
						config->switches[i].falling,
						config->switches[i].rising);
					AT_printf("%d,%d \r\n",
						config->switches[i].pull,
						config->switches[i].pull_state);
				}
			}
		}
		break;

	case AT_SENSOR_SET_SWITCH_MONITORING:
		if (AT_argc == 7) {
			if (AT_atoll(AT_argv[0]) == 1 && AT_atoll(AT_argv[1]) >= 0
					&& AT_atoll(AT_argv[1]) <= 5 && AT_atoll(AT_argv[2]) >= 0
					&& AT_atoll(AT_argv[2]) <= 15 && AT_atoll(AT_argv[3]) >= 0
					&& AT_atoll(AT_argv[3]) <= 1 && AT_atoll(AT_argv[4]) >= 0
					&& AT_atoll(AT_argv[4]) <= 1 && AT_atoll(AT_argv[5]) >= 0
					&& AT_atoll(AT_argv[5]) <= 1 && AT_atoll(AT_argv[6]) >= 0
					&& AT_atoll(AT_argv[6]) <= 1) {
				if (!TD_SENSOR_MonitorSwitch(
					true,
					(GPIO_Port_TypeDef) AT_atoll(AT_argv[1]),
					AT_atoll(AT_argv[2]),
					AT_atoll(AT_argv[3]),
					AT_atoll(AT_argv[4]),
					AT_atoll(AT_argv[5]),
					AT_atoll(AT_argv[6]),
					0)) {
					result = AT_ERROR;
				}
			} else {
				result = AT_ERROR;
			}
		} else if (AT_argc == 3) {
			if (AT_atoll(AT_argv[0]) == 0 && AT_atoll(AT_argv[1]) >= 0
					&& AT_atoll(AT_argv[1]) <= 5 && AT_atoll(AT_argv[2]) >= 0
					&& AT_atoll(AT_argv[2]) <= 15) {
				TD_SENSOR_MonitorSwitch(
					false,
					(GPIO_Port_TypeDef) AT_atoll(AT_argv[1]),
					AT_atoll(AT_argv[2]),
					0,
					0,
					0,
					0,
					0);
			} else {
				result = AT_ERROR;
			}
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_SENSOR_SET_RETRANSMISSION_PROFILE:
		if (AT_argc == 3) {
			profile = AT_atoll(AT_argv[0]);
			repetitions = AT_atoll(AT_argv[1]);
			interval = AT_atoll(AT_argv[2]);
			if (profile <= 5 && repetitions <= 15 && interval <= 600) {
				Profiles[profile].interval = interval;
				Profiles[profile].repetition = repetitions;
				TD_SENSOR_SetTransmissionProfile((TD_SENSOR_FrameType_t) profile, repetitions, interval);
			} else {
				result = AT_ERROR;
			}
		}
		break;

	case AT_SENSOR_QUERY_RETRANSMISSION_PROFILE:
		if (AT_argc == 0) {
			AT_printf("0..5, 0..15, 0..600\r\n");
		}
		break;

	case AT_SENSOR_GET_RETRANSMISSION_PROFILE:
		if (AT_argc == 0) {
			for (i = 0; i < AT_SENSOR_PROFILE_TYPE_COUNT; i++) {
				AT_printf("%d, %d, %d\r\n", i, Profiles[i].repetition, Profiles[i].interval);
			}
		}
		break;

	case AT_SENSOR_DELETE_FLASH_VARIABLES:
		if (AT_argc == 0) {
			TD_FLASH_DeleteVariables();
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_SENSOR_SENSOR_RESET:
		if (AT_argc == 0) {
			TD_SENSOR_Reset();
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_SENSOR_SENSOR_DUMP:
		if (AT_argc == 0) {
			TD_SystemDump(DUMP_SENSOR);
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_CRASH:
		while (1);
		break;

	default:
		result = AT_NOTHING;
		break;
	}
	return result;
}

/** @} */

/*******************************************************************************
 *************************   PUBLIC VARIABLES   ********************************
 ******************************************************************************/

/** @addtogroup AT_SENSOR_GLOBAL_VARIABLES Global Variables
 * @{ */

/** AT extension structure for Sensor */
AT_extension_t sensor_extension = {
	.commands = sensor_commands,
	.init =	sensor_init,
	.help = sensor_help,
	.parse = sensor_parse,
	.persist = sensor_persist
};

/** @} */

/** @} (end addtogroup AT_SENSOR) */
