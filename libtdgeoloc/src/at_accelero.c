/***************************************************************************//**
* @file
 * @brief Accelerometer AT command extension for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 1.0.0
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

#include <em_gpio.h>

#include <td_core.h>
#include <td_spi.h>
#include <td_utils.h>
#include <at_parse.h>
#include <td_printf.h>

#include "td_accelero.h"
#include "at_accelero.h"
#include "lis3dh.h"

/***************************************************************************//**
 * @addtogroup AT_ACCELERO Accelerometer AT Command Extension
 * @brief Accelerometer AT command extension for the TDxxxx RF modules
 * @{
 ******************************************************************************/

/*******************************************************************************
 ***********************   ENUMERATIONS   **************************************
 ******************************************************************************/

/** @addtogroup AT_ACCELERO_ENUMERATIONS Enumerations
 * @{ */

/** Accelerometer AT command tokens */
typedef enum accelero_tokens_t {
	AT_EXTENSION_BASE = AT_BASE_LAST,			///< First extension token
	AT_ACCELERO_DUMP,							///< AT$AD
	AT_ACCELERO_GET_MONITOR_DATA,				///< ATS650?
	AT_ACCELERO_GET_MONITOR_EVENT,				///< ATS651?
	AT_ACCELERO_QUERY_MONITOR_DATA,				///< ATS650=?
	AT_ACCELERO_QUERY_MONITOR_EVENT,			///< ATS651=?
	AT_ACCELERO_READ,							///< AT$AR=
	AT_ACCELERO_SET_MONITOR_DATA,				///< ATS650=
	AT_ACCELERO_SET_MONITOR_EVENT,				///< ATS651=
	AT_ACCELERO_WRITE							///< AT$AW=
} accelero_tokens;

/** @} */

/*******************************************************************************
 *************************  CONSTANTS  *****************************************
 ******************************************************************************/

/** @addtogroup AT_ACCELERO_CONSTANTS Constants
 * @{ */

/** AT accelerometer extension command set */
static AT_command_t const accelero_commands[] = {
	{"ATS650=?", AT_ACCELERO_QUERY_MONITOR_DATA},	///< Data monitoring commands range
	{"ATS650=", AT_ACCELERO_SET_MONITOR_DATA},		///< Data monitoring configuration
	{"ATS650?", AT_ACCELERO_GET_MONITOR_DATA},		///< Data monitoring state
	{"ATS651=?", AT_ACCELERO_QUERY_MONITOR_EVENT},	///< Event monitoring commands range
	{"ATS651=", AT_ACCELERO_SET_MONITOR_EVENT},		///< Event monitoring configuration
	{"ATS651?", AT_ACCELERO_GET_MONITOR_EVENT},		///< Event monitoring state
	{"AT$AD", AT_ACCELERO_DUMP},					///< Full accelerometer dump
	{"AT$AR=", AT_ACCELERO_READ},					///< accelerometer read register
	{"AT$AW=", AT_ACCELERO_WRITE},					///< accelerometer write register
	{0, 0}
};

/** @} */

/*******************************************************************************
 ************************   PRIVATE VARIABLES   ********************************
 ******************************************************************************/

/** @addtogroup AT_ACCELERO_LOCAL_VARIABLES Local Variables
 * @{ */

/* Active accelerometer events to filter low events*/
uint8_t AT_AcceleroEvents = 0;

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup AT_ACCELERO_LOCAL_FUNCTIONS Local Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Callback function called when the accelerometer data changes.
 *
 * @param[in] data
 *   Pointer to the data buffer containing the accelerometer data.
 *
 * @param[in] count
 *   Length of the accelerometer data in bytes.
 *
 * @param[in] overrun
 *   Flag that is set to true if an overrun occured, false otherwise.
 ******************************************************************************/
static void data_callback(TD_ACCELERO_Data_t *data, uint8_t count, bool overrun)
{
	int i;

	for (i = 0; i < count; i++) {
		tfp_printf("%d \t %d \t %d\r\n", data[i].x, data[i].y, data[i].z);
	}
	/*if (overrun) {
		tfp_printf("overrun\r\n");
	}*/
}

/***************************************************************************//**
 * @brief
 *   Callback function called when the accelerometer generates an IRQ.
 *
 * @param[in] source
 *   The (possibly) simultaneous IRQ sources.
 ******************************************************************************/
static void event_callback(uint8_t source)
{
	AT_printf("Accelero Event:\r\n");
	if (source & TD_ACCELERO_IRQ_XL & AT_AcceleroEvents) {
		tfp_printf("x low \r\n");
	}
	if (source & TD_ACCELERO_IRQ_XH & AT_AcceleroEvents) {
		tfp_printf("x high \r\n");
	}
	if (source & TD_ACCELERO_IRQ_YL & AT_AcceleroEvents) {
		tfp_printf("y low \r\n");
	}
	if (source & TD_ACCELERO_IRQ_YH & AT_AcceleroEvents) {
		tfp_printf("y high \r\n");
	}
	if (source & TD_ACCELERO_IRQ_ZL & AT_AcceleroEvents) {
		tfp_printf("z low \r\n");
	}
	if (source & TD_ACCELERO_IRQ_ZH & AT_AcceleroEvents) {
		tfp_printf("z high \r\n");
	}
}

/***************************************************************************//**
 * @brief
 *   Initialization AT extension function for accelerometer.
 ******************************************************************************/
static void accelero_init(void)
{
	AT_AcceleroEvents = 0;
	TD_ACCELERO_Config_t config;
	config.monitoring = TD_ACCELERO_NO_MONITORING;
	TD_ACCELERO_SetConfig(&config);
}

/***************************************************************************//**
 * @brief
 *   Persistence AT extension function for accelerometer functions.
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
static uint8_t accelero_persist(bool write, uint8_t *buffer, uint8_t count)
{
	TD_ACCELERO_Config_t config;

	if (buffer != 0 && count != 0) {
		if (write == true) {
			*buffer++ = AT_AcceleroEvents;
			memcpy(buffer, TD_ACCELERO_GetConfig(), sizeof (TD_ACCELERO_Config_t));
			buffer += sizeof (TD_ACCELERO_Config_t);
		} else {
			AT_AcceleroEvents = (uint8_t) * buffer++;
			memcpy(&config, buffer, sizeof (TD_ACCELERO_Config_t));
			buffer += sizeof (TD_ACCELERO_Config_t);
			TD_ACCELERO_SetConfig(&config);
		}
	}
	return 1 + sizeof (TD_ACCELERO_Config_t);
}

/***************************************************************************//**
 * @brief
 *   Parser AT extension function for the accelerometer.
 *
 * @param[in] token
 *   The token to parse.
 *
 * @return
 *   The parse result.
 ******************************************************************************/
static int8_t accelero_parse(uint8_t token)
{
	int8_t result = AT_OK;
	TD_ACCELERO_Config_t *accelero = 0;
	int enable, low_power, rate, scale, filter, threshold, duration;

	accelero = TD_ACCELERO_GetConfig();
	switch (token) {
	case AT_ACCELERO_DUMP:
		if (AT_argc == 0) {
			TD_SystemDump(DUMP_LIS3DH);
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_ACCELERO_GET_MONITOR_DATA:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			if (accelero->monitoring == TD_ACCELERO_MONITOR_DATA) {
				AT_printf("1,%d,%d,%d,%d\r\n",
						  accelero->config.low_power,
						  accelero->config.rate,
						  accelero->config.scale,
						  accelero->config.filter);
			} else {
				AT_printf("0\r\n");
			}
		}
		break;

	case AT_ACCELERO_GET_MONITOR_EVENT:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			if (accelero->monitoring == TD_ACCELERO_MONITOR_EVENT) {
				AT_printf("1,%d,%d,%d,",
						  accelero->config.scale,
						  accelero->config.rate,
						  accelero->event_config.threshold);
				tfp_printf("%d,%d,%d\r\n",
						   accelero->event_config.duration,
						   accelero->event_config.event,
						   accelero->config.filter);
			} else {
				AT_printf("0\r\n");
			}
		}
		break;

	case AT_ACCELERO_QUERY_MONITOR_DATA:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			AT_printf("0..1, 0..1,1..4,2|4|8|16,0..1\r\n");
		}
		break;

	case AT_ACCELERO_QUERY_MONITOR_EVENT:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			AT_printf("0..1,2|4|8|16,1..4,0..127,0..127,0..63,0..1\r\n");
		}
		break;

	case AT_ACCELERO_READ:
		if (AT_argc == 1) {
			tfp_printf("Reg %x = %x\r\n",
				AT_atoll(AT_argv[0]),
				TD_ACCELERO_ReadRegister(AT_atoll(AT_argv[0])));
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_ACCELERO_SET_MONITOR_DATA:
		if (AT_argc >= 1) {
			enable = AT_atoll(AT_argv[0]);
			if (enable == 0 && AT_argc == 1) {
				TD_ACCELERO_MonitorData(false, 0, (TD_ACCELERO_Rates_t) 0, 0, (TD_ACCELERO_Scales_t) 0, 0,(TD_ACCELERO_FifoModes_t) 0, 0, 0);
			} else if (enable == 1 && AT_argc == 5) {
				low_power = AT_atoll(AT_argv[1]);
				rate = AT_atoll(AT_argv[2]);
				scale = AT_atoll(AT_argv[3]);
				filter = AT_atoll(AT_argv[4]);
				if ((low_power == 0 || low_power == 1) &&
					rate >= 1 && rate <= 4 &&
					(scale == 2 || scale == 4 || scale == 8 || scale == 16) &&
					(filter == 0 || filter == 1)) {
					TD_ACCELERO_MonitorEvent(false, (TD_ACCELERO_Rates_t) 0, 0, (TD_ACCELERO_Scales_t) 0, 0, 0, 0, 0, 0);
					TD_ACCELERO_MonitorData(true,
											low_power,
											(TD_ACCELERO_Rates_t) rate,
											TD_ACCELERO_AXIS_X | TD_ACCELERO_AXIS_Y | TD_ACCELERO_AXIS_Z,
											(TD_ACCELERO_Scales_t) scale,
											filter,
											(TD_ACCELERO_FifoModes_t) 2,
											0,
											data_callback);
				} else {
					result = AT_ERROR;
				}
			} else {
				result = AT_ERROR;
			}
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_ACCELERO_SET_MONITOR_EVENT:
		if (AT_argc >= 1) {
			enable = AT_atoll(AT_argv[0]);

			if (enable == 0 && AT_argc == 1) {
				TD_ACCELERO_MonitorEvent(false, (TD_ACCELERO_Rates_t) 0, 0, (TD_ACCELERO_Scales_t) 0, 0, 0, 0, 0, 0);
			} else if (enable == 1 && AT_argc == 7) {
				rate = AT_atoll(AT_argv[1]);
				scale = AT_atoll(AT_argv[2]);
				threshold = AT_atoll(AT_argv[3]);
				duration = AT_atoll(AT_argv[4]);
				AT_AcceleroEvents = AT_atoll(AT_argv[5]);
				filter = AT_atoll(AT_argv[6]);
				if ((rate >= 1 && rate <= 9) &&
					(scale == 2 || scale == 4 || scale == 8 || scale == 16) &&
					threshold >= 0 && threshold < 128 &&
					duration >= 0 && duration < 128 &&
					(filter == 0 || filter == 1)) {
					TD_ACCELERO_MonitorData(false, 0, (TD_ACCELERO_Rates_t) 0, 0, (TD_ACCELERO_Scales_t) 0, 0,(TD_ACCELERO_FifoModes_t) 0, 0, 0);
					TD_ACCELERO_MonitorEvent(true,
						(TD_ACCELERO_Rates_t) rate,
						TD_ACCELERO_AXIS_X | TD_ACCELERO_AXIS_Y | TD_ACCELERO_AXIS_Z,
						(TD_ACCELERO_Scales_t) scale,
						AT_AcceleroEvents,
						threshold,
						duration,
						filter,
						event_callback);
				} else {
					result = AT_ERROR;
				}
			} else {
				result = AT_ERROR;
			}
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_ACCELERO_WRITE:
		if (AT_argc == 2) {
			TD_ACCELERO_WriteRegister(AT_atoll(AT_argv[0]),AT_atoll(AT_argv[1]));
		} else {
			result = AT_ERROR;
		}
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

/** @addtogroup AT_ACCELERO_GLOBAL_VARIABLES Global Variables
 * @{ */

/** AT extension structure for accelerometer */
AT_extension_t accelero_extension = {
	.commands = accelero_commands,
	.init = accelero_init,
	.parse = accelero_parse,
	.persist = accelero_persist
};

/** @} */

/** @} (end addtogroup AT_ACCELERO) */
