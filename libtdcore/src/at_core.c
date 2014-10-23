/***************************************************************************//**
 * @file
 * @brief Core AT command extension for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Telecom Design S.A., http://www.telecomdesign.fr</b>
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

#include "at_core.h"
#include "td_watchdog.h"
#include "td_scheduler.h"

/***************************************************************************//**
 * @addtogroup AT_CORE Core AT Command Extension
 * @brief Core AT command extension for the TDxxxx RF modules
 * @{
 ******************************************************************************/

/*******************************************************************************
 ***********************   ENUMERATIONS   **************************************
 ******************************************************************************/

/** @addtogroup AT_CORE_ENUMERATIONS Enumerations
 * @{ */

/** Core AT command tokens */
typedef enum core_tokens_t {
	AT_EXTENSION_BASE = AT_BASE_LAST,	///< First extension token
	AT_CORE_GET_IO,						///< AT$GIO=
	AT_CORE_SET_IO,						///< AT$SIO=
	AT_CORE_SET_DRIVE_MODE,				///< AT$SDM=
	AT_CORE_QUERY_WATCHDOG,				///< ATS250=?
	AT_CORE_SET_WATCHDOG,				///< ATS250=
	AT_CORE_GET_WATCHDOG,				///< ATS250?
	AT_CORE_SCHEDULER_DUMP,				///< AT$SCD
	AT_CORE_GPIO_DUMP,					///< AT$IOD
	AT_CORE_IRQ_DUMP					///< AT$IRQD
} core_tokens;

/** @} */

/*******************************************************************************
 *************************  CONSTANTS  *****************************************
 ******************************************************************************/

/** @addtogroup AT_CORE_CONSTANTS Constants
 * @{ */

/** AT core command set */
static AT_command_t const core_commands[] = {
	{"AT$GIO=", AT_CORE_GET_IO},
	{"AT$SIO=", AT_CORE_SET_IO},
	{"AT$SDM=", AT_CORE_SET_DRIVE_MODE},
	{"AT$SCD", AT_CORE_SCHEDULER_DUMP},
	{"AT$IOD", AT_CORE_GPIO_DUMP},
	{"AT$IRQD", AT_CORE_IRQ_DUMP},
	{"ATS250=?", AT_CORE_QUERY_WATCHDOG},
	{"ATS250=", AT_CORE_SET_WATCHDOG},
	{"ATS250?", AT_CORE_GET_WATCHDOG},
	{0, 0}
};

/** @} */

/*******************************************************************************
 ************************   PRIVATE VARIABLES   ********************************
 ******************************************************************************/

/** @addtogroup AT_CORE_LOCAL_VARIABLES Local Variables
 * @{ */

/** Watchdog enabled flag */
static bool WatchdogEnabled = false;

/** Watchdog interval in seconds */
static uint16_t WatchdogInterval = 0;

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup AT_CORE_LOCAL_FUNCTIONS Local Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Persistence AT extension function for TD core functions.
 *
 * @param[in] write
 *   Flag set to true for writing persistent data, false for reading persistent
 *   data.
 *
 * @param[in/out] buffer
 *   Pointer to the persistent data buffer to read/write.
 *
 * @param[in] count
 *   The length in bytes of the persistent data buffer.
 *
 * @return
 *   The number of bytes read from/written to the persistent data buffer.
 ******************************************************************************/
static uint8_t core_persist(bool write, uint8_t *buffer, uint8_t count)
{
	uint8_t temp;
	uint16_t interval = WatchdogInterval;

	if (buffer != 0 && count != 0) {
		if (write == true) {
			for (temp = 0; temp <= 8; temp++, interval >>= 1) {
				if (interval & 1) {
					break;
				}
			}
			temp <<= 4;
			temp |= WatchdogEnabled ? 1 : 0;
			*(buffer++) = temp;
		} else {
			temp =  *buffer++ ;
			WatchdogEnabled = (temp & 0x01) ? true : false;
			WatchdogInterval = 1 << (temp >> 4);
			if (WatchdogEnabled) {
				TD_WATCHDOG_Init(WatchdogInterval);
				TD_WATCHDOG_Enable(true, true);
			}
		}
	}
	return 1;
}

/***************************************************************************//**
 * @brief
 *   Parser AT extension function for TD core functions.
 *
 * @param[in] token
 *   The token to parse.
 *
 * @return
 *   The parse result.
 ******************************************************************************/
static int8_t core_parse(uint8_t token)
{
	int8_t result = AT_OK;
	int port, pin, mode, out;
	int watchdog_interval;

	switch (token) {
	case AT_CORE_GET_IO:
		if (AT_argc == 2) {
			port = AT_atoll(AT_argv[0]);
			pin = AT_atoll(AT_argv[1]);
			AT_printf("%d",
					  GPIO_PinInGet((GPIO_Port_TypeDef) port, pin));
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_CORE_SET_IO:
		if (AT_argc == 4) {
			port = AT_atoll(AT_argv[0]);
			pin = AT_atoll(AT_argv[1]);
			mode = AT_atoll(AT_argv[2]);
			out = AT_atoll(AT_argv[3]);
			if (port >= 0 && port <= 5 &&
				pin >= 0 && pin <= 15 &&
				mode >= 0 && mode <= 15
				&& out >= 0 && out <= 1) {
				GPIO_PinModeSet((GPIO_Port_TypeDef) port,
								pin,
								(GPIO_Mode_TypeDef) mode,
								out);
			} else {
				result = AT_ERROR;
			}
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_CORE_SET_DRIVE_MODE:
		if (AT_argc == 2) {
			port = AT_atoll(AT_argv[0]);
			mode = AT_atoll(AT_argv[1]);
			if (port >= 0 && port <= 5 && mode >= 0 && mode <= 3) {
				GPIO_DriveModeSet((GPIO_Port_TypeDef) port,
								  (GPIO_DriveMode_TypeDef) mode);
			} else {
				result = AT_ERROR;
			}
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_CORE_QUERY_WATCHDOG:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			AT_printf("0..1,0|8|16|32|64|128|256\r\n");
		}
		break;

	case AT_CORE_GET_WATCHDOG:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			AT_printf("%d,%d\r\n", WatchdogEnabled, WatchdogInterval);
		}
		break;

	case AT_CORE_SET_WATCHDOG:
		if (AT_argc == 2) {
			watchdog_interval = AT_atoll(AT_argv[1]);
			if (AT_atoll(AT_argv[0]) == 1 &&
					!WatchdogEnabled) {
				if ((TD_WATCHDOG_Init(watchdog_interval) == false) ||
						(TD_WATCHDOG_Enable(true, true) == false)) {
					result = AT_ERROR;
				} else {
					WatchdogEnabled = true;
					WatchdogInterval = watchdog_interval;
				}
			} else if (AT_atoll(AT_argv[0]) == 0 && WatchdogEnabled) {
				if (TD_WATCHDOG_Enable(false, true) == false) {
					result = AT_ERROR;
				} else {
					WatchdogEnabled = false;
				}
			} else {
				result = AT_ERROR;
			}
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_CORE_SCHEDULER_DUMP:
		TD_SystemDump(DUMP_SCHEDULER);
		break;

	case AT_CORE_GPIO_DUMP:
		TD_SystemDump(DUMP_GPIO);
		break;

	case AT_CORE_IRQ_DUMP:
		TD_SystemDump(DUMP_IRQ);
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

/** @addtogroup AT_CORE_GLOBAL_VARIABLES Global Variables
 * @{ */

/** Core AT extension */
AT_extension_t core_extension = {
	.commands = core_commands,		///< Pointer to the list of extension commands
	.parse = core_parse,			//< Pointer to the extension parse function
	.persist = core_persist			///< Pointer to the extension persistence function
};

/** @} */

/** @} (end addtogroup AT_CORE) */
