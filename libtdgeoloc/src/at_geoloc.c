/***************************************************************************//**
 * @file
 * @brief Geolocalization AT command extension for the TDxxxx RF modules.
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

#include <td_core.h>
#include <at_parse.h>
#include <td_printf.h>
#include <td_utils.h>
#include <td_sensor_utils.h>

#include "ubx7.h"
#include "nmea_parser.h"
#include "sensor_data.h"
#include "sensor_event.h"
#include "sensor_data_geoloc.h"
#include "td_geoloc.h"
#include "at_geoloc.h"

/***************************************************************************//**
 * @addtogroup AT_GEOLOC Geolocation AT Command Extension
 * @brief Geolocation AT command extension for the TDxxxx RF modules
 * @{
 ******************************************************************************/

/*******************************************************************************
 ***********************   ENUMERATIONS   **************************************
 ******************************************************************************/

/** @addtogroup AT_GEOLOC_ENUMERATIONS Enumerations
 * @{ */

/** Geolocation AT command tokens */
typedef enum geoloc_tokens_t {
	AT_EXTENSION_BASE = AT_BASE_LAST,	///< First extension token
	AT_GEOLOC_DUMP,						///< AT$GD
	AT_GEOLOC_LOG,						///< AT$GLOG=
	AT_GEOLOC_MODE,						///< AT$GPS=
	AT_GEOLOC_READLOG,					///< AT$GRL
	AT_GEOLOC_RESETLOG,					///< AT$GRTL
	AT_GEOLOC_SEND_POSITION,			///< AT$GSND
	AT_GEOLOC_SIM,						///< AT$GSIM=
//	AT_GEOLOC_TIME_TRACKING,			///< ATS600= TODO
} geoloc_tokens;

/** @} */

/*******************************************************************************
 *************************  CONSTANTS  *****************************************
 ******************************************************************************/

/** @addtogroup AT_GEOLOC_CONSTANTS Constants
 * @{ */

/** AT geolocation command set */
static AT_command_t const geoloc_commands[] = {

	{"AT$GD", AT_GEOLOC_DUMP},				///< Full GPS state dump
	{"AT$GLOG=", AT_GEOLOC_LOG},			///< Flash logger activation
	{"AT$GPS=", AT_GEOLOC_MODE},			///< GPS try to fix and power mode
	{"AT$GRL", AT_GEOLOC_READLOG},			///< GPS logger dump
	{"AT$GRTL", AT_GEOLOC_RESETLOG},		///< GPS logger reset
	{"AT$GSND", AT_GEOLOC_SEND_POSITION},	///< Send last computed position
	{"AT$GSIMU=", AT_GEOLOC_SIM},			///< Allow to simulate NMEA by feeding the parser
	{0, 0}
};

/** @} */

/*******************************************************************************
 ************************   PRIVATE VARIABLES   ********************************
 ******************************************************************************/

/** @addtogroup AT_GEOLOC_LOCAL_VARIABLES Local Variables
 * @{ */

/** Print GPS position flag */
static bool PrintPosition = false;

/** GPS operation mode upon exit */
static TD_GEOLOC_PowerMode_t EndMode = TD_GEOLOC_OFF;

/** GPS operation mode upon exit */
static TD_GEOLOC_PowerMode_t Mode = TD_GEOLOC_OFF;

/** Last GPS position */
static TD_GEOLOC_Fix_t LastPosition;

/** GPS position log */
static TD_GEOLOC_LogValue_t GPSLog;

/** Required quality to accept position */
static TD_GEOLOC_Quality_t RequiredQuality;

/** Send on fix flag */
static bool SendOnFix = false;

/** Fix timeout */
static uint16_t FixTimeout;

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup AT_GEOLOC_LOCAL_FUNCTIONS Local Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Callback function called when the GPS returns a fix.
 *
 * @param[in] fix
 *   Pointer to fix structure containing position information
 *
 * @param[in] timeout
 *   Set to true if fix timeout
 *
 ******************************************************************************/
static void GPSCallback(TD_GEOLOC_Fix_t *fix, bool timeout)
{
	memcpy(&LastPosition,fix,sizeof(TD_GEOLOC_Fix_t));
	if ((fix->type >= TD_GEOLOC_2D_FIX
			&& fix->quality.sat >= RequiredQuality.sat
			&& fix->quality.hdop <= RequiredQuality.hdop) || timeout) {
		if (PrintPosition) {
			TD_GEOLOC_PrintfFix(fix);
		}

		// Stop to send if required
		if (SendOnFix) {
			if (EndMode == TD_GEOLOC_NAVIGATION || EndMode == TD_GEOLOC_POWER_SAVE_MODE) {

				// Would be more efficient to go to soft backup
				TD_GEOLOC_StopFix(TD_GEOLOC_HW_BCKP);
				//void TD_UBX7_SoftBackup(void);
			} else {
				TD_GEOLOC_StopFix(EndMode);
			}
			TD_SENSOR_SendDataPosition(GPS_DATA_XYZ_SV_HDOP, fix, 0, 0);
		} else {
			TD_GEOLOC_StopFix(EndMode);
		}

		// Apply end mode
		if (SendOnFix) {
			if ((EndMode == TD_GEOLOC_NAVIGATION || EndMode == TD_GEOLOC_POWER_SAVE_MODE)
					|| FixTimeout == 0xFFFF) {
				TD_GEOLOC_TryToFix(Mode, FixTimeout, GPSCallback);
			}
		}
	}
}

/***************************************************************************//**
 * @brief
 *   Parser AT extension function for geolocalization.
 *
 * @param[in] token
 *   The token to parse.
 *
 * @return
 *   The parse result.
 ******************************************************************************/
static int8_t geoloc_parse(uint8_t token)
{
	int8_t result = AT_OK;
	int gps_mode = -1, parser_mode = -1, sat, hdop, timeout, end_mode;
	uint8_t position[10], msg[10];
	uint8_t pos_len, msg_len;
	int i = 0;

	switch (token) {
	case AT_GEOLOC_DUMP:
		if (AT_argc == 0) {
			TD_SystemDump(DUMP_UBX7);
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_GEOLOC_LOG:
		if (AT_argc == 1) {
			TD_GEOLOC_SetLogger(AT_atoll(AT_argv[0]));
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_GEOLOC_MODE:
		if (AT_argc >= 1) {
			gps_mode = AT_atoll(AT_argv[0]);
			if (AT_argc == 6 && (gps_mode == 1 || gps_mode == 3 || gps_mode == 4)) {
				sat = AT_atoll(AT_argv[1]);
				hdop = AT_atoll(AT_argv[2]);
				timeout = AT_atoll(AT_argv[3]);
				end_mode = AT_atoll(AT_argv[4]);
				parser_mode = AT_atoll(AT_argv[5]);
				if (sat > 0 && sat < 255 &&
					hdop >= 0 && hdop <= 0xFFFF &&
					timeout >= 0 && timeout <= 0xFFFF &&
					end_mode >= 0 && end_mode <= 2  &&
					parser_mode >= 0 && parser_mode <= 3) {
					if (parser_mode == 2) {
						PrintPosition = true;
						TD_NMEA_EnableOutput(0, "*");
						SendOnFix = false;
					} else if (parser_mode==3 ) {
						SendOnFix = true;
					} else {
						PrintPosition = false;
						SendOnFix = false;
						TD_NMEA_EnableOutput(parser_mode, "*");
					}
					RequiredQuality.hdop = hdop;
					RequiredQuality.sat = sat;
					switch (end_mode) {
					case 0:
						EndMode = TD_GEOLOC_OFF;
						break;

					case 1:
						EndMode = TD_GEOLOC_NAVIGATION;
						break;

					case 2:
						EndMode = TD_GEOLOC_HW_BCKP;
						break;
					case 3:
						EndMode = TD_GEOLOC_POWER_SAVE_MODE;
						break;
					}

					switch (gps_mode) {
					case 0:
						Mode = TD_GEOLOC_OFF;
						break;

					case 1:
						Mode = TD_GEOLOC_NAVIGATION;
						break;

					case 2:
						Mode = TD_GEOLOC_HW_BCKP;
						break;
					case 3:
						Mode = TD_GEOLOC_POWER_SAVE_MODE;
						break;
					case 4:
						Mode = TD_GEOLOC_NAVIGATION_COLD_START;
						break;
					}

					FixTimeout = timeout;
					TD_GEOLOC_TryToFix(Mode, timeout, GPSCallback);
				} else {
					result = AT_ERROR;
				}
			} else if (AT_argc == 1) {
				switch (gps_mode) {
				case 0:
					TD_GEOLOC_StopFix(TD_GEOLOC_OFF);
					break;

				case 2:
					TD_GEOLOC_StopFix(TD_GEOLOC_HW_BCKP);
					break;
				case 3:
					TD_GEOLOC_StopFix(TD_GEOLOC_SOFT_BCKP);
					break;

				default:
					result = AT_ERROR;
					break;
				}
			} else {
				result = AT_ERROR;
			}
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_GEOLOC_READLOG:
		if (AT_argc == 0) {
			while (TD_GEOLOC_ReadLog(&GPSLog)) {
				TD_GEOLOC_PrintfFixLog(&GPSLog);
			}
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_GEOLOC_RESETLOG:
		if (AT_argc == 0) {
			TD_GEOLOC_ResetLogger();
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_GEOLOC_SEND_POSITION:
		if (AT_argc == 0) {
			TD_SENSOR_SendDataPosition(GPS_DATA_XYZ_SV_HDOP, &LastPosition, 0, 0);
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_GEOLOC_SIM:
		if (AT_argc == 6) {
			TD_GEOLOC_Fix_t fix;
			fix.position.latitude = AT_atoll(AT_argv[1]);
			fix.position.longitude = AT_atoll(AT_argv[2]);
			fix.quality.sat = AT_atoll(AT_argv[3]);
			fix.quality.hdop = AT_atoll(AT_argv[4]);
			msg[0] = 0;
			TD_SENSOR_EncodePosition(GPS_DATA_XYZ_SV_HDOP, &fix, position, &pos_len);
			msg_len = 4; //stuff useless type for retro compatibility
			TD_SENSOR_UTILS_BitConcat(msg, &msg_len, position, pos_len);
			for (i = 0; i < AT_atoll(AT_argv[5]); i++) {
				TD_SENSOR_SendEvent((TD_SENSOR_EVENT_Types_t) AT_atoll(AT_argv[0]), msg, BITS_TO_BYTES(msg_len));
			}
			//TD_SENSOR_SendDataPosition(GPS_DATA_XYZ_SV_HDOP, &fix, 0, 0);
		} else if (AT_argc == 2) {
			for (i = 0; i < AT_atoll(AT_argv[1]); i++) {
				TD_SENSOR_SendEvent((TD_SENSOR_EVENT_Types_t) AT_atoll(AT_argv[0]), 0, 0);
			}
		} else {
			result = AT_ERROR;
		}
		break;

#if 0
	case AT_GEOLOC_TIME_TRACKING:
		if (AT_argc >= 1) {
			int enable = AT_atoll(AT_argv[0]);
			if (enable == 0 && AT_argc == 1) {
				TD_GEOLOC_SendPositionOntimer(false, 0, 0);
			} else if (enable == 1 && AT_argc == 5) {
				int interval = AT_atoll(AT_argv[1]);
				int sv = AT_atoll(AT_argv[2]);
				int hdop = AT_atoll(AT_argv[3]);
				int timeout = AT_atoll(AT_argv[4]);

				//TODO: check values
				if (interval >= 10 && interval <= 0xFFFF &&
						sv < 16 && sv >= 3 && hdop >= 0 && hdop) {
					condition.info_parsed = POSITION_3D_UPDATE;
					condition.req_quality.sat = sv;
					condition.req_quality.hdop = hdop;
					condition.timeout = timeout;
					TD_GEOLOC_SetCallback(0);
					TD_GEOLOC_SendPositionOntimer(true, interval, &condition);
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
#endif

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

/** @addtogroup AT_GEOLOC_GLOBAL_VARIABLES Global Variables
 * @{ */

/** Geolocalization AT extension */
AT_extension_t geoloc_extension = {
	.commands = geoloc_commands,
	.parse = geoloc_parse
};

/** @} */

/** @} (end addtogroup AT_GEOLOC) */
