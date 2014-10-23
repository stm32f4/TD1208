/***************************************************************************//**
 * @file
 * @brief GPS management for TD12xx RF modules.
 * @author Telecom Design S.A.
 * @version 1.0.1
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

#include <em_gpio.h>

#include <td_core.h>
#include <td_gpio.h>
#include <td_rtc.h>
#include <td_flash.h>
#include <td_trap.h>
#include <td_module.h>
#include <td_scheduler.h>
#include <td_spi.h>
#include <td_printf.h>
#include <td_measure.h>
#include <td_utils.h>

#include "sensor_data.h"
#include "sensor_data_geoloc.h"
#include "nmea_parser.h"
#include "ubx7.h"
#include "td_geoloc.h"

/***************************************************************************//**
 * @addtogroup TD_GEOLOC Geolocalization
 * @brief API for managing the on-board GPS chip
 * @{
 ******************************************************************************/

/*******************************************************************************
 ************************   DEFINES   ********************************
 ******************************************************************************/

/** @addtogroup TD_GEOLOC_DEFINES Defines
 * @{ */

//#define GEOLOC_DEBUG
//#define GEOLOC_DEBUG_TIME

/** Turn on trace mode if tfp_printf not commented */
#ifdef GEOLOC_DEBUG
#define DEBUG_PRINTF(...) tfp_printf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif

/** Amount of data being read at once on SPI Bus */
#define GEOLOC_BUFFER_DATA	64

/** Time in seconds on which the battery level is tested when GPS ON */
#define VOLTAGE_CHECK_INTERVAL 60

/** @} */

/*******************************************************************************
 ************************   PRIVATE VARIABLES   ********************************
 ******************************************************************************/

/** @addtogroup TD_GEOLOC_LOCAL_VARIABLES Local Variables
 * @{ */

/** Contains Current position */
static TD_GEOLOC_Fix_t CurrentFix;

 /** Current GPS mode */
static TD_GEOLOC_PowerMode_t CurrentMode = TD_GEOLOC_OFF;

/** Timer ID for fix timeout */
static uint8_t FixTimer = 0xFF;

/** Configuration which must be saved in flash */
static TD_GEOLOC_Config_t Geoloc;

/** Logger flag for complete read */
static bool LogResetRead = false;

/** Current Fix start time */
static uint32_t StartFixTime = 0;

/** Last Voltage check time */
static uint32_t LastVoltageCheckTime = 0;

/** User Callback */
static void (*FixCallback)(TD_GEOLOC_Fix_t *fix, bool timeout) = 0;

/** Flag for fix timeout */
static bool FixTimeout = false;

/** Voltage level while using GPS */
static uint32_t PowerVoltageExtended = 0;

/** @} */

/*******************************************************************************
 ************************   EXTERN  ********************************************
 ******************************************************************************/

/** @addtogroup TD_GEOLOC_EXTERN Extern
 * @{ */

/** External declaration for code removal */
extern TD_FLASH_InitLogger_t const TD_FLASH_InitLogger_;
extern TD_GEOLOC_Log_t const TD_GEOLOC_Log_;

/** End of Code in Flash memory from linker script */
extern const char __cs3_regions_end;

/** End of Flash memory from linker script */
extern const char __cs3_region_end_rom;

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_GEOLOC_LOCAL_FUNCTIONS Local Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Find current fix type according to latitude/longitude/sat values.
 *
 * @param [in] fix
 *   Structure containing fix values/
 *****************************************************************************/
static void TD_GEOLOC_setFixType(TD_GEOLOC_Fix_t *fix)
{
	if (fix->position.latitude != 0x7FFFFFFF ||
		fix->position.longitude != 0x7FFFFFFF) {
		if (fix->quality.sat == 3) {
			fix->type = TD_GEOLOC_2D_FIX;
		} else if (fix->quality.sat >= 4) {
			fix->type = TD_GEOLOC_3D_FIX;
		} else {
			fix->type = TD_GEOLOC_NO_FIX;
		}
	} else if (fix->datetime.year != 0xFF ||
		fix->datetime.month != 0xFF ||
		fix->datetime.day != 0xFF) {
		fix->type = TD_GEOLOC_DATE_FIX;
	} else if (fix->datetime.hours != 0xFF ||
		fix->datetime.minutes != 0xFF ||
		fix->datetime.seconds != 0xFF) {
		fix->type = TD_GEOLOC_TIME_FIX;
	} else {
		fix->type = TD_GEOLOC_NO_FIX;
	}
}

/***************************************************************************//**
 * @brief
 *   Process nmea updates by updating fix type and duration and push
 *   it to the user callback.
 *
 * @param[in] fix
 *   Structure containing new fix values.
 *****************************************************************************/
static void TD_GEOLOC_OnNMEAUpdate(TD_GEOLOC_Fix_t *fix)
{
	uint32_t now = (TD_SCHEDULER_GetTime() >> 15);

	// Set fix duration and type
	fix->duration = (now - StartFixTime);

	// Measure battery level every minute
	if (LastVoltageCheckTime - now > VOLTAGE_CHECK_INTERVAL) {
		PowerVoltageExtended = TD_MEASURE_VoltageExtended();
		LastVoltageCheckTime = now;
	}
	TD_GEOLOC_setFixType(fix);
	memcpy(&CurrentFix, fix, sizeof (TD_GEOLOC_Fix_t));
	if (FixCallback != 0) {
		FixCallback(fix, FixTimeout);
	}
}

/***************************************************************************//**
 * @brief
 *  Fix timeout handler. Set the Fix timeout flag to true.
 *
 * @param[in] arg
 *  Unused.
 *
 * @param[in] repeat_count
 *  Unused.
 *****************************************************************************/
static void TD_GEOLOC_FixTimeout(uint32_t arg, uint8_t repeat_count)
{
	FixTimer = 0xFF;
	if (FixCallback != 0) {
		FixTimeout = true;
	}
}

/***************************************************************************//**
 * @brief
 *   Change GPS mode if different from current mode.
 *
 * @param[in] mode
 *   New GPS mode.
 *****************************************************************************/
static void TD_GEOLOC_SetPowerMode(TD_GEOLOC_PowerMode_t mode)
{
#ifdef GEOLOC_DEBUG_TIME
	uint32_t cnt = RTC->CNT;
	tfp_printf("SetPowerMode:%d\r\n", mode);
#endif

	if (mode != CurrentMode) {
		CurrentMode = mode;
		switch (mode) {
		case TD_GEOLOC_OFF:
			TD_UBX7_PowerOff();
			break;

		case TD_GEOLOC_HW_BCKP:
			TD_UBX7_HardwareBackup();
			break;

		case TD_GEOLOC_SOFT_BCKP:
			break;

		case TD_GEOLOC_POWER_SAVE_MODE:
		case TD_GEOLOC_NAVIGATION:
		case TD_GEOLOC_NAVIGATION_COLD_START:
			TD_NMEA_Reset();
			if (!TD_UBX7_PowerUp(mode == TD_GEOLOC_POWER_SAVE_MODE)) {

				TD_Trap(TRAP_GPS_HARD_ERR, 0);
			}
			if (mode == TD_GEOLOC_NAVIGATION_COLD_START) {
				TD_UBX7_SendStart(TD_UBX7_COLD);
			}
			break;
		}
	}

#ifdef GEOLOC_DEBUG_TIME
	if (((RTC->CNT - cnt) & 0xFFFFFF) > 10) {
		tfp_printf("Set power mode%d:%d\r\n", mode, (RTC->CNT - cnt) & 0xFFFFFF);
	}
#endif
}

/** @} */

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_GEOLOC_GLOBAL_FUNCTIONS Global Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *  Dump GPS state.
 ******************************************************************************/
void TD_GEOLOC_Dump(void)
{
	TD_UBX7_Dump();
}

/** @} */

/** @addtogroup TD_GEOLOC_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Start the GPS and try to get a fix. If the GPS is already running then no effect.
 *
 * @param[in] mode
 *   Power mode
 *
 * @param[in] timeout
 *  timeout in seconds. On timeout expiration the timeout parameter of the callback
 *  will be set to true. No further action is being performed and shutting down the
 *  GPS or not is left to the user application and should be done using TD_GEOLOC_StopFix.
 *
 * @param[in] callback
 *   Pointer to a function that will be called each time a parameter
 *   is being updated (time, position, speed, etc..).
 *   Fix is a pointer to a structure containing fix information.
 *   Timeout will be set to true when the specified timeout expired
 *
 *
 ******************************************************************************/
void TD_GEOLOC_TryToFix(TD_GEOLOC_PowerMode_t mode, uint16_t timeout, void (*callback)(TD_GEOLOC_Fix_t *fix, bool timeout))
{
	if (callback != 0) {
		FixCallback = callback;
		StartFixTime = TD_SCHEDULER_GetTime() >> 15;
		TD_GEOLOC_FixInit(&CurrentFix);
		TD_GEOLOC_SetPowerMode(mode);
		FixTimeout = false;
		if (timeout != TD_GEOLOC_INFINITE && FixTimer == 0xFF) {
			FixTimer = TD_SCHEDULER_AppendIrq(timeout,
				0,
				0,
				TD_SCHEDULER_ONE_SHOT,
				TD_GEOLOC_FixTimeout,
				0);
		}
	}
}

/***************************************************************************//**
 * @brief
 *   Stop the current fix timer if activated and switch to specified power mode.
 *
 * @param[in] end_mode
 *   GPS mode to switch to.
 *****************************************************************************/
void TD_GEOLOC_StopFix(TD_GEOLOC_PowerMode_t end_mode)
{
	if (FixTimer != 0xFF) {
		TD_SCHEDULER_Remove(FixTimer);
		FixTimer = 0xFF;
	} else {

		// May happen if fix timeout but user choose to keep it running
		// or if no timeout was set
	}
	TD_GEOLOC_SetPowerMode(end_mode);
}

/***************************************************************************//**
 * @brief
 *   Parse the GPS for incoming NMEA data if available and call
 *   user callback on updates. Must be called in User_Loop for
 *   proper GPS operation.
 *****************************************************************************/
void TD_GEOLOC_Process(void)
{
	uint8_t data[GEOLOC_BUFFER_DATA];

	// Poll NMEA from GPS chip if on
	if (CurrentMode == TD_GEOLOC_NAVIGATION || CurrentMode == TD_GEOLOC_POWER_SAVE_MODE) {

		// Poll NMEA
		if (TD_UBX7_Process(data, GEOLOC_BUFFER_DATA)) {

			// If data has been found feed the parser
			TD_NMEA_ParseBuffer((char *) data, GEOLOC_BUFFER_DATA);
		}
	}
}

/***************************************************************************//**
 * @brief
 *   Enables or disables GPS logger.
 *
 * @param[in] enable
 *   True to enable, false to disable.
 *****************************************************************************/
void TD_GEOLOC_SetLogger(bool enable)
{
	if (Geoloc.logger != true && enable) {
		TD_FLASH_InitLogger(true,
			1,
			sizeof (TD_GEOLOC_LogValue_t),
			(uint32_t) &__cs3_regions_end,
			(uint32_t) &__cs3_region_end_rom);
	}
	Geoloc.logger = enable;
}

/***************************************************************************//**
 * @brief
 *   Reset GPS logger.
 *****************************************************************************/
void TD_GEOLOC_ResetLogger(void)
{
	TD_FLASH_InitLogger(true,
		1,
		sizeof (TD_GEOLOC_LogValue_t),
		(uint32_t) &__cs3_regions_end,
		(uint32_t) &__cs3_region_end_rom);
}

/***************************************************************************//**
 * @brief
 *  Read each log value one by one.
 *
 * @param log
 *  Pointer to log structure (position and date) to read from flash
 *
 *  @return
 *   return true if all log has been read
 *
 *****************************************************************************/
bool TD_GEOLOC_ReadLog(TD_GEOLOC_LogValue_t *log)
{
	bool result = false;

	if (!LogResetRead) {
		TD_FLASH_LoggerResetRead(1);
		LogResetRead = true;
	}
	result = TD_FLASH_LoggerReadNext(1, (uint32_t *) log);

	// Everything has been read
	if (!result) {
		LogResetRead = false;
	}
	return result;
}

/***************************************************************************//**
 * @brief
 *  Log a fix position and time into flash.
 *
 * @param fix
 *  Pointer to fix to save.
 ******************************************************************************/
void TD_GEOLOC_Log(TD_GEOLOC_Fix_t *fix)
{
	TD_GEOLOC_LogValue_t log;

	// Copy interesting values
	memcpy(&log.position, &fix->position, sizeof (TD_GEOLOC_Position_t));
	memcpy(&log.datetime, &fix->datetime, sizeof (TD_GEOLOC_DateTime_t));

	// Write in flash
	TD_FLASH_LoggerWrite(1, (uint32_t *) &log);
}

/***************************************************************************//**
 * @brief
 *  Printf fix information as GPGGA
 *
 * @param fix
 *  Pointer to fix structure to be printed
 ******************************************************************************/
static void TD_GEOLOC_PrintfGPGGA(TD_GEOLOC_Fix_t *fix)
{
	char position[100];
	int checksum = 0;
	int32_t temp;
	long calc;
	int i;
	char ctemp[4] = ",N,";
	ctemp[3] = 0;

	if (fix->datetime.hours == 0xFF ||
		fix->datetime.minutes == 0xFF ||
		fix->datetime.seconds == 0xFF) {
		// tfp_sprintf(position,"$GPGGA,,");
		position[7] = ',';
		position[7] = 0;
	} else {
		tfp_sprintf(position, "$GPGGA,%02d%02d%02d.00,",
					fix->datetime.hours,
					fix->datetime.minutes,
					fix->datetime.seconds);
	}
	for (i = 1; position[i] != 0; i++) {
		checksum ^= position[i];
	}
	tfp_printf(position);
	temp = fix->position.latitude;
	if (temp < 0) {
		ctemp[1] = 'S';
		temp = -temp;
	}
	// tfp_printf("a%d\r\n",temp);
	if (temp != 0x7FFFFFFF) {
		calc = temp/100000;
		tfp_sprintf(position, "%04d.%03d", calc, temp - (calc * 100000));
		for (i = 0; position[i] != 0; i++) {
			checksum ^= position[i];
		}
		tfp_printf(position);
	}
	tfp_sprintf(position, ctemp);
	for (i = 0; position[i] != 0; i++) {
		checksum ^= position[i];
	}
	tfp_printf(position);
	temp = fix->position.longitude;
	ctemp[1] = 'E';
	if (temp < 0) {
		ctemp[1] = 'W';
		temp = -temp;
	}
	// tfp_printf("a%d\r\n",temp);
	if (temp != 0x7FFFFFFF) {
		calc = temp/100000;
		tfp_sprintf(position, "%05d.%03d", calc, temp - (calc * 100000));
		for (i = 0; position[i] != 0; i++) {
			checksum ^= position[i];
		}
		tfp_printf(position);
	}
	tfp_sprintf(position, ctemp);
	for (i = 0; position[i] != 0; i++) {
		checksum ^= position[i];
	}
	tfp_printf(position);
	if (fix->position.altitude != 0x7FFF) {
		tfp_sprintf(position,
					"1,%02d,%d,%d,M,0,M,,",
					fix->quality.sat,
					fix->quality.hdop / 100,
					fix->position.altitude);
	} else {
		tfp_sprintf(position,
					"1,%02d,%d,,M,0,M,,",
					fix->quality.sat,
					fix->quality.hdop / 100);
	}
	for (i = 0; position[i] != 0; i++) {
		checksum ^= position[i];
	}
	tfp_printf(position);
	tfp_sprintf(position, "*%02x\r\n", checksum);
	tfp_printf(position);
	tfp_printf("\r\n");
}

/***************************************************************************//**
 * @brief
 *  Print a fix log as GPGGA.
 *
 * @param log
 *  Pointer to log value to print.
 ******************************************************************************/
void TD_GEOLOC_PrintfFixLog(TD_GEOLOC_LogValue_t *log)
{
	TD_GEOLOC_Fix_t fix;

	memcpy(&fix.position, &log->position, sizeof (TD_GEOLOC_Position_t));
	memcpy(&fix.datetime,&log->datetime, sizeof (TD_GEOLOC_DateTime_t));
	TD_GEOLOC_PrintfGPGGA(&fix);
}

/***************************************************************************//**
 * @brief
 *  Print a fix as GPGGA.
 *
 * @param fix
 *  Pointer to fix value to print.
 ******************************************************************************/
void TD_GEOLOC_PrintfFix(TD_GEOLOC_Fix_t *fix)
{
	TD_GEOLOC_PrintfGPGGA(fix);
}

/***************************************************************************//**
 * @brief
 *   Retrieve the power supply voltage measured during current GPS session if
 *   GPS on or last GPS sesion if off. When ON this value is outdated of
 *   maximum VOLTAGE_CHECK_INTERVAL in seconds.
 *
 * @return
 *   Returns the last measured power supply voltage value in mV.
 ******************************************************************************/
uint32_t TD_GEOLOC_PowerVoltageExtended(void)
{
	return PowerVoltageExtended;
}

/***************************************************************************//**
 * @brief
 *  Init a fix structure with unknown position values.
 *
 * @param[in] fix
 *  Pointer to fix structure to init.
 ******************************************************************************/
void TD_GEOLOC_FixInit(TD_GEOLOC_Fix_t *fix)
{
	if (fix != 0) {
		fix->duration = 0;
		fix->position.altitude = 0x7FFF;
		fix->position.longitude = 0x7FFFFFFF;
		fix->position.latitude = 0x7FFFFFFF;
		fix->quality.hdop = 9999;
		fix->quality.sat = 0;
		memset(&fix->speed, 0xFF, sizeof (TD_GEOLOC_Speed_t));
		memset(&fix->datetime, 0xFF, sizeof (TD_GEOLOC_DateTime_t));
		fix->type = TD_GEOLOC_NO_FIX;
	}
}

/***************************************************************************//**
 * @brief
 *  Initialize Geolocation. Must be called in User Setup for
 *  proper GPS operation.
 ******************************************************************************/
void TD_GEOLOC_Init(void)
{
	TD_NMEA_Init(TD_GEOLOC_OnNMEAUpdate);
	TD_UBX7_Init();
	if (!TD_FLASH_DeclareVariable((uint8_t *) &Geoloc, sizeof (TD_GEOLOC_Config_t), 0)) {
		Geoloc.logger = false;
	}
	if (Geoloc.logger) {
		TD_FLASH_InitLogger_(false,
			1,
			sizeof (TD_GEOLOC_LogValue_t),
			(uint32_t) &__cs3_regions_end,
			(uint32_t) &__cs3_region_end_rom);
	}
	TD_GEOLOC_FixInit(&CurrentFix);
}

/** @} */

/** @} (end addtogroup TD_GEOLOC) */
