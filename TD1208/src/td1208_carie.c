/******************************************************************************
 * @file
 * @brief User Application implementation for TD12xx RF module.
 * @author Telecom Design S.A.
 * @version 2.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012-2014 Telecom Design S.A., http://www.telecomdesign.fr</b>
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
 *****************************************************************************/

#include "config.h"
#include <efm32.h>
#include <td_module.h>
#include <td_core.h>
#include <td_flash.h>
#include <td_uart.h>
#include <td_printf.h>
#include <at_parse.h>
#include <sensors.h>

#if AT_CORE
#include <at_core.h>
#endif

#if AT_LAN_RF
#include <at_lan.h>
#include <td_lan.h>
#endif
#if AT_RADIO_INFO
#include <at_radio.h>
#endif
#if AT_SIGFOX
#include <at_sigfox.h>
#endif

#if AT_SENSOR
#include <at_sensor.h>
#include <at_sensor_lan.h>
#include <at_sensor_send.h>
#endif

#if AT_GEOLOC
#include <at_geoloc.h>
#include <at_accelero.h>
#endif

#if defined(SIGFOX_V1) && defined(SIGFOX_DOWNLINK)
#include <td_sigfox.h>
#endif

/* This file declare all "dynamic" library data. It should be last included file
 * Standard size value can be override before including this file
 */
#if MODULE_REVISION == REVISION_TD1202
#define TD_ALL_DUMP_REMOVE_CODE
#endif

#if AT_SENSOR
#define TD_SENSOR_GATEWAY_MAX_DEVICE 5
#endif

#define TD_TRAP_RESET_CODE
#define TD_TRAP_PRINTF_CODE

#define PRODUCT_LED_BLINK 1
#include <td_config.h>

#define USE_PRINTF

#define STARTUP_PRINT 0

/******************************************************************************
 *************************  Variables declaration   ****************************
 ******************************************************************************/

/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/

/**
 * @brief  User setup function
 **/
void TD_USER_Setup(void) {
	// Initialize the LEUART
	init_printf(TD_UART_Init(9600, true, false), TD_UART_Putc, TD_UART_Start,
			TD_UART_Stop);

	// Define variables version to avoid wrong flash init
#if STARTUP_PRINT
	tfp_printf("Define variables version to avoid wrong flash init.\r\n");
#endif
	TD_FLASH_SetVariablesVersion(FLASH_VARIABLES_VERSION);

#if AT_CORE
#if STARTUP_PRINT
	tfp_printf("Add Core AT parser.\r\n");
#endif
	AT_AddExtension(&core_extension);
#endif

#if AT_RADIO_INFO
#if STARTUP_PRINT
	tfp_printf("Add Radio AT parser.\r\n");
#endif
	AT_AddExtension(&radio_extension);
#endif

#if AT_SIGFOX
#if STARTUP_PRINT
	tfp_printf("Add Sigfox AT parser.\r\n");
#endif
	AT_AddExtension(&sigfox_extension);
#endif

#if AT_LAN_RF
#if STARTUP_PRINT
	tfp_printf("Add LAN AT parser.\r\n");
#endif
	AT_AddExtension(&lan_extension);
#endif

#if AT_SENSOR
#if STARTUP_PRINT
	tfp_printf("Add Sensor AT parser.\r\n");
#endif
	AT_AddExtension(&sensor_extension);
	AT_AddExtension(&sensor_lan_extension);
	AT_AddExtension(&sensor_send_extension);
#endif

#if AT_GEOLOC
#if STARTUP_PRINT
	tfp_printf("Add GPS AT parser.\r\n");
#endif
	AT_AddExtension(&geoloc_extension);
	AT_AddExtension(&accelero_extension);
#endif

	// Initialize the AT command parser
	AT_Init();

#if AT_SENSOR

	//Initialize Sensor
#if STARTUP_PRINT
	tfp_printf("Initialize Sensor.\r\n");
#endif
	TD_SENSOR_Init(TD_SENSOR_GetModuleType(), TD_LAN_GetFrequency(),
			TD_LAN_GetPowerLevel());
#endif

#if AT_GEOLOC

	//Initialize GPS
#if STARTUP_PRINT
	tfp_printf("Initialize GPS.\r\n");
#endif
	TD_GEOLOC_Init();

	//Initialize Accelero
#if STARTUP_PRINT
	tfp_printf("Initialize Accelerometer.\r\n");
#endif
	TD_ACCELERO_Init();
#endif

	//Init custom code
	TD_USER_Init();
}

/**
 * @brief  User loop function
 **/
void TD_USER_Loop(void) {
	int c;
	//tfp_printf("Process User Loop.\r\n");

#if AT_LAN_RF

	// Process local RF
	TD_LAN_Process();
#endif

#if defined(SIGFOX_V1) && defined(SIGFOX_DOWNLINK)
	TD_SIGFOX_DOWNLINK_Process();
#endif

#if AT_SENSOR
	TD_SENSOR_Process();
#endif

#if AT_GEOLOC
	TD_GEOLOC_Process();
	TD_ACCELERO_Process();
#endif

	while ((c = TD_UART_GetChar()) >= 0) {
		// LED On
		GPIO->P[PRODUCT_LED_PORT].DOUTSET = 1 << PRODUCT_LED_BIT;

		AT_Parse(c);

		// LED Off
		GPIO->P[PRODUCT_LED_PORT].DOUTCLR = 1 << PRODUCT_LED_BIT;
	}
}

