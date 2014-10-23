/******************************************************************************
 * @file
 * @brief Standard AT interpreter-based application for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 2.0.1
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
  ******************************************************************************/

#include <efm32.h>

#include <td_core.h>
#include <td_uart.h>
#include <td_printf.h>
#include <at_parse.h>
#include <at_radio.h>
#include <at_sigfox.h>

#include "at_user.h"

/*******************************************************************************
 ******************************   DEFINES   ************************************
 ******************************************************************************/

/** Manufacturer */
#define MANUFACTURER		"Telecom Design"

/** Product */
#define PRODUCT				"TDxxxx"

/** Hardware revision */
#define HARDWARE_VERSION	"0F"

/** Software version */
#define SOFTWARE_VERSION	"SOFTxxxx"

/** Release data */
#define RELEASE_DATE		"M10+2012"

/** Telecom Design 12-digit serial number */
#define SERIAL_NUMBER		"123456789012"


/* This file declare all "dynamic" library data. It should be last included file
 * Standard size value can be override before including this file
 */
#if MODULE_REVISION == REVISION_TD1202
#define TD_ALL_DUMP_REMOVE_CODE
#endif
#define TD_SENSOR_USE_CODE 0
#define TD_GEOLOC_USE_CODE 0
#include <td_config.h>


/*******************************************************************************
 ******************************  CONSTANTS  ************************************
 ******************************************************************************/

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/**
 * @brief  User setup function
 **/
void TD_USER_Setup(void)
{
    // Initialize printf() to work with the LEUART
    init_printf(TD_UART_Init(9600, true, false),
    		TD_UART_Putc,
    		TD_UART_Start,
    		TD_UART_Stop);

    // Add the RF information AT extension
	AT_AddExtension(&radio_extension);

	// Add the SIGFOX AT extension
	AT_AddExtension(&sigfox_extension);

	// Add the user AT extension
	AT_AddExtension(&user_extension);

	// Initialize the AT command parser
    AT_Init();
}

/**
 * @brief  User loop function
 **/
void TD_USER_Loop(void)
{
	int c;

	// While there are characters
	while ((c = TD_UART_GetChar()) >= 0) {

		// Parse them using the AT interpreter
		AT_Parse(c);
	}
}
