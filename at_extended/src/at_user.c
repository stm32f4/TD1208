/***************************************************************************//**
 * @file
 * @brief User AT command extension for the TDxxxx RF modules.
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
#include <em_gpio.h>
#include <at_parse.h>
#include <stdint.h>
#include <stdbool.h>
#include <td_core.h>
#include <td_printf.h>
#include <td_measure.h>


/*******************************************************************************
 *************************   DEFINES   *****************************************
 ******************************************************************************/

#define LED_PORT	TIM2_PORT			/**< LED port */
#define LED_BIT		TIM2_BIT			/**< LED bit */

/*******************************************************************************
 ***********************   ENUMERATIONS   **************************************
 ******************************************************************************/

/** User AT command tokens */
typedef enum user_tokens_t {
	AT_EXTENSION_BASE = AT_BASE_LAST,	///< First extension token

	// Below here, please try to keep the enum in alphabetical order!!!
	AT_USER_LED,						///< AT$L=<value>
	AT_USER_TEMPERATURE,				///< AT$T?
	AT_USER_VOLTAGE,					///< AT$V?
} user_tokens;

/*******************************************************************************
 *************************  CONSTANTS   ****************************************
 ******************************************************************************/

/** User AT command set */
static AT_command_t const user_commands[] = {
	{"AT$L=", AT_USER_LED},
	{"AT$T?", AT_USER_TEMPERATURE},
	{"AT$V?", AT_USER_VOLTAGE},
	{0, 0}
};

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/***************************************************************************//**
 * @brief
 *   Initialization AT extension function for user.
 ******************************************************************************/
static void user_init(void)
{
	// Define the LED pin as an output in push-pull mode
	GPIO_PinModeSet(LED_PORT, LED_BIT, gpioModePushPull, 0);
}

/***************************************************************************//**
 * @brief
 *   Help AT extension function for user.
 ******************************************************************************/
static void user_help(void)
{
	AT_printf(
		"$L   => Turn on/off the LED\r\n"
		"$T   => Read the temperature\r\n"
		"$V   => Read the voltage\r\n");
};

/***************************************************************************//**
 * @brief
 *   Parser AT extension function for user.
 *
 * @param[in] token
 *   The token to parse.
 *
 * @return
 *   The parse result.
 ******************************************************************************/
static int8_t user_parse(uint8_t token)
{
	int8_t result = AT_OK;
	int value;

	switch (token) {

	case AT_USER_LED:
		if (AT_argc == 1) {
			value = AT_atoll(AT_argv[0]);
			if (value == 1) {

				// Turn on the LED
				GPIO_PinOutSet(LED_PORT, LED_BIT);
			} else if (value == 0) {

				// Turn off the LED
				GPIO_PinOutClear(LED_PORT, LED_BIT);
			} else {
				result = AT_ERROR;
			}
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_USER_TEMPERATURE:
		if (AT_argc == 0) {
		   value = TD_MEASURE_TemperatureExtended();
		   AT_printf("%d.%d\r\n", value / 10, value % 10);
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_USER_VOLTAGE:
		if (AT_argc == 0) {
		   value = TD_MEASURE_VoltageExtended();
		   AT_printf("%d.%03d\r\n", value / 1000, value % 1000);
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

/*******************************************************************************
 *************************   PUBLIC VARIABLES   ********************************
 ******************************************************************************/

/** User AT extension structure for */
AT_extension_t user_extension = {
	.init = user_init,				///< Pointer to the extension initialization function
	.commands = user_commands,		///< Pointer to the list of extension commands
	.help = user_help,				///< Pointer to the extension help display function
	.parse = user_parse				//< Pointer to the extension parse function
};
