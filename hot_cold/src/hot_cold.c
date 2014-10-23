/******************************************************************************
 * @file
 * @brief Sensor Temperature Monitoring Application Example
 * @author Telecom Design S.A.
 * @version 1.0.1
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

#include <stdint.h>
#include <stdbool.h>
#include <efm32.h>
#include <em_gpio.h>
#include <td_scheduler.h>
#include <td_uart.h>
#include <td_measure.h>
#include <td_flash.h>
#define USE_PRINTF
#include <td_printf.h>
#include <td_module.h>
#include <td_gpio.h>
#include <td_sensor.h>
#include <sensor_event.h>
#include <sensor_register.h>

/* This file declare all "dynamic" library data. It should be last included file
 * Standard size value can be override before including this file
 */
#define TD_GEOLOC_USE_CODE 0
#include <td_config.h>

/*******************************************************************************
 ******************************  DEFINES  ************************************
 ******************************************************************************/

#define DEVICE_CLASS 0x0001					// Define your own device class

#define TEMPERATURE_LEVEL_LOW 100			// Temperature low level in 1/10 degrees
#define TEMPERATURE_LEVEL_HIGH 180			// Temperature high level in 1/10 degrees
#define TEMPERATURE_CHECKING_INTERVAL 10	// Temperature checking interval in seconds

/*******************************************************************************
 ******************************  CONSTANTS  ************************************
 ******************************************************************************/

/*******************************************************************************
 **************************   PRIVATE FUNCTIONS   ******************************
 ******************************************************************************/


/***************************************************************************//**
 * @brief
 * Temperature callback. Called on Temperature Events.
 ******************************************************************************/
static bool TemperatureCallback(TD_SENSOR_TemperatureState_t state, int16_t level)
{
	bool return_value = false;

	int temp_int = level / 10;
	int temp_dec = level - (temp_int * 10);

	switch (state) {

		case TEMPERATURE_LOW:
			tfp_printf("Temperature low event, T=%d.%d \r\n", temp_int, temp_dec);
			return_value = true;
		break;

		case TEMPERATURE_OK:
			tfp_printf("Temperature OK event, T=%d.%d \r\n", temp_int, temp_dec);
			return_value = true;
		break;

		case TEMPERATURE_HIGH:
			tfp_printf("Temperature high event, T=%d.%d \r\n", temp_int, temp_dec);
		break;

		default:
		break;
	}

	// Return true to send event on sensor
	return return_value;
}


/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/

/***************************************************************************//**
 * @brief
 *   User setup function.
 ******************************************************************************/
void TD_USER_Setup(void)
{

	TD_FLASH_DeleteVariables();


	// Initialize the UART
	init_printf(TD_UART_Init(9600, true, false),
	    		TD_UART_Putc,
	    		TD_UART_Start,
	    		TD_UART_Stop);

	// Init LED
	GPIO_PinModeSet(TIM2_PORT, TIM2_BIT, gpioModePushPull, 0);

	// Print out current temperature
	tfp_printf("Temperature: %d degrees\r\n", TD_MEASURE_VoltageTemperatureExtended(true) / 10);

	// Set the device class
	TD_SENSOR_SetDeviceClass(DEVICE_CLASS);

	// Initialize the device as a transmitter with no local RF configuration.
	TD_SENSOR_Init(SENSOR_TRANSMITTER, 0, 0);

	// Register on Sensor
	TD_SENSOR_SendRegister();

	// Enable temperature monitoring and setup a callback on event
	TD_SENSOR_MonitorTemperature(true, TEMPERATURE_CHECKING_INTERVAL, TEMPERATURE_LEVEL_LOW, TEMPERATURE_LEVEL_HIGH, TemperatureCallback);

}

/***************************************************************************//**
 * @brief
 *   User loop function.
 ******************************************************************************/
void TD_USER_Loop(void)
{
	TD_SENSOR_Process();
}
